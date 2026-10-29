// Copyright 2026 usb_cam Authors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the usb_cam Authors nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

extern "C" {
#include <fcntl.h>  // for open()
#include <poll.h>  // for poll()
#include <unistd.h>  // for read(), write(), close()
}

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "usb_cam/see3cam.hpp"


namespace usb_cam
{

namespace
{
// USB IDs of the See3CAM_CU135 as sysfs prints them
const char SEE3CAM_VENDOR_ID[] = "2560";
const char CU135_PRODUCT_ID[] = "c1d1";

// Commands of the See3CAM_CU135 HID interface, as used by e-con's own tools
constexpr uint8_t CAMERA_CONTROL_CU135 = 0x81;
constexpr uint8_t GET_Q_FACTOR_CU135 = 0x0D;
constexpr uint8_t SET_Q_FACTOR_CU135 = 0x0E;
constexpr uint8_t GET_EXPOSURE_COMPENSATION_CU135 = 0x19;
constexpr uint8_t SET_EXPOSURE_COMPENSATION_CU135 = 0x1A;

// The answer echoes the command in its first two bytes and reports the result in byte 6
constexpr size_t STATUS_BYTE = 6;
constexpr uint8_t COMMAND_SUCCEEDED = 0x01;

constexpr int RESPONSE_TIMEOUT_MS = 1000;

std::string read_sysfs_value(const std::filesystem::path & path)
{
  std::ifstream file(path);
  std::string value;
  file >> value;
  return value;
}
}  // namespace

See3CamCU135::See3CamCU135(const std::string & video_device)
{
  namespace fs = std::filesystem;
  try {
    // /sys/class/video4linux/videoN/device is the camera's video interface, whose parent is the
    // USB device. The HID interface is another interface of that USB device, so its hidraw node
    // (/sys/class/hidraw/hidrawM/device -> <usb device>/<interface>/<hid device>) has the same
    // USB device two levels up
    const auto video_name = fs::canonical(video_device).filename();
    const auto usb_device =
      fs::canonical(fs::path("/sys/class/video4linux") / video_name / "device").parent_path();
    if (read_sysfs_value(usb_device / "idVendor") != SEE3CAM_VENDOR_ID ||
      read_sysfs_value(usb_device / "idProduct") != CU135_PRODUCT_ID)
    {
      m_last_error = video_device + " is not a See3CAM_CU135";
      return;
    }
    for (const auto & hidraw : fs::directory_iterator("/sys/class/hidraw")) {
      if (fs::canonical(hidraw.path() / "device").parent_path().parent_path() == usb_device) {
        m_hidraw_path = (fs::path("/dev") / hidraw.path().filename()).string();
        return;
      }
    }
    m_last_error = "found no HID interface of the See3CAM_CU135 at " + video_device;
  } catch (const fs::filesystem_error & error) {
    m_last_error = error.what();
  }
}

See3CamCU135::~See3CamCU135()
{
  if (m_fd >= 0) {
    close(m_fd);
  }
}

bool See3CamCU135::set_exposure_compensation(uint32_t value)
{
  if (value < 8000 || value > 1000000) {
    m_last_error = "exposure compensation must be between 8000 and 1000000";
    return false;
  }
  report_t response;
  return send_command(
    SET_EXPOSURE_COMPENSATION_CU135, {
      static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
      static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value)}, response);
}

bool See3CamCU135::get_exposure_compensation(uint32_t & value)
{
  report_t response;
  if (!send_command(GET_EXPOSURE_COMPENSATION_CU135, {}, response)) {
    return false;
  }
  value = (static_cast<uint32_t>(response[2]) << 24) | (static_cast<uint32_t>(response[3]) << 16) |
    (static_cast<uint32_t>(response[4]) << 8) | response[5];
  return true;
}

bool See3CamCU135::set_q_factor(uint8_t value)
{
  report_t response;
  return send_command(SET_Q_FACTOR_CU135, {value}, response);
}

bool See3CamCU135::get_q_factor(uint8_t & value)
{
  report_t response;
  if (!send_command(GET_Q_FACTOR_CU135, {}, response)) {
    return false;
  }
  value = response[2];
  return true;
}

bool See3CamCU135::send_command(
  uint8_t command, const std::vector<uint8_t> & arguments, report_t & response)
{
  if (m_hidraw_path.empty()) {
    return false;
  }

  if (m_fd < 0) {
    m_fd = open(m_hidraw_path.c_str(), O_RDWR | O_NONBLOCK);
    if (m_fd < 0) {
      m_last_error = "cannot open " + m_hidraw_path + ": " + strerror(errno);
      return false;
    }
  }
  const int fd = m_fd;

  // Discard answers to earlier commands that came too late, so they are not taken as this one's
  while (read(fd, response.data(), response.size()) > 0) {}

  // The camera uses unnumbered reports, so the request starts with report ID 0
  std::array<uint8_t, REPORT_SIZE + 1> request{};
  request[1] = CAMERA_CONTROL_CU135;
  request[2] = command;
  std::copy(arguments.begin(), arguments.end(), request.begin() + 3);

  bool succeeded = false;
  if (write(fd, request.data(), request.size()) < 0) {
    m_last_error = "cannot write to " + m_hidraw_path + ": " + strerror(errno);
    // e.g. the camera was unplugged; open the interface again next time
    close(m_fd);
    m_fd = -1;
  } else {
    // Wait for the report that answers this command, skipping any other
    m_last_error = "the camera did not answer within " + std::to_string(RESPONSE_TIMEOUT_MS) +
      " ms";
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(RESPONSE_TIMEOUT_MS);
    while (true) {
      const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count();
      struct pollfd pfd = {fd, POLLIN, 0};
      if (remaining_ms <= 0 || poll(&pfd, 1, static_cast<int>(remaining_ms)) <= 0) {
        break;
      }
      const ssize_t length = read(fd, response.data(), response.size());
      if (length > static_cast<ssize_t>(STATUS_BYTE) && response[0] == CAMERA_CONTROL_CU135 &&
        response[1] == command)
      {
        succeeded = response[STATUS_BYTE] == COMMAND_SUCCEEDED;
        m_last_error = succeeded ? "" : "the camera rejected the value";
        break;
      }
    }
  }
  return succeeded;
}

}  // namespace usb_cam
