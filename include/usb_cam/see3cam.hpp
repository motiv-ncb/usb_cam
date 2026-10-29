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

#ifndef USB_CAM__SEE3CAM_HPP_
#define USB_CAM__SEE3CAM_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>


namespace usb_cam
{

/// @brief Controls of the e-con Systems See3CAM_CU135 that are not V4L2 controls. The camera
/// takes them as 64-byte reports on its HID interface (/dev/hidrawN)
class See3CamCU135
{
public:
  /// @brief Find the HID interface of the USB device that video_device belongs to
  /// @param video_device V4L2 device of the camera, e.g. /dev/video0
  explicit See3CamCU135(const std::string & video_device);
  ~See3CamCU135();

  // Owns the open HID interface
  See3CamCU135(const See3CamCU135 &) = delete;
  See3CamCU135 & operator=(const See3CamCU135 &) = delete;

  /// @brief true if video_device is a See3CAM_CU135 and its HID interface was found
  bool is_available() const {return !m_hidraw_path.empty();}

  /// @brief HID interface of the camera, e.g. /dev/hidraw3, empty if not found
  const std::string & hidraw_path() const {return m_hidraw_path;}

  /// @brief Why the camera was not found or the last command failed
  const std::string & last_error() const {return m_last_error;}

  /// @brief Exposure compensation of the camera's auto exposure, 8000 - 1000000
  bool set_exposure_compensation(uint32_t value);
  bool get_exposure_compensation(uint32_t & value);

  /// @brief JPEG quality of MJPEG frames, 10 - 96
  bool set_q_factor(uint8_t value);
  bool get_q_factor(uint8_t & value);

private:
  static constexpr size_t REPORT_SIZE = 64;
  using report_t = std::array<uint8_t, REPORT_SIZE>;

  /// @brief Send a command and wait until the camera answers it
  /// @return true if the camera reports that the command succeeded
  bool send_command(uint8_t command, const std::vector<uint8_t> & arguments, report_t & response);

  std::string m_hidraw_path;
  // Kept open between commands: opening the HID interface takes ~50 ms, a command ~1 ms
  int m_fd = -1;
  std::string m_last_error;
};

}  // namespace usb_cam

#endif  // USB_CAM__SEE3CAM_HPP_
