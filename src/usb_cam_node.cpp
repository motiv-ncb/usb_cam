// Copyright 2014 Robert Bosch, LLC
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
//    * Neither the name of the Robert Bosch, LLC nor the names of its
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

#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#include "usb_cam/usb_cam_node.hpp"
#include "usb_cam/utils.hpp"

const char BASE_TOPIC_NAME[] = "image_raw";

namespace usb_cam
{

UsbCamNode::UsbCamNode(const rclcpp::NodeOptions & node_options)
: Node("usb_cam", node_options),
  m_camera(new usb_cam::UsbCam()),
  m_image_msg(new sensor_msgs::msg::Image()),
  m_compressed_img_msg(nullptr),
  m_image_publisher(nullptr),
  m_compressed_image_publisher(nullptr),
  m_compressed_cam_info_publisher(nullptr),
  m_parameters(),
  m_camera_info_msg(new sensor_msgs::msg::CameraInfo()),
  m_service_capture(
    this->create_service<std_srvs::srv::SetBool>(
      "set_capture",
      std::bind(
        &UsbCamNode::service_capture,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3)))
{
  // declare params
  this->declare_parameter("camera_name", "default_cam");
  this->declare_parameter("camera_info_url", "");
  this->declare_parameter("framerate", 30.0);
  this->declare_parameter("frame_id", "default_cam");
  this->declare_parameter("image_height", 480);
  this->declare_parameter("image_width", 640);
  this->declare_parameter("io_method", "mmap");
  this->declare_parameter("pixel_format", "yuyv");
  this->declare_parameter("av_device_format", "YUV422P");
  this->declare_parameter("video_device", "/dev/video0");
  this->declare_parameter("brightness", 50);  // 0-255, -1 "leave alone"
  this->declare_parameter("contrast", -1);    // 0-255, -1 "leave alone"
  this->declare_parameter("saturation", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("sharpness", -1);   // 0-255, -1 "leave alone"
  this->declare_parameter("gain", -1);        // 0-100?, -1 "leave alone"
  this->declare_parameter("auto_white_balance", true);
  this->declare_parameter("white_balance", 4000);
  this->declare_parameter("autoexposure", true);
  this->declare_parameter("exposure", 100);
  this->declare_parameter("autofocus", false);
  this->declare_parameter("focus", -1);  // 0-255, -1 "leave alone"
  this->declare_parameter("skip_device_check", false);  // allow bypassing V4L2 device list check

  get_params();
  init();
  m_parameters_callback_handle = add_on_set_parameters_callback(
    std::bind(
      &UsbCamNode::parameters_callback, this,
      std::placeholders::_1));
}

UsbCamNode::~UsbCamNode()
{
  RCLCPP_WARN(this->get_logger(), "Shutting down");
  m_image_msg.reset();
  m_compressed_img_msg.reset();
  m_camera_info_msg.reset();
  m_camera_info.reset();
  m_timer.reset();
  m_service_capture.reset();
  m_parameters_callback_handle.reset();

  delete (m_camera);
}

void UsbCamNode::service_capture(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request_header;
  if (request->data) {
    m_camera->start_capturing();
    response->message = "Start Capturing";
  } else {
    m_camera->stop_capturing();
    response->message = "Stop Capturing";
  }
}

std::string resolve_device_path(const std::string & path)
{
  if (std::filesystem::is_symlink(path)) {
    std::filesystem::path target_path = std::filesystem::read_symlink(path);

    // if the target path is relative, resolve it
    if (target_path.is_relative()) {
      target_path = std::filesystem::absolute(path).parent_path() / target_path;
      target_path = std::filesystem::canonical(target_path);
    }

    return target_path.string();
  }
  return path;
}

void UsbCamNode::init()
{
  while (m_parameters.frame_id == "") {
    RCLCPP_WARN_ONCE(
      this->get_logger(), "Required Parameters not set...waiting until they are set");
    get_params();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // load the camera info
  m_camera_info.reset(
    new camera_info_manager::CameraInfoManager(
      this, m_parameters.camera_name, m_parameters.camera_info_url));
  // check for default camera info
  if (!m_camera_info->isCalibrated()) {
    m_camera_info->setCameraName(m_parameters.device_name);
    m_camera_info_msg->header.frame_id = m_parameters.frame_id;
    m_camera_info_msg->width = m_parameters.image_width;
    m_camera_info_msg->height = m_parameters.image_height;
    m_camera_info->setCameraInfo(*m_camera_info_msg);
  }

  // Check if given device name is an available v4l2 device (unless skipped)
  if (!m_parameters.skip_device_check) {
    auto available_devices = usb_cam::utils::available_devices();
    if (available_devices.find(m_parameters.device_name) == available_devices.end()) {
      RCLCPP_ERROR_STREAM(
        this->get_logger(),
        "Device specified is not available or is not a vaild V4L2 device: `" <<
          m_parameters.device_name << "`"
      );
      RCLCPP_INFO(this->get_logger(), "Available V4L2 devices are:");
      for (const auto & device : available_devices) {
        RCLCPP_INFO_STREAM(this->get_logger(), "    " << device.first);
        RCLCPP_INFO_STREAM(this->get_logger(), "        " << device.second.card);
      }
      rclcpp::shutdown();
      return;
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Skipping V4L2 device availability check by request.");
  }

  // A raw_mjpeg frame is already a JPEG, so publish it as it is on image_raw/compressed. Only
  // other formats go through image_transport, whose compressed plugin would otherwise be a
  // second publisher on image_raw/compressed
  if (m_parameters.pixel_format_name == "raw_mjpeg") {
    m_compressed_img_msg.reset(new sensor_msgs::msg::CompressedImage());
    m_compressed_img_msg->header.frame_id = m_parameters.frame_id;
    m_compressed_img_msg->format = "jpeg";
    m_compressed_image_publisher =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
      std::string(BASE_TOPIC_NAME) + "/compressed", rclcpp::QoS(100));
    m_compressed_cam_info_publisher =
      this->create_publisher<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::QoS(100));
  } else {
    m_image_publisher = std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(
        this, BASE_TOPIC_NAME,
        rclcpp::QoS {100}.get_rmw_qos_profile()));
  }

  m_image_msg->header.frame_id = m_parameters.frame_id;
  RCLCPP_INFO(
    this->get_logger(), "Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
    m_parameters.camera_name.c_str(), m_parameters.device_name.c_str(),
    m_parameters.image_width, m_parameters.image_height, m_parameters.io_method_name.c_str(),
    m_parameters.pixel_format_name.c_str(), m_parameters.framerate);
  // set the IO method
  io_method_t io_method =
    usb_cam::utils::io_method_from_string(m_parameters.io_method_name);
  if (io_method == usb_cam::utils::IO_METHOD_UNKNOWN) {
    RCLCPP_ERROR_ONCE(
      this->get_logger(),
      "Unknown IO method '%s'", m_parameters.io_method_name.c_str());
    rclcpp::shutdown();
    return;
  }

  // configure the camera
  m_camera->configure(m_parameters, io_method);

  set_v4l2_params();

  // start the camera
  m_camera->start();

  auto frame_rate = m_camera->get_frame_rate();
  RCLCPP_INFO_STREAM(this->get_logger(), "Camera runs at " << frame_rate << " fps");
  if (static_cast<size_t>(m_parameters.framerate) > frame_rate) {
    RCLCPP_WARN_STREAM(
      this->get_logger(),
      "Desired framerate " << m_parameters.framerate << " is higher than the camera's capability " <<
        frame_rate << " fps");
    m_parameters.framerate = frame_rate;
  }

  // update() waits for the next frame, so poll at twice the camera's frame rate: it then waits
  // on the camera rather than on the timer, and catches up at once if frames queued up
  // TODO(lucasw) how to do small than ms, or fractional ms- std::chrono::nanoseconds?
  const int period_ms = 1000.0 / (2 * frame_rate);
  m_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int64_t>(period_ms)),
    std::bind(&UsbCamNode::update, this));
  RCLCPP_INFO_STREAM(this->get_logger(), "Timer triggering every " << period_ms << " ms");
  if (static_cast<size_t>(m_parameters.framerate) < frame_rate) {
    RCLCPP_INFO_STREAM(
      this->get_logger(),
      "Publishing " << m_parameters.framerate << " of the camera's " << frame_rate << " fps");
  }
}

void UsbCamNode::get_params()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this);
  auto parameters = parameters_client->get_parameters(
    {
      "camera_name", "camera_info_url", "frame_id", "framerate", "image_height", "image_width",
      "io_method", "pixel_format", "av_device_format", "video_device", "brightness", "contrast",
      "saturation", "sharpness", "gain", "auto_white_balance", "white_balance", "autoexposure",
      "exposure", "autofocus", "focus", "skip_device_check"
    }
  );

  assign_params(parameters);
}

void UsbCamNode::assign_params(const std::vector<rclcpp::Parameter> & parameters)
{
  for (auto & parameter : parameters) {
    if (parameter.get_name() == "camera_name") {
      RCLCPP_INFO(this->get_logger(), "camera_name value: %s", parameter.value_to_string().c_str());
      m_parameters.camera_name = parameter.value_to_string();
    } else if (parameter.get_name() == "camera_info_url") {
      m_parameters.camera_info_url = parameter.value_to_string();
    } else if (parameter.get_name() == "frame_id") {
      m_parameters.frame_id = parameter.value_to_string();
    } else if (parameter.get_name() == "framerate") {
      RCLCPP_WARN(this->get_logger(), "framerate: %f", parameter.as_double());
      m_parameters.framerate = parameter.as_double();
    } else if (parameter.get_name() == "image_height") {
      m_parameters.image_height = parameter.as_int();
    } else if (parameter.get_name() == "image_width") {
      m_parameters.image_width = parameter.as_int();
    } else if (parameter.get_name() == "io_method") {
      m_parameters.io_method_name = parameter.value_to_string();
    } else if (parameter.get_name() == "pixel_format") {
      m_parameters.pixel_format_name = parameter.value_to_string();
    } else if (parameter.get_name() == "av_device_format") {
      m_parameters.av_device_format = parameter.value_to_string();
    } else if (parameter.get_name() == "video_device") {
      m_parameters.device_name = resolve_device_path(parameter.value_to_string());
    } else if (parameter.get_name() == "brightness") {
      m_parameters.brightness = parameter.as_int();
    } else if (parameter.get_name() == "contrast") {
      m_parameters.contrast = parameter.as_int();
    } else if (parameter.get_name() == "saturation") {
      m_parameters.saturation = parameter.as_int();
    } else if (parameter.get_name() == "sharpness") {
      m_parameters.sharpness = parameter.as_int();
    } else if (parameter.get_name() == "gain") {
      m_parameters.gain = parameter.as_int();
    } else if (parameter.get_name() == "auto_white_balance") {
      m_parameters.auto_white_balance = parameter.as_bool();
    } else if (parameter.get_name() == "white_balance") {
      m_parameters.white_balance = parameter.as_int();
    } else if (parameter.get_name() == "autoexposure") {
      m_parameters.autoexposure = parameter.as_bool();
    } else if (parameter.get_name() == "exposure") {
      m_parameters.exposure = parameter.as_int();
    } else if (parameter.get_name() == "autofocus") {
      m_parameters.autofocus = parameter.as_bool();
    } else if (parameter.get_name() == "focus") {
      m_parameters.focus = parameter.as_int();
    } else if (parameter.get_name() == "skip_device_check") {
      m_parameters.skip_device_check = parameter.as_bool();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid parameter name: %s", parameter.get_name().c_str());
    }
  }
}

/// @brief Send current parameters to V4L2 device
/// TODO(flynneva): should this actuaully be part of UsbCam class?
void UsbCamNode::set_v4l2_params()
{
  // Controls are set by their V4L2 ID: v4l2-ctl names change between kernels (e.g.
  // `exposure_auto` is now `auto_exposure`) and a name the kernel no longer knows is ignored.
  // The names logged below are the current v4l2-ctl ones

  // set camera parameters
  if (m_parameters.brightness >= 0) {
    set_v4l2_control("brightness", V4L2_CID_BRIGHTNESS, m_parameters.brightness);
  }

  if (m_parameters.contrast >= 0) {
    set_v4l2_control("contrast", V4L2_CID_CONTRAST, m_parameters.contrast);
  }

  if (m_parameters.saturation >= 0) {
    set_v4l2_control("saturation", V4L2_CID_SATURATION, m_parameters.saturation);
  }

  if (m_parameters.sharpness >= 0) {
    set_v4l2_control("sharpness", V4L2_CID_SHARPNESS, m_parameters.sharpness);
  }

  if (m_parameters.gain >= 0) {
    set_v4l2_control("gain", V4L2_CID_GAIN, m_parameters.gain);
  }

  // check auto white balance
  set_v4l2_control(
    "white_balance_automatic", V4L2_CID_AUTO_WHITE_BALANCE, m_parameters.auto_white_balance);
  if (!m_parameters.auto_white_balance) {
    set_v4l2_control(
      "white_balance_temperature", V4L2_CID_WHITE_BALANCE_TEMPERATURE, m_parameters.white_balance);
  }

  // check auto exposure
  if (!m_parameters.autoexposure) {
    set_v4l2_control("auto_exposure", V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
    set_v4l2_control("exposure_time_absolute", V4L2_CID_EXPOSURE_ABSOLUTE, m_parameters.exposure);
  } else {
    // Cameras offer different automatic modes: use fully automatic exposure if there is one
    // (e.g. See3CAM_CU135, which has only auto and manual), else aperture priority, the
    // automatic mode most webcams have
    set_v4l2_control(
      "auto_exposure", V4L2_CID_EXPOSURE_AUTO,
      m_camera->has_v4l_menu_item(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO) ?
      V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_APERTURE_PRIORITY);
  }

  // check auto focus, which fixed-focus cameras do not have
  if (m_camera->has_v4l_control(V4L2_CID_FOCUS_AUTO)) {
    set_v4l2_control("focus_automatic_continuous", V4L2_CID_FOCUS_AUTO, m_parameters.autofocus);
    if (!m_parameters.autofocus && m_parameters.focus >= 0) {
      set_v4l2_control("focus_absolute", V4L2_CID_FOCUS_ABSOLUTE, m_parameters.focus);
    }
  } else if (m_parameters.autofocus || m_parameters.focus >= 0) {
    RCLCPP_WARN(
      this->get_logger(), "Camera has no focus control, ignoring 'autofocus' and 'focus'");
  }
}

void UsbCamNode::set_v4l2_control(const std::string & name, uint32_t id, int value)
{
  RCLCPP_INFO(this->get_logger(), "Setting '%s' to %d", name.c_str(), value);
  const int error = m_camera->set_v4l_control(id, value);
  if (error != 0) {
    RCLCPP_WARN(
      this->get_logger(), "Could not set '%s' to %d: %s", name.c_str(), value, strerror(error));
  }
}

bool UsbCamNode::take_and_send_image()
{
  // Only resize if required
  if (sizeof(m_image_msg->data) != m_camera->get_image_size_in_bytes()) {
    m_image_msg->width = m_camera->get_image_width();
    m_image_msg->height = m_camera->get_image_height();
    m_image_msg->encoding = m_camera->get_pixel_format()->ros();
    m_image_msg->step = m_camera->get_image_step();
    if (m_image_msg->step == 0) {
      // Some formats don't have a linesize specified by v4l2
      // Fall back to manually calculating it step = size / height
      m_image_msg->step = m_camera->get_image_size_in_bytes() / m_image_msg->height;
    }
    m_image_msg->data.resize(m_camera->get_image_size_in_bytes());
  }

  // grab the image, pass image msg buffer to fill
  if (!m_camera->get_image(reinterpret_cast<char *>(&m_image_msg->data[0]))) {
    return false;
  }

  auto stamp = m_camera->get_image_timestamp();
  m_image_msg->header.stamp.sec = stamp.tv_sec;
  m_image_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_image_msg->header;
  return true;
}

bool UsbCamNode::take_and_send_image_mjpeg()
{
  // grab the image, sized to the bytes this frame used
  if (!m_camera->get_image(m_compressed_img_msg->data)) {
    return false;
  }

  // A frame without a JPEG start marker (e.g. one cut short while the stream starts) is of no
  // use downstream, so drop it rather than publish it
  const auto & data = m_compressed_img_msg->data;
  if (data.size() < 2 || data[0] != 0xFF || data[1] != 0xD8) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Dropping a frame that is not a JPEG (%zu bytes)", data.size());
    return false;
  }

  auto stamp = m_camera->get_image_timestamp();
  m_compressed_img_msg->header.stamp.sec = stamp.tv_sec;
  m_compressed_img_msg->header.stamp.nanosec = stamp.tv_nsec;

  *m_camera_info_msg = m_camera_info->getCameraInfo();
  m_camera_info_msg->header = m_compressed_img_msg->header;

  return true;
}

rcl_interfaces::msg::SetParametersResult UsbCamNode::parameters_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  RCLCPP_DEBUG(this->get_logger(), "Setting parameters for %s", m_parameters.camera_name.c_str());
  m_timer->reset();
  assign_params(parameters);
  set_v4l2_params();
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void UsbCamNode::update()
{
  if (m_camera->is_capturing()) {
    // If the camera exposure longer higher than the framerate period
    // then that caps the framerate.
    // auto t0 = now();
    bool isSuccessful = (m_parameters.pixel_format_name == "raw_mjpeg") ?
      take_and_send_image_mjpeg() :
      take_and_send_image();
    // Publish each new frame exactly once, so nothing downstream sees a frame twice
    if (!isSuccessful) {
      return;
    }

    // V4L2 numbers every frame the camera sends, so a jump means frames were lost because
    // they were not read in time
    const uint32_t sequence = m_camera->get_image_sequence();
    if (m_frames_read > 0 && sequence > m_last_sequence + 1) {
      m_frames_lost += sequence - m_last_sequence - 1;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "%zu of %zu camera frames lost so far (not read in time)", m_frames_lost,
        m_frames_lost + m_frames_read);
    }
    m_last_sequence = sequence;
    m_frames_read++;

    // The camera only offers fixed rates and may run faster than `framerate`, so skip frames
    // to publish `framerate` per second on average
    const auto frame_rate = m_camera->get_frame_rate();
    if (static_cast<size_t>(m_parameters.framerate) < frame_rate) {
      const auto stamp = m_camera->get_image_timestamp();
      const double stamp_s = stamp.tv_sec + stamp.tv_nsec * 1e-9;
      if (stamp_s + 0.5 / frame_rate < m_next_publish_time) {
        return;
      }
      m_next_publish_time = std::max(m_next_publish_time, stamp_s) + 1.0 / m_parameters.framerate;
    }

    publish();
  }
}

void UsbCamNode::publish()
{
  if (m_parameters.pixel_format_name == "raw_mjpeg") {
    m_compressed_image_publisher->publish(*m_compressed_img_msg);
    m_compressed_cam_info_publisher->publish(*m_camera_info_msg);
  } else {
    m_image_publisher->publish(*m_image_msg, *m_camera_info_msg);
  }
}
}  // namespace usb_cam


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(usb_cam::UsbCamNode)
