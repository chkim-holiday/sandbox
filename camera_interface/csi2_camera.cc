#include "csi2_camera.h"

MIPICSI2Camera::~MIPICSI2Camera() {
  if (subdev_fd_ >= 0) {
    close(subdev_fd_);
  }
}

std::string MIPICSI2Camera::GetCameraType() const { return "mipi_csi2"; }

bool MIPICSI2Camera::SetExposureTime(double exposure_us) {
  if (subdev_fd_ < 0) {
    // Fallback to video node
    return V4L2Camera::SetExposureTime(exposure_us);
  }

  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE;
  ctrl.value = static_cast<int>(exposure_us);
  return ioctl(subdev_fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

bool MIPICSI2Camera::GetExposureTime(double& exposure_us) {
  if (subdev_fd_ < 0) {
    return V4L2Camera::GetExposureTime(exposure_us);
  }

  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE;
  if (ioctl(subdev_fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    exposure_us = ctrl.value;
    return true;
  }
  return false;
}

bool MIPICSI2Camera::SetExposureMode(ExposureMode mode) {
  if (subdev_fd_ < 0) {
    return V4L2Camera::SetExposureMode(mode);
  }

  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  ctrl.value =
      (mode == ExposureMode::Auto) ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL;
  return ioctl(subdev_fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

bool MIPICSI2Camera::GetExposureMode(ExposureMode& mode) {
  if (subdev_fd_ < 0) {
    return V4L2Camera::GetExposureMode(mode);
  }

  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  if (ioctl(subdev_fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    mode = (ctrl.value == V4L2_EXPOSURE_AUTO) ? ExposureMode::Auto
                                              : ExposureMode::Manual;
    return true;
  }
  return false;
}

bool MIPICSI2Camera::SetTriggerMode(TriggerMode mode) {
  // CSI-2 trigger support depends on sensor
  // For sensors with external trigger (e.g., IMX series)
  // This typically requires device tree configuration
  // and specific sensor driver support

  if (mode == TriggerMode::Hardware) {
    // Check if sensor supports external trigger
    // This is sensor-specific
    std::cerr << "Hardware trigger requires sensor driver support" << std::endl;
    return false;  // Needs implementation per sensor
  }

  return true;
}

bool MIPICSI2Camera::GetTriggerMode(TriggerMode& mode) {
  mode = TriggerMode::FreeRun;
  return true;
}

bool MIPICSI2Camera::SetWhiteBalance(double wb_r, double wb_b) {
  if (subdev_fd_ < 0) {
    return V4L2Camera::SetWhiteBalance(wb_r, wb_b);
  }

  // Disable auto white balance
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
  ctrl.value = 0;
  ioctl(subdev_fd_, VIDIOC_S_CTRL, &ctrl);

  // Set R gain
  ctrl.id = V4L2_CID_RED_BALANCE;
  ctrl.value = static_cast<int>(wb_r * 256);  // Typical scaling
  ioctl(subdev_fd_, VIDIOC_S_CTRL, &ctrl);

  // Set B gain
  ctrl.id = V4L2_CID_BLUE_BALANCE;
  ctrl.value = static_cast<int>(wb_b * 256);
  return ioctl(subdev_fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

bool MIPICSI2Camera::GetWhiteBalance(double& wb_r, double& wb_b) {
  if (subdev_fd_ < 0) {
    return V4L2Camera::GetWhiteBalance(wb_r, wb_b);
  }

  struct v4l2_control ctrl;

  ctrl.id = V4L2_CID_RED_BALANCE;
  if (ioctl(subdev_fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    wb_r = ctrl.value / 256.0;
  }

  ctrl.id = V4L2_CID_BLUE_BALANCE;
  if (ioctl(subdev_fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    wb_b = ctrl.value / 256.0;
    return true;
  }

  return false;
}

bool MIPICSI2Camera::Initialize(const CameraConfig& config) {
  if (!V4L2Camera::Initialize(config)) {
    return false;
  }

  // Open subdev for sensor control
  // Typically /dev/v4l-subdev0 for sensor
  subdev_path_ = FindSubdevPath(device_path_);
  if (!subdev_path_.empty()) {
    subdev_fd_ = open(subdev_path_.c_str(), O_RDWR);
  }

  return true;
}

std::string MIPICSI2Camera::FindSubdevPath(const std::string& video_path) {
  // Logic to find corresponding subdev
  // e.g., /dev/video0 -> /dev/v4l-subdev0
  // This is system-specific

  // Simple heuristic: try subdev0, subdev1, etc.
  for (int i = 0; i < 10; ++i) {
    std::string subdev = "/dev/v4l-subdev" + std::to_string(i);
    if (access(subdev.c_str(), F_OK) == 0) {
      return subdev;
    }
  }
  return "";
}
