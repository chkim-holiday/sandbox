#ifndef UVC_CAMERA_H_
#define UVC_CAMERA_H_

#include "camera_interface.h"
#include "v4l2_camera.h"

class UVCCamera : public V4L2Camera {
 public:
  std::string GetCameraType() const override { return "uvc"; }

  bool SetExposureTime(double exposure_us) override {
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = static_cast<int>(exposure_us / 100);  // 100us units
    return ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
  }

  bool GetExposureTime(double& exposure_us) override {
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
      exposure_us = ctrl.value * 100.0;  // 100us units
      return true;
    }
    return false;
  }

  bool SetExposureMode(ExposureMode mode) override {
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = (mode == ExposureMode::Auto) ? V4L2_EXPOSURE_AUTO
                                              : V4L2_EXPOSURE_MANUAL;
    return ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
  }

  bool GetExposureMode(ExposureMode& mode) override {
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
      mode = (ctrl.value == V4L2_EXPOSURE_AUTO) ? ExposureMode::Auto
                                                : ExposureMode::Manual;
      return true;
    }
    return false;
  }

  bool SetTriggerMode(TriggerMode mode) override {
    // Most UVC cameras don't support hardware trigger
    if (mode == TriggerMode::Hardware) {
      std::cerr << "Hardware trigger not supported on UVC" << std::endl;
      return false;
    }
    return true;
  }

  bool GetTriggerMode(TriggerMode& mode) override {
    mode = TriggerMode::FreeRun;
    return true;
  }

  bool SetWhiteBalance(double wb_r, double wb_b) override {
    // Disable auto white balance
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
    ctrl.value = 0;
    ioctl(fd_, VIDIOC_S_CTRL, &ctrl);

    // Set manual white balance (implementation varies by camera)
    // Many UVC cameras use color temperature instead of R/B gains
    ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    ctrl.value = 5000;  // Example: 5000K
    return ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
  }

  bool GetWhiteBalance(double& wb_r, double& wb_b) override {
    // UVC typically doesn't expose separate R/B gains
    wb_r = 1.0;
    wb_b = 1.0;
    return false;
  }
};
#endif  // UVC_CAMERA_H_
