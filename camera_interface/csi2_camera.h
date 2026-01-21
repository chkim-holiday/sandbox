#ifndef CSI2_CAMERA_H_
#define CSI2_CAMERA_H_

#include "v4l2_camera.h"

/// @brief MIPI CSI-2 Camera implementation
class MIPICSI2Camera : public V4L2Camera {
 public:
  ~MIPICSI2Camera();
  std::string GetCameraType() const override;
  bool SetExposureTime(double exposure_us) override;
  bool GetExposureTime(double& exposure_us) override;
  bool SetExposureMode(ExposureMode mode) override;
  bool GetExposureMode(ExposureMode& mode) override;
  bool SetTriggerMode(TriggerMode mode) override;
  bool GetTriggerMode(TriggerMode& mode) override;
  bool SetWhiteBalance(double wb_r, double wb_b) override;
  bool GetWhiteBalance(double& wb_r, double& wb_b) override;
  bool Initialize(const CameraConfig& config) override;

 private:
  std::string FindSubdevPath(const std::string& video_path);

  int subdev_fd_ = -1;  // For direct sensor control
  std::string subdev_path_;
};

#endif  // CSI2_CAMERA_H_
