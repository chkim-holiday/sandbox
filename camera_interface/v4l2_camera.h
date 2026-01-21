#ifndef V4L2_CAMERA_H_
#define V4L2_CAMERA_H_

#include <iostream>

#include "camera_interface.h"

class V4L2Camera : public CameraInterface {
 public:
  virtual ~V4L2Camera();
  bool Initialize(const CameraConfig& config) override;
  bool StartStreaming() override;
  bool StopStreaming() override;
  void RegisterFrameCallback(FrameCallback callback) override;

  // Camera controls
  bool SetExposureTime(double exposure_us) override;
  bool GetExposureTime(double& exposure_us) override;
  bool SetExposureMode(ExposureMode mode) override;
  bool GetExposureMode(ExposureMode& mode) override;
  bool SetWhiteBalance(double wb_r, double wb_b) override;
  bool GetWhiteBalance(double& wb_r, double& wb_b) override;

 protected:
  void RunCaptureLoop();
  uint32_t PixelFormatToV4L2(const std::string& format);
  int GetChannelCount(const std::string& format);

  struct MmapBuffer {
    void* start;
    size_t length;
  };

  int fd_ = -1;
  std::string device_path_;
  std::vector<MmapBuffer> mmap_buffers_;
  std::unique_ptr<FramePool> frame_pool_;

  std::thread capture_thread_;
  std::atomic<bool> should_stop_{false};

  int width_, height_, channels_;
  std::string pixel_format_;
  uint32_t v4l2_pixel_format_;
};

#endif  // V4L2_CAMERA_H_
