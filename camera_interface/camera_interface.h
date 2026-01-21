#ifndef CAMERA_INTERFACE_H_
#define CAMERA_INTERFACE_H_

#include <atomic>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/videodev2.h>

#include "opencv4/opencv2/opencv.hpp"

// Frame data structure
struct Frame {
  std::shared_ptr<uint8_t[]> data;
  size_t data_size;

  int width;
  int height;
  int channels;
  std::string pixel_format;  // "GRAY8", "RGB8", "YUYV", etc.

  uint64_t timestamp_ns;
  uint64_t frame_id;
  int camera_id;

  // Optional: MCU trigger timestamp
  std::optional<uint64_t> trigger_timestamp_ns;

  // Zero-copy cv::Mat wrapper
  cv::Mat ToCvMat() const {
    int cv_type = (channels == 1)   ? CV_8UC1
                  : (channels == 3) ? CV_8UC3
                                    : CV_8UC4;
    return cv::Mat(height, width, cv_type, data.get());
  }
};

using FrameCallback = std::function<void(std::shared_ptr<Frame> frame)>;

enum class ExposureMode { Manual, Auto };
enum class TriggerMode { FreeRun, Hardware, Software };

// Configuration
struct CameraConfig {
  std::string camera_type;  // "uvc" or "mipi_csi2"
  std::string device_path;  // e.g., "/dev/video0"

  int width = 1920;
  int height = 1080;
  int fps = 30;
  std::string pixel_format = "YUYV";

  double exposure_us = 10000.0;
  ExposureMode exposure_mode = ExposureMode::Manual;
  TriggerMode trigger_mode = TriggerMode::FreeRun;
  double wb_r = 1.0;
  double wb_b = 1.0;

  int num_buffers = 4;
};

// Base interface
class CameraInterface {
 public:
  virtual ~CameraInterface() = default;

  // Initialization
  virtual bool Initialize(const CameraConfig& config) = 0;

  // Callback registration
  virtual void RegisterFrameCallback(FrameCallback callback) = 0;

  // Streaming control
  virtual bool StartStreaming() = 0;
  virtual bool StopStreaming() = 0;
  virtual bool IsStreaming() const { return is_streaming_; }

  // Camera controls
  virtual bool SetExposureTime(double exposure_us) = 0;
  virtual bool GetExposureTime(double& exposure_us) = 0;

  virtual bool SetExposureMode(ExposureMode mode) = 0;
  virtual bool GetExposureMode(ExposureMode& mode) = 0;

  virtual bool SetTriggerMode(TriggerMode mode) = 0;
  virtual bool GetTriggerMode(TriggerMode& mode) = 0;

  virtual bool SetWhiteBalance(double wb_r, double wb_b) = 0;
  virtual bool GetWhiteBalance(double& wb_r, double& wb_b) = 0;

  // Camera info
  virtual std::string GetCameraType() const = 0;
  virtual int GetCameraId() const { return camera_id_; }

  // Statistics
  virtual uint64_t GetFrameCount() const { return frame_count_; }
  virtual uint64_t GetDroppedFrames() const { return dropped_frames_; }

 protected:
  FrameCallback frame_callback_;
  std::atomic<bool> is_streaming_{false};
  std::atomic<uint64_t> frame_count_{0};
  std::atomic<uint64_t> dropped_frames_{0};
  int camera_id_ = 0;

  // Delete copy, allow move
  CameraInterface() = default;
  CameraInterface(const CameraInterface&) = delete;
  CameraInterface& operator=(const CameraInterface&) = delete;
  CameraInterface(CameraInterface&&) = default;
  CameraInterface& operator=(CameraInterface&&) = default;
};

class FramePool {
 public:
  FramePool(int width, int height, int channels,
            const std::string& pixel_format, size_t pool_size = 6)
      : width_(width),
        height_(height),
        channels_(channels),
        pixel_format_(pixel_format) {
    buffer_size_ = width * height * channels;

    // Pre-allocate buffers
    for (size_t i = 0; i < pool_size; ++i) {
      auto frame = std::make_shared<Frame>();
      frame->data = std::shared_ptr<uint8_t[]>(
          new uint8_t[buffer_size_], std::default_delete<uint8_t[]>());
      frame->data_size = buffer_size_;
      frame->width = width;
      frame->height = height;
      frame->channels = channels;
      frame->pixel_format = pixel_format;

      pool_.push(frame);
    }
  }

  std::shared_ptr<Frame> Acquire() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (pool_.empty()) {
      return nullptr;  // Pool exhausted - frame will be dropped
    }

    auto frame = pool_.front();
    pool_.pop();

    // Custom deleter to return frame to pool
    return std::shared_ptr<Frame>(frame.get(), [this, frame](Frame*) mutable {
      std::lock_guard<std::mutex> lock(this->mutex_);
      this->pool_.push(std::move(frame));
    });
  }

  size_t AvailableBuffers() {
    std::lock_guard<std::mutex> lock(mutex_);
    return pool_.size();
  }

 private:
  std::queue<std::shared_ptr<Frame>> pool_;
  std::mutex mutex_;
  size_t buffer_size_;
  int width_, height_, channels_;
  std::string pixel_format_;
};

#endif  // CAMERA_INTERFACE_H_
