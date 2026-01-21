#include "v4l2_camera.h"

V4L2Camera::~V4L2Camera() {
  if (is_streaming_) StopStreaming();
  if (fd_ >= 0) close(fd_);
}

bool V4L2Camera::Initialize(const CameraConfig& config) {
  device_path_ = config.device_path;
  width_ = config.width;
  height_ = config.height;
  pixel_format_ = config.pixel_format;

  // Open device
  fd_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "Failed to open " << device_path_ << std::endl;
    return false;
  }

  // Set format
  v4l2_pixel_format_ = PixelFormatToV4L2(pixel_format_);
  channels_ = GetChannelCount(pixel_format_);

  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width_;
  fmt.fmt.pix.height = height_;
  fmt.fmt.pix.pixelformat = v4l2_pixel_format_;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
    std::cerr << "Failed to set format" << std::endl;
    return false;
  }

  // Set framerate
  struct v4l2_streamparm parm = {};
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = config.fps;
  ioctl(fd_, VIDIOC_S_PARM, &parm);

  // Apply camera controls
  SetExposureMode(config.exposure_mode);
  if (config.exposure_mode == ExposureMode::Manual) {
    SetExposureTime(config.exposure_us);
  }
  SetWhiteBalance(config.wb_r, config.wb_b);

  // Create frame pool
  frame_pool_ = std::make_unique<FramePool>(
      width_, height_, channels_, pixel_format_, config.num_buffers + 2);

  return true;
}

bool V4L2Camera::StartStreaming() {
  if (is_streaming_) return false;

  // Request buffers
  struct v4l2_requestbuffers request = {};
  request.count = 4;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd_, VIDIOC_REQBUFS, &request) < 0) {
    std::cerr << "Failed to request buffers" << std::endl;
    return false;
  }

  // Map and queue buffers
  mmap_buffers_.resize(request.count);
  for (unsigned int i = 0; i < request.count; ++i) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
      std::cerr << "Failed to query buffer" << std::endl;
      return false;
    }

    mmap_buffers_[i].length = buf.length;
    mmap_buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                  MAP_SHARED, fd_, buf.m.offset);

    if (mmap_buffers_[i].start == MAP_FAILED) {
      std::cerr << "Failed to mmap buffer" << std::endl;
      return false;
    }

    // Queue buffer
    if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
      std::cerr << "Failed to queue buffer" << std::endl;
      return false;
    }
  }

  // Start streaming
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
    std::cerr << "Failed to start streaming" << std::endl;
    return false;
  }

  is_streaming_ = true;
  should_stop_ = false;

  // Start capture thread
  capture_thread_ = std::thread(&V4L2Camera::RunCaptureLoop, this);

  return true;
}

bool V4L2Camera::StopStreaming() {
  if (!is_streaming_) return false;

  should_stop_ = true;

  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  // Stop streaming
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(fd_, VIDIOC_STREAMOFF, &type);

  // Unmap buffers
  for (auto& buf : mmap_buffers_) {
    munmap(buf.start, buf.length);
  }
  mmap_buffers_.clear();

  is_streaming_ = false;
  return true;
}

void V4L2Camera::RegisterFrameCallback(FrameCallback callback) {
  frame_callback_ = callback;
}

void V4L2Camera::RunCaptureLoop() {
  while (!should_stop_) {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(fd_, &fds);

    struct timeval tv = {1, 0};  // 1 second timeout
    int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);

    if (ret > 0) {
      struct v4l2_buffer buf = {};
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      // Dequeue buffer
      if (ioctl(fd_, VIDIOC_DQBUF, &buf) == 0) {
        // Get frame from pool
        auto frame = frame_pool_->Acquire();

        if (frame) {
          // Copy from mmap buffer
          std::memcpy(
              frame->data.get(), mmap_buffers_[buf.index].start,
              std::min(buf.bytesused, static_cast<uint32_t>(frame->data_size)));

          // Set metadata
          frame->timestamp_ns = buf.timestamp.tv_sec * 1000000000ULL +
                                buf.timestamp.tv_usec * 1000ULL;
          frame->frame_id = frame_count_++;
          frame->camera_id = camera_id_;

          // Re-queue buffer immediately
          ioctl(fd_, VIDIOC_QBUF, &buf);

          // Transfer ownership to callback
          if (frame_callback_) frame_callback_(std::move(frame));
        } else {
          // Pool exhausted - frame drop
          ioctl(fd_, VIDIOC_QBUF, &buf);
          dropped_frames_++;
        }
      }
    } else if (ret < 0 && errno != EINTR) {
      std::cerr << "Select error" << std::endl;
      break;
    }
  }
}

uint32_t V4L2Camera::PixelFormatToV4L2(const std::string& format) {
  if (format == "YUYV") return V4L2_PIX_FMT_YUYV;
  if (format == "GRAY8") return V4L2_PIX_FMT_GREY;
  if (format == "RGB8") return V4L2_PIX_FMT_RGB24;
  if (format == "MJPEG") return V4L2_PIX_FMT_MJPEG;
  return V4L2_PIX_FMT_YUYV;  // default
}

int V4L2Camera::GetChannelCount(const std::string& format) {
  if (format == "GRAY8") return 1;
  if (format == "RGB8") return 3;
  if (format == "YUYV") return 2;
  return 1;
}

// --- Camera controls ---
bool V4L2Camera::SetExposureTime(double exposure_us) {
  // 예시: 실제 V4L2 ioctl 제어 코드로 교체 필요
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  ctrl.value = static_cast<int>(exposure_us);
  return ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

bool V4L2Camera::GetExposureTime(double& exposure_us) {
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    exposure_us = ctrl.value;
    return true;
  }
  return false;
}

bool V4L2Camera::SetExposureMode(ExposureMode mode) {
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  ctrl.value =
      (mode == ExposureMode::Auto) ? V4L2_EXPOSURE_AUTO : V4L2_EXPOSURE_MANUAL;
  return ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

bool V4L2Camera::GetExposureMode(ExposureMode& mode) {
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_EXPOSURE_AUTO;
  if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    mode = (ctrl.value == V4L2_EXPOSURE_AUTO) ? ExposureMode::Auto
                                              : ExposureMode::Manual;
    return true;
  }
  return false;
}

bool V4L2Camera::SetWhiteBalance(double wb_r, double wb_b) {
  // Disable auto white balance
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
  ctrl.value = 0;
  ioctl(fd_, VIDIOC_S_CTRL, &ctrl);

  // Set R gain
  ctrl.id = V4L2_CID_RED_BALANCE;
  ctrl.value = static_cast<int>(wb_r * 256);
  ioctl(fd_, VIDIOC_S_CTRL, &ctrl);

  // Set B gain
  ctrl.id = V4L2_CID_BLUE_BALANCE;
  ctrl.value = static_cast<int>(wb_b * 256);
  return ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0;
}

bool V4L2Camera::GetWhiteBalance(double& wb_r, double& wb_b) {
  struct v4l2_control ctrl;
  ctrl.id = V4L2_CID_RED_BALANCE;
  if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    wb_r = ctrl.value / 256.0;
  }
  ctrl.id = V4L2_CID_BLUE_BALANCE;
  if (ioctl(fd_, VIDIOC_G_CTRL, &ctrl) == 0) {
    wb_b = ctrl.value / 256.0;
    return true;
  }
  return false;
}
