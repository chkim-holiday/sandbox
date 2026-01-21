#include <iostream>

#include "camera_factory.h"
#include "camera_interface.h"
#include "csi2_camera.h"
#include "uvc_camera.h"

#include "opencv4/opencv2/opencv.hpp"

/*
2.1.6 How to use the control function

    Disable auto exposure

v4l2-ctl -c auto_exposure=1

    Set up exposure

v4l2-ctl -c exposure_time_absolute=500

    Enable the external trigger snapshot mode

v4l2-ctl -c exposure_dynamic_framerate=1

*/

int main() {
  // Configuration
  CameraConfig config;
  config.camera_type = "uvc";
  config.device_path = "/dev/video0";
  config.width = 1280;
  config.height = 800;
  config.fps = 30;
  config.pixel_format = "GRAY8";
  config.exposure_us = 10000;
  config.exposure_mode = ExposureMode::Manual;

  // Create camera
  auto camera = CameraFactory::CreateCamera(config);
  if (!camera) {
    return -1;
  }

  // Register callback
  camera->RegisterFrameCallback([](std::shared_ptr<Frame> frame) {
    std::cout << "Frame " << frame->frame_id << " @ " << frame->timestamp_ns
              << " ns" << std::endl;

    // Zero-copy cv::Mat
    cv::Mat image = frame->ToCvMat();

    // Process image
    // ...

    // frame automatically returns to pool when out of scope
  });

  camera->SetExposureMode(ExposureMode::Manual);
  camera->SetExposureTime(5000.0);  // 5 ms

  // Start streaming
  camera->StartStreaming();

  // Let it run
  std::this_thread::sleep_for(std::chrono::seconds(10));

  // Stop streaming
  camera->StopStreaming();

  // Print statistics
  std::cout << "Total frames: " << camera->GetFrameCount() << std::endl;
  std::cout << "Dropped frames: " << camera->GetDroppedFrames() << std::endl;

  return 0;
}
