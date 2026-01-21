#ifndef CAMERA_FACTORY_H_
#define CAMERA_FACTORY_H_

#include "camera_interface.h"
#include "csi2_camera.h"
#include "uvc_camera.h"

class CameraFactory {
 public:
  static std::unique_ptr<CameraInterface> CreateCamera(
      const CameraConfig& config) {
    std::unique_ptr<CameraInterface> camera;

    if (config.camera_type == "uvc") {
      camera = std::make_unique<UVCCamera>();
    } else if (config.camera_type == "mipi_csi2") {
      camera = std::make_unique<MIPICSI2Camera>();
    } else {
      std::cerr << "Unknown camera type: " << config.camera_type << std::endl;
      return nullptr;
    }

    if (camera && camera->Initialize(config)) {
      return camera;
    }

    return nullptr;
  }
};

#endif  // CAMERA_FACTORY_H_
