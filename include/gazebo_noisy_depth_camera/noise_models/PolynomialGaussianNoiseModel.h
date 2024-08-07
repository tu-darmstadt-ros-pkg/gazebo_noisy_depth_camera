#ifndef GAZEBO_NOISY_DEPTH_CAMERA_POLYNOMIALGAUSSIANNOISEMODEL_H
#define GAZEBO_NOISY_DEPTH_CAMERA_POLYNOMIALGAUSSIANNOISEMODEL_H

#include <gazebo/sensors/GaussianNoiseModel.hh>
#include <gazebo/sensors/Noise.hh>
#include <gazebo_noisy_depth_camera/noise_models/PostRenderImageNoise.h>
#include <gazebo/sensors/Noise.hh>

namespace gazebo
{
namespace sensors
{

class PolynomialGaussianNoiseModel : public Noise, public PostRenderImageNoise
{
  public:
    PolynomialGaussianNoiseModel();
  void SetCamera(rendering::CameraPtr _camera) override;
  public: void ApplyFloat(float* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat) override;

  public: void Load( sdf::ElementPtr _sdf ) override;

  private: rendering::CameraPtr camera_;
  private: std::array<double, 3> coefficients_{0.0, 0.0, 0.0};
};

}
}

#endif //GAZEBO_NOISY_DEPTH_CAMERA_POLYNOMIALGAUSSIANNOISEMODEL_H
