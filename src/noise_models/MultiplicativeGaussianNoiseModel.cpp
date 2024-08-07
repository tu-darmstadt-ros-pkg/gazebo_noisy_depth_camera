#include <gazebo_noisy_depth_camera/noise_models/MultiplicativeGaussianNoiseModel.h>
#include <gazebo/rendering/Camera.hh>
#include <ignition/math/Rand.hh>

namespace gazebo
{
namespace sensors
{

void MultiplicativeGaussianNoiseModel::SetCamera(rendering::CameraPtr _camera)
{
  camera_ = _camera;
}

void MultiplicativeGaussianNoiseModel::ApplyFloat(float* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat)
{
  if (_pixelFormat == "FLOAT32")
  {
    for (size_t i = 0; i < _width * _height; ++i)
    {
      if (_buffer[i] == camera_->NearClip() || _buffer[i] == camera_->FarClip())
        continue;

      _buffer[i] *= static_cast<float>(
          ignition::math::Rand::DblNormal(this->mean, this->stdDev) + this->bias);
    }
  }
  else
    throw std::runtime_error("Unknown image pixel format.");
}

}
}