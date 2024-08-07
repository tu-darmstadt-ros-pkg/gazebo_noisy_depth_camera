#include <gazebo_noisy_depth_camera/noise_models/PolynomialGaussianNoiseModel.h>
#include <gazebo/rendering/Camera.hh>
#include <ignition/math/Rand.hh>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

namespace gazebo
{
namespace sensors
{

PolynomialGaussianNoiseModel::PolynomialGaussianNoiseModel() : Noise(Noise::CUSTOM) { }

void PolynomialGaussianNoiseModel::SetCamera(rendering::CameraPtr _camera)
{
  camera_ = _camera;
}

void PolynomialGaussianNoiseModel::ApplyFloat(float* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat)
{
  if (_pixelFormat == "FLOAT32")
  {
    for (size_t i = 0; i < _width * _height; ++i)
    {
      if (_buffer[i] == camera_->NearClip() || _buffer[i] == camera_->FarClip())
        continue;

      double stddev = coefficients_[0] + coefficients_[1] * _buffer[i] + coefficients_[2] * _buffer[i] * _buffer[i];
      _buffer[i] += static_cast<float>(
          ignition::math::Rand::DblNormal(0.0, stddev));

      // TODO add angle-dependent axial noise
      // TODO add lateral noise
    }
  }
  else
    throw std::runtime_error("Unknown image pixel format.");
}

void PolynomialGaussianNoiseModel::Load( sdf::ElementPtr _sdf ) {
  Noise::Load( _sdf );
  auto coefficients_str = _sdf->Get<std::string>("coefficients");
  if (coefficients_str.empty()) {
    gzerr << "Did not find coefficients parameter of polynomial gaussian noise model." << std::endl;
    return;
  }
  std::vector<std::string> coefficients_str_vec;
  boost::split(coefficients_str_vec, coefficients_str, boost::is_any_of(" "), boost::token_compress_on);
  if (coefficients_str_vec.size() != 3) {
    gzerr << "Expected 3 coefficients for the polynomial gaussian noise model, received " << coefficients_str_vec.size() << std::endl;
    return;
  }
  for (unsigned int i = 0; i < 3; ++i) {
    try {
      coefficients_[i] = std::stod(coefficients_str_vec[i]);
    } catch (const std::exception& e) {
      gzerr << "Could not convert " << coefficients_str_vec[i] << " to a double: " << e.what() << std::endl;
    }
  }
}

}
}