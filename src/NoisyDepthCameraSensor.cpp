#include <gazebo_noisy_depth_camera/NoisyDepthCameraSensor.h>
#include <gazebo_noisy_depth_camera/DepthImageGaussianNoiseModel.h>
#include <gazebo_noisy_depth_camera/MultiplicativeGaussianNoiseModel.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/DepthCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/RenderEngine.hh>

#include <functional>

using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;
using ignition::math::Angle;
using gazebo::common::Time;

extern "C"
{
  GZ_REGISTER_STATIC_SENSOR("depth", NoisyDepthCameraSensor)
}

namespace gazebo
{
namespace sensors
{

struct NoisyDepthCameraSensorPrivate
{
  /// \brief Local pointer to the depthCamera.
  rendering::DepthCameraPtr depthCamera;
  float *depthBuffer = nullptr;

  event::ConnectionPtr worldResetConnection;
  event::ConnectionPtr timeResetConnection;
  event::ConnectionPtr depthFrameConnection;

  explicit NoisyDepthCameraSensorPrivate() = default;
};

NoisyDepthCameraSensor::NoisyDepthCameraSensor() :
  dataPtr(new NoisyDepthCameraSensorPrivate())
{
}

void NoisyDepthCameraSensor::Load(const std::string &_worldName)
{

  DepthCameraSensor::Load(_worldName);

  gzmsg << "Noisy depth camera loaded" << std::endl;
}

void NoisyDepthCameraSensor::Init()
{
  if (!depthCameraSensorInit()) return;

  sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
  if (cameraSdf->HasElement("noise"))
  {
    registerNoiseCallback(cameraSdf->GetElement("noise"));
  }

  // Load other plugins
  Sensor::Init();

  this->dataPtr->worldResetConnection = event::Events::ConnectWorldReset(
      std::bind(&NoisyDepthCameraSensor::Reset, this));
  this->dataPtr->timeResetConnection = event::Events::ConnectTimeReset(
      std::bind(&NoisyDepthCameraSensor::Reset, this));
}

bool NoisyDepthCameraSensor::registerNoiseCallback(const sdf::ElementPtr& _sdf)
{
  this->noises[CAMERA_NOISE] =
      this->CreateNoiseModel(_sdf, this->Type());

  this->noises[CAMERA_NOISE]->SetCamera(this->camera);

  auto postRenderImageNoise = std::dynamic_pointer_cast<PostRenderImageNoise>(
      this->noises[CAMERA_NOISE]);
  if (postRenderImageNoise != nullptr)
  {
    const auto nearClip = this->dataPtr->depthCamera->NearClip();
    const auto farClip = this->dataPtr->depthCamera->FarClip();
    this->dataPtr->depthFrameConnection = this->DepthCamera()->ConnectNewDepthFrame(
        [postRenderImageNoise,nearClip,farClip](const float* _buffer, size_t _width, size_t _height, size_t _depth, const std::string& _pixelFormat)
        {
          // HACK: there's no better way to alter the generated depth image than hooking the
          // newDepthFrame callback which is passing a const pointer to the data.
          // But we know (by calling this before Sensor::Init()) that we'll be the first hook
          // that gets called, and we also know that we can const_cast the passed data
          // (because the underlying data structure is on the heap, which is always modifiable).
          auto writableBuffer = const_cast<float*>(_buffer);
          postRenderImageNoise->ApplyFloat(writableBuffer, _width, _height, _depth, _pixelFormat);
          for (size_t i = 0; i < _width * _height * _depth; ++i)
          {
            if (writableBuffer[i] < nearClip)
              writableBuffer[i] = -ignition::math::INF_F;
            else if (writableBuffer[i] > farClip)
              writableBuffer[i] = ignition::math::INF_F;
          }
        }
    );
  }
  return false;
}


bool NoisyDepthCameraSensor::depthCameraSensorInit()
{
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
       rendering::RenderEngine::NONE)
  {
    gzerr << "Unable to create DepthCameraSensor. Rendering is disabled."
          << std::endl;
    return false;
  }

  std::string worldName = this->world->Name();
  if (worldName.empty()) {
    gzerr << "No world name" << std::endl;
    return false;
  }

  this->scene = rendering::get_scene(worldName);
  if (!this->scene)
    this->scene = rendering::create_scene(worldName, false, true);

  this->dataPtr->depthCamera = this->scene->CreateDepthCamera(
      this->sdf->Get<std::string>("name"), false);
  if (!this->dataPtr->depthCamera)
  {
    gzerr << "Unable to create depth camera sensor" << std::endl;
    return false;
  }
  this->dataPtr->depthCamera->SetCaptureData(true);
  sdf::ElementPtr cameraSdf = this->sdf->GetElement("camera");
  this->dataPtr->depthCamera->Load(cameraSdf);
  // Do some sanity checks
  if (this->dataPtr->depthCamera->ImageWidth() == 0u ||
       this->dataPtr->depthCamera->ImageHeight() == 0u)
  {
    gzerr << "image has zero size" << std::endl;
  }

  this->dataPtr->depthCamera->Init();
  this->dataPtr->depthCamera->CreateRenderTexture(
      this->Name() + "_RttTex_Image");
  this->dataPtr->depthCamera->CreateDepthTexture(
      this->Name() + "_RttTex_Depth");
  this->dataPtr->depthCamera->CreateReflectanceTexture(
      this->Name() + "_RttTex_Reflectance");
  this->dataPtr->depthCamera->CreateNormalsTexture(
      this->Name() + "_RttTex_Normals");
  ignition::math::Pose3d cameraPose = this->pose;
  if (cameraSdf->HasElement("pose"))
    cameraPose = cameraSdf->Get<ignition::math::Pose3d>("pose") + cameraPose;

  this->dataPtr->depthCamera->SetWorldPose(cameraPose);
  this->dataPtr->depthCamera->AttachToVisual(this->parentId, true, 0, 0);
  this->camera = boost::dynamic_pointer_cast<rendering::Camera>(
      this->dataPtr->depthCamera);

  GZ_ASSERT(this->camera, "Unable to cast depth camera to camera");

  // Disable clouds and moon on server side until fixed and also to improve
  // performance
  this->scene->SetSkyXMode(rendering::Scene::GZ_SKYX_ALL &
                            ~rendering::Scene::GZ_SKYX_CLOUDS &
                            ~rendering::Scene::GZ_SKYX_MOON);
  return true;
}


NoisyDepthCameraSensor::~NoisyDepthCameraSensor() // NOLINT(hicpp-use-equals-default,modernize-use-equals-default)
{
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
}

void NoisyDepthCameraSensor::Reset()
{
}

NoisePtr NoisyDepthCameraSensor::CreateNoiseModel(sdf::ElementPtr _sdf,
                                                  const std::string &_sensorType)
{
  GZ_ASSERT(_sdf != nullptr, "noise sdf is null");
  GZ_ASSERT(_sdf->GetName() == "noise", "Not a noise SDF element");

  std::string typeString = _sdf->Get<std::string>("type");

  if (typeString == "gaussian" && _sensorType == "depth")
  {
    NoisePtr noise(new DepthImageGaussianNoiseModel());

    GZ_ASSERT(noise->GetNoiseType() == Noise::GAUSSIAN,
              "Noise type should be 'gaussian'");

    noise->Load(_sdf);
    auto gaussian_noise_ptr = std::dynamic_pointer_cast<GaussianNoiseModel>(noise);
    gzmsg << "Created depth camera " << this->Name() << " with Gaussian noise (mean = " << gaussian_noise_ptr->GetMean() << ", stddev = " << gaussian_noise_ptr->GetStdDev() << ")" << std::endl;
    return noise;
  }
  else if (typeString == "gaussian_multiplicative" && _sensorType == "depth")
  {
    NoisePtr noise(new MultiplicativeGaussianNoiseModel());

    GZ_ASSERT(noise->GetNoiseType() == Noise::GAUSSIAN,
              "Noise type should be 'gaussian'");

    noise->Load(_sdf);
    return noise;
  }
  // TODO: implement stereo noise model from:
  // - https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6375037
  // - https://github.com/HannesKeller/sensor_model
  else
  {
    return NoiseFactory::NewNoiseModel(_sdf, _sensorType);
  }
}

rendering::DepthCameraPtr NoisyDepthCameraSensor::DepthCamera() const
{
  return this->dataPtr->depthCamera;
}

const float *NoisyDepthCameraSensor::DepthData() const {
  return this->dataPtr->depthBuffer;
}

bool NoisyDepthCameraSensor::UpdateImpl( const bool _force )
{
  IGN_PROFILE("DepthCameraSensor::UpdateImpl");
  if (!this->Rendered())
    return false;

  IGN_PROFILE_BEGIN("PostRender");
  this->camera->PostRender();
  IGN_PROFILE_END();

  IGN_PROFILE_BEGIN("fillarray");

  if (this->imagePub && this->imagePub->HasConnections() &&
       // check if depth data is available. If not, the depth camera could be
       // generating point clouds instead
       this->dataPtr->depthCamera->DepthData())
  {
    msgs::ImageStamped msg;
    msgs::Set(msg.mutable_time(), this->scene->SimTime());
    msg.mutable_image()->set_width(this->camera->ImageWidth());
    msg.mutable_image()->set_height(this->camera->ImageHeight());
    msg.mutable_image()->set_pixel_format(common::Image::R_FLOAT32);


    msg.mutable_image()->set_step(this->camera->ImageWidth() *
                                   this->camera->ImageDepth());

    unsigned int depthSamples = msg.image().width() * msg.image().height();
    float f;
    // cppchecker recommends using sizeof(varname)
    unsigned int depthBufferSize = depthSamples * sizeof(f);

    if (!this->dataPtr->depthBuffer)
      this->dataPtr->depthBuffer = new float[depthSamples];

    memcpy(this->dataPtr->depthBuffer, this->dataPtr->depthCamera->DepthData(),
            depthBufferSize);

    for (unsigned int i = 0; i < depthSamples; ++i)
    {
      // Mask ranges outside of min/max to +/- inf, as per REP 117
      if (this->dataPtr->depthBuffer[i] >= this->camera->FarClip())
      {
        this->dataPtr->depthBuffer[i] = ignition::math::INF_D;
      }
      else if (this->dataPtr->depthBuffer[i] <= this->camera->NearClip())
      {
        this->dataPtr->depthBuffer[i] = -ignition::math::INF_D;
      }
    }
    msg.mutable_image()->set_data(this->dataPtr->depthBuffer, depthBufferSize);
    this->imagePub->Publish(msg);
  }

  this->SetRendered(false);
  IGN_PROFILE_END();
  return true;
}

}
}