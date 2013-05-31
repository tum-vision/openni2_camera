/**
 * Copyright (c) 2013 Christian Kerl <christian.kerl@in.tum.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <openni2_camera/camera.h>
#include <openni2_camera/CameraConfig.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>

#include <boost/bind.hpp>

namespace openni2_camera
{

namespace internal
{

using namespace openni;

int findVideoMode(const Array<VideoMode>& modes, int x, int y, PixelFormat format, int fps = 30)
{
  int result = 0;

  for(int idx = 0; idx < modes.getSize(); ++idx)
  {
    if(modes[idx].getResolutionX() == x && modes[idx].getResolutionY() == y && modes[idx].getPixelFormat() == format && modes[idx].getFps() == fps)
    {
      result = idx;
      break;
    }
  }

  return result;
}

std::string toString(const PixelFormat& format)
{
  switch(format)
  {
  case PIXEL_FORMAT_DEPTH_1_MM:
    return "DEPTH_1_MM";
  case PIXEL_FORMAT_DEPTH_100_UM:
    return "DEPTH_100_UM";
  case PIXEL_FORMAT_SHIFT_9_2:
    return "SHIFT_9_2";
  case PIXEL_FORMAT_SHIFT_9_3:
    return "SHIFT_9_3";

  case PIXEL_FORMAT_RGB888:
    return "RGB888";
  case PIXEL_FORMAT_YUV422:
    return "YUV422";
  case PIXEL_FORMAT_GRAY8:
    return "GRAY8";
  case PIXEL_FORMAT_GRAY16:
    return "GRAY16";
  case PIXEL_FORMAT_JPEG:
    return "JPEG";
  default:
    return "unknown";
  }
}

std::string toString(const SensorType& type)
{
  switch(type)
  {
  case SENSOR_COLOR:
    return "COLOR";
  case SENSOR_DEPTH:
    return "DEPTH";
  case SENSOR_IR:
    return "IR";
  default:
    return "unknown";
  }
}

class MethodNotSupportedException : std::exception
{
private:
  std::string cause_;
public:
  MethodNotSupportedException(const char* method) throw()
  {
    cause_ = "Method '" + std::string(method) + "' is not supported!";
  }
  virtual ~MethodNotSupportedException() throw() { }

  virtual const char* what() const throw()
  {
    return cause_.c_str();
  }
};

class SensorStreamManagerBase
{
public:
  SensorStreamManagerBase() {}
  virtual ~SensorStreamManagerBase() {}

  virtual VideoStream& stream()
  {
    throw MethodNotSupportedException("SensorStreamManagerBase::stream()");
  }

  virtual bool beginConfigure()
  {
    return false;
  }

  virtual bool tryConfigureVideoMode(VideoMode& mode)
  {
    throw MethodNotSupportedException("SensorStreamManagerBase::tryConfigureVideoMode()");
  }

  virtual void endConfigure()
  {
    throw MethodNotSupportedException("SensorStreamManagerBase::endConfigure()");
  }

  virtual void advertise(image_transport::ImageTransport& it)
  {
    throw MethodNotSupportedException("SensorStreamManagerBase::advertise()");
  }
};

class SensorStreamManager : public SensorStreamManagerBase, public VideoStream::NewFrameListener
{
protected:
  Device& device_;
  VideoStream stream_;
  VideoMode default_mode_;
  std::string name_;
  bool running_, was_running_;

  image_transport::CameraPublisher publisher_;
  image_transport::SubscriberStatusCallback callback_;

  virtual void publish(sensor_msgs::Image& image, sensor_msgs::CameraInfo& camera_info)
  {
    publisher_.publish(image, camera_info);
  }
public:
  SensorStreamManager(Device& device, SensorType type, std::string name, VideoMode& default_mode) :
    device_(device),
    default_mode_(default_mode),
    name_(name),
    running_(false)
  {
    assert(device_.hasSensor(type));

    ROS_ERROR_STREAM_COND(stream_.create(device_, type) != STATUS_OK, "Failed to create stream '" << toString(type) << "'!");
    stream_.addNewFrameListener(this);
    ROS_ERROR_STREAM_COND(stream_.setVideoMode(default_mode_) != STATUS_OK, "Failed to set default video mode for stream '" << toString(type) << "'!");
  }

  virtual ~SensorStreamManager()
  {
    stream_.removeNewFrameListener(this);
    stream_.stop();
    stream_.destroy();

    publisher_.shutdown();
  }

  virtual VideoStream& stream()
  {
    return stream_;
  }

  virtual void advertise(image_transport::ImageTransport& it)
  {
    callback_ = boost::bind(&SensorStreamManager::onSubscriptionChanged, this, _1);
    publisher_ = it.advertiseCamera(name_ + "/image_raw", 1, callback_, callback_);
  }

  virtual bool beginConfigure()
  {
    was_running_ = running_;
    if(was_running_) stream_.stop();
    running_ = false;

    return true;
  }

  virtual void endConfigure()
  {
    if(was_running_)
    {
      Status rc = stream_.start();

      if(rc != STATUS_OK)
      {
        SensorType type = stream_.getSensorInfo().getSensorType();
        ROS_WARN_STREAM("Failed to restart stream '" << name_ << "' after configuration!");

        int max_trials = 1;

        for(int trials = 0; trials < max_trials && rc != STATUS_OK; ++trials)
        {
          ros::Duration(0.1).sleep();

          stream_.removeNewFrameListener(this);
          stream_.destroy();
          stream_.create(device_, type);
          stream_.addNewFrameListener(this);
          //stream_.setVideoMode(default_mode_);
          rc = stream_.start();

          ROS_WARN_STREAM_COND(rc != STATUS_OK, "Recovery trial " << trials << " failed!");
        }

        ROS_ERROR_STREAM_COND(rc != STATUS_OK, "Failed to recover stream '" << name_ << "'! Restart required!");
        ROS_INFO_STREAM_COND(rc == STATUS_OK, "Recovered stream '" << name_ << "'.");
      }

      if(rc == STATUS_OK)
      {
        running_ = true;
      }
    }
  }

  virtual bool tryConfigureVideoMode(VideoMode& mode)
  {
    bool result = true;
    VideoMode old = stream_.getVideoMode();

    if(stream_.setVideoMode(mode) != STATUS_OK)
    {
      ROS_ERROR_STREAM_COND(stream_.setVideoMode(old) != STATUS_OK, "Failed to recover old video mode!");
      result = false;
    }

    return result;
  }

  virtual void onSubscriptionChanged(const image_transport::SingleSubscriberPublisher& topic)
  {
    if(topic.getNumSubscribers() > 0)
    {
      if(!running_ && stream_.start() == STATUS_OK)
      {
        running_ = true;
      }
    }
    else
    {
      stream_.stop();
      running_ = false;
    }
  }

  virtual void onNewFrame(VideoStream& stream)
  {
    ros::Time ts = ros::Time::now();

    VideoFrameRef frame;
    stream.readFrame(&frame);

    sensor_msgs::Image img;
    sensor_msgs::CameraInfo info;

    double scale = double(frame.getWidth()) / double(1280);

    info.header.stamp = ts;
    info.width = frame.getWidth();
    info.height = frame.getHeight();
    info.K.assign(0);
    info.K[0] = 1050.0 * scale;
    info.K[4] = 1050.0 * scale;
    info.K[2] = frame.getWidth() / 2.0 - 0.5;
    info.K[5] = frame.getHeight() / 2.0 - 0.5;
    info.P.assign(0);
    info.P[0] = 1050.0 * scale;
    info.P[5] = 1050.0 * scale;
    info.P[2] = frame.getWidth() / 2.0 - 0.5;
    info.P[6] = frame.getHeight() / 2.0 - 0.5;

    switch(frame.getVideoMode().getPixelFormat())
    {
    case PIXEL_FORMAT_GRAY8:
      img.encoding = sensor_msgs::image_encodings::MONO8;
      break;
    case PIXEL_FORMAT_GRAY16:
      img.encoding = sensor_msgs::image_encodings::MONO16;
      break;
    case PIXEL_FORMAT_YUV422:
      img.encoding = sensor_msgs::image_encodings::YUV422;
      break;
    case PIXEL_FORMAT_RGB888:
      img.encoding = sensor_msgs::image_encodings::RGB8;
      break;
    case PIXEL_FORMAT_SHIFT_9_2:
    case PIXEL_FORMAT_DEPTH_1_MM:
      img.encoding = sensor_msgs::image_encodings::MONO16;
      break;
    default:
      ROS_WARN("Unknown OpenNI pixel format!");
      break;
    }
    img.header.stamp = ts;
    img.height = frame.getHeight();
    img.width = frame.getWidth();
    img.step = frame.getStrideInBytes();
    img.data.resize(frame.getDataSize());
    std::copy(static_cast<const uint8_t*>(frame.getData()), static_cast<const uint8_t*>(frame.getData()) + frame.getDataSize(), img.data.begin());

    publish(img, info);
  }
};

class DepthSensorStreamManager : public SensorStreamManager
{
protected:
  image_transport::CameraPublisher depth_registered_publisher_, disparity_publisher_, disparity_registered_publisher_, *active_publisher_;

  virtual void publish(sensor_msgs::Image& image, sensor_msgs::CameraInfo& camera_info)
  {
    active_publisher_->publish(image, camera_info);
  }

  void updateActivePublisher()
  {
    if(device_.getImageRegistrationMode() == IMAGE_REGISTRATION_DEPTH_TO_COLOR)
    {
      if(stream_.getVideoMode().getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM)
      {
        active_publisher_ = &depth_registered_publisher_;
      }
      else if(stream_.getVideoMode().getPixelFormat() == PIXEL_FORMAT_SHIFT_9_2)
      {
        active_publisher_ = &disparity_registered_publisher_;
      }
    }
    else
    {
      if(stream_.getVideoMode().getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM)
      {
        active_publisher_ = &publisher_;
      }
      else if(stream_.getVideoMode().getPixelFormat() == PIXEL_FORMAT_SHIFT_9_2)
      {
        active_publisher_ = &disparity_publisher_;
      }
    }
  }
public:
  DepthSensorStreamManager(Device& device, VideoMode& default_mode) : SensorStreamManager(device, SENSOR_DEPTH, "depth", default_mode)
  {
  }

  virtual void advertise(image_transport::ImageTransport& it)
  {
    SensorStreamManager::advertise(it);

    depth_registered_publisher_ = it.advertiseCamera(name_ + "_registered/image_raw", 1, callback_, callback_);
    disparity_publisher_  = it.advertiseCamera(name_ + "/disparity", 1, callback_, callback_);
    disparity_registered_publisher_  = it.advertiseCamera(name_ + "_registered/disparity", 1, callback_, callback_);
  }

  virtual void onSubscriptionChanged(const image_transport::SingleSubscriberPublisher& topic)
  {
    size_t disparity_clients = disparity_publisher_.getNumSubscribers() + disparity_registered_publisher_.getNumSubscribers();
    size_t depth_clients = publisher_.getNumSubscribers() + depth_registered_publisher_.getNumSubscribers();
    size_t all_clients = disparity_clients + depth_clients;

    if(!running_ && all_clients > 0)
    {
      running_ = (stream_.start() == STATUS_OK);
    }
    else if(running_ && all_clients == 0)
    {
      stream_.stop();
      running_ = false;
    }

    if(running_)
    {
      updateActivePublisher();
    }
  }

  virtual void endConfigure()
  {
    SensorStreamManager::endConfigure();

    if(running_)
    {
      updateActivePublisher();
    }
  }
};

class CameraImpl
{
public:
  CameraImpl(ros::NodeHandle& nh, ros::NodeHandle& nh_private, openni::Device& device) :
    it_(nh),
    rgb_sensor_(new SensorStreamManagerBase()),
    depth_sensor_(new SensorStreamManagerBase()),
    ir_sensor_(new SensorStreamManagerBase()),
    reconfigure_server_(nh),
    device_(device)
  {
    printDeviceInfo();
    printVideoModes();
    buildResolutionMap();

    device_.setDepthColorSyncEnabled(true);

    if(device_.hasSensor(SENSOR_COLOR))
    {
      rgb_sensor_.reset(new SensorStreamManager(device_, SENSOR_COLOR, "rgb", resolutions_[Camera_RGB_640x480_30Hz]));
      rgb_sensor_->advertise(it_);
    }

    if(device_.hasSensor(SENSOR_DEPTH))
    {
      depth_sensor_.reset(new DepthSensorStreamManager(device_, resolutions_[Camera_DEPTH_640x480_30Hz]));
      depth_sensor_->advertise(it_);
    }

    if(device_.hasSensor(SENSOR_IR))
    {
      ir_sensor_.reset(new SensorStreamManager(device_, SENSOR_IR, "ir", resolutions_[Camera_IR_640x480_30Hz]));
      ir_sensor_->advertise(it_);
    }

    reconfigure_server_.setCallback(boost::bind(&CameraImpl::configure, this, _1, _2));
  }

  ~CameraImpl()
  {
  }

  void printDeviceInfo()
  {
    const DeviceInfo& info = device_.getDeviceInfo();

    char buffer[512];
    int size = 512;
    std::stringstream summary;

    size = 512;
    if(device_.getProperty(DEVICE_PROPERTY_HARDWARE_VERSION, buffer, &size) == STATUS_OK)
    {
      std::string hw(buffer, size_t(size));
      summary << " Hardware: " << hw;
    }

    size = 512;
    if(device_.getProperty(DEVICE_PROPERTY_FIRMWARE_VERSION, buffer, &size) == STATUS_OK)
    {
      std::string fw(buffer, size);
      summary << " Firmware: " << fw;
    }

    size = 512;
    if(device_.getProperty(DEVICE_PROPERTY_DRIVER_VERSION, buffer, &size) == STATUS_OK)
    {
      std::string drv(buffer, size);
      summary << " Driver: " << drv;
    }

    ROS_INFO_STREAM(info.getVendor() << " " << info.getName() << summary.str());
  }

  void printVideoModes()
  {
    static const size_t ntypes = 3;
    SensorType types[ntypes] = { SENSOR_COLOR, SENSOR_DEPTH, SENSOR_IR };

    for(size_t tidx = 0; tidx < ntypes; ++tidx)
    {
      if(!device_.hasSensor(types[tidx])) continue;

      const SensorInfo* info = device_.getSensorInfo(types[tidx]);
      ROS_INFO_STREAM("  " << toString(info->getSensorType()));

      const Array<VideoMode>& modes = info->getSupportedVideoModes();

      for(int idx = 0; idx < modes.getSize(); ++idx)
      {
        const VideoMode& mode = modes[idx];
        ROS_INFO_STREAM("    " << toString(mode.getPixelFormat()) << " " << mode.getResolutionX() << "x" << mode.getResolutionY() << "@" << mode.getFps());
      }
    }
  }

  void createVideoMode(VideoMode& m, int x, int y, int fps, PixelFormat format)
  {
    m.setResolution(x, y);
    m.setFps(fps);
    m.setPixelFormat(format);
  }

  void buildResolutionMap()
  {
    createVideoMode(resolutions_[Camera_RGB_320x240_30Hz], 320, 240, 30, PIXEL_FORMAT_RGB888);
    createVideoMode(resolutions_[Camera_RGB_320x240_60Hz], 320, 240, 60, PIXEL_FORMAT_RGB888);
    createVideoMode(resolutions_[Camera_RGB_640x480_30Hz], 640, 480, 30, PIXEL_FORMAT_RGB888);
    // i don't think this one is supported as it is overridden in XnHostProtocol.cpp#L429
    createVideoMode(resolutions_[Camera_RGB_1280x720_30Hz], 1280, 720, 30, PIXEL_FORMAT_GRAY8);
    createVideoMode(resolutions_[Camera_RGB_1280x1024_30Hz], 1280, 1024, 30, PIXEL_FORMAT_GRAY8);

    createVideoMode(resolutions_[Camera_DEPTH_320x240_30Hz], 320, 240, 30, PIXEL_FORMAT_DEPTH_1_MM);
    createVideoMode(resolutions_[Camera_DEPTH_320x240_60Hz], 320, 240, 60, PIXEL_FORMAT_DEPTH_1_MM);
    createVideoMode(resolutions_[Camera_DEPTH_640x480_30Hz], 640, 480, 30, PIXEL_FORMAT_DEPTH_1_MM);

    createVideoMode(resolutions_[Camera_DISPARITY_320x240_30Hz], 320, 240, 30, PIXEL_FORMAT_SHIFT_9_2);
    createVideoMode(resolutions_[Camera_DISPARITY_320x240_60Hz], 320, 240, 60, PIXEL_FORMAT_SHIFT_9_2);
    createVideoMode(resolutions_[Camera_DISPARITY_640x480_30Hz], 640, 480, 30, PIXEL_FORMAT_SHIFT_9_2);

    createVideoMode(resolutions_[Camera_IR_320x240_30Hz], 320, 240, 30, PIXEL_FORMAT_GRAY8);
    createVideoMode(resolutions_[Camera_IR_320x240_60Hz], 320, 240, 60, PIXEL_FORMAT_GRAY8);
    createVideoMode(resolutions_[Camera_IR_640x480_30Hz], 640, 480, 30, PIXEL_FORMAT_GRAY8);
    createVideoMode(resolutions_[Camera_IR_1280x1024_30Hz], 1280, 1024, 30, PIXEL_FORMAT_GRAY8);
  }

  void configure(CameraConfig& cfg, uint32_t level)
  {
    if(rgb_sensor_->beginConfigure())
    {
      if((level & 8) != 0)
      {
        ResolutionMap::iterator e = resolutions_.find(cfg.rgb_resolution);
        assert(e != resolutions_.end());

        rgb_sensor_->tryConfigureVideoMode(e->second);
      }

      if((level & 2) != 0)
      {
        rgb_sensor_->stream().getCameraSettings()->setAutoExposureEnabled(cfg.auto_exposure);
      }

      if((level & 4) != 0)
      {
        rgb_sensor_->stream().getCameraSettings()->setAutoWhiteBalanceEnabled(cfg.auto_white_balance);
      }

      if((level & 64) != 0)
      {
        rgb_sensor_->stream().setMirroringEnabled(cfg.mirror);
      }
      rgb_sensor_->endConfigure();
    }

    if(depth_sensor_->beginConfigure())
    {
      if((level & 16) != 0)
      {
        ResolutionMap::iterator e = resolutions_.find(cfg.depth_resolution);
        assert(e != resolutions_.end());

        depth_sensor_->tryConfigureVideoMode(e->second);
      }

      if((level & 1) != 0)
      {
        if(cfg.depth_registration)
        {
          if(device_.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
          {
            device_.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
          }
          else
          {
            cfg.depth_registration = false;
          }
        }
        else
        {
          device_.setImageRegistrationMode(IMAGE_REGISTRATION_OFF);
        }
      }

      if((level & 64) != 0)
      {
        depth_sensor_->stream().setMirroringEnabled(cfg.mirror);
      }

      depth_sensor_->endConfigure();
    }


    if(ir_sensor_->beginConfigure())
    {
      if((level & 32) != 0)
      {
        ResolutionMap::iterator e = resolutions_.find(cfg.ir_resolution);
        assert(e != resolutions_.end());

        ir_sensor_->tryConfigureVideoMode(e->second);
      }

      if((level & 64) != 0)
      {
        ir_sensor_->stream().setMirroringEnabled(cfg.mirror);
      }

      ir_sensor_->endConfigure();
    }

    device_.setDepthColorSyncEnabled(true);
  }
private:
  image_transport::ImageTransport it_;
  boost::shared_ptr<SensorStreamManagerBase> rgb_sensor_, depth_sensor_, ir_sensor_;
  dynamic_reconfigure::Server<CameraConfig> reconfigure_server_;

  typedef std::map<int, VideoMode> ResolutionMap;

  ResolutionMap resolutions_;

  Device& device_;
};


} /* namespace internal */

Camera::Camera(ros::NodeHandle& nh, ros::NodeHandle& nh_private, openni::Device& device) :
    impl_(new internal::CameraImpl(nh, nh_private, device))
{
}

Camera::~Camera()
{
  delete impl_;
}

} /* namespace openni2_camera */
