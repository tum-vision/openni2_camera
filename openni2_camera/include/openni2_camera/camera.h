
#ifndef CAMERA_H_
#define CAMERA_H_

#include <ros/ros.h>

#include <OpenNI2/OpenNI.h>

namespace openni2_camera
{

namespace internal
{
  class CameraImpl;
}

class Camera
{
public:
  Camera(ros::NodeHandle& nh, ros::NodeHandle& nh_private, openni::Device& device);
  virtual ~Camera();
private:
  internal::CameraImpl* impl_;
};

} /* namespace openni2_camera */
#endif /* CAMERA_H_ */
