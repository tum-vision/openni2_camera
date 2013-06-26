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

#include <openni2_camera/camera_factory.h>

namespace openni2_camera
{

CameraFactory::CameraFactory() :
  camera_(0)
{
  initialized_ = (openni::OpenNI::initialize() == openni::STATUS_OK);

  ROS_ERROR_COND(!initialized_, "OpenNI2 initialization failed!");
}

CameraFactory::~CameraFactory()
{
  if(camera_ != 0) delete camera_;

  openni::OpenNI::shutdown();
}

bool CameraFactory::create(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& device_id)
{
  bool success = false;

  openni::Array<openni::DeviceInfo> devices;
  openni::OpenNI::enumerateDevices(&devices);

  if(devices.getSize() > 0)
  {
    camera_ = new openni2_camera::Camera(nh, nh_private, devices[0]);
    success = true;
  }

  return success;
}

} /* namespace openni2_camera */

