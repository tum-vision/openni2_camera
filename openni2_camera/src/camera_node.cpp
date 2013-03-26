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
#include <ros/ros.h>
#include <openni2_camera/camera.h>

#include <OpenNI2/OpenNI.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_node");

  ros::NodeHandle nh("camera");
  ros::NodeHandle nh_private("~");

  openni::Status rc = openni::OpenNI::initialize();

  if(rc == openni::STATUS_OK)
  {

    openni::Array<openni::DeviceInfo> devices;
    openni::OpenNI::enumerateDevices(&devices);

    if(devices.getSize() != 0)
    {
      openni::Device device;
      device.open(devices[0].getUri());

      openni2_camera::Camera cam(nh, nh_private, device);

      ros::spin();
    }
    else
    {
      ROS_WARN("No devices connected!");
    }
  }
  else
  {
    ROS_ERROR("Initialization error!");
  }

  openni::OpenNI::shutdown();

  return 0;
}
