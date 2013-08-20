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

bool CameraFactory::create(ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& device_id_default)
{
  bool success = false;

  openni::Array<openni::DeviceInfo> devices;
  openni::OpenNI::enumerateDevices(&devices);

  openni::DeviceInfo used_device;
  std::string device_id;

  int device_count = devices.getSize();

  if(device_count > 0)
  {
    ROS_INFO("openni2 detected %d cameras.", device_count);
    //select the desired device via the device_id parameter of the .launch file
    if(!nh_private.getParam("device_id", device_id)) {
      ROS_WARN("parameter ~device_id is not set! Using default value %s.", device_id_default.c_str());
      device_id = device_id_default;
    }
    else
    {
      ROS_INFO("openni2_camera using parameter device_id = %s", device_id.c_str());
    }

    //device_id is given in usb bus@address format
    if (device_id.find ('@') != std::string::npos)
    {
      size_t   at_pos  = device_id.find ('@');
      unsigned bus     = atoi(device_id.substr(0, at_pos).c_str());
      unsigned address = atoi(device_id.substr(at_pos+1, device_id.length()-at_pos-1).c_str());
      ROS_INFO ("Using camera device at bus@address = %d@%d", bus, address);

      //TODO: implement function for getting device by address..
      //used_device = device_by_address(bus, address);
    }
    //device is given in #index format
    else if (device_id[0] == '#')
    {
      int dev_index = atoi(device_id.c_str() + 1);

      if(dev_index > device_count) {
        ROS_ERROR("You selected device #%d, but only %d are present.", dev_index, device_count);
        return false;
      }
      ROS_INFO ("Using camera device at index = #%d", dev_index);
      used_device = devices[dev_index - 1];
      success = true;
    }
    //device should be selected by its serial number
    else
    {
      ROS_INFO("Using device with serial number '%s'", device_id.c_str());

      used_device = devices[0];
      //TODO: implement function for getting device by address..
      //used_device = device_by_serial(device_id);
    }

    if(success) {
      camera_ = new openni2_camera::Camera(nh, nh_private, used_device);
    }
  }
  else {
    ROS_ERROR("OpenNI2 found no devices!");
  }

  return success;
}

} /* namespace openni2_camera */
