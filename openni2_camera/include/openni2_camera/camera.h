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
