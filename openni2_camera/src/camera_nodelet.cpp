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

#include <openni2_camera/camera_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(openni2_camera, camera_nodelet, openni2_camera::CameraNodelet, nodelet::Nodelet)

namespace openni2_camera
{

CameraNodelet::CameraNodelet()
{
}

CameraNodelet::~CameraNodelet()
{
}

void CameraNodelet::onInit()
{
  ROS_ERROR_COND(!camera_factory_.create(getNodeHandle(), getPrivateNodeHandle(), "#1"), "Failed to open camera!");
}

} /* namespace openni2_camera */
