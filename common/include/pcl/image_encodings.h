
/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef SENSOR_MSGS_IMAGE_ENCODINGS_H
#define SENSOR_MSGS_IMAGE_ENCODINGS_H

#include <string>

namespace sensor_msgs
{
  namespace image_encodings
  {
    extern const std::string RGB8;
    extern const std::string RGBA8;
    extern const std::string BGR8;
    extern const std::string BGRA8;
    extern const std::string MONO8;
    extern const std::string MONO16;

    // OpenCV CvMat types
    extern const std::string TYPE_8UC1;
    extern const std::string TYPE_8UC2;
    extern const std::string TYPE_8UC3;
    extern const std::string TYPE_8UC4;
    extern const std::string TYPE_8SC1;
    extern const std::string TYPE_8SC2;
    extern const std::string TYPE_8SC3;
    extern const std::string TYPE_8SC4;
    extern const std::string TYPE_16UC1;
    extern const std::string TYPE_16UC2;
    extern const std::string TYPE_16UC3;
    extern const std::string TYPE_16UC4;
    extern const std::string TYPE_16SC1;
    extern const std::string TYPE_16SC2;
    extern const std::string TYPE_16SC3;
    extern const std::string TYPE_16SC4;
    extern const std::string TYPE_32SC1;
    extern const std::string TYPE_32SC2;
    extern const std::string TYPE_32SC3;
    extern const std::string TYPE_32SC4;
    extern const std::string TYPE_32FC1;
    extern const std::string TYPE_32FC2;
    extern const std::string TYPE_32FC3;
    extern const std::string TYPE_32FC4;
    extern const std::string TYPE_64FC1;
    extern const std::string TYPE_64FC2;
    extern const std::string TYPE_64FC3;
    extern const std::string TYPE_64FC4;

    // Bayer encodings
    extern const std::string BAYER_RGGB8;
    extern const std::string BAYER_BGGR8;
    extern const std::string BAYER_GBRG8;
    extern const std::string BAYER_GRBG8;
  }
}

#endif
