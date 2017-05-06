/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Xuedong Wang
 *
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
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
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
 */

/*
 * Brief interface for bearing-angle(BA) image
 * The class is used as an interface to generate bearing-angle(BA) image,
 * and do some necessary transformation.
 *
 *  Created on: 2012.07.07
 *      Author: Qinghua Li
 */

#ifndef BEARING_ANGLE_H_
#define BEARING_ANGLE_H_

#include <cmath>
#include <vector>
#include <QImage>
#include <opencv/cv.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

const double PI = 3.14159265358979323846;

class BearingAngle
{
  public:
    BearingAngle ();
    ~BearingAngle ();

  public:
    double
    getAngle (pcl::PointXYZ point1, pcl::PointXYZ point2);

    double
    getGray (double theta);

    IplImage*
    generateBAImage (std::vector< std::vector<pcl::PointXYZ> > &points, int width, int height);

    IplImage*
    getChannelsImage (IplImage* ipl_image);

    QImage
    cvIplImage2QImage (IplImage* ipl_image);

    IplImage* BA_image;
    IplImage* channels_image;
};

#endif // BEARING_ANGLE_H_