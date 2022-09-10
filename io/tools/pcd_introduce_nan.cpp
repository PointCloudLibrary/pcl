/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 */

#include <pcl/io/pcd_io.h>
#include <boost/lexical_cast.hpp> // for lexical_cast

/** @brief PCL point object */
using PointT = pcl::PointXYZRGBA;

/** @brief PCL Point cloud object */
using PointCloudT = pcl::PointCloud<PointT>;

int
main (int argc,
      char** argv)
{
  if (argc != 3 && argc != 4)
  {
    PCL_ERROR ("Usage: %s cloud_in.pcd cloud_out_ascii.pcd percentage_of_NaN \n", argv[0]);
    return (-1);
  }

  int percentage_of_NaN = 20;
  if (argc == 4)
    percentage_of_NaN = boost::lexical_cast<int>(argv[3]);

  PCL_INFO ("Replacing approximately %d%% of the cloud with NaN values (already existing NaN values are conserved)\n", percentage_of_NaN);
  PointCloudT::Ptr cloud (new PointCloudT);
  if (pcl::io::loadPCDFile (argv[1], *cloud) != 0)
    return (-1);

  for (auto &point : *cloud)
  {
    int random = 1 + (rand () % (int) (100));
    int random_xyz = 1 + (rand () % (int) (3 - 1 + 1));

    if (random < percentage_of_NaN)
    {
      if (random_xyz == 1)
        point.x = std::numeric_limits<double>::quiet_NaN ();
      else if (random_xyz == 2)
        point.y = std::numeric_limits<double>::quiet_NaN ();
      else
        point.z = std::numeric_limits<double>::quiet_NaN ();
    }
  }

  pcl::io::savePCDFile (argv[2], *cloud);
  return (0);
}

