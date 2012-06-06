/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */

#ifndef PCL_RECOGNITION_MODEL_LIBRARY_H_
#define PCL_RECOGNITION_MODEL_LIBRARY_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

namespace pcl
{
  namespace recognition
  {
    class ModelLibrary
    {
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudIn; // For now, only pcl::PointXYZ makes sense
    typedef pcl::PointCloud<pcl::Normal> PointCloudN;

    public:
      class Entry
      {
      public:
        Entry(){}
        virtual ~Entry(){}
      };

    public:
      /** \brief This class is used by 'ObjRecRANSAC' to maintain the object models to be recognized. Normally, you do not need to use
        * this class directly. */
      ModelLibrary(double pair_width): pair_width_(pair_width), pair_width_eps_(0.1*pair_width){}
      virtual ~ModelLibrary(){}

      bool
      addModel(const PointCloudIn& model, const PointCloudN& normals, const std::string& object_name);

    protected:
      std::map<std::string,Entry*> model_entries_;
      double pair_width_, pair_width_eps_;
    };
  } // namespace recognition
} // namespace pcl

#endif // PCL_RECOGNITION_MODEL_LIBRARY_H_
