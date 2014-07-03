/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <pcl/simulation/shape_generator_base.h>

pcl::simulation::GeometricShapeBase::GeometricShapeBase ()
{  
  gen_.seed (0);
  dist01_ = boost::random::uniform_real_distribution<float> (0, 1);
  effective_transform_.setIdentity ();
  centroid_ = Eigen::Vector3f (0, 0, 0);
}

Eigen::Vector3f
pcl::simulation::GeometricShapeBase::center (const Eigen::Vector3f &new_centroid)
{
  Eigen::Vector3f translation;
  translation = new_centroid - centroid_;
  translate (translation);
  return (translation);
}

void
pcl::simulation::GeometricShapeBase::translate (const Eigen::Vector3f &translation)
{
  EigenTransformT T;
  T = Eigen::Translation<float, 3> (translation[0], translation[1], translation[2]);
  pcl::transformPoint (centroid_, centroid_, T);
  effective_transform_ = T * effective_transform_;
}

void
pcl::simulation::GeometricShapeBase::rotate (pcl::simulation::GeometricShapeBase::Axis axis,
                                             float degree,
                                             bool in_place)
{
  switch (axis)
  {
    case XAxis:
      rotate (Eigen::Vector3f (1, 0, 0), degree, in_place);
      break;
    case YAxis:
      rotate (Eigen::Vector3f (0, 1, 0), degree, in_place);
      break;
    case ZAxis:
      rotate (Eigen::Vector3f (0, 0, 1), degree, in_place);
      break;
  }
}

void
pcl::simulation::GeometricShapeBase::rotate (const Eigen::Vector3f &axis,
                                             float degree,
                                             bool in_place)
{
  EigenTransformT trans;
  if (in_place)
  {
    trans = Eigen::Translation<float, 3> (centroid_[0], centroid_[1], centroid_[2]) *
            Eigen::AngleAxis<float> (degree * M_PI / 180.f, axis.normalized ()) *
            Eigen::Translation<float, 3> (-centroid_[0], -centroid_[1], -centroid_[2]);
  }
  else
  {
    trans = Eigen::AngleAxis<float> (degree * M_PI / 180.f, axis.normalized ());
  }
  effective_transform_ = trans * effective_transform_;
  pcl::transformPoint (centroid_, centroid_, trans);
}

void
pcl::simulation::GeometricShapeBase::deletePointsFromOtherCloud (PointCloudT::Ptr other_cloud_arg,
                                                                 bool delete_inliers) const
{
  // Do a inverse transformation of the given cloud. This way I can check for inliers/outliers by solving the known equations for the geometric shape.
  PointCloudT inverse_transformed_cloud;
  pcl::transformPointCloudWithNormals (*other_cloud_arg, inverse_transformed_cloud, effective_transform_.inverse ());
  PointCloudT::iterator itrOrig = other_cloud_arg->begin ();
  PointCloudT::iterator itrTrans = inverse_transformed_cloud.begin ();
  while (itrTrans != inverse_transformed_cloud.end ())
  {
    if (isInside (*itrTrans) == delete_inliers)
    {
      itrOrig = other_cloud_arg->erase (itrOrig);
      ++itrTrans;
    }
    else
    {
      ++itrOrig;
      ++itrTrans;
    }
  }
}
