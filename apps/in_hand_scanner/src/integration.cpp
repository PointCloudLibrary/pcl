/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * $Id$
 *
 */

#include <pcl/apps/in_hand_scanner/integration.h>

#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE
#include <iostream>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::Integration::Integration ()
  : kd_tree_              (new pcl::KdTreeFLANN <PointProcessed> ()),
    squared_distance_max_ ()
{
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::Integration::merge (const CloudProcessedPtr&      cloud_model,
                              const CloudProcessedConstPtr& cloud_data,
                              const Transformation&         T) const
{
  // TODO: The size is hard coded for the kinect
  const unsigned int w = 640;
  const unsigned int h = 480;

  if (!cloud_model->size ())
  {
    std::cerr << "ERROR in integration.cpp: Model cloud is empty\n";
    return (false);
  }

  if (cloud_data->width != w || cloud_data->height != h)
  {
    std::cerr << "ERROR in integration.cpp: Wrong size of the data cloud!\n";
    exit (EXIT_FAILURE);
  }

  kd_tree_->setInputCloud (cloud_model);

  CloudProcessed::const_iterator it_d   = cloud_data->begin ();

  for (; it_d!=cloud_data->end (); ++it_d)
  {
    // Transform the data points into model coordinates (this does not change the connectivity information compared to transforming the model points into data coordinates)
    PointProcessed pt_d_trans = *it_d;
    pt_d_trans.getVector4fMap ()       = T * pt_d_trans.getVector4fMap ();
    pt_d_trans.getNormalVector4fMap () = T * pt_d_trans.getNormalVector4fMap ();

    std::vector <int>   ind (1);
    std::vector <float> squared_distance (1);

    if (!kd_tree_->nearestKSearch (pt_d_trans, 1, ind, squared_distance))
    {
      std::cerr << "ERROR in integration.cpp: nearestKSearch failed!\n";
      return (false);
    }

    if (squared_distance[0] < squared_distance_max_)
    {
      // Average out corresponding points

    }
    else
    {
      // Add new points
    }
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////
