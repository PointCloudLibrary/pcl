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
#include <pcl/kdtree/impl/kdtree_flann.hpp> // NOTE: PointModel is not registered to the default point types

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::Integration::Integration ()
  : kd_tree_              (new pcl::KdTreeFLANN <PointModel> ()),
    squared_distance_max_ (4e-2f),
    dot_normal_min_       (.6f)
{
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::setInitialModel (const CloudModelPtr&          cloud_model,
                                        const CloudProcessedConstPtr& cloud_data) const
{
  cloud_model->reserve (cloud_data->size ());
  for (CloudProcessed::const_iterator it=cloud_data->begin (); it!=cloud_data->end (); ++it)
  {
    if (pcl::isFinite (*it))
    {
      cloud_model->push_back (PointModel (it->x, it->y, it->z,
                                          it->normal_x, it->normal_y, it->normal_z,
                                          it->r, it->g, it->b,
                                          -it->normal_z // abs (dot (normal, [0; 0; 1]))
                                          ));
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::Integration::merge (const CloudModelPtr&          cloud_model,
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

  cloud_model->reserve (cloud_model->size () + cloud_data->size ());

  CloudProcessed::const_iterator it_d = cloud_data->begin ();

  for (; it_d!=cloud_data->end (); ++it_d)
  {
    if (!pcl::isFinite (*it_d)) continue;

    // Transform the data points into model coordinates (this does not change the connectivity information)
    PointModel pt_d_trans;
    pt_d_trans.getVector4fMap ()       = T * it_d->getVector4fMap ();
    pt_d_trans.getNormalVector4fMap () = T * it_d->getNormalVector4fMap ();
    pt_d_trans.getRGBVector4i ()       = it_d->getRGBVector4i ();
    pt_d_trans.age                     = 0;
    pt_d_trans.weight                  = -it_d->normal_z; // abs (dot (normal, [0; 0; 1]))

    // NN search
    std::vector <int>   index (1);
    std::vector <float> squared_distance (1);

    if (!kd_tree_->nearestKSearch (pt_d_trans, 1, index, squared_distance))
    {
      std::cerr << "ERROR in integration.cpp: nearestKSearch failed!\n";
      return (false);
    }

    // Average out corresponding points
    if (squared_distance[0] < squared_distance_max_)
    {
      PointModel& pt_m = cloud_model->operator [] (index[0]); // Non-const reference!
      if (pt_m.getNormalVector4fMap ().dot (pt_d_trans.getNormalVector4fMap ()) > dot_normal_min_)
      {
        const float W   = pt_m.weight;         // Old accumulated weight
        const float w   = pt_d_trans.weight;   // Weight of new point
        const float WW  = pt_m.weight = W + w; // New accumulated weight

        const float r_m = static_cast <float> (pt_m.r);
        const float g_m = static_cast <float> (pt_m.g);
        const float b_m = static_cast <float> (pt_m.b);

        const float r_d = static_cast <float> (it_d->r);
        const float g_d = static_cast <float> (it_d->g);
        const float b_d = static_cast <float> (it_d->b);

        pt_m.getVector4fMap ()       = ( W*pt_m.getVector4fMap ()       + w*pt_d_trans.getVector4fMap ())       / WW;
        pt_m.getNormalVector4fMap () = ((W*pt_m.getNormalVector4fMap () + w*pt_d_trans.getNormalVector4fMap ()) / WW).normalized ();
        pt_m.r                       = trimRGB ((W*r_m + w*r_d) / WW);
        pt_m.g                       = trimRGB ((W*g_m + w*g_d) / WW);
        pt_m.b                       = trimRGB ((W*b_m + w*b_d) / WW);

        // Point has been observed again -> give it some extra time to live
        pt_m.age = 0;

      }
    }
    // Add new points
    else
    {
      cloud_model->push_back (pt_d_trans);
    }
  }

  // TODO: aging (inside previous loop?)

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

uint8_t
pcl::ihs::Integration::trimRGB (const float val) const
{
  return (static_cast <uint8_t> (val > 255.f ? 255 : val));
}

////////////////////////////////////////////////////////////////////////////////
