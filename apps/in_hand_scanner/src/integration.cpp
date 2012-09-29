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

bool
pcl::ihs::Integration::reconstructMesh (const CloudProcessedConstPtr& cloud,
                                        const MeshPtr&                mesh) const
{
  if (!cloud->isOrganized ())
  {
    std::cerr << "ERROR in integration.cpp: Data cloud is not organized\n";
    return (false);
  }
  const int w = static_cast <int> (cloud->width);
  const int h = static_cast <int> (cloud->height);
  const int s = static_cast <int> (cloud->size ());

  mesh->reserveVertexes (s);
  mesh->reserveFaces (2 * (w-1) * (h-1));

  // Store which vertex is set at which position (initialized with invalid indexes)
  VertexIndexes vertex_indexes (s, VertexIndex ());

  // Point iterators
  // 2 - 1
  // | / |
  // 3 - 0
  CloudProcessed::const_iterator it_0 = cloud->begin () + w + 1;
  CloudProcessed::const_iterator it_1 = cloud->begin ()     + 1;
  CloudProcessed::const_iterator it_2 = cloud->begin ()        ;
  CloudProcessed::const_iterator it_3 = cloud->begin () + w    ;

  VertexIndexes::iterator it_vi_0 = vertex_indexes.begin () + w + 1;
  VertexIndexes::iterator it_vi_1 = vertex_indexes.begin ()     + 1;
  VertexIndexes::iterator it_vi_2 = vertex_indexes.begin ()        ;
  VertexIndexes::iterator it_vi_3 = vertex_indexes.begin () + w    ;

  for (int r=1; r<h; ++r)
  {
    for (int c=1; c<w; ++c)
    {
      if (pcl::isFinite (*it_1) && pcl::isFinite (*it_3))
      {
        // Note: weight = abs (dot (normal, [0; 0; 1])) = -nz
        if (!it_vi_1->isValid ())
        {
          *it_vi_1 = mesh->addVertex (PointModel (it_1->x, it_1->y, it_1->z, it_1->normal_x, it_1->normal_y, it_1->normal_z, it_1->r, it_1->g, it_1->b, -it_1->normal_z));
        }
        if (!it_vi_3->isValid ())
        {
          *it_vi_3 = mesh->addVertex (PointModel (it_3->x, it_3->y, it_3->z, it_3->normal_x, it_3->normal_y, it_3->normal_z, it_3->r, it_3->g, it_3->b, -it_3->normal_z));
        }

        // 1-2-3 is valid
        if (pcl::isFinite (*it_2))
        {
          if (!it_vi_2->isValid ())
          {
            *it_vi_2 = mesh->addVertex (PointModel (it_2->x, it_2->y, it_2->z, it_2->normal_x, it_2->normal_y, it_2->normal_z, it_2->r, it_2->g, it_2->b, -it_2->normal_z));
          }

          mesh->addFace (*it_vi_1, *it_vi_2, *it_vi_3);
        }

        // 0-1-3 is valid
        if (pcl::isFinite (*it_0))
        {
          if (!it_vi_0->isValid ())
          {
            *it_vi_0 = mesh->addVertex (PointModel (it_0->x, it_0->y, it_0->z, it_0->normal_x, it_0->normal_y, it_0->normal_z, it_0->r, it_0->g, it_0->b, -it_0->normal_z));
          }

          mesh->addFace (*it_vi_0, *it_vi_1, *it_vi_3);
        }
      }

      ++it_0;    ++it_1;    ++it_2;    ++it_3;
      ++it_vi_0; ++it_vi_1; ++it_vi_2; ++it_vi_3;
    }

    ++it_0;    ++it_1;    ++it_2;    ++it_3;
    ++it_vi_0; ++it_vi_1; ++it_vi_2; ++it_vi_3;
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::Integration::merge (const CloudProcessedConstPtr& /*cloud_data*/,
                              const MeshPtr&                /*mesh_model*/,
                              const Transformation&         /*T*/) const
{
//  // TODO: The size is hard coded for the kinect
//  const unsigned int w = 640;
//  const unsigned int h = 480;

//  if (!mesh_model->sizeVertexes ())
//  {
//    std::cerr << "ERROR in integration.cpp: Model cloud is empty\n";
//    return (false);
//  }

//  if (cloud_data->width != w || cloud_data->height != h)
//  {
//    std::cerr << "ERROR in integration.cpp: Wrong size of the data cloud!\n";
//    exit (EXIT_FAILURE);
//  }

//  kd_tree_->setInputCloud (mesh_model);

//  mesh_model->reserve (mesh_model->size () + cloud_data->size ());

//  CloudProcessed::const_iterator it_d = cloud_data->begin ();

//  for (; it_d!=cloud_data->end (); ++it_d)
//  {
//    if (!pcl::isFinite (*it_d)) continue;

//    // Transform the data points into model coordinates (this does not change the connectivity information)
//    PointModel pt_d_trans;
//    pt_d_trans.getVector4fMap ()       = T * it_d->getVector4fMap ();
//    pt_d_trans.getNormalVector4fMap () = T * it_d->getNormalVector4fMap ();
//    pt_d_trans.getRGBVector4i ()       = it_d->getRGBVector4i ();
//    pt_d_trans.age                     = 0;
//    pt_d_trans.weight                  = -it_d->normal_z; // abs (dot (normal, [0; 0; 1]))

//    // NN search
//    std::vector <int>   index (1);
//    std::vector <float> squared_distance (1);

//    if (!kd_tree_->nearestKSearch (pt_d_trans, 1, index, squared_distance))
//    {
//      std::cerr << "ERROR in integration.cpp: nearestKSearch failed!\n";
//      return (false);
//    }

//    // Average out corresponding points
//    if (squared_distance[0] < squared_distance_max_)
//    {
//      PointModel& pt_m = mesh_model->operator [] (index[0]); // Non-const reference!
//      if (pt_m.getNormalVector4fMap ().dot (pt_d_trans.getNormalVector4fMap ()) > dot_normal_min_)
//      {
//        const float W   = pt_m.weight;         // Old accumulated weight
//        const float w   = pt_d_trans.weight;   // Weight of new point
//        const float WW  = pt_m.weight = W + w; // New accumulated weight

//        const float r_m = static_cast <float> (pt_m.r);
//        const float g_m = static_cast <float> (pt_m.g);
//        const float b_m = static_cast <float> (pt_m.b);

//        const float r_d = static_cast <float> (it_d->r);
//        const float g_d = static_cast <float> (it_d->g);
//        const float b_d = static_cast <float> (it_d->b);

//        pt_m.getVector4fMap ()       = ( W*pt_m.getVector4fMap ()       + w*pt_d_trans.getVector4fMap ())       / WW;
//        pt_m.getNormalVector4fMap () = ((W*pt_m.getNormalVector4fMap () + w*pt_d_trans.getNormalVector4fMap ()) / WW).normalized ();
//        pt_m.r                       = trimRGB ((W*r_m + w*r_d) / WW);
//        pt_m.g                       = trimRGB ((W*g_m + w*g_d) / WW);
//        pt_m.b                       = trimRGB ((W*b_m + w*b_d) / WW);

//        // Point has been observed again -> give it some extra time to live
//        pt_m.age = 0;

//      }
//    }
//    // Add new points
//    else
//    {
//      mesh_model->push_back (pt_d_trans);
//    }
//  }

//  // TODO: aging (inside previous loop?)

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

uint8_t
pcl::ihs::Integration::trimRGB (const float val) const
{
  return (static_cast <uint8_t> (val > 255.f ? 255 : val));
}

////////////////////////////////////////////////////////////////////////////////
