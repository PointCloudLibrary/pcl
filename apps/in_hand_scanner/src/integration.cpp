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
#include <limits>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp> // NOTE: PointModel is not registered to the default point types
#include <pcl/apps/in_hand_scanner/impl/common_functions.hpp>

////////////////////////////////////////////////////////////////////////////////

pcl::ihs::Integration::Integration ()
  : kd_tree_              (new pcl::KdTreeFLANN <PointXYZ> ()),
    squared_distance_max_ (4e-2f),
    dot_normal_min_       (.6f),
    weight_min_           (.3f),
    age_max_              (30),
    visconf_min_          (.12)
{
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::Integration::reconstructMesh (const CloudProcessedConstPtr& cloud_data,
                                        const MeshPtr&                mesh_model) const
{
  if (!cloud_data->isOrganized ())
  {
    std::cerr << "ERROR in integration.cpp: Cloud is not organized\n";
    return (false);
  }
  const uint32_t width  = cloud_data->width;
  const uint32_t height = cloud_data->height;
  const size_t   size   = cloud_data->size ();

  mesh_model->clear ();
  mesh_model->reserveVertexes (size);
  mesh_model->reserveFaces (2 * (width-1) * (height-1));

  // Store which vertex is set at which position (initialized with invalid indexes)
  VertexIndexes vertex_indexes (size, VertexIndex ());

  // Convert to the model cloud type. This is actually not needed but avoids code duplication (see merge). And reconstructMesh is called only the first reconstruction step anyway.
  // NOTE: The default constructor of PointModel has to initialize with NaNs!
  CloudModelPtr cloud_model (new CloudModel ());
  cloud_model->resize (size);

  // Set the model points not reached by the main loop
  for (uint32_t c=0; c<width; ++c)
  {
    const PointProcessed& pt_d = cloud_data->operator [] (c);
    const float weight = -pt_d.normal_z; // weight = -dot (normal, [0; 0; 1])

    if (pcl::isFinite (pt_d) && weight >= weight_min_)
    {
      cloud_model->operator [] (c) = PointModel (pt_d, weight);
    }
  }
  for (uint32_t r=1; r<height; ++r)
  {
    for (uint32_t c=0; c<2; ++c)
    {
      const PointProcessed& pt_d = cloud_data->operator [] (r*width + c);
      const float weight = -pt_d.normal_z; // weight = -dot (normal, [0; 0; 1])

      if (pcl::isFinite (pt_d) && weight >= weight_min_)
      {
        cloud_model->operator [] (r*width + c) = PointModel (pt_d, weight);
      }
    }
  }

  // 4   2 - 1  //
  //     |   |  //
  // *   3 - 0  //
  //            //
  // 4 - 2   1  //
  //   \   \    //
  // *   3 - 0  //
  CloudProcessed::const_iterator it_d_0 = cloud_data->begin ()  + width + 2;
  CloudModel::iterator           it_m_0 = cloud_model->begin () + width + 2;
  CloudModel::const_iterator     it_m_1 = cloud_model->begin ()         + 2;
  CloudModel::const_iterator     it_m_2 = cloud_model->begin ()         + 1;
  CloudModel::const_iterator     it_m_3 = cloud_model->begin () + width + 1;
  CloudModel::const_iterator     it_m_4 = cloud_model->begin ()            ;

  VertexIndexes::iterator it_vi_0 = vertex_indexes.begin () + width + 2;
  VertexIndexes::iterator it_vi_1 = vertex_indexes.begin ()         + 2;
  VertexIndexes::iterator it_vi_2 = vertex_indexes.begin ()         + 1;
  VertexIndexes::iterator it_vi_3 = vertex_indexes.begin () + width + 1;
  VertexIndexes::iterator it_vi_4 = vertex_indexes.begin ()            ;

  for (uint32_t r=1; r<height; ++r)
  {
    for (uint32_t c=2; c<width; ++c)
    {
      const float weight = -it_d_0->normal_z; // weight = -dot (normal, [0; 0; 1])
      if (pcl::isFinite(*it_d_0) && weight >= weight_min_)
      {
        *it_m_0 = PointModel (*it_d_0, weight);
      }

      this->addToMesh (it_m_0,  it_m_1,  it_m_2,  it_m_3,
                       it_vi_0, it_vi_1, it_vi_2, it_vi_3,
                       mesh_model);
      this->addToMesh (it_m_0,  it_m_2,  it_m_4,  it_m_3,
                       it_vi_0, it_vi_2, it_vi_4, it_vi_3,
                       mesh_model);

      ++it_d_0;
      ++it_m_0;  ++it_m_1;  ++it_m_2;  ++it_m_3;  ++it_m_4;
      ++it_vi_0; ++it_vi_1; ++it_vi_2; ++it_vi_3; ++it_vi_4;
    } // for (uint32_t c=2; c<width; ++c)

    it_d_0  += 2;
    it_m_0  += 2; it_m_1  += 2; it_m_2  += 2; it_m_3  += 2, it_m_4  += 2;
    it_vi_0 += 2; it_vi_1 += 2; it_vi_2 += 2; it_vi_3 += 2, it_vi_4 += 2;
  } // for (uint32_t r=1; r<height; ++r)

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::Integration::merge (const CloudProcessedConstPtr& cloud_data,
                              const MeshPtr&                mesh_model,
                              const Transformation&         T) const
{
  if (!cloud_data->isOrganized ())
  {
    std::cerr << "ERROR in integration.cpp: Data cloud is not organized\n";
    return (false);
  }
  const uint32_t width  = cloud_data->width;
  const uint32_t height = cloud_data->height;
  const size_t   size   = cloud_data->size ();

  if (!mesh_model->sizeVertexes ())
  {
    std::cerr << "ERROR in integration.cpp: Model mesh is empty\n";
    return (false);
  }

  // Nearest neighbor search
  // TODO: remove this unnecessary copy
  CloudXYZPtr xyz_model (new CloudXYZ ());
  xyz_model->reserve (mesh_model->sizeVertexes ());
  for (VertexConstIterator it=mesh_model->beginVertexes (); it!=mesh_model->endVertexes (); ++it)
  {
    xyz_model->push_back (PointXYZ (it->x, it->y, it->z));
  }
  kd_tree_->setInputCloud (xyz_model);
  std::vector <int>   index (1);
  std::vector <float> squared_distance (1);

  mesh_model->reserveVertexes (mesh_model->sizeVertexes () + cloud_data->size ());
  mesh_model->reserveFaces (mesh_model->sizeFaces () + 2 * (width-1) * (height-1));

  // Data cloud in model coordinates (this does not change the connectivity information) and weights
  CloudModelPtr cloud_data_transformed (new CloudModel ());
  cloud_data_transformed->resize (size);

  // Sensor position in model coordinates
  const Eigen::Vector4f& sensor_eye = T * Eigen::Vector4f (0.f, 0.f, 0.f, 1.f);

  // Store which vertex is set at which position (initialized with invalid indexes)
  VertexIndexes vertex_indexes (size, VertexIndex ());

  // Set the transformed points not reached by the main loop
  for (uint32_t c=0; c<width; ++c)
  {
    const PointProcessed& pt_d = cloud_data->operator [] (c);
    const float weight = -pt_d.normal_z; // weight = -dot (normal, [0; 0; 1])

    if (pcl::isFinite (pt_d) && weight >= weight_min_)
    {
      PointModel& pt_d_t = cloud_data_transformed->operator [] (c);
      pt_d_t = PointModel (pt_d, weight);
      pt_d_t.getVector4fMap ()       = T * pt_d_t.getVector4fMap ();
      pt_d_t.getNormalVector4fMap () = T * pt_d_t.getNormalVector4fMap ();
    }
  }
  for (uint32_t r=1; r<height; ++r)
  {
    for (uint32_t c=0; c<2; ++c)
    {
      const PointProcessed& pt_d = cloud_data->operator [] (r*width + c);
      const float weight = -pt_d.normal_z; // weight = -dot (normal, [0; 0; 1])

      if (pcl::isFinite (pt_d) && weight >= weight_min_)
      {
        PointModel& pt_d_t = cloud_data_transformed->operator [] (r*width + c);
        pt_d_t = PointModel (pt_d, weight);
        pt_d_t.getVector4fMap ()       = T * pt_d_t.getVector4fMap ();
        pt_d_t.getNormalVector4fMap () = T * pt_d_t.getNormalVector4fMap ();
      }
    }
  }

  // 4   2 - 1  //
  //     |   |  //
  // *   3 - 0  //
  //            //
  // 4 - 2   1  //
  //   \   \    //
  // *   3 - 0  //
  CloudProcessed::const_iterator it_d_0   = cloud_data->begin ()             + width + 2;
  CloudModel::iterator           it_d_t_0 = cloud_data_transformed->begin () + width + 2;
  CloudModel::const_iterator     it_d_t_1 = cloud_data_transformed->begin ()         + 2;
  CloudModel::const_iterator     it_d_t_2 = cloud_data_transformed->begin ()         + 1;
  CloudModel::const_iterator     it_d_t_3 = cloud_data_transformed->begin () + width + 1;
  CloudModel::const_iterator     it_d_t_4 = cloud_data_transformed->begin ()            ;

  VertexIndexes::iterator it_vi_0 = vertex_indexes.begin () + width + 2;
  VertexIndexes::iterator it_vi_1 = vertex_indexes.begin ()         + 2;
  VertexIndexes::iterator it_vi_2 = vertex_indexes.begin ()         + 1;
  VertexIndexes::iterator it_vi_3 = vertex_indexes.begin () + width + 1;
  VertexIndexes::iterator it_vi_4 = vertex_indexes.begin ()            ;

  for (uint32_t r=1; r<height; ++r)
  {
    for (uint32_t c=2; c<width; ++c)
    {
      const float weight = -it_d_0->normal_z; // weight = -dot (normal, [0; 0; 1])

      if (pcl::isFinite (*it_d_0) && weight >= weight_min_)
      {
        *it_d_t_0 = PointModel (*it_d_0, weight);
        it_d_t_0->getVector4fMap ()       = T * it_d_t_0->getVector4fMap ();
        it_d_t_0->getNormalVector4fMap () = T * it_d_t_0->getNormalVector4fMap ();

        // NN search
        if (!kd_tree_->nearestKSearchT (*it_d_t_0, 1, index, squared_distance))
        {
          std::cerr << "ERROR in integration.cpp: nearestKSearch failed!\n";
          return (false);
        }

        // Average out corresponding points
        if (squared_distance[0] <= squared_distance_max_)
        {
          Vertex& v_m = mesh_model->getElement (VertexIndex (index[0])); // Non-const reference!

          if (v_m.getNormalVector4fMap ().dot (it_d_t_0->getNormalVector4fMap ()) >= dot_normal_min_)
          {
            *it_vi_0 = VertexIndex (index[0]);

            const float W   = v_m.weight;         // Old accumulated weight
            const float w   = it_d_t_0->weight;   // Weight of new point
            const float WW  = v_m.weight = W + w; // New accumulated weight

            const float r_m = static_cast <float> (v_m.r);
            const float g_m = static_cast <float> (v_m.g);
            const float b_m = static_cast <float> (v_m.b);

            const float r_d = static_cast <float> (it_d_t_0->r);
            const float g_d = static_cast <float> (it_d_t_0->g);
            const float b_d = static_cast <float> (it_d_t_0->b);

            v_m.getVector4fMap ()       = ( W*v_m.getVector4fMap ()       + w*it_d_t_0->getVector4fMap ())       / WW;
            v_m.getNormalVector4fMap () = ((W*v_m.getNormalVector4fMap () + w*it_d_t_0->getNormalVector4fMap ()) / WW).normalized ();
            v_m.r                       = this->trimRGB ((W*r_m + w*r_d) / WW);
            v_m.g                       = this->trimRGB ((W*g_m + w*g_d) / WW);
            v_m.b                       = this->trimRGB ((W*b_m + w*b_d) / WW);

            // Point has been observed again -> give it some extra time to live
            v_m.age = 0;

            // add a direction to the visibility confidence
            v_m.visconf.addDirection (v_m.getNormalVector4fMap (), sensor_eye-v_m.getVector4fMap (), w);

          } // dot normals
        } // squared distance
      } // isfinite && min weight

      // Connect
      // 4   2 - 1  //
      //     |   |  //
      // *   3 - 0  //
      //            //
      // 4 - 2   1  //
      //   \   \    //
      // *   3 - 0  //
      this->addToMesh (it_d_t_0, it_d_t_1, it_d_t_2, it_d_t_3,
                       it_vi_0,  it_vi_1,  it_vi_2,  it_vi_3,
                       mesh_model);
      this->addToMesh (it_d_t_0, it_d_t_2, it_d_t_4, it_d_t_3,
                       it_vi_0,  it_vi_2,  it_vi_4,  it_vi_3,
                       mesh_model);

      ++it_d_0;
      ++it_d_t_0; ++it_d_t_1; ++it_d_t_2; ++it_d_t_3; ++it_d_t_4;
      ++it_vi_0;  ++it_vi_1;  ++it_vi_2;  ++it_vi_3;  ++it_vi_4;
    } // for (uint32_t c=2; c<width; ++c)

    it_d_0   += 2;
    it_d_t_0 += 2; it_d_t_1 += 2; it_d_t_2 += 2; it_d_t_3 += 2, it_d_t_4 += 2;
    it_vi_0  += 2; it_vi_1  += 2; it_vi_2  += 2; it_vi_3  += 2, it_vi_4  += 2;
  } // for (uint32_t r=1; r<height; ++r)

  return (true);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::age (const MeshPtr& mesh, const bool cleanup) const
{
  for (Mesh::VertexIterator it = mesh->beginVertexes (); it!=mesh->endVertexes (); ++it)
  {
    if(it->age < age_max_)
    {
       // Point survives
       ++it->age;
    }
    else if(it->age == age_max_) // Judgement Day
    {
      if(it->visconf.getValue () < visconf_min_)
      {
        // Point dies (no need to transform it)
        mesh->deleteVertex (*it);
      }
      else
      {
        // Point becomes immortal
        it->age = std::numeric_limits <unsigned int>::max ();
      }
    }
  }

  if (cleanup)
  {
    mesh->cleanUp ();
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::setDistanceThreshold (const float squared_distance)
{
  squared_distance_max_ = squared_distance;
}

////////////////////////////////////////////////////////////////////////////////

float
pcl::ihs::Integration::getDistanceThreshold () const
{
  return (squared_distance_max_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::setAngleThreshold (const float dot_product)
{
  dot_normal_min_ = dot_product;
}

////////////////////////////////////////////////////////////////////////////////

float
pcl::ihs::Integration::getAngleThreshold () const
{
  return (dot_normal_min_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::setMinimumWeight (const float weight)
{
  weight_min_ = weight;
}

////////////////////////////////////////////////////////////////////////////////

float
pcl::ihs::Integration::getMinimumWeight () const
{
  return (weight_min_);
}

void
pcl::ihs::Integration::setMaximumAge (const unsigned int age)
{
  age_max_ = age;
}

////////////////////////////////////////////////////////////////////////////////

unsigned int
pcl::ihs::Integration::getMaximumAge () const
{
  return (age_max_);
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::setMinimumVisibilityConfidence (const float visconf)
{
  visconf_min_ = visconf;
}

////////////////////////////////////////////////////////////////////////////////

float
pcl::ihs::Integration::getMinimumVisibilityConfidence () const
{
  return (visconf_min_);
}

////////////////////////////////////////////////////////////////////////////////

uint8_t
pcl::ihs::Integration::trimRGB (const float val) const
{
  return (static_cast <uint8_t> (val > 255.f ? 255 : val));
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::addToMesh (const CloudModel::const_iterator& it_pt_0,
                                  const CloudModel::const_iterator& it_pt_1,
                                  const CloudModel::const_iterator& it_pt_2,
                                  const CloudModel::const_iterator& it_pt_3,
                                  const VertexIndexes::iterator&    it_vi_0,
                                  const VertexIndexes::iterator&    it_vi_1,
                                  const VertexIndexes::iterator&    it_vi_2,
                                  const VertexIndexes::iterator&    it_vi_3,
                                  const MeshPtr&                    mesh) const
{
  // Treated bitwise
  // 2 - 1
  // |   |
  // 3 - 0
  const unsigned char is_finite = static_cast <unsigned char> ((1 * pcl::isFinite (*it_pt_0)) | (2 * pcl::isFinite (*it_pt_1)) | (4 * pcl::isFinite (*it_pt_2)) | (8 * pcl::isFinite (*it_pt_3)));

  switch (is_finite)
  {
    case  7: this->addToMesh (it_pt_0, it_pt_1, it_pt_2, it_vi_0, it_vi_1, it_vi_2, mesh); break; // 0-1-2
    case 11: this->addToMesh (it_pt_0, it_pt_1, it_pt_3, it_vi_0, it_vi_1, it_vi_3, mesh); break; // 0-1-3
    case 13: this->addToMesh (it_pt_0, it_pt_2, it_pt_3, it_vi_0, it_vi_2, it_vi_3, mesh); break; // 0-2-3
    case 14: this->addToMesh (it_pt_1, it_pt_2, it_pt_3, it_vi_1, it_vi_2, it_vi_3, mesh); break; // 1-2-3
    case 15: // 0-1-2-3
    {
      if (!distanceThreshold (*it_pt_0, *it_pt_1, *it_pt_2, *it_pt_3)) break;
      if (!it_vi_0->isValid ()) *it_vi_0 = mesh->addVertex (*it_pt_0);
      if (!it_vi_1->isValid ()) *it_vi_1 = mesh->addVertex (*it_pt_1);
      if (!it_vi_2->isValid ()) *it_vi_2 = mesh->addVertex (*it_pt_2);
      if (!it_vi_3->isValid ()) *it_vi_3 = mesh->addVertex (*it_pt_3);
      mesh->addFace (*it_vi_0, *it_vi_1, *it_vi_2, *it_vi_3);
      break;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void
pcl::ihs::Integration::addToMesh (const CloudModel::const_iterator& it_pt_0,
                                  const CloudModel::const_iterator& it_pt_1,
                                  const CloudModel::const_iterator& it_pt_2,
                                  const VertexIndexes::iterator&    it_vi_0,
                                  const VertexIndexes::iterator&    it_vi_1,
                                  const VertexIndexes::iterator&    it_vi_2,
                                  const MeshPtr&                    mesh) const
{
  if (!distanceThreshold (*it_pt_0, *it_pt_1, *it_pt_2)) return;

  if (!it_vi_0->isValid ()) *it_vi_0 = mesh->addVertex (*it_pt_0);
  if (!it_vi_1->isValid ()) *it_vi_1 = mesh->addVertex (*it_pt_1);
  if (!it_vi_2->isValid ()) *it_vi_2 = mesh->addVertex (*it_pt_2);

  mesh->addFace (*it_vi_0, *it_vi_1, *it_vi_2);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::Integration::distanceThreshold (const PointModel& pt_0,
                                          const PointModel& pt_1,
                                          const PointModel& pt_2) const
{
  if ((pt_0.getVector3fMap () - pt_1.getVector3fMap ()).squaredNorm () > squared_distance_max_) return (false);
  if ((pt_1.getVector3fMap () - pt_2.getVector3fMap ()).squaredNorm () > squared_distance_max_) return (false);
  if ((pt_2.getVector3fMap () - pt_0.getVector3fMap ()).squaredNorm () > squared_distance_max_) return (false);
  return (true);
}

////////////////////////////////////////////////////////////////////////////////

bool
pcl::ihs::Integration::distanceThreshold (const PointModel& pt_0,
                                          const PointModel& pt_1,
                                          const PointModel& pt_2,
                                          const PointModel& pt_3) const
{
  if ((pt_0.getVector3fMap () - pt_1.getVector3fMap ()).squaredNorm () > squared_distance_max_) return (false);
  if ((pt_1.getVector3fMap () - pt_2.getVector3fMap ()).squaredNorm () > squared_distance_max_) return (false);
  if ((pt_2.getVector3fMap () - pt_3.getVector3fMap ()).squaredNorm () > squared_distance_max_) return (false);
  if ((pt_3.getVector3fMap () - pt_0.getVector3fMap ()).squaredNorm () > squared_distance_max_) return (false);
  return (true);
}

////////////////////////////////////////////////////////////////////////////////
