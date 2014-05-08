/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2014, RadiantBlue Technologies, Inc.
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
 * $Id$
 *
 */

#ifndef PCL_FILTERS_LOCAL_MIN_MAX_H_
#define PCL_FILTERS_LOCAL_MIN_MAX_H_

#include <pcl/filters/filter_indices.h>
#include <pcl/search/pcl_search.h>
#include <pcl/ModelCoefficients.h>

namespace pcl
{
  /** \brief LocalMinMax downsamples the cloud, by eliminating points that are locally minimal/maximal.
    *
    * The LocalMinMax class analyzes each point and removes those that are
    * found to be locally minimal/maximal with respect to their neighbors.  
    * Box, radius, K-Nearest Neighbors and grid locality types are supported.
    * The comparison can be made in the x, y, or z dimensions.
    *
    * \author James W. O'Meara
    * \ingroup filters
    */
  template <typename PointT>
  class LocalMinMax: public FilterIndices<PointT>
  {
    protected:
      typedef typename FilterIndices<PointT>::PointCloud PointCloud;
      typedef typename pcl::search::Search<PointT>::Ptr SearcherPtr;
      typedef typename pcl::octree::OctreePointCloudSearch<PointT>::Ptr OctreePtr;

    public:
      typedef enum LocalityType
      {
        LT_BOX    = 0, /**< Compare with neighbors within box.    */
        LT_RADIUS = 1, /**< Compare with neighbors within radius. */
        LT_KNN    = 2, /**< Compare with K nearest neighbors.     */
        LT_GRID   = 3, /**< Compare with neighbors within grid.   */
        LT_LENGTH = 4
      } LocalityType;

      typedef enum StatType
      {
        ST_MIN    = 0, /**< Compute the min of each locality.    */
        ST_MAX    = 1, /**< Compute the max of each locality.    */
        ST_LENGTH = 2
      } StatType;

      /** \brief Empty constructor. */
      LocalMinMax (bool extract_removed_indices = false) :
        FilterIndices<PointT>::FilterIndices (extract_removed_indices),
        searcher_ (),
        radius_ (0),
        num_neighbors_ (0),
        resolution_ (1.0f),
        locality_type_ (LT_BOX),
        stat_type_ (ST_MIN)
      {
        filter_name_ = "LocalMinMax";

        // set the default model coefficients to represent the XY plane
        ModelCoefficientsPtr model_coeffs (new pcl::ModelCoefficients ());
        model_coeffs->values.resize (4);
        model_coeffs->values[0] = model_coeffs->values[1] = model_coeffs->values[3] = 0;
        model_coeffs->values[2] = 1.0;
        model_ = model_coeffs;
      }

      /** \brief Set the radius to use to determine if a point is within our locality.
        * \param[in] radius The radius to use to determine if a point is within our locality.
        */
      inline void
      setRadius (float radius) { radius_ = radius; }

      /** \brief Get the radius to use to determine if a point is within our locality.
        * \param[in] radius The radius to use to determine if a point is within our locality.
        */
      inline float
      getRadius () const { return (radius_); }

      /** \brief Set the number of neighbors to use to determine if a point is within our locality.
        * \param[in] num_neighbors The number of neighbors to use to determine if a point is within our locality.
        */
      inline void
      setNumNeighbors (const int num_neighbors) { num_neighbors_ = num_neighbors; }

      /** \brief Get the number of neighbors to use to determine if a point is within our locality.
        */
      inline int
      getNumNeighbors () const { return (num_neighbors_); }
      
      /** \brief Set the bounding/grid box resolution.
        * \param[in] resolution The bounding box/grid resolution
        */
      inline void
      setResolution (const float resolution)
      {
        resolution_ = resolution;
        inverse_resolution_ = 1.0 / resolution_;
      }

      /** \brief Get the bounding box/grid resolution. */
      inline float
      getResolution () { return (resolution_); }

      /** \brief Set the method that will be used to define our locality. 
        * \param[in] locality_type The method that will be used to define our locality. 
        */
      inline void
      setLocalityType (const LocalityType locality_type) { locality_type_ = locality_type; }

      /** \brief Get the method that will be used to define our locality. 
        */
      inline LocalityType
      getLocalityType () const { return (locality_type_); }

      /** \brief Set the stat that will be computed for each locality.
        * \param[in] stat_type The stat that will be computed for each locality. 
        */
      inline void
      setStatType (const StatType stat_type) { stat_type_ = stat_type; }

      /** \brief Get the stat that will be computed for each locality.
        */
      inline StatType
      getStatType () const { return (stat_type_); }

      /** \brief Provide a pointer to the vector of model coefficients which represent the plane to which points will be projected.
        * \param model a pointer to the vector of model coefficients which represent the plane to which points will be projected.
        */
      inline void
      setModelCoefficients (const ModelCoefficientsConstPtr &model)
      {
        model_ = model;
      }

      /** \brief Get a pointer to the vector of model coefficients which represent the plane to which points will be projected.
        */
      inline ModelCoefficientsConstPtr
      getModelCoefficients ()
      {
        return (model_);
      }

    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Downsample a Point Cloud by eliminating points that are locally minimal/maximal in x, y, or z
        * \param[out] output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilter (std::vector<int> &indices)
      {
        applyFilterIndices (indices);
      }

      /** \brief Filtered results are indexed by an indices array.
        * \param[out] indices The resultant indices.
        */
      void
      applyFilterIndices (std::vector<int> &indices);

      /** \brief A pointer to the spatial search object. */
      SearcherPtr searcher_;

      /** \brief A pointer to the octree search object. */
      OctreePtr octree_;

      /** \brief The radius to use to determine if a point is within our locality. */
      float radius_;

      /** \brief The number of neighbors to use to determine if a point is within our locality. */
      int num_neighbors_;
      
      /** \brief The resolution to use to determine if a point is within our locality. */
      float resolution_;

      /** \brief The inverse resolution to use to determine if a point is within our locality. */
      float inverse_resolution_;

      /** \brief The method that will be used to define our locality. */
      LocalityType locality_type_;

      /** \brief The stat that will be computed for each locality. */
      StatType stat_type_;

      /** \brief A pointer to the vector of model coefficients which represent the plane to which points will be projected. */
      ModelCoefficientsConstPtr model_;

      /** \brief Filtered results are indexed by an indices array using grid locality.
        * \param[out] indices The resultant indices.
        */
      virtual void
      applyGridFilter (std::vector<int> &indices);

      /** \brief Filtered results are indexed by an indices array using local locality.
        * \param[out] indices The resultant indices.
        */
      virtual void
      applyLocalFilter (std::vector<int> &indices);
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/filters/impl/local_min_max.hpp>
#endif

#endif  //#ifndef PCL_FILTERS_LOCAL_MIN_MAX_H_

