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

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/random.h>

namespace pcl
{

  namespace common
  {
    /** \brief CloudGenerator class generates a point cloud using some randoom number generator.
      * Generators can be found in \file common/random.h and easily extensible.
      *  
      * \ingroup common
      * \author Nizar Sallem
      */
    template <typename PointT, typename GeneratorT>
    class CloudGenerator
    {
      public:
      using GeneratorParameters = typename GeneratorT::Parameters;

      /// Default constructor
      CloudGenerator ();

      /** Constructor with single generator to ensure all X, Y and Z values are within same range
        * \param params parameters for X, Y and Z values generation. Uniqueness is ensured through
        * seed incrementation
        */
      CloudGenerator (const GeneratorParameters& params);

      /** Constructor with independent generators per axis
        * \param x_params parameters for x values generation
        * \param y_params parameters for y values generation
        * \param z_params parameters for z values generation
        */
      CloudGenerator (const GeneratorParameters& x_params,
                      const GeneratorParameters& y_params,
                      const GeneratorParameters& z_params);

      /** Set parameters for x, y and z values. Uniqueness is ensured through seed incrementation.
        * \param params parameteres for X, Y and Z values generation. 
        */
      void
      setParameters (const GeneratorParameters& params);
      
      /** Set parameters for x values generation
        * \param x_params parameters for x values generation
        */
      void
      setParametersForX (const GeneratorParameters& x_params);

      /** Set parameters for y values generation
        * \param y_params parameters for y values generation
        */
      void
      setParametersForY (const GeneratorParameters& y_params);
      
      /** Set parameters for z values generation
        * \param z_params parameters for z values generation
        */
      void
      setParametersForZ (const GeneratorParameters& z_params);

      /// \return x values generation parameters
      const GeneratorParameters& 
      getParametersForX () const;

      /// \return y values generation parameters
      const GeneratorParameters& 
      getParametersForY () const;

      /// \return z values generation parameters
      const GeneratorParameters& 
      getParametersForZ () const;
      
      /// \return a single random generated point 
      PointT 
      get ();
        
      /** Generates a cloud with X Y Z picked within given ranges. This function assumes that
        * cloud is properly defined else it raises errors and does nothing.
        * \param[out] cloud cloud to generate coordinates for
        * \return 0 if generation went well else -1.
        */
      int
      fill (pcl::PointCloud<PointT>& cloud);

      /** Generates a cloud of specified dimensions with X Y Z picked within given ranges. 
        * \param[in] width width of generated cloud
        * \param[in] height height of generated cloud
        * \param[out] cloud output cloud
        * \return 0 if generation went well else -1.
        */
      int 
      fill (int width, int height, pcl::PointCloud<PointT>& cloud);
      
      private:
        GeneratorT x_generator_, y_generator_, z_generator_;
    };

    template <typename GeneratorT>
    class CloudGenerator<pcl::PointXY, GeneratorT>
    {
      public:
      using GeneratorParameters = typename GeneratorT::Parameters;
      
      CloudGenerator ();
      
      CloudGenerator (const GeneratorParameters& params);

      CloudGenerator (const GeneratorParameters& x_params,
                      const GeneratorParameters& y_params);
      
      void
      setParameters (const GeneratorParameters& params);

      void
      setParametersForX (const GeneratorParameters& x_params);

      void
      setParametersForY (const GeneratorParameters& y_params);

      const GeneratorParameters& 
      getParametersForX () const;

      const GeneratorParameters& 
      getParametersForY () const;

      pcl::PointXY
      get ();

      int 
      fill (pcl::PointCloud<pcl::PointXY>& cloud);

      int 
      fill (int width, int height, pcl::PointCloud<pcl::PointXY>& cloud);
      
      private:
        GeneratorT x_generator_;
        GeneratorT y_generator_;
    };

    /**
     * @brief Allows generation of a point cloud using a random point generator
     * 
     * @tparam PointT Type of point cloud (eg: PointNormal, PointXYZRGB, etc.)
     * @tparam GeneratorT A generator following the generator classes as shown in \file common/random.h
     */
    template <typename PointT, typename GeneratorT>
    class PointCloudGenerator {
    public:
      using GeneratorParameters = typename GeneratorT::Parameters;

      /// Default constructor
      PointCloudGenerator() : point_generator_() {}

      /**
       * \brief Constructor with a generator to fill a cloud
       * \details Uniqueness is ensured by incrementing the seed
       * \param params parameters for point generation.
       */
      PointCloudGenerator(const GeneratorParameters& params) : point_generator_(params)
      {}

      /** Set parameters for point generation
       * \param x_params parameters for point generation
       */
      void
      setParameters(const GeneratorParameters& pt_params)
      {
        _point_generator.setParameters(pt_params);
      }

      /// \return Point generation parameters
      const GeneratorParameters&
      getParameters() const
      {
        return point_generator_.getParameters();
      }

      /// \return a single random generated point
      PointT
      get()
      {
        return point_generator_.run();
      }

      /**
       * \brief Generates a cloud with the supplied generator and parameters.
       * \note This function assumes that cloud is properly defined
       * \param[out] cloud cloud to generate coordinates for
       * \return 0 if generation went well else -1.
       */
      int
      fill(pcl::PointCloud<PointT>& cloud)
      {
        return fill(cloud.width, cloud.height, cloud);
      }

      /**
       * \brief Generates a cloud with the supplied generator and parameters.
       * \param[in] width width of generated cloud
       * \param[in] height height of generated cloud
       * \param[out] cloud output cloud
       * \return 0 if generation went well else -1.
       */
      int
      fill(int width, int height, pcl::PointCloud<PointT>& cloud)
      {
        if (width < 1) {
          PCL_ERROR("[pcl::common::CloudGenerator] Cloud width must be >= 1!\n");
          return (-1);
        }

        if (height < 1) {
          PCL_ERROR("[pcl::common::CloudGenerator] Cloud height must be >= 1!\n");
          return (-1);
        }

        if (!cloud.empty()) {
          PCL_WARN("[pcl::common::CloudGenerator] Cloud data will be erased with new "
                   "data!\n");
        }

        cloud.resize(width, height);
        cloud.is_dense = true;

        for (auto& point : cloud) {
          point = point_generator_.run();
        }
        return (0);
      }

    private:
      GeneratorT point_generator_;
    };
  }
}

#include <pcl/common/impl/generate.hpp>
