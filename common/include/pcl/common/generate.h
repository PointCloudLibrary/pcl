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

#ifndef PCL_COMMON_GENERATE_H_
#define PCL_COMMON_GENERATE_H_

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
      typedef typename GeneratorT::Parameters GeneratorParameters;

      /// Default constructor
      CloudGenerator ();

      /** Consttructor with single generator to ensure all X, Y and Z values are within same range
        * \param params paramteres for X, Y and Z values generation. Uniqueness is ensured through
        * seed incrementation
        */
      CloudGenerator (const GeneratorParameters& params);

      /** Constructor with independant generators per axis
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
        * \param x_params paramters for x values generation
        */
      void
      setParametersForX (const GeneratorParameters& x_params);

      /** Set parameters for y values generation
        * \param y_params paramters for y values generation
        */
      void
      setParametersForY (const GeneratorParameters& y_params);
      
      /** Set parameters for z values generation
        * \param z_params paramters for z values generation
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
      typedef typename GeneratorT::Parameters GeneratorParameters;
      
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
  }
}

#include <pcl/common/impl/generate.hpp>

#endif
