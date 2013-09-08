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

#ifndef PCL_FILTERS_ModelOutlierRemoval_H_
#define PCL_FILTERS_ModelOutlierRemoval_H_

#include <pcl/filters/filter_indices.h>
#include <pcl/ModelCoefficients.h>

// Sample Consensus models
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_normal_plane.h>
#include <pcl/sample_consensus/sac_model_normal_sphere.h>
#include <pcl/sample_consensus/sac_model_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_normal_parallel_plane.h>
#include <pcl/sample_consensus/sac_model_parallel_line.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>





namespace pcl
{

  /** \brief @b ModelOutlierRemoval filters points in a cloud based on the distance between model and point.
    * \details Iterates through the entire input once, automatically filtering non-finite points and the points outside
    * the model specified by setSampleConsensusModelPointer() and the threshold specified by setThreholdFunctionPointer().
    * <br><br>
    * Usage example:
    * \code
    * pcl::ModelCoefficients model_coeff;
    * model_coeff.values.resize(4);
    * model_coeff.values[0] = 0; model_coeff.values[1] = 0; model_coeff.values[2] = 1.5; model_coeff.values[3] = 0.5;
    * pcl::ModelOutlierRemoval<pcl::PointXYZ> filter;
    * filter.setModelCoefficients( model_coeff);
    * filter.setThreshold( 0.1 );
    * filter.setModelType( pcl::SACMODEL_PLANE );
    * filter.setInputCloud( *cloud_in );
    * filter.setFilterLimitsNegative(false);
    * filter.filter( *cloud_out );
    * \endcode
    */
  template <typename PointT>
  class ModelOutlierRemoval : public FilterIndices<PointT>
  {
    protected:
      typedef typename FilterIndices<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef typename SampleConsensusModel<PointT>::Ptr SampleConsensusModelPtr;


    public:
      /** \brief Constructor.
        * \param[in] extract_removed_indices Set to true if you want to be able to extract the indices of points being removed (default = false).
        */
      ModelOutlierRemoval (bool extract_removed_indices = false) :
        FilterIndices<PointT>::FilterIndices (extract_removed_indices)
      {
        model_from_normals = NULL;
        thresh = 0;
        normals_distance_weight = 0;
        filter_name_ = "ModelOutlierRemoval";
        setThresholdFunction(&pcl::ModelOutlierRemoval<PointT>::single_threshold, *this );
      }
      /** \brief sets the models coefficients */
      inline void 
      setModelCoefficients (const pcl::ModelCoefficients model_coefficients_)
      {
         model_coefficients.resize( model_coefficients_.values.size() );
         for(unsigned int i = 0; i < model_coefficients_.values.size(); i++)
         {
           model_coefficients[i] = model_coefficients_.values[i];
         }
      }
      /** \brief returns the models coefficients
        */
      pcl::ModelCoefficients 
      getModelCoefficients()
      {
         pcl::ModelCoefficients mc;
         mc.values.resize( model_coefficients.size() );
         for( unsigned int i = 0; i < mc.values.size(); i++)
           mc.values[i] = model_coefficients[i];
         return mc;
      }
      /** \brief Set the type of SAC model used. */
      inline void
      setModelType (int model)
      {
        model_type_ = model;
      }

      /** \brief Get the type of SAC model used. */
      inline int
      getModelType ()
      {
        return (model_type_);
      }

      /** \brief Set the thresholdfunction*/
      inline void
      setThreshold( float thresh_)
      {
        thresh = thresh_;
      }

      /** \brief Get the thresholdfunction*/
      inline float
      getThreshhold()
      {
        return thresh;
      }

      /** \brief Set the normals cloud*/
      inline void
      setInputNormals( pcl::PointCloud<pcl::Normal>::Ptr normals_ptr)
      {
        cloud_normals = normals_ptr;   
      }
      /** \brief Get the normals cloud*/
      inline pcl::PointCloud<pcl::Normal>::Ptr
      getInputNormals()
      {
        return cloud_normals;   
      }
      /** \brief Set the normals distance weight*/
      inline void
      setNormalDistanceWeight( const double& weight)
      {
        normals_distance_weight = weight;
      }
      /** \brief get the normal distance weight*/
      inline double
      getNormalDistanceWeight()
      {
        return normals_distance_weight;
      }



      /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max)
        * Default: false.
        * \warning This method will be removed in the future. Use setNegative() instead.
        * \param[in] limit_negative return data inside the interval (false) or outside (true)
        */
      inline void
      setFilterLimitsNegative (const bool limit_negative)
      {
        negative_ = limit_negative;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \warning This method will be removed in the future. Use getNegative() instead.
        * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline void
      getFilterLimitsNegative (bool &limit_negative)
      {
        limit_negative = negative_;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \warning This method will be removed in the future. Use getNegative() instead.
        * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline bool
      getFilterLimitsNegative ()
      {
        return (negative_);
      }

     /** \brief Register a different threshold function
      * \param[in] pointer to a threshold function
      */
 
      void
      setThresholdFunction (boost::function<bool (double)> thresh_)
      {
        threshold_function = thresh_;
      };


     /** \brief Register a different threshold function
      * \param[in] pointer to a threshold function
      * \param[in] instance
      */
      template<typename T> void
      setThresholdFunction (bool (T::*thresh_function) (double), T& instance)
      {
        setThresholdFunction (boost::bind (thresh_function, boost::ref (instance), _1));
      }


    protected:
      using PCLBase<PointT>::input_;
      using PCLBase<PointT>::indices_;
      using Filter<PointT>::filter_name_;
      using Filter<PointT>::getClassName;
      using FilterIndices<PointT>::negative_;
      using FilterIndices<PointT>::keep_organized_;
      using FilterIndices<PointT>::user_filter_value_;
      using FilterIndices<PointT>::extract_removed_indices_;
      using FilterIndices<PointT>::removed_indices_;

      /** \brief Filtered results are stored in a separate point cloud.
        * \param[out] output The resultant point cloud.
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

    protected:
      double normals_distance_weight;
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
      SampleConsensusModelFromNormals< PointT, pcl::Normal > *model_from_normals;
      /** \brief The model used to calculate distances */
      SampleConsensusModelPtr model;
      /** \brief The threshold used to seperate outliers (removed_indices) from inliers (indices) */
      float thresh;
      /** \brief The model coefficients */
      Eigen::VectorXf model_coefficients;
      /** \brief The type of model to use (user given parameter). */
      int model_type_;
      boost::function<bool (double)> threshold_function;

      bool 
      single_threshold( double value )
      {
        if( value < thresh ) return true;
        return false;
      }

    private:
      virtual bool
      initSACModel (int model_type);
  };


  ////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b ModelOutlierRemoval filters points in a cloud based on the distance between model and point.
    * \details Iterates through the entire input once, automatically filtering non-finite points and the points outside
    * the model specified by setSampleConsensusModelPointer() and the threshold specified by setThreholdFunctionPointer().
    */
  template<>
  class PCL_EXPORTS ModelOutlierRemoval<pcl::PCLPointCloud2> : public Filter<pcl::PCLPointCloud2>
  {
    typedef pcl::PCLPointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;
    typedef SampleConsensusModel<pcl::PointXYZ>::Ptr SampleConsensusModelPtr;

    using Filter<pcl::PCLPointCloud2>::removed_indices_;
    using Filter<pcl::PCLPointCloud2>::extract_removed_indices_;

    public:
      /** \brief Constructor. */
      ModelOutlierRemoval (bool extract_removed_indices = false) :
        Filter<pcl::PCLPointCloud2>::Filter (extract_removed_indices)
      {
        filter_name_ = "ModelOutlierRemoval";
        model_from_normals = NULL;
        normals_distance_weight = 0;
        thresh = 0;
        setThresholdFunction(&pcl::ModelOutlierRemoval<pcl::PCLPointCloud2>::single_threshold, *this );
      }

      /** \brief Set whether the filtered points should be kept and set to the
        * value given through \a setUserFilterValue (default: NaN), or removed
        * from the PointCloud, thus potentially breaking its organized
        * structure. By default, points are removed.
        *
        * \param[in] val set to true whether the filtered points should be kept and
        * set to a given user value (default: NaN)
        */
      inline void
      setKeepOrganized (bool val)
      {
        keep_organized_ = val;
      }

      /** \brief Obtain the value of the internal \a keep_organized_ parameter. */
      inline bool
      getKeepOrganized ()
      {
        return (keep_organized_);
      }

      /** \brief Provide a value that the filtered points should be set to
        * instead of removing them.  Used in conjunction with \a
        * setKeepOrganized ().
        * \param[in] val the user given value that the filtered point dimensions should be set to
        */
      inline void
      setUserFilterValue (float val)
      {
        user_filter_value_ = val;
      }

      /** \brief sets the models coefficients */
      inline void 
      setModelCoefficients (const pcl::ModelCoefficients model_coefficients_)
      {
         model_coefficients.resize( model_coefficients_.values.size() );
         for(unsigned int i = 0; i < model_coefficients_.values.size(); i++)
         {
           model_coefficients[i] = model_coefficients_.values[i];
         }
      }
      /** \brief returns the models coefficients
        */
      pcl::ModelCoefficients 
      getModelCoefficients()
      {
         pcl::ModelCoefficients mc;
         mc.values.resize( model_coefficients.size() );
         for( unsigned int i = 0; i < mc.values.size(); i++)
           mc.values[i] = model_coefficients[i];
         return mc;
      }
      /** \brief Set the type of SAC model used. */
      inline void
      setModelType (int model)
      {
        model_type_ = model;
      }

      /** \brief Get the type of SAC model used. */
      inline int
      getModelType ()
      {
        return (model_type_);
      }

      /** \brief Set the thresholdfunction*/
      inline void
      setThreshold( float thresh_)
      {
         thresh = thresh_;
      }

      /** \brief Get the thresholdfunction*/
      inline float
      getThreshold()
      {
         return thresh;
      }

      /** \brief Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max)
        * Default: false.
        * \warning This method will be removed in the future. Use setNegative() instead.
        * \param[in] limit_negative return data inside the interval (false) or outside (true)
        */
      inline void
      setFilterLimitsNegative (const bool limit_negative)
      {
        negative_ = limit_negative;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \warning This method will be removed in the future. Use getNegative() instead.
        * \param[out] limit_negative true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline void
      getFilterLimitsNegative (bool &limit_negative)
      {
        limit_negative = negative_;
      }

      /** \brief Get whether the data outside the interval (min/max) is to be returned (true) or inside (false). 
        * \warning This method will be removed in the future. Use getNegative() instead.
        * \return true if data \b outside the interval [min; max] is to be returned, false otherwise
        */
      inline bool
      getFilterLimitsNegative ()
      {
        return (negative_);
      }

      /** \brief Set the normals cloud*/
      inline void
      setInputNormals( pcl::PointCloud<pcl::Normal>::Ptr normals_ptr)
      {
        cloud_normals = normals_ptr;   
      }
      /** \brief Get the normals cloud*/
      inline pcl::PointCloud<pcl::Normal>::Ptr
      getInputNormals()
      {
        return cloud_normals;   
      }
      /** \brief Set the normals distance weight*/
      inline void
      setNormalDistanceWeight( const double& weight)
      {
        normals_distance_weight = weight;
      }
      /** \brief get the normal distance weight*/
      inline double
      getNormalDistanceWeight()
      {
        return normals_distance_weight;
      }

      /** \brief Register a different threshold function
      * \param[in] pointer to a threshold function
      */
 
      void
      setThresholdFunction (boost::function<bool (double)> thresh_)
      {
        threshold_function = thresh_;
      };


      /** \brief Register a different threshold function
      * \param[in] pointer to a threshold function
      * \param[in] instance
      */
      template<typename T> void
      setThresholdFunction (bool (T::*thresh_function) (double), T& instance)
      {
        setThresholdFunction (boost::bind (thresh_function, boost::ref (instance), _1));
      }


    protected:
      void
      applyFilter (PointCloud2 &output);

      boost::function<bool (double)> threshold_function;

      bool 
      single_threshold( double value )
      {
        if( value < thresh ) return true;
        return false;
      }

    private:
      /** \brief Keep the structure of the data organized, by setting the
        * filtered points to the a user given value (NaN by default). 
        */
      bool keep_organized_;

      /** \brief User given value to be set to any filtered point. Casted to
        * the correct field type. 
        */
      float user_filter_value_;

      /** \brief Set to true if we want to return the data outside (\a filter_limit_min_;\a filter_limit_max_). Default: false. */
      bool negative_;

      double normals_distance_weight;
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

      SampleConsensusModelFromNormals< pcl::PointXYZ, pcl::Normal > *model_from_normals;
      /** \brief The model used to calculate distances */
      SampleConsensusModelPtr model;
      /** \brief The threshold used to seperate outliers (removed_indices) from inliers (indices) */
      float thresh;
      /** \brief The model coefficients */
      Eigen::VectorXf model_coefficients;
      /** \brief The type of model to use (user given parameter). */
      int model_type_;
      virtual bool
      initSACModel (int model_type);
  };

}

#endif  // PCL_FILTERS_ModelOutlierRemoval_H_

