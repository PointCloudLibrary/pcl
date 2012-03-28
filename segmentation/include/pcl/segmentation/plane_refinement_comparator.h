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
 *
 */

#ifndef PCL_SEGMENTATION_PLANAR_REFINEMENT_COMPARATOR_H_
#define PCL_SEGMENTATION_PLANAR_REFINEMENT_COMPARATOR_H_

#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <boost/make_shared.hpp>

namespace pcl
{
  /** \brief PlaneRefinementComparator is a Comparator that operates on plane coefficients, 
    * for use in planar segmentation.
    * In conjunction with OrganizedConnectedComponentSegmentation, this allows planes to be segmented from organized data.
    *
    * \author Alex Trevor, Suat Gedikli
    */
  template<typename PointT, typename PointNT, typename PointLT>
  class PlaneRefinementComparator: public PlaneCoefficientComparator<PointT, PointNT>
  {
    public:
      typedef typename Comparator<PointT>::PointCloud PointCloud;
      typedef typename Comparator<PointT>::PointCloudConstPtr PointCloudConstPtr;
      
      typedef typename pcl::PointCloud<PointNT> PointCloudN;
      typedef typename PointCloudN::Ptr PointCloudNPtr;
      typedef typename PointCloudN::ConstPtr PointCloudNConstPtr;

      typedef typename pcl::PointCloud<PointLT> PointCloudL;
      typedef typename PointCloudL::Ptr PointCloudLPtr;
      typedef typename PointCloudL::ConstPtr PointCloudLConstPtr;

      using pcl::PlaneCoefficientComparator<PointT, PointNT>::input_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::normals_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::plane_coeff_d_;

      typedef boost::shared_ptr<PlaneRefinementComparator<PointT, PointNT, PointLT> > Ptr;

      /** \brief Empty constructor for PlaneCoefficientComparator. */
     PlaneRefinementComparator ()
        : distance_map_ ()
        , angular_threshold_ (0.0f)
        , distance_threshold_ (0.02f)
        , models_ ()
        , labels_ ()
        , refine_labels_ ()
        , label_to_model_ ()
        , depth_dependent_ (true)
      {
      }

      PlaneRefinementComparator (boost::shared_ptr<std::vector<pcl::ModelCoefficients> >& models,
                                 boost::shared_ptr<std::vector<bool> >& refine_labels)
        : distance_map_ ()
        , angular_threshold_ (0.0f)
        , distance_threshold_ (0.02f)
        , models_ (models)
        , labels_ ()
        , refine_labels_ (refine_labels)
        , label_to_model_ ()
        , depth_dependent_ (true)
      {
      }

      /** \brief Constructor for PlaneCoefficientComparator.
        * \param[in] plane_coeff_d a reference to a vector of d coefficients of plane equations.  Must be the same size as the input cloud and input normals.  a, b, and c coefficients are in the input normals.
        */
      PlaneRefinementComparator (boost::shared_ptr<std::vector<float> >& plane_coeff_d) 
        : angular_threshold_ (0), distance_threshold_ (0.02)
      {
      }   
      /** \brief Destructor for PlaneCoefficientComparator. */
      virtual
      ~PlaneRefinementComparator ()
      {
      }

      /** \brief Provide a pointer to the input normals.
        * \param[in] normals the input normal cloud
        */
      inline void
      setInputNormals (const PointCloudNConstPtr &normals)
      {
        normals_ = normals;
      }

      /** \brief Get the input normals. */
      inline PointCloudNConstPtr
      getInputNormals () const
      {
        return (normals_);
      }

      /** \brief Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.  a, b, and c are provided by the normal cloud.
        * \param[in] plane_coeff_d a pointer to the plane coefficients.
        */
      void
      setPlaneCoeffD (boost::shared_ptr<std::vector<float> >& plane_coeff_d)
      {
        plane_coeff_d_ = plane_coeff_d;
      }

      /** \brief Provide a pointer to a vector of the d-coefficient of the planes' hessian normal form.  a, b, and c are provided by the normal cloud.
        * \param[in] plane_coeff_d a pointer to the plane coefficients.
        */
      void
      setPlaneCoeffD (std::vector<float>& plane_coeff_d)
      {
        plane_coeff_d_ = boost::make_shared<std::vector<float> >(plane_coeff_d);
      }
      
      /** \brief Get a pointer to the vector of the d-coefficient of the planes' hessian normal form. */
      const std::vector<float>&
      getPlaneCoeffD () const
      {
        return (plane_coeff_d_);
      }

      /** \brief Set the tolerance in radians for difference in normal direction between neighboring points, to be considered part of the same plane.
        * \param[in] angular_threshold the tolerance in radians
        */
      inline void
      setAngularThreshold (float angular_threshold)
      {
        printf ("euclidean set angular threshold!\n");
        angular_threshold_ = cosf (angular_threshold);
      }
      
      /** \brief Get the angular threshold in radians for difference in normal direction between neighboring points, to be considered part of the same plane. */
      inline float
      getAngularThreshold () const
      {
        return (acos (angular_threshold_) );
      }

      /** \brief Set the tolerance in meters for difference in perpendicular distance (d component of plane equation) to the plane between neighboring points, to be considered part of the same plane.
        * \param[in] distance_threshold the tolerance in meters
        */
      inline void
      setDistanceThreshold (float distance_threshold, bool depth_dependent = true)
      {
        distance_threshold_ = distance_threshold;// * distance_threshold;
        depth_dependent_ = depth_dependent;
      }

      void
      setDistanceMap (float* distance_map)
      {
        distance_map_ = distance_map;
      }

      /** \brief Set the vector of model coefficients to which we will compare.
        * \param[in] models a vector of model coefficients produced by the initial segmentation step.
        */
      void
      setModelCoefficients (boost::shared_ptr<std::vector<pcl::ModelCoefficients> >& models)
      {
        models_ = models;
      }

      /** \brief Set the vector of model coefficients to which we will compare.
        * \param[in] models a vector of model coefficients produced by the initial segmentation step.
        */
      void
      setModelCoefficients (std::vector<pcl::ModelCoefficients>& models)
      {
        models_ = boost::make_shared<std::vector<pcl::ModelCoefficients> >(models);
      }

      /** \brief Set which labels should be refined.  This is a vector of bools 0-max_label, true if the label should be refined.
        * \param[in] refine_labels A vector of bools 0-max_label, true if the label should be refined.
        */
      void
      setRefineLabels (boost::shared_ptr<std::vector<bool> >& refine_labels)
      {
        refine_labels_ = refine_labels;
      }
      
      /** \brief Set which labels should be refined.  This is a vector of bools 0-max_label, true if the label should be refined.
        * \param[in] refine_labels A vector of bools 0-max_label, true if the label should be refined.
        */
      void
      setRefineLabels (std::vector<bool>& refine_labels)
      {
        refine_labels_ = boost::make_shared<std::vector<bool> >(refine_labels);
      }

      /** \brief A mapping from label to index in the vector of models, allowing the model coefficients of a label to be accessed.
        * \param[in] label_to_model A vector of size max_label, with the index of each corresponding model in models
        */
      void
      setLabelToModel (boost::shared_ptr<std::vector<int> >& label_to_model)
      {
        label_to_model_ = label_to_model;
      }
      

      void
      setLabelToModel (std::vector<int>& label_to_model)
      {
        label_to_model_ = boost::make_shared<std::vector<int> >(label_to_model);
      }

      boost::shared_ptr<std::vector<pcl::ModelCoefficients> >
      getModelCoefficients () const
      {
        return (models_);
      }

      void
      setLabels (PointCloudLPtr& labels)
      {
        labels_ = labels;
      }

      /** \brief Get the distance threshold in meters (d component of plane equation) between neighboring points, to be considered part of the same plane. */
      inline float
      getDistanceThreshold () const
      {
        return (distance_threshold_);
      }

      bool
      compare (int idx1, int idx2) const
      {
        int current_label = labels_->points[idx1].label;
        int next_label = labels_->points[idx2].label;

        // Suat: ????
        if (!((*refine_labels_)[current_label] && !(*refine_labels_)[next_label]))
          return false;
        
        const pcl::ModelCoefficients& model_coeff = (*models_)[(*label_to_model_)[current_label]];
        
        PointT pt = input_->points[idx2];
        double ptp_dist = fabs (model_coeff.values[0] * pt.x + 
                                model_coeff.values[1] * pt.y + 
                                model_coeff.values[2] * pt.z +
                                model_coeff.values[3]);
        
        // depth dependent
        float threshold = distance_threshold_;
        if (depth_dependent_)
        {
          //Eigen::Vector4f origin = input_->sensor_origin_;
          Eigen::Vector3f vec = input_->points[idx1].getVector3fMap ();// - origin.head<3> ();
          
          float z = vec.dot (z_axis_);
          threshold *= z * z;
        }
        
        return (ptp_dist < threshold);
      }

    protected:
      float* distance_map_;
      float angular_threshold_;
      float distance_threshold_;
      boost::shared_ptr<std::vector<pcl::ModelCoefficients> > models_;
      PointCloudLPtr labels_;
      boost::shared_ptr<std::vector<bool> > refine_labels_;
      boost::shared_ptr<std::vector<int> > label_to_model_;
      bool depth_dependent_;
      using PlaneCoefficientComparator<PointT, PointNT>::z_axis_;
  };
}

#endif // PCL_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_
