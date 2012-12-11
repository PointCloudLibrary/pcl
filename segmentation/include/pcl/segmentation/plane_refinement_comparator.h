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
 * $Id: extract_clusters.h 5027 2012-03-12 03:10:45Z rusu $
 *
 */

#ifndef PCL_SEGMENTATION_PLANAR_REFINEMENT_COMPARATOR_H_
#define PCL_SEGMENTATION_PLANAR_REFINEMENT_COMPARATOR_H_

#include <pcl/segmentation/boost.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>

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

      typedef boost::shared_ptr<PlaneRefinementComparator<PointT, PointNT, PointLT> > Ptr;
      typedef boost::shared_ptr<const PlaneRefinementComparator<PointT, PointNT, PointLT> > ConstPtr;

      using pcl::PlaneCoefficientComparator<PointT, PointNT>::input_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::normals_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::distance_threshold_;
      using pcl::PlaneCoefficientComparator<PointT, PointNT>::plane_coeff_d_;


      /** \brief Empty constructor for PlaneCoefficientComparator. */
     PlaneRefinementComparator ()
        : models_ ()
        , labels_ ()
        , refine_labels_ ()
        , label_to_model_ ()
        , depth_dependent_ (false)
      {
      }

      /** \brief Empty constructor for PlaneCoefficientComparator. 
        * \param[in] models
        * \param[in] refine_labels
        */
      PlaneRefinementComparator (boost::shared_ptr<std::vector<pcl::ModelCoefficients> >& models,
                                 boost::shared_ptr<std::vector<bool> >& refine_labels)
        : models_ (models)
        , labels_ ()
        , refine_labels_ (refine_labels)
        , label_to_model_ ()
        , depth_dependent_ (false)
      {
      }

      /** \brief Destructor for PlaneCoefficientComparator. */
      virtual
      ~PlaneRefinementComparator ()
      {
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
      inline void
      setLabelToModel (boost::shared_ptr<std::vector<int> >& label_to_model)
      {
        label_to_model_ = label_to_model;
      }
      
      /** \brief A mapping from label to index in the vector of models, allowing the model coefficients of a label to be accessed.
        * \param[in] label_to_model A vector of size max_label, with the index of each corresponding model in models
        */
      inline void
      setLabelToModel (std::vector<int>& label_to_model)
      {
        label_to_model_ = boost::make_shared<std::vector<int> >(label_to_model);
      }

      /** \brief Get the vector of model coefficients to which we will compare. */
      inline boost::shared_ptr<std::vector<pcl::ModelCoefficients> >
      getModelCoefficients () const
      {
        return (models_);
      }

      /** \brief ...
        * \param[in] labels
        */
      inline void
      setLabels (PointCloudLPtr& labels)
      {
        labels_ = labels;
      }

      /** \brief Compare two neighboring points
        * \param[in] idx1 The index of the first point.
        * \param[in] idx2 The index of the second point.
        */
      virtual bool
      compare (int idx1, int idx2) const
      {
        int current_label = labels_->points[idx1].label;
        int next_label = labels_->points[idx2].label;

        if (!((*refine_labels_)[current_label] && !(*refine_labels_)[next_label]))
          return (false);
        
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
      boost::shared_ptr<std::vector<pcl::ModelCoefficients> > models_;
      PointCloudLPtr labels_;
      boost::shared_ptr<std::vector<bool> > refine_labels_;
      boost::shared_ptr<std::vector<int> > label_to_model_;
      bool depth_dependent_;
      using PlaneCoefficientComparator<PointT, PointNT>::z_axis_;
  };
}

#endif // PCL_SEGMENTATION_PLANE_COEFFICIENT_COMPARATOR_H_
