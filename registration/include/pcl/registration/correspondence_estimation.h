/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_
#define PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>

namespace pcl
{
  namespace registration
  {
    /** \brief Abstract @b CorrespondenceEstimationBase class. 
      * All correspondence estimation methods should inherit from this.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimationBase: public PCLBase<PointSource>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> > ConstPtr;

        using PCLBase<PointSource>::initCompute;
        using PCLBase<PointSource>::deinitCompute;
        using PCLBase<PointSource>::input_;
        using PCLBase<PointSource>::indices_;
        using PCLBase<PointSource>::setIndices;

        typedef typename pcl::KdTree<PointTarget> KdTree;
        typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceEstimationBase () 
          : corr_name_ ("CorrespondenceEstimationBase")
          , tree_ (new pcl::KdTreeFLANN<PointTarget>)
          , target_ ()
          , target_indices_ ()
          , point_representation_ ()
          , input_transformed_ ()
          , input_fields_ ()
        {
        }

        /** \brief Provide a pointer to the input source 
          * (e.g., the point cloud that we want to align to the target)
          *
          * \param[in] cloud the input point cloud source
          */
        inline void 
        setInputCloud (const PointCloudSourceConstPtr &cloud)
        {
          PCL_WARN ("[pcl::registration::%s::setInputCloud] setInputCloud is deprecated. Please use setInputSource instead.\n", getClassName ().c_str ());
          PCLBase<PointSource>::setInputCloud (cloud);
          pcl::getFields (*cloud, input_fields_);
        }

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudSourceConstPtr const 
        getInputCloud () 
        { 
          PCL_WARN ("[pcl::registration::%s::getInputCloud] getInputCloud is deprecated. Please use getInputSource instead.\n", getClassName ().c_str ());
          return (input_ ); 
        }

        /** \brief Provide a pointer to the input source 
          * (e.g., the point cloud that we want to align to the target)
          *
          * \param[in] cloud the input point cloud source
          */
        inline void 
        setInputSource (const PointCloudSourceConstPtr &cloud)
        {
          PCLBase<PointSource>::setInputCloud (cloud);
          pcl::getFields (*cloud, input_fields_);
        }

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudSourceConstPtr const 
        getInputSource () 
        { 
          return (input_ ); 
        }

        /** \brief Provide a pointer to the input target 
          * (e.g., the point cloud that we want to align the input source to)
          * \param[in] cloud the input point cloud target
          */
        inline void 
        setInputTarget (const PointCloudTargetConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudTargetConstPtr const 
        getInputTarget () { return (target_ ); }

        /** \brief Provide a pointer to the vector of indices that represent the 
          * input source point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesSource (const IndicesPtr &indices)
        {
          setIndices (indices);
        }

        /** \brief Get a pointer to the vector of indices used for the source dataset. */
        inline IndicesPtr const 
        getIndicesSource () { return (indices_); }

        /** \brief Provide a pointer to the vector of indices that represent the 
          * input target point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesTarget (const IndicesPtr &indices)
        {
          target_indices_ = indices;
        }

        /** \brief Get a pointer to the vector of indices used for the target dataset. */
        inline IndicesPtr const 
        getIndicesTarget () { return (target_indices_); }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ()) = 0;

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ()) = 0;

        /** \brief Provide a boost shared pointer to the PointRepresentation to be used 
          * when searching for nearest neighbors.
          *
          * \param[in] point_representation the PointRepresentation to be used by the 
          * k-D tree for nearest neighbor search
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
        }

        /** \brief Provide a simple mechanism to update the internal source cloud
          * using a given transformation. Used in registration loops.
          * \param[in] transform the transform to apply over the source cloud
          */
        virtual bool
        updateSource (const Eigen::Matrix<Scalar, 4, 4> &transform) = 0;

      protected:
        /** \brief The correspondence estimation method name. */
        std::string corr_name_;

        /** \brief A pointer to the spatial search object. */
        KdTreePtr tree_;

        /** \brief The input point cloud dataset target. */
        PointCloudTargetConstPtr target_;

        /** \brief The target point cloud dataset indices. */
        IndicesPtr target_indices_;

        /** \brief The point representation used (internal). */
        PointRepresentationConstPtr point_representation_;

        /** \brief The transformed input source point cloud dataset. */
        PointCloudTargetPtr input_transformed_;

        /** \brief The types of input point fields available. */
        std::vector<sensor_msgs::PointField> input_fields_;

        /** \brief Abstract class get name method. */
        inline const std::string& 
        getClassName () const { return (corr_name_); }

        /** \brief Internal computation initalization. */
        bool
        initCompute ();
     };

    /** \brief @b CorrespondenceEstimation represents the base class for
      * determining correspondences between target and query point
      * sets/features.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CorrespondenceEstimation : public CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<CorrespondenceEstimation<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const CorrespondenceEstimation<PointSource, PointTarget, Scalar> > ConstPtr;

        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
        using PCLBase<PointSource>::deinitCompute;

        typedef typename pcl::KdTree<PointTarget> KdTree;
        typedef typename pcl::KdTree<PointTarget>::Ptr KdTreePtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        CorrespondenceEstimation () 
        {
          corr_name_  = "CorrespondenceEstimation";
        }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ());

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ());

        /** \brief Provide a simple mechanism to update the internal source cloud
          * using a given transformation. Used in registration loops.
          * \param[in] transform the transform to apply over the source cloud
          */
        virtual bool
        updateSource (const Eigen::Matrix<Scalar, 4, 4> &transform)
        {
          if (!input_)
          {
            PCL_ERROR ("[pcl::registration::%s::updateSource] No input dataset given. Please specify the input source cloud using setInputSource.\n", getClassName ().c_str ());
            return (false);
          }

          // Check if XYZ or normal data is available
          int x_idx = -1, nx_idx = -1;

          for (int i = 0; i < int (input_fields_.size ()); ++i)
          {
            if (input_fields_[i].name == "x")
              x_idx = i;
            if (input_fields_[i].name == "normal_x")
              nx_idx = i;
          }

          // If no XYZ available, then return
          if (x_idx == -1)
            return (true);

          input_transformed_.reset (new PointCloudSource (*input_));
         
          int y_idx = x_idx + 1, z_idx = x_idx + 2, ny_idx = nx_idx + 1, nz_idx = nx_idx + 2;
          Eigen::Vector4f pt (0.0f, 0.0f, 0.0f, 1.0f), pt_t;
          Eigen::Matrix4f tr = transform.template cast<float> ();

          if (nx_idx != -1)
          {
            Eigen::Vector3f nt, nt_t;
            Eigen::Matrix3f rot = tr.block<3, 3> (0, 0);

            //pcl::transformPointCloudWithNormals<PointSource, Scalar> (*input_, *input_transformed_, transform);
            for (size_t i = 0; i < input_transformed_->size (); ++i)
            {
              uint8_t* pt_data = reinterpret_cast<uint8_t*> (&input_transformed_->points[i]);
              memcpy (&pt[0], pt_data + input_fields_[x_idx].offset, sizeof (float));
              memcpy (&pt[1], pt_data + input_fields_[y_idx].offset, sizeof (float));
              memcpy (&pt[2], pt_data + input_fields_[z_idx].offset, sizeof (float));

              if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2])) 
                continue;

              pt_t = tr * pt;

              memcpy (pt_data + input_fields_[x_idx].offset, &pt_t[0], sizeof (float));
              memcpy (pt_data + input_fields_[y_idx].offset, &pt_t[1], sizeof (float));
              memcpy (pt_data + input_fields_[z_idx].offset, &pt_t[2], sizeof (float));

              memcpy (&nt[0], pt_data + input_fields_[nx_idx].offset, sizeof (float));
              memcpy (&nt[1], pt_data + input_fields_[ny_idx].offset, sizeof (float));
              memcpy (&nt[2], pt_data + input_fields_[nz_idx].offset, sizeof (float));

              if (!pcl_isfinite (nt[0]) || !pcl_isfinite (nt[1]) || !pcl_isfinite (nt[2])) 
                continue;

              nt_t = rot * nt;

              memcpy (pt_data + input_fields_[nx_idx].offset, &nt_t[0], sizeof (float));
              memcpy (pt_data + input_fields_[ny_idx].offset, &nt_t[1], sizeof (float));
              memcpy (pt_data + input_fields_[nz_idx].offset, &nt_t[2], sizeof (float));
            }
          }
          else
          {
            //pcl::transformPointCloud<PointSource, Scalar> (*input_, *input_transformed_, transform);
            for (size_t i = 0; i < input_transformed_->size (); ++i)
            {
              uint8_t* pt_data = reinterpret_cast<uint8_t*> (&input_transformed_->points[i]);
              memcpy (&pt[0], pt_data + input_fields_[x_idx].offset, sizeof (float));
              memcpy (&pt[1], pt_data + input_fields_[y_idx].offset, sizeof (float));
              memcpy (&pt[2], pt_data + input_fields_[z_idx].offset, sizeof (float));

              if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2])) 
                continue;

              pt_t = tr * pt;

              memcpy (pt_data + input_fields_[x_idx].offset, &pt_t[0], sizeof (float));
              memcpy (pt_data + input_fields_[y_idx].offset, &pt_t[1], sizeof (float));
              memcpy (pt_data + input_fields_[z_idx].offset, &pt_t[2], sizeof (float));
            }
          }
          
          input_ = input_transformed_;
          return (true);
        }
     };
  }
}

#include <pcl/registration/impl/correspondence_estimation.hpp>

#endif /* PCL_REGISTRATION_CORRESPONDENCE_ESTIMATION_H_ */
