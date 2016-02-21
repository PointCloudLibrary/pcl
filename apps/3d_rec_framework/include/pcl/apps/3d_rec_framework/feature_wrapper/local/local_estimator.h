/*
 * estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_LOCAL_ESTIMATOR_H_
#define REC_FRAMEWORK_LOCAL_ESTIMATOR_H_

#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/susan.h>

namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ & p) const
      {
        return p.z;
      }
    };
}

namespace pcl
{
  namespace rec_3d_framework
  {

    template<typename PointInT>
      class KeypointExtractor
      {
      protected:
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<PointInT>::Ptr PointOutTPtr;
        typename pcl::PointCloud<PointInT>::Ptr input_;
        float radius_;

      public:
        void
        setInputCloud (PointInTPtr & input)
        {
          input_ = input;
        }

        void
        setSupportRadius (float f)
        {
          radius_ = f;
        }

        virtual void
        compute (PointOutTPtr & keypoints) = 0;

        virtual void
        setNormals (const pcl::PointCloud<pcl::Normal>::Ptr & /*normals*/)
        {

        }

        virtual bool
        needNormals ()
        {
          return false;
        }

      };

    template<typename PointInT>
      class UniformSamplingExtractor : public KeypointExtractor<PointInT>
      {
      private:
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        bool filter_planar_;
        using KeypointExtractor<PointInT>::input_;
        using KeypointExtractor<PointInT>::radius_;
        float sampling_density_;
        boost::shared_ptr<std::vector<std::vector<int> > > neighborhood_indices_;
        boost::shared_ptr<std::vector<std::vector<float> > > neighborhood_dist_;

        void
        filterPlanar (PointInTPtr & input, PointInTPtr & keypoints_cloud)
        {
          pcl::PointCloud<int> filtered_keypoints;
          //create a search object
          typename pcl::search::Search<PointInT>::Ptr tree;
          if (input->isOrganized ())
            tree.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
          else
            tree.reset (new pcl::search::KdTree<PointInT> (false));
          tree->setInputCloud (input);

          neighborhood_indices_.reset (new std::vector<std::vector<int> >);
          neighborhood_indices_->resize (keypoints_cloud->points.size ());
          neighborhood_dist_.reset (new std::vector<std::vector<float> >);
          neighborhood_dist_->resize (keypoints_cloud->points.size ());

          filtered_keypoints.points.resize (keypoints_cloud->points.size ());
          int good = 0;

          for (size_t i = 0; i < keypoints_cloud->points.size (); i++)
          {

            if (tree->radiusSearch (keypoints_cloud->points[i], radius_, (*neighborhood_indices_)[good], (*neighborhood_dist_)[good]))
            {

              EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
              Eigen::Vector4f xyz_centroid;
              EIGEN_ALIGN16 Eigen::Vector3f eigenValues;
              EIGEN_ALIGN16 Eigen::Matrix3f eigenVectors;

              //compute planarity of the region
              computeMeanAndCovarianceMatrix (*input, (*neighborhood_indices_)[good], covariance_matrix, xyz_centroid);
              pcl::eigen33 (covariance_matrix, eigenVectors, eigenValues);

              float eigsum = eigenValues.sum ();
              if (!pcl_isfinite(eigsum))
              {
                PCL_ERROR("Eigen sum is not finite\n");
              }

              if ((fabs (eigenValues[0] - eigenValues[1]) < 1.5e-4) || (eigsum != 0 && fabs (eigenValues[0] / eigsum) > 1.e-2))
              {
                //region is not planar, add to filtered keypoint
                keypoints_cloud->points[good] = keypoints_cloud->points[i];
                good++;
              }
            }
          }

          neighborhood_indices_->resize (good);
          neighborhood_dist_->resize (good);
          keypoints_cloud->points.resize (good);

          neighborhood_indices_->clear ();
          neighborhood_dist_->clear ();

        }

      public:

        void
        setFilterPlanar (bool b)
        {
          filter_planar_ = b;
        }

        void
        setSamplingDensity (float f)
        {
          sampling_density_ = f;
        }

        void
        compute (PointInTPtr & keypoints)
        {
          keypoints.reset (new pcl::PointCloud<PointInT>);

          pcl::UniformSampling<PointInT> keypoint_extractor;
          keypoint_extractor.setRadiusSearch (sampling_density_);
          keypoint_extractor.setInputCloud (input_);

          keypoint_extractor.filter (*keypoints);

          if (filter_planar_)
            filterPlanar (input_, keypoints);
        }
      };

    template<typename PointInT>
      class SIFTKeypointExtractor : public KeypointExtractor<PointInT>
      {
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        using KeypointExtractor<PointInT>::input_;
        using KeypointExtractor<PointInT>::radius_;

      public:
        void
        compute (PointInTPtr & keypoints)
        {
          keypoints.reset (new pcl::PointCloud<PointInT>);

          typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
          pcl::SIFTKeypoint<PointInT, pcl::PointXYZI> sift3D;
          sift3D.setScales (0.003f, 3, 2);
          sift3D.setMinimumContrast (0.1f);
          sift3D.setInputCloud (input_);
          sift3D.setSearchSurface (input_);
          sift3D.compute (*intensity_keypoints);
          pcl::copyPointCloud (*intensity_keypoints, *keypoints);
        }
      };

    template<typename PointInT>
      class SIFTSurfaceKeypointExtractor : public KeypointExtractor<PointInT>
      {
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        using KeypointExtractor<PointInT>::input_;
        using KeypointExtractor<PointInT>::radius_;

        bool
        needNormals ()
        {
          return true;
        }

        void
        setNormals (const pcl::PointCloud<pcl::Normal>::Ptr & normals)
        {
          normals_ = normals;
        }

      public:
        void
        compute (PointInTPtr & keypoints)
        {
          if (normals_ == 0 || (normals_->points.size () != input_->points.size ()))
            PCL_WARN("SIFTSurfaceKeypointExtractor -- Normals are not valid\n");

          keypoints.reset (new pcl::PointCloud<PointInT>);

          typename pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointNormal>);
          input_cloud->width = input_->width;
          input_cloud->height = input_->height;
          input_cloud->points.resize (input_->width * input_->height);
          for (size_t i = 0; i < input_->points.size (); i++)
          {
            input_cloud->points[i].getVector3fMap () = input_->points[i].getVector3fMap ();
            input_cloud->points[i].getNormalVector3fMap () = normals_->points[i].getNormalVector3fMap ();
          }

          typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints (new pcl::PointCloud<pcl::PointXYZI>);
          pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointXYZI> sift3D;
          sift3D.setScales (0.003f, 3, 2);
          sift3D.setMinimumContrast (0.0);
          sift3D.setInputCloud (input_cloud);
          sift3D.setSearchSurface (input_cloud);
          sift3D.compute (*intensity_keypoints);
          pcl::copyPointCloud (*intensity_keypoints, *keypoints);
        }
      };

    template<typename PointInT, typename NormalT = pcl::Normal>
      class HarrisKeypointExtractor : public KeypointExtractor<PointInT>
      {

        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        using KeypointExtractor<PointInT>::input_;
        using KeypointExtractor<PointInT>::radius_;
        typename pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI>::ResponseMethod m_;
        float non_max_radius_;
        float threshold_;

      public:

        HarrisKeypointExtractor ()
        {
          m_ = pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI>::HARRIS;
          non_max_radius_ = 0.01f;
          threshold_ = 0.f;
        }

        bool
        needNormals ()
        {
          return true;
        }

        void
        setNormals (const pcl::PointCloud<pcl::Normal>::Ptr & normals)
        {
          normals_ = normals;
        }

        void
        setThreshold(float t) {
          threshold_ = t;
        }

        void
        setResponseMethod (typename pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI>::ResponseMethod m)
        {
          m_ = m;
        }

        void
        setNonMaximaRadius(float r) {
          non_max_radius_ = r;
        }

        void
        compute (PointInTPtr & keypoints)
        {
          keypoints.reset (new pcl::PointCloud<PointInT>);

          if (normals_ == 0 || (normals_->points.size () != input_->points.size ()))
            PCL_WARN("HarrisKeypointExtractor -- Normals are not valid\n");

          typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints (new pcl::PointCloud<pcl::PointXYZI>);

          pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI> harris;
          harris.setNonMaxSupression (true);
          harris.setRefine (false);
          harris.setThreshold (threshold_);
          harris.setInputCloud (input_);
          harris.setNormals (normals_);
          harris.setRadius (non_max_radius_);
          harris.setRadiusSearch (non_max_radius_);
          harris.setMethod (m_);
          harris.compute (*intensity_keypoints);

          pcl::copyPointCloud (*intensity_keypoints, *keypoints);
        }
      };

    template<typename PointInT, typename NormalT = pcl::Normal>
      class SUSANKeypointExtractor : public KeypointExtractor<PointInT>
      {

        pcl::PointCloud<pcl::Normal>::Ptr normals_;
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        using KeypointExtractor<PointInT>::input_;
        using KeypointExtractor<PointInT>::radius_;

      public:

        SUSANKeypointExtractor ()
        {

        }

        bool
        needNormals ()
        {
          return true;
        }

        void
        setNormals (const pcl::PointCloud<pcl::Normal>::Ptr & normals)
        {
          normals_ = normals;
        }

        void
        compute (PointInTPtr & keypoints)
        {
          keypoints.reset (new pcl::PointCloud<PointInT>);

          if (normals_ == 0 || (normals_->points.size () != input_->points.size ()))
            PCL_WARN("SUSANKeypointExtractor -- Normals are not valid\n");

          typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints (new pcl::PointCloud<pcl::PointXYZI>);

          pcl::SUSANKeypoint<PointInT, pcl::PointXYZI> susan;
          susan.setNonMaxSupression (true);
          susan.setInputCloud (input_);
          susan.setNormals (normals_);
          susan.setRadius (0.01f);
          susan.setRadiusSearch (0.01f);
          susan.compute (*intensity_keypoints);

          pcl::copyPointCloud (*intensity_keypoints, *keypoints);
        }
      };

    template<typename PointInT, typename FeatureT>
      class LocalEstimator
      {
      protected:
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;

        typename boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > normal_estimator_;
        //typename boost::shared_ptr<UniformSampling<PointInT> > keypoint_extractor_;
        std::vector<typename boost::shared_ptr<KeypointExtractor<PointInT> > > keypoint_extractor_; //this should be a vector
        float support_radius_;
        //bool filter_planar_;

        bool adaptative_MLS_;

        boost::shared_ptr<std::vector<std::vector<int> > > neighborhood_indices_;
        boost::shared_ptr<std::vector<std::vector<float> > > neighborhood_dist_;

        //std::vector< std::vector<int> > neighborhood_indices_;
        //std::vector< std::vector<float> > neighborhood_dist_;

        void
        computeKeypoints (PointInTPtr & cloud, PointInTPtr & keypoints, pcl::PointCloud<pcl::Normal>::Ptr & normals)
        {
          keypoints.reset (new pcl::PointCloud<PointInT>);
          for (size_t i = 0; i < keypoint_extractor_.size (); i++)
          {
            keypoint_extractor_[i]->setInputCloud (cloud);
            if (keypoint_extractor_[i]->needNormals ())
              keypoint_extractor_[i]->setNormals (normals);

            keypoint_extractor_[i]->setSupportRadius (support_radius_);

            PointInTPtr detected_keypoints;
            keypoint_extractor_[i]->compute (detected_keypoints);
            *keypoints += *detected_keypoints;
          }
        }

      public:

        LocalEstimator ()
        {
          adaptative_MLS_ = false;
          keypoint_extractor_.clear ();
        }

        void
        setAdaptativeMLS (bool b)
        {
          adaptative_MLS_ = b;
        }

        virtual bool
        estimate (PointInTPtr & in, PointInTPtr & processed, PointInTPtr & keypoints, FeatureTPtr & signatures)=0;

        void
        setNormalEstimator (boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > & ne)
        {
          normal_estimator_ = ne;
        }

        /**
         * \brief Right now only uniformSampling keypoint extractor is allowed
         */
        void
        addKeypointExtractor (boost::shared_ptr<KeypointExtractor<PointInT> > & ke)
        {
          keypoint_extractor_.push_back (ke);
        }

        void
        setKeypointExtractors (std::vector<typename boost::shared_ptr<KeypointExtractor<PointInT> > > & ke)
        {
          keypoint_extractor_ = ke;
        }

        void
        setSupportRadius (float r)
        {
          support_radius_ = r;
        }

        /*void
         setFilterPlanar (bool b)
         {
         filter_planar_ = b;
         }

         void
         filterPlanar (PointInTPtr & input, KeypointCloud & keypoints_cloud)
         {
         pcl::PointCloud<int> filtered_keypoints;
         //create a search object
         typename pcl::search::Search<PointInT>::Ptr tree;
         if (input->isOrganized ())
         tree.reset (new pcl::search::OrganizedNeighbor<PointInT> ());
         else
         tree.reset (new pcl::search::KdTree<PointInT> (false));
         tree->setInputCloud (input);

         //std::vector<int> nn_indices;
         //std::vector<float> nn_distances;

         neighborhood_indices_.reset (new std::vector<std::vector<int> >);
         neighborhood_indices_->resize (keypoints_cloud.points.size ());
         neighborhood_dist_.reset (new std::vector<std::vector<float> >);
         neighborhood_dist_->resize (keypoints_cloud.points.size ());

         filtered_keypoints.points.resize (keypoints_cloud.points.size ());
         int good = 0;

         //#pragma omp parallel for num_threads(8)
         for (size_t i = 0; i < keypoints_cloud.points.size (); i++)
         {

         if (tree->radiusSearch (keypoints_cloud[i], support_radius_, (*neighborhood_indices_)[good], (*neighborhood_dist_)[good]))
         {

         EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
         Eigen::Vector4f xyz_centroid;
         EIGEN_ALIGN16 Eigen::Vector3f eigenValues;
         EIGEN_ALIGN16 Eigen::Matrix3f eigenVectors;

         //compute planarity of the region
         computeMeanAndCovarianceMatrix (*input, (*neighborhood_indices_)[good], covariance_matrix, xyz_centroid);
         pcl::eigen33 (covariance_matrix, eigenVectors, eigenValues);

         float eigsum = eigenValues.sum ();
         if (!pcl_isfinite(eigsum))
         {
         PCL_ERROR("Eigen sum is not finite\n");
         }

         if ((fabs (eigenValues[0] - eigenValues[1]) < 1.5e-4) || (eigsum != 0 && fabs (eigenValues[0] / eigsum) > 1.e-2))
         {
         //region is not planar, add to filtered keypoint
         keypoints_cloud.points[good] = keypoints_cloud.points[i];
         good++;
         }
         }
         }

         neighborhood_indices_->resize (good);
         neighborhood_dist_->resize (good);
         keypoints_cloud.points.resize (good);
         }*/

      };
  }
}

#endif /* REC_FRAMEWORK_LOCAL_ESTIMATOR_H_ */
