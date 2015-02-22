/*
 * rf_face_detector_trainer.h
 *
 *  Created on: 22 Sep 2012
 *      Author: Aitor Aldoma
 */

#ifndef PCL_RF_FACE_DETECTOR_TRAINER_H_
#define PCL_RF_FACE_DETECTOR_TRAINER_H_

#include "pcl/recognition/face_detection/face_detector_data_provider.h"
#include "pcl/recognition/face_detection/rf_face_utils.h"
#include "pcl/ml/dt/decision_forest.h"
#include <pcl/features/integral_image2D.h>

namespace pcl
{
  class PCL_EXPORTS RFFaceDetectorTrainer
  {
    private:
      int w_size_;
      int max_patch_size_;
      int stride_sw_;
      int ntrees_;
      std::string forest_filename_;
      int nfeatures_;
      float thres_face_;
      int num_images_;
      float trans_max_variance_;
      size_t min_votes_size_;
      int used_for_pose_;
      bool use_normals_;
      std::string directory_;
      float HEAD_ST_DIAMETER_;
      float larger_radius_ratio_;
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > head_center_votes_;
      std::vector<std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > > head_center_votes_clustered_;
      std::vector<std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > > head_center_original_votes_clustered_;
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > angle_votes_;
      std::vector<float> uncertainties_;
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > head_clusters_centers_;
      std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > head_clusters_rotation_;

      pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
      pcl::PointCloud<pcl::PointXYZI>::Ptr face_heat_map_;

      typedef face_detection::RFTreeNode<face_detection::FeatureType> NodeType;
      pcl::DecisionForest<NodeType> forest_;

      std::string model_path_;
      bool pose_refinement_;
      int icp_iterations_;

      pcl::PointCloud<pcl::PointXYZ>::Ptr model_original_;
      float res_;

    public:

      RFFaceDetectorTrainer()
      {
        w_size_ = 80;
        max_patch_size_ = 40;
        stride_sw_ = 4;
        ntrees_ = 10;
        forest_filename_ = std::string ("forest.txt");
        nfeatures_ = 10000;
        thres_face_ = 1.f;
        num_images_ = 1000;
        trans_max_variance_ = 1600.f;
        used_for_pose_ = std::numeric_limits<int>::max ();
        use_normals_ = false;
        directory_ = std::string ("");
        HEAD_ST_DIAMETER_ = 0.2364f;
        larger_radius_ratio_ = 1.5f;
        face_heat_map_.reset ();
        model_path_ = std::string ("face_mesh.ply");
        pose_refinement_ = false;
        res_ = 0.005f;
      }

      virtual ~RFFaceDetectorTrainer()
      {

      }

      /*
       * Common parameters
       */
      void setForestFilename(std::string & ff)
      {
        forest_filename_ = ff;
      }

      void setUseNormals(bool use)
      {
        use_normals_ = use;
      }

      void setWSize(int s)
      {
        w_size_ = s;
      }

      /*
       * Training parameters
       */

      void setDirectory(std::string & dir)
      {
        directory_ = dir;
      }
      void setNumTrainingImages(int num)
      {
        num_images_ = num;
      }

      void setNumTrees(int num)
      {
        ntrees_ = num;
      }

      void setNumFeatures(int num)
      {
        nfeatures_ = num;
      }

      /*
       * Detection parameters
       */

      void setModelPath(std::string & model);

      void setPoseRefinement(bool do_it, int iters = 5)
      {
        pose_refinement_ = do_it;
        icp_iterations_ = iters;
      }

      void setLeavesFaceThreshold(float p)
      {
        thres_face_ = p;
      }

      void setLeavesFaceMaxVariance(float max)
      {
        trans_max_variance_ = max;
      }

      void setWStride(int s)
      {
        stride_sw_ = s;
      }

      void setFaceMinVotes(int mv)
      {
        min_votes_size_ = mv;
      }

      void setNumVotesUsedForPose(int n)
      {
        used_for_pose_ = n;
      }

      void setForest(pcl::DecisionForest<NodeType> & forest)
      {
        forest_ = forest;
      }

      /*
       * Get functions
       */

      void getFaceHeatMap(pcl::PointCloud<pcl::PointXYZI>::Ptr & heat_map)
      {
        heat_map = face_heat_map_;
      }

      //get votes
      void getVotes(pcl::PointCloud<pcl::PointXYZ>::Ptr & votes_cloud)
      {
        votes_cloud->points.resize (head_center_votes_.size ());
        votes_cloud->width = static_cast<int>(head_center_votes_.size ());
        votes_cloud->height = 1;

        for (size_t i = 0; i < head_center_votes_.size (); i++)
        {
          votes_cloud->points[i].getVector3fMap () = head_center_votes_[i];
        }
      }

      void getVotes(pcl::PointCloud<pcl::PointXYZI>::Ptr & votes_cloud)
      {
        votes_cloud->points.resize (head_center_votes_.size ());
        votes_cloud->width = static_cast<int>(head_center_votes_.size ());
        votes_cloud->height = 1;

        int p = 0;
        for (size_t i = 0; i < head_center_votes_clustered_.size (); i++)
        {
          for (size_t j = 0; j < head_center_votes_clustered_[i].size (); j++, p++)
          {
            votes_cloud->points[p].getVector3fMap () = head_center_votes_clustered_[i][j];
            votes_cloud->points[p].intensity = 0.1f * static_cast<float> (i);
          }
        }

        votes_cloud->points.resize (p);
      }

      void getVotes2(pcl::PointCloud<pcl::PointXYZI>::Ptr & votes_cloud)
      {
        votes_cloud->points.resize (head_center_votes_.size ());
        votes_cloud->width = static_cast<int>(head_center_votes_.size ());
        votes_cloud->height = 1;

        int p = 0;
        for (size_t i = 0; i < head_center_original_votes_clustered_.size (); i++)
        {
          for (size_t j = 0; j < head_center_original_votes_clustered_[i].size (); j++, p++)
          {
            votes_cloud->points[p].getVector3fMap () = head_center_original_votes_clustered_[i][j];
            votes_cloud->points[p].intensity = 0.1f * static_cast<float> (i);
          }
        }

        votes_cloud->points.resize (p);
      }

      //get heads
      void getDetectedFaces(std::vector<Eigen::VectorXf> & faces)
      {
        for (size_t i = 0; i < head_clusters_centers_.size (); i++)
        {
          Eigen::VectorXf head (6);
          head[0] = head_clusters_centers_[i][0];
          head[1] = head_clusters_centers_[i][1];
          head[2] = head_clusters_centers_[i][2];
          head[3] = head_clusters_rotation_[i][0];
          head[4] = head_clusters_rotation_[i][1];
          head[5] = head_clusters_rotation_[i][2];
          faces.push_back (head);
        }
      }
      /*
       * Other functions
       */
      void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
      {
        input_ = cloud;
      }

      void setFaceHeatMapCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr & heat_map)
      {
        face_heat_map_ = heat_map;
      }

      void trainWithDataProvider();
      void faceVotesClustering();
      void detectFaces();
  };
}

#endif /* PCL_RF_FACE_DETECTOR_TRAINER_H_ */
