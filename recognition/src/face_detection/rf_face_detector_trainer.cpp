/*
 * rf_face_detector_trainer.cpp
 *
 *  Created on: 22 Sep 2012
 *      Author: ari
 */

#include "pcl/recognition/face_detection/rf_face_detector_trainer.h"
#include "pcl/recognition/face_detection/face_common.h"
#include "pcl/io/pcd_io.h"
#include "pcl/ml/dt/decision_tree_trainer.h"
#include "pcl/ml/dt/decision_tree_evaluator.h"
#include "pcl/ml/dt/decision_forest_trainer.h"
#include "pcl/ml/dt/decision_forest_evaluator.h"
#include "pcl/filters/passthrough.h"
#include <pcl/features/integral_image_normal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include "pcl/filters/voxel_grid.h"
#include <pcl/recognition/hv/hv_papazov.h>
#include <pcl/features/normal_3d.h>

void pcl::RFFaceDetectorTrainer::trainWithDataProvider()
{

  face_detection::FeatureHandlerDepthAverage<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, int> fhda;
  fhda.setWSize (w_size_);
  fhda.setMaxPatchSize (max_patch_size_);
  fhda.setNumChannels (1);
  if (use_normals_)
    fhda.setNumChannels (4);

  pcl::TernaryTreeMissingDataBranchEstimator * btt = new pcl::TernaryTreeMissingDataBranchEstimator ();
  pcl::face_detection::PoseClassRegressionVarianceStatsEstimator<float, NodeType, std::vector<face_detection::TrainingExample>, int> rse (btt);

  std::vector<float> thresholds_;
  thresholds_.push_back (0.5f);

  pcl::DecisionForestTrainer<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, float, int, NodeType> dft;
  dft.setMaxTreeDepth (15);
  dft.setNumOfFeatures (nfeatures_);
  dft.setNumOfThresholds (1);
  dft.setNumberOfTreesToTrain (ntrees_);
  dft.setMinExamplesForSplit (20);
  dft.setFeatureHandler (fhda);
  dft.setStatsEstimator (rse);
  dft.setRandomFeaturesAtSplitNode (true);
  dft.setThresholds (thresholds_);

  boost::shared_ptr < face_detection::FaceDetectorDataProvider<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, float, int, NodeType>
      > dtdp;
  dtdp.reset (new face_detection::FaceDetectorDataProvider<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, float, int, NodeType>);
  dtdp->setUseNormals (use_normals_);
  dtdp->setWSize (w_size_);
  dtdp->setNumImages (num_images_);
  dtdp->setMinImagesPerBin (300);

  dtdp->initialize (directory_);

  boost::shared_ptr < pcl::DecisionTreeTrainerDataProvider<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, float, int, NodeType>
      > cast_dtdp;
  cast_dtdp = boost::dynamic_pointer_cast
      < pcl::DecisionTreeTrainerDataProvider<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, float, int, NodeType> > (dtdp);
  dft.setDecisionTreeDataProvider (cast_dtdp);

  pcl::DecisionForest<NodeType> forest;
  dft.train (forest);

  PCL_INFO("Finished training forest...\n");

  std::filebuf fb;
  fb.open (forest_filename_.c_str (), std::ios::out);
  std::ostream os (&fb);
  forest.serialize (os);
  fb.close ();
}

void pcl::RFFaceDetectorTrainer::faceVotesClustering()
{
  float HEAD_DIAMETER_SQ = HEAD_ST_DIAMETER_ * HEAD_ST_DIAMETER_;
  float large_radius = HEAD_DIAMETER_SQ / (larger_radius_ratio_ * larger_radius_ratio_);

  std::vector < Eigen::Vector3f > clusters_mean;
  std::vector < std::vector<int> > votes_indices;

  for (size_t i = 0; i < head_center_votes_.size (); i++)
  {
    Eigen::Vector3f center_vote = head_center_votes_[i];
    std::vector<bool> valid_in_cluster (clusters_mean.size (), false);
    bool found = false;
    for (size_t j = 0; j < clusters_mean.size () /*&& !found*/; j++)
    {
      float sq_norm = (clusters_mean[j] - center_vote).squaredNorm ();
      if (sq_norm < large_radius)
      {
        //found one cluster, update cluster mean and append index
        valid_in_cluster[j] = true;
        found = true;
      }
    }

    //no cluster found, create new cluster
    if (!found)
    {
      std::vector < int > ind;
      ind.push_back (static_cast<int>(i));
      votes_indices.push_back (ind);

      std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > votes_in_cluster;
      votes_in_cluster.push_back (center_vote);
      head_center_original_votes_clustered_.push_back (votes_in_cluster);

      clusters_mean.push_back (center_vote);
      continue;
    }

    //get the largest biggest cluster and put if there
    int idx = -1;
    size_t biggest_num = 0;
    for (size_t j = 0; j < clusters_mean.size () /*&& !found*/; j++)
    {
      if ((votes_indices[j].size () > biggest_num) && (valid_in_cluster[j]))
      {
        idx = static_cast<int>(j);
        biggest_num = votes_indices[j].size ();
      }
    }

    clusters_mean[idx] = (clusters_mean[idx] * (static_cast<float> (votes_indices[idx].size ())) + center_vote)
        / (static_cast<float> (votes_indices[idx].size ()) + 1.f);
    votes_indices[idx].push_back (static_cast<int>(i));
    head_center_original_votes_clustered_[idx].push_back (center_vote);
  }

  //mean shift
  //float SMALL_HEAD_RADIUS = HEAD_ST_DIAMETER_ / 6.f;
  float SMALL_HEAD_RADIUS_SQ = HEAD_ST_DIAMETER_ * HEAD_ST_DIAMETER_ / 36.f;
  int msi = 10;
  std::cout << "Number of clusters:" << clusters_mean.size () << " votes:" << head_center_votes_.size () << std::endl;

  int valid = 0;
  for (size_t i = 0; i < clusters_mean.size (); i++)
  {
    //ignore this cluster
    if (votes_indices[i].size () < min_votes_size_)
      continue;

    std::vector < int > new_cluster;

    for (int it = 0; it < msi; it++)
    {
      Eigen::Vector3f mean;
      mean.setZero ();
      int good_votes = 0;
      new_cluster.clear ();
      for (size_t j = 0; j < votes_indices[i].size (); j++)
      {
        Eigen::Vector3f center_vote = head_center_votes_[votes_indices[i][j]];
        float sq_norm = (clusters_mean[i] - center_vote).squaredNorm ();
        if (sq_norm < SMALL_HEAD_RADIUS_SQ)
        {
          mean += center_vote;
          new_cluster.push_back (votes_indices[i][j]);
          good_votes++;
        }
      }

      mean = mean / static_cast<float> (good_votes);
      clusters_mean[i] = mean;
    }

    clusters_mean[valid] = clusters_mean[i];
    votes_indices[valid] = new_cluster;
    valid++;
  }

  clusters_mean.resize (valid);
  votes_indices.resize (valid);

  std::cout << "Valid:" << valid << std::endl;

  head_clusters_centers_.clear ();
  head_clusters_rotation_.clear ();
  head_center_votes_clustered_.resize (clusters_mean.size ());

  for (size_t i = 0; i < clusters_mean.size (); i++)
  {
    if (votes_indices[i].size () > min_votes_size_)
    {
      //compute rotation using the first less uncertain votes
      std::vector < std::pair<int, float> > uncertainty;
      for (size_t j = 0; j < votes_indices[i].size (); j++)
      {
        uncertainty.push_back (std::make_pair (votes_indices[i][j], uncertainties_[votes_indices[i][j]]));
      }

      std::sort (uncertainty.begin (), uncertainty.end (), boost::bind (&std::pair<int, float>::second, _1) < boost::bind (&std::pair<int, float>::second, _2));

      Eigen::Vector3f rot;
      rot.setZero ();
      int num = std::min (used_for_pose_, static_cast<int> (uncertainty.size ()));
      for (int j = 0; j < num; j++)
      {
        rot += angle_votes_[uncertainty[j].first];
      }

      rot = rot / static_cast<float> (num);

      Eigen::Vector3f pos;
      pos.setZero ();
      for (int j = 0; j < num; j++)
        pos += head_center_votes_[uncertainty[j].first];

      pos = pos / static_cast<float> (num);

      head_clusters_centers_.push_back (pos); //clusters_mean[i]
      head_clusters_rotation_.push_back (rot);

      for (size_t j = 0; j < votes_indices[i].size (); j++)
      {
        head_center_votes_clustered_[i].push_back (head_center_votes_[votes_indices[i][j]]);
      }
    }
  }

  std::cout << "Number of heads:" << head_clusters_centers_.size () << std::endl;
}

void pcl::RFFaceDetectorTrainer::setModelPath(std::string & model)
{
  model_path_ = model;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::io::loadPCDFile (model_path_, *model_cloud);

  model_original_.reset (new pcl::PointCloud<pcl::PointXYZ> ());

  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_icp;
    voxel_grid_icp.setInputCloud (model_cloud);
    voxel_grid_icp.setLeafSize (res_, res_, res_);
    voxel_grid_icp.filter (*model_original_);

    pcl::PassThrough<pcl::PointXYZ> pass_;
    pass_.setFilterLimits (-1.f, 0.03f);
    pass_.setFilterFieldName ("z");
    pass_.setInputCloud (model_original_);
    pass_.filter (*model_original_);

    pass_.setFilterLimits (-0.1f, 0.07f);
    pass_.setFilterFieldName ("y");
    pass_.setInputCloud (model_original_);
    pass_.filter (*model_original_);
  }
}

void pcl::RFFaceDetectorTrainer::detectFaces()
{
  //clear stuff from last round
  head_center_votes_.clear ();
  head_center_original_votes_clustered_.clear ();
  head_center_votes_clustered_.clear ();
  angle_votes_.clear ();
  uncertainties_.clear ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass_;
  pass_.setFilterLimits (0.f, 1.25f);
  pass_.setFilterFieldName ("z");
  pass_.setInputCloud (input_);
  pass_.setKeepOrganized (true);
  pass_.filter (*cloud);

  //compute depth integral image
  boost::shared_ptr<pcl::IntegralImage2D<float, 1> > integral_image_depth;
  integral_image_depth.reset (new pcl::IntegralImage2D<float, 1> (false));

  int element_stride = sizeof(pcl::PointXYZ) / sizeof(float);
  int row_stride = element_stride * cloud->width;
  const float *data = reinterpret_cast<const float*> (&cloud->points[0]);
  integral_image_depth->setInput (data + 2, cloud->width, cloud->height, element_stride, row_stride);

  //Compute normals and normal integral images
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

  if (use_normals_)
  {
    typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> NormalEstimator_;
    NormalEstimator_ n3d;
    n3d.setNormalEstimationMethod (n3d.COVARIANCE_MATRIX);
    n3d.setInputCloud (cloud);
    n3d.setRadiusSearch (0.02);
    n3d.setKSearch (0);
    n3d.compute (*normals);
  }

  int element_stride_normal = sizeof(pcl::Normal) / sizeof(float);
  int row_stride_normal = element_stride_normal * normals->width;
  boost::shared_ptr<pcl::IntegralImage2D<float, 1> > integral_image_normal_x;
  boost::shared_ptr<pcl::IntegralImage2D<float, 1> > integral_image_normal_y;
  boost::shared_ptr<pcl::IntegralImage2D<float, 1> > integral_image_normal_z;

  if (use_normals_)
  {
    integral_image_normal_x.reset (new pcl::IntegralImage2D<float, 1> (false));
    const float *data_nx = reinterpret_cast<const float*> (&normals->points[0]);
    integral_image_normal_x->setInput (data_nx, normals->width, normals->height, element_stride_normal, row_stride_normal);

    integral_image_normal_y.reset (new pcl::IntegralImage2D<float, 1> (false));
    const float *data_ny = reinterpret_cast<const float*> (&normals->points[0]);
    integral_image_normal_y->setInput (data_ny + 1, normals->width, normals->height, element_stride_normal, row_stride_normal);

    integral_image_normal_z.reset (new pcl::IntegralImage2D<float, 1> (false));
    const float *data_nz = reinterpret_cast<const float*> (&normals->points[0]);
    integral_image_normal_z->setInput (data_nz + 2, normals->width, normals->height, element_stride_normal, row_stride_normal);
  }

  {
    //instantiate evaluator
    pcl::DecisionForestEvaluator<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, float, int, NodeType> dfe;
    face_detection::FeatureHandlerDepthAverage<face_detection::FeatureType, std::vector<face_detection::TrainingExample>, int> fhda;
    fhda.setWSize (w_size_);
    fhda.setNumChannels (1);
    if (use_normals_)
      fhda.setNumChannels (4);

    //pcl::BinaryTreeThresholdBasedBranchEstimator * btt = new pcl::BinaryTreeThresholdBasedBranchEstimator ();
    pcl::TernaryTreeMissingDataBranchEstimator * btt = new pcl::TernaryTreeMissingDataBranchEstimator ();
    face_detection::PoseClassRegressionVarianceStatsEstimator<float, NodeType, std::vector<face_detection::TrainingExample>, int> rse (btt);

    std::vector<float> weights;
    weights.resize (cloud->points.size ());
    for (size_t i = 0; i < cloud->points.size (); i++)
      weights[i] = 0;

    int w_size_2 = static_cast<int> (w_size_ / 2);

    //do sliding window
    for (int col = 0; col < (static_cast<int> (cloud->width) - w_size_); col += stride_sw_)
    {
      for (int row = 0; row < (static_cast<int> (cloud->height) - w_size_); row += stride_sw_)
      {

        if (!pcl::isFinite (cloud->at (col + w_size_2, row + w_size_2))) //reject patches with invalid center point
          continue;

        if (integral_image_depth->getFiniteElementsCount (col, row, w_size_, w_size_) > (0.1 * w_size_ * w_size_))
        {
          face_detection::TrainingExample te;
          //te.iimage_ = integral_image_depth;
          te.iimages_.push_back (integral_image_depth);
          if (use_normals_)
          {
            te.iimages_.push_back (integral_image_normal_x);
            te.iimages_.push_back (integral_image_normal_y);
            te.iimages_.push_back (integral_image_normal_z);
          }
          te.row_ = row;
          te.col_ = col;
          te.wsize_ = w_size_;

          std::vector<face_detection::TrainingExample> eval_examples;
          eval_examples.push_back (te);
          /*std::vector<int> example_indices;
           example_indices.push_back(0);*/

          //evaluate this patch through the trees
          std::vector<NodeType> leaves;
          dfe.evaluate (forest_, fhda, rse, eval_examples, 0, leaves);

          for (size_t l = 0; l < leaves.size (); l++)
          {
            if (leaves[l].value >= thres_face_)
            {
              if ((leaves[l].covariance_trans_.trace () + leaves[l].covariance_rot_.trace ()) > trans_max_variance_)
                continue;

              Eigen::Vector3f head_center = Eigen::Vector3f (static_cast<float>(leaves[l].trans_mean_[0]),
                                                             static_cast<float>(leaves[l].trans_mean_[1]),
                                                             static_cast<float>(leaves[l].trans_mean_[2]));
              head_center *= 0.001f;

              pcl::PointXYZ patch_center_point;
              patch_center_point.x = cloud->at (col + w_size_2, row + w_size_2).x;
              patch_center_point.y = cloud->at (col + w_size_2, row + w_size_2).y;
              patch_center_point.z = cloud->at (col + w_size_2, row + w_size_2).z;

              head_center = patch_center_point.getVector3fMap () + head_center;

              pcl::PointXYZ ppp;
              ppp.getVector3fMap () = head_center;
              if (!pcl::isFinite (ppp))
                continue;

              //this is a good leave
              for (int j = te.col_; j < (te.col_ + w_size_); j++)
              {
                for (int k = te.row_; k < (te.row_ + w_size_); k++)
                  weights[k * cloud->width + j]++;
              }

              head_center_votes_.push_back (head_center);
              float mult_fact = 0.0174532925f;
              angle_votes_.push_back (
                  Eigen::Vector3f (static_cast<float>(leaves[l].rot_mean_[0]) * mult_fact,
                                   static_cast<float>(leaves[l].rot_mean_[1]) * mult_fact,
                                   static_cast<float>(leaves[l].rot_mean_[2]) * mult_fact));
              uncertainties_.push_back (static_cast<float>(leaves[l].covariance_trans_.trace () + leaves[l].covariance_rot_.trace ()));
            }
          }
        }
      }
    }

    if (face_heat_map_)
    {
      face_heat_map_.reset (new pcl::PointCloud<pcl::PointXYZI>);
      face_heat_map_->resize (cloud->points.size ());
      face_heat_map_->height = 1;
      face_heat_map_->width = static_cast<unsigned int>(cloud->points.size ());
      face_heat_map_->is_dense = false;

      for (size_t i = 0; i < cloud->points.size (); i++)
      {
        face_heat_map_->points[i].getVector4fMap () = cloud->points[i].getVector4fMap ();
        face_heat_map_->points[i].intensity = weights[i];
      }
    }
  }

  faceVotesClustering ();

  if (pose_refinement_ && (head_clusters_centers_.size () > 0))
  {
    Eigen::Matrix4f icp_trans;
    float max_distance = 0.015f;
    int iter = icp_iterations_;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_voxelized (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::Normal>::Ptr scene_normals (new pcl::PointCloud<pcl::Normal> ());

    {
      typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> NormalEstimator_;
      NormalEstimator_ n3d;
      n3d.setNormalEstimationMethod (n3d.COVARIANCE_MATRIX);
      n3d.setInputCloud (input_);
      n3d.setRadiusSearch (0.f);
      n3d.setKSearch (10);
      n3d.compute (*scene_normals);
    }

    pcl::copyPointCloud (*input_, *cloud_voxelized);
    pcl::copyPointCloud (*scene_normals, *cloud_voxelized);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_voxelized_icp_normals (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::VoxelGrid<pcl::PointNormal> voxel_grid_icp;
    voxel_grid_icp.setInputCloud (cloud_voxelized);
    voxel_grid_icp.setDownsampleAllData (true);
    voxel_grid_icp.setLeafSize (res_, res_, res_);
    voxel_grid_icp.filter (*cloud_voxelized_icp_normals);

    //compute normals
    pcl::PointCloud<pcl::PointNormal>::Ptr model_aligned_normals (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::copyPointCloud (*model_original_, *model_aligned_normals);

    pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal> normal_est_;
    normal_est_.setKSearch (10);

    {
      normal_est_.setInputCloud (model_aligned_normals);
      normal_est_.compute (*model_aligned_normals);
    }

    //do pose refinement for the detected heads
    //std::vector<pcl::PointCloud<pcl::PointNormal>::ConstPtr> aligned_models_;
    pcl::PointCloud<pcl::PointNormal>::Ptr output (new pcl::PointCloud<pcl::PointNormal> ());

    pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> reg;
    for (size_t i = 0; i < head_clusters_centers_.size (); i++)
    {
      Eigen::Matrix3f matrixxx;

      matrixxx = Eigen::AngleAxisf (head_clusters_rotation_[i][0], Eigen::Vector3f::UnitX ())
          * Eigen::AngleAxisf (head_clusters_rotation_[i][1], Eigen::Vector3f::UnitY ())
          * Eigen::AngleAxisf (head_clusters_rotation_[i][2], Eigen::Vector3f::UnitZ ());

      Eigen::Matrix4f guess;
      guess.setIdentity ();
      guess.block<3, 3> (0, 0) = matrixxx;
      guess (0, 3) = head_clusters_centers_[i][0];
      guess (1, 3) = head_clusters_centers_[i][1];
      guess (2, 3) = head_clusters_centers_[i][2];

      pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>::Ptr trans_lls (
          new pcl::registration::TransformationEstimationPointToPlaneLLS<pcl::PointNormal, pcl::PointNormal>);

      pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>::Ptr cens (
          new pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointNormal, pcl::PointNormal, pcl::PointNormal>);

      cens->setInputSource (model_aligned_normals);
      cens->setInputTarget (cloud_voxelized_icp_normals);
      cens->setSourceNormals (model_aligned_normals);

      pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal>::Ptr rej (
          new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> ());

      rej->setInputSource (model_aligned_normals);
      rej->setInputTarget (cloud_voxelized_icp_normals);
      rej->setMaximumIterations (1000);
      rej->setInlierThreshold (0.01f);

      reg.addCorrespondenceRejector (rej);
      reg.setCorrespondenceEstimation (cens);
      reg.setTransformationEstimation (trans_lls);

      reg.setInputSource (model_aligned_normals); //model
      reg.setInputTarget (cloud_voxelized_icp_normals); //scene
      reg.setMaximumIterations (iter);
      reg.setMaxCorrespondenceDistance (max_distance);
      reg.setTransformationEpsilon (1e-12);
      reg.align (*output, guess);
      icp_trans = reg.getFinalTransformation ();

      //update values
      head_clusters_centers_[i][0] = icp_trans (0, 3);
      head_clusters_centers_[i][1] = icp_trans (1, 3);
      head_clusters_centers_[i][2] = icp_trans (2, 3);

      Eigen::Vector3f ea = icp_trans.block<3, 3> (0, 0).eulerAngles (0, 1, 2);
      head_clusters_rotation_[i][0] = ea[0];
      head_clusters_rotation_[i][1] = ea[1];
      head_clusters_rotation_[i][2] = ea[2];

    }

    //do HV
    /*pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ> papazov;
     papazov.setResolution (0.005f);
     papazov.setInlierThreshold (0.01f);
     papazov.setSupportThreshold (0.1f);
     papazov.setPenaltyThreshold (0.2f);
     papazov.setConflictThreshold (0.01f);

     std::vector<bool> mask_hv;
     papazov.setOcclusionCloud (input_);
     papazov.setSceneCloud (cloud_voxelized_icp);
     papazov.addModels (aligned_models_, true);
     papazov.verify ();
     papazov.getMask (mask_hv);

     size_t valid=0;
     for(size_t i=0; i < mask_hv.size(); i++) {
     if (!mask_hv[i])
     continue;

     if(valid < i) {
     head_clusters_centers_[valid] = head_clusters_centers_[i];
     head_clusters_rotation_[valid] = head_clusters_rotation_[i];
     }
     }

     std::cout << "Valid heads after HV:" << valid << " before was:" << mask_hv.size() << std::endl;
     head_clusters_centers_.resize(valid);
     head_clusters_rotation_.resize(valid);*/
  }
}

