/*
 * estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/feature_wrapper/normal_estimator.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/susan.h>
#include <pcl/surface/mls.h>

#include <memory>

namespace pcl {

template <>
struct SIFTKeypointFieldSelector<PointXYZ> {
  inline float
  operator()(const PointXYZ& p) const
  {
    return p.z;
  }
};

} // namespace pcl

namespace pcl {
namespace rec_3d_framework {

template <typename PointInT>
class KeypointExtractor {
protected:
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using PointOutTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  typename pcl::PointCloud<PointInT>::Ptr input_;
  float radius_;

public:
  virtual ~KeypointExtractor() = default;

  void
  setInputCloud(PointInTPtr& input)
  {
    input_ = input;
  }

  void
  setSupportRadius(float f)
  {
    radius_ = f;
  }

  virtual void
  compute(PointOutTPtr& keypoints) = 0;

  virtual void
  setNormals(const pcl::PointCloud<pcl::Normal>::Ptr& /*normals*/)
  {}

  virtual bool
  needNormals()
  {
    return false;
  }
};

template <typename PointInT>
class UniformSamplingExtractor : public KeypointExtractor<PointInT> {
private:
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  bool filter_planar_;
  using KeypointExtractor<PointInT>::input_;
  using KeypointExtractor<PointInT>::radius_;
  float sampling_density_;
  std::shared_ptr<std::vector<pcl::Indices>> neighborhood_indices_;
  std::shared_ptr<std::vector<std::vector<float>>> neighborhood_dist_;

  void
  filterPlanar(PointInTPtr& input, PointInTPtr& keypoints_cloud)
  {
    pcl::PointCloud<int> filtered_keypoints;
    // create a search object
    typename pcl::search::Search<PointInT>::Ptr tree;
    if (input->isOrganized())
      tree.reset(new pcl::search::OrganizedNeighbor<PointInT>());
    else
      tree.reset(new pcl::search::KdTree<PointInT>(false));
    tree->setInputCloud(input);

    neighborhood_indices_.reset(new std::vector<pcl::Indices>);
    neighborhood_indices_->resize(keypoints_cloud->size());
    neighborhood_dist_.reset(new std::vector<std::vector<float>>);
    neighborhood_dist_->resize(keypoints_cloud->size());

    filtered_keypoints.resize(keypoints_cloud->size());
    int good = 0;

    for (std::size_t i = 0; i < keypoints_cloud->size(); i++) {

      if (tree->radiusSearch((*keypoints_cloud)[i],
                             radius_,
                             (*neighborhood_indices_)[good],
                             (*neighborhood_dist_)[good])) {

        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        Eigen::Vector4f xyz_centroid;
        EIGEN_ALIGN16 Eigen::Vector3f eigenValues;
        EIGEN_ALIGN16 Eigen::Matrix3f eigenVectors;

        // compute planarity of the region
        computeMeanAndCovarianceMatrix(
            *input, (*neighborhood_indices_)[good], covariance_matrix, xyz_centroid);
        pcl::eigen33(covariance_matrix, eigenVectors, eigenValues);

        float eigsum = eigenValues.sum();
        if (!std::isfinite(eigsum)) {
          PCL_ERROR("Eigen sum is not finite\n");
        }

        if ((std::abs(eigenValues[0] - eigenValues[1]) < 1.5e-4) ||
            (eigsum != 0 && std::abs(eigenValues[0] / eigsum) > 1.e-2)) {
          // region is not planar, add to filtered keypoint
          (*keypoints_cloud)[good] = (*keypoints_cloud)[i];
          good++;
        }
      }
    }

    neighborhood_indices_->resize(good);
    neighborhood_dist_->resize(good);
    keypoints_cloud->points.resize(good);

    neighborhood_indices_->clear();
    neighborhood_dist_->clear();
  }

public:
  void
  setFilterPlanar(bool b)
  {
    filter_planar_ = b;
  }

  void
  setSamplingDensity(float f)
  {
    sampling_density_ = f;
  }

  void
  compute(PointInTPtr& keypoints) override
  {
    keypoints.reset(new pcl::PointCloud<PointInT>);

    pcl::UniformSampling<PointInT> keypoint_extractor;
    keypoint_extractor.setRadiusSearch(sampling_density_);
    keypoint_extractor.setInputCloud(input_);

    keypoint_extractor.filter(*keypoints);

    if (filter_planar_)
      filterPlanar(input_, keypoints);
  }
};

template <typename PointInT>
class SIFTKeypointExtractor : public KeypointExtractor<PointInT> {
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using KeypointExtractor<PointInT>::input_;
  using KeypointExtractor<PointInT>::radius_;

public:
  void
  compute(PointInTPtr& keypoints)
  {
    keypoints.reset(new pcl::PointCloud<PointInT>);

    typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::SIFTKeypoint<PointInT, pcl::PointXYZI> sift3D;
    sift3D.setScales(0.003f, 3, 2);
    sift3D.setMinimumContrast(0.1f);
    sift3D.setInputCloud(input_);
    sift3D.setSearchSurface(input_);
    sift3D.compute(*intensity_keypoints);
    pcl::copyPointCloud(*intensity_keypoints, *keypoints);
  }
};

template <typename PointInT>
class SIFTSurfaceKeypointExtractor : public KeypointExtractor<PointInT> {
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  using KeypointExtractor<PointInT>::input_;
  using KeypointExtractor<PointInT>::radius_;

  bool
  needNormals()
  {
    return true;
  }

  void
  setNormals(const pcl::PointCloud<pcl::Normal>::Ptr& normals)
  {
    normals_ = normals;
  }

public:
  void
  compute(PointInTPtr& keypoints)
  {
    if (normals_ == nullptr || (normals_->size() != input_->size()))
      PCL_WARN("SIFTSurfaceKeypointExtractor -- Normals are not valid\n");

    keypoints.reset(new pcl::PointCloud<PointInT>);

    typename pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointNormal>);
    input_cloud->width = input_->width;
    input_cloud->height = input_->height;
    input_cloud->points.resize(input_->width * input_->height);
    for (std::size_t i = 0; i < input_->points.size(); i++) {
      (*input_cloud)[i].getVector3fMap() = (*input_)[i].getVector3fMap();
      (*input_cloud)[i].getNormalVector3fMap() = (*normals_)[i].getNormalVector3fMap();
    }

    typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointXYZI> sift3D;
    sift3D.setScales(0.003f, 3, 2);
    sift3D.setMinimumContrast(0.0);
    sift3D.setInputCloud(input_cloud);
    sift3D.setSearchSurface(input_cloud);
    sift3D.compute(*intensity_keypoints);
    pcl::copyPointCloud(*intensity_keypoints, *keypoints);
  }
};

template <typename PointInT, typename NormalT = pcl::Normal>
class HarrisKeypointExtractor : public KeypointExtractor<PointInT> {

  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using KeypointExtractor<PointInT>::input_;
  using KeypointExtractor<PointInT>::radius_;
  typename pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI>::ResponseMethod m_;
  float non_max_radius_;
  float threshold_;

public:
  HarrisKeypointExtractor()
  {
    m_ = pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI>::HARRIS;
    non_max_radius_ = 0.01f;
    threshold_ = 0.f;
  }

  bool
  needNormals()
  {
    return true;
  }

  void
  setNormals(const pcl::PointCloud<pcl::Normal>::Ptr& normals)
  {
    normals_ = normals;
  }

  void
  setThreshold(float t)
  {
    threshold_ = t;
  }

  void
  setResponseMethod(
      typename pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI>::ResponseMethod m)
  {
    m_ = m;
  }

  void
  setNonMaximaRadius(float r)
  {
    non_max_radius_ = r;
  }

  void
  compute(PointInTPtr& keypoints)
  {
    keypoints.reset(new pcl::PointCloud<PointInT>);

    if (normals_ == nullptr || (normals_->size() != input_->size()))
      PCL_WARN("HarrisKeypointExtractor -- Normals are not valid\n");

    typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::HarrisKeypoint3D<PointInT, pcl::PointXYZI> harris;
    harris.setNonMaxSupression(true);
    harris.setRefine(false);
    harris.setThreshold(threshold_);
    harris.setInputCloud(input_);
    harris.setNormals(normals_);
    harris.setRadius(non_max_radius_);
    harris.setRadiusSearch(non_max_radius_);
    harris.setMethod(m_);
    harris.compute(*intensity_keypoints);

    pcl::copyPointCloud(*intensity_keypoints, *keypoints);
  }
};

template <typename PointInT, typename NormalT = pcl::Normal>
class SUSANKeypointExtractor : public KeypointExtractor<PointInT> {

  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using KeypointExtractor<PointInT>::input_;
  using KeypointExtractor<PointInT>::radius_;

public:
  SUSANKeypointExtractor() = default;

  bool
  needNormals()
  {
    return true;
  }

  void
  setNormals(const pcl::PointCloud<pcl::Normal>::Ptr& normals)
  {
    normals_ = normals;
  }

  void
  compute(PointInTPtr& keypoints)
  {
    keypoints.reset(new pcl::PointCloud<PointInT>);

    if (normals_ == nullptr || (normals_->size() != input_->size()))
      PCL_WARN("SUSANKeypointExtractor -- Normals are not valid\n");

    typename pcl::PointCloud<pcl::PointXYZI>::Ptr intensity_keypoints(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::SUSANKeypoint<PointInT, pcl::PointXYZI> susan;
    susan.setNonMaxSupression(true);
    susan.setInputCloud(input_);
    susan.setNormals(normals_);
    susan.setRadius(0.01f);
    susan.setRadiusSearch(0.01f);
    susan.compute(*intensity_keypoints);

    pcl::copyPointCloud(*intensity_keypoints, *keypoints);
  }
};

template <typename PointInT, typename FeatureT>
class LocalEstimator {
protected:
  using PointInTPtr = typename pcl::PointCloud<PointInT>::Ptr;
  using FeatureTPtr = typename pcl::PointCloud<FeatureT>::Ptr;

  std::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal>>
      normal_estimator_;
  std::vector<std::shared_ptr<KeypointExtractor<PointInT>>> keypoint_extractor_;
  float support_radius_;

  bool adaptative_MLS_;

  std::shared_ptr<std::vector<std::vector<int>>> neighborhood_indices_;
  std::shared_ptr<std::vector<std::vector<float>>> neighborhood_dist_;

  void
  computeKeypoints(PointInTPtr& cloud,
                   PointInTPtr& keypoints,
                   pcl::PointCloud<pcl::Normal>::Ptr& normals)
  {
    keypoints.reset(new pcl::PointCloud<PointInT>);
    for (std::size_t i = 0; i < keypoint_extractor_.size(); i++) {
      keypoint_extractor_[i]->setInputCloud(cloud);
      if (keypoint_extractor_[i]->needNormals())
        keypoint_extractor_[i]->setNormals(normals);

      keypoint_extractor_[i]->setSupportRadius(support_radius_);

      PointInTPtr detected_keypoints;
      keypoint_extractor_[i]->compute(detected_keypoints);
      *keypoints += *detected_keypoints;
    }
  }

public:
  LocalEstimator()
  {
    adaptative_MLS_ = false;
    keypoint_extractor_.clear();
  }

  virtual ~LocalEstimator() = default;

  void
  setAdaptativeMLS(bool b)
  {
    adaptative_MLS_ = b;
  }

  virtual bool
  estimate(PointInTPtr& in,
           PointInTPtr& processed,
           PointInTPtr& keypoints,
           FeatureTPtr& signatures) = 0;

  void
  setNormalEstimator(
      std::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal>>& ne)
  {
    normal_estimator_ = ne;
  }

  /**
   * \brief Right now only uniformSampling keypoint extractor is allowed
   */
  void
  addKeypointExtractor(std::shared_ptr<KeypointExtractor<PointInT>>& ke)
  {
    keypoint_extractor_.push_back(ke);
  }

  void
  setKeypointExtractors(std::vector<std::shared_ptr<KeypointExtractor<PointInT>>>& ke)
  {
    keypoint_extractor_ = ke;
  }

  void
  setSupportRadius(float r)
  {
    support_radius_ = r;
  }
};

} // namespace rec_3d_framework
} // namespace pcl
