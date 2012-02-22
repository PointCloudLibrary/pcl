#include <vector>
#include <map>
#include <cassert>

#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/distances.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_based_signature.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace pcl
{
  /** \brief FeatureCorrespondenceTest is the base class implementing the functionality for running Feature Correspondence tests.
    *
    * To test a Feature Descriptor algorithm, derive a separate class corresponding to that algorithm from this base class.
    * Implement following methods:
    * setParameters(ParameterList) Provide input parameters
    * computeFeatures() Compute feature descriptors
    * computeCorrespondences() Compute correspondences between source and target feature descriptors
    */
  template <typename PointIn>
  class FeatureCorrespondenceTest: public PCLBase<PointIn>
  {
  public:
    typedef pcl::PointCloud<PointIn> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef std::map<int, int> MapSourceTargetIndices;
    typedef MapSourceTargetIndices* MapSourceTargetIndicesPtr;

    typedef std::map <std::string, std::string> ParameterList;
    typedef std::map <float, int> MapThresholdToSuccesses;

    typedef typename boost::shared_ptr<FeatureCorrespondenceTest<PointIn> > Ptr;

  protected:
    class GroundTruth {
    public:
      GroundTruth () {
        offset_ = Eigen::Vector3f::Identity();
        rotation_ = Eigen::Quaternionf::Identity();
      }

      void
      setGroundTruth (const Eigen::Vector3f &vec, const Eigen::Quaternionf &quat) {
        offset_ = vec;
        rotation_ = quat;
      }

      void
      transformCloud (PointCloudInPtr& cloud_in, PointCloudInPtr& cloud_out) {
        pcl::transformPointCloud (*cloud_in, *cloud_out, offset_, rotation_);
      }

    protected:
      Eigen::Vector3f offset_;
      Eigen::Quaternionf rotation_;
    };

  public:
    /** \brief Empty constructor
      */
     FeatureCorrespondenceTest () : source_input_(), target_input_(), source_transform_(new pcl::PointCloud<PointIn>),
                                    ground_truth_(GroundTruth ()), correspondences_(), done_downsampling_(false),
                                    lower_threshold_(0.01f), upper_threshold_(0.01f), delta_threshold_(0.01f) {}

    inline void
    setInputClouds (const PointCloudInPtr &source, const PointCloudInPtr &target)
    {
      source_input_ = source;
      target_input_ = target;

      preprocessed_source_ = source;
      preprocessed_target_ = target;

      done_downsampling_ = false;
    }

    inline void
    setThreshold (float lower, float upper, float delta)
    {
      if (delta <= 0)
      {
        PCL_ERROR ("Illegal value of delta");
        return;
      }

      if (upper < lower)
      {
        PCL_ERROR ("Illegal lower and upper bounds of threshold range");
        return;
      }

      lower_threshold_ = lower;
      upper_threshold_ = upper;
      delta_threshold_ = delta;
    }

    inline void
    setThreshold (float threshold)
    {
      lower_threshold_ = upper_threshold_ = threshold;
      delta_threshold_ = 1; //any positive value will do;
    }

    inline void
    setGroundTruth (Eigen::Vector3f &vec, Eigen::Quaternionf &quat)
    {
      ground_truth_.setGroundTruth (vec, quat);
    }

    virtual void
    setParameters (ParameterList) {}

    void
    performDownsampling (float leaf_x, float leaf_y, float leaf_z);

    void
    performDownsampling (float leaf_size);

    virtual void
    computeFeatures (double&, double&) {}

    virtual void
    computeFeatures () {}

    /** \brief Calculate the nearest neighbour of each source_feature_ point in the target_feature_ cloud in n-D feature space
      *
      */
    virtual void
    computeCorrespondences () {}

    /** \brief Calculate number of correspondences within \a search_threshold_ of respective ground truth point
      *
      */
    void
    computeResults ();

    inline void
    getPreprocessedSourceSize (int &source_size) { source_size = preprocessed_source_->points.size(); }

    inline void
    getPreprocessedTargetSize (int &target_size) { target_size = preprocessed_target_->points.size(); }

    inline void
    getSuccesses (MapThresholdToSuccesses& result_map) { result_map = no_of_successes_; }

    /** \brief Temporary fix until FeatureCorrespondenceTest is made a friend of the Feature Estimation class.
      *
      */
    virtual std::string
    getClassName () { return "FeatureTest"; }

    virtual void
    clearData () {
      if (source_input_ != NULL) source_input_.reset();
      if (target_input_ != NULL) target_input_.reset();
      if (preprocessed_source_ != NULL) preprocessed_source_.reset();
      if (preprocessed_target_ != NULL) preprocessed_target_.reset();
      if (source_transform_ != NULL) source_transform_.reset();
      correspondences_->clear();
      delete correspondences_;
      no_of_successes_.clear();
    }

  protected:

    PointCloudInPtr source_input_;
    PointCloudInPtr target_input_;
    PointCloudInPtr preprocessed_source_;
    PointCloudInPtr preprocessed_target_;
    PointCloudInPtr source_transform_;

    GroundTruth ground_truth_;

    MapSourceTargetIndicesPtr correspondences_;

    bool done_downsampling_;

    float lower_threshold_, upper_threshold_, delta_threshold_;

    MapThresholdToSuccesses no_of_successes_;

  };

  template <typename PointIn, typename NormalT, typename FeatureDescriptor>
  class FPFHTest : public FeatureCorrespondenceTest<PointIn>
  {
  public:
    using FeatureCorrespondenceTest<PointIn>::preprocessed_source_;
    using FeatureCorrespondenceTest<PointIn>::preprocessed_target_;
    using FeatureCorrespondenceTest<PointIn>::correspondences_;

    typedef pcl::PointCloud<FeatureDescriptor> Features;
    typedef typename Features::Ptr FeaturesPtr;
    typedef typename Features::ConstPtr FeaturesConstPtr;

    typedef typename pcl::search::KdTree<FeatureDescriptor> KdTree;
    typedef typename pcl::search::KdTree<FeatureDescriptor>::Ptr KdTreePtr;


    typedef pcl::PointCloud<NormalT> NormalIn;
    typedef typename NormalIn::Ptr NormalInPtr;
    typedef typename NormalIn::ConstPtr NormalInConstPtr;

    typedef typename pcl::search::KdTree<PointIn> KdTreePointIn;
    typedef typename KdTreePointIn::Ptr KdTreePointInPtr;

    typedef typename FeatureCorrespondenceTest<PointIn>::ParameterList ParameterList;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndices MapSourceTargetIndices;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndicesPtr MapSourceTargetIndicesPtr;

  public:
    FPFHTest () : source_normals_(), target_normals_(), source_features_(),
                  target_features_(), search_radius_(0.05f)
    {
      FeatureCorrespondenceTest<PointIn> ();
    }

    inline void setRadiusSearch (float radius) { search_radius_ = radius; }

    /** \brief Calculate surface normals of input source and target clouds.
      *
      */
    void
    computeNormals (float search_radius);

    /** \brief Set parameters for feature correspondence test algorithm
      *
      */
    void
    setParameters (ParameterList params);

    /** \brief Compute the FPFH feature descriptors of source and target clouds, and return the time taken for both source and target features
      *
      */
    void
    computeFeatures (double& time_source, double& time_target);

    /** \brief Compute the FPFH feature descriptors of source and target clouds
      *
      */
    void
    computeFeatures ();

    /** \brief Calculate the nearest neighbour of each source_feature_ point in the target_feature_ cloud in n-D feature space
      *
      */
    void
    computeCorrespondences ();

    std::string
    getClassName () { return "FPFHEstimation"; }

    void
    clearData () {
      if (source_normals_ != NULL) source_normals_.reset();
      if (target_normals_ != NULL) target_normals_.reset();
      if (source_features_ != NULL) source_features_.reset();
      if (target_features_ != NULL) target_features_.reset();
      FeatureCorrespondenceTest<PointIn>::clearData();
    }

  protected:
    NormalInPtr source_normals_;
    NormalInPtr target_normals_;

    FeaturesPtr source_features_;
    FeaturesPtr target_features_;

    float search_radius_;
  };


  template <typename PointIn, typename NormalT, typename FeatureDescriptor>
  class NormalBasedSignatureTest : public FeatureCorrespondenceTest<PointIn>
  {
  public:
    using FeatureCorrespondenceTest<PointIn>::preprocessed_source_;
    using FeatureCorrespondenceTest<PointIn>::preprocessed_target_;
    using FeatureCorrespondenceTest<PointIn>::correspondences_;

    typedef pcl::PointCloud<FeatureDescriptor> Features;
    typedef typename Features::Ptr FeaturesPtr;
    typedef typename Features::ConstPtr FeaturesConstPtr;

    typedef typename pcl::search::KdTree<FeatureDescriptor> KdTree;
    typedef typename pcl::search::KdTree<FeatureDescriptor>::Ptr KdTreePtr;


    typedef pcl::PointCloud<NormalT> NormalIn;
    typedef typename NormalIn::Ptr NormalInPtr;
    typedef typename NormalIn::ConstPtr NormalInConstPtr;

    typedef typename pcl::search::KdTree<PointIn> KdTreePointIn;
    typedef typename KdTreePointIn::Ptr KdTreePointInPtr;

    typedef typename FeatureCorrespondenceTest<PointIn>::ParameterList ParameterList;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndices MapSourceTargetIndices;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndicesPtr MapSourceTargetIndicesPtr;

  public:
    NormalBasedSignatureTest () : source_normals_(), target_normals_(), source_features_(),
                                  target_features_(), search_radius_(0.05f), scale_(0.05f)
    {
      FeatureCorrespondenceTest<PointIn> ();
    }

    inline void setNormalSearchRadius (float normal_search_radius) { normal_search_radius_ = normal_search_radius; }
    inline void setRadiusSearch (float radius) { search_radius_ = radius; }
    inline void setScale (float scale) { scale_ = scale; }

    /** \brief Calculate surface normals of input source and target clouds.
      *
      */
    void
    computeNormals (float search_radius);

    /** \brief Set parameters for feature correspondence test algorithm
      *
      */
    void
    setParameters (ParameterList params);

    /** \brief Compute the FPFH feature descriptors of source and target clouds, and return the time taken for both source and target features
      *
      */
    void
    computeFeatures (double& time_source, double& time_target);

    /** \brief Compute the FPFH feature descriptors of source and target clouds
      *
      */
    void
    computeFeatures ();

    /** \brief Calculate the nearest neighbour of each source_feature_ point in the target_feature_ cloud in n-D feature space
      *
      */
    void
    computeCorrespondences ();

    std::string
    getClassName () { return "NormalBasedSignatureTest"; }

    void
    clearData () {
      if (source_normals_ != NULL) source_normals_.reset();
      if (target_normals_ != NULL) target_normals_.reset();
      if (source_features_ != NULL) source_features_.reset();
      if (target_features_ != NULL) target_features_.reset();
      FeatureCorrespondenceTest<PointIn>::clearData();
    }

  protected:
    NormalInPtr source_normals_;
    NormalInPtr target_normals_;

    FeaturesPtr source_features_;
    FeaturesPtr target_features_;

    float normal_search_radius_, search_radius_, scale_;
  };


  template <typename PointIn, typename NormalT, typename FeatureDescriptor>
  class PFHTest : public FeatureCorrespondenceTest<PointIn>
  {
  public:
    using FeatureCorrespondenceTest<PointIn>::preprocessed_source_;
    using FeatureCorrespondenceTest<PointIn>::preprocessed_target_;
    using FeatureCorrespondenceTest<PointIn>::correspondences_;

    typedef pcl::PointCloud<FeatureDescriptor> Features;
    typedef typename Features::Ptr FeaturesPtr;
    typedef typename Features::ConstPtr FeaturesConstPtr;

    typedef typename pcl::search::KdTree<FeatureDescriptor> KdTree;
    typedef typename pcl::search::KdTree<FeatureDescriptor>::Ptr KdTreePtr;


    typedef pcl::PointCloud<NormalT> NormalIn;
    typedef typename NormalIn::Ptr NormalInPtr;
    typedef typename NormalIn::ConstPtr NormalInConstPtr;

    typedef typename pcl::search::KdTree<PointIn> KdTreePointIn;
    typedef typename KdTreePointIn::Ptr KdTreePointInPtr;

    typedef typename FeatureCorrespondenceTest<PointIn>::ParameterList ParameterList;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndices MapSourceTargetIndices;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndicesPtr MapSourceTargetIndicesPtr;

  public:
    PFHTest () : source_normals_(), target_normals_(), source_features_(),
    target_features_(), search_radius_(0.05f)
    {
      FeatureCorrespondenceTest<PointIn> ();
    }

    inline void setRadiusSearch (float radius) { search_radius_ = radius; }

    /** \brief Calculate surface normals of input source and target clouds.
     *
     */
    void
    computeNormals (float search_radius);

    /** \brief Set parameters for feature correspondence test algorithm
     *
     */
    void
    setParameters (ParameterList params);

    /** \brief Compute the FPFH feature descriptors of source and target clouds, and return the time taken for both source and target features
     *
     */
    void
    computeFeatures (double& time_source, double& time_target);

    /** \brief Compute the FPFH feature descriptors of source and target clouds
     *
     */
    void
    computeFeatures ();

    /** \brief Calculate the nearest neighbour of each source_feature_ point in the target_feature_ cloud in n-D feature space
     *
     */
    void
    computeCorrespondences ();

    std::string
    getClassName () { return "PFHEstimation"; }

    void
    clearData () {
      if (source_normals_ != NULL) source_normals_.reset();
      if (target_normals_ != NULL) target_normals_.reset();
      if (source_features_ != NULL) source_features_.reset();
      if (target_features_ != NULL) target_features_.reset();
      FeatureCorrespondenceTest<PointIn>::clearData();
    }

  protected:
    NormalInPtr source_normals_;
    NormalInPtr target_normals_;

    FeaturesPtr source_features_;
    FeaturesPtr target_features_;

    float search_radius_;

  };


  template <typename PointIn, typename NormalT, typename FeatureDescriptor>
  class PFHRGBTest : public FeatureCorrespondenceTest<PointIn>
  {
  public:
    using FeatureCorrespondenceTest<PointIn>::preprocessed_source_;
    using FeatureCorrespondenceTest<PointIn>::preprocessed_target_;
    using FeatureCorrespondenceTest<PointIn>::correspondences_;

    typedef pcl::PointCloud<FeatureDescriptor> Features;
    typedef typename Features::Ptr FeaturesPtr;
    typedef typename Features::ConstPtr FeaturesConstPtr;

    typedef typename pcl::search::KdTree<FeatureDescriptor> KdTree;
    typedef typename pcl::search::KdTree<FeatureDescriptor>::Ptr KdTreePtr;


    typedef pcl::PointCloud<NormalT> NormalIn;
    typedef typename NormalIn::Ptr NormalInPtr;
    typedef typename NormalIn::ConstPtr NormalInConstPtr;

    typedef typename pcl::search::KdTree<PointIn> KdTreePointIn;
    typedef typename KdTreePointIn::Ptr KdTreePointInPtr;

    typedef typename FeatureCorrespondenceTest<PointIn>::ParameterList ParameterList;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndices MapSourceTargetIndices;
    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndicesPtr MapSourceTargetIndicesPtr;

  public:
    PFHRGBTest () : source_normals_(), target_normals_(), source_features_(),
    target_features_(), search_radius_(0.05f)
    {
      FeatureCorrespondenceTest<PointIn> ();
    }

    inline void setRadiusSearch (float radius) { search_radius_ = radius; }

    /** \brief Calculate surface normals of input source and target clouds.
     *
     */
    void
    computeNormals (float search_radius);

    /** \brief Set parameters for feature correspondence test algorithm
     *
     */
    void
    setParameters (ParameterList params);

    /** \brief Compute the FPFH feature descriptors of source and target clouds, and return the time taken for both source and target features
     *
     */
    void
    computeFeatures (double& time_source, double& time_target);

    /** \brief Compute the FPFH feature descriptors of source and target clouds
     *
     */
    void
    computeFeatures ();

    /** \brief Calculate the nearest neighbour of each source_feature_ point in the target_feature_ cloud in n-D feature space
     *
     */
    void
    computeCorrespondences ();

    std::string
    getClassName () { return "PFHRGBEstimation"; }

    void
    clearData () {
      if (source_normals_ != NULL) source_normals_.reset();
      if (target_normals_ != NULL) target_normals_.reset();
      if (source_features_ != NULL) source_features_.reset();
      if (target_features_ != NULL) target_features_.reset();
      FeatureCorrespondenceTest<PointIn>::clearData();
    }

  protected:
    NormalInPtr source_normals_;
    NormalInPtr target_normals_;

    FeaturesPtr source_features_;
    FeaturesPtr target_features_;

    float search_radius_;

  };

}

/////////////////////////////////////////////////////////////////////////////////
////////////////// FeatureCorrespondenceTest ////////////////////////////////////
template <typename PointIn> void
pcl::FeatureCorrespondenceTest<PointIn>::performDownsampling (float leaf_x, float leaf_y, float leaf_z)
{
  pcl::VoxelGrid<PointIn> vox_grid;
  vox_grid.setLeafSize (leaf_x, leaf_y, leaf_z);

  if (preprocessed_source_ == NULL)
    preprocessed_source_ = PointCloudInPtr (new pcl::PointCloud<PointIn>);
  if (preprocessed_target_ == NULL)
    preprocessed_target_ = PointCloudInPtr (new pcl::PointCloud<PointIn>);

  vox_grid.setInputCloud (source_input_);
  vox_grid.filter (*preprocessed_source_);

  vox_grid.setInputCloud (target_input_);
  vox_grid.filter (*preprocessed_target_);

  done_downsampling_ = true;
}

template <typename PointIn> void
pcl::FeatureCorrespondenceTest<PointIn>::performDownsampling (float leaf_size)
{
  performDownsampling (leaf_size, leaf_size, leaf_size);
}

template <typename PointIn> void
pcl::FeatureCorrespondenceTest<PointIn>::computeResults ()
{
  if (correspondences_ == NULL)
    return;

  no_of_successes_.clear();

  int no_of_bins = (int)((upper_threshold_ - lower_threshold_)/delta_threshold_) + 1;
  std::vector<int> bins (no_of_bins, 0);

  ground_truth_.transformCloud (preprocessed_source_, source_transform_);

  assert ((preprocessed_source_->points).size() == correspondences_->size());
  assert ((preprocessed_source_->points).size() == (source_transform_->points).size());
  //assert ((preprocessed_source_->points).size() == (preprocessed_target_->points).size());

  for (unsigned index = 0; index < (preprocessed_source_->points).size(); index++)
  {
    int corresponding_point = (*correspondences_)[index];
    float distance_3d = pcl::euclideanDistance<PointIn, PointIn> ((preprocessed_target_->points)[corresponding_point],
                                                                  (source_transform_->points)[index]);
    if (distance_3d <= upper_threshold_)
    {
      int bin_index = (int) ((distance_3d - lower_threshold_)/delta_threshold_) + 1;
      if (bin_index < 0) bin_index = 0;
      if (bin_index >= no_of_bins) bin_index = no_of_bins - 1; //to take care of rounding errors
      bins[bin_index]++;
    }
  }

  int success_count = 0;
  for (unsigned i = 0; i < bins.size(); i++)
  {
    success_count += bins[i];
    float threshold = lower_threshold_ + delta_threshold_*((float) i);
    no_of_successes_[threshold] = success_count;
  }
}

/////////////////////////////////////////////////////////////////////////////////
//////////////////////////  FPFHTest  ///////////////////////////////////////////

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::setParameters (ParameterList params)
{
  if (params.find ("searchradius") != params.end ())
  {
    float radius = boost::lexical_cast<float>(params["searchradius"]);
    setRadiusSearch (radius);
  }
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeNormals (float search_radius)
{
  NormalEstimation<PointIn, NormalT> ne_source;
  ne_source.setInputCloud (preprocessed_source_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  ne_source.setSearchMethod (tree_source);

  if (source_normals_ == NULL)
    source_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_source.setRadiusSearch (search_radius);

  ne_source.compute (*source_normals_);


  NormalEstimation<PointIn, NormalT> ne_target;
  ne_target.setInputCloud (preprocessed_target_);

  KdTreePointInPtr tree_target (new search::KdTree<PointIn> ());
  ne_target.setSearchMethod (tree_target);

  if(target_normals_ == NULL)
    target_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_target.setRadiusSearch (search_radius);

  ne_target.compute (*target_normals_);

}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures (double& time_source, double& time_target)
{
  std::cout << "FPFHTest: computing normals" << std::endl;
  computeNormals(0.5f*search_radius_);

  FPFHEstimation<PointIn, NormalT, FeatureDescriptor> fpfh_source;
  fpfh_source.setInputCloud (preprocessed_source_);
  fpfh_source.setInputNormals (source_normals_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  fpfh_source.setSearchMethod (tree_source);

  if (source_features_ == NULL)
    source_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());

  fpfh_source.setRadiusSearch (search_radius_);

  std::cout << "FPFHTest: computing source features" << std::endl;
  boost::timer time_1;
  fpfh_source.compute (*source_features_);
  time_source = time_1.elapsed();

  FPFHEstimation<PointIn, NormalT, FeatureDescriptor> fpfh_target;
  fpfh_target.setInputCloud (preprocessed_target_);
  fpfh_target.setInputNormals (target_normals_);

  KdTreePointInPtr tree_target (new search::KdTree<PointIn> ());
  fpfh_target.setSearchMethod (tree_target);

  if (target_features_ == NULL)
    target_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());

  fpfh_target.setRadiusSearch (search_radius_);

  std::cout << "FPFHTest: computing target features" << std::endl;
  boost::timer time_2;
  fpfh_target.compute (*target_features_);
  time_target = time_2.elapsed();
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures ()
{
  double t1, t2;
  computeFeatures (t1, t2);
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeCorrespondences ()
{
  if (source_features_ == NULL || target_features_ == NULL)
    return;

  KdTreePtr tree_ = KdTreePtr(new search::KdTree<FeatureDescriptor>);
  tree_->setInputCloud (target_features_);

  std::vector<int> nearest_neighbour (1,0);
  std::vector<float> distance (1,0.0);

  if (correspondences_ == NULL)
    correspondences_ = new MapSourceTargetIndices;
  else
    correspondences_->clear();

  for (unsigned index = 0; index < (source_features_->points).size(); index++)
  {
    tree_->nearestKSearch ( (source_features_->points)[index], 1, nearest_neighbour, distance);
    (*correspondences_)[index] = nearest_neighbour[0];
  }
}





/////////////////////////////////////////////////////////////////////////////////
//////////////////////////  NormalBasedSignatureTest  ///////////////////////////////////////////

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::NormalBasedSignatureTest<PointIn, NormalT, FeatureDescriptor>::setParameters (ParameterList params)
{
  if (params.find ("normalsearchradius") != params.end ())
  {
    float radius = boost::lexical_cast<float>(params["normalsearchradius"]);
    setNormalSearchRadius (radius);
  }

  if (params.find ("searchradius") != params.end ())
  {
    float radius = boost::lexical_cast<float>(params["searchradius"]);
    setRadiusSearch (radius);
  }

  if (params.find ("scale") != params.end ())
  {
    float scale = boost::lexical_cast<float>(params["scale"]);
    setScale (scale);
  }
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::NormalBasedSignatureTest<PointIn, NormalT, FeatureDescriptor>::computeNormals (float search_radius)
{
  PCL_INFO ("Computing normals from NormalBasedSignatureTest with search_radius %f\n", search_radius);
  NormalEstimation<PointIn, NormalT> ne_source;
  ne_source.setInputCloud (preprocessed_source_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  ne_source.setSearchMethod (tree_source);

  if (source_normals_ == NULL)
    source_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_source.setRadiusSearch (search_radius);

  ne_source.compute (*source_normals_);


  NormalEstimation<PointIn, NormalT> ne_target;
  ne_target.setInputCloud (preprocessed_target_);

  KdTreePointInPtr tree_target (new search::KdTree<PointIn> ());
  ne_target.setSearchMethod (tree_target);

  if(target_normals_ == NULL)
    target_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_target.setRadiusSearch (search_radius);

  ne_target.compute (*target_normals_);

}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::NormalBasedSignatureTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures (double& time_source, double& time_target)
{
  std::cout << "NormalBasedSignatureTest: computing normals" << std::endl;
  computeNormals(normal_search_radius_);

  NormalBasedSignatureEstimation<PointIn, NormalT, FeatureDescriptor> nbs_estimator;
  nbs_estimator.setInputCloud (preprocessed_source_);
  nbs_estimator.setInputNormals (source_normals_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  nbs_estimator.setSearchMethod (tree_source);

  if (source_features_ == NULL)
    source_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());

  nbs_estimator.setRadiusSearch (search_radius_);
  nbs_estimator.setScale (scale_);

  std::cout << "NormalBasedSignatureTest: computing source features" << std::endl;
  boost::timer time_1;
  nbs_estimator.compute (*source_features_);
  time_source = time_1.elapsed();

  nbs_estimator.setInputCloud (preprocessed_target_);
  nbs_estimator.setInputNormals (target_normals_);

  if (target_features_ == NULL)
    target_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());


  std::cout << "NormalBasedSignatureTest: computing target features" << std::endl;
  boost::timer time_2;
  nbs_estimator.compute (*target_features_);
  time_target = time_2.elapsed();
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::NormalBasedSignatureTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures ()
{
  double t1, t2;
  computeFeatures (t1, t2);
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::NormalBasedSignatureTest<PointIn, NormalT, FeatureDescriptor>::computeCorrespondences ()
{
  if (source_features_ == NULL || target_features_ == NULL)
    return;

  KdTreePtr tree_ = KdTreePtr(new search::KdTree<FeatureDescriptor>);
  tree_->setInputCloud (target_features_);

  std::vector<int> nearest_neighbour (1,0);
  std::vector<float> distance (1,0.0);

  if (correspondences_ == NULL)
    correspondences_ = new MapSourceTargetIndices;
  else
    correspondences_->clear();

  for (unsigned index = 0; index < (source_features_->points).size(); index++)
  {
    tree_->nearestKSearch ( (source_features_->points)[index], 1, nearest_neighbour, distance);
    (*correspondences_)[index] = nearest_neighbour[0];
  }
}





/////////////////////////////////////////////////////////////////////////////////
//////////////////////////  PFHTest  ///////////////////////////////////////////

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHTest<PointIn, NormalT, FeatureDescriptor>::setParameters (ParameterList params)
{
  if (params.find ("searchradius") != params.end ())
  {
    float radius = boost::lexical_cast<float>(params["searchradius"]);
    setRadiusSearch (radius);
  }
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHTest<PointIn, NormalT, FeatureDescriptor>::computeNormals (float search_radius)
{
  NormalEstimation<PointIn, NormalT> ne_source;
  ne_source.setInputCloud (preprocessed_source_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  ne_source.setSearchMethod (tree_source);

  if (source_normals_ == NULL)
    source_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_source.setRadiusSearch (search_radius);

  ne_source.compute (*source_normals_);


  NormalEstimation<PointIn, NormalT> ne_target;
  ne_target.setInputCloud (preprocessed_target_);

  KdTreePointInPtr tree_target (new search::KdTree<PointIn> ());
  ne_target.setSearchMethod (tree_target);

  if(target_normals_ == NULL)
    target_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_target.setRadiusSearch (search_radius);

  ne_target.compute (*target_normals_);

}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures (double& time_source, double& time_target)
{
  std::cout << "FHTest: computing normals" << std::endl;
  computeNormals(0.5f*search_radius_);

  PFHEstimation<PointIn, NormalT, FeatureDescriptor> pfh_source;
  pfh_source.setInputCloud (preprocessed_source_);
  pfh_source.setInputNormals (source_normals_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  pfh_source.setSearchMethod (tree_source);

  if (source_features_ == NULL)
    source_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());

  pfh_source.setRadiusSearch (search_radius_);

  std::cout << "PFHTest: computing source features" << std::endl;
  boost::timer time_1;
  pfh_source.compute (*source_features_);
  time_source = time_1.elapsed();

  PFHEstimation<PointIn, NormalT, FeatureDescriptor> pfh_target;
  pfh_target.setInputCloud (preprocessed_target_);
  pfh_target.setInputNormals (target_normals_);

  KdTreePointInPtr tree_target (new search::KdTree<PointIn> ());
  pfh_target.setSearchMethod (tree_target);

  if (target_features_ == NULL)
    target_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());

  pfh_target.setRadiusSearch (search_radius_);

  std::cout << "PFHTest: computing target features" << std::endl;
  boost::timer time_2;
  pfh_target.compute (*target_features_);
  time_target = time_2.elapsed();
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures ()
{
  double t1, t2;
  computeFeatures (t1, t2);
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHTest<PointIn, NormalT, FeatureDescriptor>::computeCorrespondences ()
{
  if (source_features_ == NULL || target_features_ == NULL)
    return;

  KdTreePtr tree_ = KdTreePtr(new search::KdTree<FeatureDescriptor>);
  tree_->setInputCloud (target_features_);

  std::vector<int> nearest_neighbour (1,0);
  std::vector<float> distance (1,0.0);

  if (correspondences_ == NULL)
    correspondences_ = new MapSourceTargetIndices;
  else
    correspondences_->clear();

  for (unsigned index = 0; index < (source_features_->points).size(); index++)
  {
    tree_->nearestKSearch ( (source_features_->points)[index], 1, nearest_neighbour, distance);
    (*correspondences_)[index] = nearest_neighbour[0];
  }
}



/////////////////////////////////////////////////////////////////////////////////
//////////////////////////  PFHRGBTest  ///////////////////////////////////////////

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHRGBTest<PointIn, NormalT, FeatureDescriptor>::setParameters (ParameterList params)
{
  if (params.find ("searchradius") != params.end ())
  {
    float radius = boost::lexical_cast<float>(params["searchradius"]);
    setRadiusSearch (radius);
  }
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHRGBTest<PointIn, NormalT, FeatureDescriptor>::computeNormals (float search_radius)
{
  NormalEstimation<PointIn, NormalT> ne_source;
  ne_source.setInputCloud (preprocessed_source_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  ne_source.setSearchMethod (tree_source);

  if (source_normals_ == NULL)
    source_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_source.setRadiusSearch (search_radius);

  ne_source.compute (*source_normals_);


  NormalEstimation<PointIn, NormalT> ne_target;
  ne_target.setInputCloud (preprocessed_target_);

  KdTreePointInPtr tree_target (new search::KdTree<PointIn> ());
  ne_target.setSearchMethod (tree_target);

  if(target_normals_ == NULL)
    target_normals_ = NormalInPtr(new pcl::PointCloud<NormalT>);

  ne_target.setRadiusSearch (search_radius);

  ne_target.compute (*target_normals_);

}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHRGBTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures (double& time_source, double& time_target)
{
  std::cout << "PFHRGBTest: computing normals" << std::endl;
  computeNormals(0.5f*search_radius_);

  PFHRGBEstimation<PointIn, NormalT, FeatureDescriptor> pfhrgb_source;
  pfhrgb_source.setInputCloud (preprocessed_source_);
  pfhrgb_source.setInputNormals (source_normals_);

  KdTreePointInPtr tree_source (new search::KdTree<PointIn> ());
  pfhrgb_source.setSearchMethod (tree_source);

  if (source_features_ == NULL)
    source_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());

  pfhrgb_source.setRadiusSearch (search_radius_);

  std::cout << "PFHRGBTest: computing source features" << std::endl;
  boost::timer time_1;
  pfhrgb_source.compute (*source_features_);
  time_source = time_1.elapsed();

  PFHRGBEstimation<PointIn, NormalT, FeatureDescriptor> pfhrgb_target;
  pfhrgb_target.setInputCloud (preprocessed_target_);
  pfhrgb_target.setInputNormals (target_normals_);

  KdTreePointInPtr tree_target (new search::KdTree<PointIn> ());
  pfhrgb_target.setSearchMethod (tree_target);

  if (target_features_ == NULL)
    target_features_ = FeaturesPtr(new pcl::PointCloud<FeatureDescriptor> ());

  pfhrgb_target.setRadiusSearch (search_radius_);

  std::cout << "PFHRGBTest: computing target features" << std::endl;
  boost::timer time_2;
  pfhrgb_target.compute (*target_features_);
  time_target = time_2.elapsed();
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHRGBTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures ()
{
  double t1, t2;
  computeFeatures (t1, t2);
}

template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
pcl::PFHRGBTest<PointIn, NormalT, FeatureDescriptor>::computeCorrespondences ()
{
  if (source_features_ == NULL || target_features_ == NULL)
    return;

  KdTreePtr tree_ = KdTreePtr(new search::KdTree<FeatureDescriptor>);
  tree_->setInputCloud (target_features_);

  std::vector<int> nearest_neighbour (1,0);
  std::vector<float> distance (1,0.0);

  if (correspondences_ == NULL)
    correspondences_ = new MapSourceTargetIndices;
  else
    correspondences_->clear();

  for (unsigned index = 0; index < (source_features_->points).size(); index++)
  {
    tree_->nearestKSearch ( (source_features_->points)[index], 1, nearest_neighbour, distance);
    (*correspondences_)[index] = nearest_neighbour[0];
  }
}
