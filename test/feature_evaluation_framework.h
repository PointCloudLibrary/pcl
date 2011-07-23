#include <vector>
#include <map>
#include <fstream>
#include <string>

#include <boost/tokenizer.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/timer.hpp>

#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/io.h>

#include <pcl/kdtree/tree_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/voxel_grid.h>

#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

#include <pcl/registration/transforms.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "feature_test.h"

namespace pcl
{
  /** \brief Framework class for running multiple feature correspondence trials on specified datasets and input parameters.
    *
    */
  template <typename PointIn>
  class FeatureEvaluationFramework: public PCLBase<PointIn>
  {
  public:
    typedef pcl::PointCloud<PointIn> PointCloudIn;
    typedef typename PointCloudIn::Ptr PointCloudInPtr;
    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;

    typedef std::map <std::string, std::string> ParameterList;
    typedef std::map <float, int> MapThresholdToSuccesses;

    typedef typename FeatureCorrespondenceTest<PointIn>::Ptr FeatureCorrespondenceTestPtr;

  public:
    /** \brief Adds all feature descriptor test classes to list of tests.
      *
      */
    FeatureEvaluationFramework () : source_input_(), target_input_(), ground_truths_(Eigen::Matrix4f::Identity())
    {
      tests_.clear();
      log_file_ = "test_results.txt";

      params_.clear();
      feature_name_ = "FeatureTest";
      dataset_label_ = "Dataset";
      params_label_ = "Parameters";

      lower_threshold_ = upper_threshold_ = 0.1;
      delta_threshold_ = 1;

      leaf_x_ = leaf_y_ = leaf_z_ = 0.1;

      do_downsampling_ = false;
      verbose_ = true;

      log_file_ = "test_results.txt";

      // Build our Test registry (We'll need a line here for every feature test we've implemented)
      //includeTest<PFHTest<PointIn, Normal, FPFHSignature33> > ();
      includeTest<FPFHTest<PointIn, Normal, FPFHSignature33> > ("FPFHTest");
      //includeTest<MySuperAwesomeFeatureTest<PointIn, Histogram<123> > > ();
      // and so on ..
    }

    void setFeatureTest (std::string feature_name)
    {
      feature_name_ = feature_name;
    }

    void setInputClouds (std::string source_file, std::string target_file, std::string label = "")
    {
      source_input_ = PointCloudInPtr (new pcl::PointCloud<PointIn>);
      target_input_ = PointCloudInPtr (new pcl::PointCloud<PointIn>);

      pcl::io::loadPCDFile (source_file.c_str(), *source_input_);
      pcl::io::loadPCDFile (target_file.c_str(), *target_input_);

      if (label == "") dataset_label_ = "Dataset";
      else dataset_label_ = label;
    }

    void setGroundTruth (std::string ground_t)
    {
      std::ifstream infile (ground_t.c_str(), std::ios::in);
      for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
          infile >> ground_truths_(i,j);
    }

    void setThreshold (float l_t, float u_t, float d_t)
    {
      lower_threshold_ = l_t;
      upper_threshold_ = u_t;
      delta_threshold_ = d_t;
    }

    void setThreshold (float t)
    {
      lower_threshold_ = upper_threshold_ = t;
      delta_threshold_ = 1;
    }

    void setParameters (std::string params_str, std::string label = "")
    {
      if (label == "") params_label_ = params_str;
      else params_label_ = label;
      params_.clear();

      boost::char_separator<char> sep(", ");
      boost::tokenizer<boost::char_separator<char> > tokens(params_str, sep);

      for (boost::tokenizer<boost::char_separator<char> >::iterator it = tokens.begin(); it != tokens.end(); it++)
      {
        size_t found = (*it).find('=');
        if (found == std::string::npos) continue;
        else
        {
          params_[(*it).substr(0,found)] = (*it).substr(found+1);
        }
      }
    }

    /** \brief Controls the preprocessing (downsampling) of input clouds before running the tests
      *
      */
    void setDownsampling (bool flag)
    {
      do_downsampling_ = flag;
    }

    void setLeafSize (float x, float y, float z)
    {
      leaf_x_ = x;
      leaf_y_ = y;
      leaf_z_ = z;
    }

    void setLeafSize (float l)
    {
      leaf_x_ = leaf_y_ = leaf_z_ = l;
    }

    void setLogFile (std::string s)
    {
      log_file_ = s;
    }

    void setVerbose (bool flag)
    {
      verbose_ = flag;
    }

    void runSingleTest ()
    {
      if (tests_.find(feature_name_) == tests_.end())
      {
        PCL_ERROR ("Unrecognized feature name! (%s)", feature_name_.c_str());
        return;
      }

      if (verbose_) std::cout << "Set input clouds" << std::endl;
      (tests_[feature_name_])->setInputClouds (source_input_, target_input_);

      if (verbose_) std::cout << "Set ground truths" << std::endl;
      (tests_[feature_name_])->setGroundTruths (ground_truths_);

      if (verbose_) std::cout << "Set threshold" << std::endl;
      (tests_[feature_name_])->setThreshold (lower_threshold_, upper_threshold_, delta_threshold_);

      if (do_downsampling_)
      {
        if (verbose_) std::cout << "Perform downsampling" << std::endl;
        (tests_[feature_name_])->performDownsampling (leaf_x_, leaf_y_, leaf_z_);
      }

      if (verbose_) std::cout << "Set parameters" << std::endl;
      (tests_[feature_name_])->setParameters(params_);

      if (verbose_) printDetails ();

      if (verbose_) std::cout << "Computing features" << std::endl;
      boost::timer time_1;
      (tests_[feature_name_])->computeFeatures(time_source_, time_target_);
      time_features_ = time_1.elapsed();
      if (verbose_) std::cout << "Time taken: " << time_features_ << std::endl;

      if (verbose_) std::cout << "Computing correspondences" << std::endl;
      (tests_[feature_name_])->computeCorrespondences();

      if (verbose_) std::cout << "Computing results" << std::endl;
      (tests_[feature_name_])->computeResults();

      (tests_[feature_name_])->getPreprocessedSourceSize(preprocessed_source_size_);
      (tests_[feature_name_])->getPreprocessedTargetSize(preprocessed_target_size_);
      (tests_[feature_name_])->getSuccesses (no_of_successes_);

      if (verbose_) printResults ();
    }

    void runMultipleFeatures (std::string feature_names_file)
    {
      std::ifstream infile (feature_names_file.c_str(), std::ios::in);
      std::string f_name;
      std::ofstream outfile (log_file_.c_str(), std::ios::out);
      while (infile >> f_name)
      {
        setFeatureTest (f_name);
        runSingleTest ();
        outfile << feature_name_ << "," << time_source_ << "," << time_target_ << "," << time_features_ << std::endl;
        for (MapThresholdToSuccesses::iterator it = no_of_successes_.begin(); it != no_of_successes_.end(); it++)
        {
          outfile << it->first << "," << it->second << ",";
        }
        outfile << std::endl;
      }
      infile.close();
      outfile.close();
    }

    void runMultipleClouds (std::string clouds_file)
    {
      std::ifstream infile (clouds_file.c_str(), std::ios::in);
      std::string s_name, t_name, label;
      std::ofstream outfile (log_file_.c_str(), std::ios::out);
      while (infile >> s_name >> t_name >> label)
      {
        setInputClouds (s_name, t_name, label);
        runSingleTest ();
        outfile << dataset_label_ << "," << time_source_ << "," << time_target_ << "," << time_features_ << std::endl;
        for (MapThresholdToSuccesses::iterator it = no_of_successes_.begin(); it != no_of_successes_.end(); it++)
        {
          outfile << it->first << "," << it->second << ",";
        }
        outfile << std::endl;
      }
      infile.close();
      outfile.close();
    }

    void runMultipleParameters (std::string params_file)
    {
      std::ifstream infile (params_file.c_str(), std::ios::in);
      std::string p_name;
      std::ofstream outfile (log_file_.c_str(), std::ios::out);
      while (infile >> p_name)
      {
        setParameters (p_name);
        runSingleTest ();
        outfile << params_label_ << "," << time_source_ << "," << time_target_ << "," << time_features_ << std::endl;
        for (MapThresholdToSuccesses::iterator it = no_of_successes_.begin(); it != no_of_successes_.end(); it++)
        {
          outfile << it->first << "," << it->second << ",";
        }
        outfile << std::endl;
      }
      infile.close();
      outfile.close();
    }

    void runMultipleLeafSizes (std::string leaf_file)
    {
      if (!do_downsampling_) do_downsampling_ = true;
      std::ifstream infile (leaf_file.c_str(), std::ios::in);
      float l_x, l_y, l_z;
      std::string line;
      std::ofstream outfile (log_file_.c_str(), std::ios::out);

      while (getline(infile,line))
      {
        boost::char_separator<char> sep(" ");
        boost::tokenizer<boost::char_separator<char> > tokens(line, sep);

        boost::tokenizer<boost::char_separator<char> >::iterator it = tokens.begin();
        if (it == tokens.end()) continue;
        l_x = boost::lexical_cast<float>(*it);
        it++;
        if (it == tokens.end())
        {
          l_y = l_z = l_x;
        }
        else
        {
          l_y = boost::lexical_cast<float>(*it);
          it++;
          if (it == tokens.end()) continue;
          l_z = boost::lexical_cast<float>(*it);
        }

        setLeafSize (l_x, l_y, l_z);
        runSingleTest ();
        outfile << leaf_x_ << "," << leaf_y_ << "," << leaf_z_ << ",";
        outfile << preprocessed_source_size_ << "," << preprocessed_target_size_ << ",";
        outfile << time_source_ << "," << time_target_ << "," << time_features_ << std::endl;
        for (MapThresholdToSuccesses::iterator it = no_of_successes_.begin(); it != no_of_successes_.end(); it++)
        {
          outfile << it->first << "," << it->second << ",";
        }
        outfile << std::endl;
      }
      infile.close();
      outfile.close();
    }

    void printDetails ()
    {
      std::cout << "----------Test Details:----------" << std::endl;
      std::cout << "Feature Name:  " << feature_name_ << std::endl;
      std::cout << "Input Dataset: " << dataset_label_ << std::endl;
      std::cout << "Parameters:    " << params_label_ << std::endl;
      if (do_downsampling_)
        std::cout << "Leaf size:     " << leaf_x_ << " " << leaf_y_ << " " << leaf_z_ << " " << std::endl;
      std::cout << "---------------------------------" << std::endl;

      std::cout << std::endl;
    }

    void printResults ()
    {
      std::cout << "----------Test Results:----------" << std::endl;
      std::cout << "Source Size:   " << preprocessed_source_size_ << std::endl;
      std::cout << "Target Size:   " << preprocessed_target_size_ << std::endl;
      std::cout << "Time Taken For Feature Computations:" << std::endl;
      std::cout << "  Source:      " << time_source_ << std::endl;
      std::cout << "  Target:      " << time_target_ << std::endl;
      std::cout << "  Total:       " << time_features_ << std::endl;
      std::cout << "Threshold -> Successes" << std::endl;
      for (MapThresholdToSuccesses::iterator it = no_of_successes_.begin(); it != no_of_successes_.end(); it++)
      {
        std::cout << "  " << it->first << " -> " << it->second << std::endl;
      }
      std::cout << "---------------------------------" << std::endl;

      std::cout << std::endl;
    }

  private:

    /** \brief Add the given test class to our registry of correspondence tests
      *
      */
    template <class FeatureCorrespondenceTest>
    void includeTest (std::string feature_name)
    {
      tests_[feature_name] = typename FeatureCorrespondenceTest::Ptr  (new FeatureCorrespondenceTest);
    }

    /** \brief A map from class name to the FeatureCorrespondenceTests for those tests
      */
    std::map<std::string, FeatureCorrespondenceTestPtr> tests_;

    std::string feature_name_;

    std::string params_label_;
    ParameterList params_;

    std::string dataset_label_;
    PointCloudInPtr source_input_;
    PointCloudInPtr target_input_;

    Eigen::Matrix4f ground_truths_;

    float lower_threshold_, upper_threshold_, delta_threshold_;

    /** \brief Flag for controlling preprocessing of input clouds
      */
    bool do_downsampling_;
    float leaf_x_, leaf_y_, leaf_z_;

    int preprocessed_source_size_, preprocessed_target_size_;
    double time_source_, time_target_, time_features_;
    MapThresholdToSuccesses no_of_successes_;

    /** \brief File for recording test outputs
      */
    std::string log_file_;

    /** \brief Control console output during execution of tests
      */
    bool verbose_;
  };

}
