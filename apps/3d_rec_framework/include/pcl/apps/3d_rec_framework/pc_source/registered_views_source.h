/*
 * ply_source.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/pc_source/source.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl {
namespace rec_3d_framework {

/**
 * \brief Data source class based on partial views from sensor.
 * In this case, the training data is obtained directly from a depth sensor.
 * The filesystem should contain pcd files (representing a view of an object in
 * camera coordinates) and each view needs to be associated with a txt file
 * containing a 4x4 matrix representing the transformation from camera coordinates
 * to a global object coordinates frame.
 * \author Aitor Aldoma
 */

template <typename PointInT>
class RegisteredViewsSource : public Source<PointInT> {
  using SourceT = Source<PointInT>;
  using ModelT = Model<PointInT>;

  using SourceT::createTrainingDir;
  using SourceT::getModelsInDirectory;
  using SourceT::model_scale_;
  using SourceT::models_;
  using SourceT::path_;

  std::string view_prefix_;
  int pose_files_order_; // 0 is row, 1 is column

public:
  RegisteredViewsSource()
  {
    view_prefix_ = std::string("view");
    pose_files_order_ = 0;
  }

  void
  setPrefix(std::string& pre)
  {
    view_prefix_ = pre;
  }

  void
  setPoseRowOrder(int o)
  {
    pose_files_order_ = o;
  }

  void
  getViewsFilenames(bf::path& path_with_views, std::vector<std::string>& view_filenames)
  {
    int number_of_views = 0;
    for (const auto& dir_entry : bf::directory_iterator(path_with_views)) {
      if (!(bf::is_directory(dir_entry))) {
        std::vector<std::string> strs;
        std::vector<std::string> strs_;

        std::string file = (dir_entry.path().filename()).string();

        boost::split(strs, file, boost::is_any_of("."));
        boost::split(strs_, file, boost::is_any_of("_"));

        std::string extension = strs[strs.size() - 1];

        if (extension == "pcd" && (strs_[0].compare(view_prefix_) == 0)) {
          view_filenames.push_back((dir_entry.path().filename()).string());

          number_of_views++;
        }
      }
    }
  }

  void
  assembleModelFromViewsAndPoses(
      ModelT& model,
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>& poses)
  {
    for (std::size_t i = 0; i < model.views_->size(); i++) {
      Eigen::Matrix4f inv = poses[i];
      typename pcl::PointCloud<PointInT>::Ptr global_cloud(
          new pcl::PointCloud<PointInT>);
      pcl::transformPointCloud(*(model.views_->at(i)), *global_cloud, inv);
      *(model.assembled_) += *global_cloud;
    }
  }

  void
  loadOrGenerate(std::string& dir, std::string& model_path, ModelT& model)
  {
    const std::string pathmodel = dir + '/' + model.class_ + '/' + model.id_;
    const bf::path trained_dir = pathmodel;

    model.views_.reset(new std::vector<typename pcl::PointCloud<PointInT>::Ptr>);
    model.poses_.reset(
        new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>);
    model.self_occlusions_.reset(new std::vector<float>);

    if (bf::exists(trained_dir)) {
      // load views and poses
      std::vector<std::string> view_filenames;
      for (const auto& dir_entry : bf::directory_iterator(trained_dir)) {
        // check if its a directory, then get models in it
        if (!(bf::is_directory(*itr))) {
          // check that it is a ply file and then add, otherwise ignore..
          std::vector<std::string> strs;
          std::vector<std::string> strs_;

          std::string file = (dir_entry.path().filename()).string();

          boost::split(strs, file, boost::is_any_of("."));
          boost::split(strs_, file, boost::is_any_of("_"));

          std::string extension = strs[strs.size() - 1];

          if (extension == "pcd" && strs_[0] == "view") {
            view_filenames.push_back((dir_entry.path().filename()).string());
          }
        }
      }

      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
          poses_to_assemble_;

      for (std::size_t i = 0; i < view_filenames.size(); i++) {
        const std::string view_file = pathmodel + '/' + view_filenames[i];
        typename pcl::PointCloud<PointInT>::Ptr cloud(new pcl::PointCloud<PointInT>());
        pcl::io::loadPCDFile(view_file, *cloud);

        model.views_->push_back(cloud);

        std::string file_replaced1(view_filenames[i]);
        boost::replace_all(file_replaced1, "view", "pose");
        boost::replace_all(file_replaced1, ".pcd", ".txt");

        // read pose as well
        const std::string pose_file = pathmodel + '/' + file_replaced1;
        Eigen::Matrix4f pose;
        PersistenceUtils::readMatrixFromFile(pose_file, pose);

        if (pose_files_order_ != 0) {
          Eigen::Matrix4f pose_trans = pose.transpose();
          poses_to_assemble_.push_back(pose_trans);
        }

        std::cout << pose << std::endl;

        // the recognizer assumes transformation from M to CC
        Eigen::Matrix4f inv = pose.inverse();
        model.poses_->push_back(inv);

        model.self_occlusions_->push_back(-1.f);
      }

      model.assembled_.reset(new pcl::PointCloud<PointInT>);
      assembleModelFromViewsAndPoses(model, poses_to_assemble_);
    }
    else {

      // we just need to copy the views to the training directory
      const std::string direc = dir + '/' + model.class_ + '/' << model.id_;
      createClassAndModelDirectories(dir, model.class_, model.id_);

      std::vector<std::string> view_filenames;
      bf::path model_dir = model_path;

      getViewsFilenames(model_dir, view_filenames);
      std::cout << view_filenames.size() << std::endl;

      for (std::size_t i = 0; i < view_filenames.size(); i++) {
        const std::string view_file = model_path + '/' + view_filenames[i];
        typename pcl::PointCloud<PointInT>::Ptr cloud(new pcl::PointCloud<PointInT>());
        pcl::io::loadPCDFile(view_file, *cloud);

        std::cout << view_file << std::endl;

        const std::string path_view = direc + "/view_" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(path_view, *cloud);

        boost::replace_all(view_file, view_prefix_, "pose");
        boost::replace_all(view_file, ".pcd", ".txt");

        Eigen::Matrix4f pose;
        PersistenceUtils::readMatrixFromFile(view_file, pose);

        std::cout << pose << std::endl;

        if (pose_files_order_ == 0) {
          std::cout << "Transpose..." << std::endl;
          Eigen::Matrix4f pose_trans = pose.transpose();
          pose = pose_trans;
          std::cout << pose << std::endl;
        }

        const std::string path_pose = direc + "/pose_" + std::to_string(i) + ".txt";
        pcl::rec_3d_framework::PersistenceUtils::writeMatrixToFile(path_pose, pose);
      }

      loadOrGenerate(dir, model_path, model);
    }
  }

  bool
  isleafDirectory(bf::path& path)
  {
    bool no_dirs_inside = true;
    for (const auto& dir_entry : bf::directory_iterator(path)) {
      if (bf::is_directory(dir_entry)) {
        no_dirs_inside = false;
      }
    }

    return no_dirs_inside;
  }

  void
  getModelsInDirectory(bf::path& dir,
                       std::string& rel_path_so_far,
                       std::vector<std::string>& relative_paths)
  {
    for (const auto& dir_entry : bf::directory_iterator(dir)) {
      // check if its a directory, then get models in it
      if (bf::is_directory(dir_entry)) {
        std::string so_far =
            rel_path_so_far + (dir_entry.path().filename()).string() + '/';
        bf::path curr_path = dir_entry.path();

        if (isleafDirectory(curr_path)) {
          std::string path = rel_path_so_far + (dir_entry.path().filename()).string();
          relative_paths.push_back(path);
        }
        else {
          getModelsInDirectory(curr_path, so_far, relative_paths);
        }
      }
    }
  }

  /**
   * \brief Creates the model representation of the training set, generating views if
   * needed
   */
  void
  generate(std::string& training_dir)
  {

    // create training dir fs if not existent
    createTrainingDir(training_dir);

    // get models in directory
    std::vector<std::string> files;
    std::string start = "";
    bf::path dir = path_;
    getModelsInDirectory(dir, start, files);

    models_.reset(new std::vector<ModelT>);

    for (std::size_t i = 0; i < files.size(); i++) {
      ModelT m;

      std::vector<std::string> strs;
      boost::split(strs, files[i], boost::is_any_of("/\\"));
      std::string name = strs[strs.size() - 1];

      if (strs.size() == 1) {
        m.id_ = strs[0];
      }
      else {
        m.class_ = boost::algorithm::join(strs, '/');
        m.id_ = strs[strs.size() - 1];
      }

      std::cout << m.class_ << " . " << m.id_ << std::endl;
      // check which of them have been trained using training_dir and the model_id_
      // load views, poses and self-occlusions for those that exist
      // generate otherwise

      const std::string model_path = path_ + '/' + files[i];
      loadOrGenerate(training_dir, model_path, m);

      models_->push_back(m);
    }
  }
};
} // namespace rec_3d_framework
} // namespace pcl
