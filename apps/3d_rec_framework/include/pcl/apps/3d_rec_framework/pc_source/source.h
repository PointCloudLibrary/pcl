/*
 * source.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#pragma once

#include <pcl/apps/3d_rec_framework/utils/persistence_utils.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

namespace bf = boost::filesystem;

namespace pcl {
namespace rec_3d_framework {

/**
 * \brief Model representation
 * \author Aitor Aldoma
 */

template <typename PointT>
class Model {
  using PointTPtr = typename pcl::PointCloud<PointT>::Ptr;
  using PointTPtrConst = typename pcl::PointCloud<PointT>::ConstPtr;

public:
  std::shared_ptr<std::vector<PointTPtr>> views_;
  std::shared_ptr<
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>>
      poses_;
  std::shared_ptr<std::vector<float>> self_occlusions_;
  std::string id_;
  std::string class_;
  PointTPtr assembled_;
  typename std::map<float, PointTPtrConst> voxelized_assembled_;

  bool
  operator==(const Model& other) const
  {
    return (id_ == other.id_) && (class_ == other.class_);
  }

  PointTPtrConst
  getAssembled(float resolution)
  {
    if (resolution <= 0)
      return assembled_;

    typename std::map<float, PointTPtrConst>::iterator it =
        voxelized_assembled_.find(resolution);
    if (it == voxelized_assembled_.end()) {
      PointTPtr voxelized(new pcl::PointCloud<PointT>);
      pcl::VoxelGrid<PointT> grid_;
      grid_.setInputCloud(assembled_);
      grid_.setLeafSize(resolution, resolution, resolution);
      grid_.setDownsampleAllData(true);
      grid_.filter(*voxelized);

      PointTPtrConst voxelized_const(new pcl::PointCloud<PointT>(*voxelized));
      voxelized_assembled_[resolution] = voxelized_const;
      return voxelized_const;
    }

    return it->second;
  }
};

/**
 * \brief Abstract data source class, manages filesystem, incremental training, etc.
 * \author Aitor Aldoma
 */

template <typename PointInT>
class Source {

protected:
  using ModelT = Model<PointInT>;
  std::string path_;
  std::shared_ptr<std::vector<ModelT>> models_;
  float model_scale_;
  bool filter_duplicate_views_;
  bool load_views_;

  void
  getIdAndClassFromFilename(const std::string& filename,
                            std::string& id,
                            std::string& classname)
  {
    const bf::path path = filename;
    classname = path.parent_path().string() + '/';
    id = path.stem().string();
  }

  void
  createTrainingDir(std::string& training_dir)
  {
    bf::path trained_dir = training_dir;
    if (!bf::exists(trained_dir))
      bf::create_directory(trained_dir);
  }

  void
  createClassAndModelDirectories(std::string& training_dir,
                                 std::string& class_str,
                                 std::string& id_str)
  {
    bf::create_directories(training_dir + '/' + class_str + '/' + id_str);
  }

public:
  Source() { load_views_ = true; }

  virtual ~Source() = default;

  float
  getScale()
  {
    return model_scale_;
  }

  void
  setModelScale(float s)
  {
    model_scale_ = s;
  }

  void
  setFilterDuplicateViews(bool f)
  {
    filter_duplicate_views_ = f;
    std::cout << "setting filter duplicate views to " << f << std::endl;
  }

  void
  getModelsInDirectory(bf::path& dir,
                       std::string& rel_path_so_far,
                       std::vector<std::string>& relative_paths,
                       std::string& ext)
  {
    for (const auto& dir_entry : bf::directory_iterator(dir)) {
      // check if its a directory, then get models in it
      if (bf::is_directory(dir_entry)) {
        std::string so_far =
            rel_path_so_far + (dir_entry.path().filename()).string() + '/';

        bf::path curr_path = dir_entry.path();
        getModelsInDirectory(curr_path, so_far, relative_paths, ext);
      }
      else {
        // check that it is a ply file and then add, otherwise ignore..
        std::vector<std::string> strs;
        std::string file = (dir_entry.path().filename()).string();

        boost::split(strs, file, boost::is_any_of("."));
        std::string extension = strs[strs.size() - 1];

        if (extension == ext) {
          std::string path = rel_path_so_far + (dir_entry.path().filename()).string();

          relative_paths.push_back(path);
        }
      }
    }
  }

  void
  voxelizeAllModels(float resolution)
  {
    for (std::size_t i = 0; i < models_->size(); i++) {
      models_->at(i).getAssembled(resolution);
    }
  }

  /**
   * \brief Generate model representation
   */
  virtual void
  generate(std::string& training_dir) = 0;

  /**
   * \brief Get the generated model
   */
  std::shared_ptr<std::vector<ModelT>>
  getModels()
  {
    return models_;
  }

  std::shared_ptr<std::vector<ModelT>>
  getModels(std::string& model_id)
  {
    models_->erase(std::remove_if(models_->begin(),
                                  models_->end(),
                                  [=](ModelT& s) { return (s.id_ != model_id); }),
                   models_->end());

    return models_;
  }

  bool
  modelAlreadyTrained(ModelT m, std::string& base_dir, std::string& descr_name)
  {
    return bf::exists(getModelDescriptorDir(m, base_dir, descr_name));
  }

  std::string
  getModelDescriptorDir(ModelT m, std::string& base_dir, std::string& descr_name)
  {
    return base_dir + '/' + m.class_ + '/' + m.id_ + '/' + descr_name;
  }

  void
  removeDescDirectory(ModelT m, std::string& base_dir, std::string& descr_name)
  {
    std::string dir = getModelDescriptorDir(m, base_dir, descr_name);

    bf::path desc_dir = dir;
    if (bf::exists(desc_dir))
      bf::remove_all(desc_dir);
  }

  void
  setPath(std::string& path)
  {
    path_ = path;
  }

  void
  setLoadViews(bool load)
  {
    load_views_ = load;
  }
};

} // namespace rec_3d_framework
} // namespace pcl
