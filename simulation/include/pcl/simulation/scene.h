/*
 * scene.hpp
 *
 *  Created on: Aug 16, 2011
 *      Author: Hordur Johannsson
 */

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
//#include <pcl/win32_macros.h>
#include <pcl/simulation/camera.h>
#include <pcl/simulation/model.h>

namespace pcl {
namespace simulation {

class PCL_EXPORTS Scene {
public:
  using Ptr = shared_ptr<Scene>;
  using ConstPtr = shared_ptr<Scene>;

  void
  draw();

  void
  add(Model::Ptr model);

  void
  addCompleteModel(std::vector<Model::Ptr> model);

  void
  clear();

private:
  std::vector<Model::Ptr> models_;
};

} // namespace simulation
} // namespace pcl
