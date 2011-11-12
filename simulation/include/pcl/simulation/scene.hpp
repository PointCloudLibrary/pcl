/*
 * scene.hpp
 *
 *  Created on: Aug 16, 2011
 *      Author: Hordur Johannsson
 */

#ifndef MRG_SCENE_HPP_
#define MRG_SCENE_HPP_

#include <boost/shared_ptr.hpp>

#include "camera.hpp"
#include "model.hpp"

namespace pcl
{

class Scene
{
public:
  void draw();

  void add(Model::Ptr model);

  typedef boost::shared_ptr<Scene> Ptr;
  typedef boost::shared_ptr<Scene> ConstPtr;
private:
  std::vector<Model::Ptr> models_;
};

}

#endif
