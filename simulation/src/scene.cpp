/*
 * scene.cpp
 *
 *  Created on: Aug 16, 2011
 *      Author: Hordur Johannsson
 */

#include "pcl/simulation/scene.hpp"

namespace pcl
{
void Scene::add(Model::Ptr model)
{
  models_.push_back(model);
}

void Scene::draw()
{
  for (std::vector<Model::Ptr>::iterator model = models_.begin(); model != models_.end(); ++model)
    (*model)->draw();
}
}
