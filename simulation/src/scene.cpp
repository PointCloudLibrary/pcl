/*
 * scene.cpp
 *
 *  Created on: Aug 16, 2011
 *      Author: Hordur Johannsson
 */

#include <pcl/simulation/scene.h>

namespace pcl
{

namespace simulation
{

void
Scene::add (Model::Ptr model)
{
  models_.push_back(model);
}

void
Scene::addCompleteModel (std::vector<Model::Ptr> model)
{
  models_.push_back (model[0]);
}

void
Scene::draw ()
{
  for (std::vector<Model::Ptr>::iterator model = models_.begin (); model != models_.end (); ++model)
    (*model)->draw ();
}

void
Scene::clear ()
{
  models_.clear();
}

} // namespace - simulation
} // namespace - pcl
