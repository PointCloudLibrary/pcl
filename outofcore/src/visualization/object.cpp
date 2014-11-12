/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


// PCL
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/scene.h>

// Operators
// -----------------------------------------------------------------------------
Object::Object (std::string name)
{
  name_ = name;

  actors_ = vtkSmartPointer<vtkActorCollection>::New ();
}

// Accessors
// -----------------------------------------------------------------------------
std::string
Object::getName () const
{
  return name_;
}

void
Object::setName (std::string name)
{
  name_ = name;
}

vtkSmartPointer<vtkActorCollection>
Object::getActors ()
{
  return actors_;
}

void
Object::render (vtkRenderer* renderer)
{
  boost::mutex::scoped_lock lock (actors_mutex_);
  // Iterate over the objects actors
  actors_->InitTraversal ();
  for (vtkIdType i = 0; i < actors_->GetNumberOfItems (); i++)
  {
    vtkActor* actor = actors_->GetNextActor ();

    // If the actor hasn't been added to the renderer add it
    std::set<vtkRenderer*>::iterator renderer_it;
    renderer_it = associated_renderers_[actor].find (renderer);
    if (renderer_it == associated_renderers_[actor].end ())
    {
      associated_renderers_[actor].insert (renderer);
      renderer->AddActor (actor);
    }
  }
}

bool
Object::hasActor (vtkActor *actor)
{
  boost::mutex::scoped_lock lock (actors_mutex_);

  return actors_->IsItemPresent (actor);
}

void
Object::addActor (vtkActor *actor)
{
//  Scene::instance ()->lock ();
  boost::mutex::scoped_lock lock (actors_mutex_);

  if (!actors_->IsItemPresent (actor))
    actors_->AddItem (actor);

  // If the actor doesn't exist in the associated_renderers_ map add it
  std::map<vtkActor*, std::set<vtkRenderer*> >::iterator actor_it;
  actor_it = associated_renderers_.find (actor);
  if (actor_it == associated_renderers_.end ())
  {
    associated_renderers_[actor] = std::set<vtkRenderer*> ();
  }
//  Scene::instance ()->unlock ();

//  Scene *scene = Scene::instance();
//  std::vector<Viewport*> viewports = scene->getViewports();
//  for(int i=0; i < viewports.size(); i++)
//  {
//
//    vtkRenderer *renderer = viewports[i]->getRenderer();
//    // If the actor hasn't been added to the renderer add it
//    std::set<vtkRenderer*>::iterator renderer_it;
//    renderer_it = associated_renderers_[actor].find(renderer);
//    if (renderer_it == associated_renderers_[actor].end())
//    {
//      associated_renderers_[actor].insert(renderer);
//      renderer->AddActor(actor);
//    }
//  }
}

void
Object::removeActor (vtkActor *actor)
{
//  Scene::instance ()->lock ();
  //std::cout << "Removing Actor" << std::endl;
  boost::mutex::scoped_lock lock (actors_mutex_);
  actors_->RemoveItem (actor);

  std::map<vtkActor*, std::set<vtkRenderer*> >::iterator actor_it;
  actor_it = associated_renderers_.find (actor);

  if (actor_it != associated_renderers_.end ())
  {
    std::set<vtkRenderer*>::iterator renderer_it;
    for (renderer_it = associated_renderers_[actor].begin (); renderer_it != associated_renderers_[actor].end ();
        ++renderer_it)
    {
      (*renderer_it)->RemoveActor (actor);
    }
    associated_renderers_.erase (actor);
  }
  //std::cout << "Removing Actor - DONE" << std::endl;
//  Scene::instance ()->unlock ();
}
