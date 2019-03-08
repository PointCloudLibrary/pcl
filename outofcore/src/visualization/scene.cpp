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
#include <pcl/outofcore/visualization/camera.h>
#include <pcl/outofcore/visualization/object.h>
#include <pcl/outofcore/visualization/scene.h>
#include <pcl/outofcore/visualization/viewport.h>

Scene* Scene::instance_ = NULL;

Scene::Scene ()
{

}

// Accessors - Cameras
// -----------------------------------------------------------------------------
void
Scene::addCamera (Camera *camera)
{
  cameras_.push_back (camera);
}

std::vector<Camera*>
Scene::getCameras ()
{
  return cameras_;
}

Camera*
Scene::getCamera (vtkCamera *camera)
{
  for (const auto &c : cameras_)
  {
    if (c->getCamera ().GetPointer () == camera)
    {
      return c;
    }
  }

  return NULL;
}

Camera*
Scene::getCamera (std::string name)
{
  for (const auto &camera : cameras_)
    if (camera->getName () == name)
      return camera;

  return NULL;
}

// Accessors - Objects
// -----------------------------------------------------------------------------
void
Scene::addObject (Object *object)
{
  objects_.push_back (object);
}

Object*
Scene::getObjectByName (std::string name)
{
  for (const auto &object : objects_)
    if (object->getName () == name)
      return object;

  return NULL;
}

std::vector<Object*>
Scene::getObjects ()
{
  return objects_;
}

//  void removeObject(Object *object)
//  {
//
//  }

// Accessors - Viewports
// -----------------------------------------------------------------------------

void
Scene::addViewport (Viewport *viewport)
{
  viewports_.push_back (viewport);
}

std::vector<Viewport*>
Scene::getViewports ()
{
  return viewports_;
}
