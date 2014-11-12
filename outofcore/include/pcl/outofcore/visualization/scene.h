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


#ifndef PCL_OUTOFCORE_SCENE_H_
#define PCL_OUTOFCORE_SCENE_H_

// PCL
#include "camera.h"
#include "object.h"
#include "outofcore_cloud.h"
#include "viewport.h"

class Object;

class Scene
{
private:

  static Scene *instance_;

  Scene ();
  Scene (const Scene& op);
  Scene&
  operator= (const Scene& op);

public:

  // Singleton
  static Scene*
  instance ()
  {
    if (!Scene::instance_)
      Scene::instance_ = new Scene ();

    return Scene::instance_;
  }

  // Accessors - Cameras
  // -----------------------------------------------------------------------------
  void
  addCamera (Camera *camera);

  std::vector<Camera*>
  getCameras ();

  Camera*
  getCamera (vtkCamera *camera);

  Camera*
  getCamera (std::string name);

  // Accessors - Objects
  // -----------------------------------------------------------------------------
  void
  addObject (Object *object);

  Object*
  getObjectByName (std::string name);

  std::vector<Object*>
  getObjects ();

  // Accessors - Viewports
  // -----------------------------------------------------------------------------

  void
  addViewport (Viewport *viewport);

  std::vector<Viewport*>
  getViewports ();

  void
  lock ()
  {
    render_mutex_.lock ();
  }

  void
  unlock ()
  {
    render_mutex_.unlock ();
  }

private:
  std::vector<Camera*> cameras_;
  std::vector<Viewport*> viewports_;
  std::vector<Object*> objects_;

  boost::mutex render_mutex_;

};

#endif
