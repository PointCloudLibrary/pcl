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


#ifndef PCL_OUTOFCORE_OBJECT_H_
#define PCL_OUTOFCORE_OBJECT_H_

// C++
#include <map>
#include <set>
#include <string>

// VTK
#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

// Boost
//#include <boost/date_time.hpp>
//#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

//Forward Declaration
class Scene;

class Object
{
public:

  // Operators
  // -----------------------------------------------------------------------------
  Object (std::string name);

  virtual
  ~Object () { }


  // Accessors
  // -----------------------------------------------------------------------------
  std::string
  getName () const;

  void
  setName (std::string name);

  virtual void
  render (vtkRenderer* renderer);

  bool
  hasActor (vtkActor *actor);

  void
  addActor (vtkActor *actor);

  void
  removeActor (vtkActor *actor);

  vtkSmartPointer<vtkActorCollection>
  getActors ();

protected:
  vtkSmartPointer<vtkActorCollection> actors_;
  boost::mutex actors_mutex_;

private:

  // Members
  // -----------------------------------------------------------------------------
  std::string name_;
  std::map<vtkActor*, std::set<vtkRenderer*> > associated_renderers_;

};

#endif
