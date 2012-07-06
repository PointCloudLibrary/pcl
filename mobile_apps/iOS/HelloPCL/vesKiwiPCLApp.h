/*========================================================================
  VES --- VTK OpenGL ES Rendering Toolkit

      http://www.kitware.com/ves

  Copyright 2011 Kitware, Inc.

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
 ========================================================================*/
/// \class vesKiwiPCLApp
/// \ingroup KiwiPlatform
#ifndef __vesKiwiPCLApp_h
#define __vesKiwiPCLApp_h

#include "vesKiwiViewerApp.h"
#include "vesKiwiPCLDemo.h"

#include <vtksys/SystemTools.hxx>

class vesShaderProgram;
class vesKiwiPolyDataRepresentation;
class vesGeometryData;

class vesKiwiPCLApp : public vesKiwiViewerApp
{
public:

  vesTypeMacro(vesKiwiPCLApp);

  vesKiwiPCLApp()
  {
    this->addBuiltinDataset("Kinect point cloud", "pointcloud.pcd");
  }

  ~vesKiwiPCLApp()
  {
  }

  virtual  bool loadDatasetWithCustomBehavior(const std::string& filename)
  {
    if (this->mRep) {
      this->mRep->removeSelfFromRenderer(this->renderer());
      this->mRep.reset();
    }

    if (vtksys::SystemTools::GetFilenameLastExtension(filename) == ".pcd") {
      return this->loadPCLDemo(filename);
    }
    return vesKiwiViewerApp::loadDatasetWithCustomBehavior(filename);
  }

  bool loadPCLDemo(const std::string& filename)
  {
    this->mRep = vesKiwiPCLDemo::Ptr(new vesKiwiPCLDemo);
    this->mRep->initialize(filename, this->shaderProgram());
    this->mRep->addSelfToRenderer(this->renderer());
    this->setBackgroundColor(0., 0., 0.);
    return true;
  }

  vesKiwiPCLDemo::Ptr getPCLDemo()
  {
    return this->mRep;
  }


protected:


  vesKiwiPCLDemo::Ptr mRep;


private:

  vesKiwiPCLApp(const vesKiwiPCLApp&); // Not implemented
  void operator=(const vesKiwiPCLApp&); // Not implemented

};

#endif
