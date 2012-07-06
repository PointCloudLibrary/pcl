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
/// \class vesKiwiPCLDemo
/// \ingroup KiwiPlatform
#ifndef __vesKiwiPCLDemo_h
#define __vesKiwiPCLDemo_h

#include "vesKiwiWidgetRepresentation.h"

class vesShaderProgram;
class vesKiwiPolyDataRepresentation;
class vesGeometryData;

class vesKiwiPCLDemo : public vesKiwiWidgetRepresentation
{
public:

  vesTypeMacro(vesKiwiPCLDemo);
  typedef vesKiwiWidgetRepresentation Superclass;

  vesKiwiPCLDemo();
  ~vesKiwiPCLDemo();

  void initialize(const std::string& filename, vesSharedPtr<vesShaderProgram> shader);


  void setLeafSize(double value);

  void setPlaneDistanceThreshold(double value);


  vesSharedPtr<vesKiwiPolyDataRepresentation> cloudRepresentation();

  virtual void addSelfToRenderer(vesSharedPtr<vesRenderer> renderer);
  virtual void removeSelfFromRenderer(vesSharedPtr<vesRenderer> renderer);
  virtual void willRender(vesSharedPtr<vesRenderer> renderer);

  virtual int numberOfFacets();
  virtual int numberOfVertices();
  virtual int numberOfLines();

  virtual bool handleSingleTouchTap(int displayX, int displayY);
  virtual bool handleSingleTouchDown(int displayX, int displayY);
  virtual bool handleSingleTouchPanGesture(double deltaX, double deltaY);
  virtual bool handleSingleTouchUp();

  class vesInternal;

protected:

  vesSharedPtr<vesGeometryData> updateGeometryData();

private:

  vesKiwiPCLDemo(const vesKiwiPCLDemo&); // Not implemented
  void operator=(const vesKiwiPCLDemo&); // Not implemented


  vesInternal* Internal;
};

#endif
