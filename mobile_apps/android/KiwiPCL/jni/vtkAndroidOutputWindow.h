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

#include <vtkOutputWindow.h>
#include <vtkObjectFactory.h>

class vtkAndroidOutputWindow : public vtkOutputWindow
{
public:
  vtkTypeMacro(vtkAndroidOutputWindow, vtkOutputWindow);
  static vtkAndroidOutputWindow* New();

  void DisplayDebugText(const char* t)
    {
    LOGI(t);
    }

  void DisplayWarningText(const char* t)
    {
    LOGW(t);
    }

  void DisplayErrorText(const char* t)
    {
    LOGE(t);
    }

  void DisplayText(const char* t)
    {
    LOGI(t);
    }

  void DisplayGenericWarningText(const char* t)
    {
    LOGW(t);
    }

  static void Install()
    {
    vtkAndroidOutputWindow* window = vtkAndroidOutputWindow::New();
    vtkOutputWindow::SetInstance(window);
    }

private:
  vtkAndroidOutputWindow() { }

  vtkAndroidOutputWindow(const vtkAndroidOutputWindow&);
  void operator=(const vtkAndroidOutputWindow&);

};

vtkStandardNewMacro(vtkAndroidOutputWindow);
