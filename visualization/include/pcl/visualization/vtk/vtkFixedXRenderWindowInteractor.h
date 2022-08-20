/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkXRenderWindowInteractor.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
/**
 * @class   vtkXRenderWindowInteractor
 * @brief   an X event driven interface for a RenderWindow
 *
 * vtkXRenderWindowInteractor is a convenience object that provides event
 * bindings to common graphics functions. For example, camera and actor
 * functions such as zoom-in/zoom-out, azimuth, roll, and pan. IT is one of
 * the window system specific subclasses of vtkRenderWindowInteractor. Please
 * see vtkRenderWindowInteractor documentation for event bindings.
 *
 * @sa
 * vtkRenderWindowInteractor
 */

#ifndef vtkXRenderWindowInteractor_h
#define vtkXRenderWindowInteractor_h

//===========================================================
// now we define the C++ class

#include "vtkRenderWindowInteractor.h"
#include "vtkRenderingUIModule.h" // For export macro
#include <X11/Xlib.h>             // Needed for X types in the public interface

namespace pcl {
class vtkCallbackCommand;
class vtkXRenderWindowInteractorInternals;

class VTKRENDERINGUI_EXPORT vtkXRenderWindowInteractor : public vtkRenderWindowInteractor
{
public:
  vtkXRenderWindowInteractor(const vtkXRenderWindowInteractor&) = delete;
  void operator=(const vtkXRenderWindowInteractor&) = delete;

  static vtkXRenderWindowInteractor* New();
  vtkTypeMacro(vtkXRenderWindowInteractor, vtkRenderWindowInteractor);
  void PrintSelf(ostream& os, vtkIndent indent) override;

  /**
   * Initializes the event handlers without an XtAppContext.  This is
   * good for when you don't have a user interface, but you still
   * want to have mouse interaction.
   */
  void Initialize() override;

  /**
   * Break the event loop on 'q','e' keypress. Want more ???
   */
  void TerminateApp() override;

  /**
   * Run the event loop and return. This is provided so that you can
   * implement your own event loop but yet use the vtk event handling as
   * well.
   */
  void ProcessEvents() override;

  ///@{
  /**
   * Enable/Disable interactions.  By default interactors are enabled when
   * initialized.  Initialize() must be called prior to enabling/disabling
   * interaction. These methods are used when a window/widget is being
   * shared by multiple renderers and interactors.  This allows a "modal"
   * display where one interactor is active when its data is to be displayed
   * and all other interactors associated with the widget are disabled
   * when their data is not displayed.
   */
  void Enable() override;
  void Disable() override;
  ///@}

  /**
   * Update the Size data member and set the associated RenderWindow's
   * size.
   */
  void UpdateSize(int, int) override;

  /**
   * Re-defines virtual function to get mouse position by querying X-server.
   */
  void GetMousePosition(int* x, int* y) override;

  void DispatchEvent(XEvent*);

protected:
  vtkXRenderWindowInteractor();
  ~vtkXRenderWindowInteractor() override;

  /**
   * Update the Size data member and set the associated RenderWindow's
   * size but do not resize the XWindow.
   */
  void UpdateSizeNoXResize(int, int);

  // Using static here to avoid destroying context when many apps are open:
  static int NumAppInitialized;

  Display* DisplayId;
  bool OwnDisplay = false;
  Window WindowId;
  Atom KillAtom;
  int PositionBeforeStereo[2];
  vtkXRenderWindowInteractorInternals* Internal;

  // Drag and drop related
  int XdndSourceVersion;
  Window XdndSource;
  Atom XdndFormatAtom;
  Atom XdndURIListAtom;
  Atom XdndTypeListAtom;
  Atom XdndEnterAtom;
  Atom XdndPositionAtom;
  Atom XdndDropAtom;
  Atom XdndActionCopyAtom;
  Atom XdndStatusAtom;
  Atom XdndFinishedAtom;

  ///@{
  /**
   * X-specific internal timer methods. See the superclass for detailed
   * documentation.
   */
  int InternalCreateTimer(int timerId, int timerType, unsigned long duration) override;
  int InternalDestroyTimer(int platformTimerId) override;
  ///@}

  void FireTimers();

  /**
   * This will start up the X event loop and never return. If you
   * call this method it will loop processing X events until the
   * application is exited.
   */
  void StartEventLoop() override;

  /**
   * Deallocate X ressource that may have been allocated
   * Also calls finalize on the render window if available
   */
  void Finalize();

};
} // namespace pcl

#endif
