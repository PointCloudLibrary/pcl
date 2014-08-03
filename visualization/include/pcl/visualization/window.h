/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
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

#ifndef PCL_VISUALIZER_WINDOW_H__
#define	PCL_VISUALIZER_WINDOW_H__

#include <pcl/pcl_macros.h>
#include <boost/signals2/signal.hpp>

#include <vtkCommand.h>

template <typename T> class vtkSmartPointer;
class vtkObject;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkCallbackCommand;
class vtkRendererCollection;
class PCLVisualizerInteractorStyle;

namespace pcl
{
  namespace visualization
  {
    class MouseEvent;
    class KeyboardEvent;

    class PCL_EXPORTS Window
    {
      public:
        Window (const std::string& window_name = "");
        Window (const Window &src);
        Window& operator = (const Window &src);

        virtual ~Window ();

        /** \brief Spin method. Calls the interactor and runs an internal loop. */
        void
        spin ();

        /** \brief Spin once method. Calls the interactor and updates the screen once.
          *  \param time - How long (in ms) should the visualization loop be allowed to run.
          *  \param force_redraw - if false it might return without doing anything if the
          *  interactor's framerate does not require a redraw yet.
          */
        void
        spinOnce (int time = 1, bool force_redraw = false);

        /** \brief Returns true when the user tried to close the window */
        bool
        wasStopped () const { return (stopped_); }

        /**
          * @brief registering a callback function for keyboard events
          * @param callback  the function that will be registered as a callback for a keyboard event
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*),
                                  void* cookie = NULL)
        {
          return registerKeyboardCallback (boost::bind (callback, _1, cookie));
        }

        /**
          * @brief registering a callback function for keyboard events
          * @param callback  the member function that will be registered as a callback for a keyboard event
          * @param instance  instance to the class that implements the callback function
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
        template<typename T> boost::signals2::connection
        registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*),
                                  T& instance, void* cookie = NULL)
        {
          return registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie));
        }

        /**
          * @brief
          * @param callback  the function that will be registered as a callback for a mouse event
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerMouseCallback (void (*callback) (const pcl::visualization::MouseEvent&, void*),
                               void* cookie = NULL)
        {
          return registerMouseCallback (boost::bind (callback, _1, cookie));
        }

        /**
          * @brief registering a callback function for mouse events
          * @param callback  the member function that will be registered as a callback for a mouse event
          * @param instance  instance to the class that implements the callback function
          * @param cookie    user data that is passed to the callback
          * @return          connection object that allows to disconnect the callback function.
          */
        template<typename T> boost::signals2::connection
        registerMouseCallback (void (T::*callback) (const pcl::visualization::MouseEvent&, void*),
                               T& instance, void* cookie = NULL)
        {
          return registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie));
        }

      protected: // methods

        /** \brief Set the stopped flag back to false */
        void
        resetStoppedFlag () { stopped_ = false; }

        /**
          * @brief   registering a callback function for mouse events
          * @return  connection object that allows to disconnect the callback function.
          */
         // param   the boost function that will be registered as a callback for a mouse event
        boost::signals2::connection
        registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> );

        /**
         * @brief   registering a callback boost::function for keyboard events
         * @return  connection object that allows to disconnect the callback function.
         */
         // param   the boost function that will be registered as a callback for a keyboard event
        boost::signals2::connection
        registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> );

        void
        emitMouseEvent (unsigned long event_id);

        void
        emitKeyboardEvent (unsigned long event_id);

        // Callbacks used to register for vtk command
        static void
        MouseCallback (vtkObject*, unsigned long eid, void* clientdata, void *calldata);
        static void
        KeyboardCallback (vtkObject*, unsigned long eid, void* clientdata, void *calldata);

      protected: // types
        struct ExitMainLoopTimerCallback : public vtkCommand
        {
          static ExitMainLoopTimerCallback* New ()
          {
            return (new ExitMainLoopTimerCallback);
          }

          ExitMainLoopTimerCallback ();
          ExitMainLoopTimerCallback (const ExitMainLoopTimerCallback& src);
          ExitMainLoopTimerCallback& operator = (const ExitMainLoopTimerCallback& src);

          virtual void 
          Execute (vtkObject*, unsigned long event_id, void* call_data);

          int right_timer_id;
          Window* window;
        };

        struct ExitCallback : public vtkCommand
        {
          static ExitCallback* New ()
          {
            return (new ExitCallback);
          }

          ExitCallback ();
          ExitCallback (const ExitCallback &src);
          ExitCallback& operator = (const ExitCallback &src);
 
          virtual void 
          Execute (vtkObject*, unsigned long event_id, void*);

          Window* window;
        };

        bool stopped_;
        int timer_id_;

    protected: // member fields
        boost::signals2::signal<void (const pcl::visualization::MouseEvent&)> mouse_signal_;
        boost::signals2::signal<void (const pcl::visualization::KeyboardEvent&)> keyboard_signal_;

        vtkSmartPointer<vtkRenderWindow> win_;
        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
        vtkCallbackCommand* mouse_command_;
        vtkCallbackCommand* keyboard_command_;
        /** \brief The render window interactor style. */
        vtkSmartPointer<PCLVisualizerInteractorStyle> style_;
        /** \brief The collection of renderers used. */
        vtkSmartPointer<vtkRendererCollection> rens_;
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
        vtkSmartPointer<ExitCallback> exit_callback_;
    };
  }
}

#endif	/* __WINDOW_H__ */

