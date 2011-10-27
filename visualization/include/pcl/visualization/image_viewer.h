/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Suat Gedikli (gedikli@willowgarage.com)
 */

#ifndef PCL_VISUALIZATION_IMAGE_VISUALIZER_H__
#define	PCL_VISUALIZATION_IMAGE_VISUALIZER_H__

#include <vtkImageViewer.h>
#include <vtkImageViewer2.h>
#include <vtkInteractorStyle.h>
#include <boost/shared_array.hpp>
#include <pcl/pcl_macros.h>
#include <pcl/console/print.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>
#include <boost/signals2.hpp>
#include <vtkCallbackCommand.h>
#include <pcl/visualization/interactor_style.h>

namespace pcl
{
  namespace visualization
  {
    typedef Eigen::Array<unsigned char, 3, 1> Vector3ub;
    static const Vector3ub green_color(0,255,0);
    static const Vector3ub red_color(255,0,0);
    static const Vector3ub blue_color(0,0,255);

    class PCL_EXPORTS ImageViewer
    {
      public:
        ImageViewer (const std::string& window_title = "");
        virtual ~ImageViewer ();
        
        void 
        showRGBImage (const unsigned char* data, unsigned width, unsigned height);

        void 
        showRGBImage (const pcl::PointCloud<pcl::PointXYZRGB> &data);

        void 
        showFloatImage (const float* data, unsigned int width, unsigned int height, 
                        float min_value, float max_value, bool grayscale = false);
        
        void
        showShortImage (const unsigned short* short_image, unsigned int width, unsigned int height, 
                        unsigned short min_value, unsigned short max_value, bool grayscale);

        void 
        showAngleImage (const float* data, unsigned width, unsigned height);

        void 
        showHalfAngleImage (const float* data, unsigned width, unsigned height);

        /**\brief Sets the pixel at coordinates(u,v) to color while setting the neighborhood to another
          * \param fg_color the pixel color
          * \param bg_color the neighborhood color
          * \param radius the circle radius around the pixel
          */
        void
        markPoint (size_t u, size_t v, Vector3ub fg_color, Vector3ub bg_color = red_color, float radius = 2);

        void
        setName(const std::string& name)
        {
          strcpy(image_viewer_->GetWindowName (), name.c_str ());
        }

        /** \brief Spin method. Calls the interactor and runs an internal loop. */
        void 
        spin ();
        
        /** \brief Spin once method. Calls the interactor and updates the screen once. 
          * \param time - How long (in ms) should the visualization loop be allowed to run.
          * \param force_redraw - if false it might return without doing anything if the 
          * interactor's framerate does not require a redraw yet.
          */
        void 
        spinOnce (int time = 1, bool force_redraw = false);
        
        /** \brief registering a callback function for keyboard events
          * \param callback  the function that will be registered as a callback for a keyboard event
          * \param cookie    user data that is passed to the callback
          * \return          connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), 
                                  void* cookie = NULL)
        {
          return (registerKeyboardCallback (boost::bind (callback, _1, cookie)));
        }
        
        /** \brief registering a callback function for keyboard events
          * \param callback  the member function that will be registered as a callback for a keyboard event
          * \param instance  instance to the class that implements the callback function
          * \param cookie    user data that is passed to the callback
          * \return          connection object that allows to disconnect the callback function.
          */
        template<typename T> boost::signals2::connection 
        registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), 
                                  T& instance, void* cookie = NULL)
        {
          return (registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie)));
        }
        
        /** \brief   registering a callback boost::function for keyboard events
          * \param   the boost function that will be registered as a callback for a keyboard event
          * \return  connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> );

        /** \brief 
          * \param callback  the function that will be registered as a callback for a mouse event
          * \param cookie    user data that is passed to the callback
          * \return          connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerMouseCallback (void (*callback) (const pcl::visualization::MouseEvent&, void*), 
                               void* cookie = NULL)
        {
          return (registerMouseCallback (boost::bind (callback, _1, cookie)));
        }
        
        /** \brief registering a callback function for mouse events
          * \param callback  the member function that will be registered as a callback for a mouse event
          * \param instance  instance to the class that implements the callback function
          * \param cookie    user data that is passed to the callback
          * \return          connection object that allows to disconnect the callback function.
          */
        template<typename T> boost::signals2::connection 
        registerMouseCallback (void (T::*callback) (const pcl::visualization::MouseEvent&, void*), 
                               T& instance, void* cookie = NULL)
        {
          return (registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }

        /** \brief   registering a callback function for mouse events
          * \param   the boost function that will be registered as a callback for a mouse event
          * \return  connection object that allows to disconnect the callback function.
          */        
        boost::signals2::connection 
        registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> );
        
      protected: // methods
        /** \brief Set the stopped flag back to false. */
        void 
        resetStoppedFlag () { stopped_ = false; }

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
          virtual void 
          Execute (vtkObject* vtkNotUsed (caller), unsigned long event_id, void* call_data)
          {
            if (event_id != vtkCommand::TimerEvent)
              return;
            int timer_id = *(int*)call_data;
            if (timer_id != right_timer_id)
              return;
            window->interactor_->TerminateApp ();
          }
          int right_timer_id;
          ImageViewer* window;
        };
        struct ExitCallback : public vtkCommand
        {
          static ExitCallback* New ()
          {
            return (new ExitCallback);
          }
          virtual void 
          Execute (vtkObject* caller, unsigned long event_id, void* call_data)
          {
            if (event_id != vtkCommand::ExitEvent)
              return;
            window->stopped_ = true;
            window->interactor_->TerminateApp ();
          }
          ImageViewer* window;
        };

    private:
        boost::signals2::signal<void (const pcl::visualization::MouseEvent&)> mouse_signal_;
        boost::signals2::signal<void (const pcl::visualization::KeyboardEvent&)> keyboard_signal_;
        
        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
        vtkCallbackCommand* mouse_command_;
        vtkCallbackCommand* keyboard_command_;

        /** \brief Callback object enabling us to leave the main loop, when a timer fires. */
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
        vtkSmartPointer<ExitCallback> exit_callback_;

        /** \brief The ImageViewer widget. */
        vtkSmartPointer<vtkImageViewer> image_viewer_;
   
        /** \brief The data array representing the image. Used internally. */
        boost::shared_array<unsigned char> data_;
        /** \brief The data array (representing the image) size. Used internally. */
        size_t data_size_;

        /** \brief Set to false if the interaction loop is running. */
        bool stopped_;

        /** \brief Global timer ID. Used in destructor only. */
        int timer_id_;
     };
  }
}

#endif	/* __IMAGE_VISUALIZER_H__ */

