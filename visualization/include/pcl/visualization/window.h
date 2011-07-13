#ifndef __WINDOW_H__
#define	__WINDOW_H__
#include <pcl/pcl_macros.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
#include <boost/signals2.hpp>
#include <vtkCallbackCommand.h>
#include <vtkInteractorStyle.h>
#include <pcl/visualization/interactor.h>

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
        inline virtual ~Window ();
        
        void spin ();
        
        void spinOnce (int time = 1, bool force_redraw = false);
        
        /** \brief Returns true when the user tried to close the window */
        bool 
        wasStopped () const { return (interactor_->stopped); }

        /**
         * @brief registering a callback function for keyboard events
         * @param callback  the function that will be registered as a callback for a keyboard event
         * @param cookie    user data that is passed to the callback
         * @return          connection object that allows to disconnect the callback function.
         */
        boost::signals2::connection registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), void* cookie = NULL)
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
        template<typename T>
        boost::signals2::connection registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), T& instance, void* cookie = NULL)
        {
          return registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie));
        }
        
        /**
         * @brief 
         * @param callback  the function that will be registered as a callback for a mouse event
         * @param cookie    user data that is passed to the callback
         * @return          connection object that allows to disconnect the callback function.
         */
        boost::signals2::connection registerMouseCallback (void (*callback) (const pcl::visualization::MouseEvent&, void*), void* cookie = NULL)
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
        template<typename T>
        boost::signals2::connection registerMouseCallback (void (T::*callback) (const pcl::visualization::MouseEvent&, void*), T& instance, void* cookie = NULL)
        {
          return registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie));
        }
        
      protected: // methods

        /** \brief Set the stopped flag back to false */
        void resetStoppedFlag () { interactor_->stopped = false; }
        /**
         * @brief   registering a callback function for mouse events
         * @param   the boost function that will be registered as a callback for a mouse event
         * @return  connection object that allows to disconnect the callback function.
         */        
        boost::signals2::connection registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> );
        
        /**
         * @brief   registering a callback boost::function for keyboard events
         * @param   the boost function that will be registered as a callback for a keyboard event
         * @return  connection object that allows to disconnect the callback function.
         */
        boost::signals2::connection registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> );
        
        void emitMouseEvent (unsigned long event_id);
        
        void emitKeyboardEvent (unsigned long event_id);
        
        // Callbacks used to register for vtk command
        static void MouseCallback (vtkObject*, unsigned long eid, void* clientdata, void *calldata);
        static void KeyboardCallback (vtkObject*, unsigned long eid, void* clientdata, void *calldata);
        
    protected: // types
      struct ExitMainLoopTimerCallback : public vtkCommand
        {
          static ExitMainLoopTimerCallback* New()
          {
            return new ExitMainLoopTimerCallback;
          }
          virtual void Execute(vtkObject* vtkNotUsed(caller), unsigned long event_id, void* call_data)
          {
            if (event_id != vtkCommand::TimerEvent)
              return;
            int timer_id = *(int*)call_data;
            //cout << "Timer "<<timer_id<<" called.\n";
            if (timer_id != right_timer_id)
              return;
            // Stop vtk loop and send notification to app to wake it up
            window_->interactor_->stopLoop ();
          }
          int right_timer_id;
          Window* window_;
        };
        struct ExitCallback : public vtkCommand
        {
          static ExitCallback* New ()
          {
            return new ExitCallback;
          }
          virtual void Execute (vtkObject* caller, unsigned long event_id, void* call_data)
          {
            if (event_id != vtkCommand::ExitEvent)
              return;
            window_->interactor_->stopped = true;
            //this tends to close the window...
            window_->interactor_->stopLoop ();
          }
          Window* window_;
        };
        
    protected: // member fields
        boost::signals2::signal<void (const pcl::visualization::MouseEvent&)> mouse_signal_;
        boost::signals2::signal<void (const pcl::visualization::KeyboardEvent&)> keyboard_signal_;
        
        vtkSmartPointer<vtkRenderWindow> window_;
        vtkSmartPointer<PCLVisualizerInteractor> interactor_;
        vtkCallbackCommand* mouse_command_;
        vtkCallbackCommand* keyboard_command_;
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
        vtkSmartPointer<ExitCallback> exit_callback_;
        
    };
    
    Window::~Window ()
    {}
  }
}

#endif	/* __WINDOW_H__ */

