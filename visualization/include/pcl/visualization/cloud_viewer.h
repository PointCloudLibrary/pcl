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
 *
 */
#ifndef PCL_CLOUD_VIEWER_H_
#define PCL_CLOUD_VIEWER_H_

#include <pcl/visualization/pcl_visualizer.h> //pcl vis

#include <string>

namespace pcl
{
  namespace visualization
  {
    /** \brief Simple point cloud visualization class
      * \author Ethan Rublee
      * \ingroup visualization
      */
    class PCL_EXPORTS CloudViewer : boost::noncopyable
    {
      public:
        typedef pcl::PointCloud<pcl::PointXYZRGBA> ColorACloud;
        typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
        typedef pcl::PointCloud<pcl::PointXYZI> GrayCloud;
        typedef pcl::PointCloud<pcl::PointXYZ> MonochromeCloud;

        /** \brief Construct a cloud viewer, with a window name.
         * \param window_name This is displayed at the top of the window
         */
        CloudViewer (const std::string& window_name);

        /** \brief Will quit the window,
         * and release all resources held by the viewer.
         * @return
         */
        ~CloudViewer ();

        /** \brief Show a cloud, with an optional key for multiple clouds.
          * \param[in] cloud RGB point cloud
          * \param[in] cloudname a key for the point cloud, use the same name if you would like to overwrite the existing cloud.
          */
        void
        showCloud (const ColorCloud::ConstPtr &cloud, const std::string& cloudname = "cloud");

        /** \brief Show a cloud, with an optional key for multiple clouds.
          * \param[in] cloud RGB point cloud
          * \param[in] cloudname a key for the point cloud, use the same name if you would like to overwrite the existing cloud.
          */
        void
        showCloud (const ColorACloud::ConstPtr &cloud, const std::string& cloudname = "cloud");

        /** \brief Show a cloud, with an optional key for multiple clouds.
          * \param[in] cloud XYZI point cloud
          * \param[in] cloudname a key for the point cloud, use the same name if you would like to overwrite the existing cloud.
          */
        void
        showCloud (const GrayCloud::ConstPtr &cloud, const std::string& cloudname = "cloud");


        /** \brief Show a cloud, with an optional key for multiple clouds.
          * \param[in] cloud XYZ point cloud
          * \param[in] cloudname a key for the point cloud, use the same name if you would like to overwrite the existing cloud.
          */
        void
        showCloud (const MonochromeCloud::ConstPtr &cloud, const std::string& cloudname = "cloud");
        
        /** \brief Check if the gui was quit, you should quit also
         * \param millis_to_wait This will request to "spin" for the number of milliseconds, before exiting.
         * \return true if the user signaled the gui to stop
         */
        bool
        wasStopped (int millis_to_wait = 1);

        /** Visualization callable function, may be used for running things on the UI thread.
         */
        typedef boost::function1<void, pcl::visualization::PCLVisualizer&> VizCallable;

        /** \brief Run a callbable object on the UI thread. Will persist until removed
         * @param x Use boost::ref(x) for a function object that you would like to not copy
         * \param key The key for the callable -- use the same key to overwrite.
         */
        void
        runOnVisualizationThread (VizCallable x, const std::string& key = "callable");

        /** \brief Run a callbable object on the UI thread. This will run once and be removed
         * @param x Use boost::ref(x) for a function object that you would like to not copy
         */
        void
        runOnVisualizationThreadOnce (VizCallable x);

        /** \brief Remove a previously added callable object, NOP if it doesn't exist.
         * @param key the key that was registered with the callable object.
         */
        void
        removeVisualizationCallable (const std::string& key = "callable");
        
        /** \brief Register a callback function for keyboard events
          * \param[in] callback  the function that will be registered as a callback for a keyboard event
          * \param[in] cookie    user data that is passed to the callback
          * \return              connection object that allows to disconnect the callback function.
          */
        inline boost::signals2::connection 
        registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), void* cookie = NULL)
        {
          return (registerKeyboardCallback (boost::bind (callback, _1, cookie)));
        }
        
        /** \brief Register a callback function for keyboard events
          * \param[in] callback  the member function that will be registered as a callback for a keyboard event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return              connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection 
        registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie)));
        }
        
        /** \brief Register a callback function for mouse events
          * \param[in] callback  the function that will be registered as a callback for a mouse event
          * \param[in] cookie    user data that is passed to the callback
          * \return              connection object that allows to disconnect the callback function.
          */
        inline boost::signals2::connection 
        registerMouseCallback (void (*callback) (const pcl::visualization::MouseEvent&, void*), void* cookie = NULL)
        {
          return (registerMouseCallback (boost::bind (callback, _1, cookie)));
        }
        
        /** \brief Register a callback function for mouse events
          * \param[in] callback  the member function that will be registered as a callback for a mouse event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return              connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection 
        registerMouseCallback (void (T::*callback) (const pcl::visualization::MouseEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }

        
        /** \brief Register a callback function for point picking events
          * \param[in] callback  the function that will be registered as a callback for a point picking event
          * \param[in] cookie    user data that is passed to the callback
          * \return              connection object that allows to disconnect the callback function.
          */
        inline boost::signals2::connection 
        registerPointPickingCallback (void (*callback) (const pcl::visualization::PointPickingEvent&, void*), void* cookie = NULL)
        {
          return (registerPointPickingCallback (boost::bind (callback, _1, cookie)));
        }
        
        /** \brief Register a callback function for point picking events
          * \param[in] callback  the member function that will be registered as a callback for a point picking event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return              connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection 
        registerPointPickingCallback (void (T::*callback) (const pcl::visualization::PointPickingEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerPointPickingCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }
        
      private:
        /** \brief Private implementation. */
        struct CloudViewer_impl;
        std::auto_ptr<CloudViewer_impl> impl_;
        
        boost::signals2::connection 
        registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)>);

        boost::signals2::connection 
        registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)>);

        boost::signals2::connection 
        registerPointPickingCallback (boost::function<void (const pcl::visualization::PointPickingEvent&)>);
    };
  }
}

#endif /* CLOUD_VIEWER_H_ */
