/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
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

#pragma once

#include <pcl/memory.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/vtk/pcl_image_canvas_source_2d.h>
#include <pcl/visualization/vtk/pcl_context_item.h>
#include <pcl/geometry/planar_polygon.h>
#include <pcl/correspondence.h>

#include <boost/shared_array.hpp>

#include <vtkVersion.h>
#include <vtkInteractorStyleImage.h>
#include <vtkRenderWindowInteractor.h>

class vtkImageSlice;
class vtkContextActor;
class vtkImageViewer;
class vtkImageFlip;

namespace pcl
{
  namespace visualization
  {
    using Vector3ub = Eigen::Array<unsigned char, 3, 1>;
    static const Vector3ub green_color (0, 255, 0);
    static const Vector3ub red_color (255, 0, 0);
    static const Vector3ub blue_color (0, 0, 255);

    /** \brief An image viewer interactor style, tailored for ImageViewer.
      * \author Radu B. Rusu
      * \ingroup visualization
      */
    class PCL_EXPORTS ImageViewerInteractorStyle : public vtkInteractorStyleImage
    {
      public:
        static ImageViewerInteractorStyle *New ();
        ImageViewerInteractorStyle ();

        void OnMouseWheelForward () override {}
        void OnMouseWheelBackward () override {}
        void OnMiddleButtonDown () override {}
        void OnRightButtonDown () override {}
        void OnLeftButtonDown () override;

        void
        OnChar () override;

        void
        adjustCamera (vtkImageData *image, vtkRenderer *ren);

        void
        adjustCamera (vtkRenderer *ren);
    };

    /** \brief ImageViewer is a class for 2D image visualization.
      *
      * Features include:
      *  - add and remove different layers with different opacity (transparency) values
      *  - add 2D geometric shapes (circles, boxes, etc) in separate layers
      *  - display RGB, monochrome, float, angle images
      *
      * Simple usage example:
      * \code
      * pcl::visualization::ImageViewer iv;
      * iv.addCircle (10, 10, 5, 1.0, 0.0, 0.0, "circles", 1.0);    // add a red, fully opaque circle with radius 5 pixels at (10,10) in layer "circles"
      * iv.addFilledRectangle (10, 20, 10, 20, 0.0, 1.0, 0.0, "boxes", 0.5);    // add a green, 50% transparent box at (10,10->20,20) in layer "boxes"
      * iv.addRGBImage<pcl::PointXYZRGBA> (cloud);                  // add a RGB image from a point cloud dataset in an "rgb_image" default layer
      * iv.spin ();                                                 // press 'q' to exit
      * iv.removeLayer ("circles");                                 // remove layer "circles"
      * iv.spin ();                                                 // press 'q' to exit
      * \endcode
      * 
      * \author Radu B. Rusu, Suat Gedikli
      * \ingroup visualization
      */
    class PCL_EXPORTS ImageViewer
    {
      public:
        using Ptr = shared_ptr<ImageViewer>;
        using ConstPtr = shared_ptr<const ImageViewer>;

        /** \brief Constructor.
          * \param[in] window_title the title of the window
          */
        ImageViewer (const std::string& window_title = "");

        /** \brief Destructor. */
        virtual ~ImageViewer ();
       
        /** \brief Set up the interactor style. By default the interactor style is set to
          * vtkInteractorStyleImage you can use this to set it to another type.
          * \param[in] style user set interactor style.
          */
        void
        setInteractorStyle (vtkInteractorObserver *style)
        {
          interactor_->SetInteractorStyle (style);
        }

        /** \brief Show a monochrome 2D image on screen.
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        showMonoImage (const unsigned char* data, unsigned width, unsigned height,
                       const std::string &layer_id = "mono_image", double opacity = 1.0);

        /** \brief Add a monochrome 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        addMonoImage (const unsigned char* data, unsigned width, unsigned height,
                      const std::string &layer_id = "mono_image", double opacity = 1.0);

        /** \brief Show a monochrome 2D image on screen.
          * \param[in] cloud the input data representing the grayscale point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        inline void
        showMonoImage (const pcl::PointCloud<pcl::Intensity>::ConstPtr &cloud,
                      const std::string &layer_id = "mono_image", double opacity = 1.0)
        {
          return (showMonoImage (*cloud, layer_id, opacity));
        }

        /** \brief Add a monochrome 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] cloud the input data representing the grayscale point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        inline void
        addMonoImage (const pcl::PointCloud<pcl::Intensity>::ConstPtr &cloud,
                     const std::string &layer_id = "mono_image", double opacity = 1.0)
        {
          return (addMonoImage (*cloud, layer_id, opacity));
        }

        /** \brief Show a monochrome 2D image on screen.
          * \param[in] cloud the input data representing the grayscale point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        showMonoImage (const pcl::PointCloud<pcl::Intensity> &cloud,
                      const std::string &layer_id = "mono_image", double opacity = 1.0);

        /** \brief Add a monochrome 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] cloud the input data representing the RGB point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        addMonoImage (const pcl::PointCloud<pcl::Intensity> &cloud,
                     const std::string &layer_id = "mono_image", double opacity = 1.0);

        /** \brief Show a monochrome 2D image on screen.
          * \param[in] cloud the input data representing the grayscale point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        inline void
        showMonoImage (const pcl::PointCloud<pcl::Intensity8u>::ConstPtr &cloud,
                      const std::string &layer_id = "mono_image", double opacity = 1.0)
        {
          return (showMonoImage (*cloud, layer_id, opacity));
        }

        /** \brief Add a monochrome 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] cloud the input data representing the grayscale point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        inline void
        addMonoImage (const pcl::PointCloud<pcl::Intensity8u>::ConstPtr &cloud,
                     const std::string &layer_id = "mono_image", double opacity = 1.0)
        {
          return (addMonoImage (*cloud, layer_id, opacity));
        }

        /** \brief Show a monochrome 2D image on screen.
          * \param[in] cloud the input data representing the grayscale point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        showMonoImage (const pcl::PointCloud<pcl::Intensity8u> &cloud,
                      const std::string &layer_id = "mono_image", double opacity = 1.0);

        /** \brief Add a monochrome 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] cloud the input data representing the RGB point cloud
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        addMonoImage (const pcl::PointCloud<pcl::Intensity8u> &cloud,
                     const std::string &layer_id = "mono_image", double opacity = 1.0);

        /** \brief Show a 2D RGB image on screen.
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        showRGBImage (const unsigned char* data, unsigned width, unsigned height, 
                      const std::string &layer_id = "rgb_image", double opacity = 1.0);

        /** \brief Add an RGB 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          * \param[in] autoresize flag to enable window to adapt to image size (default true)
          */
        void 
        addRGBImage (const unsigned char* data, unsigned width, unsigned height, 
                     const std::string &layer_id = "rgb_image", double opacity = 1.0,
                     bool autoresize = true);

        /** \brief Show a 2D image on screen, obtained from the RGB channel of a point cloud.
          * \param[in] cloud the input data representing the RGB point cloud 
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        template <typename T> inline void 
        showRGBImage (const typename pcl::PointCloud<T>::ConstPtr &cloud,
                      const std::string &layer_id = "rgb_image", double opacity = 1.0)
        {
          return (showRGBImage<T> (*cloud, layer_id, opacity));
        }

        /** \brief Add an RGB 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] cloud the input data representing the RGB point cloud 
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        template <typename T> inline void 
        addRGBImage (const typename pcl::PointCloud<T>::ConstPtr &cloud,
                     const std::string &layer_id = "rgb_image", double opacity = 1.0)
        {
          return (addRGBImage<T> (*cloud, layer_id, opacity));
        }

        /** \brief Show a 2D image on screen, obtained from the RGB channel of a point cloud.
          * \param[in] cloud the input data representing the RGB point cloud 
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        template <typename T> void 
        showRGBImage (const pcl::PointCloud<T> &cloud,
                      const std::string &layer_id = "rgb_image", double opacity = 1.0);

        /** \brief Add an RGB 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] cloud the input data representing the RGB point cloud 
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        template <typename T> void 
        addRGBImage (const pcl::PointCloud<T> &cloud,
                     const std::string &layer_id = "rgb_image", double opacity = 1.0);

        /** \brief Show a 2D image (float) on screen.
          * \param[in] data the input data representing the image in float format
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] min_value filter all values in the image to be larger than this minimum value
          * \param[in] max_value filter all values in the image to be smaller than this maximum value
          * \param[in] grayscale show data as grayscale (true) or not (false). Default: false
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        showFloatImage (const float* data, unsigned int width, unsigned int height, 
                        float min_value = std::numeric_limits<float>::min (), 
                        float max_value = std::numeric_limits<float>::max (), bool grayscale = false,
                        const std::string &layer_id = "float_image", double opacity = 1.0);

        /** \brief Add a float 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] data the input data representing the image in float format
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] min_value filter all values in the image to be larger than this minimum value
          * \param[in] max_value filter all values in the image to be smaller than this maximum value
          * \param[in] grayscale show data as grayscale (true) or not (false). Default: false
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        addFloatImage (const float* data, unsigned int width, unsigned int height, 
                       float min_value = std::numeric_limits<float>::min (), 
                       float max_value = std::numeric_limits<float>::max (), bool grayscale = false,
                       const std::string &layer_id = "float_image", double opacity = 1.0);
        
        /** \brief Show a 2D image (unsigned short) on screen.
          * \param[in] short_image the input data representing the image in unsigned short format
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] min_value filter all values in the image to be larger than this minimum value
          * \param[in] max_value filter all values in the image to be smaller than this maximum value
          * \param[in] grayscale show data as grayscale (true) or not (false). Default: false
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        showShortImage (const unsigned short* short_image, unsigned int width, unsigned int height, 
                        unsigned short min_value = std::numeric_limits<unsigned short>::min (), 
                        unsigned short max_value = std::numeric_limits<unsigned short>::max (), bool grayscale = false,
                        const std::string &layer_id = "short_image", double opacity = 1.0);

        /** \brief Add a short 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] short_image the input data representing the image in unsigned short format
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] min_value filter all values in the image to be larger than this minimum value
          * \param[in] max_value filter all values in the image to be smaller than this maximum value
          * \param[in] grayscale show data as grayscale (true) or not (false). Default: false
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        addShortImage (const unsigned short* short_image, unsigned int width, unsigned int height, 
                       unsigned short min_value = std::numeric_limits<unsigned short>::min (), 
                       unsigned short max_value = std::numeric_limits<unsigned short>::max (), bool grayscale = false,
                       const std::string &layer_id = "short_image", double opacity = 1.0);

        /** \brief Show a 2D image on screen representing angle data.
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        showAngleImage (const float* data, unsigned width, unsigned height,
                        const std::string &layer_id = "angle_image", double opacity = 1.0);

        /** \brief Add an angle 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        addAngleImage (const float* data, unsigned width, unsigned height,
                       const std::string &layer_id = "angle_image", double opacity = 1.0);

        /** \brief Show a 2D image on screen representing half angle data.
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        showHalfAngleImage (const float* data, unsigned width, unsigned height,
                            const std::string &layer_id = "half_angle_image", double opacity = 1.0);

        /** \brief Add a half angle 2D image layer, but do not render it (use spin/spinOnce to update).
          * \param[in] data the input data representing the image
          * \param[in] width the width of the image
          * \param[in] height the height of the image
          * \param[in] layer_id the name of the layer (default: "image")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void 
        addHalfAngleImage (const float* data, unsigned width, unsigned height,
                           const std::string &layer_id = "half_angle_image", double opacity = 1.0);

        /** \brief Sets the pixel at coordinates(u,v) to color while setting the neighborhood to another
          * \param[in] u the u/x coordinate of the pixel
          * \param[in] v the v/y coordinate of the pixel
          * \param[in] fg_color the pixel color
          * \param[in] bg_color the neighborhood color
          * \param[in] radius the circle radius around the pixel
          * \param[in] layer_id the name of the layer (default: "points")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        markPoint (std::size_t u, std::size_t v, Vector3ub fg_color, Vector3ub bg_color = red_color, double radius = 3.0,
                   const std::string &layer_id = "points", double opacity = 1.0);

        /** \brief Sets the pixel at coordinates(u,v) to color while setting the neighborhood to another
          * \param[in] uv the u/x, v/y coordinate of the pixels to be marked
          * \param[in] fg_color the pixel color
          * \param[in] bg_color the neighborhood color
          * \param[in] size edge of the square surrounding each pixel
          * \param[in] layer_id the name of the layer (default: "markers")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        markPoints (const std::vector<int>& uv, Vector3ub fg_color, Vector3ub bg_color = red_color, double size = 3.0,
                    const std::string &layer_id = "markers", double opacity = 1.0);

        /** \brief Sets the pixel at coordinates(u,v) to color while setting the neighborhood to another (float coordinates version).
          * \param[in] uv the u/x, v/y coordinate of the pixels to be marked
          * \param[in] fg_color the pixel color
          * \param[in] bg_color the neighborhood color
          * \param[in] size edge of the square surrounding each pixel
          * \param[in] layer_id the name of the layer (default: "markers")
          * \param[in] opacity the opacity of the layer (default: 1.0)
          */
        void
        markPoints (const std::vector<float>& uv, Vector3ub fg_color, Vector3ub bg_color = red_color, double size = 3.0,
                    const std::string &layer_id = "markers", double opacity = 1.0);

        /** \brief Set the window title name
          * \param[in] name the window title
          */
        void
        setWindowTitle (const std::string& name);

        /** \brief Spin method. Calls the interactor and runs an internal loop. */
        void 
        spin ();
        
        /** \brief Spin once method. Calls the interactor and updates the screen once. 
          * \param[in] time - How long (in ms) should the visualization loop be allowed to run.
          * \param[in] force_redraw - if false it might return without doing anything if the 
          * interactor's framerate does not require a redraw yet.
          */
        void 
        spinOnce (int time = 1, bool force_redraw = true);
        
        /** \brief Register a callback function for keyboard events
          * \param[in] callback  the function that will be registered as a callback for a keyboard event
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerKeyboardCallback (void (*callback) (const pcl::visualization::KeyboardEvent&, void*), 
                                  void* cookie = nullptr)
        {
          return (registerKeyboardCallback ([=] (const pcl::visualization::KeyboardEvent& e) { (*callback) (e, cookie); }));
        }
        
        /** \brief Register a callback function for keyboard events
          * \param[in] callback  the member function that will be registered as a callback for a keyboard event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> boost::signals2::connection 
        registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), 
                                  T& instance, void* cookie = nullptr)
        {
          return (registerKeyboardCallback ([=, &instance] (const pcl::visualization::KeyboardEvent& e) { (instance.*callback) (e, cookie); }));
        }
        
        /** \brief Register a callback std::function for keyboard events
          * \param[in] cb the boost function that will be registered as a callback for a keyboard event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerKeyboardCallback (std::function<void (const pcl::visualization::KeyboardEvent&)> cb);

        /** \brief Register a callback std::function for mouse events
          * \param[in] callback  the function that will be registered as a callback for a mouse event
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerMouseCallback (void (*callback) (const pcl::visualization::MouseEvent&, void*), 
                               void* cookie = nullptr)
        {
          return (registerMouseCallback ([=] (const pcl::visualization::MouseEvent& e) { (*callback) (e, cookie); }));
        }
        
        /** \brief Register a callback function for mouse events
          * \param[in] callback  the member function that will be registered as a callback for a mouse event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> boost::signals2::connection 
        registerMouseCallback (void (T::*callback) (const pcl::visualization::MouseEvent&, void*), 
                               T& instance, void* cookie = nullptr)
        {
          return (registerMouseCallback ([=, &instance] (const pcl::visualization::MouseEvent& e) { (instance.*callback) (e, cookie); }));
        }

        /** \brief Register a callback function for mouse events
          * \param[in] cb the boost function that will be registered as a callback for a mouse event
          * \return a connection object that allows to disconnect the callback function.
          */        
        boost::signals2::connection 
        registerMouseCallback (std::function<void (const pcl::visualization::MouseEvent&)> cb);
        
        /** \brief Set the position in screen coordinates.
          * \param[in] x where to move the window to (X)
          * \param[in] y where to move the window to (Y)
          */
        void
        setPosition (int x, int y);

        /** \brief Set the window size in screen coordinates.
          * \param[in] xw window size in horizontal (pixels)
          * \param[in] yw window size in vertical (pixels)
          */
        void
        setSize (int xw, int yw);

        /** \brief Return the window size in pixels. */
        int*
        getSize ();

        /** \brief Returns true when the user tried to close the window */
        bool
        wasStopped () const { return (stopped_); }

        /** \brief Stop the interaction and close the visualizaton window. */
        void
        close ()
        {
          stopped_ = true;
          // This tends to close the window...
          interactor_->TerminateApp ();
        }

        /** \brief Add a circle shape from a point and a radius
          * \param[in] x the x coordinate of the circle center
          * \param[in] y the y coordinate of the circle center
          * \param[in] radius the radius of the circle
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addCircle (unsigned int x, unsigned int y, double radius, 
                   const std::string &layer_id = "circles", double opacity = 1.0);

        /** \brief Add a circle shape from a point and a radius
          * \param[in] x the x coordinate of the circle center
          * \param[in] y the y coordinate of the circle center
          * \param[in] radius the radius of the circle
          * \param[in] r the red channel of the color that the sphere should be rendered with (0.0 -> 1.0)
          * \param[in] g the green channel of the color that the sphere should be rendered with (0.0 -> 1.0)
          * \param[in] b the blue channel of the color that the sphere should be rendered with (0.0 -> 1.0)
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addCircle (unsigned int x, unsigned int y, double radius, 
                   double r, double g, double b,
                   const std::string &layer_id = "circles", double opacity = 1.0);

        /** \brief Add a 2D box and color its edges with a given color
          * \param[in] min_pt the X,Y min coordinate
          * \param[in] max_pt the X,Y max coordinate
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addRectangle (const pcl::PointXY &min_pt, const pcl::PointXY &max_pt,
                      const std::string &layer_id = "rectangles", double opacity = 1.0);

        /** \brief Add a 2D box and color its edges with a given color
          * \param[in] min_pt the X,Y min coordinate
          * \param[in] max_pt the X,Y max coordinate
          * \param[in] r the red channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] g the green channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] b the blue channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addRectangle (const pcl::PointXY &min_pt, const pcl::PointXY &max_pt,
                      double r, double g, double b,
                      const std::string &layer_id = "rectangles", double opacity = 1.0);

        /** \brief Add a 2D box and color its edges with a given color
          * \param[in] x_min the X min coordinate
          * \param[in] x_max the X max coordinate
          * \param[in] y_min the Y min coordinate
          * \param[in] y_max the Y max coordinate 
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addRectangle (unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max,  
                      const std::string &layer_id = "rectangles", double opacity = 1.0);

        /** \brief Add a 2D box and color its edges with a given color
          * \param[in] x_min the X min coordinate
          * \param[in] x_max the X max coordinate
          * \param[in] y_min the Y min coordinate
          * \param[in] y_max the Y max coordinate 
          * \param[in] r the red channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] g the green channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] b the blue channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addRectangle (unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max,  
                      double r, double g, double b,
                      const std::string &layer_id = "rectangles", double opacity = 1.0);

        /** \brief Add a 2D box and color its edges with a given color
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] min_pt the X,Y min coordinate
          * \param[in] max_pt the X,Y max coordinate
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        template <typename T> bool
        addRectangle (const typename pcl::PointCloud<T>::ConstPtr &image, 
                      const T &min_pt, const T &max_pt,
                      const std::string &layer_id = "rectangles", double opacity = 1.0);

        /** \brief Add a 2D box and color its edges with a given color
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] min_pt the X,Y min coordinate
          * \param[in] max_pt the X,Y max coordinate
          * \param[in] r the red channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] g the green channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] b the blue channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        template <typename T> bool
        addRectangle (const typename pcl::PointCloud<T>::ConstPtr &image, 
                      const T &min_pt, const T &max_pt,
                      double r, double g, double b,
                      const std::string &layer_id = "rectangles", double opacity = 1.0);

        /** \brief Add a 2D box that contains a given image mask and color its edges
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] mask the point data representing the mask that we want to draw
          * \param[in] r the red channel of the color that the mask should be rendered with 
          * \param[in] g the green channel of the color that the mask should be rendered with
          * \param[in] b the blue channel of the color that the mask should be rendered with
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        template <typename T> bool
        addRectangle (const typename pcl::PointCloud<T>::ConstPtr &image, const pcl::PointCloud<T> &mask, 
                      double r, double g, double b, 
                      const std::string &layer_id = "rectangles", double opacity = 1.0);

        /** \brief Add a 2D box that contains a given image mask and color its edges in red
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] mask the point data representing the mask that we want to draw
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        template <typename T> bool
        addRectangle (const typename pcl::PointCloud<T>::ConstPtr &image, const pcl::PointCloud<T> &mask, 
                      const std::string &layer_id = "image_mask", double opacity = 1.0);

        /** \brief Add a 2D box and fill it in with a given color
          * \param[in] x_min the X min coordinate
          * \param[in] x_max the X max coordinate
          * \param[in] y_min the Y min coordinate
          * \param[in] y_max the Y max coordinate 
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 0.5)
          */
        bool
        addFilledRectangle (unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max,  
                            const std::string &layer_id = "boxes", double opacity = 0.5);

        /** \brief Add a 2D box and fill it in with a given color
          * \param[in] x_min the X min coordinate
          * \param[in] x_max the X max coordinate
          * \param[in] y_min the Y min coordinate
          * \param[in] y_max the Y max coordinate 
          * \param[in] r the red channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] g the green channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] b the blue channel of the color that the box should be rendered with (0.0 -> 1.0)
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 0.5)
          */
        bool
        addFilledRectangle (unsigned int x_min, unsigned int x_max, unsigned int y_min, unsigned int y_max,  
                            double r, double g, double b,
                            const std::string &layer_id = "boxes", double opacity = 0.5);

        /** \brief Add a 2D line with a given color
          * \param[in] x_min the X min coordinate
          * \param[in] y_min the Y min coordinate
          * \param[in] x_max the X max coordinate
          * \param[in] y_max the Y max coordinate 
          * \param[in] r the red channel of the color that the line should be rendered with (0.0 -> 1.0)
          * \param[in] g the green channel of the color that the line should be rendered with (0.0 -> 1.0)
          * \param[in] b the blue channel of the color that the line should be rendered with (0.0 -> 1.0)
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addLine (unsigned int x_min, unsigned int y_min, unsigned int x_max, unsigned int y_max,
                 double r, double g, double b, 
                 const std::string &layer_id = "line", double opacity = 1.0);

        /** \brief Add a 2D line with a given color
          * \param[in] x_min the X min coordinate
          * \param[in] y_min the Y min coordinate
          * \param[in] x_max the X max coordinate
          * \param[in] y_max the Y max coordinate 
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn. 
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addLine (unsigned int x_min, unsigned int y_min, unsigned int x_max, unsigned int y_max,
                 const std::string &layer_id = "line", double opacity = 1.0);

        /** \brief Add a 2D text with a given color
          * \param[in] x the X coordinate
          * \param[in] y the Y coordinate
          * \param[in] text the text string to be displayed
          * \param[in] r the red channel of the color that the line should be rendered with (0.0 -> 1.0)
          * \param[in] g the green channel of the color that the line should be rendered with (0.0 -> 1.0)
          * \param[in] b the blue channel of the color that the line should be rendered with (0.0 -> 1.0)
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addText (unsigned int x, unsigned int y, const std::string& text,
                 double r, double g, double b,
                 const std::string &layer_id = "line", double opacity = 1.0);

        /** \brief Add a 2D text with a given color
          * \param[in] x the X coordinate
          * \param[in] y the Y coordinate
          * \param[in] text the text string to be displayed
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        bool
        addText (unsigned int x, unsigned int y, const std::string& text,
                 const std::string &layer_id = "line", double opacity = 1.0);

        /** \brief Add a generic 2D mask to an image 
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] mask the point data representing the mask that we want to draw
          * \param[in] r the red channel of the color that the mask should be rendered with 
          * \param[in] g the green channel of the color that the mask should be rendered with
          * \param[in] b the blue channel of the color that the mask should be rendered with
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 0.5)
          */
        template <typename T> bool
        addMask (const typename pcl::PointCloud<T>::ConstPtr &image, const pcl::PointCloud<T> &mask, 
                 double r, double g, double b, 
                 const std::string &layer_id = "image_mask", double opacity = 0.5);

        /** \brief Add a generic 2D mask to an image (colored in red)
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] mask the point data representing the mask that we want to draw
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 0.5)
          */
        template <typename T> bool
        addMask (const typename pcl::PointCloud<T>::ConstPtr &image, const pcl::PointCloud<T> &mask, 
                 const std::string &layer_id = "image_mask", double opacity = 0.5);

        /** \brief Add a generic 2D planar polygon to an image 
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] polygon the point data representing the polygon that we want to draw. 
          * A line will be drawn from each point to the next in the dataset.
          * \param[in] r the red channel of the color that the polygon should be rendered with 
          * \param[in] g the green channel of the color that the polygon should be rendered with
          * \param[in] b the blue channel of the color that the polygon should be rendered with
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        template <typename T> bool
        addPlanarPolygon (const typename pcl::PointCloud<T>::ConstPtr &image, const pcl::PlanarPolygon<T> &polygon, 
                          double r, double g, double b, 
                          const std::string &layer_id = "planar_polygon", double opacity = 1.0);

        /** \brief Add a generic 2D planar polygon to an image 
          * \param[in] image the organized point cloud dataset containing the image data
          * \param[in] polygon the point data representing the polygon that we want to draw. 
          * A line will be drawn from each point to the next in the dataset.
          * \param[in] layer_id the 2D layer ID where we want the extra information to be drawn.
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 1.0)
          */
        template <typename T> bool
        addPlanarPolygon (const typename pcl::PointCloud<T>::ConstPtr &image, const pcl::PlanarPolygon<T> &polygon, 
                          const std::string &layer_id = "planar_polygon", double opacity = 1.0);

        /** \brief Add a new 2D rendering layer to the viewer. 
          * \param[in] layer_id the name of the layer
          * \param[in] width the width of the layer
          * \param[in] height the height of the layer
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 0.5)
          */
        bool
        addLayer (const std::string &layer_id, int width, int height, double opacity = 0.5);

        /** \brief Remove a 2D layer given by its ID.
          * \param[in] layer_id the name of the layer
          */
        void
        removeLayer (const std::string &layer_id);

        /** \brief Add the specified correspondences to the display.
          * \param[in] source_img The source RGB image
          * \param[in] target_img The target RGB image
          * \param[in] correspondences The list of correspondences to display.
          * \param[in] nth display only the Nth correspondence (e.g., skip the rest)
          * \param[in] layer_id the layer id (default: "correspondences")
          */
        template <typename PointT> bool
        showCorrespondences (const pcl::PointCloud<PointT> &source_img,
                             const pcl::PointCloud<PointT> &target_img,
                             const pcl::Correspondences &correspondences,
                             int nth = 1,
                             const std::string &layer_id = "correspondences");

      protected:
        /** \brief Trigger a render call. */
        void
        render ();

        /** \brief Convert the Intensity information in a PointCloud<Intensity> to an unsigned char array
          * \param[in] cloud the input cloud containing the grayscale intensity information
          * \param[out] data a boost shared array of unsigned char type
          * \note The method assumes that the data array has already been allocated and
          * contains enough space to copy all the data from cloud!
          */
        void
        convertIntensityCloudToUChar (const pcl::PointCloud<pcl::Intensity> &cloud,
                                boost::shared_array<unsigned char> data);

        /** \brief Convert the Intensity8u information in a PointCloud<Intensity8u> to an unsigned char array
          * \param[in] cloud the input cloud containing the grayscale intensity information
          * \param[out] data a boost shared array of unsigned char type
          * \note The method assumes that the data array has already been allocated and
          * contains enough space to copy all the data from cloud!
          */
        void
        convertIntensityCloud8uToUChar (const pcl::PointCloud<pcl::Intensity8u> &cloud,
                                boost::shared_array<unsigned char> data);

        /** \brief Convert the RGB information in a PointCloud<T> to an unsigned char array
          * \param[in] cloud the input cloud containing the RGB information
          * \param[out] data a boost shared array of unsigned char type
          * \note The method assumes that the data array has already been allocated and
          * contains enough space to copy all the data from cloud!
          */
        template <typename T> void
        convertRGBCloudToUChar (const pcl::PointCloud<T> &cloud,
                                boost::shared_array<unsigned char> &data);

        /** \brief Set the stopped flag back to false */
        void
        resetStoppedFlag () { stopped_ = false; }

        /** \brief Fire up a mouse event with a specified event ID
          * \param[in] event_id the id of the event
          */
        void 
        emitMouseEvent (unsigned long event_id);
        
        /** \brief Fire up a keyboard event with a specified event ID
          * \param[in] event_id the id of the event
          */
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
          ExitMainLoopTimerCallback () : right_timer_id (), window () {}

          static ExitMainLoopTimerCallback* New ()
          {
            return (new ExitMainLoopTimerCallback);
          }
          void 
          Execute (vtkObject* vtkNotUsed (caller), unsigned long event_id, void* call_data) override
          {
            if (event_id != vtkCommand::TimerEvent)
              return;
            int timer_id = *static_cast<int*> (call_data);
            if (timer_id != right_timer_id)
              return;
            window->interactor_->TerminateApp ();
          }
          int right_timer_id;
          ImageViewer* window;
        };
        struct ExitCallback : public vtkCommand
        {
          ExitCallback () : window () {}

          static ExitCallback* New ()
          {
            return (new ExitCallback);
          }
          void 
          Execute (vtkObject*, unsigned long event_id, void*) override
          {
            if (event_id != vtkCommand::ExitEvent)
              return;
            window->stopped_ = true;
            window->interactor_->TerminateApp ();
          }
          ImageViewer* window;
        };

    private:
        /** \brief Internal structure describing a layer. */
        struct Layer
        {
          Layer () = default;
          vtkSmartPointer<vtkContextActor> actor;
          std::string layer_name;
        };

        using LayerMap = std::vector<Layer>;

        /** \brief Add a new 2D rendering layer to the viewer. 
          * \param[in] layer_id the name of the layer
          * \param[in] width the width of the layer
          * \param[in] height the height of the layer
          * \param[in] opacity the opacity of the layer: 0 for invisible, 1 for opaque. (default: 0.5)
          * \param[in] fill_box set to true to fill in the image with one black box before starting
          */
        LayerMap::iterator
        createLayer (const std::string &layer_id, int width, int height, double opacity = 0.5, bool fill_box = true);

        boost::signals2::signal<void (const pcl::visualization::MouseEvent&)> mouse_signal_;
        boost::signals2::signal<void (const pcl::visualization::KeyboardEvent&)> keyboard_signal_;
        
        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
        vtkSmartPointer<vtkCallbackCommand> mouse_command_;
        vtkSmartPointer<vtkCallbackCommand> keyboard_command_;

        /** \brief Callback object enabling us to leave the main loop, when a timer fires. */
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
        vtkSmartPointer<ExitCallback> exit_callback_;

        /** \brief The ImageViewer widget. */
        vtkSmartPointer<vtkImageViewer> image_viewer_;

        /** \brief The render window. */
        vtkSmartPointer<vtkRenderWindow> win_;

        /** \brief The renderer. */
        vtkSmartPointer<vtkRenderer> ren_;

        /** \brief Global prop. This is the actual "actor". */
        vtkSmartPointer<vtkImageSlice> slice_;

        /** \brief The interactor style. */
        vtkSmartPointer<ImageViewerInteractorStyle> interactor_style_;

        /** \brief The data array representing the image. Used internally. */
        boost::shared_array<unsigned char> data_;
  
        /** \brief The data array (representing the image) size. Used internally. */
        std::size_t data_size_;

        /** \brief Set to false if the interaction loop is running. */
        bool stopped_;

        /** \brief Global timer ID. Used in destructor only. */
        int timer_id_;

        // /** \brief Internal blender used to overlay 2D geometry over the image. */
        // vtkSmartPointer<vtkImageBlend> blend_;
 
        /** \brief Internal list with different 2D layers shapes. */
        LayerMap layer_map_;

        /** \brief Image reslice, used for flipping the image. */
        vtkSmartPointer<vtkImageFlip> algo_;

        /** \brief Internal data array. Used everytime add***Image is called. 
          * Cleared, everytime the render loop is executed. 
          */
        std::vector<unsigned char*> image_data_;

        struct LayerComparator
        {
          LayerComparator (const std::string &str) : str_ (str) {}
          const std::string &str_;

          bool
          operator () (const Layer &layer)
          {
            return (layer.layer_name == str_);
          }
        };

      public:
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
  }
}

#include <pcl/visualization/impl/image_viewer.hpp>
