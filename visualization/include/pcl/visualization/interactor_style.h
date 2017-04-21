/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 * $Id$
 *
 */
#ifndef PCL_PCL_VISUALIZER_INTERACTOR_STYLE_H_
#define PCL_PCL_VISUALIZER_INTERACTOR_STYLE_H_

#include <pcl/console/print.h>
#include <pcl/visualization/common/actor_map.h>
#include <pcl/visualization/common/ren_win_interact_map.h>
#include <pcl/visualization/keyboard_event.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/area_picking_event.h>
#ifndef Q_MOC_RUN
#include <boost/signals2/signal.hpp>
#endif
#include <vtkInteractorStyleRubberBandPick.h>

class vtkRendererCollection;
class vtkLegendScaleActor;
class vtkScalarBarActor;
class vtkPNGWriter;
class vtkWindowToImageFilter;
class vtkPointPicker;

namespace pcl
{
  namespace visualization
  {

    /** \brief A list of potential keyboard modifiers for \ref pcl::visualization::PCLVisualizerInteractorStyle::PCLVisualizerInteractorStyle()
      * Defaults to Alt. 
      */ 
    enum InteractorKeyboardModifier
    {
      INTERACTOR_KB_MOD_ALT,
      INTERACTOR_KB_MOD_CTRL,
      INTERACTOR_KB_MOD_SHIFT
    };

    /** \brief PCLVisualizerInteractorStyle defines an unique, custom VTK
      * based interactory style for PCL Visualizer applications. Besides
      * defining the rendering style, we also create a list of custom actions
      * that are triggered on different keys being pressed:
      *
      * -        p, P   : switch to a point-based representation
      * -        w, W   : switch to a wireframe-based representation (where available)
      * -        s, S   : switch to a surface-based representation (where available)
      * -        j, J   : take a .PNG snapshot of the current window view
      * -        c, C   : display current camera/window parameters
      * -        f, F   : fly to point mode
      * -        e, E   : exit the interactor\
      * -        q, Q   : stop and call VTK's TerminateApp
      * -       + / -   : increment/decrement overall point size
      * -        g, G   : display scale grid (on/off)
      * -        u, U   : display lookup table (on/off)
      * -  r, R [+ ALT] : reset camera [to viewpoint = {0, 0, 0} -> center_{x, y, z}]
      * -  CTRL + s, S  : save camera parameters
      * -  CTRL + r, R  : restore camera parameters
      * -  ALT + s, S   : turn stereo mode on/off
      * -  ALT + f, F   : switch between maximized window mode and original size
      * -        l, L           : list all available geometric and color handlers for the current actor map
      * -  ALT + 0..9 [+ CTRL]  : switch between different geometric handlers (where available)
      * -        0..9 [+ CTRL]  : switch between different color handlers (where available)
      * - 
      * -  SHIFT + left click   : select a point
      * -        x, X   : toggle rubber band selection mode for left mouse button
      *
      * \author Radu B. Rusu
      * \ingroup visualization
      */
    class PCL_EXPORTS PCLVisualizerInteractorStyle : public vtkInteractorStyleRubberBandPick
    {
      typedef boost::shared_ptr<CloudActorMap> CloudActorMapPtr;

      public:
        static PCLVisualizerInteractorStyle *New ();

        /** \brief Empty constructor. */
        PCLVisualizerInteractorStyle () : 
          init_ (), rens_ (), cloud_actors_ (), shape_actors_ (), win_height_ (), win_width_ (), win_pos_x_ (), win_pos_y_ (),
          max_win_height_ (), max_win_width_ (), use_vbos_ (false), grid_enabled_ (), grid_actor_ (), lut_enabled_ (),
          lut_actor_ (), snapshot_writer_ (), wif_ (), mouse_signal_ (), keyboard_signal_ (),
          point_picking_signal_ (), area_picking_signal_ (), stereo_anaglyph_mask_default_ (),
          mouse_callback_ (), modifier_ (), camera_file_ (), camera_ (), camera_saved_ (), win_ (), lut_actor_id_ ("")
        {}
      
        /** \brief Empty destructor */
        virtual ~PCLVisualizerInteractorStyle () {}

        // this macro defines Superclass, the isA functionality and the safe downcast method
        vtkTypeMacro (PCLVisualizerInteractorStyle, vtkInteractorStyleRubberBandPick);
        
        /** \brief Initialization routine. Must be called before anything else. */
        virtual void 
        Initialize ();
        
        /** \brief Pass a pointer to the cloud actor map
          * \param[in] actors the actor map that will be used with this style
          */
        inline void
        setCloudActorMap (const CloudActorMapPtr &actors) { cloud_actors_ = actors; }

        /** \brief Pass a pointer to the shape actor map
          * \param[in] actors the actor map that will be used with this style
          */
        inline void 
        setShapeActorMap (const ShapeActorMapPtr &actors) { shape_actors_ = actors; }

        /** \brief Get the cloud actor map pointer. */
        inline CloudActorMapPtr 
        getCloudActorMap () { return (cloud_actors_); }

        /** \brief Get the cloud actor map pointer. */
        inline ShapeActorMapPtr
        getShapeActorMap () { return (shape_actors_); }

        /** \brief Pass a set of renderers to the interactor style. 
          * \param[in] rens the vtkRendererCollection to use
          */
        void 
        setRendererCollection (vtkSmartPointer<vtkRendererCollection> &rens) { rens_ = rens; }

        /** \brief Use Vertex Buffer Objects renderers.
          * This is an optimization for the obsolete OpenGL backend. Modern OpenGL2 backend (VTK ≥ 6.3) uses vertex
          * buffer objects by default, transparently for the user.
          * \param[in] use_vbos set to true to use VBOs
          */
        inline void
        setUseVbos (const bool use_vbos) { use_vbos_ = use_vbos; }

        /** \brief Register a callback function for mouse events
          * \param[in] cb a boost function that will be registered as a callback for a mouse event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> cb);

        /** \brief Register a callback boost::function for keyboard events
          * \param[in] cb a boost function that will be registered as a callback for a keyboard event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> cb);

        /** \brief Register a callback function for point picking events
          * \param[in] cb a boost function that will be registered as a callback for a point picking event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection 
        registerPointPickingCallback (boost::function<void (const pcl::visualization::PointPickingEvent&)> cb);

        /** \brief Register a callback function for area picking events
          * \param[in] cb a boost function that will be registered as a callback for a area picking event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerAreaPickingCallback (boost::function<void (const pcl::visualization::AreaPickingEvent&)> cb);

        /** \brief Save the current rendered image to disk, as a PNG screenshot.
          * \param[in] file the name of the PNG file
          */
        void
        saveScreenshot (const std::string &file);

        /** \brief Save the camera parameters to disk, as a .cam file.
          * \param[in] file the name of the .cam file
          */
        bool
        saveCameraParameters (const std::string &file);

        /** \brief Get camera parameters and save them to a \ref pcl::visualization::Camera.
          * \param[out] camera the name of the \ref pcl::visualization::Camera
          */
        void
        getCameraParameters (Camera &camera);

        /** \brief Load camera parameters from a camera parameter file.
          * \param[in] file the name of the camera parameter file
          */
        bool
        loadCameraParameters (const std::string &file);

        /** \brief Set the camera parameters via an intrinsics and and extrinsics matrix
          * \note This assumes that the pixels are square and that the center of the image is at the center of the sensor.
          * \param[in] intrinsics the intrinsics that will be used to compute the VTK camera parameters
          * \param[in] extrinsics the extrinsics that will be used to compute the VTK camera parameters
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void
        setCameraParameters (const Eigen::Matrix3f &intrinsics, const Eigen::Matrix4f &extrinsics, int viewport = 0); 

        /** \brief Set the camera parameters by given a full camera data structure.
          * \param[in] camera camera structure containing all the camera parameters.
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void
        setCameraParameters (const Camera &camera, int viewport = 0);

        /** \brief Set camera file for camera parameter saving/restoring.
          * \param[in] file the name of the camera parameter file
          */
        void
        setCameraFile (const std::string file)
        {
          camera_file_ = file;
        }

        /** \brief Get camera file for camera parameter saving/restoring. */
        std::string
        getCameraFile () const
        {
          return (camera_file_);
        }

        /** \brief Change the default keyboard modified from ALT to a different special key.
          * Allowed values are:
          * - INTERACTOR_KB_MOD_ALT
          * - INTERACTOR_KB_MOD_CTRL
          * - INTERACTOR_KB_MOD_SHIFT
          * \param[in] modifier the new keyboard modifier
          */
        inline void
        setKeyboardModifier (const InteractorKeyboardModifier &modifier)
        {
          modifier_ = modifier;
        }

       protected:
        /** \brief Set to true after initialization is complete. */
        bool init_;

        /** \brief Collection of vtkRenderers stored internally. */
        vtkSmartPointer<vtkRendererCollection> rens_;

        /** \brief Cloud actor map stored internally. */
        CloudActorMapPtr cloud_actors_;

        /** \brief Shape map stored internally. */
        ShapeActorMapPtr shape_actors_;

        /** \brief The current window width/height. */
        int win_height_, win_width_;

        /** \brief The current window position x/y. */
        int win_pos_x_, win_pos_y_;

        /** \brief The maximum resizeable window width/height. */
        int max_win_height_, max_win_width_;

        /** \brief Boolean that holds whether or not to use the vtkVertexBufferObjectMapper*/
        bool use_vbos_;

        /** \brief Set to true if the grid actor is enabled. */
        bool grid_enabled_;
        /** \brief Actor for 2D grid on screen. */
        vtkSmartPointer<vtkLegendScaleActor> grid_actor_;

        /** \brief Set to true if the LUT actor is enabled. */
        bool lut_enabled_;
        /** \brief Actor for 2D lookup table on screen. */
        vtkSmartPointer<vtkScalarBarActor> lut_actor_;

        /** \brief A PNG writer for screenshot captures. */
        vtkSmartPointer<vtkPNGWriter> snapshot_writer_;
        /** \brief Internal window to image filter. Needed by \a snapshot_writer_. */
        vtkSmartPointer<vtkWindowToImageFilter> wif_;
        /** \brief Stores the point picker when switching to an area picker. */
        vtkSmartPointer<vtkPointPicker> point_picker_;

        boost::signals2::signal<void (const pcl::visualization::MouseEvent&)> mouse_signal_;
        boost::signals2::signal<void (const pcl::visualization::KeyboardEvent&)> keyboard_signal_;
        boost::signals2::signal<void (const pcl::visualization::PointPickingEvent&)> point_picking_signal_;
        boost::signals2::signal<void (const pcl::visualization::AreaPickingEvent&)> area_picking_signal_;

        /** \brief Interactor style internal method. Gets called whenever a key is pressed. */
        virtual void 
        OnChar ();

        // Keyboard events
        virtual void 
        OnKeyDown ();
        virtual void 
        OnKeyUp ();
        
        // mouse button events
        virtual void 	
        OnMouseMove ();
        virtual void 	
        OnLeftButtonDown ();
        virtual void 	
        OnLeftButtonUp ();
        virtual void 	
        OnMiddleButtonDown ();
        virtual void 	
        OnMiddleButtonUp ();
        virtual void 	
        OnRightButtonDown ();
        virtual void 	
        OnRightButtonUp ();
        virtual void 	
        OnMouseWheelForward ();
        virtual void 	
        OnMouseWheelBackward ();
        
        // mouse move event
        /** \brief Interactor style internal method. Gets called periodically if a timer is set. */
        virtual void 
        OnTimer ();

        /** \brief Interactor style internal method. Zoom in. */
        void 
        zoomIn ();

        /** \brief Interactor style internal method. Zoom out. */
        void 
        zoomOut ();

        /** \brief Get camera parameters from a string vector.
          * \param[in] camera A string vector:
          * Clipping Range, Focal Point, Position, ViewUp, Distance, Field of View Y, Window Size, Window Pos.
          * Values in each string are seperated by a ','
          */
        bool
        getCameraParameters (const std::vector<std::string> &camera);

        /** \brief Set render window. */
        void
        setRenderWindow (const vtkSmartPointer<vtkRenderWindow> &win)
        {
          win_ = win;
        }

        /** \brief True if we're using red-blue colors for anaglyphic stereo, false if magenta-green. */
        bool stereo_anaglyph_mask_default_;

        /** \brief A VTK Mouse Callback object, used for point picking. */
        vtkSmartPointer<PointPickingCallback> mouse_callback_;

        /** \brief The keyboard modifier to use. Default: Alt. */
        InteractorKeyboardModifier modifier_;

        /** \brief Camera file for camera parameter saving/restoring. */
        std::string camera_file_;
        /** \brief A \ref pcl::visualization::Camera for camera parameter saving/restoring. */
        Camera camera_;
        /** \brief A \ref pcl::visualization::Camera is saved or not. */
        bool camera_saved_;
        /** \brief The render window.
          * Only used when interactor maybe not available
          */
        vtkSmartPointer<vtkRenderWindow> win_;

        friend class PointPickingCallback;
        friend class PCLVisualizer;

       private:
        /** \brief ID used to fetch/display the look up table on the visualizer
         * It should be set by PCLVisualizer \ref setLookUpTableID
         * @note If empty, a random actor added to the interactor will be used */
        std::string lut_actor_id_;

        /** \brief Add/remove the look up table displayed when 'u' is pressed, can also be used to update the current LUT displayed
         * \ref lut_actor_id_ is used (if not empty) to chose which cloud/shape actor LUT will be updated (depending on what is available)
         * If \ref lut_actor_id_ is empty the first actor with LUT support found will be used. */
        void
        updateLookUpTableDisplay (bool add_lut = false);
    };

    /** \brief PCL histogram visualizer interactory style class.
      * \author Radu B. Rusu
      */
    class PCLHistogramVisualizerInteractorStyle : public vtkInteractorStyleTrackballCamera
    {
      public:
        static PCLHistogramVisualizerInteractorStyle *New ();

        /** \brief Empty constructor. */
        PCLHistogramVisualizerInteractorStyle () : wins_ (), init_ (false) {}

        /** \brief Initialization routine. Must be called before anything else. */
        void 
        Initialize ();
        
        /** \brief Pass a map of render/window/interactors to the interactor style. 
          * \param[in] wins the RenWinInteract map to use
          */
        void 
        setRenWinInteractMap (const RenWinInteractMap &wins) { wins_ = wins; }

      private:
        /** \brief A map of all windows on screen (with their renderers and interactors). */
        RenWinInteractMap wins_;

        /** \brief Set to true after initialization is complete. */
        bool init_;

        /** \brief Interactor style internal method. Gets called whenever a key is pressed. */
        void OnKeyDown ();

        /** \brief Interactor style internal method. Gets called periodically if a timer is set. */
        void OnTimer ();
    };
  }
}

#endif
