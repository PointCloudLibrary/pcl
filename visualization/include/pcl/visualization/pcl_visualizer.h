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
 */
#ifndef PCL_PCL_VISUALIZER_H_
#define PCL_PCL_VISUALIZER_H_

// PCL includes
#include <pcl/correspondence.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
//
#include <pcl/console/print.h>
#include <pcl/visualization/common/actor_map.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/point_cloud_geometry_handlers.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/visualization/area_picking_event.h>
#include <pcl/visualization/interactor_style.h>

// VTK includes
class vtkPolyData;
class vtkTextActor;
class vtkRenderWindow;
class vtkOrientationMarkerWidget;
class vtkAppendPolyData;
class vtkRenderWindow;
class vtkRenderWindowInteractor;
class vtkTransform;
class vtkInteractorStyle;
class vtkLODActor;
class vtkProp;
class vtkActor;
class vtkDataSet;
class vtkUnstructuredGrid;

namespace pcl
{
  template <typename T> class PointCloud;
  template <typename T> class PlanarPolygon;

  namespace visualization
  {
    /** \brief PCL Visualizer main class.
      * \author Radu B. Rusu
      * \ingroup visualization
      * \note This class can NOT be used across multiple threads. Only call functions of objects of this class
      * from the same thread that they were created in! Some methods, e.g. addPointCloud, will crash if called
      * from other threads.
      */
    class PCL_EXPORTS PCLVisualizer
    {
      public:
        typedef boost::shared_ptr<PCLVisualizer> Ptr;
        typedef boost::shared_ptr<const PCLVisualizer> ConstPtr;

        typedef PointCloudGeometryHandler<pcl::PCLPointCloud2> GeometryHandler;
        typedef GeometryHandler::Ptr GeometryHandlerPtr;
        typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

        typedef PointCloudColorHandler<pcl::PCLPointCloud2> ColorHandler;
        typedef ColorHandler::Ptr ColorHandlerPtr;
        typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

        /** \brief PCL Visualizer constructor.
          * \param[in] name the window name (empty by default)
          * \param[in] create_interactor if true (default), create an interactor, false otherwise
          */
        PCLVisualizer (const std::string &name = "", const bool create_interactor = true);

        /** \brief PCL Visualizer constructor.
          * \param[in] argc
          * \param[in] argv
          * \param[in] name the window name (empty by default)
          * \param[in] style interactor style (defaults to PCLVisualizerInteractorStyle)
          * \param[in] create_interactor if true (default), create an interactor, false otherwise
          */
        PCLVisualizer (int &argc, char **argv, const std::string &name = "",
            PCLVisualizerInteractorStyle* style = PCLVisualizerInteractorStyle::New (), const bool create_interactor = true);

        /** \brief PCL Visualizer destructor. */
        virtual ~PCLVisualizer ();

        /** \brief Enables/Disabled the underlying window mode to full screen.
          * \note This might or might not work, depending on your window manager.
          * See the VTK documentation for additional details.
          * \param[in] mode true for full screen, false otherwise
          */
        void
        setFullScreen (bool mode);

        /** \brief Set the visualizer window name.
          * \param[in] name the name of the window
          */
        void
        setWindowName (const std::string &name);

        /** \brief Enables or disable the underlying window borders.
          * \note This might or might not work, depending on your window manager.
          * See the VTK documentation for additional details.
          * \param[in] mode true for borders, false otherwise
          */
        void
        setWindowBorders (bool mode);

        /** \brief Register a callback boost::function for keyboard events
          * \param[in] cb a boost function that will be registered as a callback for a keyboard event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerKeyboardCallback (boost::function<void (const pcl::visualization::KeyboardEvent&)> cb);

        /** \brief Register a callback function for keyboard events
          * \param[in] callback  the function that will be registered as a callback for a keyboard event
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
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
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerKeyboardCallback (void (T::*callback) (const pcl::visualization::KeyboardEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerKeyboardCallback (boost::bind (callback,  boost::ref (instance), _1, cookie)));
        }

        /** \brief Register a callback function for mouse events
          * \param[in] cb a boost function that will be registered as a callback for a mouse event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerMouseCallback (boost::function<void (const pcl::visualization::MouseEvent&)> cb);

        /** \brief Register a callback function for mouse events
          * \param[in] callback  the function that will be registered as a callback for a mouse event
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
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
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerMouseCallback (void (T::*callback) (const pcl::visualization::MouseEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerMouseCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }

        /** \brief Register a callback function for point picking events
          * \param[in] cb a boost function that will be registered as a callback for a point picking event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerPointPickingCallback (boost::function<void (const pcl::visualization::PointPickingEvent&)> cb);

        /** \brief Register a callback function for point picking events
          * \param[in] callback  the function that will be registered as a callback for a point picking event
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerPointPickingCallback (void (*callback) (const pcl::visualization::PointPickingEvent&, void*), void* cookie = NULL);

        /** \brief Register a callback function for point picking events
          * \param[in] callback  the member function that will be registered as a callback for a point picking event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerPointPickingCallback (void (T::*callback) (const pcl::visualization::PointPickingEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerPointPickingCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }

        /** \brief Register a callback function for area picking events
          * \param[in] cb a boost function that will be registered as a callback for an area picking event
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerAreaPickingCallback (boost::function<void (const pcl::visualization::AreaPickingEvent&)> cb);

        /** \brief Register a callback function for area picking events
          * \param[in] callback  the function that will be registered as a callback for an area picking event
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        boost::signals2::connection
        registerAreaPickingCallback (void (*callback) (const pcl::visualization::AreaPickingEvent&, void*), void* cookie = NULL);

        /** \brief Register a callback function for area picking events
          * \param[in] callback  the member function that will be registered as a callback for an area picking event
          * \param[in] instance  instance to the class that implements the callback function
          * \param[in] cookie    user data that is passed to the callback
          * \return a connection object that allows to disconnect the callback function.
          */
        template<typename T> inline boost::signals2::connection
        registerAreaPickingCallback (void (T::*callback) (const pcl::visualization::AreaPickingEvent&, void*), T& instance, void* cookie = NULL)
        {
          return (registerAreaPickingCallback (boost::bind (callback, boost::ref (instance), _1, cookie)));
        }

        /** \brief Spin method. Calls the interactor and runs an internal loop. */
        void
        spin ();

        /** \brief Spin once method. Calls the interactor and updates the screen once.
          *  \param[in] time - How long (in ms) should the visualization loop be allowed to run.
          *  \param[in] force_redraw - if false it might return without doing anything if the
          *  interactor's framerate does not require a redraw yet.
          */
        void
        spinOnce (int time = 1, bool force_redraw = false);

        /** \brief Adds a widget which shows an interactive axes display for orientation
         *  \param[in] interactor - Pointer to the vtk interactor object used by the PCLVisualizer window 
         */
        void
        addOrientationMarkerWidgetAxes (vtkRenderWindowInteractor* interactor);
        
        /** \brief Disables the Orientatation Marker Widget so it is removed from the renderer */
        void
        removeOrientationMarkerWidgetAxes ();
        
        /** \brief Adds 3D axes describing a coordinate system to screen at 0,0,0.
          * \param[in] scale the scale of the axes (default: 1)
          * \param[in] viewport the view port where the 3D axes should be added (default: all)
          */
        PCL_DEPRECATED (
        "addCoordinateSystem (scale, viewport) is deprecated, please use function "
        "addCoordinateSystem (scale, id, viewport) with id a unique string identifier.")
        void
        addCoordinateSystem (double scale, int viewport = 0);

        /** \brief Adds 3D axes describing a coordinate system to screen at 0,0,0.
          * \param[in] scale the scale of the axes (default: 1)
          * \param[in] id the coordinate system object id (default: reference)
          * \param[in] viewport the view port where the 3D axes should be added (default: all)
          */
        void
        addCoordinateSystem (double scale = 1.0, const std::string& id = "reference", int viewport = 0);

        /** \brief Adds 3D axes describing a coordinate system to screen at x, y, z
          * \param[in] scale the scale of the axes (default: 1)
          * \param[in] x the X position of the axes
          * \param[in] y the Y position of the axes
          * \param[in] z the Z position of the axes
          * \param[in] viewport the view port where the 3D axes should be added (default: all)
          */
        PCL_DEPRECATED (
        "addCoordinateSystem (scale, x, y, z, viewport) is deprecated, please use function "
        "addCoordinateSystem (scale, x, y, z, id, viewport) with id a unique string identifier.")
        void
        addCoordinateSystem (double scale, float x, float y, float z, int viewport = 0);

        /** \brief Adds 3D axes describing a coordinate system to screen at x, y, z
          * \param[in] scale the scale of the axes (default: 1)
          * \param[in] x the X position of the axes
          * \param[in] y the Y position of the axes
          * \param[in] z the Z position of the axes
          * \param[in] id the coordinate system object id (default: reference)
          * \param[in] viewport the view port where the 3D axes should be added (default: all)
          */
        void
        addCoordinateSystem (double scale, float x, float y, float z, const std::string &id = "reference", int viewport = 0);

         /** \brief Adds 3D axes describing a coordinate system to screen at x, y, z, Roll,Pitch,Yaw
           *
           * \param[in] scale the scale of the axes (default: 1)
           * \param[in] t transformation matrix
           * \param[in] viewport the view port where the 3D axes should be added (default: all)
           */
        PCL_DEPRECATED (
        "addCoordinateSystem (scale, t, viewport) is deprecated, please use function "
        "addCoordinateSystem (scale, t, id, viewport) with id a unique string identifier.")
        void
        addCoordinateSystem (double scale, const Eigen::Affine3f& t, int viewport = 0);

         /** \brief Adds 3D axes describing a coordinate system to screen at x, y, z, Roll,Pitch,Yaw
           *
           * \param[in] scale the scale of the axes (default: 1)
           * \param[in] t transformation matrix
           * \param[in] id the coordinate system object id (default: reference)
           * \param[in] viewport the view port where the 3D axes should be added (default: all)
           *
           * RPY Angles
           * Rotate the reference frame by the angle roll about axis x
           * Rotate the reference frame by the angle pitch about axis y
           * Rotate the reference frame by the angle yaw about axis z
           *
           * Description:
           * Sets the orientation of the Prop3D.  Orientation is specified as
           * X,Y and Z rotations in that order, but they are performed as
           * RotateZ, RotateX, and finally RotateY.
           *
           * All axies use right hand rule. x=red axis, y=green axis, z=blue axis
           * z direction is point into the screen.
           * \code
           *     z
           *      \
           *       \
           *        \
           *         -----------> x
           *         |
           *         |
           *         |
           *         |
           *         |
           *         |
           *         y
           * \endcode
           */

        void
        addCoordinateSystem (double scale, const Eigen::Affine3f& t, const std::string &id = "reference", int viewport = 0);

        /** \brief Removes a previously added 3D axes (coordinate system)
          * \param[in] viewport view port where the 3D axes should be removed from (default: all)
          */
        PCL_DEPRECATED (
        "removeCoordinateSystem (viewport) is deprecated, please use function "
        "addCoordinateSystem (id, viewport) with id a unique string identifier.")
        bool
        removeCoordinateSystem (int viewport = 0);

        /** \brief Removes a previously added 3D axes (coordinate system)
          * \param[in] id the coordinate system object id (default: reference)
          * \param[in] viewport view port where the 3D axes should be removed from (default: all)
          */
        bool
        removeCoordinateSystem (const std::string &id = "reference", int viewport = 0);

        /** \brief Removes a Point Cloud from screen, based on a given ID.
          * \param[in] id the point cloud object id (i.e., given on \a addPointCloud)
          * \param[in] viewport view port from where the Point Cloud should be removed (default: all)
          * \return true if the point cloud is successfully removed and false if the point cloud is
          * not actually displayed
          */
        bool
        removePointCloud (const std::string &id = "cloud", int viewport = 0);

        /** \brief Removes a PolygonMesh from screen, based on a given ID.
          * \param[in] id the polygon object id (i.e., given on \a addPolygonMesh)
          * \param[in] viewport view port from where the PolygonMesh should be removed (default: all)
          */
        inline bool
        removePolygonMesh (const std::string &id = "polygon", int viewport = 0)
        {
          // Polygon Meshes are represented internally as point clouds with special cell array structures since 1.4
          return (removePointCloud (id, viewport));
        }

        /** \brief Removes an added shape from screen (line, polygon, etc.), based on a given ID
          * \note This methods also removes PolygonMesh objects and PointClouds, if they match the ID
          * \param[in] id the shape object id (i.e., given on \a addLine etc.)
          * \param[in] viewport view port from where the Point Cloud should be removed (default: all)
          */
        bool
        removeShape (const std::string &id = "cloud", int viewport = 0);

        /** \brief Removes an added 3D text from the scene, based on a given ID
          * \param[in] id the 3D text id (i.e., given on \a addText3D etc.)
          * \param[in] viewport view port from where the 3D text should be removed (default: all)
          */
        bool
        removeText3D (const std::string &id = "cloud", int viewport = 0);

        /** \brief Remove all point cloud data on screen from the given viewport.
          * \param[in] viewport view port from where the clouds should be removed (default: all)
          */
        bool
        removeAllPointClouds (int viewport = 0);

        /** \brief Remove all 3D shape data on screen from the given viewport.
          * \param[in] viewport view port from where the shapes should be removed (default: all)
          */
        bool
        removeAllShapes (int viewport = 0);

        /** \brief Set the viewport's background color.
          * \param[in] r the red component of the RGB color
          * \param[in] g the green component of the RGB color
          * \param[in] b the blue component of the RGB color
          * \param[in] viewport the view port (default: all)
          */
        void
        setBackgroundColor (const double &r, const double &g, const double &b, int viewport = 0);

        /** \brief Add a text to screen
          * \param[in] text the text to add
          * \param[in] xpos the X position on screen where the text should be added
          * \param[in] ypos the Y position on screen where the text should be added
          * \param[in] id the text object id (default: equal to the "text" parameter)
          * \param[in] viewport the view port (default: all)
          */
        bool
        addText (const std::string &text,
                 int xpos, int ypos,
                 const std::string &id = "", int viewport = 0);

        /** \brief Add a text to screen
          * \param[in] text the text to add
          * \param[in] xpos the X position on screen where the text should be added
          * \param[in] ypos the Y position on screen where the text should be added
          * \param[in] r the red color value
          * \param[in] g the green color value
          * \param[in] b the blue color vlaue
          * \param[in] id the text object id (default: equal to the "text" parameter)
          * \param[in] viewport the view port (default: all)
          */
        bool
        addText (const std::string &text, int xpos, int ypos, double r, double g, double b,
                 const std::string &id = "", int viewport = 0);

        /** \brief Add a text to screen
          * \param[in] text the text to add
          * \param[in] xpos the X position on screen where the text should be added
          * \param[in] ypos the Y position on screen where the text should be added
          * \param[in] fontsize the fontsize of the text
          * \param[in] r the red color value
          * \param[in] g the green color value
          * \param[in] b the blue color vlaue
          * \param[in] id the text object id (default: equal to the "text" parameter)
          * \param[in] viewport the view port (default: all)
          */
        bool
        addText (const std::string &text, int xpos, int ypos, int fontsize, double r, double g, double b,
                 const std::string &id = "", int viewport = 0);


        /** \brief Update a text to screen
          * \param[in] text the text to update
          * \param[in] xpos the new X position on screen
          * \param[in] ypos the new Y position on screen 
          * \param[in] id the text object id (default: equal to the "text" parameter)
          */
        bool
        updateText (const std::string &text,
                    int xpos, int ypos,
                    const std::string &id = "");

        /** \brief Update a text to screen
          * \param[in] text the text to update
          * \param[in] xpos the new X position on screen
          * \param[in] ypos the new Y position on screen 
          * \param[in] r the red color value
          * \param[in] g the green color value
          * \param[in] b the blue color vlaue
          * \param[in] id the text object id (default: equal to the "text" parameter)
          */
        bool
        updateText (const std::string &text, 
                    int xpos, int ypos, double r, double g, double b,
                    const std::string &id = "");

        /** \brief Update a text to screen
          * \param[in] text the text to update
          * \param[in] xpos the new X position on screen
          * \param[in] ypos the new Y position on screen 
          * \param[in] fontsize the fontsize of the text
          * \param[in] r the red color value
          * \param[in] g the green color value
          * \param[in] b the blue color vlaue
          * \param[in] id the text object id (default: equal to the "text" parameter)
          */
        bool
        updateText (const std::string &text, 
                    int xpos, int ypos, int fontsize, double r, double g, double b,
                    const std::string &id = "");

        /** \brief Set the pose of an existing shape. 
          * 
          * Returns false if the shape doesn't exist, true if the pose was successfully 
          * updated.
          *
          * \param[in] id the shape or cloud object id (i.e., given on \a addLine etc.)
          * \param[in] pose the new pose
          * \return false if no shape or cloud with the specified ID was found
          */
        bool
        updateShapePose (const std::string &id, const Eigen::Affine3f& pose);

        /** \brief Set the pose of an existing coordinate system.
          *
          * Returns false if the coordinate system doesn't exist, true if the pose was successfully
          * updated.
          *
          * \param[in] id the point cloud object id (i.e., given on \a addCoordinateSystem etc.)
          * \param[in] pose the new pose
          * \return false if no coordinate system with the specified ID was found
          */
        bool
        updateCoordinateSystemPose (const std::string &id, const Eigen::Affine3f& pose);

        /** \brief Set the pose of an existing point cloud.
          *
          * Returns false if the point cloud doesn't exist, true if the pose was successfully
          * updated.
          *
          * \param[in] id the point cloud object id (i.e., given on \a addPointCloud etc.)
          * \param[in] pose the new pose
          * \return false if no point cloud with the specified ID was found
          */
        bool
        updatePointCloudPose (const std::string &id, const Eigen::Affine3f& pose);

        /** \brief Add a 3d text to the scene
          * \param[in] text the text to add
          * \param[in] position the world position where the text should be added
          * \param[in] textScale the scale of the text to render
          * \param[in] r the red color value
          * \param[in] g the green color value
          * \param[in] b the blue color value
          * \param[in] id the text object id (default: equal to the "text" parameter)
          * \param[in] viewport the view port (default: all)
          */
        template <typename PointT> bool
        addText3D (const std::string &text,
                   const PointT &position,
                   double textScale = 1.0,
                   double r = 1.0, double g = 1.0, double b = 1.0,
                   const std::string &id = "", int viewport = 0);

        /** \brief Add the estimated surface normals of a Point Cloud to screen.
          * \param[in] cloud the input point cloud dataset containing XYZ data and normals
          * \param[in] level display only every level'th point (default: 100)
          * \param[in] scale the normal arrow scale (default: 0.02m)
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointNT> bool
        addPointCloudNormals (const typename pcl::PointCloud<PointNT>::ConstPtr &cloud,
                              int level = 100, float scale = 0.02f,
                              const std::string &id = "cloud", int viewport = 0);

        /** \brief Add the estimated surface normals of a Point Cloud to screen.
          * \param[in] cloud the input point cloud dataset containing the XYZ data
          * \param[in] normals the input point cloud dataset containing the normal data
          * \param[in] level display only every level'th point (default: 100)
          * \param[in] scale the normal arrow scale (default: 0.02m)
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT, typename PointNT> bool
        addPointCloudNormals (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                              const typename pcl::PointCloud<PointNT>::ConstPtr &normals,
                              int level = 100, float scale = 0.02f,
                              const std::string &id = "cloud", int viewport = 0);

        /** \brief Add the estimated principal curvatures of a Point Cloud to screen.
          * \param[in] cloud the input point cloud dataset containing the XYZ data
          * \param[in] normals the input point cloud dataset containing the normal data
          * \param[in] pcs the input point cloud dataset containing the principal curvatures data
          * \param[in] level display only every level'th point. Default: 100
          * \param[in] scale the normal arrow scale. Default: 1.0
          * \param[in] id the point cloud object id. Default: "cloud"
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        bool
        addPointCloudPrincipalCurvatures (
            const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
            const pcl::PointCloud<pcl::Normal>::ConstPtr &normals,
            const pcl::PointCloud<pcl::PrincipalCurvatures>::ConstPtr &pcs,
            int level = 100, float scale = 1.0f,
            const std::string &id = "cloud", int viewport = 0);

        /** \brief Add the estimated surface intensity gradients of a Point Cloud to screen.
          * \param[in] cloud the input point cloud dataset containing the XYZ data
          * \param[in] gradients the input point cloud dataset containing the intensity gradient data
          * \param[in] level display only every level'th point (default: 100)
          * \param[in] scale the intensity gradient arrow scale (default: 1e-6m)
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT, typename GradientT> bool
        addPointCloudIntensityGradients (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                         const typename pcl::PointCloud<GradientT>::ConstPtr &gradients,
                                         int level = 100, double scale = 1e-6,
                                         const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a Point Cloud (templated) to screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id (default: cloud)
          * \param viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT> bool
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Updates the XYZ data for an existing cloud object id on screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id to update (default: cloud)
          * \return false if no cloud with the specified ID was found
          */
        template <typename PointT> bool
        updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const std::string &id = "cloud");

         /** \brief Updates the XYZ data for an existing cloud object id on screen.
           * \param[in] cloud the input point cloud dataset
           * \param[in] geometry_handler the geometry handler to use
           * \param[in] id the point cloud object id to update (default: cloud)
           * \return false if no cloud with the specified ID was found
           */
        template <typename PointT> bool
        updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const PointCloudGeometryHandler<PointT> &geometry_handler,
                          const std::string &id = "cloud");

         /** \brief Updates the XYZ data for an existing cloud object id on screen.
           * \param[in] cloud the input point cloud dataset
           * \param[in] color_handler the color handler to use
           * \param[in] id the point cloud object id to update (default: cloud)
           * \return false if no cloud with the specified ID was found
           */
        template <typename PointT> bool
        updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                          const PointCloudColorHandler<PointT> &color_handler,
                          const std::string &id = "cloud");

        /** \brief Add a Point Cloud (templated) to screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] geometry_handler use a geometry handler object to extract the XYZ data
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT> bool
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       const PointCloudGeometryHandler<PointT> &geometry_handler,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a Point Cloud (templated) to screen.
          *
          * Because the geometry handler is given as a pointer, it will be pushed back to the list of available
          * handlers, rather than replacing the current active geometric handler. This makes it possible to
          * switch between different geometric handlers 'on-the-fly' at runtime, from the PCLVisualizer
          * interactor interface (using Alt+0..9).
          *
          * \param[in] cloud the input point cloud dataset
          * \param[in] geometry_handler use a geometry handler object to extract the XYZ data
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT> bool
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       const GeometryHandlerConstPtr &geometry_handler,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a Point Cloud (templated) to screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] color_handler a specific PointCloud visualizer handler for colors
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT> bool
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       const PointCloudColorHandler<PointT> &color_handler,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a Point Cloud (templated) to screen.
          *
          * Because the color handler is given as a pointer, it will be pushed back to the list of available
          * handlers, rather than replacing the current active color handler. This makes it possible to
          * switch between different color handlers 'on-the-fly' at runtime, from the PCLVisualizer
          * interactor interface (using 0..9).
          *
          * \param[in] cloud the input point cloud dataset
          * \param[in] color_handler a specific PointCloud visualizer handler for colors
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT> bool
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       const ColorHandlerConstPtr &color_handler,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a Point Cloud (templated) to screen.
          *
          * Because the geometry/color handler is given as a pointer, it will be pushed back to the list of
          * available handlers, rather than replacing the current active handler. This makes it possible to
          * switch between different handlers 'on-the-fly' at runtime, from the PCLVisualizer interactor
          * interface (using [Alt+]0..9).
          *
          * \param[in] cloud the input point cloud dataset
          * \param[in] geometry_handler a specific PointCloud visualizer handler for geometry
          * \param[in] color_handler a specific PointCloud visualizer handler for colors
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT> bool
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       const GeometryHandlerConstPtr &geometry_handler,
                       const ColorHandlerConstPtr &color_handler,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a binary blob Point Cloud to screen.
          *
          * Because the geometry/color handler is given as a pointer, it will be pushed back to the list of
          * available handlers, rather than replacing the current active handler. This makes it possible to
          * switch between different handlers 'on-the-fly' at runtime, from the PCLVisualizer interactor
          * interface (using [Alt+]0..9).
          *
          * \param[in] cloud the input point cloud dataset
          * \param[in] geometry_handler a specific PointCloud visualizer handler for geometry
          * \param[in] color_handler a specific PointCloud visualizer handler for colors
          * \param[in] sensor_origin the origin of the cloud data in global coordinates (defaults to 0,0,0)
          * \param[in] sensor_orientation the orientation of the cloud data in global coordinates (defaults to 1,0,0,0)
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        bool
        addPointCloud (const pcl::PCLPointCloud2::ConstPtr &cloud,
                       const GeometryHandlerConstPtr &geometry_handler,
                       const ColorHandlerConstPtr &color_handler,
                       const Eigen::Vector4f& sensor_origin,
                       const Eigen::Quaternion<float>& sensor_orientation,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a binary blob Point Cloud to screen.
          *
          * Because the geometry/color handler is given as a pointer, it will be pushed back to the list of
          * available handlers, rather than replacing the current active handler. This makes it possible to
          * switch between different handlers 'on-the-fly' at runtime, from the PCLVisualizer interactor
          * interface (using [Alt+]0..9).
          *
          * \param[in] cloud the input point cloud dataset
          * \param[in] geometry_handler a specific PointCloud visualizer handler for geometry
          * \param[in] sensor_origin the origin of the cloud data in global coordinates (defaults to 0,0,0)
          * \param[in] sensor_orientation the orientation of the cloud data in global coordinates (defaults to 1,0,0,0)
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        bool
        addPointCloud (const pcl::PCLPointCloud2::ConstPtr &cloud,
                       const GeometryHandlerConstPtr &geometry_handler,
                       const Eigen::Vector4f& sensor_origin,
                       const Eigen::Quaternion<float>& sensor_orientation,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a binary blob Point Cloud to screen.
          *
          * Because the geometry/color handler is given as a pointer, it will be pushed back to the list of
          * available handlers, rather than replacing the current active handler. This makes it possible to
          * switch between different handlers 'on-the-fly' at runtime, from the PCLVisualizer interactor
          * interface (using [Alt+]0..9).
          *
          * \param[in] cloud the input point cloud dataset
          * \param[in] color_handler a specific PointCloud visualizer handler for colors
          * \param[in] sensor_origin the origin of the cloud data in global coordinates (defaults to 0,0,0)
          * \param[in] sensor_orientation the orientation of the cloud data in global coordinates (defaults to 1,0,0,0)
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        bool
        addPointCloud (const pcl::PCLPointCloud2::ConstPtr &cloud,
                       const ColorHandlerConstPtr &color_handler,
                       const Eigen::Vector4f& sensor_origin,
                       const Eigen::Quaternion<float>& sensor_orientation,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a Point Cloud (templated) to screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] color_handler a specific PointCloud visualizer handler for colors
          * \param[in] geometry_handler use a geometry handler object to extract the XYZ data
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        template <typename PointT> bool
        addPointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                       const PointCloudColorHandler<PointT> &color_handler,
                       const PointCloudGeometryHandler<PointT> &geometry_handler,
                       const std::string &id = "cloud", int viewport = 0);

        /** \brief Add a PointXYZ Point Cloud to screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        inline bool
        addPointCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                       const std::string &id = "cloud", int viewport = 0)
        {
          return (addPointCloud<pcl::PointXYZ> (cloud, id, viewport));
        }


        /** \brief Add a PointXYZRGB Point Cloud to screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        inline bool
        addPointCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                       const std::string &id = "cloud", int viewport = 0)
        {
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler (cloud);
          return (addPointCloud<pcl::PointXYZRGB> (cloud, color_handler, id, viewport));
        }

        /** \brief Add a PointXYZRGBA Point Cloud to screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud should be added (default: all)
          */
        inline bool
        addPointCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                       const std::string &id = "cloud", int viewport = 0)
        {
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color_handler (cloud);
          return (addPointCloud<pcl::PointXYZRGBA> (cloud, color_handler, id, viewport));
        }

        /** \brief Updates the XYZ data for an existing cloud object id on screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id to update (default: cloud)
          * \return false if no cloud with the specified ID was found
          */
        inline bool
        updatePointCloud (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                          const std::string &id = "cloud")
        {
          return (updatePointCloud<pcl::PointXYZ> (cloud, id));
        }

        /** \brief Updates the XYZRGB data for an existing cloud object id on screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id to update (default: cloud)
          * \return false if no cloud with the specified ID was found
          */
        inline bool
        updatePointCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                          const std::string &id = "cloud")
        {
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> color_handler (cloud);
          return (updatePointCloud<pcl::PointXYZRGB> (cloud, color_handler, id));
        }

        /** \brief Updates the XYZRGBA data for an existing cloud object id on screen.
          * \param[in] cloud the input point cloud dataset
          * \param[in] id the point cloud object id to update (default: cloud)
          * \return false if no cloud with the specified ID was found
          */
        inline bool
        updatePointCloud (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                          const std::string &id = "cloud")
        {
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> color_handler (cloud);
          return (updatePointCloud<pcl::PointXYZRGBA> (cloud, color_handler, id));
        }

        /** \brief Add a PolygonMesh object to screen
          * \param[in] polymesh the polygonal mesh
          * \param[in] id the polygon object id (default: "polygon")
          * \param[in] viewport the view port where the PolygonMesh should be added (default: all)
          */
        bool
        addPolygonMesh (const pcl::PolygonMesh &polymesh,
                        const std::string &id = "polygon",
                        int viewport = 0);

        /** \brief Add a PolygonMesh object to screen
          * \param[in] cloud the polygonal mesh point cloud
          * \param[in] vertices the polygonal mesh vertices
          * \param[in] id the polygon object id (default: "polygon")
          * \param[in] viewport the view port where the PolygonMesh should be added (default: all)
          */
        template <typename PointT> bool
        addPolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                        const std::vector<pcl::Vertices> &vertices,
                        const std::string &id = "polygon",
                        int viewport = 0);

        /** \brief Update a PolygonMesh object on screen
          * \param[in] cloud the polygonal mesh point cloud
          * \param[in] vertices the polygonal mesh vertices
          * \param[in] id the polygon object id (default: "polygon")
          * \return false if no polygonmesh with the specified ID was found
          */
        template <typename PointT> bool
        updatePolygonMesh (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                           const std::vector<pcl::Vertices> &vertices,
                           const std::string &id = "polygon");

        /** \brief Update a PolygonMesh object on screen
          * \param[in] polymesh the polygonal mesh
          * \param[in] id the polygon object id (default: "polygon")
          * \return false if no polygonmesh with the specified ID was found
          */
        bool
        updatePolygonMesh (const pcl::PolygonMesh &polymesh,
                           const std::string &id = "polygon");

        /** \brief Add a Polygonline from a polygonMesh object to screen
          * \param[in] polymesh the polygonal mesh from where the polylines will be extracted
          * \param[in] id the polygon object id (default: "polygon")
          * \param[in] viewport the view port where the PolygonMesh should be added (default: all)
          */
        bool
        addPolylineFromPolygonMesh (const pcl::PolygonMesh &polymesh,
                                    const std::string &id = "polyline",
                                    int viewport = 0);

        /** \brief Add the specified correspondences to the display.
          * \param[in] source_points The source points
          * \param[in] target_points The target points
          * \param[in] correspondences The mapping from source points to target points. Each element must be an index into target_points
          * \param[in] id the polygon object id (default: "correspondences")
          * \param[in] viewport the view port where the correspondences should be added (default: all)
          */
        template <typename PointT> bool
        addCorrespondences (const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
                            const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
                            const std::vector<int> & correspondences,
                            const std::string &id = "correspondences",
                            int viewport = 0);

        /** \brief Add a TextureMesh object to screen
          * \param[in] polymesh the textured polygonal mesh
          * \param[in] id the texture mesh object id (default: "texture")
          * \param[in] viewport the view port where the TextureMesh should be added (default: all)
          */
        bool
        addTextureMesh (const pcl::TextureMesh &polymesh,
                        const std::string &id = "texture",
                        int viewport = 0);

        /** \brief Add the specified correspondences to the display.
          * \param[in] source_points The source points
          * \param[in] target_points The target points
          * \param[in] correspondences The mapping from source points to target points. Each element must be an index into target_points
          * \param[in] nth display only the Nth correspondence (e.g., skip the rest)
          * \param[in] id the polygon object id (default: "correspondences")
          * \param[in] viewport the view port where the correspondences should be added (default: all)
          */
        template <typename PointT> bool
        addCorrespondences (const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
                            const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
                            const pcl::Correspondences &correspondences,
                            int nth,
                            const std::string &id = "correspondences",
                            int viewport = 0);

        /** \brief Add the specified correspondences to the display.
          * \param[in] source_points The source points
          * \param[in] target_points The target points
          * \param[in] correspondences The mapping from source points to target points. Each element must be an index into target_points
          * \param[in] id the polygon object id (default: "correspondences")
          * \param[in] viewport the view port where the correspondences should be added (default: all)
          */
        template <typename PointT> bool
        addCorrespondences (const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
                            const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
                            const pcl::Correspondences &correspondences,
                            const std::string &id = "correspondences",
                            int viewport = 0)
        {
          // If Nth not given, display all correspondences
          return (addCorrespondences<PointT> (source_points, target_points, 
                                              correspondences, 1, id, viewport));
        }

        /** \brief Update the specified correspondences to the display.
          * \param[in] source_points The source points
          * \param[in] target_points The target points
          * \param[in] correspondences The mapping from source points to target points. Each element must be an index into target_points
          * \param[in] nth display only the Nth correspondence (e.g., skip the rest)
          * \param[in] id the polygon object id (default: "correspondences")
          */
        template <typename PointT> bool
        updateCorrespondences (
            const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
            const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
            const pcl::Correspondences &correspondences,
            int nth,
            const std::string &id = "correspondences");

        /** \brief Remove the specified correspondences from the display.
          * \param[in] id the polygon correspondences object id (i.e., given on \ref addCorrespondences)
          * \param[in] viewport view port from where the correspondences should be removed (default: all)
          */
        void
        removeCorrespondences (const std::string &id = "correspondences", int viewport = 0);

        /** \brief Get the color handler index of a rendered PointCloud based on its ID
          * \param[in] id the point cloud object id
          */
        int
        getColorHandlerIndex (const std::string &id);

        /** \brief Get the geometry handler index of a rendered PointCloud based on its ID
          * \param[in] id the point cloud object id
          */
        int
        getGeometryHandlerIndex (const std::string &id);

        /** \brief Update/set the color index of a renderered PointCloud based on its ID
          * \param[in] id the point cloud object id
          * \param[in] index the color handler index to use
          */
        bool
        updateColorHandlerIndex (const std::string &id, int index);

        /** \brief Set the rendering properties of a PointCloud (3x values - e.g., RGB)
          * \param[in] property the property type
          * \param[in] val1 the first value to be set
          * \param[in] val2 the second value to be set
          * \param[in] val3 the third value to be set
          * \param[in] id the point cloud object id (default: cloud)
          * \param[in] viewport the view port where the Point Cloud's rendering properties should be modified (default: all)
          */
        bool
        setPointCloudRenderingProperties (int property, double val1, double val2, double val3,
                                          const std::string &id = "cloud", int viewport = 0);

       /** \brief Set the rendering properties of a PointCloud
         * \param[in] property the property type
         * \param[in] value the value to be set
         * \param[in] id the point cloud object id (default: cloud)
         * \param[in] viewport the view port where the Point Cloud's rendering properties should be modified (default: all)
         */
        bool
        setPointCloudRenderingProperties (int property, double value,
                                          const std::string &id = "cloud", int viewport = 0);

       /** \brief Get the rendering properties of a PointCloud
         * \param[in] property the property type
         * \param[in] value the resultant property value
         * \param[in] id the point cloud object id (default: cloud)
         */
        bool
        getPointCloudRenderingProperties (int property, double &value,
                                          const std::string &id = "cloud");
        
        /** \brief Set whether the point cloud is selected or not 
         *  \param[in] selected whether the cloud is selected or not (true = selected)
         *  \param[in] id the point cloud object id (default: cloud)
         */
        bool
        setPointCloudSelected (const bool selected, const std::string &id = "cloud" );
        
       /** \brief Set the rendering properties of a shape
         * \param[in] property the property type
         * \param[in] value the value to be set
         * \param[in] id the shape object id
         * \param[in] viewport the view port where the shape's properties should be modified (default: all)
         */
        bool
        setShapeRenderingProperties (int property, double value,
                                     const std::string &id, int viewport = 0);

        /** \brief Set the rendering properties of a shape (3x values - e.g., RGB)
          * \param[in] property the property type
          * \param[in] val1 the first value to be set
          * \param[in] val2 the second value to be set
          * \param[in] val3 the third value to be set
          * \param[in] id the shape object id
          * \param[in] viewport the view port where the shape's properties should be modified (default: all)
          */
         bool
         setShapeRenderingProperties (int property, double val1, double val2, double val3,
                                      const std::string &id, int viewport = 0);

        /** \brief Returns true when the user tried to close the window */
        bool
        wasStopped () const;

        /** \brief Set the stopped flag back to false */
        void
        resetStoppedFlag ();

        /** \brief Stop the interaction and close the visualizaton window. */
        void
        close ();

        /** \brief Create a new viewport from [xmin,ymin] -> [xmax,ymax].
          * \param[in] xmin the minimum X coordinate for the viewport (0.0 <= 1.0)
          * \param[in] ymin the minimum Y coordinate for the viewport (0.0 <= 1.0)
          * \param[in] xmax the maximum X coordinate for the viewport (0.0 <= 1.0)
          * \param[in] ymax the maximum Y coordinate for the viewport (0.0 <= 1.0)
          * \param[in] viewport the id of the new viewport
          *
          * \note If no renderer for the current window exists, one will be created, and 
          * the viewport will be set to 0 ('all'). In case one or multiple renderers do 
          * exist, the viewport ID will be set to the total number of renderers - 1.
          */
        void
        createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport);

        /** \brief Create a new separate camera for the given viewport.
          * \param[in] viewport the viewport to create a new camera for.
          */
        void
        createViewPortCamera (const int viewport);

        /** \brief Add a polygon (polyline) that represents the input point cloud (connects all
          * points in order)
          * \param[in] cloud the point cloud dataset representing the polygon
          * \param[in] r the red channel of the color that the polygon should be rendered with
          * \param[in] g the green channel of the color that the polygon should be rendered with
          * \param[in] b the blue channel of the color that the polygon should be rendered with
          * \param[in] id (optional) the polygon id/name (default: "polygon")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                    double r, double g, double b,
                    const std::string &id = "polygon", int viewport = 0);

        /** \brief Add a polygon (polyline) that represents the input point cloud (connects all
          * points in order)
          * \param[in] cloud the point cloud dataset representing the polygon
          * \param[in] id the polygon id/name (default: "polygon")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addPolygon (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                    const std::string &id = "polygon",
                    int viewport = 0);

        /** \brief Add a planar polygon that represents the input point cloud (connects all points in order)
          * \param[in] polygon the polygon to draw
          * \param[in] r the red channel of the color that the polygon should be rendered with
          * \param[in] g the green channel of the color that the polygon should be rendered with
          * \param[in] b the blue channel of the color that the polygon should be rendered with
          * \param[in] id the polygon id/name (default: "polygon")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addPolygon (const pcl::PlanarPolygon<PointT> &polygon,
                    double r, double g, double b,
                    const std::string &id = "polygon",
                    int viewport = 0);

        /** \brief Add a line segment from two points
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] id the line id/name (default: "line")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename P1, typename P2> bool
        addLine (const P1 &pt1, const P2 &pt2, const std::string &id = "line",
                 int viewport = 0);

        /** \brief Add a line segment from two points
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] id the line id/name (default: "line")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename P1, typename P2> bool
        addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b,
                 const std::string &id = "line", int viewport = 0);

        /** \brief Add a line arrow segment between two points, and display the distance between them
          *
          * Arrow heads are attached to both end points of the arrow.
          *
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] id the arrow id/name (default: "arrow")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename P1, typename P2> bool
        addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b,
                  const std::string &id = "arrow", int viewport = 0);

        /** \brief Add a line arrow segment between two points, and (optianally) display the distance between them
          *
          * Arrow head is attached on the **start** point (\c pt1) of the arrow.
          *
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r the red channel of the color that the line should be rendered with
          * \param[in] g the green channel of the color that the line should be rendered with
          * \param[in] b the blue channel of the color that the line should be rendered with
          * \param[in] display_length true if the length should be displayed on the arrow as text
          * \param[in] id the line id/name (default: "arrow")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename P1, typename P2> bool
        addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, bool display_length,
                  const std::string &id = "arrow", int viewport = 0);

        /** \brief Add a line arrow segment between two points, and display the distance between them in a given color
          *
          * Arrow heads are attached to both end points of the arrow.
          *
          * \param[in] pt1 the first (start) point on the line
          * \param[in] pt2 the second (end) point on the line
          * \param[in] r_line the red channel of the color that the line should be rendered with
          * \param[in] g_line the green channel of the color that the line should be rendered with
          * \param[in] b_line the blue channel of the color that the line should be rendered with
          * \param[in] r_text the red channel of the color that the text should be rendered with
          * \param[in] g_text the green channel of the color that the text should be rendered with
          * \param[in] b_text the blue channel of the color that the text should be rendered with
          * \param[in] id the line id/name (default: "arrow")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
	      template <typename P1, typename P2> bool
	      addArrow (const P1 &pt1, const P2 &pt2,
		              double r_line, double g_line, double b_line,
		              double r_text, double g_text, double b_text,
		              const std::string &id = "arrow", int viewport = 0);


        /** \brief Add a sphere shape from a point and a radius
          * \param[in] center the center of the sphere
          * \param[in] radius the radius of the sphere
          * \param[in] id the sphere id/name (default: "sphere")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addSphere (const PointT &center, double radius, const std::string &id = "sphere",
                   int viewport = 0);

        /** \brief Add a sphere shape from a point and a radius
          * \param[in] center the center of the sphere
          * \param[in] radius the radius of the sphere
          * \param[in] r the red channel of the color that the sphere should be rendered with
          * \param[in] g the green channel of the color that the sphere should be rendered with
          * \param[in] b the blue channel of the color that the sphere should be rendered with
          * \param[in] id the sphere id/name (default: "sphere")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        template <typename PointT> bool
        addSphere (const PointT &center, double radius, double r, double g, double b,
                   const std::string &id = "sphere", int viewport = 0);

        /** \brief Update an existing sphere shape from a point and a radius
          * \param[in] center the center of the sphere
          * \param[in] radius the radius of the sphere
          * \param[in] r the red channel of the color that the sphere should be rendered with
          * \param[in] g the green channel of the color that the sphere should be rendered with
          * \param[in] b the blue channel of the color that the sphere should be rendered with
          * \param[in] id the sphere id/name (default: "sphere")
          */
        template <typename PointT> bool
        updateSphere (const PointT &center, double radius, double r, double g, double b,
                      const std::string &id = "sphere");

         /** \brief Add a vtkPolydata as a mesh
          * \param[in] polydata vtkPolyData
          * \param[in] id the model id/name (default: "PolyData")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata,
                              const std::string & id = "PolyData",
                              int viewport = 0);

        /** \brief Add a vtkPolydata as a mesh
          * \param[in] polydata vtkPolyData
          * \param[in] transform transformation to apply
          * \param[in] id the model id/name (default: "PolyData")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addModelFromPolyData (vtkSmartPointer<vtkPolyData> polydata,
                              vtkSmartPointer<vtkTransform> transform,
                              const std::string &id = "PolyData",
                              int viewport = 0);

        /** \brief Add a PLYmodel as a mesh
          * \param[in] filename of the ply file
          * \param[in] id the model id/name (default: "PLYModel")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addModelFromPLYFile (const std::string &filename,
                             const std::string &id = "PLYModel",
                             int viewport = 0);

        /** \brief Add a PLYmodel as a mesh and applies given transformation
          * \param[in] filename of the ply file
          * \param[in] transform transformation to apply
          * \param[in] id the model id/name (default: "PLYModel")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addModelFromPLYFile (const std::string &filename,
                             vtkSmartPointer<vtkTransform> transform,
                             const std::string &id = "PLYModel",
                             int viewport = 0);

        /** \brief Add a cylinder from a set of given model coefficients
          * \param[in] coefficients the model coefficients (point_on_axis, axis_direction, radius)
          * \param[in] id the cylinder id/name (default: "cylinder")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelCylinder for more information.
          * // Eigen::Vector3f pt_on_axis, axis_direction;
          * // float radius;
          *
          * pcl::ModelCoefficients cylinder_coeff;
          * cylinder_coeff.values.resize (7);    // We need 7 values
          * cylinder_coeff.values[0] = pt_on_axis.x ();
          * cylinder_coeff.values[1] = pt_on_axis.y ();
          * cylinder_coeff.values[2] = pt_on_axis.z ();
          *
          * cylinder_coeff.values[3] = axis_direction.x ();
          * cylinder_coeff.values[4] = axis_direction.y ();
          * cylinder_coeff.values[5] = axis_direction.z ();
          *
          * cylinder_coeff.values[6] = radius;
          *
          * addCylinder (cylinder_coeff);
          * \endcode
          */
        bool
        addCylinder (const pcl::ModelCoefficients &coefficients,
                     const std::string &id = "cylinder",
                     int viewport = 0);

        /** \brief Add a sphere from a set of given model coefficients
          * \param[in] coefficients the model coefficients (sphere center, radius)
          * \param[in] id the sphere id/name (default: "sphere")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelSphere for more information
          * // Eigen::Vector3f sphere_center;
          * // float radius;
          *
          * pcl::ModelCoefficients sphere_coeff;
          * sphere_coeff.values.resize (4);    // We need 4 values
          * sphere_coeff.values[0] = sphere_center.x ();
          * sphere_coeff.values[1] = sphere_center.y ();
          * sphere_coeff.values[2] = sphere_center.z ();
          *
          * sphere_coeff.values[3] = radius;
          *
          * addSphere (sphere_coeff);
          * \endcode
          */
        bool
        addSphere (const pcl::ModelCoefficients &coefficients,
                   const std::string &id = "sphere",
                   int viewport = 0);

        /** \brief Add a line from a set of given model coefficients
          * \param[in] coefficients the model coefficients (point_on_line, direction)
          * \param[in] id the line id/name (default: "line")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelLine for more information
          * // Eigen::Vector3f point_on_line, line_direction;
          *
          * pcl::ModelCoefficients line_coeff;
          * line_coeff.values.resize (6);    // We need 6 values
          * line_coeff.values[0] = point_on_line.x ();
          * line_coeff.values[1] = point_on_line.y ();
          * line_coeff.values[2] = point_on_line.z ();
          *
          * line_coeff.values[3] = line_direction.x ();
          * line_coeff.values[4] = line_direction.y ();
          * line_coeff.values[5] = line_direction.z ();
          *
          * addLine (line_coeff);
          * \endcode
          */
        bool
        addLine (const pcl::ModelCoefficients &coefficients,
                 const std::string &id = "line",
                 int viewport = 0);

        /** \brief Add a plane from a set of given model coefficients
          * \param[in] coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
          * \param[in] id the plane id/name (default: "plane")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelPlane for more information
          * // Eigen::Vector4f plane_parameters;
          *
          * pcl::ModelCoefficients plane_coeff;
          * plane_coeff.values.resize (4);    // We need 4 values
          * plane_coeff.values[0] = plane_parameters.x ();
          * plane_coeff.values[1] = plane_parameters.y ();
          * plane_coeff.values[2] = plane_parameters.z ();
          * plane_coeff.values[3] = plane_parameters.w ();
          *
          * addPlane (plane_coeff);
          * \endcode
          */
        bool
        addPlane (const pcl::ModelCoefficients &coefficients,
                  const std::string &id = "plane",
                  int viewport = 0);

        bool
        addPlane (const pcl::ModelCoefficients &coefficients, double x, double y, double z,
                  const std::string &id = "plane",
                  int viewport = 0);
        /** \brief Add a circle from a set of given model coefficients
          * \param[in] coefficients the model coefficients (x, y, radius)
          * \param[in] id the circle id/name (default: "circle")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          *
          * \code
          * // The following are given (or computed using sample consensus techniques)
          * // See SampleConsensusModelCircle2D for more information
          * // float x, y, radius;
          *
          * pcl::ModelCoefficients circle_coeff;
          * circle_coeff.values.resize (3);    // We need 3 values
          * circle_coeff.values[0] = x;
          * circle_coeff.values[1] = y;
          * circle_coeff.values[2] = radius;
          *
          * vtkSmartPointer<vtkDataSet> data = pcl::visualization::create2DCircle (circle_coeff, z);
          * \endcode
           */
        bool
        addCircle (const pcl::ModelCoefficients &coefficients,
                   const std::string &id = "circle",
                   int viewport = 0);

        /** \brief Add a cone from a set of given model coefficients
          * \param[in] coefficients the model coefficients (point_on_axis, axis_direction, radiu)
          * \param[in] id the cone id/name (default: "cone")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addCone (const pcl::ModelCoefficients &coefficients,
                 const std::string &id = "cone",
                 int viewport = 0);

        /** \brief Add a cube from a set of given model coefficients
          * \param[in] coefficients the model coefficients (Tx, Ty, Tz, Qx, Qy, Qz, Qw, width, height, depth)
          * \param[in] id the cube id/name (default: "cube")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addCube (const pcl::ModelCoefficients &coefficients,
                 const std::string &id = "cube",
                 int viewport = 0);

        /** \brief Add a cube from a set of given model coefficients
          * \param[in] translation a translation to apply to the cube from 0,0,0
          * \param[in] rotation a quaternion-based rotation to apply to the cube
          * \param[in] width the cube's width
          * \param[in] height the cube's height
          * \param[in] depth the cube's depth
          * \param[in] id the cube id/name (default: "cube")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addCube (const Eigen::Vector3f &translation, const Eigen::Quaternionf &rotation,
                 double width, double height, double depth,
                 const std::string &id = "cube",
                 int viewport = 0);

        /** \brief Add a cube
          * \param[in] x_min the min X coordinate
          * \param[in] x_max the max X coordinate
          * \param[in] y_min the min Y coordinate
          * \param[in] y_max the max Y coordinate
          * \param[in] z_min the min Z coordinate
          * \param[in] z_max the max Z coordinate
          * \param[in] r how much red (0.0 -> 1.0)
          * \param[in] g how much green (0.0 -> 1.0)
          * \param[in] b how much blue (0.0 -> 1.0)
          * \param[in] id the cube id/name (default: "cube")
          * \param[in] viewport (optional) the id of the new viewport (default: 0)
          */
        bool
        addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
                 double r = 1.0, double g = 1.0, double b = 1.0, const std::string &id = "cube", int viewport = 0);

        /** \brief Changes the visual representation for all actors to surface representation. */
        void
        setRepresentationToSurfaceForAllActors ();

        /** \brief Changes the visual representation for all actors to points representation. */
        void
        setRepresentationToPointsForAllActors ();

        /** \brief Changes the visual representation for all actors to wireframe representation. */
        void
        setRepresentationToWireframeForAllActors ();

        /** \brief Sets whether the 2D overlay text showing the framerate of the window is displayed or not.
          * \param[in] show_fps determines whether the fps text will be shown or not.
          */
        void
        setShowFPS (bool show_fps);

        /** \brief Renders a virtual scene as seen from the camera viewpoint and returns the rendered point cloud.
          * ATT: This method will only render the scene if only on viewport exists. Otherwise, returns an empty
          * point cloud and exits immediately.
          * \param[in] xres is the size of the window (X) used to render the scene
          * \param[in] yres is the size of the window (Y) used to render the scene
          * \param[in] cloud is the rendered point cloud
          */
        void
        renderView (int xres, int yres, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

        /** \brief The purpose of this method is to render a CAD model added to the visualizer from different viewpoints
          * in order to simulate partial views of model. The viewpoint locations are the vertices of a tesselated sphere
          * build from an icosaheadron. The tesselation paremeter controls how many times the triangles of the original
          * icosahedron are divided to approximate the sphere and thus the number of partial view generated for a model,
          * with a tesselation_level of 0, 12 views are generated if use_vertices=true and 20 views if use_vertices=false
          *
          * \param[in] xres the size of the window (X) used to render the partial view of the object
          * \param[in] yres the size of the window (Y) used to render the partial view of the object
          * \param[in] cloud is a vector of pointcloud with XYZ information that represent the model as seen from the respective viewpoints.
          * \param[out] poses represent the transformation from object coordinates to camera coordinates for the respective viewpoint.
          * \param[out] enthropies are values between 0 and 1 representing which percentage of the model is seen from the respective viewpoint.
          * \param[in] tesselation_level represents the number of subdivisions applied to the triangles of original icosahedron.
          * \param[in] view_angle field of view of the virtual camera. Default: 45
          * \param[in] radius_sphere the tesselated sphere radius. Default: 1
          * \param[in] use_vertices if true, use the vertices of tesselated icosahedron (12,42,...) or if false, use the faces of tesselated
          * icosahedron (20,80,...). Default: true
          */
        void
        renderViewTesselatedSphere (
            int xres, int yres,
            pcl::PointCloud<pcl::PointXYZ>::CloudVectorType & cloud,
            std::vector<Eigen::Matrix4f,Eigen::aligned_allocator< Eigen::Matrix4f > > & poses, std::vector<float> & enthropies, int tesselation_level,
            float view_angle = 45, float radius_sphere = 1, bool use_vertices = true);


        /** \brief Initialize camera parameters with some default values. */
        void
        initCameraParameters ();

        /** \brief Search for camera parameters at the command line and set them internally.
          * \param[in] argc
          * \param[in] argv
          */
        bool
        getCameraParameters (int argc, char **argv);

        /** \brief Load camera parameters from a camera parameters file.
          * \param[in] file the name of the camera parameters file
          */
        bool
        loadCameraParameters (const std::string &file);

        /** \brief Checks whether the camera parameters were manually loaded.
          * \return True if valid "-cam" option is available in command line.
          * \sa cameraFileLoaded ()
          */
        bool
        cameraParamsSet () const;

        /** \brief Checks whether a camera file were automatically loaded.
          * \return True if a valid camera file is automatically loaded.
          * \note The camera file is saved by pressing "ctrl + s" during last run of the program
          * and restored automatically when the program runs this time.
          * \sa cameraParamsSet ()
          */
        bool
        cameraFileLoaded () const;

        /** \brief Get camera file for camera parameter saving/restoring.
          * \note This will be valid only when valid "-cam" option were available in command line
          * or a saved camera file were automatically loaded. 
          * \sa cameraParamsSet (), cameraFileLoaded ()
          */
        std::string
        getCameraFile () const;

        /** \brief Update camera parameters and render. */
        void
        updateCamera ();

        /** \brief Reset camera parameters and render. */
        void
        resetCamera ();

        /** \brief Reset the camera direction from {0, 0, 0} to the center_{x, y, z} of a given dataset.
          * \param[in] id the point cloud object id (default: cloud)
          */
        void
        resetCameraViewpoint (const std::string &id = "cloud");

        /** \brief Set the camera pose given by position, viewpoint and up vector
          * \param[in] pos_x the x coordinate of the camera location
          * \param[in] pos_y the y coordinate of the camera location
          * \param[in] pos_z the z coordinate of the camera location
          * \param[in] view_x the x component of the view point of the camera
          * \param[in] view_y the y component of the view point of the camera
          * \param[in] view_z the z component of the view point of the camera
          * \param[in] up_x the x component of the view up direction of the camera
          * \param[in] up_y the y component of the view up direction of the camera
          * \param[in] up_z the y component of the view up direction of the camera
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void
        setCameraPosition (double pos_x, double pos_y, double pos_z,
                           double view_x, double view_y, double view_z,
                           double up_x, double up_y, double up_z, int viewport = 0);

        /** \brief Set the camera location and viewup according to the given arguments
          * \param[in] pos_x the x coordinate of the camera location
          * \param[in] pos_y the y coordinate of the camera location
          * \param[in] pos_z the z coordinate of the camera location
          * \param[in] up_x the x component of the view up direction of the camera
          * \param[in] up_y the y component of the view up direction of the camera
          * \param[in] up_z the z component of the view up direction of the camera
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void
        setCameraPosition (double pos_x, double pos_y, double pos_z,
                           double up_x, double up_y, double up_z, int viewport = 0);

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

        /** \brief Set the camera clipping distances.
          * \param[in] near the near clipping distance (no objects closer than this to the camera will be drawn)
          * \param[in] far the far clipping distance (no objects further away than this to the camera will be drawn)
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void
        setCameraClipDistances (double near, double far, int viewport = 0);

        /** \brief Set the camera vertical field of view.
          * \param[in] fovy vertical field of view in radians
          * \param[in] viewport the viewport to modify camera of (0 modifies all cameras)
          */
        void
        setCameraFieldOfView (double fovy, int viewport = 0);

        /** \brief Get the current camera parameters. */
        void
        getCameras (std::vector<Camera>& cameras);


        /** \brief Get the current viewing pose. */
        Eigen::Affine3f
        getViewerPose (int viewport = 0);

        /** \brief Save the current rendered image to disk, as a PNG screenshot.
          * \param[in] file the name of the PNG file
          */
        void
        saveScreenshot (const std::string &file);

        /** \brief Save the camera parameters to disk, as a .cam file.
          * \param[in] file the name of the .cam file
          */
        void
        saveCameraParameters (const std::string &file);

        /** \brief Get camera parameters and save them to a pcl::visualization::Camera.
          * \param[out] camera the name of the pcl::visualization::Camera
          */
        void
        getCameraParameters (Camera &camera);

        /** \brief Return a pointer to the underlying VTK Render Window used. */
        vtkSmartPointer<vtkRenderWindow>
        getRenderWindow ()
        {
          return (win_);
        }
        
        /** \brief Return a pointer to the underlying VTK Renderer Collection. */
        vtkSmartPointer<vtkRendererCollection>
        getRendererCollection ()
        {
          return (rens_);
        }
        
        /** \brief Return a pointer to the CloudActorMap this visualizer uses. */
        CloudActorMapPtr
        getCloudActorMap ()
        {
          return (cloud_actor_map_);
        }
        
        /** \brief Return a pointer to the ShapeActorMap this visualizer uses. */
        ShapeActorMapPtr
        getShapeActorMap ()
        {
          return (shape_actor_map_);
        }

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

        /** \brief Use Vertex Buffer Objects renderers.
          * \param[in] use_vbos set to true to use VBOs 
          */
        void
        setUseVbos (bool use_vbos);

        /** \brief Create the internal Interactor object. */
        void
        createInteractor ();

        /** \brief Set up our unique PCL interactor style for a given vtkRenderWindowInteractor object
          * attached to a given vtkRenderWindow
          * \param[in,out] iren the vtkRenderWindowInteractor object to set up
          * \param[in,out] win a vtkRenderWindow object that the interactor is attached to
          */
        void
        setupInteractor (vtkRenderWindowInteractor *iren,
                         vtkRenderWindow *win);

        /** \brief Set up PCLVisualizer with custom interactor style for a given vtkRenderWindowInteractor object
          * attached to a given vtkRenderWindow
          * \param[in,out] iren the vtkRenderWindowInteractor object to set up
          * \param[in,out] win a vtkRenderWindow object that the interactor is attached to
          * \param[in,out] style a vtkInteractorStyle object 
          */
        void
        setupInteractor (vtkRenderWindowInteractor *iren,
                         vtkRenderWindow *win,
                         vtkInteractorStyle *style);
        
        /** \brief Get a pointer to the current interactor style used. */
        inline vtkSmartPointer<PCLVisualizerInteractorStyle>
        getInteractorStyle ()
        {
          return (style_);
        }
      protected:
        /** \brief The render window interactor. */
#if ((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
        vtkSmartPointer<PCLVisualizerInteractor> interactor_;
#else
        vtkSmartPointer<vtkRenderWindowInteractor> interactor_;
#endif
      private:
        struct ExitMainLoopTimerCallback : public vtkCommand
        {
          static ExitMainLoopTimerCallback* New ()
          {
            return (new ExitMainLoopTimerCallback);
          }
          virtual void 
          Execute (vtkObject*, unsigned long event_id, void*);

          int right_timer_id;
          PCLVisualizer* pcl_visualizer;
        };

        struct ExitCallback : public vtkCommand
        {
          static ExitCallback* New ()
          {
            return (new ExitCallback);
          }
          virtual void 
          Execute (vtkObject*, unsigned long event_id, void*);

          PCLVisualizer* pcl_visualizer;
        };

        //////////////////////////////////////////////////////////////////////////////////////////////
        struct FPSCallback : public vtkCommand
        {
          static FPSCallback *New () { return (new FPSCallback); }

          FPSCallback () : actor (), pcl_visualizer (), decimated () {}
          FPSCallback (const FPSCallback& src) : vtkCommand (), actor (src.actor), pcl_visualizer (src.pcl_visualizer), decimated (src.decimated) {}
          FPSCallback& operator = (const FPSCallback& src) { actor = src.actor; pcl_visualizer = src.pcl_visualizer; decimated = src.decimated; return (*this); }

          virtual void 
          Execute (vtkObject*, unsigned long event_id, void*);
            
          vtkTextActor *actor;
          PCLVisualizer* pcl_visualizer;
          bool decimated;
        };

        /** \brief The FPSCallback object for the current visualizer. */
        vtkSmartPointer<FPSCallback> update_fps_;

#if !((VTK_MAJOR_VERSION == 5) && (VTK_MINOR_VERSION <= 4))
        /** \brief Set to false if the interaction loop is running. */
        bool stopped_;

        /** \brief Global timer ID. Used in destructor only. */
        int timer_id_;
#endif
        /** \brief Callback object enabling us to leave the main loop, when a timer fires. */
        vtkSmartPointer<ExitMainLoopTimerCallback> exit_main_loop_timer_callback_;
        vtkSmartPointer<ExitCallback> exit_callback_;

        /** \brief The collection of renderers used. */
        vtkSmartPointer<vtkRendererCollection> rens_;

        /** \brief The render window. */
        vtkSmartPointer<vtkRenderWindow> win_;

        /** \brief The render window interactor style. */
        vtkSmartPointer<PCLVisualizerInteractorStyle> style_;

        /** \brief Internal list with actor pointers and name IDs for point clouds. */
        CloudActorMapPtr cloud_actor_map_;

        /** \brief Internal list with actor pointers and name IDs for shapes. */
        ShapeActorMapPtr shape_actor_map_;

        /** \brief Internal list with actor pointers and viewpoint for coordinates. */
        CoordinateActorMapPtr coordinate_actor_map_;

        /** \brief Internal pointer to widget which contains a set of axes */
        vtkSmartPointer<vtkOrientationMarkerWidget> axes_widget_;
        
        /** \brief Boolean that holds whether or not the camera parameters were manually initialized */
        bool camera_set_;

        /** \brief Boolean that holds whether or not a camera file were automatically loaded */
        bool camera_file_loaded_;

        /** \brief Boolean that holds whether or not to use the vtkVertexBufferObjectMapper*/
        bool use_vbos_;

        /** \brief Internal method. Removes a vtk actor from the screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport the view port where the actor should be removed from (default: all)
          */
        bool
        removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor,
                                 int viewport = 0);

        /** \brief Internal method. Removes a vtk actor from the screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport the view port where the actor should be removed from (default: all)
          */
        bool
        removeActorFromRenderer (const vtkSmartPointer<vtkActor> &actor,
                                 int viewport = 0);

        /** \brief Internal method. Adds a vtk actor to screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport port where the actor should be added to (default: 0/all)
          *
          * \note If viewport is set to 0, the actor will be added to all existing 
          * renders. To select a specific viewport use an integer between 1 and N.
          */
        void
        addActorToRenderer (const vtkSmartPointer<vtkProp> &actor,
                            int viewport = 0);

        /** \brief Internal method. Adds a vtk actor to screen.
          * \param[in] actor a pointer to the vtk actor object
          * \param[in] viewport the view port where the actor should be added to (default: all)
          */
        bool
        removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor,
                                 int viewport = 0);

        /** \brief Internal method. Creates a vtk actor from a vtk polydata object.
          * \param[in] data the vtk polydata object to create an actor for
          * \param[out] actor the resultant vtk actor object
          * \param[in] use_scalars set scalar properties to the mapper if it exists in the data. Default: true.
          */
        void
        createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data,
                                   vtkSmartPointer<vtkActor> &actor,
                                   bool use_scalars = true);

        /** \brief Internal method. Creates a vtk actor from a vtk polydata object.
          * \param[in] data the vtk polydata object to create an actor for
          * \param[out] actor the resultant vtk actor object
          * \param[in] use_scalars set scalar properties to the mapper if it exists in the data. Default: true.
          */
        void
        createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data,
                                   vtkSmartPointer<vtkLODActor> &actor,
                                   bool use_scalars = true);

        /** \brief Converts a PCL templated PointCloud object to a vtk polydata object.
          * \param[in] cloud the input PCL PointCloud dataset
          * \param[out] polydata the resultant polydata containing the cloud
          * \param[out] initcells a list of cell indices used for the conversion. This can be set once and then passed
          * around to speed up the conversion.
          */
        template <typename PointT> void
        convertPointCloudToVTKPolyData (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                        vtkSmartPointer<vtkPolyData> &polydata,
                                        vtkSmartPointer<vtkIdTypeArray> &initcells);

        /** \brief Converts a PCL templated PointCloud object to a vtk polydata object.
          * \param[in] geometry_handler the geometry handler object used to extract the XYZ data
          * \param[out] polydata the resultant polydata containing the cloud
          * \param[out] initcells a list of cell indices used for the conversion. This can be set once and then passed
          * around to speed up the conversion.
          */
        template <typename PointT> void
        convertPointCloudToVTKPolyData (const PointCloudGeometryHandler<PointT> &geometry_handler,
                                        vtkSmartPointer<vtkPolyData> &polydata,
                                        vtkSmartPointer<vtkIdTypeArray> &initcells);

        /** \brief Converts a PCL templated PointCloud object to a vtk polydata object.
          * \param[in] geometry_handler the geometry handler object used to extract the XYZ data
          * \param[out] polydata the resultant polydata containing the cloud
          * \param[out] initcells a list of cell indices used for the conversion. This can be set once and then passed
          * around to speed up the conversion.
          */
        void
        convertPointCloudToVTKPolyData (const GeometryHandlerConstPtr &geometry_handler,
                                        vtkSmartPointer<vtkPolyData> &polydata,
                                        vtkSmartPointer<vtkIdTypeArray> &initcells);

        /** \brief Updates a set of cells (vtkIdTypeArray) if the number of points in a cloud changes
          * \param[out] cells the vtkIdTypeArray object (set of cells) to update
          * \param[out] initcells a previously saved set of cells. If the number of points in the current cloud is
          * higher than the number of cells in \a cells, and initcells contains enough data, then a copy from it
          * will be made instead of regenerating the entire array.
          * \param[in] nr_points the number of points in the new cloud. This dictates how many cells we need to
          * generate
          */
        void
        updateCells (vtkSmartPointer<vtkIdTypeArray> &cells,
                     vtkSmartPointer<vtkIdTypeArray> &initcells,
                     vtkIdType nr_points);

        /** \brief Internal function which converts the information present in the geometric
          * and color handlers into VTK PolyData+Scalars, constructs a vtkActor object, and adds
          * all the required information to the internal cloud_actor_map_ object.
          * \param[in] geometry_handler the geometric handler that contains the XYZ data
          * \param[in] color_handler the color handler that contains the "RGB" (scalar) data
          * \param[in] id the point cloud object id
          * \param[in] viewport the view port where the Point Cloud should be added
          * \param[in] sensor_origin the origin of the cloud data in global coordinates (defaults to 0,0,0)
          * \param[in] sensor_orientation the orientation of the cloud data in global coordinates (defaults to 1,0,0,0)
          */
        template <typename PointT> bool
        fromHandlersToScreen (const PointCloudGeometryHandler<PointT> &geometry_handler,
                              const PointCloudColorHandler<PointT> &color_handler,
                              const std::string &id,
                              int viewport,
                              const Eigen::Vector4f& sensor_origin = Eigen::Vector4f (0, 0, 0, 0),
                              const Eigen::Quaternion<float>& sensor_orientation = Eigen::Quaternion<float> (1, 0, 0 ,0));

        /** \brief Internal function which converts the information present in the geometric
          * and color handlers into VTK PolyData+Scalars, constructs a vtkActor object, and adds
          * all the required information to the internal cloud_actor_map_ object.
          * \param[in] geometry_handler the geometric handler that contains the XYZ data
          * \param[in] color_handler the color handler that contains the "RGB" (scalar) data
          * \param[in] id the point cloud object id
          * \param[in] viewport the view port where the Point Cloud should be added
          * \param[in] sensor_origin the origin of the cloud data in global coordinates (defaults to 0,0,0)
          * \param[in] sensor_orientation the orientation of the cloud data in global coordinates (defaults to 1,0,0,0)
          */
        template <typename PointT> bool
        fromHandlersToScreen (const PointCloudGeometryHandler<PointT> &geometry_handler,
                              const ColorHandlerConstPtr &color_handler,
                              const std::string &id,
                              int viewport,
                              const Eigen::Vector4f& sensor_origin = Eigen::Vector4f (0, 0, 0, 0),
                              const Eigen::Quaternion<float>& sensor_orientation = Eigen::Quaternion<float> (1, 0, 0 ,0));

        /** \brief Internal function which converts the information present in the geometric
          * and color handlers into VTK PolyData+Scalars, constructs a vtkActor object, and adds
          * all the required information to the internal cloud_actor_map_ object.
          * \param[in] geometry_handler the geometric handler that contains the XYZ data
          * \param[in] color_handler the color handler that contains the "RGB" (scalar) data
          * \param[in] id the point cloud object id
          * \param[in] viewport the view port where the Point Cloud should be added
          * \param[in] sensor_origin the origin of the cloud data in global coordinates (defaults to 0,0,0)
          * \param[in] sensor_orientation the orientation of the cloud data in global coordinates (defaults to 1,0,0,0)
          */
        bool
        fromHandlersToScreen (const GeometryHandlerConstPtr &geometry_handler,
                              const ColorHandlerConstPtr &color_handler,
                              const std::string &id,
                              int viewport,
                              const Eigen::Vector4f& sensor_origin = Eigen::Vector4f (0, 0, 0, 0),
                              const Eigen::Quaternion<float>& sensor_orientation = Eigen::Quaternion<float> (1, 0, 0 ,0));

        /** \brief Internal function which converts the information present in the geometric
          * and color handlers into VTK PolyData+Scalars, constructs a vtkActor object, and adds
          * all the required information to the internal cloud_actor_map_ object.
          * \param[in] geometry_handler the geometric handler that contains the XYZ data
          * \param[in] color_handler the color handler that contains the "RGB" (scalar) data
          * \param[in] id the point cloud object id
          * \param[in] viewport the view port where the Point Cloud should be added
          * \param[in] sensor_origin the origin of the cloud data in global coordinates (defaults to 0,0,0)
          * \param[in] sensor_orientation the orientation of the cloud data in global coordinates (defaults to 1,0,0,0)
          */
        template <typename PointT> bool
        fromHandlersToScreen (const GeometryHandlerConstPtr &geometry_handler,
                              const PointCloudColorHandler<PointT> &color_handler,
                              const std::string &id,
                              int viewport,
                              const Eigen::Vector4f& sensor_origin = Eigen::Vector4f (0, 0, 0, 0),
                              const Eigen::Quaternion<float>& sensor_orientation = Eigen::Quaternion<float> (1, 0, 0 ,0));

        /** \brief Allocate a new polydata smartpointer. Internal
          * \param[out] polydata the resultant poly data
          */
        void
        allocVtkPolyData (vtkSmartPointer<vtkAppendPolyData> &polydata);

        /** \brief Allocate a new polydata smartpointer. Internal
          * \param[out] polydata the resultant poly data
          */
        void
        allocVtkPolyData (vtkSmartPointer<vtkPolyData> &polydata);

        /** \brief Allocate a new unstructured grid smartpointer. Internal
          * \param[out] polydata the resultant poly data
          */
        void
        allocVtkUnstructuredGrid (vtkSmartPointer<vtkUnstructuredGrid> &polydata);

        /** \brief Transform the point cloud viewpoint to a transformation matrix
          * \param[in] origin the camera origin
          * \param[in] orientation the camera orientation
          * \param[out] transformation the camera transformation matrix
          */
        void
        getTransformationMatrix (const Eigen::Vector4f &origin,
                                 const Eigen::Quaternion<float> &orientation,
                                 Eigen::Matrix4f &transformation);

        /** \brief Fills a vtkTexture structure from pcl::TexMaterial.
          * \param[in] tex_mat texture material in PCL format
          * \param[out] vtk_tex texture material in VTK format
          * \return 0 on success and -1 else.
          * \note for now only image based textures are supported, image file must be in 
          * tex_file attribute of \a tex_mat.
          */
        int
        textureFromTexMaterial (const pcl::TexMaterial& tex_mat,
                                vtkTexture* vtk_tex) const;

        /** \brief Get camera file for camera parameter saving/restoring from command line.
          * Camera filename is calculated using sha1 value of all pathes of input .pcd files
          * \return empty string if failed.
          */
        std::string
        getUniqueCameraFile (int argc, char **argv);
        
        //There's no reason these conversion functions shouldn't be public and static so others can use them.
      public:
        /** \brief Convert Eigen::Matrix4f to vtkMatrix4x4
          * \param[in] m the input Eigen matrix
          * \param[out] vtk_matrix the resultant VTK matrix
          */
        static void
        convertToVtkMatrix (const Eigen::Matrix4f &m,
                            vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);

        /** \brief Convert origin and orientation to vtkMatrix4x4
          * \param[in] origin the point cloud origin
          * \param[in] orientation the point cloud orientation
          * \param[out] vtk_matrix the resultant VTK 4x4 matrix
          */
        static void
        convertToVtkMatrix (const Eigen::Vector4f &origin,
                            const Eigen::Quaternion<float> &orientation,
                            vtkSmartPointer<vtkMatrix4x4> &vtk_matrix);
        
        /** \brief Convert vtkMatrix4x4 to an Eigen4f
          * \param[in] vtk_matrix the original VTK 4x4 matrix
          * \param[out] m the resultant Eigen 4x4 matrix
          */
        static void
        convertToEigenMatrix (const vtkSmartPointer<vtkMatrix4x4> &vtk_matrix,
                              Eigen::Matrix4f &m);

    };
  }
}

#include <pcl/visualization/impl/pcl_visualizer.hpp>

#endif

