/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 * $Id: pcl_visualizer.h 36093 2011-02-19 21:54:55Z rusu $
 *
 */
#ifndef PCL_PCL_VISUALIZER_H_
#define PCL_PCL_VISUALIZER_H_

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
// 
#include <pcl/terminal_tools/print.h>
#include <pcl/visualization/interactor.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/common/shapes.h>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
// VTK includes
#include <vtkAxes.h>
#include <vtkFloatArray.h>
#include <vtkAppendPolyData.h>
#include <vtkPointData.h>
#include <vtkTubeFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSetMapper.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkPLYReader.h>
#include <vtkTransformFilter.h>

namespace pcl_visualization
{
  /** \brief PCL Visualizer main class. 
    * \author Radu Bogdan Rusu
    */
  class PCLVisualizer
  {
    public:
      typedef PointCloudGeometryHandler<sensor_msgs::PointCloud2> GeometryHandler;
      typedef GeometryHandler::Ptr GeometryHandlerPtr;
      typedef GeometryHandler::ConstPtr GeometryHandlerConstPtr;

      typedef PointCloudColorHandler<sensor_msgs::PointCloud2> ColorHandler;
      typedef ColorHandler::Ptr ColorHandlerPtr;
      typedef ColorHandler::ConstPtr ColorHandlerConstPtr;

      /** \brief PCL Visualizer constructor.
        * \param name the window name (empty by default)
        */
      PCLVisualizer (const std::string &name = "");
      /** \brief PCL Visualizer constructor.
        * \param argc
        * \param argv
        * \param name the window name (empty by default)
        * \param interactor style (defaults to PCLVisualizerInteractorStyle)
        */
      PCLVisualizer (int &argc, char **argv, const std::string &name = "", 
          PCLVisualizerInteractorStyle* style = PCLVisualizerInteractorStyle::New ());

      /** \brief PCL Visualizer destructor. */
      virtual ~PCLVisualizer ();

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

      /** \brief Adds 3D axes describing a coordinate system to screen at 0,0,0.
        * \param scale the scale of the axes (default: 1)
        * \param viewport the view port where the 3D axes should be added (default: all)
        */
      void 
      addCoordinateSystem (double scale = 1.0, int viewport = 0);
      
      /** \brief Adds 3D axes describing a coordinate system to screen at x, y, z
        * \param scale the scale of the axes (default: 1)
        * \param x the X position of the axes
        * \param y the Y position of the axes
        * \param z the Z position of the axes
        * \param viewport the view port where the 3D axes should be added (default: all)
        */
      void 
      addCoordinateSystem (double scale, float x, float y, float z, int viewport = 0);

      /** \brief Removes a previously added 3D axes (coordinate system)
        * \param viewport view port where the 3D axes should be removed from (default: all)
        */
      void 
      removeCoordinateSystem (int viewport = 0);

      /** \brief Removes a Point Cloud from screen, based on a given ID.
        * \param id the point cloud object id (i.e., given on \a addPointCloud)
        * \param viewport view port from where the Point Cloud should be removed (default: all)
        */
      bool 
      removePointCloud (const std::string &id = "cloud", int viewport = 0);

      /** \brief Removes an added shape from screen (line, polygon, etc.), based on a given ID
        * \param id the shape object id (i.e., given on \a addLine etc.)
        * \param viewport view port from where the Point Cloud should be removed (default: all)
        */
      bool 
      removeShape (const std::string &id = "cloud", int viewport = 0);

      /** \brief Set the viewport's background color.
        * \param r the red component of the RGB color
        * \param g the green component of the RGB color
        * \param b the blue component of the RGB color
        * \param viewport the view port (default: all)
        */
      void 
      setBackgroundColor (const double &r, const double &g, const double &b, int viewport = 0);

      /** \brief Add a text to screen
        * \param text the text to add
        * \param xpos the X position on screen where the text should be added
        * \param ypos the Y position on screen where the text should be added
        * \param id the text object id (default: equal to the "text" parameter)
        * \param viewport the view port (default: all)
        */
      bool 
      addText (const std::string &text, int xpos, int ypos, const std::string &id = "", int viewport = 0);

      /** \brief Add a text to screen
        * \param text the text to add
        * \param xpos the X position on screen where the text should be added
        * \param ypos the Y position on screen where the text should be added
        * \param r the red color value
        * \param g the green color value
        * \param b the blue color vlaue
        * \param id the text object id (default: equal to the "text" parameter)
        * \param viewport the view port (default: all)
        */
      bool 
      addText (const std::string &text, int xpos, int ypos, int r, int g, int b, const std::string &id = "", int viewport = 0);

      /** \brief Add the estimated surface normals of a Point Cloud to screen. 
        * \param cloud the input point cloud dataset containing XYZ data and normals
        * \param level display only every level'th point (default: 100)
        * \param scale the normal arrow scale (default: 0.02m)
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointNT>
      bool addPointCloudNormals (const pcl::PointCloud<PointNT> &cloud,
                                 int level = 100, double scale = 0.02,
                                 const std::string &id = "cloud", int viewport = 0);
      /** \brief Add the estimated surface normals of a Point Cloud to screen.
        * \param cloud the input point cloud dataset containing the XYZ data
        * \param normals the input point cloud dataset containing the normal data
        * \param level display only every level'th point (default: 100)
        * \param scale the normal arrow scale (default: 0.02m)
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT, typename PointNT>
      bool addPointCloudNormals (const pcl::PointCloud<PointT> &cloud,
                                 const pcl::PointCloud<PointNT> &normals,
                                 int level = 100, double scale = 0.02,
                                 const std::string &id = "cloud", int viewport = 0);
      /** \brief Add the estimated principal curvatures of a Point Cloud to screen. 
        * \param cloud the input point cloud dataset containing the XYZ data
        * \param normals the input point cloud dataset containing the normal data
        * \param pcs the input point cloud dataset containing the principal curvatures data
        * \param level display only every level'th point (default: 100)
        * \param scale the normal arrow scale (default: 1.0))
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      bool 
      addPointCloudPrincipalCurvatures (const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                                        const pcl::PointCloud<pcl::Normal> &normals,
                                        const pcl::PointCloud<pcl::PrincipalCurvatures> &pcs,
                                        int level = 100, double scale = 1.0,
                                        const std::string &id = "cloud", int viewport = 0);
 
      /** \brief Add a Point Cloud to screen. 
        * \param cloud the input point cloud dataset
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      bool 
      addPointCloud (const pcl::PointCloud<pcl::PointXYZ> &cloud, 
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a Point Cloud (templated) to screen. 
        * \param cloud the input point cloud dataset
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT> bool 
      addPointCloud (const pcl::PointCloud<PointT> &cloud, 
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a Point Cloud (templated) to screen. 
        * \param cloud the input point cloud dataset
        * \param geometry_handler use a geometry handler object to extract the XYZ data
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT> bool 
      addPointCloud (const pcl::PointCloud<PointT> &cloud, 
                     const PointCloudGeometryHandler<PointT> &geometry_handler,
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a Point Cloud (templated) to screen. 
        * \param cloud the input point cloud dataset
        * \param geometry_handler use a geometry handler object to extract the XYZ data
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT> bool 
      addPointCloud (const pcl::PointCloud<PointT> &cloud, 
                     const GeometryHandlerConstPtr &geometry_handler,
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a Point Cloud (templated) to screen. 
        * \param cloud the input point cloud dataset
        * \param color_handler a specific PointCloud visualizer handler for colors
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT> bool 
      addPointCloud (const pcl::PointCloud<PointT> &cloud, 
                     const PointCloudColorHandler<PointT> &color_handler,
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a Point Cloud (templated) to screen. 
        * \param cloud the input point cloud dataset
        * \param color_handler a specific PointCloud visualizer handler for colors
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT> bool 
      addPointCloud (const pcl::PointCloud<PointT> &cloud, 
                     const ColorHandlerConstPtr &color_handler,
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a Point Cloud (templated) to screen. 
        * \param cloud the input point cloud dataset
        * \param geometry_handler a specific PointCloud visualizer handler for geometry
        * \param color_handler a specific PointCloud visualizer handler for colors
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT> bool 
      addPointCloud (const pcl::PointCloud<PointT> &cloud, 
                     const GeometryHandlerConstPtr &geometry_handler,
                     const ColorHandlerConstPtr &color_handler,
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a Point Cloud (templated) to screen. 
        * \param cloud the input point cloud dataset
        * \param color_handler a specific PointCloud visualizer handler for colors
        * \param geometry_handler use a geometry handler object to extract the XYZ data
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud should be added (default: all)
        */
      template <typename PointT> bool 
      addPointCloud (const pcl::PointCloud<PointT> &cloud, 
                     const PointCloudColorHandler<PointT> &color_handler,
                     const PointCloudGeometryHandler<PointT> &geometry_handler,
                     const std::string &id = "cloud", int viewport = 0);

      /** \brief Add a PolygonMesh object to screen
        * \param polymesh the polygonal mesh
        * \param id the polygon object id (default: "polygon")
        * \param viewport the view port where the PolygonMesh should be added (default: all)
        */
      bool
      addPolygonMesh (const pcl::PolygonMesh &poly_mesh, const std::string &id = "polygon", 
                      int viewport = 0);

      /** \brief Get the color handler index of a rendered PointCloud based on its ID
        * \param id the point cloud object id
        */
      inline int 
      getColorHandlerIndex (const std::string &id)
      {
        CloudActorMap::iterator am_it = style_->getCloudActorMap ()->find (id);
        if (am_it == cloud_actor_map_.end ())
          return (-1);

        return (am_it->second.color_handler_index_);
      }
      
      /** \brief Get the geometry handler index of a rendered PointCloud based on its ID
        * \param id the point cloud object id
        */
      inline int 
      getGeometryHandlerIndex (const std::string &id)
      {
        CloudActorMap::iterator am_it = style_->getCloudActorMap ()->find (id);
        if (am_it != cloud_actor_map_.end ())
          return (-1);

        return (am_it->second.geometry_handler_index_);
      }

      /** \brief Update/set the color index of a renderered PointCloud based on its ID
        * \param id the point cloud object id
        * \param index the color handler index to use
        */
      bool 
      updateColorHandlerIndex (const std::string &id, int index);

      /** \brief Set the rendering properties of a PointCloud (3x values - e.g., RGB)
        * \param property the property type
        * \param val1 the first value to be set
        * \param val2 the second value to be set
        * \param val2 the third value to be set
        * \param id the point cloud object id (default: cloud)
        * \param viewport the view port where the Point Cloud's rendering properties should be modified (default: all)
        */
      bool 
      setPointCloudRenderingProperties (int property, double val1, double val2, double val3, 
                                        const std::string &id = "cloud", int viewport = 0);

     /** \brief Set the rendering properties of a PointCloud
       * \param property the property type
       * \param value the value to be set
       * \param id the point cloud object id (default: cloud)
       * \param viewport the view port where the Point Cloud's rendering properties should be modified (default: all)
       */
      bool 
      setPointCloudRenderingProperties (int property, double value, 
                                        const std::string &id = "cloud", int viewport = 0);

     /** \brief Get the rendering properties of a PointCloud
       * \param property the property type
       * \param value the resultant property value
       * \param id the point cloud object id (default: cloud)
       */
      bool 
      getPointCloudRenderingProperties (int property, double &value, 
                                        const std::string &id = "cloud");
      
     /** \brief Set the rendering properties of a shape
       * \param property the property type
       * \param value the value to be set
       * \param id the shape object id
       * \param viewport the view port where the shape's properties should be modified (default: all)
       */
      bool 
      setShapeRenderingProperties (int property, double value, 
                                   const std::string &id, int viewport = 0);

      /** \brief Returns true when the user tried to close the window */
      bool 
      wasStopped () const { return (interactor_->stopped); }
      /** \brief Set the stopped flag back to false */
      void 
      resetStoppedFlag () { interactor_->stopped = false; }

      /** \brief Create a new viewport from [xmin,ymin] -> [xmax,ymax].
        * \param xmin the minimum X coordinate for the viewport (0.0 <= 1.0)
        * \param ymin the minimum Y coordinate for the viewport (0.0 <= 1.0)
        * \param xmax the maximum X coordinate for the viewport (0.0 <= 1.0)
        * \param ymax the maximum Y coordinate for the viewport (0.0 <= 1.0)
        * \param viewport the id of the new viewport
        */
      void 
      createViewPort (double xmin, double ymin, double xmax, double ymax, int &viewport);

      /** \brief Add a polygon (polyline) that represents the input point cloud (connects all 
        * points in order)
        * \param cloud the point cloud dataset representing the polygon
        * \param r the red channel of the color that the polygon should be rendered with
        * \param g the green channel of the color that the polygon should be rendered with
        * \param b the blue channel of the color that the polygon should be rendered with
        * \param id (optional) the polygon id/name (default: "polygon")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      template <typename PointT> bool
      addPolygon (const pcl::PointCloud<PointT> &cloud, double r, double g, double b, 
                  const std::string &id = "polygon", int viewport = 0);

      /** \brief Add a polygon (polyline) that represents the input point cloud (connects all 
        * points in order)
        * \param cloud the point cloud dataset representing the polygon
        * \param id the polygon id/name (default: "polygon")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      template <typename PointT> bool
      addPolygon (const pcl::PointCloud<PointT> &cloud, const std::string &id = "polygon", 
                  int viewport = 0);

      /** \brief Add a line segment from two points
        * \param pt1 the first (start) point on the line
        * \param pt2 the second (end) point on the line
        * \param id the line id/name (default: "line")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      template <typename P1, typename P2> bool 
      addLine (const P1 &pt1, const P2 &pt2, const std::string &id = "line", 
               int viewport = 0);

      /** \brief Add a line segment from two points
        * \param pt1 the first (start) point on the line
        * \param pt2 the second (end) point on the line
        * \param r the red channel of the color that the line should be rendered with
        * \param g the green channel of the color that the line should be rendered with
        * \param b the blue channel of the color that the line should be rendered with
        * \param id the line id/name (default: "line")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      template <typename P1, typename P2> bool 
      addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b, 
               const std::string &id = "line", int viewport = 0);

      /** \brief Add a line segment from two points to a group of lines. If the group doesn't 
        * exist, it will get created.
        * \param pt1 the first (start) point on the line
        * \param pt2 the second (end) point on the line
        * \param group_id the line group id/name (default: "line_group")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
/*      template <typename P1, typename P2> inline bool addLineToGroup (const P1 &pt1, const P2 &pt2, const std::string &group_id = "line_group", int viewport = 0);*/
      
      /** \brief Add a sphere shape from a point and a radius
        * \param center the center of the sphere
        * \param radius the radius of the sphere
        * \param id the line id/name (default: "sphere")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      template <typename PointT> bool 
      addSphere (const PointT &center, double radius, const std::string &id = "sphere", 
                 int viewport = 0);

      /** \brief Add a sphere shape from a point and a radius
        * \param center the center of the sphere
        * \param radius the radius of the sphere
        * \param r the red channel of the color that the sphere should be rendered with
        * \param g the green channel of the color that the sphere should be rendered with
        * \param b the blue channel of the color that the sphere should be rendered with
        * \param id the line id/name (default: "sphere")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      template <typename PointT> bool 
      addSphere (const PointT &center, double radius, double r, double g, double b, 
                 const std::string &id = "sphere", int viewport = 0);

      /** \brief Add a PLYmodel as a mesh
       * \param filename of the ply file
       * \param id the model id/name (default: "PLYModel")
       * \param viewport (optional) the id of the new viewport (default: 0)
       */
      bool 
      addModelFromPLYFile (const std::string & filename, const std::string & id = "PLYModel", 
                           int viewport = 0);

      /** \brief Add a PLYmodel as a mesh and applies given transformation
       * \param filename of the ply file
       * \param transform transformation to apply
       * \param id the model id/name (default: "PLYModel")
       * \param viewport (optional) the id of the new viewport (default: 0)
       */

      bool 
      addModelFromPLYFile (const std::string & filename, 
                           vtkSmartPointer<vtkTransform> transform, 
                           const std::string & id = "PLYModel", int viewport = 0);
      /** \brief Add a cylinder from a set of given model coefficients 
        * \param coefficients the model coefficients (point_on_axis, axis_direction, radius)
        * \param id the cylinder id/name (default: "cylinder")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      bool 
      addCylinder (const pcl::ModelCoefficients &coefficients, 
                   const std::string &id = "cylinder", int viewport = 0);

      /** \brief Add a sphere from a set of given model coefficients 
        * \param coefficients the model coefficients (sphere center, radius)
        * \param id the sphere id/name (default: "sphere")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      bool 
      addSphere (const pcl::ModelCoefficients &coefficients, 
                 const std::string &id = "sphere", int viewport = 0);
      /** \brief Add a line from a set of given model coefficients 
        * \param coefficients the model coefficients (point_on_line, direction)
        * \param id the line id/name (default: "line")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      bool 
      addLine (const pcl::ModelCoefficients &coefficients, 
               const std::string &id = "line", int viewport = 0);
      /** \brief Add a plane from a set of given model coefficients 
        * \param coefficients the model coefficients (a, b, c, d with ax+by+cz+d=0)
        * \param id the plane id/name (default: "plane")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      bool 
      addPlane (const pcl::ModelCoefficients &coefficients, 
                const std::string &id = "plane", int viewport = 0);
      /** \brief Add a circle from a set of given model coefficients 
        * \param coefficients the model coefficients (x, y, radius)
        * \param id the circle id/name (default: "circle")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      bool 
      addCircle (const pcl::ModelCoefficients &coefficients, 
                 const std::string &id = "circle", int viewport = 0);
      /** \brief Add a cone from a set of given model coefficients 
        * \param coefficients the model coefficients (point_on_axis, axis_direction, radiu)
        * \param id the cone id/name (default: "cone")
        * \param viewport (optional) the id of the new viewport (default: 0)
        */
      bool 
      addCone (const pcl::ModelCoefficients &coefficients, 
               const std::string &id = "cone", int viewport = 0);

      /** \brief Camera view, window position and size. */
      Camera camera_;

      /** \brief Initialize camera parameters with some default values. */
      void 
      initCameraParameters ();

      /** \brief Search for camera parameters at the command line and set them internally.
        * \param argc
        * \param argv
        */
      bool 
      getCameraParameters (int argc, char **argv);

      /** \brief Update camera parameters and render. */
      void 
      updateCamera ();

      /** \brief Reset camera parameters and render. */
      void 
      resetCamera ();

    protected:
      /** \brief The render window interactor. */
      vtkSmartPointer<PCLVisualizerInteractor> interactor_;

    private:
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
          pcl_visualizer->interactor_->stopLoop ();
        }
        int right_timer_id;
        PCLVisualizer* pcl_visualizer;
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
          //cout << "Exit event called.\n";
          pcl_visualizer->interactor_->stopped = true;
        }
        PCLVisualizer* pcl_visualizer;
      };

      
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
      CloudActorMap cloud_actor_map_;

      /** \brief Internal list with actor pointers and name IDs for shapes. */
      ShapeActorMap shape_actor_map_;

    
      /** \brief Internal method. Removes a vtk actor from the screen.
        * \param actor a pointer to the vtk actor object
        * \param viewport the view port where the actor should be removed from (default: all)
        */
      void removeActorFromRenderer (const vtkSmartPointer<vtkLODActor> &actor, int viewport = 0);

      /** \brief Internal method. Adds a vtk actor to screen.
        * \param actor a pointer to the vtk actor object
        * \param viewport the view port where the actor should be added to (default: all)
        */
      void addActorToRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport = 0);

      /** \brief Internal method. Adds a vtk actor to screen.
        * \param actor a pointer to the vtk actor object
        * \param viewport the view port where the actor should be added to (default: all)
        */
      void removeActorFromRenderer (const vtkSmartPointer<vtkProp> &actor, int viewport = 0);

      /** \brief Internal method. Creates a vtk actor from a vtk polydata object.
        * \param data the vtk polydata object to create an actor for
        * \param actor the resultant vtk actor object
        */
      void createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data, vtkSmartPointer<vtkLODActor> &actor);

      /** \brief Converts a PCL templated PointCloud object to a vtk polydata object.
        * \param cloud the input PCL PointCloud dataset
        * \param polydata the resultant polydata containing the cloud
        */
      void convertPointCloudToVTKPolyData (const pcl::PointCloud<pcl::PointXYZ> &cloud, vtkSmartPointer<vtkPolyData> &polydata);

      /** \brief Converts a PCL templated PointCloud object to a vtk polydata object.
        * \param cloud the input PCL PointCloud dataset
        * \param polydata the resultant polydata containing the cloud
        */
      template <typename PointT> void 
      convertPointCloudToVTKPolyData (const pcl::PointCloud<PointT> &cloud, vtkSmartPointer<vtkPolyData> &polydata);

      /** \brief Converts a PCL templated PointCloud object to a vtk polydata object.
        * \param geometry_handler the geometry handler object used to extract the XYZ data
        * \param polydata the resultant polydata containing the cloud
        */
      template <typename PointT> void 
        convertPointCloudToVTKPolyData (const PointCloudGeometryHandler<PointT> &geometry_handler, vtkSmartPointer<vtkPolyData> &polydata);

      /** \brief Converts a PCL templated PointCloud object to a vtk polydata object.
        * \param geometry_handler the geometry handler object used to extract the XYZ data
        * \param polydata the resultant polydata containing the cloud
        */
      void convertPointCloudToVTKPolyData (const GeometryHandlerConstPtr &geometry_handler, vtkSmartPointer<vtkPolyData> &polydata);
  };
}

#include <pcl/visualization/impl/pcl_visualizer.hpp>

#endif
