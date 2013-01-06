/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2009-2012, Willow Garage, Inc.
 * Copyright (c) 2012-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of the copyright holder(s) nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef PCL_APPS_IN_HAND_SCANNER_OPENGL_VIEWER_H
#define PCL_APPS_IN_HAND_SCANNER_OPENGL_VIEWER_H

#include <string>

#include <QGLWidget>

#include <pcl/pcl_exports.h>
#include <pcl/apps/in_hand_scanner/boost.h>
#include <pcl/apps/in_hand_scanner/common_types.h>
#include <pcl/apps/in_hand_scanner/eigen.h>

namespace pcl
{
  namespace ihs
  {
    /** \brief Viewer for the in-hand scanner based on Qt and OpenGL. */
    class PCL_EXPORTS OpenGLViewer : public QGLWidget
    {
      Q_OBJECT

      public:

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        typedef pcl::ihs::Mesh         Mesh;
        typedef pcl::ihs::MeshPtr      MeshPtr;
        typedef pcl::ihs::MeshConstPtr MeshConstPtr;

        /** \brief How to draw the mesh. */
        typedef enum DisplayMode
        {
          DM_POINTS, /**< Draw the points. */
          DM_FACES   /**< Draw the faces of the mesh without edges. */
        } DisplayMode;

        /** \brief Coefficients for the wireframe cube. */
        class CubeCoefficients
        {
          public:
            CubeCoefficients ()
              : x_min (0), x_max (0),
                y_min (0), y_max (0),
                z_min (0), z_max (0),
                transformation (Eigen::Isometry3d::Identity ())
            {
            }

            CubeCoefficients (const float x_min, const float x_max,
                              const float y_min, const float y_max,
                              const float z_min, const float z_max,
                              const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity ())
              : x_min (x_min), x_max (x_max),
                y_min (y_min), y_max (y_max),
                z_min (z_min), z_max (z_max),
                transformation (T)
            {
            }

            float x_min; float x_max;
            float y_min; float y_max;
            float z_min; float z_max;
            Eigen::Isometry3d transformation;

          public:

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        /** \brief Constructor. */
        explicit OpenGLViewer (QWidget* parent=0);

        /** \brief Destructor. */
        ~OpenGLViewer ();

        /** \brief Add a mesh to be drawn.
          * \param[in] mesh The input mesh.
          * \param[in] id Unique identifier for the mesh. The internal mesh is replaced by the input mesh if the id already exists.
          * \param[in] T Transformation applied to the mesh. Defaults to an identity transformation.
          * \return true if success.
          * \note Converts the mesh to the internal representation better suited for visualization. Therefore this method takes some time.
          */
        bool
        addMesh (const MeshConstPtr& mesh, const std::string& id, const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity ());

        /** \brief Convert an organized cloud to a mesh and draw it.
          * \param[in] cloud Organized input cloud.
          * \param[in] id Unique identifier for the mesh. The internal mesh is replaced by the converted input mesh if the id already exists.
          ** \param[in] T Transformation applied to the mesh. Defaults to an identity transformation.
          * \return true if success.
          * \note This method takes some time for the conversion).
          */
        bool
        addMesh (const CloudXYZRGBNormalConstPtr& cloud, const std::string& id, const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity ());

        /** \brief Remove the mesh with the given id.
          * \param[in] id Identifier of the mesh (results in a failure if the id does not exist).
          * \return true if success.
          */
        bool
        removeMesh (const std::string& id);

        /** \brief Remove all meshes that are currently drawn. */
        void
        removeAllMeshes ();

        /** \brief Set the cube coefficients. */
        void
        setCubeCoefficients (const CubeCoefficients& coeffs);

        /** \brief Enable drawing the cube. */
        void
        enableDrawCube ();

        /** \brief Disable drawing the cube. */
        void
        disableDrawCube ();

        /** \brief Set the point around which the camera rotates during mouse navigation. */
        void
        setPivot (const Eigen::Vector3d& pivot);

        /** \brief Searches the given id in the drawn meshes and calculates the pivot as the centroid of the found geometry.
          * \note Returns immediately and computes the pivot in another thread.
          */
        void
        setPivot (const std::string& id);

        /** \brief Stop the visualization timer. */
        void
        stopTimer ();

        /** \brief Toggle the display mode. */
        void
        toggleDisplayMode ();

        /** \brief Reset the virtual camera position and orientation. */
        void
        resetCamera ();

     public slots:

        /** \brief Requests the scene to be re-drawn (called periodically from a timer). */
        void
        timerCallback ();

      protected:

        /** \see http://doc.qt.digia.com/qt/qwidget.html#paintEvent
          * \see http://doc.qt.digia.com/qt/opengl-overpainting.html
          */
        virtual void
        paintEvent (QPaintEvent* event);

      private:

        typedef boost::unordered::unordered_map <std::string, CloudXYZRGBNormalPtr> CloudXYZRGBNormalMap;

        typedef pcl::ihs::PointIHS         PointIHS;
        typedef pcl::ihs::CloudIHS         CloudIHS;
        typedef pcl::ihs::CloudIHSPtr      CloudIHSPtr;
        typedef pcl::ihs::CloudIHSConstPtr CloudIHSConstPtr;

        /** \brief Mesh format more efficient for visualization than the half-edge data structure.
          * \see http://en.wikipedia.org/wiki/Polygon_mesh#Face-vertex_meshes
          * \note Only triangles are currently supported.
          */
        class FaceVertexMesh
        {
          public:

            class Triangle
            {
              public:

                Triangle () : first (0), second (0), third (0) {}
                Triangle (const unsigned int first, const unsigned int second, const unsigned int third)
                  : first (first), second (second), third (third)
                {
                }

                unsigned int first;
                unsigned int second;
                unsigned int third;
            };

            /** \brief Constructor */
            FaceVertexMesh ();

            /** \brief Constructor. Converts the input mesh into a face vertex mesh. */
            FaceVertexMesh (const Mesh& mesh, const Eigen::Isometry3d& T);

            pcl::ihs::OpenGLViewer::CloudIHS vertices;
            std::vector <Triangle>           triangles;
            Eigen::Isometry3d                transformation;

          public:

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };

        typedef boost::shared_ptr <      FaceVertexMesh>                         FaceVertexMeshPtr;
        typedef boost::shared_ptr <const FaceVertexMesh>                         FaceVertexMeshConstPtr;
        typedef boost::unordered::unordered_map <std::string, FaceVertexMeshPtr> FaceVertexMeshMap;

        /** \brief Calculate the pivot for the stored id. */
        void
        calcPivot ();

        /** \brief Draw all meshes.
          * \note Only triangle meshes are currently supported.
          */
        void
        drawMeshes ();

        /** \brief Draw a wireframe cube. */
        void
        drawCube ();

        /** \see http://doc.qt.digia.com/qt/qglwidget.html#initializeGL */
        void
        initializeGL ();

        /** \see http://www.opengl.org/sdk/docs/man/xhtml/glViewport.xml */
        void
        setupViewport (const int w, const int h);

        /** \see http://doc.qt.digia.com/qt/qglwidget.html#resizeGL */
        void
        resizeGL (int w, int h);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#mousePressEvent */
        void
        mousePressEvent (QMouseEvent* event);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#mouseMoveEvent */
        void
        mouseMoveEvent (QMouseEvent* event);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#wheelEvent */
        void
        wheelEvent (QWheelEvent* event);

        ////////////////////////////////////////////////////////////////////////
        // Members
        ////////////////////////////////////////////////////////////////////////

        /** \brief Synchronization. */
        boost::mutex mutex_vis_;

        /** \brief Visualization timer. */
        boost::shared_ptr <QTimer> timer_;

        /** \brief Meshes stored for visualization. */
        FaceVertexMeshMap drawn_meshes_;

        /** \brief How to draw the mesh. */
        DisplayMode display_mode_;

        /** \brief A cube is drawn if this value is true. */
        bool draw_cube_;

        /** \brief Coefficients of the drawn cube. */
        CubeCoefficients cube_coefficients_;

        /** \brief Rotation of the camera. */
        Eigen::Quaterniond R_cam_;

        /** \brief Translation of the camera. */
        Eigen::Vector3d t_cam_;

        /** \brief Center of rotation during mouse navigation. */
        Eigen::Vector3d cam_pivot_;

        /** \brief Compute the pivot for the mesh with the given id. */
        std::string cam_pivot_id_;

        /** \brief Set to true right after the mouse got pressed and false if the mouse got moved. */
        bool mouse_pressed_begin_;

        /** \brief Mouse x-position of the previous mouse move event. */
        int x_prev_;

        /** \brief Mouse y-position of the previous mouse move event. */
        int y_prev_;

      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_OPENGL_VIEWER_H
