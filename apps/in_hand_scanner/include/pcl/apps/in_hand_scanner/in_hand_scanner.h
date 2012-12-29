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

#ifndef PCL_APPS_IN_HAND_SCANNER_IN_HAND_SCANNER_H
#define PCL_APPS_IN_HAND_SCANNER_IN_HAND_SCANNER_H

#include <string>
#include <sstream>
#include <iomanip>

#include <QGLWidget>
#include <QQuaternion>
#include <QVector3D>

#include <pcl/pcl_exports.h>
#include <pcl/common/time.h>
#include <pcl/apps/in_hand_scanner/boost.h>
#include <pcl/apps/in_hand_scanner/common_types.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  class OpenNIGrabber;

  namespace ihs
  {
    class ICP;
    class InputDataProcessing;
    class Integration;
  } // End namespace ihs
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InHandScanner
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    /** \brief
      * \todo Add Documentation
      */
    class PCL_EXPORTS InHandScanner : public QGLWidget
    {
      Q_OBJECT

      public:

        /** \brief Switch between different branches of the scanning pipeline. */
        typedef enum RunningMode
        {
          RM_SHOW_MODEL          = 0, /**< Show the model shape (if one is available). */
          RM_UNPROCESSED         = 1, /**< Shows the unprocessed input data. */
          RM_PROCESSED           = 2, /**< Shows the processed input data. */
          RM_REGISTRATION_CONT   = 3, /**< Registers new data to the first acquired data continuously. */
          RM_REGISTRATION_SINGLE = 4  /**< Registers new data once and returns to showing the processed data. */
        } RunningMode;

        /** \brief How to draw the object. */
        typedef enum DisplayMode
        {
          DM_POINTS = 0, /**< Draw the points. */
          DM_EDGES  = 1, /**< Draw the edges of the mesh. */
          DM_FACES  = 2  /**< Draw the faces of the mesh without edges. */
        } DisplayMode;

        /** \brief Constructor. */
        explicit InHandScanner (QWidget* parent=0);

        /** \brief Destructor. */
        ~InHandScanner ();

        /** \see http://doc.qt.digia.com/qt/qwidget.html#minimumSizeHint-prop */
        virtual QSize
        minimumSizeHint () const;

        /** \see http://doc.qt.digia.com/qt/qwidget.html#sizeHint-prop */
        virtual QSize
        sizeHint () const;

     public slots:

        /** \brief Start the grabber (enables the scanning pipeline). */
        void
        startGrabber ();

        /** \brief Requests the scene to be re-drawn (called preiodically from a timer). */
        void
        timerCallback ();

        /** \brief Set which branches of the scanning pipeline is executed. */
        void
        setRunningMode (const RunningMode& mode);

        /** \brief Specify how the object is drawn. */
        void
        setDisplayMode (const DisplayMode& mode);

        /** \brief Reset the scanning pipeline. */
        void
        reset ();

        /** \brief Reset the virtual camera position and orientation to that of the sensor. */
        void
        resetCamera ();

      private:

        typedef pcl::PointXYZRGBA              PointXYZRGBA;
        typedef pcl::PointCloud <PointXYZRGBA> CloudXYZRGBA;
        typedef CloudXYZRGBA::Ptr              CloudXYZRGBAPtr;
        typedef CloudXYZRGBA::ConstPtr         CloudXYZRGBAConstPtr;

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        typedef pcl::ihs::PointIHS         PointIHS;
        typedef pcl::ihs::CloudIHS         CloudIHS;
        typedef pcl::ihs::CloudIHSPtr      CloudIHSPtr;
        typedef pcl::ihs::CloudIHSConstPtr CloudIHSConstPtr;

        typedef pcl::ihs::Mesh         Mesh;
        typedef pcl::ihs::MeshPtr      MeshPtr;
        typedef pcl::ihs::MeshConstPtr MeshConstPtr;
        typedef std::vector <MeshPtr>  MeshPtrVec;

        typedef pcl::OpenNIGrabber                Grabber;
        typedef boost::shared_ptr <Grabber>       GrabberPtr;
        typedef boost::shared_ptr <const Grabber> GrabberConstPtr;

        typedef pcl::ihs::InputDataProcessing                 InputDataProcessing;
        typedef boost::shared_ptr <InputDataProcessing>       InputDataProcessingPtr;
        typedef boost::shared_ptr <const InputDataProcessing> InputDataProcessingConstPtr;

        typedef pcl::ihs::ICP                 ICP;
        typedef boost::shared_ptr <ICP>       ICPPtr;
        typedef boost::shared_ptr <const ICP> ICPConstPtr;

        typedef pcl::ihs::Integration                 Integration;
        typedef boost::shared_ptr <Integration>       IntegrationPtr;
        typedef boost::shared_ptr <const Integration> IntegrationConstPtr;

        /** \brief Please have a look at the documentation of calcFPS. */
        class FPS
        {
          public:

            FPS () : fps_ (0.) {}

            inline double& value ()       {return (fps_);}
            inline double  value () const {return (fps_);}

            inline std::string
            str () const
            {
              std::stringstream ss;
              ss << std::setprecision (1) << std::fixed << fps_;
              return (ss.str ());
            }

          protected:

            ~FPS () {}

          private:

            double fps_;
        };

        /** \brief Helper object for the computation thread. Please have a look at the documentation of calcFPS. */
        class ComputationFPS : public FPS
        {
          public:
            ComputationFPS () : FPS () {}
            ~ComputationFPS () {}
        };

        /** \brief Helper object for the visualization thread. Please have a look at the documentation of calcFPS. */
        class VisualizationFPS : public FPS
        {
          public:
            VisualizationFPS () : FPS () {}
            ~VisualizationFPS () {}
        };

        /** Measures the performance of the current thread (selected by passing the corresponding 'fps' helper object). The resulting value is stored in the fps object. */
        template <class FPS> void
        calcFPS (FPS& fps) const
        {
          static pcl::StopWatch sw;
          static unsigned int count = 0;

          ++count;
          if (sw.getTimeSeconds () >= 1.)
          {
            fps.value () = static_cast <double> (count) / sw.getTimeSeconds ();
            count = 0;
            sw.reset ();
          }
        }

        /** \brief Actual implementeation of startGrabber (needed so it can be run in a different thread and doesn't block the application when starting up). */
        void
        startGrabberImpl ();

        /** \brief Called when new data arries from the grabber. The grabbing - registration - integration pipeline is implemented here. */
        void
        newDataCallback (const CloudXYZRGBAConstPtr& cloud_in);

        /** \see http://doc.qt.digia.com/qt/qglwidget.html#initializeGL */
        void
        initializeGL ();

        /** \brief Set the lighting of the scene. */
        void
        setupLighting ();

        /** \see http://www.opengl.org/sdk/docs/man/xhtml/glViewport.xml */
        void
        setupViewport (const int w, const int h);

        /** \see http://doc.qt.digia.com/qt/qglwidget.html#resizeGL */
        void
        resizeGL (int w, int h);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#paintEvent
          * \see http://doc.qt.digia.com/qt/opengl-overpainting.html
          */
//        void
//        paintEvent (QPaintEvent* event);
        void
        paintGL ();

        /** \brief Draw cloud if it is available. */
        void
        drawCloud ();

        /** \brief Draw a wireframe box that shows where the data is cropped during input data processing. */
        void
        drawCropBox ();

        /** \brief Draw the current framerate. */
        void
        drawFPS ();

        /** \see http://doc.qt.digia.com/qt/qwidget.html#mousePressEvent */
        void
        mousePressEvent (QMouseEvent* event);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#mouseMoveEvent */
        void
        mouseMoveEvent (QMouseEvent* event);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#wheelEvent */
        void
        wheelEvent (QWheelEvent* event);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#keyPressEvent */
        void
        keyPressEvent (QKeyEvent* event);

        ////////////////////////////////////////////////////////////////////////
        // Members
        ////////////////////////////////////////////////////////////////////////

        /** \brief Synchronization between the computation and visualization thread. */
        boost::mutex mutex_comp_vis_;

        /** \brief Synchronization between the computation thread and user events. */
        boost::mutex mutex_comp_events_;

        /** \brief Synchronization between the visualization thread and user events. */
        boost::mutex mutex_vis_events_;

        /** \brief Please have a look at the documentation of ComputationFPS. */
        ComputationFPS computation_fps_;

        /** \brief Please have a look at the documentation of VisualizationFPS. */
        VisualizationFPS visualization_fps_;

        /** \brief Switch between different branches of the scanning pipeline. */
        RunningMode running_mode_;

        /** \brief The iteration of the scanning pipeline (grab - register - integrate). */
        unsigned int iteration_;

        /** \brief Used to get new data from the sensor. */
        GrabberPtr grabber_;

        /** \brief Connection of the grabber signal with the data processing thread. */
        boost::signals2::connection new_data_connection_;

        /** \brief Used to pass the cloud from the computation to the visualization thread. */
        CloudXYZRGBNormalPtr cloud_draw_tmp_;

        /** \brief Cloud stored for visualization. */
        // TODO: Use Vertex Buffer Objects and upload everything to the GPU.
        CloudXYZRGBNormal cloud_draw_;

        /** \brief Processes the data from the sensor. Output is input to the registration. */
        InputDataProcessingPtr input_data_processing_;

        /** \brief Draw a wireframe box that shows where the data is cropped during input data processing. */
        bool draw_crop_box_;

        /** \brief Registration (Iterative Closest Point). */
//        ICPPtr icp_;

        /** \brief Transformation that brings the data cloud into model coordinates. */
//        Eigen::Matrix4f transformation_;

        /** \brief Integrate the data cloud into a common model (model cloud). */
//        IntegrationPtr integration_;

        /** \brief Model to which new data is registered to (stored as a mesh). */
//        MeshPtr mesh_model_;

        /** \brief Model mesh stored for visualization. */
        // TODO: Use Vertex Buffer Objects and upload everything to the GPU.
//        MeshPtr mesh_model_draw_;

        /** \brief How to draw the object. */
        DisplayMode display_mode_;

        /** \brief Rotation of the camera. */
        QQuaternion cam_R_;

        /** \brief Translation of the camera. */
        QVector3D cam_t_;

        /** \brief Center of rotation during mouse navigation. */
        QVector3D cam_pivot_;

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

#endif // PCL_APPS_IN_HAND_SCANNER_IN_HAND_SCANNER_H
