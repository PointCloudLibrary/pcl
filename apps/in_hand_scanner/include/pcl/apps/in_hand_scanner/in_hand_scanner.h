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

#include <pcl/pcl_exports.h>
#include <pcl/apps/in_hand_scanner/boost.h>
#include <pcl/apps/in_hand_scanner/common_types.h>
#include <pcl/apps/in_hand_scanner/opengl_viewer.h>

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
    class MeshProcessing;
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
    class PCL_EXPORTS InHandScanner : public pcl::ihs::OpenGLViewer
    {
      Q_OBJECT

      public:

        typedef pcl::ihs::OpenGLViewer  Base;
        typedef pcl::ihs::InHandScanner Self;

        typedef pcl::ihs::InputDataProcessing                 InputDataProcessing;
        typedef boost::shared_ptr <InputDataProcessing>       InputDataProcessingPtr;
        typedef boost::shared_ptr <const InputDataProcessing> InputDataProcessingConstPtr;

        typedef pcl::ihs::ICP                 ICP;
        typedef boost::shared_ptr <ICP>       ICPPtr;
        typedef boost::shared_ptr <const ICP> ICPConstPtr;

        typedef pcl::ihs::Integration                 Integration;
        typedef boost::shared_ptr <Integration>       IntegrationPtr;
        typedef boost::shared_ptr <const Integration> IntegrationConstPtr;

        typedef pcl::ihs::MeshProcessing                 MeshProcessing;
        typedef boost::shared_ptr <MeshProcessing>       MeshProcessingPtr;
        typedef boost::shared_ptr <const MeshProcessing> MeshProcessingConstPtr;

        /** \brief Switch between different branches of the scanning pipeline. */
        typedef enum RunningMode
        {
          RM_SHOW_MODEL          = 0, /**< Shows the model shape (if it is available). */
          RM_UNPROCESSED         = 1, /**< Shows the unprocessed input data. */
          RM_PROCESSED           = 2, /**< Shows the processed input data. */
          RM_REGISTRATION_CONT   = 3, /**< Registers new data to the first acquired data continuously. */
          RM_REGISTRATION_SINGLE = 4  /**< Registers new data once and returns to showing the processed data. */
        } RunningMode;

        /** \brief File type for saving and loading files. */
        typedef enum FileType
        {
          FT_PLY = 0, /**< Polygon File Format. */
          FT_VTK = 1  /**< VTK File Format. */
        } FileType;

        /** \brief Constructor. */
        explicit InHandScanner (Base* parent=0);

        /** \brief Destructor. */
        ~InHandScanner ();

        /** \brief Get the input data processing. */
        inline InputDataProcessing&
        getInputDataProcessing () {return (*input_data_processing_);}

        /** \brief Get the registration. */
        inline ICP&
        getICP () {return (*icp_);}

        /** \brief Get the integration. */
        inline Integration&
        getIntegration () {return (*integration_);}

      signals:

        /** \brief Emitted when the running mode changes. */
        void
        runningModeChanged (RunningMode new_running_mode) const;

      public slots:

        /** \brief Start the grabber (enables the scanning pipeline). */
        void
        startGrabber ();

        /** \brief Shows the unprocessed input data. */
        void
        showUnprocessedData ();

        /** \brief Shows the processed input data. */
        void
        showProcessedData ();

        /** \brief Registers new data to the first acquired data continuously. */
        void
        registerContinuously ();

        /** \brief Registers new data once and returns to showing the processed data. */
        void
        registerOnce ();

        /** \brief Show the model shape (if one is available). */
        void
        showModel ();

        /** \brief Removes unfit vertices regardless of their age. Unfit vertices are those that have not been observed from enough directions. */
        void
        removeUnfitVertices ();

        /** \brief Reset the scanning pipeline. */
        void
        reset ();

        /** \brief Saves the model mesh in a file with the given filename and filetype.
          * \note The extension of the filename is ignored!
          */
        void
        saveAs (const std::string& filename, const FileType& filetype);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#keyPressEvent */
        void
        keyPressEvent (QKeyEvent* event);

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

        typedef pcl::OpenNIGrabber                Grabber;
        typedef boost::shared_ptr <Grabber>       GrabberPtr;
        typedef boost::shared_ptr <const Grabber> GrabberConstPtr;

        /** \brief Helper object for the computation thread. Please have a look at the documentation of calcFPS. */
        class ComputationFPS : public Base::FPS
        {
          public:
            ComputationFPS () : Base::FPS () {}
            ~ComputationFPS () {}
        };

        /** \brief Helper object for the visualization thread. Please have a look at the documentation of calcFPS. */
        class VisualizationFPS : public Base::FPS
        {
          public:
            VisualizationFPS () : Base::FPS () {}
            ~VisualizationFPS () {}
        };

        /** \brief Called when new data arries from the grabber. The grabbing - registration - integration pipeline is implemented here. */
        void
        newDataCallback (const CloudXYZRGBAConstPtr& cloud_in);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#paintEvent
          * \see http://doc.qt.digia.com/qt/opengl-overpainting.html
          */
        void
        paintEvent (QPaintEvent* event);

        /** \brief Draw text over the opengl scene.
          * \see http://doc.qt.digia.com/qt/opengl-overpainting.html
          */
        void
        drawText ();

        /** \brief Actual implementeation of startGrabber (needed so it can be run in a different thread and doesn't block the application when starting up). */
        void
        startGrabberImpl ();

        ////////////////////////////////////////////////////////////////////////
        // Members
        ////////////////////////////////////////////////////////////////////////

        /** \brief Synchronization. */
        boost::mutex mutex_;

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

        /** \brief This variable is true if the grabber is starting. */
        bool starting_grabber_;

        /** \brief Connection of the grabber signal with the data processing thread. */
        boost::signals2::connection new_data_connection_;

        /** \brief Processes the data from the sensor. Output is input to the registration. */
        InputDataProcessingPtr input_data_processing_;

        /** \brief Registration (Iterative Closest Point). */
        ICPPtr icp_;

        /** \brief Transformation that brings the data cloud into model coordinates. */
        Eigen::Matrix4f transformation_;

        /** \brief Integrate the data cloud into a common model. */
        IntegrationPtr integration_;

        /** \brief Methods called after the integration. */
        MeshProcessingPtr mesh_processing_;

        /** \brief Model to which new data is registered to (stored as a mesh). */
        MeshPtr mesh_model_;

        /** \brief Prevent the application to crash while closing. */
        bool destructor_called_;

      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // End namespace ihs
} // End namespace pcl

// http://doc.qt.digia.com/qt/qmetatype.html#Q_DECLARE_METATYPE
Q_DECLARE_METATYPE (pcl::ihs::InHandScanner::RunningMode)

#endif // PCL_APPS_IN_HAND_SCANNER_IN_HAND_SCANNER_H
