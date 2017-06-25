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

#ifndef PCL_APPS_IN_HAND_SCANNER_OFFLINE_INTEGRATION_H
#define PCL_APPS_IN_HAND_SCANNER_OFFLINE_INTEGRATION_H

#include <vector>
#include <string>

#include <pcl/pcl_exports.h>
#include <pcl/common/time.h>
#include <pcl/apps/in_hand_scanner/common_types.h>
#include <pcl/apps/in_hand_scanner/boost.h>
#include <pcl/apps/in_hand_scanner/eigen.h>
#include <pcl/apps/in_hand_scanner/opengl_viewer.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  template <class PointInT, class PointOutT>
  class IntegralImageNormalEstimation;

  namespace ihs
  {
    class Integration;
  } // End namespace ihs
} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// OfflineIntegration
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {
    /** \brief Read the clouds and transformations from files and integrate them into one common model.
      * \todo Add Documentation
      */
    class PCL_EXPORTS OfflineIntegration : public pcl::ihs::OpenGLViewer
    {
      Q_OBJECT

      public:

        typedef pcl::ihs::OpenGLViewer       Base;
        typedef pcl::ihs::OfflineIntegration Self;

        /** \brief Constructor. */
        explicit OfflineIntegration (Base* parent=0);

        /** \brief Destructor. */
        ~OfflineIntegration ();

      public Q_SLOTS:

        /** \brief Start the procedure from a path. */
        void
        start ();

      private Q_SLOTS:

        /** \brief Loads in new data. */
        void
        computationThread ();

      private:

        typedef pcl::PointXYZRGBA              PointXYZRGBA;
        typedef pcl::PointCloud <PointXYZRGBA> CloudXYZRGBA;
        typedef CloudXYZRGBA::Ptr              CloudXYZRGBAPtr;
        typedef CloudXYZRGBA::ConstPtr         CloudXYZRGBAConstPtr;

        typedef pcl::PointXYZRGBNormal              PointXYZRGBNormal;
        typedef pcl::PointCloud <PointXYZRGBNormal> CloudXYZRGBNormal;
        typedef CloudXYZRGBNormal::Ptr              CloudXYZRGBNormalPtr;
        typedef CloudXYZRGBNormal::ConstPtr         CloudXYZRGBNormalConstPtr;

        typedef pcl::ihs::Mesh         Mesh;
        typedef pcl::ihs::MeshPtr      MeshPtr;
        typedef pcl::ihs::MeshConstPtr MeshConstPtr;

        typedef pcl::ihs::Integration                 Integration;
        typedef boost::shared_ptr <Integration>       IntegrationPtr;
        typedef boost::shared_ptr <const Integration> IntegrationConstPtr;

        typedef pcl::IntegralImageNormalEstimation <PointXYZRGBA, PointXYZRGBNormal> NormalEstimation;
        typedef boost::shared_ptr <NormalEstimation>                                 NormalEstimationPtr;
        typedef boost::shared_ptr <const NormalEstimation>                           NormalEstimationConstPtr;

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

        /** \brief Get a list of files with from a given directory.
          * \param[in] path_dir Path to search for the files.
          * \param[in] extension File extension (must start with a dot). E.g. '.pcd'.
          * \param[out] files Paths to the files.
          * \return True if success.
          */
        bool
        getFilesFromDirectory (const std::string          path_dir,
                               const std::string          extension,
                               std::vector <std::string>& files) const;

        /** \brief Load the transformation matrix from the given file.
          * \param[in] filename Path to the file.
          * \param[out] transform The loaded transform.
          * \return True if success.
          */
        bool
        loadTransform (const std::string& filename,
                       Eigen::Matrix4f&   transform) const;

        /** \brief Load the cloud and transformation from the files and compute the normals.
          * \param[in] filename Path to the pcd file.
          * \param[out] cloud Cloud with computed normals.
          * \param[out] T Loaded transformation.
          * \return True if success.
          */
        bool
        load (const std::string&    filename,
              CloudXYZRGBNormalPtr& cloud,
              Eigen::Matrix4f&      T) const;

        /** \see http://doc.qt.digia.com/qt/qwidget.html#paintEvent
          * \see http://doc.qt.digia.com/qt/opengl-overpainting.html
          */
        void
        paintEvent (QPaintEvent* event);

        /** \see http://doc.qt.digia.com/qt/qwidget.html#keyPressEvent */
        void
        keyPressEvent (QKeyEvent* event);

        //////////////////////////////////////////////////////////////////////////
        // Members
        //////////////////////////////////////////////////////////////////////////

        /** \brief Synchronization. */
        boost::mutex mutex_;

        /** \brief Wait until the data finished processing. */
        boost::mutex mutex_quit_;

        /** \brief Please have a look at the documentation of ComputationFPS. */
        ComputationFPS computation_fps_;

        /** \brief Please have a look at the documentation of VisualizationFPS. */
        VisualizationFPS visualization_fps_;

        /** \brief Path to the pcd and transformation files. */
        std::string path_dir_;

        /** \brief Model to which new data is registered to (stored as a mesh). */
        MeshPtr mesh_model_;

        /** \brief Compute the normals for the input clouds. */
        NormalEstimationPtr normal_estimation_;

        /** \brief Integrate the data cloud into a common model. */
        IntegrationPtr integration_;

        /** \brief Prevent the application to crash while closing. */
        bool destructor_called_;

      public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  } // End namespace ihs
} // End namespace pcl

#endif // PCL_APPS_IN_HAND_SCANNER_OFFLINE_INTEGRATION_H
