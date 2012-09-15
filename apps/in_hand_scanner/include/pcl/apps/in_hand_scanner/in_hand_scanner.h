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

#ifndef PCL_IN_HAND_SCANNER_IN_HAND_SCANNER_H
#define PCL_IN_HAND_SCANNER_IN_HAND_SCANNER_H

#include <string>
#include <sstream>
#include <iomanip>

#include <boost/thread/mutex.hpp>

#include <pcl/common/time.h>
#include <pcl/apps/in_hand_scanner/common_types.h>

////////////////////////////////////////////////////////////////////////////////
// Forward declarations
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  class OpenNIGrabber;

  namespace ihs
  {
    class InputDataProcessing;
    class ICP;
    class Integration;
  } // End namespace ihs

  namespace visualization
  {
    class PCLVisualizer;
    class KeyboardEvent;
  } // End namespace visualization

} // End namespace pcl

////////////////////////////////////////////////////////////////////////////////
// InHandScanner
////////////////////////////////////////////////////////////////////////////////

namespace pcl
{
  namespace ihs
  {

    class InHandScanner
    {

      public:

        /** \brief Mode in which the scanner is currently running */
        typedef enum RunningMode
        {
          RM_UNPROCESSED         = 0, /**< Shows the unprocessed input data */
          RM_PROCESSED           = 1, /**< Shows the processed input data */
          RM_REGISTRATION_CONT   = 2, /**< Registers new data to the first acquired data continuously */
          RM_REGISTRATION_SINGLE = 3  /**< Registers new data once and returns to showing the processed data */
        } RunningMode;

      private:

        typedef pcl::ihs::PointInput         PointInput;
        typedef pcl::ihs::CloudInput         CloudInput;
        typedef pcl::ihs::CloudInputPtr      CloudInputPtr;
        typedef pcl::ihs::CloudInputConstPtr CloudInputConstPtr;

        typedef pcl::ihs::PointProcessed         PointProcessed;
        typedef pcl::ihs::CloudProcessed         CloudProcessed;
        typedef pcl::ihs::CloudProcessedPtr      CloudProcessedPtr;
        typedef pcl::ihs::CloudProcessedConstPtr CloudProcessedConstPtr;

        typedef pcl::ihs::PointModel         PointModel;
        typedef pcl::ihs::CloudModel         CloudModel;
        typedef pcl::ihs::CloudModelPtr      CloudModelPtr;
        typedef pcl::ihs::CloudModelConstPtr CloudModelConstPtr;

        typedef pcl::OpenNIGrabber                Grabber;
        typedef boost::shared_ptr <Grabber>       GrabberPtr;
        typedef boost::shared_ptr <const Grabber> GrabberConstPtr;

        typedef pcl::ihs::InputDataProcessing                 InputDataProcessing;
        typedef boost::shared_ptr <InputDataProcessing>       InputDataProcessingPtr;
        typedef boost::shared_ptr <const InputDataProcessing> InputDataProcessingConstPtr;

        typedef pcl::ihs::ICP                 ICP;
        typedef boost::shared_ptr <ICP>       ICPPtr;
        typedef boost::shared_ptr <const ICP> ICPConstPtr;
        typedef pcl::ihs::Transformation      Transformation;

        typedef pcl::ihs::Integration                 Integration;
        typedef boost::shared_ptr <Integration>       IntegrationPtr;
        typedef boost::shared_ptr <const Integration> IntegrationConstPtr;

        typedef pcl::visualization::PCLVisualizer       PCLVisualizer;
        typedef boost::shared_ptr <PCLVisualizer>       PCLVisualizerPtr;
        typedef boost::shared_ptr <const PCLVisualizer> PCLVisualizerConstPtr;

        class FPS
        {
          public:
            FPS () : fps_ (0.) {}
          protected:
            ~FPS () {}

          public:
            double& value ()       {return (fps_);}
            double  value () const {return (fps_);}

            std::string
            str () const
            {
              std::stringstream ss;
              ss << std::setprecision (1) << std::fixed << fps_;
              return (ss.str ());
            }

          private:
            double fps_;
        };

        class VisualizationFPS : public FPS
        {
          public:
            VisualizationFPS () : FPS () {}
            ~VisualizationFPS () {}
        };
        class ComputationFPS : public FPS
        {
          public:
            ComputationFPS () : FPS () {}
            ~ComputationFPS () {}
        };

      public:

        InHandScanner (int argc, char** argv);
        ~InHandScanner ();

        void
        run ();

        void
        quit ();

        void
        setRunningMode (const RunningMode& mode);

        void
        resetRegistration ();

        void
        resetCamera ();

      private:

        void
        newDataCallback (const CloudInputConstPtr& cloud_in);

        void
        drawClouds ();

        void
        drawMesh ();

        void
        drawCropBox ();

        void
        drawFPS ();

        void
        keyboardCallback (const pcl::visualization::KeyboardEvent& event, void*);

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

      private:

        boost::mutex                mutex_;
        bool                        run_;
        VisualizationFPS            visualization_fps_;
        ComputationFPS              computation_fps_;
        RunningMode                 running_mode_;
        unsigned int                iteration_;

        PCLVisualizerPtr            visualizer_;
        bool                        draw_crop_box_;

        GrabberPtr                  grabber_;
        boost::signals2::connection new_data_connection_;

        InputDataProcessingPtr      input_data_processing_;

        ICPPtr                      icp_;
        Transformation              transformation_;

        IntegrationPtr              integration_;

        CloudProcessedPtr           cloud_data_draw_;
        CloudModelPtr               cloud_model_draw_;
        CloudModelPtr               cloud_model_;
    };

  } // End namespace ihs
} // End namespace pcl

#endif // PCL_IN_HAND_SCANNER_IN_HAND_SCANNER_H
