/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 */

#ifndef PCL_MODELER_ICP_REGISTRATION_WORKER_H_
#define PCL_MODELER_ICP_REGISTRATION_WORKER_H_

#include <pcl/apps/modeler/abstract_worker.h>
#include <pcl/apps/modeler/cloud_mesh.h>

namespace pcl
{
  namespace modeler
  {
    class IntParameter;
    class DoubleParameter;

    class ICPRegistrationWorker : public AbstractWorker 
    {
      public:
        ICPRegistrationWorker(CloudMesh::PointCloudPtr cloud, const QList<CloudMeshItem*>& cloud_mesh_items, QWidget* parent=0);
        ~ICPRegistrationWorker(void);

      protected:
        virtual std::string
        getName () const {return ("Normal Estimation");}

        virtual void
        initParameters(CloudMeshItem* cloud_mesh_item);

        virtual void
        setupParameters();

        virtual void
        processImpl(CloudMeshItem* cloud_mesh_item);

      private:
        CloudMesh::PointCloudPtr    cloud_;

        double x_min_, x_max_;
        double y_min_, y_max_;
        double z_min_, z_max_;

        DoubleParameter*  max_correspondence_distance_;
        IntParameter*     max_iterations_;
        DoubleParameter*  transformation_epsilon_;
        DoubleParameter*  euclidean_fitness_epsilon_;
    };

  }
}

#endif // PCL_MODELER_ICP_REGISTRATION_WORKER_H_
