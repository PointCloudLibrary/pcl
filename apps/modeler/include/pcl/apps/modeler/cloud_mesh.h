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
 *
 */
#ifndef PCL_MODELER_CLOUD_MESH_H_
#define PCL_MODELER_CLOUD_MESH_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/Vertices.h>
#include <vtkSmartPointer.h>

class vtkPoints;
class vtkCellArray;
class vtkDataArray;

namespace pcl
{
  namespace modeler
  {
    class CloudMesh
    {
    public:
      typedef pcl::PointSurfel                    PointT;
      typedef pcl::PointCloud<PointT>             PointCloud;
      typedef PointCloud::Ptr                     PointCloudPtr;
      typedef PointCloud::ConstPtr                PointCloudConstPtr;

      CloudMesh ();
      CloudMesh (PointCloudPtr cloud);
      ~CloudMesh ();

      PointCloudPtr& 
      getCloud() {return cloud_;}
      PointCloudConstPtr
      getCloud() const {return cloud_;}

      std::vector<pcl::Vertices>&
      getPolygons() {return polygons_;}
      const std::vector<pcl::Vertices>&
      getPolygons() const {return polygons_;}

      vtkSmartPointer<vtkPoints>&
      getVtkPoints() {return vtk_points_;}
      const vtkSmartPointer<vtkPoints>&
      getVtkPoints() const {return vtk_points_;}

      vtkSmartPointer<vtkCellArray>&
      getVtkPolygons() {return vtk_polygons_;}
      const vtkSmartPointer<vtkCellArray>&
      getVtkPolygons() const {return vtk_polygons_;}

      std::vector<std::string>
      getAvaiableFieldNames() const;

      bool
      open(const std::string& filename);

      bool
      save(const std::string& filename) const;

      static bool
      save(const std::vector<const CloudMesh*>& cloud_meshes, const std::string& filename);

      void
      getColorScalarsFromField(vtkSmartPointer<vtkDataArray> &scalars, const std::string& field) const;

      void
      updateVtkPoints();

      void
      updateVtkPolygons();

      void
      transform(double tx, double ty, double tz, double rx, double ry, double rz);

    protected:


    private:
      PointCloudPtr               cloud_;
      std::vector<pcl::Vertices>  polygons_;

      vtkSmartPointer<vtkPoints>            vtk_points_;
      vtkSmartPointer<vtkCellArray>         vtk_polygons_;
    };
  }
}

#endif // PCL_MODELER_CLOUD_MESH_H_