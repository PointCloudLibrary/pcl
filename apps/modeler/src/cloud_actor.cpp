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

#include <pcl/apps/modeler/cloud_actor.h>
#include <pcl/apps/modeler/main_window.h>

#include <QMenu>

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudActor::CloudActor (MainWindow* main_window,
  const sensor_msgs::PointCloud2::Ptr &cloud,
  const std::string &id,
  const Eigen::Vector4f& sensor_origin,
  const Eigen::Quaternion<float>& sensor_orientation) : 
  cloud_(cloud),
  id_(id),
  viewpoint_transformation_(vtkSmartPointer<vtkMatrix4x4>::New()),
  actor_(vtkSmartPointer<vtkLODActor>::New ()),
  main_window_(main_window),
  TreeItem(id.c_str())
{
  color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2>(cloud));
  geometry_handler_.reset(new pcl::visualization::PointCloudGeometryHandlerXYZ<sensor_msgs::PointCloud2>(cloud));

  if (!color_handler_->isCapable())
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRandom<sensor_msgs::PointCloud2>(cloud));

  convertToVtkMatrix (sensor_origin, sensor_orientation, viewpoint_transformation_);
  createActorFromHandlers();
}

//////////////////////////////////////////////////////////////////////////////////////////////
pcl::modeler::CloudActor::~CloudActor ()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::modeler::CloudActor::createActorFromHandlers ()
{
  if (!geometry_handler_->isCapable ())
  {
    PCL_WARN ("[createActorFromHandlers] PointCloud <%s> requested with an invalid geometry handler (%s)!\n", id_.c_str (), geometry_handler_->getName ().c_str ());
    return (false);
  }

  if (!color_handler_->isCapable ())
  {
    PCL_WARN ("[createActorFromHandlers] PointCloud <%s> requested with an invalid color handler (%s)!\n", id_.c_str (), color_handler_->getName ().c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkIdTypeArray> initcells;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler_, polydata, initcells);
  // use the given geometry handler
  polydata->Update ();

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  color_handler_->getColor (scalars);
  polydata->GetPointData ()->SetScalars (scalars);
  double minmax[2];
  scalars->GetRange (minmax);

  // Create an Actor
  createActorFromVTKDataSet (polydata, actor_);
  actor_->GetMapper ()->SetScalarRange (minmax);

  cells_ = reinterpret_cast<vtkPolyDataMapper*>(actor_->GetMapper ())->GetInput ()->GetVerts ()->GetData ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudActor::convertToVtkMatrix (
  const Eigen::Vector4f &origin,
  const Eigen::Quaternion<float> &orientation,
  vtkSmartPointer<vtkMatrix4x4> &vtk_matrix)
{
  // set rotation
  Eigen::Matrix3f rot = orientation.toRotationMatrix ();
  for (int i = 0; i < 3; i++)
    for (int k = 0; k < 3; k++)
      vtk_matrix->SetElement (i, k, rot (i, k));

  // set translation
  vtk_matrix->SetElement (0, 3, origin (0));
  vtk_matrix->SetElement (1, 3, origin (1));
  vtk_matrix->SetElement (2, 3, origin (2));
  vtk_matrix->SetElement (3, 3, 1.0f);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudActor::convertPointCloudToVTKPolyData (
  const GeometryHandlerConstPtr &geometry_handler,
  vtkSmartPointer<vtkPolyData> &polydata,
  vtkSmartPointer<vtkIdTypeArray> &initcells)
{
  vtkSmartPointer<vtkCellArray> vertices;

  if (!polydata)
  {
    polydata = vtkSmartPointer<vtkPolyData>::New ();
    vertices = vtkSmartPointer<vtkCellArray>::New ();
    polydata->SetVerts (vertices);
  }

  // Use the handler to obtain the geometry
  vtkSmartPointer<vtkPoints> points;
  geometry_handler->getGeometry (points);
  polydata->SetPoints (points);

  vtkIdType nr_points = points->GetNumberOfPoints ();

  // Create the supporting structures
  vertices = polydata->GetVerts ();
  if (!vertices)
    vertices = vtkSmartPointer<vtkCellArray>::New ();

  vtkSmartPointer<vtkIdTypeArray> cells = vertices->GetData ();
  updateCells (cells, initcells, nr_points);
  // Set the cells and the vertices
  vertices->SetCells (nr_points, cells);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudActor::updateCells (vtkSmartPointer<vtkIdTypeArray> &cells,
  vtkSmartPointer<vtkIdTypeArray> &initcells,
  vtkIdType nr_points)
{
  // If no init cells and cells has not been initialized...
  if (!cells)
    cells = vtkSmartPointer<vtkIdTypeArray>::New ();

  // If we have less values then we need to recreate the array
  if (cells->GetNumberOfTuples () < nr_points)
  {
    cells = vtkSmartPointer<vtkIdTypeArray>::New ();

    // If init cells is given, and there's enough data in it, use it
    if (initcells && initcells->GetNumberOfTuples () >= nr_points)
    {
      cells->DeepCopy (initcells);
      cells->SetNumberOfComponents (2);
      cells->SetNumberOfTuples (nr_points);
    }
    else
    {
      // If the number of tuples is still too small, we need to recreate the array
      cells->SetNumberOfComponents (2);
      cells->SetNumberOfTuples (nr_points);
      vtkIdType *cell = cells->GetPointer (0);
      // Fill it with 1s
      std::fill_n (cell, nr_points * 2, 1);
      cell++;
      for (vtkIdType i = 0; i < nr_points; ++i, cell += 2)
        *cell = i;
      // Save the results in initcells
      initcells = vtkSmartPointer<vtkIdTypeArray>::New ();
      initcells->DeepCopy (cells);
    }
  }
  else
  {
    // The assumption here is that the current set of cells has more data than needed
    cells->SetNumberOfComponents (2);
    cells->SetNumberOfTuples (nr_points);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudActor::createActorFromVTKDataSet (const vtkSmartPointer<vtkDataSet> &data,
  vtkSmartPointer<vtkLODActor> &actor,
  bool use_scalars)
{
  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (data);

  if (use_scalars)
  {
    vtkSmartPointer<vtkDataArray> scalars = data->GetPointData ()->GetScalars ();
    double minmax[2];
    if (scalars)
    {
      scalars->GetRange (minmax);
      mapper->SetScalarRange (minmax);

      mapper->SetScalarModeToUsePointData ();
      mapper->InterpolateScalarsBeforeMappingOn ();
      mapper->ScalarVisibilityOn ();
    }
  }
  mapper->ImmediateModeRenderingOff ();

  actor->SetNumberOfCloudPoints (int (std::max<vtkIdType> (1, data->GetNumberOfPoints () / 10)));
  actor->GetProperty ()->SetInterpolationToFlat ();

  /// FIXME disabling backface culling due to known VTK bug: vtkTextActors are not
  /// shown when there is a vtkActor with backface culling on present in the scene
  /// Please see VTK bug tracker for more details: http://www.vtk.org/Bug/view.php?id=12588
  // actor->GetProperty ()->BackfaceCullingOn ();

  actor->SetMapper (mapper);
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudActor::showContextMenu(const QPoint& position)
{
  QMenu menu(main_window_);
  main_window_->addActionsToCloudActor(&menu);
  menu.exec(position);
}

/////////////////////////////////////////////////////////////////////////////////////////////
std::vector<std::string>
pcl::modeler::CloudActor::getAvaiableFieldNames() const
{
  const std::vector< ::sensor_msgs::PointField >& fields = cloud_->fields;

  std::vector<std::string> field_names;
  for (size_t i = 0, i_end = fields.size(); i < i_end; ++ i)
  {
    field_names.push_back(fields[i].name);
  }

  return (field_names);
}


/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudActor::setColorHandler(double r, double g, double b)
{
  color_handler_.reset(new pcl::visualization::PointCloudColorHandlerCustom<sensor_msgs::PointCloud2>(cloud_, r, g, b));

  createActorFromHandlers();

  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::modeler::CloudActor::setColorHandler(const std::string& field_name)
{
  if (field_name == "rgb")
  {
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerRGBField<sensor_msgs::PointCloud2>(cloud_));
  }
  else if (field_name == "hsv")
  {
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerHSVField<sensor_msgs::PointCloud2>(cloud_));
  }
  else
  {
    color_handler_.reset(new pcl::visualization::PointCloudColorHandlerGenericField<sensor_msgs::PointCloud2>(cloud_, field_name));
  }

  createActorFromHandlers();
  main_window_->triggerRender(actor_.GetPointer());

  return;
}
