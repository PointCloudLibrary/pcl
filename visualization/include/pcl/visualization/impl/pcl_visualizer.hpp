/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012, Open Perception, Inc.
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

#ifndef PCL_PCL_VISUALIZER_IMPL_H_
#define PCL_PCL_VISUALIZER_IMPL_H_

#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkLeaderActor2D.h>
#include <vtkVectorText.h>
#include <vtkAlgorithmOutput.h>
#include <vtkFollower.h>
#include <vtkSphereSource.h>
#include <vtkProperty2D.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkMapper.h>
#include <vtkCellData.h>
#include <vtkDataSetMapper.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkAppendPolyData.h>
#include <vtkTextProperty.h>
#include <vtkLODActor.h>

#include <pcl/visualization/common/shapes.h>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPointCloud (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const std::string &id, int viewport)
{
  // Convert the PointCloud to VTK PolyData
  PointCloudGeometryHandlerXYZ<PointT> geometry_handler (cloud);
  return (addPointCloud<PointT> (cloud, geometry_handler, id, viewport));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPointCloud (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const PointCloudGeometryHandler<PointT> &geometry_handler,
  const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it != cloud_actor_map_->end ())
  {
    PCL_WARN ("[addPointCloud] A PointCloud with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  //PointCloudColorHandlerRandom<PointT> color_handler (cloud);
  PointCloudColorHandlerCustom<PointT> color_handler (cloud, 255, 255, 255);
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, cloud->sensor_origin_, cloud->sensor_orientation_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPointCloud (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const GeometryHandlerConstPtr &geometry_handler,
  const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it != cloud_actor_map_->end ())
  {
    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    am_it->second.geometry_handlers.push_back (geometry_handler);
    return (true);
  }

  //PointCloudColorHandlerRandom<PointT> color_handler (cloud);
  PointCloudColorHandlerCustom<PointT> color_handler (cloud, 255, 255, 255);
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, cloud->sensor_origin_, cloud->sensor_orientation_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPointCloud (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const PointCloudColorHandler<PointT> &color_handler,
  const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it != cloud_actor_map_->end ())
  {
    PCL_WARN ("[addPointCloud] A PointCloud with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());

    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    //cloud_actor_map_[id].color_handlers.push_back (color_handler);
    //style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));
    return (false);
  }
  // Convert the PointCloud to VTK PolyData
  PointCloudGeometryHandlerXYZ<PointT> geometry_handler (cloud);
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, cloud->sensor_origin_, cloud->sensor_orientation_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPointCloud (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const ColorHandlerConstPtr &color_handler,
  const std::string &id, int viewport)
{
  // Check to see if this entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
  {
    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    am_it->second.color_handlers.push_back (color_handler);
    return (true);
  }

  PointCloudGeometryHandlerXYZ<PointT> geometry_handler (cloud);
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, cloud->sensor_origin_, cloud->sensor_orientation_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPointCloud (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const GeometryHandlerConstPtr &geometry_handler,
  const ColorHandlerConstPtr &color_handler,
  const std::string &id, int viewport)
{
  // Check to see if this entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
  {
    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    am_it->second.geometry_handlers.push_back (geometry_handler);
    am_it->second.color_handlers.push_back (color_handler);
    return (true);
  }
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, cloud->sensor_origin_, cloud->sensor_orientation_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPointCloud (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const PointCloudColorHandler<PointT> &color_handler,
  const PointCloudGeometryHandler<PointT> &geometry_handler,
  const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it != cloud_actor_map_->end ())
  {
    PCL_WARN ("[addPointCloud] A PointCloud with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    // Here we're just pushing the handlers onto the queue. If needed, something fancier could
    // be done such as checking if a specific handler already exists, etc.
    //cloud_actor_map_[id].geometry_handlers.push_back (geometry_handler);
    //cloud_actor_map_[id].color_handlers.push_back (color_handler);
    //style_->setCloudActorMap (boost::make_shared<CloudActorMap> (cloud_actor_map_));
    return (false);
  }
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport, cloud->sensor_origin_, cloud->sensor_orientation_));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::visualization::PCLVisualizer::convertPointCloudToVTKPolyData (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  vtkSmartPointer<vtkPolyData> &polydata,
  vtkSmartPointer<vtkIdTypeArray> &initcells)
{
  vtkSmartPointer<vtkCellArray> vertices;
  if (!polydata)
  {
    allocVtkPolyData (polydata);
    vertices = vtkSmartPointer<vtkCellArray>::New ();
    polydata->SetVerts (vertices);
  }

  // Create the supporting structures
  vertices = polydata->GetVerts ();
  if (!vertices)
    vertices = vtkSmartPointer<vtkCellArray>::New ();

  vtkIdType nr_points = cloud->points.size ();
  // Create the point set
  vtkSmartPointer<vtkPoints> points = polydata->GetPoints ();
  if (!points)
  {
    points = vtkSmartPointer<vtkPoints>::New ();
    points->SetDataTypeToFloat ();
    polydata->SetPoints (points);
  }
  points->SetNumberOfPoints (nr_points);

  // Get a pointer to the beginning of the data array
  float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

  // Set the points
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
      memcpy (&data[i * 3], &cloud->points[i].x, 12);    // sizeof (float) * 3
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) ||
          !pcl_isfinite (cloud->points[i].y) ||
          !pcl_isfinite (cloud->points[i].z))
        continue;

      memcpy (&data[j * 3], &cloud->points[i].x, 12);    // sizeof (float) * 3
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
  }

  vtkSmartPointer<vtkIdTypeArray> cells = vertices->GetData ();
  updateCells (cells, initcells, nr_points);

  // Set the cells and the vertices
  vertices->SetCells (nr_points, cells);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::visualization::PCLVisualizer::convertPointCloudToVTKPolyData (
  const pcl::visualization::PointCloudGeometryHandler<PointT> &geometry_handler,
  vtkSmartPointer<vtkPolyData> &polydata,
  vtkSmartPointer<vtkIdTypeArray> &initcells)
{
  vtkSmartPointer<vtkCellArray> vertices;
  if (!polydata)
  {
    allocVtkPolyData (polydata);
    vertices = vtkSmartPointer<vtkCellArray>::New ();
    polydata->SetVerts (vertices);
  }

  // Use the handler to obtain the geometry
  vtkSmartPointer<vtkPoints> points;
  geometry_handler.getGeometry (points);
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

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPolygon (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  double r, double g, double b, const std::string &id, int viewport)
{
  vtkSmartPointer<vtkDataSet> data = createPolygon<PointT> (cloud);
  if (!data)
    return (false);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    vtkSmartPointer<vtkAppendPolyData> all_data = vtkSmartPointer<vtkAppendPolyData>::New ();
    
    // Add old data
    all_data->AddInput (reinterpret_cast<vtkPolyDataMapper*> ((vtkActor::SafeDownCast (am_it->second))->GetMapper ())->GetInput ());
   
    // Add new data
    vtkSmartPointer<vtkDataSetSurfaceFilter> surface_filter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New ();
    surface_filter->SetInput (vtkUnstructuredGrid::SafeDownCast (data));
    vtkSmartPointer<vtkPolyData> poly_data = surface_filter->GetOutput ();
    all_data->AddInput (poly_data);

    // Create an Actor
    vtkSmartPointer<vtkActor> actor;
    createActorFromVTKDataSet (all_data->GetOutput (), actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetColor (r, g, b);
    actor->GetMapper ()->ScalarVisibilityOff ();
    removeActorFromRenderer (am_it->second, viewport);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
  }
  else
  {
    // Create an Actor
    vtkSmartPointer<vtkActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetColor (r, g, b);
    actor->GetMapper ()->ScalarVisibilityOff ();
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
  }

  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPolygon (
  const pcl::PlanarPolygon<PointT> &polygon,
  double r, double g, double b, const std::string &id, int viewport)
{
  vtkSmartPointer<vtkDataSet> data = createPolygon<PointT> (polygon);
  if (!data)
    return (false);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    vtkSmartPointer<vtkAppendPolyData> all_data = vtkSmartPointer<vtkAppendPolyData>::New ();

    // Add old data
    all_data->AddInput (reinterpret_cast<vtkPolyDataMapper*> ((vtkActor::SafeDownCast (am_it->second))->GetMapper ())->GetInput ());

    // Add new data
    vtkSmartPointer<vtkDataSetSurfaceFilter> surface_filter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New ();
    surface_filter->SetInput (vtkUnstructuredGrid::SafeDownCast (data));
    vtkSmartPointer<vtkPolyData> poly_data = surface_filter->GetOutput ();
    all_data->AddInput (poly_data);

    // Create an Actor
    vtkSmartPointer<vtkActor> actor;
    createActorFromVTKDataSet (all_data->GetOutput (), actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetColor (r, g, b);
    actor->GetMapper ()->ScalarVisibilityOn ();
    actor->GetProperty ()->BackfaceCullingOff ();
    removeActorFromRenderer (am_it->second, viewport);
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
  }
  else
  {
    // Create an Actor
    vtkSmartPointer<vtkActor> actor;
    createActorFromVTKDataSet (data, actor);
    actor->GetProperty ()->SetRepresentationToWireframe ();
    actor->GetProperty ()->SetColor (r, g, b);
    actor->GetMapper ()->ScalarVisibilityOn ();
    actor->GetProperty ()->BackfaceCullingOff ();
    addActorToRenderer (actor, viewport);

    // Save the pointer/ID pair to the global actor map
    (*shape_actor_map_)[id] = actor;
  }
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPolygon (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const std::string &id, int viewport)
{
  return (!addPolygon<PointT> (cloud, 0.5, 0.5, 0.5, id, viewport));
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addLine] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createLine (pt1.getVector4fMap (), pt2.getVector4fMap ());

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  actor->GetProperty ()->SetColor (r, g, b);
  actor->GetMapper ()->ScalarVisibilityOff ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addArrow] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkLeaderActor2D> leader = vtkSmartPointer<vtkLeaderActor2D>::New ();
  leader->GetPositionCoordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPositionCoordinate ()->SetValue (pt1.x, pt1.y, pt1.z);
  leader->GetPosition2Coordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPosition2Coordinate ()->SetValue (pt2.x, pt2.y, pt2.z);
  leader->SetArrowStyleToFilled ();
  leader->AutoLabelOn ();

  leader->GetProperty ()->SetColor (r, g, b);
  addActorToRenderer (leader, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = leader;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addArrow (const P1 &pt1, const P2 &pt2, double r, double g, double b, bool display_length, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addArrow] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkLeaderActor2D> leader = vtkSmartPointer<vtkLeaderActor2D>::New ();
  leader->GetPositionCoordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPositionCoordinate ()->SetValue (pt1.x, pt1.y, pt1.z);
  leader->GetPosition2Coordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPosition2Coordinate ()->SetValue (pt2.x, pt2.y, pt2.z);
  leader->SetArrowStyleToFilled ();
  leader->SetArrowPlacementToPoint1 ();
  if (display_length)
    leader->AutoLabelOn ();
  else
    leader->AutoLabelOff ();

  leader->GetProperty ()->SetColor (r, g, b);
  addActorToRenderer (leader, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = leader;
  return (true);
}
////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addArrow (const P1 &pt1, const P2 &pt2,
					     double r_line, double g_line, double b_line,
					     double r_text, double g_text, double b_text,
					     const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addArrow] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  // Create an Actor
  vtkSmartPointer<vtkLeaderActor2D> leader = vtkSmartPointer<vtkLeaderActor2D>::New ();
  leader->GetPositionCoordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPositionCoordinate ()->SetValue (pt1.x, pt1.y, pt1.z);
  leader->GetPosition2Coordinate ()->SetCoordinateSystemToWorld ();
  leader->GetPosition2Coordinate ()->SetValue (pt2.x, pt2.y, pt2.z);
  leader->SetArrowStyleToFilled ();
  leader->AutoLabelOn ();

  leader->GetLabelTextProperty()->SetColor(r_text, g_text, b_text);
  
  leader->GetProperty ()->SetColor (r_line, g_line, b_line);
  addActorToRenderer (leader, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = leader;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addLine (const P1 &pt1, const P2 &pt2, const std::string &id, int viewport)
{
  return (!addLine (pt1, pt2, 0.5, 0.5, 0.5, id, viewport));
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addSphere (const PointT &center, double radius, double r, double g, double b, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addSphere] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkSphereSource> data = vtkSmartPointer<vtkSphereSource>::New ();
  data->SetRadius (radius);
  data->SetCenter (double (center.x), double (center.y), double (center.z));
  data->SetPhiResolution (10);
  data->SetThetaResolution (10);
  data->LatLongTessellationOff ();
  data->Update ();
 
  // Setup actor and mapper 
  vtkSmartPointer <vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  mapper->SetInputConnection (data->GetOutputPort ());

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  actor->SetMapper (mapper);
  //createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToSurface ();
  actor->GetProperty ()->SetInterpolationToFlat ();
  actor->GetProperty ()->SetColor (r, g, b);
  actor->GetMapper ()->ImmediateModeRenderingOn ();
  actor->GetMapper ()->StaticOn ();
  actor->GetMapper ()->ScalarVisibilityOff ();
  actor->GetMapper ()->Update ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addSphere (const PointT &center, double radius, const std::string &id, int viewport)
{
  return (addSphere (center, radius, 0.5, 0.5, 0.5, id, viewport));
}

////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> bool
pcl::visualization::PCLVisualizer::updateSphere (const PointT &center, double radius, double r, double g, double b, const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it == shape_actor_map_->end ())
    return (false);

  //////////////////////////////////////////////////////////////////////////
  // Get the actor pointer
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
  vtkAlgorithm *algo = actor->GetMapper ()->GetInput ()->GetProducerPort ()->GetProducer ();
  vtkSphereSource *src = vtkSphereSource::SafeDownCast (algo);

  src->SetCenter (double (center.x), double (center.y), double (center.z));
  src->SetRadius (radius);
  src->Update ();
  actor->GetProperty ()->SetColor (r, g, b);
  actor->Modified ();

  return (true);
}

//////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addText3D (
  const std::string &text,
  const PointT& position,
  double textScale,
  double r,
  double g,
  double b,
  const std::string &id,
  int viewport)
{
  std::string tid;
  if (id.empty ())
    tid = text;
  else
    tid = id;

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (tid);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, "[addText3d] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New ();
  textSource->SetText (text.c_str());
  textSource->Update ();

  vtkSmartPointer<vtkPolyDataMapper> textMapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  textMapper->SetInputConnection (textSource->GetOutputPort ());

  // Since each follower may follow a different camera, we need different followers
  rens_->InitTraversal ();
  vtkRenderer* renderer = NULL;
  int i = 1;
  while ((renderer = rens_->GetNextItem ()) != NULL)
  {
    // Should we add the actor to all renderers or just to i-nth renderer?
    if (viewport == 0 || viewport == i)
    {
      vtkSmartPointer<vtkFollower> textActor = vtkSmartPointer<vtkFollower>::New ();
      textActor->SetMapper (textMapper);
      textActor->SetPosition (position.x, position.y, position.z);
      textActor->SetScale (textScale);
      textActor->GetProperty ()->SetColor (r, g, b);
      textActor->SetCamera (renderer->GetActiveCamera ());

      renderer->AddActor (textActor);
      renderer->Render ();

      // Save the pointer/ID pair to the global actor map. If we are saving multiple vtkFollowers
      // for multiple viewport
      std::string alternate_tid = tid;
      alternate_tid.append(i, '*');

      (*shape_actor_map_)[(viewport == 0) ? tid : alternate_tid] = textActor;
    }

    ++i;
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> bool
pcl::visualization::PCLVisualizer::addPointCloudNormals (
  const typename pcl::PointCloud<PointNT>::ConstPtr &cloud,
  int level, float scale, const std::string &id, int viewport)
{
  return (addPointCloudNormals<PointNT, PointNT> (cloud, cloud, level, scale, id, viewport));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::visualization::PCLVisualizer::addPointCloudNormals (
  const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
  const typename pcl::PointCloud<PointNT>::ConstPtr &normals,
  int level, float scale,
  const std::string &id, int viewport)
{
  if (normals->points.size () != cloud->points.size ())
  {
    PCL_ERROR ("[addPointCloudNormals] The number of points differs from the number of normals!\n");
    return (false);
  }
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it != cloud_actor_map_->end ())
  {
    PCL_WARN ("[addPointCloudNormals] A PointCloud with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  points->SetDataTypeToFloat ();
  vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
  data->SetNumberOfComponents (3);


  vtkIdType nr_normals = 0;
  float* pts = 0;

  // If the cloud is organized, then distribute the normal step in both directions
  if (cloud->isOrganized () && normals->isOrganized ())
  {
    vtkIdType point_step = static_cast<vtkIdType> (sqrt (double (level)));
    nr_normals = (static_cast<vtkIdType> ((cloud->width - 1)/ point_step) + 1) *
                 (static_cast<vtkIdType> ((cloud->height - 1) / point_step) + 1);
    pts = new float[2 * nr_normals * 3];

    vtkIdType cell_count = 0;
    for (vtkIdType y = 0; y < normals->height; y += point_step)
      for (vtkIdType x = 0; x < normals->width; x += point_step)
      {
        PointT p = (*cloud)(x, y);
        p.x += (*normals)(x, y).normal[0] * scale;
        p.y += (*normals)(x, y).normal[1] * scale;
        p.z += (*normals)(x, y).normal[2] * scale;

        pts[2 * cell_count * 3 + 0] = (*cloud)(x, y).x;
        pts[2 * cell_count * 3 + 1] = (*cloud)(x, y).y;
        pts[2 * cell_count * 3 + 2] = (*cloud)(x, y).z;
        pts[2 * cell_count * 3 + 3] = p.x;
        pts[2 * cell_count * 3 + 4] = p.y;
        pts[2 * cell_count * 3 + 5] = p.z;

        lines->InsertNextCell (2);
        lines->InsertCellPoint (2 * cell_count);
        lines->InsertCellPoint (2 * cell_count + 1);
        cell_count ++;
    }
  }
  else
  {
    nr_normals = (cloud->points.size () - 1) / level + 1 ;
    pts = new float[2 * nr_normals * 3];

    for (vtkIdType i = 0, j = 0; j < nr_normals; j++, i = j * level)
    {
      PointT p = cloud->points[i];
      p.x += normals->points[i].normal[0] * scale;
      p.y += normals->points[i].normal[1] * scale;
      p.z += normals->points[i].normal[2] * scale;

      pts[2 * j * 3 + 0] = cloud->points[i].x;
      pts[2 * j * 3 + 1] = cloud->points[i].y;
      pts[2 * j * 3 + 2] = cloud->points[i].z;
      pts[2 * j * 3 + 3] = p.x;
      pts[2 * j * 3 + 4] = p.y;
      pts[2 * j * 3 + 5] = p.z;

      lines->InsertNextCell (2);
      lines->InsertCellPoint (2 * j);
      lines->InsertCellPoint (2 * j + 1);
    }
  }

  data->SetArray (&pts[0], 2 * nr_normals * 3, 0);
  points->SetData (data);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints (points);
  polyData->SetLines (lines);

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (polyData);
  mapper->SetColorModeToMapScalars();
  mapper->SetScalarModeToUsePointData();

  // create actor
  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  actor->SetMapper (mapper);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename GradientT> bool
pcl::visualization::PCLVisualizer::addPointCloudIntensityGradients (
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const typename pcl::PointCloud<GradientT>::ConstPtr &gradients,
    int level, double scale,
    const std::string &id, int viewport)
{
  if (gradients->points.size () != cloud->points.size ())
  {
    PCL_ERROR ("[addPointCloudGradients] The number of points differs from the number of gradients!\n");
    return (false);
  }
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it != cloud_actor_map_->end ())
  {
    PCL_WARN ("[addPointCloudGradients] A PointCloud with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();

  points->SetDataTypeToFloat ();
  vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
  data->SetNumberOfComponents (3);

  vtkIdType nr_gradients = (cloud->points.size () - 1) / level + 1 ;
  float* pts = new float[2 * nr_gradients * 3];

  for (vtkIdType i = 0, j = 0; j < nr_gradients; j++, i = j * level)
  {
    PointT p = cloud->points[i];
    p.x += gradients->points[i].gradient[0] * scale;
    p.y += gradients->points[i].gradient[1] * scale;
    p.z += gradients->points[i].gradient[2] * scale;

    pts[2 * j * 3 + 0] = cloud->points[i].x;
    pts[2 * j * 3 + 1] = cloud->points[i].y;
    pts[2 * j * 3 + 2] = cloud->points[i].z;
    pts[2 * j * 3 + 3] = p.x;
    pts[2 * j * 3 + 4] = p.y;
    pts[2 * j * 3 + 5] = p.z;

    lines->InsertNextCell(2);
    lines->InsertCellPoint(2*j);
    lines->InsertCellPoint(2*j+1);
  }

  data->SetArray (&pts[0], 2 * nr_gradients * 3, 0);
  points->SetData (data);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->SetLines(lines);

  vtkSmartPointer<vtkDataSetMapper> mapper = vtkSmartPointer<vtkDataSetMapper>::New ();
  mapper->SetInput (polyData);
  mapper->SetColorModeToMapScalars();
  mapper->SetScalarModeToUsePointData();

  // create actor
  vtkSmartPointer<vtkLODActor> actor = vtkSmartPointer<vtkLODActor>::New ();
  actor->SetMapper (mapper);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addCorrespondences (
  const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
  const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
  const std::vector<int> &correspondences,
  const std::string &id,
  int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addCorrespondences] A set of correspondences with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  int n_corr = int (correspondences.size ());
  vtkSmartPointer<vtkPolyData> line_data = vtkSmartPointer<vtkPolyData>::New ();

  // Prepare colors
  vtkSmartPointer<vtkUnsignedCharArray> line_colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  line_colors->SetNumberOfComponents (3);
  line_colors->SetName ("Colors");
  line_colors->SetNumberOfTuples (n_corr);
  unsigned char* colors = line_colors->GetPointer (0);
  memset (colors, 0, line_colors->GetNumberOfTuples () * line_colors->GetNumberOfComponents ());
  pcl::RGB rgb;
  // Will use random colors or RED by default
  rgb.r = 255; rgb.g = rgb.b = 0;

  // Prepare coordinates
  vtkSmartPointer<vtkPoints> line_points = vtkSmartPointer<vtkPoints>::New ();
  line_points->SetNumberOfPoints (2 * n_corr);

  vtkSmartPointer<vtkIdTypeArray> line_cells_id = vtkSmartPointer<vtkIdTypeArray>::New ();
  line_cells_id->SetNumberOfComponents (3);
  line_cells_id->SetNumberOfTuples (n_corr);
  vtkIdType *line_cell_id = line_cells_id->GetPointer (0);
  vtkSmartPointer<vtkCellArray> line_cells = vtkSmartPointer<vtkCellArray>::New ();

  vtkSmartPointer<vtkFloatArray> line_tcoords = vtkSmartPointer<vtkFloatArray>::New ();
  line_tcoords->SetNumberOfComponents (1);
  line_tcoords->SetNumberOfTuples (n_corr * 2);
  line_tcoords->SetName ("Texture Coordinates");

  double tc[3] = {0.0, 0.0, 0.0};

  int j = 0, idx = 0;
  // Draw lines between the best corresponding points
  for (int i = 0; i < n_corr; ++i)
  {
    const PointT &p_src = source_points->points[i];
    const PointT &p_tgt = target_points->points[correspondences[i]];

    int id1 = j * 2 + 0, id2 = j * 2 + 1;
    // Set the points
    line_points->SetPoint (id1, p_src.x, p_src.y, p_src.z);
    line_points->SetPoint (id2, p_tgt.x, p_tgt.y, p_tgt.z);
    // Set the cell ID
    *line_cell_id++ = 2;
    *line_cell_id++ = id1;
    *line_cell_id++ = id2;
    // Set the texture coords
    tc[0] = 0.; line_tcoords->SetTuple (id1, tc);
    tc[0] = 1.; line_tcoords->SetTuple (id2, tc);

    getRandomColors (rgb);
    colors[idx+0] = rgb.r;
    colors[idx+1] = rgb.g;
    colors[idx+2] = rgb.b;
  }
  line_colors->SetNumberOfTuples (j);
  line_cells_id->SetNumberOfTuples (j);
  line_cells->SetCells (n_corr, line_cells_id);
  line_points->SetNumberOfPoints (j*2);
  line_tcoords->SetNumberOfTuples (j*2);
 
  // Fill in the lines
  line_data->SetPoints (line_points);
  line_data->SetLines (line_cells);
  line_data->GetPointData ()->SetTCoords (line_tcoords);
  line_data->GetCellData ()->SetScalars (line_colors);
  line_data->Update ();

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (line_data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  actor->GetProperty ()->SetOpacity (0.5);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addCorrespondences (
  const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
  const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
  const pcl::Correspondences &correspondences,
  int nth,
  const std::string &id,
  int viewport)
{
  if (correspondences.empty ())
  {
    PCL_DEBUG ("[addCorrespondences] An empty set of correspondences given! Nothing to display.\n");
    return (false);
  }

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addCorrespondences] A set of correspondences with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  int n_corr = int (correspondences.size () / nth + 1);
  vtkSmartPointer<vtkPolyData> line_data = vtkSmartPointer<vtkPolyData>::New ();

  // Prepare colors
  vtkSmartPointer<vtkUnsignedCharArray> line_colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  line_colors->SetNumberOfComponents (3);
  line_colors->SetName ("Colors");
  line_colors->SetNumberOfTuples (n_corr);
  unsigned char* colors = line_colors->GetPointer (0);
  memset (colors, 0, line_colors->GetNumberOfTuples () * line_colors->GetNumberOfComponents ());
  pcl::RGB rgb;
  // Will use random colors or RED by default
  rgb.r = 255; rgb.g = rgb.b = 0;

  // Prepare coordinates
  vtkSmartPointer<vtkPoints> line_points = vtkSmartPointer<vtkPoints>::New ();
  line_points->SetNumberOfPoints (2 * n_corr);

  vtkSmartPointer<vtkIdTypeArray> line_cells_id = vtkSmartPointer<vtkIdTypeArray>::New ();
  line_cells_id->SetNumberOfComponents (3);
  line_cells_id->SetNumberOfTuples (n_corr);
  vtkIdType *line_cell_id = line_cells_id->GetPointer (0);
  vtkSmartPointer<vtkCellArray> line_cells = vtkSmartPointer<vtkCellArray>::New ();

  vtkSmartPointer<vtkFloatArray> line_tcoords = vtkSmartPointer<vtkFloatArray>::New ();
  line_tcoords->SetNumberOfComponents (1);
  line_tcoords->SetNumberOfTuples (n_corr * 2);
  line_tcoords->SetName ("Texture Coordinates");

  double tc[3] = {0.0, 0.0, 0.0};

  int j = 0, idx = 0;
  // Draw lines between the best corresponding points
  for (size_t i = 0; i < correspondences.size (); i += nth, idx = j * 3, ++j)
  {
    const PointT &p_src = source_points->points[correspondences[i].index_query];
    const PointT &p_tgt = target_points->points[correspondences[i].index_match];

    int id1 = j * 2 + 0, id2 = j * 2 + 1;
    // Set the points
    line_points->SetPoint (id1, p_src.x, p_src.y, p_src.z);
    line_points->SetPoint (id2, p_tgt.x, p_tgt.y, p_tgt.z);
    // Set the cell ID
    *line_cell_id++ = 2;
    *line_cell_id++ = id1;
    *line_cell_id++ = id2;
    // Set the texture coords
    tc[0] = 0.; line_tcoords->SetTuple (id1, tc);
    tc[0] = 1.; line_tcoords->SetTuple (id2, tc);

    getRandomColors (rgb);
    colors[idx+0] = rgb.r;
    colors[idx+1] = rgb.g;
    colors[idx+2] = rgb.b;
  }
  line_colors->SetNumberOfTuples (j);
  line_cells_id->SetNumberOfTuples (j);
  line_cells->SetCells (n_corr, line_cells_id);
  line_points->SetNumberOfPoints (j*2);
  line_tcoords->SetNumberOfTuples (j*2);
 
  // Fill in the lines
  line_data->SetPoints (line_points);
  line_data->SetLines (line_cells);
  line_data->GetPointData ()->SetTCoords (line_tcoords);
  line_data->GetCellData ()->SetScalars (line_colors);
  line_data->Update ();

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (line_data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  actor->GetProperty ()->SetOpacity (0.5);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::updateCorrespondences (
  const typename pcl::PointCloud<PointT>::ConstPtr &source_points,
  const typename pcl::PointCloud<PointT>::ConstPtr &target_points,
  const pcl::Correspondences &correspondences,
  int nth,
  const std::string &id)
{
  if (correspondences.empty ())
  {
    PCL_DEBUG ("[updateCorrespondences] An empty set of correspondences given! Nothing to display.\n");
    return (false);
  }

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it == shape_actor_map_->end ())
    return (false);

  vtkSmartPointer<vtkLODActor> actor = vtkLODActor::SafeDownCast (am_it->second);
  vtkSmartPointer<vtkPolyData> line_data = reinterpret_cast<vtkPolyDataMapper*>(actor->GetMapper ())->GetInput ();

  int n_corr = int (correspondences.size () / nth + 1);

  // Prepare colors
  vtkSmartPointer<vtkUnsignedCharArray> line_colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  line_colors->SetNumberOfComponents (3);
  line_colors->SetName ("Colors");
  line_colors->SetNumberOfTuples (n_corr);
  unsigned char* colors = line_colors->GetPointer (0);
  memset (colors, 0, line_colors->GetNumberOfTuples () * line_colors->GetNumberOfComponents ());
  pcl::RGB rgb;
  // Will use random colors or RED by default
  rgb.r = 255.0; rgb.g = rgb.b = 0.0;

  // Prepare coordinates
  vtkSmartPointer<vtkPoints> line_points = vtkSmartPointer<vtkPoints>::New ();
  line_points->SetNumberOfPoints (2 * n_corr);

  vtkSmartPointer<vtkIdTypeArray> line_cells_id = vtkSmartPointer<vtkIdTypeArray>::New ();
  line_cells_id->SetNumberOfComponents (3);
  line_cells_id->SetNumberOfTuples (n_corr);
  vtkIdType *line_cell_id = line_cells_id->GetPointer (0);
  vtkSmartPointer<vtkCellArray> line_cells = line_data->GetLines ();

  vtkSmartPointer<vtkFloatArray> line_tcoords = vtkSmartPointer<vtkFloatArray>::New ();
  line_tcoords->SetNumberOfComponents (1);
  line_tcoords->SetNumberOfTuples (n_corr * 2);
  line_tcoords->SetName ("Texture Coordinates");

  double tc[3] = {0.0, 0.0, 0.0};

  int j = 0, idx = 0;
  // Draw lines between the best corresponding points
  for (size_t i = 0; i < correspondences.size (); i += nth, idx = j * 3, ++j)
  {
    const PointT &p_src = source_points->points[correspondences[i].index_query];
    const PointT &p_tgt = target_points->points[correspondences[i].index_match];

    int id1 = j * 2 + 0, id2 = j * 2 + 1;
    // Set the points
    line_points->SetPoint (id1, p_src.x, p_src.y, p_src.z);
    line_points->SetPoint (id2, p_tgt.x, p_tgt.y, p_tgt.z);
    // Set the cell ID
    *line_cell_id++ = 2;
    *line_cell_id++ = id1;
    *line_cell_id++ = id2;
    // Set the texture coords
    tc[0] = 0.; line_tcoords->SetTuple (id1, tc);
    tc[0] = 1.; line_tcoords->SetTuple (id2, tc);

    getRandomColors (rgb);
    colors[idx+0] = rgb.r;
    colors[idx+1] = rgb.g;
    colors[idx+2] = rgb.b;
  }
  line_colors->SetNumberOfTuples (j);
  line_cells_id->SetNumberOfTuples (j);
  line_cells->SetCells (n_corr, line_cells_id);
  line_points->SetNumberOfPoints (j*2);
  line_tcoords->SetNumberOfTuples (j*2);
 
  // Fill in the lines
  line_data->SetPoints (line_points);
  line_data->SetLines (line_cells);
  line_data->GetPointData ()->SetTCoords (line_tcoords);
  line_data->GetCellData ()->SetScalars (line_colors);
  line_data->Update ();

  // Update the mapper
  reinterpret_cast<vtkPolyDataMapper*>(actor->GetMapper ())->SetInput (line_data);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::fromHandlersToScreen (
  const PointCloudGeometryHandler<PointT> &geometry_handler,
  const PointCloudColorHandler<PointT> &color_handler,
  const std::string &id,
  int viewport,
  const Eigen::Vector4f& sensor_origin,
  const Eigen::Quaternion<float>& sensor_orientation)
{
  if (!geometry_handler.isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid geometry handler (%s)!\n", id.c_str (), geometry_handler.getName ().c_str ());
    return (false);
  }

  if (!color_handler.isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid color handler (%s)!\n", id.c_str (), color_handler.getName ().c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkIdTypeArray> initcells;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (geometry_handler, polydata, initcells);
  // use the given geometry handler
  polydata->Update ();

  // Get the colors from the handler
  bool has_colors = false;
  double minmax[2];
  vtkSmartPointer<vtkDataArray> scalars;
  if (color_handler.getColor (scalars))
  {
    polydata->GetPointData ()->SetScalars (scalars);
    scalars->GetRange (minmax);
    has_colors = true;
  }

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);
  if (has_colors)
    actor->GetMapper ()->SetScalarRange (minmax);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  CloudActor& cloud_actor = (*cloud_actor_map_)[id];
  cloud_actor.actor = actor;
  cloud_actor.cells = initcells;

  // Save the viewpoint transformation matrix to the global actor map
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New();
  convertToVtkMatrix (sensor_origin, sensor_orientation, transformation);
  cloud_actor.viewpoint_transformation_ = transformation;
  cloud_actor.actor->SetUserMatrix (transformation);
  cloud_actor.actor->Modified ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::fromHandlersToScreen (
  const PointCloudGeometryHandler<PointT> &geometry_handler,
  const ColorHandlerConstPtr &color_handler,
  const std::string &id,
  int viewport,
  const Eigen::Vector4f& sensor_origin,
  const Eigen::Quaternion<float>& sensor_orientation)
{
  if (!geometry_handler.isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid geometry handler (%s)!\n", id.c_str (), geometry_handler.getName ().c_str ());
    return (false);
  }

  if (!color_handler->isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid color handler (%s)!\n", id.c_str (), color_handler->getName ().c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkIdTypeArray> initcells;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (geometry_handler, polydata, initcells);
  // use the given geometry handler
  polydata->Update ();

  // Get the colors from the handler
  bool has_colors = false;
  double minmax[2];
  vtkSmartPointer<vtkDataArray> scalars;
  if (color_handler->getColor (scalars))
  {
    polydata->GetPointData ()->SetScalars (scalars);
    scalars->GetRange (minmax);
    has_colors = true;
  }

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);
  if (has_colors)
    actor->GetMapper ()->SetScalarRange (minmax);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  CloudActor& cloud_actor = (*cloud_actor_map_)[id];
  cloud_actor.actor = actor;
  cloud_actor.cells = initcells;
  cloud_actor.color_handlers.push_back (color_handler);

  // Save the viewpoint transformation matrix to the global actor map
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New();
  convertToVtkMatrix (sensor_origin, sensor_orientation, transformation);
  cloud_actor.viewpoint_transformation_ = transformation;
  cloud_actor.actor->SetUserMatrix (transformation);
  cloud_actor.actor->Modified ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::fromHandlersToScreen (
  const GeometryHandlerConstPtr &geometry_handler,
  const PointCloudColorHandler<PointT> &color_handler,
  const std::string &id,
  int viewport,
  const Eigen::Vector4f& sensor_origin,
  const Eigen::Quaternion<float>& sensor_orientation)
{
  if (!geometry_handler->isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid geometry handler (%s)!\n", id.c_str (), geometry_handler->getName ().c_str ());
    return (false);
  }

  if (!color_handler.isCapable ())
  {
    PCL_WARN ("[fromHandlersToScreen] PointCloud <%s> requested with an invalid color handler (%s)!\n", id.c_str (), color_handler.getName ().c_str ());
    return (false);
  }

  vtkSmartPointer<vtkPolyData> polydata;
  vtkSmartPointer<vtkIdTypeArray> initcells;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler, polydata, initcells);
  // use the given geometry handler
  polydata->Update ();

  // Get the colors from the handler
  bool has_colors = false;
  double minmax[2];
  vtkSmartPointer<vtkDataArray> scalars;
  if (color_handler.getColor (scalars))
  {
    polydata->GetPointData ()->SetScalars (scalars);
    scalars->GetRange (minmax);
    has_colors = true;
  }

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);
  if (has_colors)
    actor->GetMapper ()->SetScalarRange (minmax);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  CloudActor& cloud_actor = (*cloud_actor_map_)[id];
  cloud_actor.actor = actor;
  cloud_actor.cells = initcells;
  cloud_actor.geometry_handlers.push_back (geometry_handler);

  // Save the viewpoint transformation matrix to the global actor map
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New ();
  convertToVtkMatrix (sensor_origin, sensor_orientation, transformation);
  cloud_actor.viewpoint_transformation_ = transformation;
  cloud_actor.actor->SetUserMatrix (transformation);
  cloud_actor.actor->Modified ();

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                     const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);

  vtkSmartPointer<vtkPolyData> polydata = reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (cloud, polydata, am_it->second.cells);
  polydata->Update ();

  // Set scalars to blank, since there is no way we can update them here.
  vtkSmartPointer<vtkDataArray> scalars;
  polydata->GetPointData ()->SetScalars (scalars);
  polydata->Update ();
  double minmax[2];
  minmax[0] = std::numeric_limits<double>::min ();
  minmax[1] = std::numeric_limits<double>::max ();
  am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();
  am_it->second.actor->GetMapper ()->SetScalarRange (minmax);

  // Update the mapper
  reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->SetInput (polydata);
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &,
                                                     const PointCloudGeometryHandler<PointT> &geometry_handler,
                                                     const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);

  vtkSmartPointer<vtkPolyData> polydata = reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
  if (!polydata)
    return (false);
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler, polydata, am_it->second.cells);

  // Set scalars to blank, since there is no way we can update them here.
  vtkSmartPointer<vtkDataArray> scalars;
  polydata->GetPointData ()->SetScalars (scalars);
  polydata->Update ();
  double minmax[2];
  minmax[0] = std::numeric_limits<double>::min ();
  minmax[1] = std::numeric_limits<double>::max ();
  am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();
  am_it->second.actor->GetMapper ()->SetScalarRange (minmax);

  // Update the mapper
  reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->SetInput (polydata);
  return (true);
}


/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
                                                     const PointCloudColorHandler<PointT> &color_handler,
                                                     const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);

  // Get the current poly data
  vtkSmartPointer<vtkPolyData> polydata = reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
  if (!polydata)
    return (false);
  vtkSmartPointer<vtkCellArray> vertices = polydata->GetVerts ();
  vtkSmartPointer<vtkPoints> points      = polydata->GetPoints ();
  // Copy the new point array in
  vtkIdType nr_points = cloud->points.size ();
  points->SetNumberOfPoints (nr_points);

  // Get a pointer to the beginning of the data array
  float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

  int pts = 0;
  // If the dataset is dense (no NaNs)
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i, pts += 3)
      memcpy (&data[pts], &cloud->points[i].x, 12);    // sizeof (float) * 3
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!isFinite (cloud->points[i]))
        continue;

      memcpy (&data[pts], &cloud->points[i].x, 12);    // sizeof (float) * 3
      pts += 3;
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
  }

  vtkSmartPointer<vtkIdTypeArray> cells = vertices->GetData ();
  updateCells (cells, am_it->second.cells, nr_points);

  // Set the cells and the vertices
  vertices->SetCells (nr_points, cells);

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  color_handler.getColor (scalars);
  double minmax[2];
  scalars->GetRange (minmax);
  // Update the data
  polydata->GetPointData ()->SetScalars (scalars);
  polydata->Update ();

  am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();
  am_it->second.actor->GetMapper ()->SetScalarRange (minmax);

  // Update the mapper
  reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->SetInput (polydata);
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPolygonMesh (
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const std::vector<pcl::Vertices> &vertices,
    const std::string &id,
    int viewport)
{
  if (vertices.empty () || cloud->points.empty ())
    return (false);

  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
  if (am_it != cloud_actor_map_->end ())
  {
    pcl::console::print_warn (stderr, 
                                "[addPolygonMesh] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  int rgb_idx = -1;
  std::vector<pcl::PCLPointField> fields;
  vtkSmartPointer<vtkUnsignedCharArray> colors;
  rgb_idx = pcl::getFieldIndex (*cloud, "rgb", fields);
  if (rgb_idx == -1)
    rgb_idx = pcl::getFieldIndex (*cloud, "rgba", fields);
  if (rgb_idx != -1)
  {
    colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");

    pcl::RGB rgb_data;
    for (size_t i = 0; i < cloud->size (); ++i)
    {
      if (!isFinite (cloud->points[i]))
        continue;
      memcpy (&rgb_data, reinterpret_cast<const char*> (&cloud->points[i]) + fields[rgb_idx].offset, sizeof (pcl::RGB));
      unsigned char color[3];
      color[0] = rgb_data.r;
      color[1] = rgb_data.g;
      color[2] = rgb_data.b;
      colors->InsertNextTupleValue (color);
    }
  }

  // Create points from polyMesh.cloud
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkIdType nr_points = cloud->points.size ();
  points->SetNumberOfPoints (nr_points);
  vtkSmartPointer<vtkLODActor> actor;

  // Get a pointer to the beginning of the data array
  float *data = static_cast<vtkFloatArray*> (points->GetData ())->GetPointer (0);

  int ptr = 0;
  std::vector<int> lookup;
  // If the dataset is dense (no NaNs)
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i, ptr += 3)
      memcpy (&data[ptr], &cloud->points[i].x, sizeof (float) * 3);
  }
  else
  {
    lookup.resize (nr_points);
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!isFinite (cloud->points[i]))
        continue;

      lookup[i] = static_cast<int> (j);
      memcpy (&data[ptr], &cloud->points[i].x, sizeof (float) * 3);
      j++;
      ptr += 3;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
  }

  // Get the maximum size of a polygon
  int max_size_of_polygon = -1;
  for (size_t i = 0; i < vertices.size (); ++i)
    if (max_size_of_polygon < static_cast<int> (vertices[i].vertices.size ()))
      max_size_of_polygon = static_cast<int> (vertices[i].vertices.size ());

  if (vertices.size () > 1)
  {
    // Create polys from polyMesh.polygons
    vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New ();
    vtkIdType *cell = cell_array->WritePointer (vertices.size (), vertices.size () * (max_size_of_polygon + 1));
    int idx = 0;
    if (lookup.size () > 0)
    {
      for (size_t i = 0; i < vertices.size (); ++i, ++idx)
      {
        size_t n_points = vertices[i].vertices.size ();
        *cell++ = n_points;
        //cell_array->InsertNextCell (n_points);
        for (size_t j = 0; j < n_points; j++, ++idx)
          *cell++ = lookup[vertices[i].vertices[j]];
          //cell_array->InsertCellPoint (lookup[vertices[i].vertices[j]]);
      }
    }
    else
    {
      for (size_t i = 0; i < vertices.size (); ++i, ++idx)
      {
        size_t n_points = vertices[i].vertices.size ();
        *cell++ = n_points;
        //cell_array->InsertNextCell (n_points);
        for (size_t j = 0; j < n_points; j++, ++idx)
          *cell++ = vertices[i].vertices[j];
          //cell_array->InsertCellPoint (vertices[i].vertices[j]);
      }
    }
    vtkSmartPointer<vtkPolyData> polydata;
    allocVtkPolyData (polydata);
    cell_array->GetData ()->SetNumberOfValues (idx);
    cell_array->Squeeze ();
    polydata->SetStrips (cell_array);
    polydata->SetPoints (points);
  
    if (colors)
      polydata->GetPointData ()->SetScalars (colors);

    createActorFromVTKDataSet (polydata, actor, false);
  }
  else
  {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
    size_t n_points = vertices[0].vertices.size ();
    polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

    if (lookup.size () > 0)
    {
      for (size_t j = 0; j < (n_points - 1); ++j)
        polygon->GetPointIds ()->SetId (j, lookup[vertices[0].vertices[j]]);
    }
    else
    {
      for (size_t j = 0; j < (n_points - 1); ++j)
        polygon->GetPointIds ()->SetId (j, vertices[0].vertices[j]);
    }
    vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
    allocVtkUnstructuredGrid (poly_grid);
    poly_grid->Allocate (1, 1);
    poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
    poly_grid->SetPoints (points);
    poly_grid->Update ();
    if (colors)
      poly_grid->GetPointData ()->SetScalars (colors);

    createActorFromVTKDataSet (poly_grid, actor, false);
  }
  addActorToRenderer (actor, viewport);
  actor->GetProperty ()->SetRepresentationToSurface ();
  // Backface culling renders the visualization slower, but guarantees that we see all triangles
  actor->GetProperty ()->BackfaceCullingOff ();
  actor->GetProperty ()->SetInterpolationToFlat ();
  actor->GetProperty ()->EdgeVisibilityOff ();
  actor->GetProperty ()->ShadingOff ();

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;

  // Save the viewpoint transformation matrix to the global actor map
  vtkSmartPointer<vtkMatrix4x4> transformation = vtkSmartPointer<vtkMatrix4x4>::New();
  convertToVtkMatrix (cloud->sensor_origin_, cloud->sensor_orientation_, transformation);
  (*cloud_actor_map_)[id].viewpoint_transformation_ = transformation;

  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::updatePolygonMesh (
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const std::vector<pcl::Vertices> &verts,
    const std::string &id)
{
  if (verts.empty ())
  {
     pcl::console::print_error ("[addPolygonMesh] No vertices given!\n");
     return (false);
  }

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);
  if (am_it == cloud_actor_map_->end ())
    return (false);

  // Get the current poly data
  vtkSmartPointer<vtkPolyData> polydata = static_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
  if (!polydata)
    return (false);
  vtkSmartPointer<vtkCellArray> cells = polydata->GetStrips ();
  if (!cells)
    return (false);
  vtkSmartPointer<vtkPoints> points   = polydata->GetPoints ();
  // Copy the new point array in
  vtkIdType nr_points = cloud->points.size ();
  points->SetNumberOfPoints (nr_points);

  // Get a pointer to the beginning of the data array
  float *data = (static_cast<vtkFloatArray*> (points->GetData ()))->GetPointer (0);

  int ptr = 0;
  std::vector<int> lookup;
  // If the dataset is dense (no NaNs)
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i, ptr += 3)
      memcpy (&data[ptr], &cloud->points[i].x, sizeof (float) * 3);
  }
  else
  {
    lookup.resize (nr_points);
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!isFinite (cloud->points[i]))
        continue;

      lookup [i] = static_cast<int> (j);
      memcpy (&data[ptr], &cloud->points[i].x, sizeof (float) * 3);
      j++;
      ptr += 3;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
  }

  // Update colors
  vtkUnsignedCharArray* colors = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());
  int rgb_idx = -1;
  std::vector<pcl::PCLPointField> fields;
  rgb_idx = pcl::getFieldIndex (*cloud, "rgb", fields);
  if (rgb_idx == -1)
    rgb_idx = pcl::getFieldIndex (*cloud, "rgba", fields);
  if (rgb_idx != -1 && colors)
  {
    pcl::RGB rgb_data;
    int j = 0;
    for (size_t i = 0; i < cloud->size (); ++i)
    {
      if (!isFinite (cloud->points[i]))
        continue;
      memcpy (&rgb_data, 
              reinterpret_cast<const char*> (&cloud->points[i]) + fields[rgb_idx].offset,
              sizeof (pcl::RGB));
      unsigned char color[3];
      color[0] = rgb_data.r;
      color[1] = rgb_data.g;
      color[2] = rgb_data.b;
      colors->SetTupleValue (j++, color);
    }
  }

  // Get the maximum size of a polygon
  int max_size_of_polygon = -1;
  for (size_t i = 0; i < verts.size (); ++i)
    if (max_size_of_polygon < static_cast<int> (verts[i].vertices.size ()))
      max_size_of_polygon = static_cast<int> (verts[i].vertices.size ());

  // Update the cells
  cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkIdType *cell = cells->WritePointer (verts.size (), verts.size () * (max_size_of_polygon + 1));
  int idx = 0;
  if (lookup.size () > 0)
  {
    for (size_t i = 0; i < verts.size (); ++i, ++idx)
    {
      size_t n_points = verts[i].vertices.size ();
      *cell++ = n_points;
      for (size_t j = 0; j < n_points; j++, cell++, ++idx)
        *cell = lookup[verts[i].vertices[j]];
    }
  }
  else
  {
    for (size_t i = 0; i < verts.size (); ++i, ++idx)
    {
      size_t n_points = verts[i].vertices.size ();
      *cell++ = n_points;
      for (size_t j = 0; j < n_points; j++, cell++, ++idx)
        *cell = verts[i].vertices[j];
    }
  }
  cells->GetData ()->SetNumberOfValues (idx);
  cells->Squeeze ();
  // Set the the vertices
  polydata->SetStrips (cells);
  polydata->Update ();

  return (true);
}

#endif
