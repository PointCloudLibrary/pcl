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
 * $Id$
 *
 */

#ifndef PCL_PCL_VISUALIZER_IMPL_H_
#define PCL_PCL_VISUALIZER_IMPL_H_

#include <boost/make_shared.hpp>

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
  
  PointCloudColorHandlerRandom<PointT> color_handler (cloud);
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport));
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

  PointCloudColorHandlerRandom<PointT> color_handler (cloud);
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport));
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
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport));
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
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport));
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
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport));
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
  return (fromHandlersToScreen (geometry_handler, color_handler, id, viewport));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::visualization::PCLVisualizer::convertPointCloudToVTKPolyData (
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud, 
    vtkSmartPointer<vtkPolyData> &polydata)
{
  if (!polydata)
    polydata = vtkSmartPointer<vtkPolyData>::New ();

  vtkIdType nr_points = cloud->points.size ();
  // Create the supporting structures
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkIdTypeArray> cells  = vtkSmartPointer<vtkIdTypeArray>::New ();
  vtkSmartPointer<vtkPoints> points      = vtkSmartPointer<vtkPoints>::New ();
  points->SetDataTypeToFloat ();

  vtkSmartPointer<vtkFloatArray> data = vtkSmartPointer<vtkFloatArray>::New ();
  data->SetNumberOfComponents (3);

  // Set the points
  vtkIdType j = 0;    // true point index
  float* pts = new float[nr_points * 3];

  cells->SetNumberOfValues (nr_points * 2);
  vtkIdType *cell = cells->GetPointer (0);
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      memcpy (&pts[j * 3], &cloud->points[i].x, 12);    // sizeof (float) * 3
      *cell++ = 1;
      *cell++ = i;
    }
    data->SetArray (&pts[0], nr_points * 3, 0);
    points->SetData (data);
    vertices->SetCells (nr_points, cells);
  }
  else
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) || !pcl_isfinite (cloud->points[i].y) || !pcl_isfinite (cloud->points[i].z))
        continue;

      memcpy (&pts[j * 3], &cloud->points[i].x, 12);    // sizeof (float) * 3
      *cell++ = 1;
      *cell++ = j;
      j++;
    }
    data->SetArray (&pts[0], j * 3, 0);
    points->SetData (data);
    cells->SetNumberOfValues (j * 2);
    vertices->SetCells (j, cells);
  }
  polydata->SetPoints (points);
  polydata->SetVerts (vertices);
  //delete [] pts;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void 
pcl::visualization::PCLVisualizer::convertPointCloudToVTKPolyData (
    const pcl::visualization::PointCloudGeometryHandler<PointT> &geometry_handler, 
    vtkSmartPointer<vtkPolyData> &polydata)
{
  if (!polydata)
    polydata = vtkSmartPointer<vtkPolyData>::New ();

  // Create the supporting structures
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkIdTypeArray> cells  = vtkSmartPointer<vtkIdTypeArray>::New ();

  // Use the handler to obtain the geometry
  vtkSmartPointer<vtkPoints> points;
  geometry_handler.getGeometry (points);
  
  cells->SetNumberOfValues (points->GetNumberOfPoints () * 2);
  vtkIdType *cell = cells->GetPointer (0);
  // Set the vertices
  for (vtkIdType i = 0; i < (int)points->GetNumberOfPoints (); ++i)
  {
    *cell++ = 1;
    *cell++ = i;
  }
  vertices->SetCells ((vtkIdType)points->GetNumberOfPoints (), (vtkIdTypeArray*)cells);
  polydata->SetPoints (points);
  polydata->SetVerts (vertices);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPolygon (
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud, 
    const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    PCL_WARN ("[addPolygon] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createPolygon<PointT> (cloud);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addPolygon (
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud, 
    double r, double g, double b, const std::string &id, int viewport)
{
  if (!addPolygon<PointT> (cloud, id, viewport))
    return (false);

  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
  actor->GetProperty ()->SetColor (r, g, b);
  actor->Modified ();
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addLine (const P1 &pt1, const P2 &pt2, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    PCL_WARN ("[addLine] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createLine<P1, P2> (pt1, pt2);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id, int viewport)
{
  if (!addLine (pt1, pt2, id, viewport))
    return (false);

  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
  actor->GetProperty ()->SetColor (r, g, b);
  actor->Modified ();
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
/*template <typename P1, typename P2> bool
  pcl::visualization::PCLVisualizer::addLineToGroup (const P1 &pt1, const P2 &pt2, const std::string &group_id, int viewport)
{
  vtkSmartPointer<vtkDataSet> data = createLine<P1, P2> (pt1, pt2);

  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (group_id);
  if (am_it != shape_actor_map_.end ())
  {
    // Get the old data
    vtkPolyDataMapper* mapper = static_cast<vtkPolyDataMapper*>(am_it->second->GetMapper ());
    vtkPolyData* olddata = static_cast<vtkPolyData*>(mapper->GetInput ());
    // Add it to the current data
    vtkSmartPointer<vtkAppendPolyData> alldata = vtkSmartPointer<vtkAppendPolyData>::New ();
    alldata->AddInput (olddata);

    // Convert to vtkPolyData
    vtkSmartPointer<vtkPolyData> curdata = (vtkPolyData::SafeDownCast (data));
    alldata->AddInput (curdata);

    mapper->SetInput (alldata->GetOutput ());

    am_it->second->SetMapper (mapper);
    am_it->second->Modified ();
    return (true);
  }

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  actor->GetProperty ()->SetColor (1, 0, 0);
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[group_id] = actor;

//ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
//vtkSmartPointer<vtkLODActor> actor = am_it->second;
  //actor->GetProperty ()->SetColor (r, g, b);
//actor->Modified ();
  return (true);
}*/

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addSphere (const PointT &center, double radius, const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  if (am_it != shape_actor_map_.end ())
  {
    PCL_WARN ("[addSphere] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createSphere<PointT> (center, radius);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  shape_actor_map_[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addSphere (const PointT &center, double radius, double r, double g, double b, const std::string &id, int viewport)
{
  if (!addSphere (center, radius, id, viewport))
    return (false);

  ShapeActorMap::iterator am_it = shape_actor_map_.find (id);
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
  actor->GetProperty ()->SetColor (r, g, b);
  actor->Modified ();
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointNT> bool
pcl::visualization::PCLVisualizer::addPointCloudNormals (
    const typename pcl::PointCloud<PointNT>::ConstPtr &cloud,
    int level, double scale, const std::string &id, int viewport)
{
  return (addPointCloudNormals<PointNT, PointNT>(cloud, cloud, level, scale, id, viewport));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename PointNT> bool
pcl::visualization::PCLVisualizer::addPointCloudNormals (
    const typename pcl::PointCloud<PointT>::ConstPtr &cloud,
    const typename pcl::PointCloud<PointNT>::ConstPtr &normals,
    int level, double scale,
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

  vtkSmartPointer<vtkAppendPolyData> polydata = vtkSmartPointer<vtkAppendPolyData>::New ();

  for (size_t i = 0; i < cloud->points.size (); i+=level)
  {
    PointT p = cloud->points[i];
    p.x += normals->points[i].normal[0] * scale; 
    p.y += normals->points[i].normal[1] * scale; 
    p.z += normals->points[i].normal[2] * scale;
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New ();
    line->SetPoint1 (cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
    line->SetPoint2 (p.x, p.y, p.z);
    line->Update ();
    polydata->AddInput (line->GetOutput ());
  }
  // Convert the PointCloud to VTK PolyData
  polydata->Update ();

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata->GetOutput (), actor);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  CloudActor act;
  //act.color_handlers.push_back (handler);
  act.actor = actor;
  (*cloud_actor_map_)[id] = act;
  //style_->setCloudActorMap (cloud_actor_map_);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::fromHandlersToScreen (
    const PointCloudGeometryHandler<PointT> &geometry_handler,
    const PointCloudColorHandler<PointT> &color_handler, 
    const std::string &id,
    int viewport)
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
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (geometry_handler, polydata);
  // use the given geometry handler
  polydata->Update ();

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  color_handler.getColor (scalars);
  polydata->GetPointData ()->SetScalars (scalars);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;
  //style_->setCloudActorMap (cloud_actor_map_);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::fromHandlersToScreen (
    const PointCloudGeometryHandler<PointT> &geometry_handler,
    const ColorHandlerConstPtr &color_handler, 
    const std::string &id,
    int viewport)
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
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (geometry_handler, polydata);
  // use the given geometry handler
  polydata->Update ();

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  color_handler->getColor (scalars);
  polydata->GetPointData ()->SetScalars (scalars);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;
  (*cloud_actor_map_)[id].color_handlers.push_back (color_handler);
  //style_->setCloudActorMap (cloud_actor_map_);
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::fromHandlersToScreen (
    const GeometryHandlerConstPtr &geometry_handler,
    const PointCloudColorHandler<PointT> &color_handler, 
    const std::string &id,
    int viewport)
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
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler, polydata);
  // use the given geometry handler
  polydata->Update ();

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  color_handler.getColor (scalars);
  polydata->GetPointData ()->SetScalars (scalars);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (polydata, actor);

  // Add it to all renderers
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*cloud_actor_map_)[id].actor = actor;
  (*cloud_actor_map_)[id].geometry_handlers.push_back (geometry_handler);
  //style_->setCloudActorMap (cloud_actor_map_);
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, 
                                                     const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);

  vtkSmartPointer<vtkPolyData> polydata;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (cloud, polydata);
  polydata->Update ();

  // Update the mapper
  reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->SetInput (polydata);
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::updatePointCloud (const typename pcl::PointCloud<PointT>::ConstPtr &cloud, 
                                                     const PointCloudGeometryHandler<PointT> &geometry_handler,
                                                     const std::string &id)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  CloudActorMap::iterator am_it = cloud_actor_map_->find (id);

  if (am_it == cloud_actor_map_->end ())
    return (false);

  vtkSmartPointer<vtkPolyData> polydata;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler, polydata);
  polydata->Update ();

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
  vtkPolyData* polydata = (vtkPolyData*)am_it->second.actor->GetMapper ()->GetInput ();
  vtkCellArray* vertices = polydata->GetVerts ();
  vtkIdTypeArray* cells  = vertices->GetData ();
  vtkPoints* points      = polydata->GetPoints ();
  vtkFloatArray* data    = (vtkFloatArray*)points->GetData ();

  // Copy the new point array in
  vtkIdType nr_points = cloud->points.size ();
  vtkIdType j = 0;    // true point index
  float* pts = new float[nr_points * 3];

  cells->SetNumberOfValues (nr_points * 2);
  vtkIdType *cell = cells->GetPointer (0);
  // If the dataset is dense (no NaNs)
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      memcpy (&pts[j * 3], &cloud->points[i].x, 12);    // sizeof (float) * 3
      *cell++ = 1;
      *cell++ = i;
    }
    data->SetArray (&pts[0], nr_points * 3, 0);
    points->SetData (data);
    vertices->SetCells (nr_points, cells);
  }
  else
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) || !pcl_isfinite (cloud->points[i].y) || !pcl_isfinite (cloud->points[i].z))
        continue;

      memcpy (&pts[j * 3], &cloud->points[i].x, 12);    // sizeof (float) * 3
      *cell++ = 1;
      *cell++ = j;
      j++;
    }
    data->SetArray (&pts[0], j * 3, 0);
    points->SetData (data);
    cells->SetNumberOfValues (j * 2);
    vertices->SetCells (j, cells);
  }
  ////polydata->SetPoints (points);
  ////polydata->SetVerts (vertices);

  // Convert the PointCloud to VTK PolyData
  //convertPointCloudToVTKPolyData<PointT> (cloud, polydata);

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  color_handler.getColor (scalars);
  polydata->GetPointData ()->SetScalars (scalars);
  polydata->Update ();

  // Update the mapper
  reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->SetInput (polydata);
  //delete [] pts;
  return (true);
}


#endif
