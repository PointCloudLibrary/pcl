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

  //PointCloudColorHandlerRandom<PointT> color_handler (cloud);
  PointCloudColorHandlerCustom<PointT> color_handler (cloud, 255, 255, 255);
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
  float *data = ((vtkFloatArray*)points->GetData ())->GetPointer (0);

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
    const std::string &id, int viewport)
{
  // Check to see if this ID entry already exists (has it been already added to the visualizer?)
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
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
  (*shape_actor_map_)[id] = actor;
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

  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
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
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename P1, typename P2> bool
pcl::visualization::PCLVisualizer::addLine (const P1 &pt1, const P2 &pt2, double r, double g, double b, const std::string &id, int viewport)
{
  if (!addLine (pt1, pt2, id, viewport))
    return (false);

  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
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
  ShapeActorMap::iterator am_it = shape_actor_map_->find (group_id);
  if (am_it != shape_actor_map_->end ())
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
  (*shape_actor_map_)[group_id] = actor;

//ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
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
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    PCL_WARN ("[addSphere] A shape with id <%s> already exists! Please choose a different id and retry.\n", id.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkDataSet> data = createSphere (center.getVector4fMap (), radius);

  // Create an Actor
  vtkSmartPointer<vtkLODActor> actor;
  createActorFromVTKDataSet (data, actor);
  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}

////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
pcl::visualization::PCLVisualizer::addSphere (const PointT &center, double radius, double r, double g, double b, const std::string &id, int viewport)
{
  if (!addSphere (center, radius, id, viewport))
    return (false);

  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  vtkLODActor* actor = vtkLODActor::SafeDownCast (am_it->second);
  actor->GetProperty ()->SetColor (r, g, b);
  actor->Modified ();
  return (true);
}

/////////////////////////////////////////////////////////////////////////////////////////////
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
    pcl::console::print_warn ("[addText3d] A text with id <%s> already exists! Please choose a different id and retry.\n", tid.c_str ());
    return (false);
  }

  vtkSmartPointer<vtkVectorText> textSource = vtkSmartPointer<vtkVectorText>::New();
  textSource->SetText(text.c_str());
  textSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> textMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  textMapper->SetInputConnection(textSource->GetOutputPort());

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
      textActor->SetMapper(textMapper);
      textActor->SetPosition(position.x, position.y, position.z);
      textActor->SetScale(textScale);
      textActor->GetProperty()->SetColor( r, g, b );
      textActor->SetCamera(renderer->GetActiveCamera());

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
  vtkSmartPointer<vtkIdTypeArray> initcells;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (geometry_handler, polydata, initcells);
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
  (*cloud_actor_map_)[id].cells = initcells;
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
  vtkSmartPointer<vtkIdTypeArray> initcells;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData<PointT> (geometry_handler, polydata, initcells);
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
  (*cloud_actor_map_)[id].cells = initcells;
  (*cloud_actor_map_)[id].color_handlers.push_back (color_handler);
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
  vtkSmartPointer<vtkIdTypeArray> initcells;
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler, polydata, initcells);
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
  (*cloud_actor_map_)[id].cells = initcells;
  (*cloud_actor_map_)[id].geometry_handlers.push_back (geometry_handler);
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
  am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();

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

  vtkSmartPointer<vtkPolyData> polydata = reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->GetInput ();
  if (!polydata)
    return (false);
  // Convert the PointCloud to VTK PolyData
  convertPointCloudToVTKPolyData (geometry_handler, polydata, am_it->second.cells);

  // Set scalars to blank, since there is no way we can update them here. 
  vtkSmartPointer<vtkDataArray> scalars;
  polydata->GetPointData ()->SetScalars (scalars);
  polydata->Update ();
  am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();

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
  float *data = ((vtkFloatArray*)points->GetData ())->GetPointer (0);

  // If the dataset is dense (no NaNs)
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
  updateCells (cells, am_it->second.cells, nr_points);

  // Set the cells and the vertices
  vertices->SetCells (nr_points, cells);

  // Get the colors from the handler
  vtkSmartPointer<vtkDataArray> scalars;
  color_handler.getColor (scalars);
  polydata->GetPointData ()->SetScalars (scalars);
  polydata->Update ();
  
  am_it->second.actor->GetMapper ()->ImmediateModeRenderingOff ();

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
  ShapeActorMap::iterator am_it = shape_actor_map_->find (id);
  if (am_it != shape_actor_map_->end ())
  {
    pcl::console::print_warn (
                                "[addPolygonMesh] A shape with id <%s> already exists! Please choose a different id and retry.\n",
                                id.c_str ());
    return (false);
  }

  // Create points from polyMesh.cloud
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkIdType nr_points = cloud->points.size ();
  points->SetNumberOfPoints (nr_points);

  // Get a pointer to the beginning of the data array
  float *data = ((vtkFloatArray*)points->GetData ())->GetPointer (0);

  // If the dataset is dense (no NaNs)
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

  vtkSmartPointer<vtkLODActor> actor;
  if (vertices.size () > 1) 
  {
    //create polys from polyMesh.polygons
    vtkSmartPointer<vtkCellArray> cell_array = vtkSmartPointer<vtkCellArray>::New ();
    
    for (size_t i = 0; i < vertices.size (); ++i) 
    {
      size_t n_points = vertices[i].vertices.size ();
      cell_array->InsertNextCell (n_points);
      for (size_t j = 0; j < n_points; j++) 
        cell_array->InsertCellPoint (vertices[i].vertices[j]);
    }

    vtkSmartPointer<vtkPolyData> polydata;
    allocVtkPolyData (polydata);
    polydata->SetStrips (cell_array);
    polydata->SetPoints (points);

    createActorFromVTKDataSet (polydata, actor);
  } 
  else 
  {
    vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New ();
    size_t n_points = vertices[0].vertices.size ();
    polygon->GetPointIds ()->SetNumberOfIds (n_points - 1);

    for (size_t j = 0; j < (n_points - 1); ++j) 
      polygon->GetPointIds ()->SetId (j, vertices[0].vertices[j]);

    vtkSmartPointer<vtkUnstructuredGrid> poly_grid;
    allocVtkUnstructuredGrid (poly_grid);
    poly_grid->Allocate (1, 1);
    poly_grid->InsertNextCell (polygon->GetCellType (), polygon->GetPointIds ());
    poly_grid->SetPoints (points);
    poly_grid->Update ();

    createActorFromVTKDataSet (poly_grid, actor);
  }

  actor->GetProperty ()->SetRepresentationToWireframe ();
  addActorToRenderer (actor, viewport);

  // Save the pointer/ID pair to the global actor map
  (*shape_actor_map_)[id] = actor;
  return (true);
}


/////////////////////////////////////////////////////////////////////////////////////////////
/* Optimized function: need to do something with the signature as it colides with the general T one
bool
pcl::visualization::PCLVisualizer::updatePointCloud (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                                                     const PointCloudColorHandlerRGBField<pcl::PointXYZRGB> &color_handler,
                                                     const std::string &id)
{
  win_->SetAbortRender (1);
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
//  vtkUnsignedCharArray* scalars = vtkUnsignedCharArray::SafeDownCast (polydata->GetPointData ()->GetScalars ());
  vtkSmartPointer<vtkUnsignedCharArray> scalars = vtkSmartPointer<vtkUnsignedCharArray>::New ();
  // Copy the new point array in
  vtkIdType nr_points = cloud->points.size ();
  points->SetNumberOfPoints (nr_points);
  scalars->SetNumberOfComponents (3);
  scalars->SetNumberOfTuples (nr_points);
  polydata->GetPointData ()->SetScalars (scalars);
  unsigned char* colors = scalars->GetPointer (0);
 
  // Get a pointer to the beginning of the data array
  float *data = ((vtkFloatArray*)points->GetData ())->GetPointer (0);

  // If the dataset is dense (no NaNs)
  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      memcpy (&data[i * 3], &cloud->points[i].x, 12);    // sizeof (float) * 3
      int idx = i * 3;
      colors[idx    ] = cloud->points[i].r;
      colors[idx + 1] = cloud->points[i].g;
      colors[idx + 2] = cloud->points[i].b;
    }
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
      int idx = j * 3;
      colors[idx    ] = cloud->points[i].r;
      colors[idx + 1] = cloud->points[i].g;
      colors[idx + 2] = cloud->points[i].b;
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints (nr_points);
    scalars->SetNumberOfTuples (nr_points);
  }

  vtkSmartPointer<vtkIdTypeArray> cells = vertices->GetData ();
  updateCells (cells, am_it->second.cells, nr_points);

  // Set the cells and the vertices
  vertices->SetCells (nr_points, cells);

  // Update the mapper
  reinterpret_cast<vtkPolyDataMapper*>(am_it->second.actor->GetMapper ())->SetInput (polydata);
  return (true);
}*/


#endif
