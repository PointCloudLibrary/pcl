/*
 * render_views_tesselated_sphere.cpp
 *
 *  Created on: Dec 23, 2011
 *      Author: aitor
 */
#include <pcl/point_types.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <vtkCellData.h>
#include <vtkWorldPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkTriangle.h>
#include <vtkTransform.h>
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>4)
#include <vtkHardwareSelector.h>
#include <vtkSelectionNode.h>
#else 
#include <vtkVisibleCellSelector.h>
#endif
#include <vtkSelection.h>
#include <vtkCellArray.h>
#include <vtkTransformFilter.h>
#include <vtkCamera.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointPicker.h>

void
pcl::apps::RenderViewsTesselatedSphere::generateViews() {
  //center object
  double CoM[3];
  vtkIdType npts_com = 0, *ptIds_com = NULL;
  vtkSmartPointer<vtkCellArray> cells_com = polydata_->GetPolys ();

  double center[3], p1_com[3], p2_com[3], p3_com[3], area_com, totalArea_com = 0;
  double comx = 0, comy = 0, comz = 0;
  for (cells_com->InitTraversal (); cells_com->GetNextCell (npts_com, ptIds_com);)
  {
    polydata_->GetPoint (ptIds_com[0], p1_com);
    polydata_->GetPoint (ptIds_com[1], p2_com);
    polydata_->GetPoint (ptIds_com[2], p3_com);
    vtkTriangle::TriangleCenter (p1_com, p2_com, p3_com, center);
    area_com = vtkTriangle::TriangleArea (p1_com, p2_com, p3_com);
    comx += center[0] * area_com;
    comy += center[1] * area_com;
    comz += center[2] * area_com;
    totalArea_com += area_com;
  }

  CoM[0] = comx / totalArea_com;
  CoM[1] = comy / totalArea_com;
  CoM[2] = comz / totalArea_com;

  vtkSmartPointer<vtkTransform> trans_center = vtkSmartPointer<vtkTransform>::New ();
  trans_center->Translate (-CoM[0], -CoM[1], -CoM[2]);
  vtkSmartPointer<vtkMatrix4x4> matrixCenter = trans_center->GetMatrix ();

  vtkSmartPointer<vtkTransformFilter> trans_filter_center = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter_center->SetTransform (trans_center);
#if VTK_MAJOR_VERSION < 6
  trans_filter_center->SetInput (polydata_);
#else
  trans_filter_center->SetInputData (polydata_);
#endif
  trans_filter_center->Update ();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New ();
  mapper->SetInputConnection (trans_filter_center->GetOutputPort ());
  mapper->Update ();

  //scale so it fits in the unit sphere!
  double bb[6];
  mapper->GetBounds (bb);
  double ms = (std::max) ((std::fabs) (bb[0] - bb[1]),
                          (std::max) ((std::fabs) (bb[2] - bb[3]), (std::fabs) (bb[4] - bb[5])));
  double max_side = radius_sphere_ / 2.0;
  double scale_factor = max_side / ms;

  vtkSmartPointer<vtkTransform> trans_scale = vtkSmartPointer<vtkTransform>::New ();
  trans_scale->Scale (scale_factor, scale_factor, scale_factor);
  vtkSmartPointer<vtkMatrix4x4> matrixScale = trans_scale->GetMatrix ();

  vtkSmartPointer<vtkTransformFilter> trans_filter_scale = vtkSmartPointer<vtkTransformFilter>::New ();
  trans_filter_scale->SetTransform (trans_scale);
  trans_filter_scale->SetInputConnection (trans_filter_center->GetOutputPort ());
  trans_filter_scale->Update ();

  mapper->SetInputConnection (trans_filter_scale->GetOutputPort ());
  mapper->Update ();

  //////////////////////////////
  // * Compute area of the mesh
  //////////////////////////////
  vtkSmartPointer<vtkCellArray> cells = mapper->GetInput ()->GetPolys ();
  vtkIdType npts = 0, *ptIds = NULL;

  double p1[3], p2[3], p3[3], area, totalArea = 0;
  for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds);)
  {
    polydata_->GetPoint (ptIds[0], p1);
    polydata_->GetPoint (ptIds[1], p2);
    polydata_->GetPoint (ptIds[2], p3);
    area = vtkTriangle::TriangleArea (p1, p2, p3);
    totalArea += area;
  }

  //create icosahedron
  vtkSmartPointer<vtkPlatonicSolidSource> ico = vtkSmartPointer<vtkPlatonicSolidSource>::New ();
  ico->SetSolidTypeToIcosahedron ();
  ico->Update ();

  //tesselate cells from icosahedron
  vtkSmartPointer<vtkLoopSubdivisionFilter> subdivide = vtkSmartPointer<vtkLoopSubdivisionFilter>::New ();
  subdivide->SetNumberOfSubdivisions (tesselation_level_);
  subdivide->SetInputConnection (ico->GetOutputPort ());
#if VTK_MAJOR_VERSION>=6
  subdivide->Update();
#endif

  // Get camera positions
  vtkPolyData *sphere = subdivide->GetOutput ();
#if VTK_MAJOR_VERSION<6
  sphere->Update ();
#endif

  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > cam_positions;
  if (!use_vertices_)
  {
    vtkSmartPointer<vtkCellArray> cells_sphere = sphere->GetPolys ();
    cam_positions.resize (sphere->GetNumberOfPolys ());

    size_t i=0;
    for (cells_sphere->InitTraversal (); cells_sphere->GetNextCell (npts_com, ptIds_com);)
    {
      sphere->GetPoint (ptIds_com[0], p1_com);
      sphere->GetPoint (ptIds_com[1], p2_com);
      sphere->GetPoint (ptIds_com[2], p3_com);
      vtkTriangle::TriangleCenter (p1_com, p2_com, p3_com, center);
      cam_positions[i] = Eigen::Vector3f (float (center[0]), float (center[1]), float (center[2]));
      i++;
    }

  }
  else
  {
    cam_positions.resize (sphere->GetNumberOfPoints ());
    for (int i = 0; i < sphere->GetNumberOfPoints (); i++)
    {
      double cam_pos[3];
      sphere->GetPoint (i, cam_pos);
      cam_positions[i] = Eigen::Vector3f (float (cam_pos[0]), float (cam_pos[1]), float (cam_pos[2]));
    }
  }

  /*int valid = 0;
  for (size_t i = 0; i < cam_positions.size (); i++)
  {
    if (campos_constraints_func_ (cam_positions[i]))
    {
      cam_positions[valid] = cam_positions[i];
      valid++;
    }
  }

  cam_positions.resize (valid);*/

  double camera_radius = radius_sphere_;
  double cam_pos[3];
  double first_cam_pos[3];

  first_cam_pos[0] = cam_positions[0][0] * radius_sphere_;
  first_cam_pos[1] = cam_positions[0][1] * radius_sphere_;
  first_cam_pos[2] = cam_positions[0][2] * radius_sphere_;

  //create renderer and window
  vtkSmartPointer<vtkRenderWindow> render_win = vtkSmartPointer<vtkRenderWindow>::New ();
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New ();
  render_win->AddRenderer (renderer);
  render_win->SetSize (resolution_, resolution_);
  renderer->SetBackground (1.0, 0, 0);

  //create picker
  vtkSmartPointer<vtkWorldPointPicker> worldPicker = vtkSmartPointer<vtkWorldPointPicker>::New ();

  vtkSmartPointer<vtkCamera> cam = vtkSmartPointer<vtkCamera>::New ();
  cam->SetFocalPoint (0, 0, 0);

  Eigen::Vector3f cam_pos_3f = cam_positions[0];
  Eigen::Vector3f perp = cam_pos_3f.cross (Eigen::Vector3f::UnitY ());
  cam->SetViewUp (perp[0], perp[1], perp[2]);

  cam->SetPosition (first_cam_pos);
  cam->SetViewAngle (view_angle_);
  cam->Modified ();

  //For each camera position, traposesnsform the object and render view
  for (size_t i = 0; i < cam_positions.size (); i++)
  {
    cam_pos[0] = cam_positions[i][0];
    cam_pos[1] = cam_positions[i][1];
    cam_pos[2] = cam_positions[i][2];

    //create temporal virtual camera
    vtkSmartPointer<vtkCamera> cam_tmp = vtkSmartPointer<vtkCamera>::New ();
    cam_tmp->SetViewAngle (view_angle_);

    Eigen::Vector3f cam_pos_3f (static_cast<float> (cam_pos[0]), static_cast<float> (cam_pos[1]), static_cast<float> (cam_pos[2]));
    cam_pos_3f = cam_pos_3f.normalized ();
    Eigen::Vector3f test = Eigen::Vector3f::UnitY ();

    //If the view up is parallel to ray cam_pos - focalPoint then the transformation
    //is singular and no points are rendered...
    //make sure it is perpendicular
    if (fabs (cam_pos_3f.dot (test)) == 1)
    {
      //parallel, create
      test = cam_pos_3f.cross (Eigen::Vector3f::UnitX ());
    }

    cam_tmp->SetViewUp (test[0], test[1], test[2]);

    for (int k = 0; k < 3; k++)
    {
      cam_pos[k] = cam_pos[k] * camera_radius;
    }

    cam_tmp->SetPosition (cam_pos);
    cam_tmp->SetFocalPoint (0, 0, 0);
    cam_tmp->Modified ();

    //rotate model so it looks the same as if we would look from the new position
    vtkSmartPointer<vtkMatrix4x4> view_trans_inverted = vtkSmartPointer<vtkMatrix4x4>::New ();
    vtkMatrix4x4::Invert (cam->GetViewTransformMatrix (), view_trans_inverted);
    vtkSmartPointer<vtkTransform> trans_rot_pose = vtkSmartPointer<vtkTransform>::New ();
    trans_rot_pose->Identity ();
    trans_rot_pose->Concatenate (view_trans_inverted);
    trans_rot_pose->Concatenate (cam_tmp->GetViewTransformMatrix ());
    vtkSmartPointer<vtkTransformFilter> trans_rot_pose_filter = vtkSmartPointer<vtkTransformFilter>::New ();
    trans_rot_pose_filter->SetTransform (trans_rot_pose);
    trans_rot_pose_filter->SetInputConnection (trans_filter_scale->GetOutputPort ());

    //translate model so we can place camera at (0,0,0)
    vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New ();
    translation->Translate (first_cam_pos[0] * -1, first_cam_pos[1] * -1, first_cam_pos[2] * -1);
    vtkSmartPointer<vtkTransformFilter> translation_filter = vtkSmartPointer<vtkTransformFilter>::New ();
    translation_filter->SetTransform (translation);
    translation_filter->SetInputConnection (trans_rot_pose_filter->GetOutputPort ());

    //modify camera
    cam_tmp->SetPosition (0, 0, 0);
    cam_tmp->SetFocalPoint (first_cam_pos[0] * -1, first_cam_pos[1] * -1, first_cam_pos[2] * -1);
    cam_tmp->Modified ();

    //notice transformations for final pose
    vtkSmartPointer<vtkMatrix4x4> matrixRotModel = trans_rot_pose->GetMatrix ();
    vtkSmartPointer<vtkMatrix4x4> matrixTranslation = translation->GetMatrix ();

    mapper->SetInputConnection (translation_filter->GetOutputPort ());
    mapper->Update ();

    //render view
    vtkSmartPointer<vtkActor> actor_view = vtkSmartPointer<vtkActor>::New ();
    actor_view->SetMapper (mapper);
    renderer->SetActiveCamera (cam_tmp);
    renderer->AddActor (actor_view);
    renderer->Modified ();
    //renderer->ResetCameraClippingRange ();
    render_win->Render ();

    //back to real scale transform
    vtkSmartPointer<vtkTransform> backToRealScale = vtkSmartPointer<vtkTransform>::New ();
    backToRealScale->PostMultiply ();
    backToRealScale->Identity ();
    backToRealScale->Concatenate (matrixScale);
    backToRealScale->Concatenate (matrixTranslation);
    backToRealScale->Inverse ();
    backToRealScale->Modified ();
    backToRealScale->Concatenate (matrixTranslation);
    backToRealScale->Modified ();

    Eigen::Matrix4f backToRealScale_eigen;
    backToRealScale_eigen.setIdentity ();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        backToRealScale_eigen (x, y) = float (backToRealScale->GetMatrix ()->GetElement (x, y));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize (resolution_ * resolution_);

    if (gen_organized_)
    {
      cloud->width = resolution_;
      cloud->height = resolution_;
      cloud->is_dense = false;

      double coords[3];
      float * depth = new float[resolution_ * resolution_];
      render_win->GetZbufferData (0, 0, resolution_ - 1, resolution_ - 1, &(depth[0]));

      for (int x = 0; x < resolution_; x++)
      {
        for (int y = 0; y < resolution_; y++)
        {
          float value = depth[y * resolution_ + x];
          if (value == 1.0)
          {
            cloud->at (y, x).x = cloud->at (y, x).y = cloud->at (y, x).z = std::numeric_limits<float>::quiet_NaN ();
          }
          else
          {
            worldPicker->Pick (x, y, value, renderer);
            worldPicker->GetPickPosition (coords);
            cloud->at (y, x).x = static_cast<float> (coords[0]);
            cloud->at (y, x).y = static_cast<float> (coords[1]);
            cloud->at (y, x).z = static_cast<float> (coords[2]);
            cloud->at (y, x).getVector4fMap () = backToRealScale_eigen
                                  * cloud->at (y, x).getVector4fMap ();
          }
        }
      }

      delete[] depth;

    }
    else
    {
      cloud->width = resolution_ * resolution_;
      cloud->height = 1;

      double coords[3];
      float * depth = new float[resolution_ * resolution_];
      render_win->GetZbufferData (0, 0, resolution_ - 1, resolution_ - 1, &(depth[0]));

      int count_valid_depth_pixels = 0;
      for (int x = 0; x < resolution_; x++)
      {
        for (int y = 0; y < resolution_; y++)
        {
          float value = depth[y * resolution_ + x];
          if (value == 1.0)
            continue;

          worldPicker->Pick (x, y, value, renderer);
          worldPicker->GetPickPosition (coords);
          cloud->points[count_valid_depth_pixels].x = static_cast<float> (coords[0]);
          cloud->points[count_valid_depth_pixels].y = static_cast<float> (coords[1]);
          cloud->points[count_valid_depth_pixels].z = static_cast<float> (coords[2]);
          cloud->points[count_valid_depth_pixels].getVector4fMap () = backToRealScale_eigen
                      * cloud->points[count_valid_depth_pixels].getVector4fMap ();
          count_valid_depth_pixels++;
        }
      }

      delete[] depth;

      cloud->points.resize (count_valid_depth_pixels);
      cloud->width = count_valid_depth_pixels;
    }

    if(compute_entropy_) {
      //////////////////////////////
      // * Compute area of the mesh
      //////////////////////////////

      vtkSmartPointer<vtkPolyData> polydata = mapper->GetInput ();
      polydata->BuildCells ();

      vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys ();
      vtkIdType npts = 0, *ptIds = NULL;

      double p1[3], p2[3], p3[3], area, totalArea = 0;
      for (cells->InitTraversal (); cells->GetNextCell (npts, ptIds);)
      {
        polydata->GetPoint (ptIds[0], p1);
        polydata->GetPoint (ptIds[1], p2);
        polydata->GetPoint (ptIds[2], p3);
        area = vtkTriangle::TriangleArea (p1, p2, p3);
        totalArea += area;
      }

      /////////////////////////////////////
      // * Select visible cells (triangles)
      /////////////////////////////////////
#if (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION<6)
      vtkSmartPointer<vtkVisibleCellSelector> selector = vtkSmartPointer<vtkVisibleCellSelector>::New ();
      vtkSmartPointer<vtkIdTypeArray> selection = vtkSmartPointer<vtkIdTypeArray>::New ();

      selector->SetRenderer (renderer);
      selector->SetArea (0, 0, resolution_ - 1, resolution_ - 1);
      selector->Select ();
      selector->GetSelectedIds (selection);

      double visible_area = 0;
      for (int sel_id = 3; sel_id < (selection->GetNumberOfTuples () * selection->GetNumberOfComponents ()); sel_id
          += selection->GetNumberOfComponents ())
      {
        int id_mesh = int (selection->GetValue (sel_id));

        if (id_mesh >= polydata->GetNumberOfCells ())
          continue;

        vtkCell * cell = polydata->GetCell (id_mesh);
        vtkTriangle* triangle = dynamic_cast<vtkTriangle*> (cell);
        double p0[3];
        double p1[3];
        double p2[3];
        triangle->GetPoints ()->GetPoint (0, p0);
        triangle->GetPoints ()->GetPoint (1, p1);
        triangle->GetPoints ()->GetPoint (2, p2);
        visible_area += vtkTriangle::TriangleArea (p0, p1, p2);
      }

#else 
      vtkSmartPointer<vtkHardwareSelector> hardware_selector = vtkSmartPointer<vtkHardwareSelector>::New ();
      hardware_selector->ClearBuffers();
      vtkSmartPointer<vtkSelection> hdw_selection = vtkSmartPointer<vtkSelection>::New ();
      hardware_selector->SetRenderer (renderer);
      hardware_selector->SetArea (0, 0, resolution_ - 1, resolution_ - 1);
      hardware_selector->SetFieldAssociation(vtkDataObject::FIELD_ASSOCIATION_CELLS);
      hdw_selection = hardware_selector->Select ();
      vtkSmartPointer<vtkIdTypeArray> ids = vtkSmartPointer<vtkIdTypeArray>::New ();
      ids = vtkIdTypeArray::SafeDownCast(hdw_selection->GetNode(0)->GetSelectionList());
      double visible_area = 0;
      for (int sel_id = 0; sel_id < (ids->GetNumberOfTuples ()); sel_id++)
      {
        int id_mesh = int (ids->GetValue (sel_id));
        if(id_mesh >= polydata->GetNumberOfPolys())
          continue;

        vtkCell * cell = polydata->GetCell (id_mesh);
        vtkTriangle* triangle = dynamic_cast<vtkTriangle*> (cell);
        double p0[3];
        double p1[3];
        double p2[3];
        triangle->GetPoints ()->GetPoint (0, p0);
        triangle->GetPoints ()->GetPoint (1, p1);
        triangle->GetPoints ()->GetPoint (2, p2);
        area = vtkTriangle::TriangleArea (p0, p1, p2);
        visible_area += area;
      }
#endif

      entropies_.push_back (float (visible_area / totalArea));
    }

    //transform cloud to give camera coordinates instead of world coordinates!
    vtkSmartPointer<vtkMatrix4x4> view_transform = cam_tmp->GetViewTransformMatrix ();
    Eigen::Matrix4f trans_view;
    trans_view.setIdentity ();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        trans_view (x, y) = float (view_transform->GetElement (x, y));

    //NOTE: vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right)
    //thus, the fliping in y and z
    for (size_t i = 0; i < cloud->points.size (); i++)
    {
      cloud->points[i].getVector4fMap () = trans_view * cloud->points[i].getVector4fMap ();
      cloud->points[i].y *= -1.0f;
      cloud->points[i].z *= -1.0f;
    }

    renderer->RemoveActor (actor_view);

    generated_views_.push_back (cloud);

    //create pose, from OBJECT coordinates to CAMERA coordinates!
    vtkSmartPointer<vtkTransform> transOCtoCC = vtkSmartPointer<vtkTransform>::New ();
    transOCtoCC->PostMultiply ();
    transOCtoCC->Identity ();
    transOCtoCC->Concatenate (matrixCenter);
    transOCtoCC->Concatenate (matrixRotModel);
    transOCtoCC->Concatenate (matrixTranslation);
    transOCtoCC->Concatenate (cam_tmp->GetViewTransformMatrix ());

    //NOTE: vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right)
    //thus, the fliping in y and z
    vtkSmartPointer<vtkMatrix4x4> cameraSTD = vtkSmartPointer<vtkMatrix4x4>::New ();
    cameraSTD->Identity ();
    cameraSTD->SetElement (0, 0, 1);
    cameraSTD->SetElement (1, 1, -1);
    cameraSTD->SetElement (2, 2, -1);

    transOCtoCC->Concatenate (cameraSTD);
    transOCtoCC->Modified ();

    Eigen::Matrix4f pose_view;
    pose_view.setIdentity ();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        pose_view (x, y) = float (transOCtoCC->GetMatrix ()->GetElement (x, y));

    poses_.push_back (pose_view);

  }
}
