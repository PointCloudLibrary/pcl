/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
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

#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>
#include <pcl/console/print.h>
#include <pcl/common/colors.h>
#include <stdlib.h>

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::getRandomColors (double &r, double &g, double &b, double min, double max)
{
  double sum;
  static unsigned stepRGBA = 100;
  do
  {
    sum = 0;
    r = (rand () % stepRGBA) / static_cast<double> (stepRGBA);
    while ((g = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) {}
    while (((b = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) && (b == g)) {}
    sum = r + g + b;
  }
  while (sum <= min || sum >= max);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::getRandomColors (pcl::RGB &rgb, double min, double max)
{
  double sum;
  static unsigned stepRGBA = 100;
  double r, g, b;
  do
  {
    sum = 0;
    r = (rand () % stepRGBA) / static_cast<double> (stepRGBA);
    while ((g = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) {}
    while (((b = (rand () % stepRGBA) / static_cast<double> (stepRGBA)) == r) && (b == g)) {}
    sum = r + g + b;
  }
  while (sum <= min || sum >= max);
  rgb.r = uint8_t (r * 255.0);
  rgb.g = uint8_t (g * 255.0);
  rgb.b = uint8_t (b * 255.0);
}

/////////////////////////////////////////////////////////////////////////////////////////////

Eigen::Matrix4d
pcl::visualization::vtkToEigen (vtkMatrix4x4* vtk_matrix)
{
  Eigen::Matrix4d eigen_matrix = Eigen::Matrix4d::Identity ();
  for (int i=0; i < 4; i++)
  {
    for (int j=0; j < 4; j++)
    {
      // VTK
      eigen_matrix (i, j) = vtk_matrix->GetElement (i, j);
    }
  }
  return eigen_matrix;
}

/////////////////////////////////////////////////////////////////////////////////////////////
Eigen::Vector2i
pcl::visualization::worldToView (const Eigen::Vector4d &world_pt, const Eigen::Matrix4d &view_projection_matrix, int width, int height)
{
  // Transform world to clipping coordinates
  Eigen::Vector4d world (view_projection_matrix * world_pt);
  // Normalize w-component
  world /= world.w ();

  // X/Y screen space coordinate
  int screen_x = int (floor (double (((world.x () + 1) / 2.0) * width) + 0.5));
  int screen_y = int (floor (double (((world.y () + 1) / 2.0) * height) + 0.5));

  // Calculate -world_pt.y () because the screen Y axis is oriented top->down, ie 0 is top-left
  //int winY = (int) floor ( (double) (((1 - world_pt.y ()) / 2.0) * height) + 0.5); // top left

  return (Eigen::Vector2i (screen_x, screen_y));
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::getViewFrustum (const Eigen::Matrix4d &view_projection_matrix, double planes[24])
{
  // Set up the normals
  Eigen::Vector4d normals[6];
  for (int i=0; i < 6; i++)
  {
    normals[i] = Eigen::Vector4d (0.0, 0.0, 0.0, 1.0);

    // if i is even set to -1, if odd set to +1
    normals[i] (i/2) = 1 - (i%2)*2;
  }

  // Transpose the matrix for use with normals
  Eigen::Matrix4d view_matrix = view_projection_matrix.transpose ();

  // Transform the normals to world coordinates
  for (int i=0; i < 6; i++)
  {
    normals[i] = view_matrix * normals[i];

    double f = 1.0/sqrt (normals[i].x () * normals[i].x () +
                         normals[i].y () * normals[i].y () +
                         normals[i].z () * normals[i].z ());

    planes[4*i + 0] = normals[i].x ()*f;
    planes[4*i + 1] = normals[i].y ()*f;
    planes[4*i + 2] = normals[i].z ()*f;
    planes[4*i + 3] = normals[i].w ()*f;
  }
}

int
pcl::visualization::cullFrustum (double frustum[24], const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb)
{
  int result = PCL_INSIDE_FRUSTUM;

  for(int i =0; i < 6; i++){
    double a = frustum[(i*4)];
    double b = frustum[(i*4)+1];
    double c = frustum[(i*4)+2];
    double d = frustum[(i*4)+3];

    //cout << i << ": " << a << "x + " << b << "y + " << c << "z + " << d << endl;

    //  Basic VFC algorithm
    Eigen::Vector3d center ((max_bb.x () - min_bb.x ()) / 2 + min_bb.x (),
                            (max_bb.y () - min_bb.y ()) / 2 + min_bb.y (),
                            (max_bb.z () - min_bb.z ()) / 2 + min_bb.z ());

    Eigen::Vector3d radius (fabs (static_cast<double> (max_bb.x () - center.x ())),
                            fabs (static_cast<double> (max_bb.y () - center.y ())),
                            fabs (static_cast<double> (max_bb.z () - center.z ())));

    double m = (center.x () * a) + (center.y () * b) + (center.z () * c) + d;
    double n = (radius.x () * fabs(a)) + (radius.y () * fabs(b)) + (radius.z () * fabs(c));

    if (m + n < 0){
      result = PCL_OUTSIDE_FRUSTUM;
      break;
    }

    if (m - n < 0)
    {
      result = PCL_INTERSECT_FRUSTUM;
    }
  }

  return result;
}

//void
//pcl::visualization::getModelViewPosition (Eigen::Matrix4d model_view_matrix, Eigen::Vector3d &position)
//{
//  //Compute eye or position from model view matrix
//  Eigen::Matrix4d inverse_model_view_matrix = model_view_matrix.inverse();
//  for (int i=0; i < 3; i++)
//  {
//    position(i) = inverse_model_view_matrix(i, 3);
//  }
//}

// Lookup table of max 6 bounding box vertices, followed by number of vertices, ie {v0, v1, v2, v3, v4, v5, nv}
//
//      3--------2
//     /|       /|       Y      0 = xmin, ymin, zmin
//    / |      / |       |      6 = xmax, ymax. zmax
//   7--------6  |       |
//   |  |     |  |       |
//   |  0-----|--1       +------X
//   | /      | /       /
//   |/       |/       /
//   4--------5       Z

int hull_vertex_table[43][7] = {
  { 0, 0, 0, 0, 0, 0, 0 }, // inside
  { 0, 4, 7, 3, 0, 0, 4 }, // left
  { 1, 2, 6, 5, 0, 0, 4 }, // right
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 5, 4, 0, 0, 4 }, // bottom
  { 0, 1, 5, 4, 7, 3, 6 }, // bottom, left
  { 0, 1, 2, 6, 5, 4, 6 }, // bottom, right
  { 0, 0, 0, 0, 0, 0, 0 },
  { 2, 3, 7, 6, 0, 0, 4 }, // top
  { 4, 7, 6, 2, 3, 0, 6 }, // top, left
  { 2, 3, 7, 6, 5, 1, 6 }, // top, right
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 3, 2, 1, 0, 0, 4 }, // front
  { 0, 4, 7, 3, 2, 1, 6 }, // front, left
  { 0, 3, 2, 6, 5, 1, 6 }, // front, right
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 3, 2, 1, 5, 4, 6 }, // front, bottom
  { 2, 1, 5, 4, 7, 3, 6 }, // front, bottom, left
  { 0, 3, 2, 6, 5, 4, 6 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 3, 7, 6, 2, 1, 6 }, // front, top
  { 0, 4, 7, 6, 2, 1, 6 }, // front, top, left
  { 0, 3, 7, 6, 5, 1, 6 }, // front, top, right
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0 },
  { 4, 5, 6, 7, 0, 0, 4 }, // back
  { 4, 5, 6, 7, 3, 0, 6 }, // back, left
  { 1, 2, 6, 7, 4, 5, 6 }, // back, right
  { 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 5, 6, 7, 4, 6 }, // back, bottom
  { 0, 1, 5, 6, 7, 3, 6 }, // back, bottom, left
  { 0, 1, 2, 6, 7, 4, 6 }, // back, bottom, right
  { 0, 0, 0, 0, 0, 0, 0 },
  { 2, 3, 7, 4, 5, 6, 6 }, // back, top
  { 0, 4, 5, 6, 2, 3, 6 }, // back, top, left
  { 1, 2, 3, 7, 4, 5, 6 }  // back, top, right
};

/////////////////////////////////////////////////////////////////////////////////////////////
float
pcl::visualization::viewScreenArea (
    const Eigen::Vector3d &eye, 
    const Eigen::Vector3d &min_bb, const Eigen::Vector3d &max_bb, 
    const Eigen::Matrix4d &view_projection_matrix, int width, int height)
{
  Eigen::Vector4d bounding_box[8];
  bounding_box[0] = Eigen::Vector4d(min_bb.x (), min_bb.y (), min_bb.z (), 1.0);
  bounding_box[1] = Eigen::Vector4d(max_bb.x (), min_bb.y (), min_bb.z (), 1.0);
  bounding_box[2] = Eigen::Vector4d(max_bb.x (), max_bb.y (), min_bb.z (), 1.0);
  bounding_box[3] = Eigen::Vector4d(min_bb.x (), max_bb.y (), min_bb.z (), 1.0);
  bounding_box[4] = Eigen::Vector4d(min_bb.x (), min_bb.y (), max_bb.z (), 1.0);
  bounding_box[5] = Eigen::Vector4d(max_bb.x (), min_bb.y (), max_bb.z (), 1.0);
  bounding_box[6] = Eigen::Vector4d(max_bb.x (), max_bb.y (), max_bb.z (), 1.0);
  bounding_box[7] = Eigen::Vector4d(min_bb.x (), max_bb.y (), max_bb.z (), 1.0);

  // Compute 6-bit code to classify eye with respect to the 6 defining planes
  int pos = ((eye.x () < bounding_box[0].x ()) )  // 1 = left
      + ((eye.x () > bounding_box[6].x ()) << 1)  // 2 = right
      + ((eye.y () < bounding_box[0].y ()) << 2)  // 4 = bottom
      + ((eye.y () > bounding_box[6].y ()) << 3)  // 8 = top
      + ((eye.z () < bounding_box[0].z ()) << 4)  // 16 = front
      + ((eye.z () > bounding_box[6].z ()) << 5); // 32 = back

  // Look up number of vertices
  int num = hull_vertex_table[pos][6];
  if (num == 0)
  {
    return (float (width * height));
  }
    //return 0.0;


//  cout << "eye: " << eye.x() << " " << eye.y() << " " << eye.z() << endl;
//  cout << "min: " << bounding_box[0].x() << " " << bounding_box[0].y() << " " << bounding_box[0].z() << endl;
//
//  cout << "pos: " << pos << " ";
//  switch(pos){
//    case 0:  cout << "inside" << endl; break;
//    case 1:  cout << "left" << endl; break;
//    case 2:  cout << "right" << endl; break;
//    case 3:
//    case 4:  cout << "bottom" << endl; break;
//    case 5:  cout << "bottom, left" << endl; break;
//    case 6:  cout << "bottom, right" << endl; break;
//    case 7:
//    case 8:  cout << "top" << endl; break;
//    case 9:  cout << "top, left" << endl; break;
//    case 10:  cout << "top, right" << endl; break;
//    case 11:
//    case 12:
//    case 13:
//    case 14:
//    case 15:
//    case 16:  cout << "front" << endl; break;
//    case 17:  cout << "front, left" << endl; break;
//    case 18:  cout << "front, right" << endl; break;
//    case 19:
//    case 20:  cout << "front, bottom" << endl; break;
//    case 21:  cout << "front, bottom, left" << endl; break;
//    case 22:
//    case 23:
//    case 24:  cout << "front, top" << endl; break;
//    case 25:  cout << "front, top, left" << endl; break;
//    case 26:  cout << "front, top, right" << endl; break;
//    case 27:
//    case 28:
//    case 29:
//    case 30:
//    case 31:
//    case 32:  cout << "back" << endl; break;
//    case 33:  cout << "back, left" << endl; break;
//    case 34:  cout << "back, right" << endl; break;
//    case 35:
//    case 36:  cout << "back, bottom" << endl; break;
//    case 37:  cout << "back, bottom, left" << endl; break;
//    case 38:  cout << "back, bottom, right" << endl; break;
//    case 39:
//    case 40:  cout << "back, top" << endl; break;
//    case 41:  cout << "back, top, left" << endl; break;
//    case 42:  cout << "back, top, right" << endl; break;
//  }

  //return -1 if inside
  Eigen::Vector2d dst[8];
  for (int i = 0; i < num; i++)
  {
    Eigen::Vector4d world_pt = bounding_box[hull_vertex_table[pos][i]];
    Eigen::Vector2i screen_pt = pcl::visualization::worldToView(world_pt, view_projection_matrix, width, height);
//    cout << "point[" << i << "]: " << screen_pt.x() << " " << screen_pt.y() << endl;
    dst[i] = Eigen::Vector2d(screen_pt.x (), screen_pt.y ());
  }

  double sum = 0.0;
  for (int i = 0; i < num; ++i)
  {
    sum += (dst[i].x () - dst[(i+1) % num].x ()) * (dst[i].y () + dst[(i+1) % num].y ());
  }

  return (fabsf (float (sum * 0.5f)));
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::visualization::getColormapLUT (LookUpTableRepresentationProperties colormap_type, vtkSmartPointer<vtkLookupTable> &table)
{
  table = vtkSmartPointer<vtkLookupTable>::New ();
  switch (colormap_type)
  {
    case PCL_VISUALIZER_LUT_JET:
    {
      table->SetHueRange (0, 0.667);
      table->SetSaturationRange (1, 1);
      table->SetAlphaRange (1, 1);
      break;
    }
      
    case PCL_VISUALIZER_LUT_JET_INVERSE:
    {
      table->SetHueRange (0.667, 0);
      table->SetSaturationRange (1, 1);
      table->SetAlphaRange (1, 1);
      break;
    }
      
    case PCL_VISUALIZER_LUT_HSV:
    {
      table->SetHueRange (0, 1);
      table->SetSaturationRange (1, 1);
      table->SetAlphaRange (1, 1);
      break;
    }
      
    case PCL_VISUALIZER_LUT_HSV_INVERSE:
    {
      table->SetHueRange (1, 0);
      table->SetSaturationRange (1, 1);
      table->SetAlphaRange (1, 1);
      break;
    }
      
    case PCL_VISUALIZER_LUT_GREY:
    {
      table->SetValueRange (0, 1);
      table->SetHueRange (0, 0);
      table->SetSaturationRange (0, 0);
      table->SetAlphaRange (1, 1);
      break;
    }
      
    case PCL_VISUALIZER_LUT_BLUE2RED:
    {
      table->SetSaturationRange (1, 1);
      table->SetAlphaRange (1, 1);
      table->SetNumberOfTableValues (256);

      double red[3]   = {1.0, 0.0, 0.0};
      double white[3] = {1.0, 1.0, 1.0};
      double blue[3]  = {0.0, 0.0, 1.0};
      
      for (size_t i = 0; i < 128; i++)
      {
        double weight = static_cast<double>(i) / 128.0;
        table->SetTableValue  ( i,
                                white[0] * weight + blue[0] * (1 - weight),
                                white[1] * weight + blue[1] * (1 - weight),
                                white[2] * weight + blue[2] * (1 - weight)  );
      }
      
      for (size_t i = 128; i < 256; i++)
      {
        double weight = (static_cast<double>(i) -128.0) / 128.0;
        table->SetTableValue  ( i,
                                red[0] * weight + white[0] * (1 - weight),
                                red[1] * weight + white[1] * (1 - weight),
                                red[2] * weight + white[2] * (1 - weight)  );
      }
      break;
    }

    case PCL_VISUALIZER_LUT_VIRIDIS:
    {
      table->SetSaturationRange (1, 1);
      table->SetAlphaRange (1, 1);
      table->SetNumberOfTableValues (pcl::ViridisLUT::size ());
      for (size_t i = 0; i < pcl::ViridisLUT::size (); i++)
      {
        pcl::RGB c = pcl::ViridisLUT::at (i);
        table->SetTableValue (i, static_cast<double> (c.r) / 255.0,
                                 static_cast<double> (c.g) / 255.0,
                                 static_cast<double> (c.b) / 255.0);
      }
      break;
    }

    default:
      PCL_WARN ("[pcl::visualization::getColormapLUT] Requested colormap type does not exist!\n");
      return false;
  }
  table->Build ();
  return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::visualization::Camera::computeViewMatrix (Eigen::Matrix4d &view_mat) const
{
//constructs view matrix from camera pos, view up, and the point it is looking at
//this code is based off of gluLookAt http://www.opengl.org/wiki/GluLookAt_code
	Eigen::Vector3d focal_point (focal[0], focal[1], focal[2]);
	Eigen::Vector3d posv        (pos[0]  , pos[1]  , pos[2]);
	Eigen::Vector3d up          (view[0] , view[1] , view[2]);

	Eigen::Vector3d zAxis = (focal_point - posv).normalized();
  Eigen::Vector3d xAxis = zAxis.cross(up).normalized();
  // make sure the y-axis is orthogonal to the other two
  Eigen::Vector3d yAxis = xAxis.cross (zAxis);

	view_mat.block <1, 3> (0, 0) = xAxis;
	view_mat.block <1, 3> (1, 0) = yAxis;
	view_mat.block <1, 3> (2, 0) = -zAxis;
	view_mat.row (3) << 0, 0, 0, 1;

	view_mat.block <3, 1> (0, 3) = view_mat.topLeftCorner<3, 3> () * (-posv);
}

///////////////////////////////////////////////////////////////////////
void
pcl::visualization::Camera::computeProjectionMatrix (Eigen::Matrix4d& proj) const
{
  float top    = static_cast<float> (clip[0]) * tanf (0.5f * static_cast<float> (fovy));
  float left   = -top * static_cast<float> (window_size[0] / window_size[1]);
  float right  = -left;
  float bottom = -top;

  float temp1, temp2, temp3, temp4;
	temp1 = 2.0f * static_cast<float> (clip[0]);
	temp2 = 1.0f / (right - left);
	temp3 = 1.0f / (top - bottom);
	temp4 = 1.0f / static_cast<float> (clip[1] - clip[0]);

  proj.setZero ();

	proj(0,0) = temp1 * temp2;
	proj(1,1) = temp1 * temp3;
	proj(0,2) = (right + left) * temp2;
	proj(1,2) = (top + bottom) * temp3;
	proj(2,2) = (-clip[1] - clip[0]) * temp4;
	proj(3,2) = -1.0;
	proj(2,3) = (-temp1 * clip[1]) * temp4;
}
