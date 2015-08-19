/*
 * Software License Agreement (BSD License)
 *
 * Point Cloud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2014-, Open Perception, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the copyright holder(s) nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
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
 */

#include <pcl/simulation/shape_generator_shapes.h>
typedef pcl::simulation::GeometricShapeBase::PointCloudT PointCloudT;


void
pcl::simulation::ConvexPolygon::Face::sampleOnFace (float resolution, 
                                                    PointCloudT::Ptr cloud_ptr, 
                                                    int label)
{
  std::size_t num_triangles = triangle_areas_.size ();
  // sample points for each triangle in the face
  for (std::size_t ctriangle = 0; ctriangle < num_triangles; ++ctriangle)
  {
    unsigned int num_points = parent_->calculateNumberOfPoints (triangle_areas_[ctriangle], resolution);
    for (unsigned int ci = 0; ci < num_points; ++ci)
    {
      float a, b;
      a = parent_->dist01_ (parent_->gen_);
      b = parent_->dist01_ (parent_->gen_);
      PointT new_point;
      const Eigen::Vector3f p1 = parent_->vertices_[vertex_list_[0]];
      const Eigen::Vector3f p12 = parent_->vertices_[vertex_list_[ctriangle + 1]] - parent_->vertices_[vertex_list_[0]];
      const Eigen::Vector3f p13 = parent_->vertices_[vertex_list_[ctriangle + 2]] - parent_->vertices_[vertex_list_[0]];
      new_point.getNormalVector3fMap () = normal_;
      if (a + b <= 1)
        new_point.getVector3fMap () = p1 + a * p12 + b * p13;
      else
        new_point.getVector3fMap () = p1 + (1 - a) * p12 + (1 - b) * p13;
      new_point.label = label;
      cloud_ptr->push_back (new_point);
    }
  }
}

void
pcl::simulation::ConvexPolygon::setPolygons (const VerticeVectorT &vertices_vector,
                                             const FaceInformationT &face_information)
{
  area_ = 0;
  vertices_ = vertices_vector;
  for (std::size_t cface = 0; cface < face_information.size (); ++cface)
  {
    Face::Ptr new_face (new Face (this));
    new_face->vertex_list_ = face_information[cface];

    bool normal_calculated = false;
    Eigen::Vector3f old_normal;
    std::size_t num_triangles = new_face->vertex_list_.size () - 2;

    if (num_triangles < 1)
      PCL_ERROR ("ERROR ConvexPolygon::ConvexPolygon: Cannot add face number %d, because it needs at least 3 vertices.\n", cface + 1);

    new_face->triangle_areas_.resize (num_triangles);
    // use the first three nodes to calculate the normal, but check all nodes if the normals would differ. If that is the case the points do not lie in a common plane.
    for (std::size_t ctriangle = 0; ctriangle < num_triangles; ++ctriangle)
    {
      const Eigen::Vector3f p12 = vertices_[new_face->vertex_list_[ctriangle + 1]] - vertices_[new_face->vertex_list_[0]];
      const Eigen::Vector3f p13 = vertices_[new_face->vertex_list_[ctriangle + 2]] - vertices_[new_face->vertex_list_[0]];

      Eigen::Vector3f cross_vec = p12.cross (p13);
      float current_triangle_area = cross_vec.norm () / 2;
      new_face->triangle_areas_[ctriangle] = current_triangle_area;
      new_face->face_area_ += current_triangle_area;
      Eigen::Vector3f new_normal = cross_vec.normalized ();
      area_ += current_triangle_area;
      // normals for the whole planar patch should be the same. This performs a simple check.
      if (normal_calculated)
      {
        float dot_normal = new_normal.dot (old_normal);
        if (dot_normal < 0.99)
          PCL_ERROR ("ERROR ConvexPolygon::ConvexPolygon: Cannot add face number %d, because it is not a planar and convex face.\n", cface + 1);
      }
      else
      {
        normal_calculated = true;
        new_face->normal_ = new_normal;
      }
      old_normal = new_normal;
    }
    faces_.push_back (new_face);
  }
  faces_set_ = true;
}

PointCloudT::Ptr
pcl::simulation::ConvexPolygon::generate (float resolution)
{
  PointCloudT::Ptr cloud_ptr (new PointCloudT ());
  if (!faces_set_)
  {
    PCL_WARN ("ConvexPolygon::generate: Please run setPolygons before calling generate\n");
    return (cloud_ptr);
  }
  
  for (std::size_t cf = 0; cf < faces_.size (); ++cf)
  {
    faces_[cf]->sampleOnFace (resolution, cloud_ptr);
  }
  // apply transformation which may have been set before generate was called
  applyTransformations (cloud_ptr);
  return (cloud_ptr);
}

bool
pcl::simulation::ConvexPolygon::isInside (const PointT &point) const
{
  for (std::size_t cf = 0; cf < faces_.size (); ++cf)
  {
    // skip empty triangles
    if (faces_[cf]->face_area_ == 0)
      continue;
    if (!faces_[cf]->isInside (point))
      return (false);
  }
  return (true);
}

PointCloudT::Ptr
pcl::simulation::Sphere::generate (float resolution)
{
  PointCloudT::Ptr cloud_ptr (new PointCloudT ());
  float area = 4 * M_PI * pow (radius_, 2);
  unsigned int num_points = calculateNumberOfPoints (area, resolution);
  while (cloud_ptr->size () < num_points)
  {
    float x (-1 + 2 * dist01_ (gen_)), y (-1 + 2 * dist01_ (gen_)), z (-1 + 2 * dist01_ (gen_));
    if (x * x + y * y + z * z <= 1)
    {
      float theta = atan2 (sqrt (x * x + y * y), z);
      float phi = atan2 (y, x);
      PointT p;
      p.x = radius_ * cos (phi) * sin (theta);
      p.y = radius_ * sin (phi) * sin (theta);
      p.z = radius_ * cos (theta);
      p.normal_x = cos (phi) * sin (theta);
      p.normal_y = sin (phi) * sin (theta);
      p.normal_z = cos (theta);
      cloud_ptr->push_back (p);
    }
  }
  // apply transformation which may have been set before generate was called
  applyTransformations (cloud_ptr);
  return (cloud_ptr);
}

bool
pcl::simulation::Sphere::isInside (const PointT &point) const
{
  return (point.x * point.x + point.y * point.y + point.z * point.z < radius_ * radius_);
}

void
pcl::simulation::Cylinder::addCylinderMiddle (float resolution, 
                                              PointCloudT::Ptr cloud_ptr)
{
  float circum = 2 * M_PI * radius_;
  float area = circum * height_;
  unsigned int  num_points = calculateNumberOfPoints (area, resolution);
  for (unsigned int  ci = 0; ci < num_points; ++ci)
  {
    PointT p;
    p.y = (dist01_ (gen_) * height_) - (height_ / 2);
    float phi = dist01_ (gen_) * 2 * M_PI;
    p.x = cos (phi) * radius_;
    p.z = sin (phi) * radius_;
    p.normal_x = cos (phi);
    p.normal_z = sin (phi);
    p.normal_y = 0;
    cloud_ptr->push_back (p);
  }
}

void
pcl::simulation::Cylinder::addCylinderTopAndBottom (float resolution, 
                                                    PointCloudT::Ptr cloud_ptr)
{
  float area = 2 * (M_PI * radius_ * radius_);
  unsigned int  num_points = calculateNumberOfPoints (area, resolution);
  for (unsigned int  ci = 0; ci < num_points;)
  {
    float z = (2 * dist01_ (gen_) * radius_) - (radius_);
    float x = (2 * dist01_ (gen_) * radius_) - (radius_);
    if (z * z + x * x <= radius_ * radius_)
    {
      bool top = dist01_ (gen_) > 0.5;
      float y = height_ / 2 * (top ? 1 : -1);
      PointT p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.normal_x = 0;
      p.normal_z = 0;
      p.normal_y = (top ? 1 : -1);
      ++ci;
      cloud_ptr->push_back (p);
    }
  }
}

PointCloudT::Ptr
pcl::simulation::Cylinder::generate (float resolution)
{
  PointCloudT::Ptr cloud_ptr (new PointCloudT ());

  addCylinderMiddle (resolution, cloud_ptr);
  addCylinderTopAndBottom (resolution, cloud_ptr);

  // apply transformation which may have been set before generate was called
  applyTransformations (cloud_ptr);
  return (cloud_ptr);
}

bool
pcl::simulation::Cylinder::isInside (const PointT &point) const
{
  return ((point.x * point.x + point.z * point.z < radius_ * radius_) && (point.y < height_ / 2) && (point.y > -height_ / 2));
}

pcl::simulation::Wedge::Wedge (float width,
                               float depth,
                               float upwidth,
                               float updepth,
                               float height)
{
  VerticeVectorT vertices;
  FaceInformationT faces;
  vertices.resize (8);

  vertices[FrontBottomLeft] = Eigen::Vector3f (0, 0, 0);
  vertices[FrontBottomRight] = Eigen::Vector3f (width, 0, 0);
  vertices[BackBottomLeft] = Eigen::Vector3f (0, 0, -depth);
  vertices[BackBottomRight] = Eigen::Vector3f (width, 0, -depth);

  float delx = (width - upwidth) / 2;
  float delz = - (depth - updepth) / 2;
  vertices[FrontTopLeft] = Eigen::Vector3f (delx, height, delz);
  vertices[FrontTopRight] = Eigen::Vector3f (delx + upwidth, height, delz);
  vertices[BackTopLeft] = Eigen::Vector3f (delx, height, delz - updepth);
  vertices[BackTopRight] = Eigen::Vector3f (delx + upwidth, height, delz - updepth);

  faces.resize (6);
  faces[0] = boost::assign::list_of (FrontBottomLeft) (BackBottomLeft) (BackBottomRight) (FrontBottomRight);
  faces[1] = boost::assign::list_of (FrontTopLeft) (FrontTopRight) (BackTopRight) (BackTopLeft);
  faces[2] = boost::assign::list_of (FrontTopRight) (FrontTopLeft) (FrontBottomLeft) (FrontBottomRight);
  faces[3] = boost::assign::list_of (BackTopRight) (BackBottomRight) (BackBottomLeft) (BackTopLeft);
  faces[4] = boost::assign::list_of (FrontTopLeft) (BackTopLeft) (BackBottomLeft) (FrontBottomLeft);
  faces[5] = boost::assign::list_of (FrontTopRight) (FrontBottomRight) (BackBottomRight) (BackTopRight);

  // set centroid to the center of the shape and center the shape to the coordinate system
  centroid_ = Eigen::Vector3f (width / 2, height / 2, -depth / 2);
  center ();

  setPolygons (vertices, faces);
}

PointCloudT::Ptr
pcl::simulation::Cone::generate (float resolution)
{
  PointCloudT::Ptr cloud_ptr (new PointCloudT ());

  boost::random::triangle_distribution<float> height_distribution (0, 0, height_);
  float l = sqrt (radius_ * radius_ + height_ * height_);
  float area = M_PI * radius_ * l;
  unsigned int num_points = calculateNumberOfPoints (area, resolution);
  float cosalpha = height_ / l;
  float sinalpha = radius_ / l;
  for (unsigned int  ci = 0; ci < num_points; ++ci)
  {
    float h = height_distribution (gen_);
    float phi = dist01_ (gen_) * 2 * M_PI;
    PointT p;
    p.y = h;
    p.x = cos (phi) * (1 - h / height_) * radius_;
    p.z = sin (phi) * (1 - h / height_) * radius_;
    p.normal_y = sinalpha;
    p.normal_x = cosalpha * cos (phi);
    p.normal_z = cosalpha * sin (phi);
    cloud_ptr->push_back (p);
  }

  // Bottom
  area = M_PI * radius_ * radius_;
  num_points = calculateNumberOfPoints (area, resolution);
  for (unsigned int  ci = 0; ci < num_points;)
  {
    float y = 0;
    float x = (2 * dist01_ (gen_) * radius_) - radius_;
    float z = (2 * dist01_ (gen_) * radius_) - radius_;
    if (z * z + x * x < radius_ * radius_)
    {
      PointT p;
      p.x = x;
      p.y = y;
      p.z = z;
      p.normal_x = 0;
      p.normal_y = -1;
      p.normal_z = 0;
      cloud_ptr->push_back (p);
      ++ci;
    }
  }

  // The center of the sampling is a (0, height_ / 2, 0)
  // This will effectively center the cone at 0 0 0
  effective_transform_ = effective_transform_ * Eigen::Translation<float, 3> (0, -height_ / 2, 0);
// apply transformation which may have been set before generate was called
  applyTransformations (cloud_ptr);
  return (cloud_ptr);
}

bool
pcl::simulation::Cone::isInside (const PointT &point) const
{
  bool inside = true;

  // check for correct height (y-value)
  inside &= (point.y > 0);
  inside &= (point.y < height_);
  if (!inside)
    return (false);

  float radius_of_h = (1 - point.y / height_) * radius_;
  inside &= (point.x * point.x + point.z * point.z < radius_of_h * radius_of_h);
  return inside;
}

pcl::simulation::Torus::Torus (float R,
                               float r) :
    is_full_ (true),
    R_ (R),
    r_ (r),
    min_theta_ (0),
    max_theta_ (2 * M_PI)
{
}

pcl::simulation::Torus::Torus (float R,
                               float r,
                               float min_theta,
                               float max_theta) :
    is_full_ (false),
    R_ (R),
    r_ (r),
    min_theta_ (min_theta),
    max_theta_ (max_theta)
{
}

PointCloudT::Ptr
pcl::simulation::Torus::generate (float resolution)
{
  PointCloudT::Ptr cloud_ptr (new PointCloudT ());
  if (max_theta_ < min_theta_)
    max_theta_ += 2 * M_PI;

  float area = 4 * M_PI * M_PI * R_ * r_ * (max_theta_ - min_theta_) / (2 * M_PI);
  unsigned int num_points = calculateNumberOfPoints (area, resolution);

  // initialze the random number generator to generate equidensity points on the torus (theta can be drawn uniformly, phi has to be weighted by the number of points on the circumference of the torus (distance from the center (R+r cos (phi) )
  const unsigned int  NUM_SEGMENTS = 50;
  std::vector<float> intervals, weights;
  intervals.resize (NUM_SEGMENTS + 1);
  weights.resize (NUM_SEGMENTS + 1);
  for (unsigned int  cs = 0; cs <= NUM_SEGMENTS; ++cs)
  {
    float segment_position = 2 * M_PI / NUM_SEGMENTS * cs;
    intervals[cs] = segment_position;
    weights[cs] = std::cos (segment_position) * r_ + R_;
  }
  boost::random::piecewise_linear_distribution<float> phi_dist (intervals.begin (), intervals.end (), weights.begin ());
  for (unsigned int  ci = 0; ci < num_points; ++ci)
  {
    PointT p;
    float phi = phi_dist (gen_);
    float theta = min_theta_ + dist01_ (gen_) * (max_theta_ - min_theta_);
    p.x = (R_ + r_ * cos (phi)) * cos (theta);
    p.z = (R_ + r_ * cos (phi)) * sin (theta);
    p.y = r_ * sin (phi);
    p.normal_x = cos (phi) * cos (theta);
    p.normal_y = sin (phi);
    p.normal_z = cos (phi) * sin (theta);
    cloud_ptr->push_back (p);
  }

  // if the Torus is not full, generate also the surfaces on the sides
  if (!is_full_)
  {
    // probability distribution of the drawn r scales with the radius
    std::vector<float> intervals = boost::assign::list_of (0.0) (r_);
    std::vector<float> weights = boost::assign::list_of (0.0) (r_);
    boost::random::piecewise_linear_distribution<float> radius_distribution (intervals.begin (), intervals.end (), weights.begin ());
    area = 2 * M_PI * r_ * r_;
    num_points = calculateNumberOfPoints (area, resolution);
    float cosmin_theta_ = cos (min_theta_);
    float cosmax_theta_ = cos (max_theta_);
    float sinmin_theta_ = sin (min_theta_);
    float sinmax_theta_ = sin (max_theta_);
    for (unsigned int  ci = 0; ci < num_points; ++ci)
    {
      PointT p;
      float phi = dist01_ (gen_) * 2 * M_PI;
      float cos_theta, sin_theta;
      bool MinFace = dist01_ (gen_) > 0.5;
      if (MinFace)
      {
        cos_theta = cosmin_theta_;
        sin_theta = sinmin_theta_;
      }
      else
      {
        cos_theta = cosmax_theta_;
        sin_theta = sinmax_theta_;
      }
      float r = radius_distribution (gen_);
      p.x = (R_ + r * cos (phi)) * cos_theta;
      p.z = (R_ + r * cos (phi)) * sin_theta;
      p.y = r * sin (phi);
      p.normal_x = -sin_theta * (MinFace ? -1 : 1);
      p.normal_y = 0;
      p.normal_z = cos_theta * (MinFace ? -1 : 1);
      cloud_ptr->push_back (p);
    }
  }
  // apply transformation which may have been set before generate was called
  applyTransformations (cloud_ptr);
  return (cloud_ptr);
}

bool
pcl::simulation::Torus::isInside (const PointT &point) const
{
  // check if the heigth is within the maximum torus height (will negate most points)
  if (point.y > r_ || point.y < -r_)
    return (false);

  // check for right distance from the centroid (is it possible to be within the minor circle)
  float r_sq = point.x * point.x + point.z * point.z;
  if ( (r_sq < (R_ - r_) * (R_ - r_)) || (r_sq > ( (R_ + r_) * (R_ + r_))))
    return (false);

  // check theta range in case of not full Torus
  if (!is_full_)
  {
    float theta = atan2 (point.z, point.x);
    if (theta < min_theta_ || theta > max_theta_)
      return (false);
  }

  // check for correct height inside the circle
  r_sq = sqrt (r_sq);
  float y_max_sq = r_ * r_ - (R_ - r_sq) * (R_ - r_sq);

  if (point.y * point.y < y_max_sq)
    return (true);
  else
    return (false);
}
