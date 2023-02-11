/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 * 
 *
 */

#include <pcl/surface/on_nurbs/sequential_fitter.h>

#include <Eigen/Geometry> // for cross

using namespace pcl;
using namespace on_nurbs;
using namespace Eigen;

SequentialFitter::Parameter::Parameter (int order, int refinement, int iterationsQuad, int iterationsBoundary,
                                        int iterationsAdjust, int iterationsInterior, double forceBoundary,
                                        double forceBoundaryInside, double forceInterior, double stiffnessBoundary,
                                        double stiffnessInterior, int resolution)
{
  this->order = order;
  this->refinement = refinement;
  this->iterationsQuad = iterationsQuad;
  this->iterationsBoundary = iterationsBoundary;
  this->iterationsAdjust = iterationsAdjust;
  this->iterationsInterior = iterationsInterior;
  this->forceBoundary = forceBoundary;
  this->forceBoundaryInside = forceBoundaryInside;
  this->forceInterior = forceInterior;
  this->stiffnessBoundary = stiffnessBoundary;
  this->stiffnessInterior = stiffnessInterior;
  this->resolution = resolution;
}

SequentialFitter::SequentialFitter (Parameter p)
{
  this->m_params = p;
  this->m_have_cloud = false;
  this->m_have_corners = false;
  this->m_surf_id = -1;
}

void
SequentialFitter::compute_quadfit ()
{
  ON_NurbsSurface nurbs;

  if (m_have_corners)
  {
    nurbs = FittingSurface::initNurbs4Corners (2, m_corners[0], m_corners[1], m_corners[2], m_corners[3]);
  }
  else
  {
    nurbs = FittingSurface::initNurbsPCA (2, &m_data);
    nurbs.GetCV (0, 0, m_corners[0]);
    nurbs.GetCV (1, 0, m_corners[1]);
    nurbs.GetCV (1, 1, m_corners[2]);
    nurbs.GetCV (0, 1, m_corners[3]);
    Eigen::Vector3d v0 (m_corners[0].x, m_corners[0].y, m_corners[0].z);
    Eigen::Vector3d v1 (m_corners[1].x, m_corners[1].y, m_corners[1].z);
    Eigen::Vector3d v2 (m_corners[2].x, m_corners[2].y, m_corners[2].z);
    Eigen::Vector3d v3 (m_corners[3].x, m_corners[3].y, m_corners[3].z);
    if (is_back_facing (v0, v1, v2, v3))
    {
      ON_3dPoint tmp[4];
      tmp[0] = m_corners[0];
      tmp[1] = m_corners[1];
      tmp[2] = m_corners[2];
      tmp[3] = m_corners[3];
      m_corners[3] = tmp[0];
      m_corners[2] = tmp[1];
      m_corners[1] = tmp[2];
      m_corners[0] = tmp[3];
      nurbs = FittingSurface::initNurbs4Corners (2, m_corners[0], m_corners[1], m_corners[2], m_corners[3]);
    }
  }

  FittingSurface fitting (&m_data, nurbs);

  FittingSurface::Parameter paramFP (m_params.forceInterior, 1.0, 0.0, m_params.forceBoundary, 1.0, 0.0);

  // Quad fitting
  //  if( !m_quiet && m_dbgWin != NULL )
  //    NurbsConvertion::Nurbs2TomGine(m_dbgWin, *fitting->m_nurbs, m_surf_id, m_params.resolution);
  for (int r = 0; r < m_params.iterationsQuad; r++)
  {
    fitting.assemble (paramFP);
    fitting.solve ();
  }
  fitting.m_nurbs.GetCV (0, 0, m_corners[0]);
  fitting.m_nurbs.GetCV (1, 0, m_corners[1]);
  fitting.m_nurbs.GetCV (1, 1, m_corners[2]);
  fitting.m_nurbs.GetCV (0, 1, m_corners[3]);
}

void
SequentialFitter::compute_refinement (FittingSurface* fitting) const
{
  // Refinement
  FittingSurface::Parameter paramFP (0.0, 1.0, 0.0, m_params.forceBoundary, m_params.stiffnessBoundary, 0.0);

  for (int r = 0; r < m_params.refinement; r++)
  {
    fitting->assemble (paramFP);
    fitting->solve ();
    fitting->refine (0);
    fitting->refine (1);
  }
}

void
SequentialFitter::compute_boundary (FittingSurface* fitting) const
{
  FittingSurface::Parameter paramFP (0.0, 1.0, 0.0, m_params.forceBoundary, m_params.stiffnessBoundary, 0.0);

  // iterate boundary points
  for (int i = 0; i < m_params.iterationsBoundary; i++)
  {
    fitting->assemble (paramFP);
    fitting->solve ();
  }
}
void
SequentialFitter::compute_interior (FittingSurface* fitting) const
{
  // iterate interior points
  //std::vector<double> wInt(m_data.interior.PointCount(), m_params.forceInterior);
  FittingSurface::Parameter paramFP (m_params.forceInterior, m_params.stiffnessInterior, 0.0, m_params.forceBoundary,
                                     m_params.stiffnessBoundary, 0.0);

  for (int i = 0; i < m_params.iterationsInterior; i++)
  {
    fitting->assemble (paramFP);
    //    fitting->assemble(wBnd, wInt, m_params.stiffnessBoundary, m_params.stiffnessInterior);
    fitting->solve ();
  }
}

Eigen::Vector2d
SequentialFitter::project (const Eigen::Vector3d &pt)
{
  Eigen::Vector4d pr (m_intrinsic * m_extrinsic * Eigen::Vector4d (pt (0), pt (1), pt (2), 1.0));
  pr (0) = pr (0) / pr (2);
  pr (1) = pr (1) / pr (2);
  if (pt.dot (Eigen::Vector3d (m_extrinsic (0, 2), m_extrinsic (1, 2), m_extrinsic (2, 2))) < 0.0f)
  { // avoids backprojection
    pr (0) = -pr (0);
    pr (1) = -pr (1);
  }
  return {pr(0), pr(1)};
}

bool
SequentialFitter::is_back_facing (const Eigen::Vector3d &v0, const Eigen::Vector3d &v1, const Eigen::Vector3d &v2,
                                  const Eigen::Vector3d &)
{
  Eigen::Vector3d e1, e2, e3;
  e1 = v1 - v0;
  e2 = v2 - v0;

  Eigen::Vector3d z (m_extrinsic (0, 2), m_extrinsic (1, 2), m_extrinsic (2, 2));
  return z.dot (e1.cross (e2)) > 0.0;
}

/********************************************************************************/
/** DIRT interface **/

void
SequentialFitter::setInputCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  if (pcl_cloud.get () == nullptr || pcl_cloud->points.empty ())
    throw std::runtime_error ("[SequentialFitter::setInputCloud] Error: Empty or invalid pcl-point-cloud.\n");

  m_cloud = pcl_cloud;
  m_have_cloud = true;
}

void
SequentialFitter::setBoundary (pcl::PointIndices::Ptr &pcl_cloud_indexes)
{
  if (m_cloud.get () == nullptr || m_cloud->points.empty ())
    throw std::runtime_error ("[SequentialFitter::setBoundary] Error: Empty or invalid pcl-point-cloud.\n");

  this->m_boundary_indices = pcl_cloud_indexes;
  m_data.clear_boundary ();
  PCL2ON (m_cloud, pcl_cloud_indexes->indices, m_data.boundary);
}

void
SequentialFitter::setInterior (pcl::PointIndices::Ptr &pcl_cloud_indexes)
{
  if (m_cloud.get () == nullptr || m_cloud->points.empty ())
    throw std::runtime_error ("[SequentialFitter::setIndices] Error: Empty or invalid pcl-point-cloud.\n");

  this->m_interior_indices = pcl_cloud_indexes;
  m_data.clear_interior ();
  PCL2ON (m_cloud, pcl_cloud_indexes->indices, m_data.interior);
}

void
SequentialFitter::setCorners (pcl::PointIndices::Ptr &corners, bool flip_on_demand)
{
  if (m_cloud.get () == nullptr || m_cloud->points.empty ())
    throw std::runtime_error ("[SequentialFitter::setCorners] Error: Empty or invalid pcl-point-cloud.\n");

  if (corners->indices.size () < 4)
    throw std::runtime_error ("[SequentialFitter::setCorners] Error: too few corners (<4)\n");

  if (corners->indices.size () > 4)
    printf ("[SequentialFitter::setCorners] Warning: too many corners (>4)\n");

  bool flip = false;
  pcl::PointXYZRGB &pt0 = m_cloud->at (corners->indices[0]);
  pcl::PointXYZRGB &pt1 = m_cloud->at (corners->indices[1]);
  pcl::PointXYZRGB &pt2 = m_cloud->at (corners->indices[2]);
  pcl::PointXYZRGB &pt3 = m_cloud->at (corners->indices[3]);

  if (flip_on_demand)
  {
    Eigen::Vector3d v0 (pt0.x, pt0.y, pt0.z);
    Eigen::Vector3d v1 (pt1.x, pt1.y, pt1.z);
    Eigen::Vector3d v2 (pt2.x, pt2.y, pt2.z);
    Eigen::Vector3d v3 (pt3.x, pt3.y, pt3.z);
    flip = is_back_facing (v0, v1, v2, v3);
  }

  if (flip)
  {
    m_corners[3] = ON_3dPoint (pt0.x, pt0.y, pt0.z);
    m_corners[2] = ON_3dPoint (pt1.x, pt1.y, pt1.z);
    m_corners[1] = ON_3dPoint (pt2.x, pt2.y, pt2.z);
    m_corners[0] = ON_3dPoint (pt3.x, pt3.y, pt3.z);
  }
  else
  {
    m_corners[0] = ON_3dPoint (pt0.x, pt0.y, pt0.z);
    m_corners[1] = ON_3dPoint (pt1.x, pt1.y, pt1.z);
    m_corners[2] = ON_3dPoint (pt2.x, pt2.y, pt2.z);
    m_corners[3] = ON_3dPoint (pt3.x, pt3.y, pt3.z);
  }

  m_have_corners = true;
}

void
SequentialFitter::setProjectionMatrix (
    const Eigen::Matrix4d &intrinsic, 
    const Eigen::Matrix4d &extrinsic)
{
  m_intrinsic = intrinsic;
  m_extrinsic = extrinsic;
}

ON_NurbsSurface
SequentialFitter::compute (bool assemble)
{
  FittingSurface* fitting;
  ON_NurbsSurface nurbs;
  FittingSurface::Parameter paramFP (m_params.forceInterior, m_params.stiffnessInterior, 0.0, m_params.forceBoundary,
                                     m_params.stiffnessBoundary, 0.0);

  //  int surfid = -1;
  //  TomGine::tgRenderModel nurbs;
  //  TomGine::tgRenderModel box;

  if (!m_data.boundary.empty ())
  {
    //    throw std::runtime_error("[SequentialFitter::compute] Error: empty boundary point-cloud.\n");

    compute_quadfit ();

    nurbs = FittingSurface::initNurbs4Corners (m_params.order, m_corners[0], m_corners[1], m_corners[2], m_corners[3]);
    fitting = new FittingSurface (&m_data, nurbs);
    if (assemble)
      fitting->assemble (paramFP);

    compute_refinement (fitting);

    compute_boundary (fitting);
  }
  else
  {
    if (this->m_have_corners)
    {
      nurbs
          = FittingSurface::initNurbs4Corners (m_params.order, m_corners[0], m_corners[1], m_corners[2], m_corners[3]);
      fitting = new FittingSurface (&m_data, nurbs);
      if (assemble)
        fitting->assemble (paramFP);
    }
    else
    {
      fitting = new FittingSurface (m_params.order, &m_data);
      if (assemble)
        fitting->assemble (m_params.stiffnessInterior);
      int ncv0 = fitting->m_nurbs.m_cv_count[0];
      int ncv1 = fitting->m_nurbs.m_cv_count[1];

      fitting->m_nurbs.GetCV (0, 0, m_corners[0]);
      fitting->m_nurbs.GetCV (ncv0 - 1, 0, m_corners[1]);
      fitting->m_nurbs.GetCV (ncv0 - 1, ncv1 - 1, m_corners[2]);
      fitting->m_nurbs.GetCV (0, ncv1 - 1, m_corners[3]);

      Eigen::Vector3d v0 (m_corners[0].x, m_corners[0].y, m_corners[0].z);
      Eigen::Vector3d v1 (m_corners[1].x, m_corners[1].y, m_corners[1].z);
      Eigen::Vector3d v2 (m_corners[2].x, m_corners[2].y, m_corners[2].z);
      Eigen::Vector3d v3 (m_corners[3].x, m_corners[3].y, m_corners[3].z);

      if (is_back_facing (v0, v1, v2, v3))
      {
        ON_3dPoint tmp[4];
        tmp[0] = m_corners[0];
        tmp[1] = m_corners[1];
        tmp[2] = m_corners[2];
        tmp[3] = m_corners[3];
        m_corners[3] = tmp[0];
        m_corners[2] = tmp[1];
        m_corners[1] = tmp[2];
        m_corners[0] = tmp[3];
        delete (fitting);
        nurbs = FittingSurface::initNurbs4Corners (m_params.order, m_corners[0], m_corners[1], m_corners[2],
                                                   m_corners[3]);
        fitting = new FittingSurface (&m_data, nurbs);
        if (assemble)
          fitting->assemble (paramFP);
      }

      for (int r = 0; r < m_params.refinement; r++)
      {
        fitting->refine (0);
        fitting->refine (1);
      }
    }
  }

  if (!m_data.interior.empty ())
    compute_interior (fitting);

  // update error
  fitting->assemble ();

  m_nurbs = fitting->m_nurbs;

  delete (fitting);

  return m_nurbs;
}

ON_NurbsSurface
SequentialFitter::compute_boundary (const ON_NurbsSurface &nurbs)
{
  if (m_data.boundary.empty ())
  {
    printf ("[SequentialFitter::compute_boundary] Warning, no boundary points given: setBoundary()\n");
    return nurbs;
  }

  FittingSurface *fitting = new FittingSurface (&m_data, nurbs);

  this->compute_boundary (fitting);

  m_nurbs = fitting->m_nurbs;

  // update error
  fitting->assemble ();

  delete (fitting);

  return m_nurbs;
}

ON_NurbsSurface
SequentialFitter::compute_interior (const ON_NurbsSurface &nurbs)
{
  if (m_data.boundary.empty ())
  {
    printf ("[SequentialFitter::compute_interior] Warning, no interior points given: setInterior()\n");
    return nurbs;
  }
  FittingSurface *fitting = new FittingSurface (&m_data, nurbs);

  this->compute_interior (fitting);

  // update error
  fitting->assemble ();

  m_nurbs = fitting->m_nurbs;

  delete (fitting);

  return m_nurbs;
}

void
SequentialFitter::getInteriorError (std::vector<double> &error) const
{
  error = m_data.interior_error;
}

void
SequentialFitter::getBoundaryError (std::vector<double> &error) const
{
  error = m_data.boundary_error;
}

void
SequentialFitter::getInteriorParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params) const
{
  params = m_data.interior_param;
}

void
SequentialFitter::getBoundaryParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params) const
{
  params = m_data.boundary_param;
}

void
SequentialFitter::getInteriorNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normals) const
{
  normals = m_data.interior_normals;
}

void
SequentialFitter::getBoundaryNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normals) const
{
  normals = m_data.boundary_normals;
}

void
SequentialFitter::getClosestPointOnNurbs (
    ON_NurbsSurface nurbs, 
    const Eigen::Vector3d &pt, 
    Eigen::Vector2d& params,
    int maxSteps, 
    double accuracy)
{
  Eigen::Vector3d p, tu, tv;
  double error;

  params = FittingSurface::findClosestElementMidPoint (nurbs, pt);
  params = FittingSurface::inverseMapping (nurbs, pt, params, error, p, tu, tv, maxSteps, accuracy);
}

ON_NurbsSurface
SequentialFitter::grow (float max_dist, float max_angle, unsigned min_length, unsigned max_length)
{
  unsigned num_bnd = unsigned (this->m_data.boundary_param.size ());

  if (num_bnd == 0)
    throw std::runtime_error ("[SequentialFitter::grow] No boundary given.");

  if (unsigned (this->m_data.boundary.size ()) != num_bnd)
  {
    printf ("[SequentialFitter::grow] %zu %u\n", this->m_data.boundary.size (), num_bnd);
    throw std::runtime_error ("[SequentialFitter::grow] size of boundary and boundary parameters do not match.");
  }

  if (this->m_boundary_indices->indices.size () != num_bnd)
  {
    printf ("[SequentialFitter::grow] %zu %u\n", this->m_boundary_indices->indices.size (), num_bnd);
    throw std::runtime_error ("[SequentialFitter::grow] size of boundary indices and boundary parameters do not match.");
  }

  float angle = std::cos (max_angle);

  for (unsigned i = 0; i < num_bnd; i++)
  {
    Eigen::Vector3d r, tu, tv, n, bn;
    double pointAndTangents[9];
    double u = this->m_data.boundary_param[i] (0);
    double v = this->m_data.boundary_param[i] (1);

    // Evaluate point and tangents
    m_nurbs.Evaluate (u, v, 1, 3, pointAndTangents);
    r (0) = pointAndTangents[0];
    r (1) = pointAndTangents[1];
    r (2) = pointAndTangents[2];
    tu (0) = pointAndTangents[3];
    tu (1) = pointAndTangents[4];
    tu (2) = pointAndTangents[5];
    tv (0) = pointAndTangents[6];
    tv (1) = pointAndTangents[7];
    tv (2) = pointAndTangents[8];

    n = tu.cross (tv);
    n.normalize ();

    // calculate boundary normal (pointing outward)
    double eps = 0.00000001;

    if (u < eps)
      bn = n.cross (tv);
    if (u > 1.0 - eps)
      bn = -n.cross (tv);
    if (v < eps)
      bn = -n.cross (tu);
    if (v > 1.0 - eps)
      bn = n.cross (tu);

    bn.normalize ();

    Eigen::Vector3d e (r (0) + bn (0) * max_dist, r (1) + bn (1) * max_dist, r (2) + bn (2) * max_dist);

    // Project into image plane
    Eigen::Vector2d ri = this->project (r);
    Eigen::Vector2d ei = this->project (e);
    Eigen::Vector2d bni = ei - ri;
    bni.normalize ();

    // search for valid points along boundary normal in image space
    float max_dist_sq = max_dist * max_dist;
    bool valid = false;
    pcl::PointXYZRGB point = m_cloud->at (this->m_boundary_indices->indices[i]);
    for (unsigned j = min_length; j < max_length; j++)
    {
      int col = int (ri (0) + bni (0) * j);
      int row = int (ri (1) + bni (1) * j);

      if (row >= int (m_cloud->height) || row < 0)
      {
        break;
      }
      if (col >= int (m_cloud->width) || col < 0)
      {
        break;
      }

      unsigned idx = row * m_cloud->width + col;

      pcl::PointXYZRGB &pt = m_cloud->at (idx);
      if (!std::isnan (pt.x) && !std::isnan (pt.y) && !std::isnan (pt.z))
      {

        // distance requirement
        Eigen::Vector3d d (pt.x - r (0), pt.y - r (1), pt.z - r (2));
        if (d.dot (d) < max_dist_sq)
        {
          d.normalize ();
          if (std::abs (d.dot (bn)) > (angle))
          {
            valid = true;
            point = pt;
          } // dot
        } // max_dist
      } // isnan
    } // j

    // if valid point found, add current boundary point to interior points and move boundary
    if (valid)
    {
      this->m_data.interior.push_back (this->m_data.boundary[i]);
      this->m_data.boundary[i] (0) = point.x;
      this->m_data.boundary[i] (1) = point.y;
      this->m_data.boundary[i] (2) = point.z;
    }

  } // i

  compute_interior (m_nurbs);

  double int_err (0.0);
  double div_err = 1.0 / double (m_data.interior_error.size ());
  for (const double &i : m_data.interior_error)
  {
    int_err += (i * div_err);
  }

  printf ("[SequentialFitter::grow] average interior error: %e\n", int_err);

  return m_nurbs;

}

unsigned
SequentialFitter::PCL2ON (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, const pcl::Indices &indices,
                          vector_vec3d &on_cloud)
{
  std::size_t numPoints = 0;

  for (const auto &index : indices)
  {

    pcl::PointXYZRGB &pt = pcl_cloud->at (index);

    if (!std::isnan (pt.x) && !std::isnan (pt.y) && !std::isnan (pt.z))
    {
      on_cloud.push_back (Eigen::Vector3d (pt.x, pt.y, pt.z));
      numPoints++;
    }

  }

  return numPoints;
}
