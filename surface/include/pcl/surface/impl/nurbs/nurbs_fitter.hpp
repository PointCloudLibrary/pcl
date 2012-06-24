/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer, Inc.
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
 *   * Neither the name of Thomas Mörwald or Jonathan Balzer nor the names of its
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

#include <pcl/surface/nurbs/nurbs_fitter.h>

template<typename PointInT>
pcl::nurbs::NurbsFitter<PointInT>::Parameter::Parameter (int order, int refinement, int iterationsQuad, int iterationsBoundary,
                                                         int iterationsAdjust, int iterationsInterior, double forceBoundary,
                                                         double forceBoundaryInside, double forceInterior,
                                                         double stiffnessBoundary, double stiffnessInterior, int resolution)
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

template<typename PointInT>
pcl::nurbs::NurbsFitter<PointInT>::NurbsFitter (Parameter p)
{
  this->params_ = p;
  this->have_cloud_ = false;
  this->have_corners_ = false;
  this->surface_id_ = -1;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::computeQuadFit ()
{
  pcl::nurbs::NurbsFitting *fitting;

  if (have_corners_)
  {
    fitting = new pcl::nurbs::NurbsFitting (2, &nurbs_data_, corners_[0], corners_[1], corners_[2], corners_[3]);
  }
  else
  {
    fitting = new pcl::nurbs::NurbsFitting (2, &nurbs_data_);
    corners_[0] = fitting->m_patch->getControlPoint (0, 0);
    corners_[1] = fitting->m_patch->getControlPoint (1, 0);
    corners_[2] = fitting->m_patch->getControlPoint (1, 1);
    corners_[3] = fitting->m_patch->getControlPoint (0, 1);
    Eigen::Vector3d v0 (corners_[0] (0), corners_[0] (1), corners_[0] (2));
    Eigen::Vector3d v1 (corners_[1] (0), corners_[1] (1), corners_[1] (2));
    Eigen::Vector3d v2 (corners_[2] (0), corners_[2] (1), corners_[2] (2));
    Eigen::Vector3d v3 (corners_[3] (0), corners_[3] (1), corners_[3] (2));
    if (isBackFacing (v0, v1, v2, v3))
    {
      pcl::nurbs::vec4 tmp[4];
      tmp[0] = corners_[0];
      tmp[1] = corners_[1];
      tmp[2] = corners_[2];
      tmp[3] = corners_[3];
      corners_[3] = tmp[0];
      corners_[2] = tmp[1];
      corners_[1] = tmp[2];
      corners_[0] = tmp[3];
      delete (fitting);
      fitting = new pcl::nurbs::NurbsFitting (2, &nurbs_data_, corners_[0], corners_[1], corners_[2], corners_[3]);
    }

  }

  // Quad fitting
  //  if( !m_quiet && m_dbgWin != NULL )
  //    NurbsConvertion::Nurbs2TomGine(m_dbgWin, *fitting->m_patch, m_surf_id, m_params.resolution);
  for (int r = 0; r < params_.iterationsQuad; r++)
  {
    fitting->assemble (0, 0, params_.forceBoundary, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0);
    fitting->solve ();
  }
  corners_[0] = fitting->m_patch->getControlPoint (0, 0);
  corners_[1] = fitting->m_patch->getControlPoint (1, 0);
  corners_[2] = fitting->m_patch->getControlPoint (1, 1);
  corners_[3] = fitting->m_patch->getControlPoint (0, 1);

  delete (fitting);
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::computeRefinement (pcl::nurbs::NurbsFitting* fitting)
{
  // Refinement
  for (int r = 0; r < params_.refinement; ++r)
  {
    fitting->assemble (0, 0, params_.forceBoundary, 0.0, 0.0, 0.0, params_.stiffnessBoundary, 1.0, 0.0);
    fitting->solve ();
    fitting->refine (0);
    fitting->refine (1);
  }
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::computeBoundary (pcl::nurbs::NurbsFitting* fitting)
{
  // iterate boundary points
  for (int i = 0; i < params_.iterationsBoundary; i++)
  {
    fitting->assemble (0, 0, params_.forceBoundary, 0.0, 0.0, 0.0, params_.stiffnessBoundary, 1.0, 0.0);
    fitting->solve ();
  }
}
template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::computeInterior (pcl::nurbs::NurbsFitting* fitting)
{
  // iterate interior points
  //std::vector<double> wInt(m_data.interior.PointCount(), m_params.forceInterior);
  for (int i = 0; i < params_.iterationsInterior; i++)
  {
    fitting->assemble (0, 0, params_.forceBoundary, params_.forceInterior, 0.0, 0.0, params_.stiffnessBoundary,
                       params_.stiffnessInterior, 0.0);
    //    fitting->assemble(wBnd, wInt, m_params.stiffnessBoundary, m_params.stiffnessInterior);
    fitting->solve ();
  }
}

template<typename PointInT> Eigen::Vector2d
pcl::nurbs::NurbsFitter<PointInT>::project (const Eigen::Vector3d &pt)
{
  Eigen::Vector4d pr = intrinsics_ * extrinsics_ * Eigen::Vector4d (pt (0), pt (1), pt (2), 1.0);
  pr (0) = pr (0) / pr (2);
  pr (1) = pr (1) / pr (2);
  if (pt.dot (Eigen::Vector3d (extrinsics_ (0, 2), extrinsics_ (1, 2), extrinsics_ (2, 2))) < 0.0f)
  { // avoids backprojection
    pr (0) = -pr (0);
    pr (1) = -pr (1);
  }
  return (Eigen::Vector2d (pr (0), pr (1)));
}

template<typename PointInT> bool
pcl::nurbs::NurbsFitter<PointInT>::isBackFacing (const Eigen::Vector3d &v0, const Eigen::Vector3d &v1,
                                                 const Eigen::Vector3d &v2, const Eigen::Vector3d &v3)
{
  Eigen::Vector3d e1, e2, e3;
  e1 = v1 - v0;
  e2 = v2 - v0;

  Eigen::Vector3d z (extrinsics_ (0, 2), extrinsics_ (1, 2), extrinsics_ (2, 2));
  if (z.dot (e1.cross (e2)) > 0.0)
    return (true);
  else
    return (false);
}

/********************************************************************************/
/** DIRT interface **/

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::setInputCloud (PointCloudPtr &pcl_cloud)
{
  if (pcl_cloud.get () == 0 || pcl_cloud->points.size () == 0)
    throw std::runtime_error ("[NurbsFitter::setInputCloud] Error: Empty or invalid pcl-point-cloud.\n");

  cloud_ = pcl_cloud;
  have_cloud_ = true;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::setBoundary (pcl::PointIndices::Ptr &pcl_cloud_indexes)
{
  if (cloud_.get () == 0 || cloud_->points.size () == 0)
    throw std::runtime_error ("[NurbsFitter::setBoundary] Error: Empty or invalid pcl-point-cloud.\n");

  this->boundary_indices_ = pcl_cloud_indexes;
  nurbs_data_.clearBoundary ();
  PCL2Eigen (cloud_, pcl_cloud_indexes->indices, nurbs_data_.boundary);
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::setInterior (pcl::PointIndices::Ptr &pcl_cloud_indexes)
{
  if (cloud_.get () == 0 || cloud_->points.size () == 0)
    throw std::runtime_error ("[NurbsFitter::setIndices] Error: Empty or invalid pcl-point-cloud.\n");

  this->interior_indices_ = pcl_cloud_indexes;
  nurbs_data_.clearInterior ();
  PCL2Eigen (cloud_, pcl_cloud_indexes->indices, nurbs_data_.interior);
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::setCorners (pcl::PointIndices::Ptr &corners, bool flip_on_demand)
{
  if (cloud_.get () == 0 || cloud_->points.size () == 0)
    throw std::runtime_error ("[NurbsFitter::setCorners] Error: Empty or invalid pcl-point-cloud.\n");

  if (corners->indices.size () < 4)
    throw std::runtime_error ("[NurbsFitter::setCorners] Error: to few corners (<4)\n");

  if (corners->indices.size () > 4)
    printf ("[NurbsFitter::setCorners] Warning: to many corners (>4)\n");

  bool flip = false;
  PointInT &pt0 = cloud_->at (corners->indices[0]);
  PointInT &pt1 = cloud_->at (corners->indices[1]);
  PointInT &pt2 = cloud_->at (corners->indices[2]);
  PointInT &pt3 = cloud_->at (corners->indices[3]);

  if (flip_on_demand)
  {
    Eigen::Vector3d v0 (pt0.x, pt0.y, pt0.z);
    Eigen::Vector3d v1 (pt1.x, pt1.y, pt1.z);
    Eigen::Vector3d v2 (pt2.x, pt2.y, pt2.z);
    Eigen::Vector3d v3 (pt3.x, pt3.y, pt3.z);
    flip = isBackFacing (v0, v1, v2, v3);
  }

  if (flip)
  {
    corners_[3] = vec4 (pt0.x, pt0.y, pt0.z, 1.0);
    corners_[2] = vec4 (pt1.x, pt1.y, pt1.z, 1.0);
    corners_[1] = vec4 (pt2.x, pt2.y, pt2.z, 1.0);
    corners_[0] = vec4 (pt3.x, pt3.y, pt3.z, 1.0);
  }
  else
  {
    corners_[0] = vec4 (pt0.x, pt0.y, pt0.z, 1.0);
    corners_[1] = vec4 (pt1.x, pt1.y, pt1.z, 1.0);
    corners_[2] = vec4 (pt2.x, pt2.y, pt2.z, 1.0);
    corners_[3] = vec4 (pt3.x, pt3.y, pt3.z, 1.0);
  }

  have_corners_ = true;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::setProjectionMatrix (const Eigen::Matrix4d &intrinsics, const Eigen::Matrix4d &extrinsics)
{
  intrinsics_ = intrinsics;
  extrinsics_ = extrinsics;
}

template<typename PointInT> pcl::nurbs::NurbsSurface
pcl::nurbs::NurbsFitter<PointInT>::compute ()
{
  pcl::nurbs::NurbsFitting* fitting;

  //  int surfid = -1;
  //  TomGine::tgRenderModel nurbs;
  //  TomGine::tgRenderModel box;

  if (nurbs_data_.boundary.size () > 0)
  {
    //    throw std::runtime_error("[NurbsFitter::compute] Error: empty boundary point-cloud.\n");

    computeQuadFit ();

    fitting = new pcl::nurbs::NurbsFitting (params_.order, &nurbs_data_, corners_[0], corners_[1], corners_[2], corners_[3]);

    computeRefinement (fitting);

    computeBoundary (fitting);
  }
  else
  {
    if (this->have_corners_)
    {
      fitting = new pcl::nurbs::NurbsFitting (params_.order, &nurbs_data_, corners_[0], corners_[1], corners_[2], corners_[3]);
    }
    else
    {
      fitting = new pcl::nurbs::NurbsFitting (params_.order, &nurbs_data_);
      int ncv0 = fitting->m_patch->nbControlPointsU ();
      int ncv1 = fitting->m_patch->nbControlPointsV ();

      corners_[0] = fitting->m_patch->getControlPoint (0, 0);
      corners_[1] = fitting->m_patch->getControlPoint (ncv0 - 1, 0);
      corners_[2] = fitting->m_patch->getControlPoint (ncv0 - 1, ncv1 - 1);
      corners_[3] = fitting->m_patch->getControlPoint (0, ncv1 - 1);

      Eigen::Vector3d v0 (corners_[0] (0), corners_[0] (1), corners_[0] (2));
      Eigen::Vector3d v1 (corners_[1] (0), corners_[1] (1), corners_[1] (2));
      Eigen::Vector3d v2 (corners_[2] (0), corners_[2] (1), corners_[2] (2));
      Eigen::Vector3d v3 (corners_[3] (0), corners_[3] (1), corners_[3] (2));

      if (isBackFacing (v0, v1, v2, v3))
      {
        vec4 tmp[4];
        tmp[0] = corners_[0];
        tmp[1] = corners_[1];
        tmp[2] = corners_[2];
        tmp[3] = corners_[3];
        corners_[3] = tmp[0];
        corners_[2] = tmp[1];
        corners_[1] = tmp[2];
        corners_[0] = tmp[3];
        delete (fitting);
        fitting = new pcl::nurbs::NurbsFitting (params_.order, &nurbs_data_, corners_[0], corners_[1], corners_[2], corners_[3]);
      }

      for (int r = 0; r < params_.refinement; r++)
      {
        fitting->refine (0);
        fitting->refine (1);
      }
    }
  }

  if (nurbs_data_.interior.size () > 0)
    computeInterior (fitting);

  // update error
  fitting->assemble (0, 0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  nurbs_surface_ = *fitting->m_patch;

  delete (fitting);

  return (nurbs_surface_);
}

template<typename PointInT> pcl::nurbs::NurbsSurface
pcl::nurbs::NurbsFitter<PointInT>::computeBoundary (const pcl::nurbs::NurbsSurface &nurbs)
{
  if (nurbs_data_.boundary.size () <= 0)
  {
    printf ("[NurbsFitter::compute_boundary] Warning, no boundary points given: setBoundary()\n");
    return (nurbs);
  }

  pcl::nurbs::NurbsFitting *fitting = new pcl::nurbs::NurbsFitting (&nurbs_data_, nurbs);

  this->computeBoundary (fitting);

  nurbs_surface_ = *fitting->m_patch;

  // update error
  fitting->assemble (0, 0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  delete (fitting);

  return (nurbs_surface_);
}

template<typename PointInT> pcl::nurbs::NurbsSurface
pcl::nurbs::NurbsFitter<PointInT>::computeInterior (const pcl::nurbs::NurbsSurface &nurbs)
{
  if (nurbs_data_.boundary.size () <= 0)
  {
    printf ("[NurbsFitter::compute_interior] Warning, no interior points given: setInterior()\n");
    return (nurbs);
  }
  pcl::nurbs::NurbsFitting *fitting = new pcl::nurbs::NurbsFitting (&nurbs_data_, nurbs);

  this->computeInterior (fitting);

  // update error
  fitting->assemble (0, 0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  nurbs_surface_ = *fitting->m_patch;

  delete (fitting);

  return (nurbs_surface_);
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::getInteriorError (std::vector<double> &error)
{
  error = nurbs_data_.interior_error;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::getBoundaryError (std::vector<double> &error)
{
  error = nurbs_data_.boundary_error;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::getInteriorParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params)
{
  params = nurbs_data_.interior_param;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::getBoundaryParams (std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &params)
{
  params = nurbs_data_.boundary_param;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::getInteriorNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normals)
{
  normals = nurbs_data_.interior_normals;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::getBoundaryNormals (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &normals)
{
  normals = nurbs_data_.boundary_normals;
}

template<typename PointInT> void
pcl::nurbs::NurbsFitter<PointInT>::getClosestPointOnNurbs (pcl::nurbs::NurbsSurface nurbs, Eigen::Vector3d pt, Eigen::Vector2d& params,
                                                           int maxSteps, double accuracy)
{
  NurbsTools ntools (&nurbs);

  Eigen::Vector3d p, tu, tv;
  double error;

  params = ntools.inverseMapping (pt, NULL, error, p, tu, tv, maxSteps, accuracy);
}

template<typename PointInT> pcl::nurbs::NurbsSurface
pcl::nurbs::NurbsFitter<PointInT>::grow (float max_dist, float max_angle, unsigned min_length, unsigned max_length)
{
  unsigned num_bnd = this->nurbs_data_.boundary_param.size ();

  if (num_bnd == 0)
    throw std::runtime_error ("[NurbsFitter::grow] No boundary given.");

  if ((unsigned)this->nurbs_data_.boundary.size () != num_bnd)
  {
    printf ("[NurbsFitter::grow] %u %u\n", (unsigned)this->nurbs_data_.boundary.size (), num_bnd);
    throw std::runtime_error ("[NurbsFitter::grow] size of boundary and boundary parameters do not match.");
  }

  if (this->boundary_indices_->indices.size () != num_bnd)
  {
    printf ("[NurbsFitter::grow] %u %u\n", (unsigned)this->boundary_indices_->indices.size (), num_bnd);
    throw std::runtime_error ("[NurbsFitter::grow] size of boundary indices and boundary parameters do not match.");
  }

  float angle = cos (max_angle);
  unsigned bnd_moved (0);

  for (unsigned i = 0; i < num_bnd; i++)
  {
    vec3 r (0.0, 0.0, 0.0), tu (0.0, 0.0, 0.0), tv (0.0, 0.0, 0.0), n (0.0, 0.0, 0.0), bn (0.0, 0.0, 0.0);
    double u = this->nurbs_data_.boundary_param[i] (0);
    double v = this->nurbs_data_.boundary_param[i] (1);

    // Evaluate point and tangents
    nurbs_surface_.evaluate (u, v, r, tu, tv);

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
    PointInT point = cloud_->at (this->boundary_indices_->indices[i]);
    for (unsigned j = min_length; j < max_length; j++)
    {
      int col = static_cast<int> (ri (0) + bni (0) * j);
      int row = static_cast<int> (ri (1) + bni (1) * j);

      if (row >= int (cloud_->height) || row < 0)
      {
        j = max_length;
        break;
      }
      if (col >= int (cloud_->width) || col < 0)
      {
        j = max_length;
        break;
      }

      unsigned idx = row * cloud_->width + col;

      const PointInT &pt = cloud_->at (idx);
      if (!pcl_isnan (pt.x) && !pcl_isnan (pt.y) && !pcl_isnan (pt.z))
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
      this->nurbs_data_.interior.push_back (this->nurbs_data_.boundary[i]);
      this->nurbs_data_.boundary[i] (0) = point.x;
      this->nurbs_data_.boundary[i] (1) = point.y;
      this->nurbs_data_.boundary[i] (2) = point.z;
      ++bnd_moved;
    }

  } // i

  computeInterior (nurbs_surface_);

  double int_err (0.0);
  double div_err = 1.0 / nurbs_data_.interior_error.size ();
  for (unsigned i = 0; i < nurbs_data_.interior_error.size (); i++)
  {
    int_err += (nurbs_data_.interior_error[i] * div_err);
  }

  printf ("[NurbsFitter::grow] average interior error: %e\n", int_err);

  return (nurbs_surface_);
}

template<typename PointInT> std::size_t
pcl::nurbs::NurbsFitter<PointInT>::PCL2Eigen (PointCloudPtr &pcl_cloud, const std::vector<int> &indices, vector_vec3 &on_cloud)
{
  std::size_t numPoints (0);

  for (unsigned i = 0; i < indices.size (); i++)
  {
    const PointInT &pt = pcl_cloud->at (indices[i]);

    if (!pcl_isnan (pt.x) && !pcl_isnan (pt.y) && !pcl_isnan (pt.z))
    {
      on_cloud.push_back (vec3 (pt.x, pt.y, pt.z));
      ++numPoints;
    }
  }

  return (numPoints);
}

#define PCL_INSTANTIATE_NurbsFitter(T) template class PCL_EXPORTS pcl::nurbs::NurbsFitter<T>;
