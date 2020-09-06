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

#include <pcl/surface/on_nurbs/global_optimization_pdm.h>
#include <pcl/surface/on_nurbs/closing_boundary.h>
#include <pcl/pcl_macros.h>

#include <Eigen/Geometry> // for cross
#include <stdexcept>

#undef DEBUG

using namespace pcl;
using namespace on_nurbs;
using namespace Eigen;

GlobalOptimization::GlobalOptimization (const std::vector<NurbsDataSurface*> &data,
                                        const std::vector<ON_NurbsSurface*> &nurbs)
{
  if (data.empty () || data.size () != nurbs.size ())
    printf ("[GlobalOptimization::GlobalOptimization] Error, data empty or size does not match.\n");

  m_data = data;
  m_nurbs = nurbs;

  im_max_steps = 100;
  im_accuracy = 1e-5;

  m_quiet = true;
}

void
GlobalOptimization::setCommonBoundary (const vector_vec3d &boundary, const vector_vec2i &nurbs_indices)
{
  if (boundary.empty () || boundary.size () != nurbs_indices.size ())
  {
    printf ("[GlobalOptimization::setCommonBoundary] Error, common boundary empty or size does not match.\n");
    return;
  }
}

void
GlobalOptimization::assemble (Parameter params)
{
  // determine number of rows of matrix
  m_ncols = 0;
  unsigned nnurbs = static_cast<unsigned> (m_nurbs.size ());
  unsigned nInt (0), nBnd (0), nCageRegInt (0), nCageRegBnd (0), nCommonBnd (0), nCommonPar (0);
  for (unsigned i = 0; i < nnurbs; i++)
  {
    nInt += static_cast<unsigned> (m_data[i]->interior.size ());
    nBnd += static_cast<unsigned> (m_data[i]->boundary.size ());
    nCommonBnd += static_cast<unsigned> (m_data[i]->common_boundary_point.size ());
    nCommonPar += static_cast<unsigned> (m_data[i]->common_idx.size ());
    nCageRegInt += (m_nurbs[i]->CVCount (0) - 2) * (m_nurbs[i]->CVCount (1) - 2);
    nCageRegBnd += 2 * (m_nurbs[i]->CVCount (0) - 1) + 2 * (m_nurbs[i]->CVCount (1) - 1);
    m_ncols += m_nurbs[i]->CVCount ();
  }

  m_nrows = 0;
  if (params.interior_weight > 0.0)
    m_nrows += nInt;
  if (params.boundary_weight > 0.0)
    m_nrows += nBnd;
  if (params.interior_smoothness > 0.0)
    m_nrows += nCageRegInt;
  if (params.boundary_smoothness > 0.0)
    m_nrows += nCageRegBnd;
  if (params.closing_weight > 0.0)
  {
    if (params.closing_samples > 0)
      m_nrows += (4 * params.closing_samples);
    else
      m_nrows += nBnd;
  }
  if (params.common_weight > 0.0)
  {
    m_nrows += nCommonBnd;
    m_nrows += nCommonPar;
  }

  if (!m_quiet)
    printf ("[GlobalOptimization::assemble] Size of matrix: %dx%d\n", m_nrows, m_ncols);

  m_solver.assign (m_nrows, m_ncols, 3);

  for (const auto &i : m_data)
    i->common_boundary_param.clear ();

  // assemble matrix
  unsigned row (0);
  int ncps (0);

  for (unsigned id = 0; id < nnurbs; id++)
  {

    // interior points should lie on surface
    assembleInteriorPoints (id, ncps, params.interior_weight, row);

    // boundary points should lie on boundary of surface
    assembleBoundaryPoints (id, ncps, params.boundary_weight, row);

    // cage regularisation
    assembleRegularisation (id, ncps, params.interior_smoothness, params.boundary_smoothness, row);

    // closing boundaries
    assembleClosingBoundaries (id, params.closing_samples, params.closing_sigma, params.closing_weight, row);

    // common boundaries
    assembleCommonBoundaries (id, params.common_weight, row);

    // common parameter points
    assembleCommonParams (id, params.common_weight, row);

    ncps += m_nurbs[id]->CVCount ();
  }
}

void
GlobalOptimization::solve (double damp)
{
  if (m_solver.solve ())
    updateSurf (damp);
}

void
GlobalOptimization::updateSurf (double damp)
{
  int ncps (0);

  for (const auto &nurbs : m_nurbs)
  {
    int ncp = nurbs->CVCount ();

    for (int A = 0; A < ncp; A++)
    {

      int I = gl2gr (*nurbs, A);
      int J = gl2gc (*nurbs, A);

      ON_3dPoint cp_prev;
      nurbs->GetCV (I, J, cp_prev);

      ON_3dPoint cp;
      cp.x = cp_prev.x + damp * (m_solver.x (ncps + A, 0) - cp_prev.x);
      cp.y = cp_prev.y + damp * (m_solver.x (ncps + A, 1) - cp_prev.y);
      cp.z = cp_prev.z + damp * (m_solver.x (ncps + A, 2) - cp_prev.z);

      nurbs->SetCV (I, J, cp);
    }

    ncps += ncp;
  }
}

void
GlobalOptimization::setInvMapParams (double im_max_steps, double im_accuracy)
{
  this->im_max_steps = static_cast<int> (im_max_steps);
  this->im_accuracy = im_accuracy;
}

void
GlobalOptimization::refine (unsigned id, int dim)
{
  std::vector<double> xi;
  std::vector<double> elements = FittingSurface::getElementVector (*m_nurbs[id], dim);

  for (std::size_t i = 0; i < elements.size () - 1; i++)
    xi.push_back (elements[i] + 0.5 * (elements[i + 1] - elements[i]));

  for (const double &i : xi)
  {
    m_nurbs[id]->InsertKnot (dim, i, 1);
  }
}

void
GlobalOptimization::assembleCommonParams (unsigned id1, double weight, unsigned &row)
{
  if (weight <= 0.0)
    return;

  NurbsDataSurface *data = m_data[id1];

  for (std::size_t i = 0; i < data->common_idx.size (); i++)
    addParamConstraint (Eigen::Vector2i (id1, data->common_idx[i]), data->common_param1[i], data->common_param2[i],
                        weight, row);
}

void
GlobalOptimization::assembleCommonBoundaries (unsigned id1, double weight, unsigned &row)
{
  if (weight <= 0.0)
    return;

  //  double ds = 1.0 / (2.0 * sigma * sigma);
  NurbsDataSurface *data1 = m_data[id1];
  ON_NurbsSurface* nurbs1 = m_nurbs[id1];

  if (nurbs1->Order (0) != nurbs1->Order (1))
    printf ("[GlobalOptimization::assembleCommonBoundaries] Warning, order in u and v direction differ (nurbs1).\n");

  for (std::size_t i = 0; i < data1->common_boundary_point.size (); i++)
  {
    Eigen::Vector3d p1, tu1, tv1, p2, tu2, tv2, t1, t2;
    Eigen::Vector2d params1, params2;
    double error1, error2;
    Eigen::Vector3d p0 = data1->common_boundary_point[i];
    Eigen::Vector2i id (id1, data1->common_boundary_idx[i]);

    if (id (1) < 0 || id (1) >= static_cast<int>(m_nurbs.size ()))
      throw std::runtime_error (
                                "[GlobalOptimization::assembleCommonBoundaries] Error, common boundary index out of bounds.\n");

    ON_NurbsSurface* nurbs2 = m_nurbs[id (1)];
    double w (1.0);

    if (nurbs2->Order (0) != nurbs2->Order (1))
      printf ("[GlobalOptimization::assembleCommonBoundaries] Warning, order in u and v direction differ (nurbs2).\n");

    if (nurbs1->Order (0) == nurbs2->Order (0))
    {
      params1 = FittingSurface::inverseMappingBoundary (*m_nurbs[id (0)], p0, error1, p1, tu1, tv1, im_max_steps,
                                                        im_accuracy, true);
      params2 = FittingSurface::inverseMappingBoundary (*m_nurbs[id (1)], p0, error2, p2, tu2, tv2, im_max_steps,
                                                        im_accuracy, true);

      if (params1 (0) == 0.0 || params1 (0) == 1.0)
        t1 = tv1;
      if (params1 (1) == 0.0 || params1 (1) == 1.0)
        t1 = tu1;
      if (params2 (0) == 0.0 || params2 (0) == 1.0)
        t2 = tv2;
      if (params2 (1) == 0.0 || params2 (1) == 1.0)
        t2 = tu2;

      t1.normalize ();
      t2.normalize ();

      // weight according to parallel-ness of boundaries
      w = t1.dot (t2);
      if (w < 0.0)
        w = -w;
    }
    else
    {

      if (nurbs1->Order (0) < nurbs2->Order (0))
      {
        params1 = FittingSurface::findClosestElementMidPoint (*m_nurbs[id (0)], p0);
        params1 = FittingSurface::inverseMapping (*m_nurbs[id (0)], p0, params1, error1, p1, tu1, tv1, im_max_steps,
                                                  im_accuracy, true);
        params2 = FittingSurface::inverseMappingBoundary (*m_nurbs[id (1)], p0, error2, p2, tu2, tv2, im_max_steps,
                                                          im_accuracy, true);
      }
      else
      {
        params1 = FittingSurface::inverseMappingBoundary (*m_nurbs[id (0)], p0, error1, p1, tu1, tv1, im_max_steps,
                                                          im_accuracy, true);
        params2 = FittingSurface::findClosestElementMidPoint (*m_nurbs[id (1)], p0);
        params2 = FittingSurface::inverseMapping (*m_nurbs[id (1)], p0, params2, error2, p2, tu2, tv2, im_max_steps,
                                                  im_accuracy);
      }

    }

    m_data[id (0)]->common_boundary_param.push_back (params1);
    m_data[id (1)]->common_boundary_param.push_back (params2);

    //    double error = (p1-p2).norm();
    //    double w = weight * std::exp(-(error * error) * ds);
    addParamConstraint (id, params1, params2, weight * w, row);

  }
}

void
GlobalOptimization::assembleClosingBoundaries (unsigned id, unsigned samples, double sigma, double weight,
                                               unsigned &row)
{
  if (weight <= 0.0 || samples <= 0 || sigma < 0.0)
    return;

  double ds = 1.0 / (2.0 * sigma * sigma);

  ON_NurbsSurface *nurbs1 = m_nurbs[id];

  // sample point list from nurbs1
  vector_vec3d boundary1, boundary2;
  vector_vec2d params1, params2;
  ClosingBoundary::sampleFromBoundary (nurbs1, boundary1, params1, samples);

  // for each other nurbs
  for (std::size_t n2 = (id + 1); n2 < m_nurbs.size (); n2++)
  {
    ON_NurbsSurface *nurbs2 = m_nurbs[n2];

    // find closest point to boundary
    for (std::size_t i = 0; i < boundary1.size (); i++)
    {
      double error;
      Eigen::Vector3d p, tu, tv;
      Eigen::Vector2d params;
      Eigen::Vector3d p0 = boundary1[i];

      params = FittingSurface::inverseMappingBoundary (*nurbs2, p0, error, p, tu, tv, im_max_steps, im_accuracy, true);

      boundary2.push_back (p);
      params2.push_back (params);

      //      double dist = (p - p0).norm();
      //      if (error < max_error && dist < max_dist) {
      double w = weight * std::exp (-(error * error) * ds);
      addParamConstraint (Eigen::Vector2i (id, n2), params1[i], params, w, row);
      //      }
    }
  }
}

void
GlobalOptimization::assembleInteriorPoints (unsigned id, int ncps, double weight, unsigned &row)
{
  if (weight <= 0.0)
    return;

  ON_NurbsSurface *nurbs = m_nurbs[id];
  NurbsDataSurface *data = m_data[id];
  std::size_t interiorSize = m_data[id]->interior.size ();

  // interior points should lie on surface
  data->interior_line_start.clear ();
  data->interior_line_end.clear ();
  data->interior_error.clear ();
  data->interior_normals.clear ();
  //  data->interior_param.clear();

  for (std::size_t p = 0; p < interiorSize; p++)
  {
    Vector3d pcp;
    pcp (0) = data->interior[p] (0);
    pcp (1) = data->interior[p] (1);
    pcp (2) = data->interior[p] (2);

    // inverse mapping
    Vector2d params;
    Vector3d pt, tu, tv, n;
    double error;
    if (p < data->interior_param.size ())
    {
      params = FittingSurface::inverseMapping (*nurbs, pcp, data->interior_param[p], error, pt, tu, tv, im_max_steps,
                                               im_accuracy);
      data->interior_param[p] = params;
    }
    else
    {
      params = FittingSurface::findClosestElementMidPoint (*nurbs, pcp);
      params = FittingSurface::inverseMapping (*nurbs, pcp, params, error, pt, tu, tv, im_max_steps, im_accuracy);
      data->interior_param.push_back (params);
    }
    data->interior_error.push_back (error);

    n = tu.cross (tv);
    n.normalize ();

    data->interior_normals.push_back (n);
    data->interior_line_start.push_back (pcp);
    data->interior_line_end.push_back (pt);

    addPointConstraint (id, ncps, params, pcp, weight, row);
  }
}

void
GlobalOptimization::assembleBoundaryPoints (unsigned id, int ncps, double weight, unsigned &row)
{
  if (weight <= 0.0)
    return;

  ON_NurbsSurface *nurbs = m_nurbs[id];
  NurbsDataSurface *data = m_data[id];
  unsigned nBnd = static_cast<unsigned> (m_data[id]->boundary.size ());

  // interior points should lie on surface
  data->boundary_line_start.clear ();
  data->boundary_line_end.clear ();
  data->boundary_error.clear ();
  data->boundary_normals.clear ();
  data->boundary_param.clear ();

  for (unsigned p = 0; p < nBnd; p++)
  {
    Vector3d pcp;
    pcp (0) = data->boundary[p] (0);
    pcp (1) = data->boundary[p] (1);
    pcp (2) = data->boundary[p] (2);

    // inverse mapping
    Vector3d pt, tu, tv, n;
    double error;

    Vector2d params =
        FittingSurface::inverseMappingBoundary (*nurbs, pcp, error, pt, tu, tv, im_max_steps, im_accuracy);
    data->boundary_error.push_back (error);

    if (p < data->boundary_param.size ())
    {
      data->boundary_param[p] = params;
    }
    else
    {
      data->boundary_param.push_back (params);
    }

    n = tu.cross (tv);
    n.normalize ();

    data->boundary_normals.push_back (n);
    data->boundary_line_start.push_back (pcp);
    data->boundary_line_end.push_back (pt);

    addPointConstraint (id, ncps, params, pcp, weight, row);
  }
}

void
GlobalOptimization::assembleRegularisation (unsigned id, int ncps, double wCageRegInt, double wCageRegBnd,
                                            unsigned &row)
{
  if (wCageRegBnd <= 0.0 || wCageRegInt <= 0.0)
  {
    printf ("[GlobalOptimization::assembleRegularisation] Warning, no regularisation may lead "
      "to under-determined equation system. Add cage regularisation (smoothing) to avoid.\n");
  }

  if (wCageRegInt > 0.0)
    addCageInteriorRegularisation (id, ncps, wCageRegInt, row);

  if (wCageRegBnd > 0.0)
  {
    addCageBoundaryRegularisation (id, ncps, wCageRegBnd, NORTH, row);
    addCageBoundaryRegularisation (id, ncps, wCageRegBnd, SOUTH, row);
    addCageBoundaryRegularisation (id, ncps, wCageRegBnd, WEST, row);
    addCageBoundaryRegularisation (id, ncps, wCageRegBnd, EAST, row);
    addCageCornerRegularisation (id, ncps, wCageRegBnd * 2.0, row);
  }
}

void
GlobalOptimization::addParamConstraint (const Eigen::Vector2i &id, const Eigen::Vector2d &params1,
                                        const Eigen::Vector2d &params2, double weight, unsigned &row)
{
  vector_vec2d params;
  params.push_back (params1);
  params.push_back (params2);

  for (unsigned n = 0; n < 2; n++)
  {
    ON_NurbsSurface* nurbs = m_nurbs[id (n)];

    double *N0 = new double[nurbs->Order (0) * nurbs->Order (0)];
    double *N1 = new double[nurbs->Order (1) * nurbs->Order (1)];

    int E = ON_NurbsSpanIndex (nurbs->Order (0), nurbs->CVCount (0), nurbs->m_knot[0], params[n] (0), 0, 0);
    int F = ON_NurbsSpanIndex (nurbs->Order (1), nurbs->CVCount (1), nurbs->m_knot[1], params[n] (1), 0, 0);
    //    int E = ntools.E(params[n](0));
    //    int F = ntools.F(params[n](1));

    m_solver.f (row, 0, 0.0);
    m_solver.f (row, 1, 0.0);
    m_solver.f (row, 2, 0.0);

    int ncps (0);
    for (int i = 0; i < id (n); i++)
      ncps += m_nurbs[i]->CVCount ();

    ON_EvaluateNurbsBasis (nurbs->Order (0), nurbs->m_knot[0] + E, params[n] (0), N0);
    ON_EvaluateNurbsBasis (nurbs->Order (1), nurbs->m_knot[1] + F, params[n] (1), N1);

    double s (1.0);
    n == 0 ? s = 1.0 : s = -1.0;

    for (int i = 0; i < nurbs->Order (0); i++)
    {

      for (int j = 0; j < nurbs->Order (1); j++)
      {

        m_solver.K (row, ncps + lrc2gl (*nurbs, E, F, i, j), weight * N0[i] * N1[j] * s);

      } // j
    } // i

    delete [] N1;
    delete [] N0;
  }

  row++;

//  if (!m_quiet)
//    printf ("[GlobalOptimization::addParamConstraint] row: %d / %d\n", row, m_nrows);
}

void
GlobalOptimization::addPointConstraint (unsigned id, int ncps, const Eigen::Vector2d &params,
                                        const Eigen::Vector3d &point, double weight, unsigned &row)
{
  ON_NurbsSurface *nurbs = m_nurbs[id];

  double *N0 = new double[nurbs->Order (0) * nurbs->Order (0)];
  double *N1 = new double[nurbs->Order (1) * nurbs->Order (1)];

  int E = ON_NurbsSpanIndex (nurbs->Order (0), nurbs->CVCount (0), nurbs->m_knot[0], params (0), 0, 0);
  int F = ON_NurbsSpanIndex (nurbs->Order (1), nurbs->CVCount (1), nurbs->m_knot[1], params (1), 0, 0);

  ON_EvaluateNurbsBasis (nurbs->Order (0), nurbs->m_knot[0] + E, params (0), N0);
  ON_EvaluateNurbsBasis (nurbs->Order (1), nurbs->m_knot[1] + F, params (1), N1);

  m_solver.f (row, 0, point (0) * weight);
  m_solver.f (row, 1, point (1) * weight);
  m_solver.f (row, 2, point (2) * weight);

  for (int i = 0; i < nurbs->Order (0); i++)
  {

    for (int j = 0; j < nurbs->Order (1); j++)
    {

      m_solver.K (row, ncps + lrc2gl (*nurbs, E, F, i, j), weight * N0[i] * N1[j]);

    } // j

  } // i

  row++;

  //  if (!m_quiet && !(row % 100))
  //    printf("[GlobalOptimization::addPointConstraint] row: %d / %d\n", row, m_nrows);

  delete [] N1;
  delete [] N0;
}

void
GlobalOptimization::addCageInteriorRegularisation (unsigned id, int ncps, double weight, unsigned &row)
{
  ON_NurbsSurface *nurbs = m_nurbs[id];

  for (int i = 1; i < (nurbs->CVCount (0) - 1); i++)
  {
    for (int j = 1; j < (nurbs->CVCount (1) - 1); j++)
    {

      m_solver.f (row, 0, 0.0);
      m_solver.f (row, 1, 0.0);
      m_solver.f (row, 2, 0.0);

      m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 0), -4.0 * weight);
      m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j - 1), 1.0 * weight);
      m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 1), 1.0 * weight);
      m_solver.K (row, ncps + grc2gl (*nurbs, i - 1, j + 0), 1.0 * weight);
      m_solver.K (row, ncps + grc2gl (*nurbs, i + 1, j + 0), 1.0 * weight);

      row++;
    }
  }
  //  if (!m_quiet && !(row % 100))
  //    printf("[GlobalOptimization::addCageInteriorRegularisation] row: %d / %d\n", row, m_nrows);
}

void
GlobalOptimization::addCageBoundaryRegularisation (unsigned id, int ncps, double weight, int side, unsigned &row)
{
  ON_NurbsSurface *nurbs = m_nurbs[id];

  int i = 0;
  int j = 0;

  switch (side)
  {
    case SOUTH:
      j = nurbs->CVCount (1) - 1;
      PCL_FALLTHROUGH
    case NORTH:
      for (i = 1; i < (nurbs->CVCount (0) - 1); i++)
      {

        m_solver.f (row, 0, 0.0);
        m_solver.f (row, 1, 0.0);
        m_solver.f (row, 2, 0.0);

        m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j), -2.0 * weight);
        m_solver.K (row, ncps + grc2gl (*nurbs, i - 1, j), 1.0 * weight);
        m_solver.K (row, ncps + grc2gl (*nurbs, i + 1, j), 1.0 * weight);

        row++;
      }
      break;

    case EAST:
      i = nurbs->CVCount (0) - 1;
      PCL_FALLTHROUGH
    case WEST:
      for (j = 1; j < (nurbs->CVCount (1) - 1); j++)
      {

        m_solver.f (row, 0, 0.0);
        m_solver.f (row, 1, 0.0);
        m_solver.f (row, 2, 0.0);

        m_solver.K (row, ncps + grc2gl (*nurbs, i, j + 0), -2.0 * weight);
        m_solver.K (row, ncps + grc2gl (*nurbs, i, j - 1), 1.0 * weight);
        m_solver.K (row, ncps + grc2gl (*nurbs, i, j + 1), 1.0 * weight);

        row++;
      }
      break;
  }
  //  if (!m_quiet && !(row % 100))
  //    printf("[GlobalOptimization::addCageBoundaryRegularisation] row: %d / %d\n", row, m_nrows);
}

void
GlobalOptimization::addCageCornerRegularisation (unsigned id, int ncps, double weight, unsigned &row)
{
  ON_NurbsSurface *nurbs = m_nurbs[id];

  { // NORTH-WEST
    int i = 0;
    int j = 0;

    m_solver.f (row, 0, 0.0);
    m_solver.f (row, 1, 0.0);
    m_solver.f (row, 2, 0.0);

    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i + 1, j + 0), 1.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 1), 1.0 * weight);

    row++;
  }

  { // NORTH-EAST
    int i = nurbs->CVCount (0) - 1;
    int j = 0;

    m_solver.f (row, 0, 0.0);
    m_solver.f (row, 1, 0.0);
    m_solver.f (row, 2, 0.0);

    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i - 1, j + 0), 1.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 1), 1.0 * weight);

    row++;
  }

  { // SOUTH-EAST
    int i = nurbs->CVCount (0) - 1;
    int j = nurbs->CVCount (1) - 1;

    m_solver.f (row, 0, 0.0);
    m_solver.f (row, 1, 0.0);
    m_solver.f (row, 2, 0.0);

    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i - 1, j + 0), 1.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j - 1), 1.0 * weight);

    row++;
  }

  { // SOUTH-WEST
    int i = 0;
    int j = nurbs->CVCount (1) - 1;

    m_solver.f (row, 0, 0.0);
    m_solver.f (row, 1, 0.0);
    m_solver.f (row, 2, 0.0);

    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j + 0), -2.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i + 1, j + 0), 1.0 * weight);
    m_solver.K (row, ncps + grc2gl (*nurbs, i + 0, j - 1), 1.0 * weight);

    row++;
  }

  //  if (!m_quiet && !(row % 100))
  //    printf("[GlobalOptimization::addCageCornerRegularisation] row: %d / %d\n", row, m_nrows);
}

