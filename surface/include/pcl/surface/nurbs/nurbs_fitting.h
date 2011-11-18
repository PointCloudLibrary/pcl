/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald, Jonathan Balzer
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
 * @author thomas.moerwald
 *
 */

#ifndef _NURBS_FITTING_H_
#define _NURBS_FITTING_H_

#include "pcl/surface/nurbs/nurbs_tools.h"
#include "pcl/surface/nurbs/nurbs_data.h"

namespace pcl
{

  class NurbsFitting
  {
  public:
    ON_TextLog on_out_;
    ON_NurbsSurface* nurbs_patch_;
    NurbsData *nurbs_data_;

    NurbsFitting (int order, NurbsData *nurbs_data_, ON_3dPoint ll, ON_3dPoint lr, ON_3dPoint ur, ON_3dPoint ul);
    NurbsFitting (int order, NurbsData *nurbs_data_, const std::vector<ON_3dPoint> &cv);
    NurbsFitting (NurbsData *nurbs_data_, const ON_NurbsSurface &ns);
    NurbsFitting (int order, NurbsData *nurbs_data_);
    ~NurbsFitting ();

    void
    refine (int dim);
    //	void assemble_minimal_surface(int resU=64, int resV=64);
    void
    assemble ();
    void
    assemble (std::vector<double> &wBnd, std::vector<double> &wInt, double wCageRegBnd, double wCageRegInt);
    void
    assemble (int resU, int resV, double wBnd, double wInt, double wCurBnd, double wCurInt, double wCageRegBnd,
              double wCageReg, double wCorner);

    void
    solve (double damp = 1.0);

    void
    updateSurf (double damp);

    void
    setInvMapParams (double inv_map_iter_bnd_, double inv_map_iter_int, double inv_map_accuracy_bnd,
                     double inv_map_accuracy_int_);

  protected:

    void
    init ();

    void
    solve_eigen (double damp);

    void
    addPointConstraint (Eigen::Vector2d params, Eigen::Vector3d point, double weight, int& row);

    void
    addBoundaryPointConstraint (double paramU, double paramV, double weight, int &row);

    void
    addCageInteriorRegularisation (double weight, int &row);

    void
    addCageBoundaryRegularisation (double weight, int side, int &row);

    void
    addCageCornerRegularisation (double weight, int &row);

    void
    addInteriorRegularisation (int order, int resU, int resV, double weight, int &row);

    void
    addBoundaryRegularisation (int order, int resU, int resV, double weight, int &row);

    bool quiet_;
    bool use_int_hints_;

    Eigen::MatrixXd x_eig_;
    Eigen::MatrixXd f_eig_;
    Eigen::MatrixXd K_eig_;

    std::vector<double> m_elementsU;
    std::vector<double> m_elementsV;

    double min_u_;
    double min_v_;
    double max_u_;
    double max_v_;

    int inv_map_iter_bnd_;
    int inv_map_iter_int_;
    double inv_map_accuracy_bnd_;
    double inv_map_accuracy_int_;
  };

} // namespace pcl_nurbs

#endif /* _NURBS_FITTING_H_ */
