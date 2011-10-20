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
 * @author thomas.moerwald
 *
 */

#ifndef _NURBS_FITTING_H_
#define _NURBS_FITTING_H_

#include "pcl/surface/nurbs/NurbsTools.h"
#include "pcl/surface/nurbs/NurbsData.h"

namespace nurbsfitting {

class NurbsFitting
{
public:
  ON_TextLog m_out;
  ON_NurbsSurface* m_patch;
  NurbsData *data;

  NurbsFitting(int order, NurbsData *data, ON_3dPoint ll, ON_3dPoint lr, ON_3dPoint ur, ON_3dPoint ul);
  NurbsFitting(int order, NurbsData *data, const std::vector<ON_3dPoint> &cv);
  NurbsFitting(NurbsData *data, const ON_NurbsSurface &ns);
  NurbsFitting(int order, NurbsData *data);
  ~NurbsFitting();

  void refine(int dim);
  //	void assemble_minimal_surface(int resU=64, int resV=64);
  void assemble();
  void assemble(std::vector<double> &wBnd, std::vector<double> &wInt, double wCageRegBnd, double wCageRegInt);
  void assemble(int resU, int resV, double wBnd, double wInt, double wCurBnd, double wCurInt, double wCageRegBnd,
      double wCageReg, double wCorner);

  void solve(double damp = 1.0);

  void updateSurf(double damp);

  void setInvMapParams(double invMapBnd_maxSteps, double invMapInt_maxSteps, double invMapBnd_accuracy,
      double invMapInt_accuracy);

protected:

  void init();

  void solve_eigen(double damp);

  void addPointConstraint(Eigen::Vector2d params, Eigen::Vector3d point, double weight, int& row);
  void addBoundaryPointConstraint(double paramU, double paramV, double weight, int &row);

  void addCageInteriorRegularisation(double weight, int &row);
  void addCageBoundaryRegularisation(double weight, int side, int &row);
  void addCageCornerRegularisation(double weight, int &row);

  void addInteriorRegularisation(int order, int resU, int resV, double weight, int &row);
  void addBoundaryRegularisation(int order, int resU, int resV, double weight, int &row);

  bool m_quiet;
  bool use_int_hints;

  Eigen::MatrixXd m_xeig;
  Eigen::MatrixXd m_feig;
  Eigen::MatrixXd m_Keig;

  std::vector<double> m_elementsU;
  std::vector<double> m_elementsV;

  double m_minU;
  double m_minV;
  double m_maxU;
  double m_maxV;

  int invMapBnd_maxSteps;
  int invMapInt_maxSteps;
  double invMapBnd_accuracy;
  double invMapInt_accuracy;
};

} // namespace nurbsfitting

#endif /* _NURBS_FITTING_H_ */
