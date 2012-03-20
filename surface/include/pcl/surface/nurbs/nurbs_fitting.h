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

#include <pcl/pcl_macros.h>

#include "nurbs_tools.h"
#include "nurbs_data.h"

namespace pcl
{
  namespace nurbs
  {
    class PCL_EXPORTS NurbsFitting
    {
      public:
        NurbsSurface* m_patch;
        NurbsData *data;

        NurbsFitting (int order, NurbsData *data, vec4 ll, vec4 lr, vec4 ur, vec4 ul);
        NurbsFitting (int order, NurbsData *data, const vector_vec3 &cv);
        NurbsFitting (NurbsData *data, const NurbsSurface &ns);
        NurbsFitting (int order, NurbsData *data, vec3 z = vec3(0.0, 0.0, 1.0));
        ~NurbsFitting ();

        void
        refine (int dim);
        //    void assemble_minimal_surface(int resU=64, int resV=64);
        void
        assemble (double smoothness = 0.00001);

        void
        assemble (std::vector<double> &wBnd, std::vector<double> &wInt, double wCageRegBnd, double wCageRegInt);

        void
        assemble (int resU, int resV, double wBnd, double wInt, double wCurBnd, double wCurInt, double wCageRegBnd,
                  double wCageReg, double wCorner);

        void
        solve (double damp = 1.0);

        void
        updateSurface (double damp);

        void
        setInvMapParams(double invMapBnd_maxSteps, double invMapInt_maxSteps, double invMapBnd_accuracy,
                        double invMapInt_accuracy);

        //protected:

        void
        init();

#ifdef USE_UMFPACK
        void solve_umfpack(double damp);
#else
        void
        solveEigen(double damp);
#endif

        void
        addPointConstraint(const vec2 &params, const vec3 &point, double weight, int& row);
        void
        addBoundaryPointConstraint(double paramU, double paramV, double weight, int &row);

        void
        addCageInteriorRegularisation(double weight, int &row);
        void
        addCageBoundaryRegularisation(double weight, int side, int &row);
        void
        addCageCornerRegularisation(double weight, int &row);

        void
        addInteriorRegularisation(int order, int resU, int resV, double weight, int &row);
        void
        addBoundaryRegularisation(int order, int resU, int resV, double weight, int &row);

        bool quiet_;
        bool use_int_hints_;

#ifdef USE_UMFPACK
        SparseMat m_Ksparse;
#endif

        Eigen::MatrixXd m_xeig;
        Eigen::MatrixXd m_feig;
        Eigen::MatrixXd m_Keig;

        std::vector<double> m_elementsU;
        std::vector<double> m_elementsV;

        double min_u_;
        double min_v_;
        double max_u_;
        double max_v_;

        int invMapBnd_maxSteps;
        int invMapInt_maxSteps;
        double invMapBnd_accuracy;
        double invMapInt_accuracy;
    };
  }
} // namespace nurbs

#endif /* _NURBS_FITTING_H_ */
