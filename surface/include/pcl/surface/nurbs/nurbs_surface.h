/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Thomas Mörwald (University of Technology Vienna)
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
 *   * Neither the name of Thomas Mörwald nor the names of its
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

#ifndef _NURBS_SURFACE_H_
#define _NURBS_SURFACE_H_

#include <pcl/pcl_macros.h>

#include "nurbs_basis.h"
#include "eigen_defs.h"

namespace pcl
{
  namespace nurbs
  {
    class PCL_EXPORTS NurbsSurface
    {

      public:
        NurbsBasis basis_u_;
        NurbsBasis basis_v_;

        std::size_t nb_control_points_u_;
        std::size_t nb_control_points_v_;

        vector_vec4 control_points_;

      public:
        NurbsSurface ()
        {}

        NurbsSurface (std::size_t degree,
                      std::size_t nb_control_points_u,
                      std::size_t nb_control_points_v,
                      const vector_vec4 &control_points)
        : basis_u_ (degree, nb_control_points_u)
        , basis_v_ (degree, nb_control_points_v)
        , nb_control_points_u_ (nb_control_points_u)
        , nb_control_points_v_ (nb_control_points_v)
        , control_points_ (control_points_)
        {}

        void
        evaluate (double u, double v, vec3 &point) const;

        void
        evaluate (double u, double v, vec3 &point, vec3 &grad_u, vec3 &grad_v) const;

        void
        insertKnotU (double u);

        void
        insertKnotV (double v);

        /** \return the linear coordinate
          * \param i coordinate in U
          * \param j coordinate in V
          */
        std::size_t
        index (std::size_t i, std::size_t j) const;

        /** \return control point at coordinate (i,j)
          * \param i coordinate in U
          * \param j coordinate in V
          */
        vec4
        getControlPoint (std::size_t i, std::size_t j) const;

        /** set control point at coordinate (i,j)
          * \param i coordinate in U
          * \param j coordinate in V
          * \param cp control point
          */
        void
        setControlPoint (std::size_t i, std::size_t j, const vec4 &cp);

        inline std::size_t
        nbControlPointsU () const
        {
          return nb_control_points_u_;
        }

        inline std::size_t
        nbControlPointsV () const
        {
          return nb_control_points_v_;
        }

        inline std::size_t
        degreeU () const
        {
          return basis_u_.degree ();
        }

        inline std::size_t
        degreeV () const
        {
            return basis_v_.degree ();
        }

        inline void
        getElementVectorU (std::vector<double> &result) const
        {
          basis_u_.getElementVector (result);
        }

        inline void
        getElementVectorV (std::vector<double> &result) const
        {
          basis_v_.getElementVector (result);
        }

        void
        dump () const;

    };
  }
}

#endif
