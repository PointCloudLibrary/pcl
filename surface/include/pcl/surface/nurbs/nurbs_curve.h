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

#ifndef _NURBS_CURVE_H_
#define _NURBS_CURVE_H_

#include <pcl/pcl_macros.h>

#include "eigen_defs.h"
#include "nurbs_basis.h"

namespace pcl
{
  namespace nurbs
  {

    class PCL_EXPORTS NurbsCurve
    {

    private:
      NurbsBasis basis;
      vector_vec4 cps;

    public:
      NurbsCurve ();
      NurbsCurve (unsigned degree, const vector_vec4 &cps);

      void
      Evaluate (double xi, vec3 &point) const;

      void
      Evaluate (double xi, vec3 &point, vec3 &grad) const;

      void
      InsertKnot (double xi);

      inline vec4
      GetCP (unsigned i) const
      {
        return cps[i];
      }

      inline void
      SetCP (unsigned i, const vec4 &cp)
      {
        cps[i] = cp;
      }

      inline unsigned
      CountCP () const
      {
        return cps.size ();
      }

      inline unsigned
      Degree () const
      {
        return basis.degree;
      }

      inline void
      GetElementVector (std::vector<double> &result) const
      {
        basis.GetElementVector (result);
      }

      void
      Dump () const;
    };
  }
}

#endif
