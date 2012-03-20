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

#ifndef _NURBS_BASIS_H_
#define _NURBS_BASIS_H_

#include <pcl/pcl_macros.h>

#include <vector>
#include <stdexcept>

namespace pcl
{
  namespace nurbs
  {
    class PCL_EXPORTS NurbsBasis
    {

    protected:
      std::size_t degree_; ///< polynomial degree
      std::vector<double> knots_; ///< knot vector

      static void
      cox (const double &xi, int knotSpan, std::size_t degree, const std::vector<double> &supKnot, std::vector<double> &N);

      static void
      coxder (std::size_t degree, const std::vector<double> &supKnot, const std::vector<double> &N,
              std::vector<double> &Nd);

    public:
      NurbsBasis ()
      {}

      NurbsBasis (std::size_t order, std::size_t ncps);

      /** @brief Find the knot span to a parameter xi */
      int
      getSpan (const double &xi) const;

      void
      getElementVector (std::vector<double> &result) const;

      void
      insertKnot (const double &xi);

      std::size_t
      nbKnots () const
      {
        return (knots_.size ());
      }

      const double &
      knot (std::size_t i) const
      {
        return (knots_ [i]);
      }

      inline size_t
      degree () const
      {
          return (degree_);
      }
      /** @brief Cox-De-Boor recursion formula */
      void
      cox (const double &xi, std::vector<double> &N) const;
      void
      cox (const double &xi, std::vector<double> &N, std::vector<double> &Nd) const;
      void
      cox (const double &xi, int span, std::vector<double> &N) const;
      void
      cox (const double &xi, int span, std::vector<double> &N, std::vector<double> &Nd) const;
    };
  }
}

#endif
