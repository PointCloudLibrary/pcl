/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2009-2012, Willow Garage, Inc.
 *  Copyright (c) 2006, Michael Kazhdan and Matthew Bolitho,
 *                      Johns Hopkins University
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: function_data.h 5531 2012-04-08 09:14:33Z aichim $
 *
 */

#ifndef PCL_POISSON_FUNCTION_DATA_H_
#define PCL_POISSON_FUNCTION_DATA_H_


#include "ppolynomial.h"

namespace pcl 
{
  namespace poisson 
  {
   
    template<int Degree,class Real>
    class FunctionData
    {
      int useDotRatios;
      int normalize;

      public:
        const static int DOT_FLAG;
        const static int D_DOT_FLAG;
        const static int D2_DOT_FLAG;
        const static int VALUE_FLAG;
        const static int D_VALUE_FLAG;

        int depth, res, res2;
        Real *dotTable, *dDotTable, *d2DotTable;
        Real *valueTables, *dValueTables;
        PPolynomial<Degree> baseFunction;
        PPolynomial<Degree-1> dBaseFunction;
        PPolynomial<Degree+1>* baseFunctions;

        FunctionData (void);
        ~FunctionData (void);

        virtual void setDotTables (const int& flags);
        virtual void clearDotTables (const int& flags);

        virtual void setValueTables (const int& flags, const double& smooth = 0);
        virtual void setValueTables (const int& flags, const double& valueSmooth, const double& normalSmooth);
        virtual void clearValueTables (void);

        void set (const int& maxDepth, const PPolynomial<Degree>& F, const int& normalize, const int& useDotRatios = 1);

        Real dotProduct (const double& center1, const double& width1,
                         const double& center2, const double& width2) const;
        Real dDotProduct (const double& center1, const double& width1,
                          const double& center2, const double& width2) const;
        Real d2DotProduct (const double& center1, const double& width1,
                           const double& center2, const double& width2) const;

        static inline int SymmetricIndex (const int& i1, const int& i2);
        static inline int SymmetricIndex (const int& i1, const int& i2, int& index);
    };
  }
}

#include <pcl/surface/impl/poisson/function_data.hpp>
#endif /* PCL_POISSON_FUNCTION_DATA_H_ */
