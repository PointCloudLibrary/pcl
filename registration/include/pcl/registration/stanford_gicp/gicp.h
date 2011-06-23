/*************************************************************
  Generalized-ICP Copyright (c) 2009 Aleksandr Segal.
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
*************************************************************/



#ifndef GICP_H_
#define GICP_H_

#include <ANN.h>
#include <vector>
#include <iostream>
//#include <gsl/gsl.h>
#include <semaphore.h>
#include "transform.h"

namespace dgc {
  namespace gicp {
    typedef double gicp_mat_t[3][3];
    
    struct GICPPoint {
      double x, y, z;
      float range;
      gicp_mat_t C; // covariance matrix
    };
    
    class GICPPointSet {
    public:
      GICPPointSet();
      ~GICPPointSet();
      void BuildKDTree();
      void ComputeMatrices();
      
      void SavePoints(const char *filename);
      void SaveMatrices(const char *filename);
      
      int NumPoints() { return (int)point_.size(); }
      void Clear(void);
      int Size() { return point_.size(); }
      inline void AppendPoint(GICPPoint const & pt) { point_.push_back(pt); }
      void SetMaxIteration(int iter) { max_iteration_ = iter; }
      void SetMaxIterationInner(int iter) { max_iteration_inner_ = iter; }
      void SetEpsilon(double eps) { epsilon_ = eps; }
      void SetSolveRotation(bool s) { solve_rotation_ = s; }
      void SetGICPEpsilon(double eps) { gicp_epsilon_ = eps; }
      void SetDebug(bool d) { debug_ = d; }


      GICPPoint & operator[](int i) { return point_[i]; }
      GICPPoint const& operator[](int i) const { return point_[i]; }
      
      // returns number of iterations it took to converge
      int AlignScan(GICPPointSet *scan, dgc_transform_t base_t, dgc_transform_t t, double max_match_dist, bool save_error_plot = 0);

    private:
      std::vector <GICPPoint> point_;
      ANNpointArray kdtree_points_;
      ANNkd_tree *kdtree_;
      int max_iteration_;
      int max_iteration_inner_;
      double epsilon_;
      double epsilon_rot_;
      double gicp_epsilon_;
      bool debug_;
      bool solve_rotation_;
      bool matrices_done_;
      bool kdtree_done_;
      pthread_mutex_t mutex_;
    };
    
  }
  
}

#endif
