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


#ifndef OPTIMIZE_H_
#define OPTIMIZE_H_

#include <ANN.h>
#include "gicp.h"
#include <vector>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_multifit_nlin.h>

namespace dgc {
  namespace gicp {
    inline void print_gsl_matrix(gsl_matrix *mat, const char * name) {
      std::cout << name << "= [";
      for(unsigned int i = 0; i < mat->size1; i++) {
	for(unsigned int j = 0; j < mat->size2; j++) {
	  std::cout << gsl_matrix_get(mat, i, j) << " ";
	}
	std::cout << ";" << std::endl;
      }
      std::cout << "]" << std::endl;
    }    
    
    struct GICPOptData {
      GICPPointSet *p1;
      GICPPointSet *p2;
      ANNidx *nn_indecies; // nearest point indecies
      gicp_mat_t *M;      // mahalanobis matrices for each pair
      dgc_transform_t base_t;
      int num_matches;
      bool solve_rotation;
    };
    
    class GICPOptimizer {
    public:
      GICPOptimizer();
      ~GICPOptimizer();

      int Iterations() { return iter; }
      const char* Status() { return gsl_strerror(status); }
      
      bool Optimize(dgc_transform_t t, GICPOptData &opt_data);
      bool OptimizeLM(dgc_transform_t t, GICPOptData &opt_data);
      
      void SetDebug(bool d) { debug = d; }
      
      void SetMaxIterations(int iter) { max_iter = iter; }

      void PlotError(dgc_transform_t t, GICPOptData &opt_data, const char* filename);
      
    private:
      static double f(const gsl_vector * x, void * params);
      static void df(const gsl_vector * x, void * params, gsl_vector * g);
      static void fdf(const gsl_vector * x, void * params, double * f, gsl_vector * g);

      static void compute_dr(gsl_vector const* x, gsl_matrix const* gsl_temp_mat_r, gsl_vector *g);
      static double mat_inner_prod(gsl_matrix const* mat1, gsl_matrix const* mat2);
      static void apply_state(dgc_transform_t t, gsl_vector const* x);
      
      gsl_multimin_fdfminimizer *gsl_minimizer;
      gsl_vector *x;
      int max_iter;
      int iter;
      int status;
      bool debug;
      const static int N = 6;
      const gsl_multimin_fdfminimizer_type *T_min;
    };
    
  }
}


#endif



