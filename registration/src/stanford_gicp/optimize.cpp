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



#include "optimize.h"

#include <iostream>
#include <assert.h>
#include <fstream>
#include <sstream>

using namespace std;

namespace dgc {
  namespace gicp {
    
    GICPOptimizer::GICPOptimizer() :
      T_min(gsl_multimin_fdfminimizer_vector_bfgs2)
    {
      max_iter = 20;
      iter = -1;
      status = GSL_CONTINUE;
      
      gsl_minimizer = gsl_multimin_fdfminimizer_alloc(T_min, N);
      
      if(gsl_minimizer == NULL) {
	//TODO: handle
      }
      x = gsl_vector_alloc(N);
      if(x == NULL) {
	//TODO: handle
      }
      debug = false;
    }

    GICPOptimizer::~GICPOptimizer() {
      if(gsl_minimizer != NULL) {
	gsl_multimin_fdfminimizer_free(gsl_minimizer);
      }
      if(x != NULL) {
	gsl_vector_free(x);
      }
    }

    // this method outputs data files containing the value of the error function on a 2d grid.
    // a seperate file and grid is constructed for each pair of coordinates
    // this is used to verify convergence of the numerical minimization
    void GICPOptimizer::PlotError(dgc_transform_t t, GICPOptData &opt_data, const char* filename) {
      double resolution = .005;
      double min = -.1;
      double max = .1;
      
      // initialize the starting point
      double tx, ty, tz, rx, ry, rz;
      dgc_transform_get_translation(t, &tx, &ty, &tz);
      dgc_transform_get_rotation(t, &rx, &ry, &rz);
      gsl_vector_set(x, 0, tx);
      gsl_vector_set(x, 1, ty);
      gsl_vector_set(x, 2, tz);
      gsl_vector_set(x, 3, rx);
      gsl_vector_set(x, 4, ry);
      gsl_vector_set(x, 5, rz);
      
      dgc_transform_t temp;

      int N = (max-min)/resolution;

      for(int n1 = 0; n1 < 6; n1++) {
	for(int n2 = 0; n2 < n1; n2++) {
	  // set up the filename for this pair of coordinates
	  ostringstream full_filename;
	  full_filename << filename << "_" << n1 << "vs" << n2 << ".dat";
	  // open the file
	  ofstream fout(full_filename.str().c_str());
	  if(!fout) {
	    cout << "Could not open file for writing." << endl;
	    return;
	  }
	  
	  // loop through pairs of values for these two coordinates while keeping others fixed
	  double val1_0 = gsl_vector_get(x, n1);
	  for(int k1 = 0; k1 < N; k1++) {    
	    gsl_vector_set(x, n1, val1_0 + min + resolution*k1);
	    
	    double val2_0 = gsl_vector_get(x, n2);
	    for(int k2 = 0; k2 < N; k2++) {
	      gsl_vector_set(x, n2, val2_0 + min + resolution*k2);
	      
	      dgc_transform_copy(temp, opt_data.base_t);
	      apply_state(temp, x);
	  
	      double error = f(x, &opt_data);
	      fout << error << "\t";	      		
	    }
	    gsl_vector_set(x, n2, val2_0); // restore to old value
	    
	    fout << endl;
	  }
	  gsl_vector_set(x, n1, val1_0); // restore to old value

	  // close the file for this pair of coordinates
	  fout.close();
	}
      }
    }

    bool GICPOptimizer::Optimize(dgc_transform_t t, GICPOptData & opt_data) {
      double line_search_tol = .01;
      double gradient_tol = 1e-2;
      double step_size = 1.;
                  
      // set up the gsl function_fdf struct
      gsl_multimin_function_fdf func;
      func.f = f;
      func.df = df;
      func.fdf = fdf;
      func.n = N;
      func.params = &opt_data;
      
      // initialize the starting point
      double tx, ty, tz, rx, ry, rz;
      dgc_transform_get_translation(t, &tx, &ty, &tz);
      dgc_transform_get_rotation(t, &rx, &ry, &rz);
      gsl_vector_set(x, 0, tx);
      gsl_vector_set(x, 1, ty);
      gsl_vector_set(x, 2, tz);
      gsl_vector_set(x, 3, rx);
      gsl_vector_set(x, 4, ry);
      gsl_vector_set(x, 5, rz);
      
      // initialize the minimizer
      gsl_multimin_fdfminimizer_set(gsl_minimizer, &func, x, step_size, line_search_tol);
      
      //iterate the minimization algorithm using gsl primatives
      status = GSL_CONTINUE;
      iter = 0;
      if(debug) {
	cout << "iter\t\tf-value\t\tstatus" << endl;
	cout << iter << "\t\t" << gsl_minimizer->f << "\t\t" << Status() << endl;
      }
      while(status == GSL_CONTINUE && iter < max_iter) {
	iter++;
	status = gsl_multimin_fdfminimizer_iterate(gsl_minimizer);
	if(debug) {
	  cout << iter << "\t\t" << gsl_minimizer->f << "\t\t" << Status() <<endl;
	}
	if(status) {
	  break;
	}
	status = gsl_multimin_test_gradient (gsl_minimizer->gradient, gradient_tol);
      }
      
      if(status == GSL_SUCCESS || iter == max_iter) {
	//set t to the converged solution
	
	dgc_transform_identity(t);
	// apply the current state to the base
	apply_state(t, gsl_minimizer->x);
	//dgc_transform_print(t, "converged to:");	
	return true;
      }
      else {
	// the algorithm failed to converge
	return false;
      }
    }
    

    double GICPOptimizer::mat_inner_prod(gsl_matrix const* mat1, gsl_matrix const* mat2) {
      double r = 0.;
      int n = mat1->size1;
      
      for(int i = 0; i < n; i++) {
	for(int j = 0; j < n; j++) { // tr(mat1^t.mat2)
	  r += gsl_matrix_get(mat1, j, i)*gsl_matrix_get(mat2, i, j);
	}
      }
      
      return r;
    }
    
    void GICPOptimizer::apply_state(dgc_transform_t t, gsl_vector const* x) {
      double tx, ty, tz, rx, ry, rz;
      tx = gsl_vector_get(x, 0);
      ty = gsl_vector_get(x, 1);
      tz = gsl_vector_get(x, 2);
      rx = gsl_vector_get(x, 3);
      ry = gsl_vector_get(x, 4);
      rz = gsl_vector_get(x, 5);

      dgc_transform_rotate_x(t, rx);
      dgc_transform_rotate_y(t, ry);
      dgc_transform_rotate_z(t, rz);      
      dgc_transform_translate(t, tx, ty, tz);
    }
    
    void GICPOptimizer::compute_dr(gsl_vector const* x, gsl_matrix const* gsl_temp_mat_r, gsl_vector *g) {
      double dR_dPhi[3][3];
      double dR_dTheta[3][3];
      double dR_dPsi[3][3];
      gsl_matrix_view gsl_d_rx = gsl_matrix_view_array(&dR_dPhi[0][0],3, 3);
      gsl_matrix_view gsl_d_ry = gsl_matrix_view_array(&dR_dTheta[0][0],3, 3);
      gsl_matrix_view gsl_d_rz = gsl_matrix_view_array(&dR_dPsi[0][0],3, 3);

      double phi = gsl_vector_get(x ,3);
      double theta = gsl_vector_get(x ,4);
      double psi = gsl_vector_get(x ,5);  
      
      double cphi = cos(phi), sphi = sin(phi);
      double ctheta = cos(theta), stheta = sin(theta);
      double cpsi = cos(psi), spsi = sin(psi);
      
      dR_dPhi[0][0] = 0.;
      dR_dPhi[1][0] = 0.;
      dR_dPhi[2][0] = 0.;
      
      dR_dPhi[0][1] = sphi*spsi + cphi*cpsi*stheta;
      dR_dPhi[1][1] = -cpsi*sphi + cphi*spsi*stheta;
      dR_dPhi[2][1] = cphi*ctheta;
      
      dR_dPhi[0][2] = cphi*spsi - cpsi*sphi*stheta;
      dR_dPhi[1][2] = -cphi*cpsi - sphi*spsi*stheta;
      dR_dPhi[2][2] = -ctheta*sphi;
      
      dR_dTheta[0][0] = -cpsi*stheta;
      dR_dTheta[1][0] = -spsi*stheta;
      dR_dTheta[2][0] = -ctheta;
      
      dR_dTheta[0][1] = cpsi*ctheta*sphi;
      dR_dTheta[1][1] = ctheta*sphi*spsi;
      dR_dTheta[2][1] = -sphi*stheta;
	
      dR_dTheta[0][2] = cphi*cpsi*ctheta;
      dR_dTheta[1][2] = cphi*ctheta*spsi;
      dR_dTheta[2][2] = -cphi*stheta;
      
      dR_dPsi[0][0] = -ctheta*spsi;
      dR_dPsi[1][0] = cpsi*ctheta;
      dR_dPsi[2][0] = 0.;
      
      dR_dPsi[0][1] = -cphi*cpsi - sphi*spsi*stheta;
      dR_dPsi[1][1] = -cphi*spsi + cpsi*sphi*stheta;
      dR_dPsi[2][1] = 0.;
      
      dR_dPsi[0][2] = cpsi*sphi - cphi*spsi*stheta;
      dR_dPsi[1][2] = sphi*spsi + cphi*cpsi*stheta;
      dR_dPsi[2][2] = 0.;
      
      // set d/d_rx = tr(dR_dPhi'*gsl_temp_mat_r) [= <dR_dPhi, gsl_temp_mat_r>]
      gsl_vector_set(g, 3, mat_inner_prod(&gsl_d_rx.matrix, gsl_temp_mat_r));
      // set d/d_ry = tr(dR_dTheta'*gsl_temp_mat_r) = [<dR_dTheta, gsl_temp_mat_r>]
      gsl_vector_set(g, 4, mat_inner_prod(&gsl_d_ry.matrix, gsl_temp_mat_r));
      // set d/d_rz = tr(dR_dPsi'*gsl_temp_mat_r) = [<dR_dPsi, gsl_temp_mat_r>]
      gsl_vector_set(g, 5, mat_inner_prod(&gsl_d_rz.matrix, gsl_temp_mat_r));      

    }

  }
}
