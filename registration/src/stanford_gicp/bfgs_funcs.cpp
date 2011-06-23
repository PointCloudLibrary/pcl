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

namespace dgc {
  namespace gicp {
    // GICP cost function
    double GICPOptimizer::f(const gsl_vector *x, void *params) {
      GICPOptData *opt_data = (GICPOptData *)params;
      double pt1[3];
      double pt2[3]; 
      double res[3]; // residual
      double temp[3];
      gsl_vector_view gsl_pt1 = gsl_vector_view_array(pt1, 3);
      gsl_vector_view gsl_pt2 = gsl_vector_view_array(pt2, 3);
      gsl_vector_view gsl_res = gsl_vector_view_array(res, 3);
      gsl_vector_view gsl_temp = gsl_vector_view_array(temp, 3);
      gsl_matrix_view gsl_M;
      dgc_transform_t t;

      // initialize the temp variable; if it happens to be NaN at start, bad things will happen in blas routines below
      temp[0] = 0;
      temp[1] = 0;
      temp[2] = 0;
      

      // take the base transformation
      dgc_transform_copy(t, opt_data->base_t); 
      // apply the current state
      apply_state(t, x);
            
      double f = 0;
      double temp_double = 0;
      int N = opt_data->p1->Size();
      for(int i = 0; i < N; i++) {
	int j = opt_data->nn_indecies[i];	
	if(j != -1) {
	  // get point 1
	  pt1[0] = (*opt_data->p1)[i].x;
	  pt1[1] = (*opt_data->p1)[i].y;
	  pt1[2] = (*opt_data->p1)[i].z;
	  
	  // get point 2
	  pt2[0] = (*opt_data->p2)[j].x;
	  pt2[1] = (*opt_data->p2)[j].y;
	  pt2[2] = (*opt_data->p2)[j].z;
	  
	  //get M-matrix
	  gsl_M = gsl_matrix_view_array(&opt_data->M[i][0][0], 3, 3);
	  
	  
	  //transform point 1
	  dgc_transform_point(&pt1[0], &pt1[1], &pt1[2], t);
	  res[0] = pt1[0] - pt2[0];
	  res[1] = pt1[1] - pt2[1];
	  res[2] = pt1[2] - pt2[2];

	  //cout << "res: (" << res[0] << ", " <<res[1] <<", " << res[2] << ")" << endl;
	  
	  // temp := M*res
	  gsl_blas_dsymv(CblasLower, 1., &gsl_M.matrix, &gsl_res.vector, 0., &gsl_temp.vector);
	  // temp_double := res'*temp = temp'*M*temp
	  gsl_blas_ddot(&gsl_res.vector, &gsl_temp.vector, &temp_double);
	  // increment total error
	  
	  f += temp_double/(double)opt_data->num_matches;	  
	  //cout << "temp: " << temp_double << endl;
	  //cout << "f: " << f << "\t (" << opt_data->num_matches << ")" << endl;
	  //print_gsl_matrix(&gsl_M.matrix, "M");
	}
      }
      
      return f;
    }
    void GICPOptimizer::df(const gsl_vector *x, void *params, gsl_vector *g) {
      GICPOptData *opt_data = (GICPOptData *)params;
      double pt1[3];
      double pt2[3]; 
      double res[3]; // residual
      double temp[3]; // temp local vector
      double temp_mat[9]; // temp matrix used for accumulating the rotation gradient
      gsl_vector_view gsl_pt1 = gsl_vector_view_array(pt1, 3);
      gsl_vector_view gsl_pt2 = gsl_vector_view_array(pt2, 3);
      gsl_vector_view gsl_res = gsl_vector_view_array(res, 3);
      gsl_vector_view gsl_temp = gsl_vector_view_array(temp, 3);
      gsl_vector_view gsl_gradient_t = gsl_vector_subvector(g, 0, 3); // translation comp. of gradient
      gsl_matrix_view gsl_temp_mat_r = gsl_matrix_view_array(temp_mat, 3, 3);
      gsl_matrix_view gsl_M;
      dgc_transform_t t;
      double temp_double;
      
      // take the base transformation
      dgc_transform_copy(t, opt_data->base_t); 
      // apply the current state
      apply_state(t, x);
      
      // zero all accumulator variables
      gsl_vector_set_zero(g);
      gsl_vector_set_zero(&gsl_temp.vector);
      gsl_matrix_set_zero(&gsl_temp_mat_r.matrix);
            
      for(int i = 0; i < opt_data->p1->Size(); i++) {
	int j = opt_data->nn_indecies[i];	
	if(j != -1) {
	  // get point 1
	  pt1[0] = (*opt_data->p1)[i].x;
	  pt1[1] = (*opt_data->p1)[i].y;
	  pt1[2] = (*opt_data->p1)[i].z;
	  
	  // get point 2
	  pt2[0] = (*opt_data->p2)[j].x;
	  pt2[1] = (*opt_data->p2)[j].y;
	  pt2[2] = (*opt_data->p2)[j].z;
	  
	  //get M-matrix
	  gsl_M = gsl_matrix_view_array(&opt_data->M[i][0][0], 3, 3);	  

	  //transform point 1
	  dgc_transform_point(&pt1[0], &pt1[1], &pt1[2], t);
	  res[0] = pt1[0] - pt2[0];
	  res[1] = pt1[1] - pt2[1];
	  res[2] = pt1[2] - pt2[2];
	  
	  // temp := M*res
	  gsl_blas_dsymv(CblasLower, 1., &gsl_M.matrix, &gsl_res.vector, 0., &gsl_temp.vector);
	  // temp_double := res'*temp = temp'*M*res
	  gsl_blas_ddot(&gsl_res.vector, &gsl_temp.vector, &temp_double);

	  // increment total translation gradient:
	  // gsl_gradient_t += 2*M*res/num_matches
	  gsl_blas_dsymv(CblasLower, 2./(double)opt_data->num_matches, &gsl_M.matrix, &gsl_res.vector, 1., &gsl_gradient_t.vector);	  
	  
	  if(opt_data->solve_rotation) {
	    // compute rotation gradient here
	    // get back the original untransformed point to compute the rotation gradient
	    pt1[0] = (*opt_data->p1)[i].x;
	    pt1[1] = (*opt_data->p1)[i].y;
	    pt1[2] = (*opt_data->p1)[i].z;
	    dgc_transform_point(&pt1[0], &pt1[1], &pt1[2], opt_data->base_t);
	    gsl_blas_dger(2./(double)opt_data->num_matches, &gsl_pt1.vector, &gsl_temp.vector, &gsl_temp_mat_r.matrix);
	  }
	}
      }
      // the above loop sets up the gradient with respect to the translation, and the matrix derivative w.r.t. the rotation matrix
      // this code sets up the matrix derivatives dR/dPhi, dR/dPsi, dR/dTheta. i.e. the derivatives of the whole rotation matrix with respect to the euler angles
      // note that this code assumes the XYZ order of euler angles, with the Z rotation corresponding to bearing. This means the Z angle is negative of what it would be
      // in the regular XYZ euler-angle convention.
      
      // now use the d/dR matrix to compute the derivative with respect to euler angles and put it directly into g[3], g[4], g[5];
      if(opt_data->solve_rotation) {
	compute_dr(x, &gsl_temp_mat_r.matrix, g);
      }
    }
    
    void GICPOptimizer::fdf(const gsl_vector *x, void *params, double * f, gsl_vector *g) {
      GICPOptData *opt_data = (GICPOptData *)params;
      double pt1[3];
      double pt2[3]; 
      double res[3]; // residual
      double temp[3]; // temp local vector
      double temp_mat[9]; // temp matrix used for accumulating the rotation gradient
      gsl_vector_view gsl_pt1 = gsl_vector_view_array(pt1, 3);
      gsl_vector_view gsl_pt2 = gsl_vector_view_array(pt2, 3);
      gsl_vector_view gsl_res = gsl_vector_view_array(res, 3);
      gsl_vector_view gsl_temp = gsl_vector_view_array(temp, 3);
      gsl_vector_view gsl_gradient_t = gsl_vector_subvector(g, 0, 3); // translation comp. of gradient
      gsl_vector_view gsl_gradient_r = gsl_vector_subvector(g, 3, 3); // rotation comp. of gradient
      gsl_matrix_view gsl_temp_mat_r = gsl_matrix_view_array(temp_mat, 3, 3);
      gsl_matrix_view gsl_M;
      dgc_transform_t t;
      double temp_double;

      // take the base transformation
      dgc_transform_copy(t, opt_data->base_t); 
      // apply the current state      
      apply_state(t, x);
            
      // zero all accumulator variables
      *f = 0;
      gsl_vector_set_zero(g);
      gsl_vector_set_zero(&gsl_temp.vector);
      gsl_matrix_set_zero(&gsl_temp_mat_r.matrix);
      
      for(int i = 0; i < opt_data->p1->Size(); i++) {
	int j = opt_data->nn_indecies[i];	
	if(j != -1) {
	  // get point 1
	  pt1[0] = (*opt_data->p1)[i].x;
	  pt1[1] = (*opt_data->p1)[i].y;
	  pt1[2] = (*opt_data->p1)[i].z;
	  
	  // get point 2
	  pt2[0] = (*opt_data->p2)[j].x;
	  pt2[1] = (*opt_data->p2)[j].y;
	  pt2[2] = (*opt_data->p2)[j].z;
	  
	  //cout << "accessing " << i << " of " << opt_data->p1->Size() << ", " << opt_data->p2->Size() << endl;
	  //get M-matrix
	  gsl_M = gsl_matrix_view_array(&opt_data->M[i][0][0], 3, 3);	  
	  
	  //transform point 1
	  dgc_transform_point(&pt1[0], &pt1[1], &pt1[2], t);
	  res[0] = pt1[0] - pt2[0];
	  res[1] = pt1[1] - pt2[1];
	  res[2] = pt1[2] - pt2[2];
	  
	  // compute the transformed residual
	  // temp := M*res
	  //print_gsl_matrix(&gsl_M.matrix, "gsl_m");
	  gsl_blas_dsymv(CblasLower, 1., &gsl_M.matrix, &gsl_res.vector, 0., &gsl_temp.vector);
	  
	  // compute M-norm of the residual
	  // temp_double := res'*temp = temp'*M*res
	  gsl_blas_ddot(&gsl_res.vector, &gsl_temp.vector, &temp_double);
	  
	  // accumulate total error: f += res'*M*res
	  *f += temp_double/(double)opt_data->num_matches;
	  
	  // accumulate translation gradient:
	  // gsl_gradient_t += 2*M*res
	  gsl_blas_dsymv(CblasLower, 2./(double)opt_data->num_matches, &gsl_M.matrix, &gsl_res.vector, 1., &gsl_gradient_t.vector);	  
	  
	  if(opt_data->solve_rotation) {
	    // accumulate the rotation gradient matrix
	    // get back the original untransformed point to compute the rotation gradient
	    pt1[0] = (*opt_data->p1)[i].x;
	    pt1[1] = (*opt_data->p1)[i].y;
	    pt1[2] = (*opt_data->p1)[i].z;
	    dgc_transform_point(&pt1[0], &pt1[1], &pt1[2], opt_data->base_t);
	    // gsl_temp_mat_r += 2*(gsl_temp).(gsl_pt1)' [ = (2*M*residual).(gsl_pt1)' ]	  
	    gsl_blas_dger(2./(double)opt_data->num_matches, &gsl_pt1.vector, &gsl_temp.vector, &gsl_temp_mat_r.matrix); 
	  }
	}
      }      
      // the above loop sets up the gradient with respect to the translation, and the matrix derivative w.r.t. the rotation matrix
      // this code sets up the matrix derivatives dR/dPhi, dR/dPsi, dR/dTheta. i.e. the derivatives of the whole rotation matrix with respect to the euler angles
      // note that this code assumes the XYZ order of euler angles, with the Z rotation corresponding to bearing. This means the Z angle is negative of what it would be
      // in the regular XYZ euler-angle convention.
      if(opt_data->solve_rotation) {
	// now use the d/dR matrix to compute the derivative with respect to euler angles and put it directly into g[3], g[4], g[5];
	compute_dr(x, &gsl_temp_mat_r.matrix, g);
      }
    }
  }
}
