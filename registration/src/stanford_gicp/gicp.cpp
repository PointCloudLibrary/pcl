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



#include "gicp.h"
#include "optimize.h"
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <fstream>

#include <iostream> //TODO: remove
#include <sstream>
#include <pthread.h>


using namespace std;//TODO: remove

namespace dgc {
  namespace gicp {

    GICPPointSet::GICPPointSet()
    {
      kdtree_points_ = NULL;
      kdtree_ = NULL;
      max_iteration_ = 200; // default value
      max_iteration_inner_ = 20; // default value for inner loop
      epsilon_ = 5e-4; // correspondes to ~1 mm (tolerence for convergence of GICP outer loop)
      epsilon_rot_ = 2e-3;
      //gicp_epsilon_ = .0004; // epsilon constant for gicp paper; this is NOT the convergence tolerence
      gicp_epsilon_ = .0004; // epsilon constant for gicp paper; this is NOT the convergence tolerence
      debug_ = false;
      solve_rotation_ = true;
      matrices_done_ = false;
      kdtree_done_ = false;
      pthread_mutex_init(&mutex_, NULL);
    }

    GICPPointSet::~GICPPointSet()
    {
      if (kdtree_ != NULL)
	delete kdtree_;
      if (kdtree_points_ != NULL)
	annDeallocPts(kdtree_points_);
    }
    
    void GICPPointSet::Clear(void) {
      pthread_mutex_lock(&mutex_);
      matrices_done_ = false;
      kdtree_done_ = false;
      if (kdtree_ != NULL) {
	delete kdtree_;
	kdtree_ = NULL;
      }
      if (kdtree_points_ != NULL) {
	annDeallocPts(kdtree_points_);
	kdtree_points_ = NULL;
      }
      point_.clear();
     pthread_mutex_unlock(&mutex_);

    }
    
    void GICPPointSet::BuildKDTree(void)
    {
      pthread_mutex_lock(&mutex_);
      if(kdtree_done_) {
	return;
      }
      kdtree_done_ = true;
     pthread_mutex_unlock(&mutex_);
      
      int i, n = NumPoints();
      
      if(n == 0) {
	return;
      }

      kdtree_points_ = annAllocPts(n, 3);
      for(i = 0; i < n; i++) {
	kdtree_points_[i][0] = point_[i].x;
	kdtree_points_[i][1] = point_[i].y;
	kdtree_points_[i][2] = point_[i].z;
      }
      kdtree_ = new ANNkd_tree(kdtree_points_, n, 3, 10);
    }
    
    void GICPPointSet::ComputeMatrices() {
      pthread_mutex_lock(&mutex_);
      if(kdtree_ == NULL) {
	return;
      }
      if(matrices_done_) {
	return;
      }
      matrices_done_ = true;
      pthread_mutex_unlock(&mutex_);

      int N  = NumPoints();
      int K = 20; // number of closest points to use for local covariance estimate
      double mean[3];
      
      ANNpoint query_point = annAllocPt(3);
      
      ANNdist *nn_dist_sq = new ANNdist[K];
      if(nn_dist_sq == NULL) {
	//TODO: handle this
      }      
      ANNidx *nn_indecies = new ANNidx[K];
      if(nn_indecies == NULL) {
	//TODO: handle this
      }
      gsl_vector *work = gsl_vector_alloc(3);
      if(work == NULL) {
	//TODO: handle
      }
      gsl_vector *gsl_singulars = gsl_vector_alloc(3);
      if(gsl_singulars == NULL) {
	//TODO: handle
      }
      gsl_matrix *gsl_v_mat = gsl_matrix_alloc(3, 3);
      if(gsl_v_mat == NULL) {
	//TODO: handle
      }
      
      for(int i = 0; i < N; i++) {
	query_point[0] = point_[i].x;
	query_point[1] = point_[i].y;
	query_point[2] = point_[i].z;
	
	gicp_mat_t &cov = point_[i].C;
	// zero out the cov and mean
	for(int k = 0; k < 3; k++) {
	  mean[k] = 0.;
	  for(int l = 0; l < 3; l++) {
	    cov[k][l] = 0.;
	  }
	}
	
	kdtree_->annkSearch(query_point, K, nn_indecies, nn_dist_sq, 0.0);
	
	// find the covariance matrix
	for(int j = 0; j < K; j++) {
	  GICPPoint &pt = point_[nn_indecies[j]];
	  
	  mean[0] += pt.x;
	  mean[1] += pt.y;
	  mean[2] += pt.z;

	  cov[0][0] += pt.x*pt.x;
	  
	  cov[1][0] += pt.y*pt.x;
	  cov[1][1] += pt.y*pt.y;
	  
	  cov[2][0] += pt.z*pt.x;
	  cov[2][1] += pt.z*pt.y;
	  cov[2][2] += pt.z*pt.z;	  
	}
	
	mean[0] /= (double)K;
	mean[1] /= (double)K;
	mean[2] /= (double)K;
	// get the actual covariance
	for(int k = 0; k < 3; k++) {
	  for(int l = 0; l <= k; l++) {
	    cov[k][l] /= (double)K;
	    cov[k][l] -= mean[k]*mean[l];
	    cov[l][k] = cov[k][l];
	  }
	}
	
	// compute the SVD
	gsl_matrix_view gsl_cov = gsl_matrix_view_array(&cov[0][0], 3, 3);
	gsl_linalg_SV_decomp(&gsl_cov.matrix, gsl_v_mat, gsl_singulars, work);
	
	// zero out the cov matrix, since we know U = V since C is symmetric
	for(int k = 0; k < 3; k++) {
	  for(int l = 0; l < 3; l++) {
	    cov[k][l] = 0;
	  }
	}
	
	// reconstitute the covariance matrix with modified singular values using the column vectors in V.
	for(int k = 0; k < 3; k++) {
	  gsl_vector_view col = gsl_matrix_column(gsl_v_mat, k);

	  double v = 1.; // biggest 2 singular values replaced by 1
	  if(k == 2) {   // smallest singular value replaced by gicp_epsilon
	    v = gicp_epsilon_;
	  }
	  
	  gsl_blas_dger(v, &col.vector, &col.vector, &gsl_cov.matrix); 
	}
      }

      if(nn_dist_sq != NULL) {
	delete [] nn_dist_sq;
      }
      if(nn_indecies != NULL) {
	delete [] nn_indecies;
      }
      if(work != NULL) {
	gsl_vector_free(work);
      }
      if(gsl_v_mat != NULL) {
	gsl_matrix_free(gsl_v_mat);
      }
      if(gsl_singulars != NULL) {
	gsl_vector_free(gsl_singulars);
      }
       query_point;
    }

    int GICPPointSet::AlignScan(GICPPointSet *scan, dgc_transform_t base_t, dgc_transform_t t, double max_match_dist, bool save_error_plot)
    {
      double max_d_sq = pow(max_match_dist, 2);
      int num_matches = 0;
      int n = scan->NumPoints();
      double delta = 0.;
      dgc_transform_t t_last;
      ofstream fout_corresp;
      ANNdist nn_dist_sq;
      ANNidx *nn_indecies = new ANNidx[n];
      ANNpoint query_point = annAllocPt(3);
      
      if(nn_indecies == NULL) {
	//TODO: fail here
      }      

      gicp_mat_t *mahalanobis = new gicp_mat_t[n];
      if(mahalanobis == NULL) {
	//TODO: fail here
      }
      gsl_matrix *gsl_R = gsl_matrix_alloc(3, 3);
      if(gsl_R == NULL) {
	//TODO: fail here
      }
      gsl_matrix *gsl_temp = gsl_matrix_alloc(3, 3);
      if(gsl_temp == NULL) {
	//TODO: fail here
      }
     
      bool converged = false;
      int iteration = 0;
      bool opt_status = false;


      /* set up the optimization parameters */
      GICPOptData opt_data;
      opt_data.nn_indecies = nn_indecies;
      opt_data.p1 = scan;
      opt_data.p2 = this;
      opt_data.M = mahalanobis;
      opt_data.solve_rotation = solve_rotation_;
      dgc_transform_copy(opt_data.base_t, base_t);
      
      GICPOptimizer opt;
      opt.SetDebug(debug_);
      opt.SetMaxIterations(max_iteration_inner_);
      /* set up the mahalanobis matricies */
      /* these are identity for now to ease debugging */
      for(int i = 0; i < n; i++) {
	for(int k = 0; k < 3; k++) {
	  for(int l = 0; l < 3; l++) {
	    mahalanobis[i][k][l] = (k == l)?1:0.;
	  }
	}
      }
      
      if(debug_) {
	dgc_transform_write(base_t, "t_base.tfm");
	dgc_transform_write(t, "t_0.tfm");
      }

      while(!converged) {
	dgc_transform_t transform_R;
	dgc_transform_copy(transform_R, base_t);
	dgc_transform_left_multiply(transform_R, t);
	// copy the rotation component of the current total transformation (including base), into a gsl matrix
	for(int i = 0; i < 3; i++) {
	  for(int j = 0; j < 3; j++) {
	    gsl_matrix_set(gsl_R, i, j, transform_R[i][j]);
	  }
	}
	if(debug_) {
	  fout_corresp.open("correspondence.txt");
	}
	/* find correpondences */
	num_matches = 0;
	for (int i = 0; i < n; i++) {
	  query_point[0] = scan->point_[i].x;
	  query_point[1] = scan->point_[i].y;
	  query_point[2] = scan->point_[i].z;
	  
	  dgc_transform_point(&query_point[0], &query_point[1], 
			      &query_point[2], base_t);
	  dgc_transform_point(&query_point[0], &query_point[1], 
			      &query_point[2], t);
	  
	  kdtree_->annkSearch(query_point, 1, &nn_indecies[i], &nn_dist_sq, 0.0);
	  
	  if (nn_dist_sq < max_d_sq) {
	    if(debug_) {
	      fout_corresp << i << "\t" << nn_indecies[i] << endl;
	    }

	    // set up the updated mahalanobis matrix here
	    gsl_matrix_view C1 = gsl_matrix_view_array(&scan->point_[i].C[0][0], 3, 3);
	    gsl_matrix_view C2 = gsl_matrix_view_array(&point_[nn_indecies[i]].C[0][0], 3, 3);
	    gsl_matrix_view M = gsl_matrix_view_array(&mahalanobis[i][0][0], 3, 3);
	    gsl_matrix_set_zero(&M.matrix);	    
	    gsl_matrix_set_zero(gsl_temp);

	    // M = R*C1  // using M as a temp variable here
	    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1., gsl_R, &C1.matrix, 1., &M.matrix);
	    
	    // temp = M*R' // move the temp value to 'temp' here
	    gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1., &M.matrix, gsl_R, 0., gsl_temp);
	    
	    // temp += C2
	    gsl_matrix_add(gsl_temp, &C2.matrix);
	    // at this point temp = C2 + R*C1*R'
	    
	    // now invert temp to get the mahalanobis distance metric for gicp
	    // M = temp^-1
	    gsl_matrix_set_identity(&M.matrix); 
	    gsl_linalg_cholesky_decomp(gsl_temp);
	    for(int k = 0; k < 3; k++) {
	      gsl_linalg_cholesky_svx(gsl_temp, &gsl_matrix_row(&M.matrix, k).vector);
	    }
	    num_matches++;
	  }
	  else {
	    nn_indecies[i] = -1; // no match
	  }
	}
	
	if(debug_) { // save the current M matrices to file for debugging
	  ofstream out("mahalanobis.txt");
	  if(out) {
	    for(int i = 0; i < n; i++) {
	      for(int k = 0; k < 3; k++) {
		for(int l = 0; l < 3; l++) {
		  out << mahalanobis[i][k][l] << "\t";
		}
	      }
	      out << endl;
	    }
	  }
	  out.close();      
	}
	
	if(debug_) {
	  fout_corresp.close();
	}
	opt_data.num_matches = num_matches;
	
	/* optimize transformation using the current assignment and Mahalanobis metrics*/
	dgc_transform_copy(t_last, t);
	opt_status = opt.Optimize(t, opt_data);
	
	if(debug_) {
	  cout << "Optimizer converged in " << opt.Iterations() << " iterations." << endl;
	  cout << "Status: " << opt.Status() << endl;

	  std::ostringstream filename;
	  filename << "t_" << iteration+1 << ".tfm";
	  dgc_transform_write(t, filename.str().c_str());
	}	
	
	/* compute the delta from this iteration */
	delta = 0.;
	for(int k = 0; k < 4; k++) {
	  for(int l = 0; l < 4; l++) {
	    double ratio = 1;
	    if(k < 3 && l < 3) { // rotation part of the transform
	      ratio = 1./epsilon_rot_;
	    }
	    else {
	      ratio = 1./epsilon_;
	    }
	    double c_delta = ratio*fabs(t_last[k][l] - t[k][l]);
	    
	    if(c_delta > delta) {
	      delta = c_delta;
	    }
	  }
	}
	if(debug_) {
	  cout << "delta = " << delta << endl;
	}
	
	/* check convergence */
	iteration++;
     	if(iteration >= max_iteration_ || delta < 1) {
	  converged = true;
	}
      }
      if(debug_) {
	cout << "Converged in " << iteration << " iterations." << endl;
	if(save_error_plot) {
	  opt.PlotError(t, opt_data, "error_func");
	}
      }
      if(nn_indecies != NULL) {
	delete [] nn_indecies;
      }
      if(mahalanobis != NULL) {
	delete [] mahalanobis;
      }
      if(gsl_R != NULL) {
	gsl_matrix_free(gsl_R);
      }
      if(gsl_temp != NULL) {
	gsl_matrix_free(gsl_temp);
      }
      annDeallocPt(query_point);

      return iteration;
    }
    void GICPPointSet::SavePoints(const char *filename) {
      ofstream out(filename);
      
      if(out) {
	int n = NumPoints();
	for(int i = 0; i < n; i++) {
	  out << point_[i].x << "\t" << point_[i].y << "\t" << point_[i].z << endl;
	}
      }
      out.close();
    }
    
    void GICPPointSet::SaveMatrices(const char *filename) {
      ofstream out(filename);
      
      if(out) {
	int n = NumPoints();
	for(int i = 0; i < n; i++) {
	  for(int k = 0; k < 3; k++) {
	    for(int l = 0; l < 3; l++) {
	      out << point_[i].C[k][l] << "\t";
	    }
	  }
	  out << endl;
	}
      }
      out.close();      
    }    
  }
}
