/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 */

#include <pcl/ml/permutohedral.h>

///////////////////////////////////////////////////////////////////////////////////////////
pcl::Permutohedral::Permutohedral () :
  N_ (0), M_ (0), d_ (0),
  blur_neighborsOLD_(NULL), offsetOLD_ (NULL), barycentricOLD_ (NULL) 
{}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::Permutohedral::init (const std::vector<float> &feature, const int feature_dimension, const int N)
{
  N_ = N;
  d_ = feature_dimension;
  
  // Create hash table
  std::vector<std::vector<short> > keys;
  keys.reserve ((d_+1) * N_);
  std::multimap<size_t, int> hash_table;

  // reserve class memory
  if (offset_.size () > 0) 
    offset_.clear ();
  offset_.resize ((d_ + 1) * N_);

  if (barycentric_.size () > 0) 
    barycentric_.clear ();
  barycentric_.resize ((d_ + 1) * N_);

  // create vectors and matrices
  Eigen::VectorXf scale_factor = Eigen::VectorXf::Zero (d_);
  Eigen::VectorXf elevated = Eigen::VectorXf::Zero (d_ + 1);
  Eigen::VectorXf rem0 = Eigen::VectorXf::Zero (d_+1);
  Eigen::VectorXf barycentric = Eigen::VectorXf::Zero (d_+2);
  Eigen::VectorXi rank = Eigen::VectorXi::Zero (d_+1);
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> canonical;
  canonical = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Zero (d_+1, d_+1);
  //short * key = new short[d_+1];
  std::vector<short> key (d_+1);

  // Compute the canonical simple
  for (int i = 0; i <= d_; i++)
  {
    for (int j = 0; j <= (d_ - i); j++)
      canonical (j, i) = i;
    for (int j = (d_ - i + 1); j <= d_; j++)
      canonical (j, i) = i - (d_ + 1);
  }

  // Expected standard deviation of our filter (p.6 in [Adams etal 2010])
  float inv_std_dev = sqrtf (2.0f / 3.0f) * static_cast<float> (d_ + 1);
  
  // Compute the diagonal part of E (p.5 in [Adams etal 2010])
  for (int i = 0; i < d_; i++)
    scale_factor (i) = 1.0f / sqrtf (static_cast<float> (i + 2) * static_cast<float> (i + 1)) * inv_std_dev;

  // Compute the simplex each feature lies in
  for (int k = 0; k < N_; k++)
    //for (int k = 0; k < 5; k++)
  {

    // Elevate the feature  (y = Ep, see p.5 in [Adams etal 2010])
    int index = k * feature_dimension;
    // sm contains the sum of 1..n of our faeture vector
    float sm = 0;
    for (int j = d_; j > 0; j--)
    {
      float cf = feature[index + j-1] * scale_factor (j-1);      
      elevated (j) = sm - static_cast<float> (j) * cf;
      sm += cf;
    }
    elevated (0) = sm;

    // Find the closest 0-colored simplex through rounding
    float down_factor = 1.0f / static_cast<float>(d_+1);
    float up_factor = static_cast<float>(d_+1);
    int sum = 0;
    for (int j = 0; j <= d_; j++){
      float rd = floorf (0.5f + (down_factor * elevated (j))) ;
      rem0 (j) = rd * up_factor;
      sum += static_cast<int> (rd);
    }
    
    // rank differential to find the permutation between this simplex and the canonical one.         
    // (See pg. 3-4 in paper.)    
    rank.setZero ();
    Eigen::VectorXf tmp = elevated - rem0;
    for (int i = 0; i < d_; i++){
      for (int j = i+1; j <= d_; j++)
        if (tmp (i) < tmp (j))
          rank (i)++;
        else
          rank (j)++;
    }

    // If the point doesn't lie on the plane (sum != 0) bring it back
    for (int j = 0; j <= d_; j++){
      rank (j) += sum;
      if (rank (j) < 0){
        rank (j) += d_+1;
        rem0 (j) += static_cast<float> (d_ + 1);
      }
      else if (rank (j) > d_){
        rank (j) -= d_+1;
        rem0 (j) -= static_cast<float> (d_ + 1);
      }
    }

    // Compute the barycentric coordinates (p.10 in [Adams etal 2010])
    barycentric.setZero ();
    Eigen::VectorXf v = (elevated - rem0) * down_factor;
    for (int j = 0; j <= d_; j++){
      barycentric (d_ - rank (j)    ) += v (j);
      barycentric (d_ + 1 - rank (j)) -= v (j);
    }
    // Wrap around
    barycentric (0) += 1.0f + barycentric (d_+1);

    // Compute all vertices and their offset
    for (int remainder = 0; remainder <= d_; remainder++)
    {
      for (int j = 0; j < d_; j++)
        key[j] = static_cast<short> (rem0 (j) + static_cast<float> (canonical ( rank (j), remainder)));

      // insert key in hash table      
      size_t hash_key = generateHashKey (key);
      std::multimap<size_t ,int>::iterator it = hash_table.find (hash_key);
      int key_index = -1;
      if (it != hash_table.end ())
      {
        key_index = it->second;
        
        // check if key is the right one
        int tmp_key_index = -1;
        //for (int ii = key_index; ii < keys.size (); ii++)
        for (it = hash_table.find (hash_key); it != hash_table.end (); ++it)
        {
          int ii = it->second;
          bool same = true;
          std::vector<short> k = keys[ii];
          for (size_t i_k = 0; i_k < k.size (); i_k++)
          {
            if (key[i_k] != k[i_k])
            {
              same = false;
              break;
            }
          }

          if (same)
          {
            tmp_key_index = ii;
            break;
          }
        }
      
        if (tmp_key_index == -1)
        {
          key_index = static_cast<int> (keys.size ());
          keys.push_back (key);
          hash_table.insert (std::pair<size_t, int> (hash_key, key_index));
        }
        else
          key_index = tmp_key_index;
      }
      
      else
      {  
        key_index = static_cast<int> (keys.size ());
        keys.push_back (key);
        hash_table.insert (std::pair<size_t, int> (hash_key, key_index));
      }
      offset_[ k * (d_ + 1) + remainder ] = static_cast<float> (key_index);
      
      barycentric_[ k * (d_ + 1) + remainder ] = barycentric (remainder);
    }
  }

  // Find the Neighbors of each lattice point
		
  // Get the number of vertices in the lattice
  M_ = static_cast<int> (hash_table.size());
		
  // Create the neighborhood structure
  if (blur_neighbors_.size () > 0) 
    blur_neighbors_.clear ();
  blur_neighbors_.resize ((d_+1)*M_);

  std::vector<short> n1 (d_+1);
  std::vector<short> n2 (d_+1);

  // For each of d+1 axes,
  for (int j = 0; j <= d_; j++)
  {
    for (int i = 0; i < M_; i++)
    {
      std::vector<short> key = keys[i];

      for (int k=0; k<d_; k++){
        n1[k] = static_cast<short> (key[k] - 1);
        n2[k] = static_cast<short> (key[k] + 1);
      }
      n1[j] = static_cast<short> (key[j] + d_);
      n2[j] = static_cast<short> (key[j] - d_);

      std::multimap<size_t ,int>::iterator it;
      size_t hash_key;
      int key_index = -1;      
      hash_key = generateHashKey (n1);
      it = hash_table.find (hash_key);
      if (it != hash_table.end ())
        key_index = it->second;
      blur_neighbors_[j*M_+i].n1 = key_index;

      key_index = -1;
      hash_key = generateHashKey (n2);
      it = hash_table.find (hash_key);
      if (it != hash_table.end ())
        key_index = it->second;
      blur_neighbors_[j*M_+i].n2 = key_index;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::Permutohedral::compute (std::vector<float> &out, const std::vector<float> &in, 
                             int value_size,
                             int in_offset, int out_offset,
                             int in_size, int out_size) const
{
  if (in_size == -1)  in_size = N_ -  in_offset;
  if (out_size == -1) out_size = N_ - out_offset;
		
  // Shift all values by 1 such that -1 -> 0 (used for blurring)
  std::vector<float> values ((M_+2)*value_size, 0.0f);
  std::vector<float> new_values ((M_+2)*value_size, 0.0f);
	
  // Splatting
  for (int i = 0;  i < in_size; i++)
  {
    for (int j = 0; j <= d_; j++)
    {
      int o = static_cast<int> (offset_[(in_offset + i) * (d_ + 1) + j]) + 1;
      float w = barycentric_[(in_offset + i) * (d_ + 1) + j];
      for (int k = 0; k < value_size; k++)
        values[ o * value_size + k ] += w * in[ i * value_size + k ];
    }
  }
		
  for (int j = 0; j <= d_; j++)
  {
    for (int i = 0; i < M_; i++)
    {
      int old_val_idx = (i+1) * value_size;
      int new_val_idx = (i+1) * value_size;
				
      int n1 = blur_neighbors_[j*M_+i].n1+1;
      int n2 = blur_neighbors_[j*M_+i].n2+1;
      int n1_val_idx = n1 * value_size;
      int n2_val_idx = n2 * value_size;
      
      for (int k = 0; k < value_size; k++)
        new_values[new_val_idx + k] = values[old_val_idx + k] + 
        0.5f * (values[n1_val_idx + k] + values[n2_val_idx + k]);        
    }
    values.swap (new_values);
  }

  // Alpha is a magic scaling constant (write Andrew if you really wanna understand this)
  float alpha = 1.0f / (1.0f + static_cast<float> (pow(2.0f, -d_)));
		
  // Slicing
  for (int i = 0; i < out_size; i++){
    for (int k = 0; k < value_size; k++)
      out[i * value_size + k] = 0;
    for (int j = 0; j <= d_; j++){
      int o = static_cast<int> (offset_[(out_offset + i) * (d_ + 1) + j]) + 1;
      float w = barycentric_[(out_offset + i) * (d_ + 1) + j];
      for (int k = 0; k <value_size; k++)
        out[ i* value_size + k ] += w * values[ o * value_size + k ] * alpha;
    }
  }		
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::Permutohedral::initOLD (const std::vector<float> &feature, const int feature_dimension, const int N)
{
  // Compute the lattice coordinates for each feature [there is going to be a lot of magic here
  N_ = N;
  d_ = feature_dimension;
  HashTableOLD hash_table(d_, N_*(d_+1));

  // Allocate the class memory
  if (offsetOLD_) delete [] offsetOLD_;
  offsetOLD_ = new int[ (d_+1)*N_ ];
  if (barycentricOLD_) delete [] barycentricOLD_;
  barycentricOLD_ = new float[ (d_+1)*N_ ];
		
  // Allocate the local memory
  float * scale_factor = new float[d_];
  float * elevated = new float[d_+1];
  float * rem0 = new float[d_+1];
  float * barycentric = new float[d_+2];
  int * rank = new int[d_+1];
  short * canonical = new short[(d_+1)*(d_+1)];
  short * key = new short[d_+1];
		
  // Compute the canonical simplex
  for (int i=0; i<=d_; i++){
    for (int j=0; j<=d_-i; j++)
      canonical[i*(d_+1)+j] = static_cast<short> (i);
    for (int j=d_-i+1; j<=d_; j++)
      canonical[i*(d_+1)+j] = static_cast<short> (i - (d_+1));
  }
		
  // Expected standard deviation of our filter (p.6 in [Adams etal 2010])
  float inv_std_dev = sqrtf (2.0f / 3.0f)* static_cast<float>(d_+1);
  // Compute the diagonal part of E (p.5 in [Adams etal 2010])
  for (int i=0; i<d_; i++)
    scale_factor[i] = 1.0f / sqrtf (static_cast<float>(i+2)*static_cast<float>(i+1)) * inv_std_dev;
		
  // Compute the simplex each feature lies in
  for (int k=0; k<N_; k++)
  {
  //for (int k = 0; k < 500; k++){
    // Elevate the feature (y = Ep, see p.5 in [Adams etal 2010])
    //const float * f = feature + k*feature_size;
    int index = k * feature_dimension;


    // sm contains the sum of 1..n of our faeture vector
    float sm = 0;
    for (int j = d_; j > 0; j--)
    {
      //float cf = f[j-1]*scale_factor[j-1];
      float cf = feature[index + j-1] * scale_factor[j-1];
      elevated[j] = sm - static_cast<float>(j)*cf;
      sm += cf;
    }
    elevated[0] = sm;

    // Find the closest 0-colored simplex through rounding
    float down_factor = 1.0f / static_cast<float>(d_+1);
    float up_factor = static_cast<float>(d_+1);
    int sum = 0;
    for (int i=0; i<=d_; i++){
      int rd = static_cast<int> (pcl_round (down_factor * elevated[i]));
      rem0[i] = static_cast<float >(rd) * up_factor;
      sum += rd;
    }
			
    // Find the simplex we are in and store it in rank (where rank describes what position coorinate i has in the sorted order of the features values)
    for (int i=0; i<=d_; i++)
      rank[i] = 0;
    for (int i=0; i<d_; i++)
    {
      double di = elevated[i] - rem0[i];
      for (int j=i+1; j<=d_; j++)
        if (di < elevated[j] - rem0[j])
          rank[i]++;
        else
          rank[j]++;
    }

    // If the point doesn't lie on the plane (sum != 0) bring it back
    for (int i=0; i<=d_; i++){
      rank[i] += sum;
      if (rank[i] < 0){
        rank[i] += d_+1;
        rem0[i] += static_cast<float> (d_+1);
      }
      else if (rank[i] > d_){
        rank[i] -= d_+1;
        rem0[i] -= static_cast<float> (d_+1);
      }
    }

    // Compute the barycentric coordinates (p.10 in [Adams etal 2010])
    for (int i=0; i<=d_+1; i++)
      barycentric[i] = 0;
    for (int i=0; i<=d_; i++){
      float v = (elevated[i] - rem0[i])*down_factor;
      barycentric[d_-rank[i]  ] += v;
      barycentric[d_-rank[i]+1] -= v;
    }
    // Wrap around
    barycentric[0] += 1.0f + barycentric[d_+1];
			
    // Compute all vertices and their offset
    for (int remainder = 0; remainder <= d_; remainder++)
    {
      for (int i = 0; i < d_; i++)
        key[i] = static_cast<short int> (rem0[i] + canonical[remainder * (d_ + 1) + rank[i]]);
      offsetOLD_[k*(d_+1)+remainder] = hash_table.find (key, true);
      barycentricOLD_[k*(d_+1)+remainder] = barycentric[remainder];
      baryOLD_.push_back (barycentric[remainder]);
    }
  }


  

  delete [] scale_factor;
  delete [] elevated;
  delete [] rem0;
  delete [] barycentric;
  delete [] rank;
  delete [] canonical;
  delete [] key;

  // Find the Neighbors of each lattice point
		
  // Get the number of vertices in the lattice
  M_ = hash_table.size();
		
  // Create the neighborhood structure
  if(blur_neighborsOLD_) delete[] blur_neighborsOLD_;
  blur_neighborsOLD_ = new Neighbors[ (d_+1)*M_ ];
		
  short * n1 = new short[d_+1];
  short * n2 = new short[d_+1];
		
  // For each of d+1 axes,
  for (int j = 0; j <= d_; j++){
    for (int i=0; i<M_; i++){
      const short * key = hash_table.getKey(i);
      //std::cout << *key << std::endl;
      for (int k=0; k<d_; k++){
        n1[k] = static_cast<short int> (key[k] - 1);
        n2[k] = static_cast<short int> (key[k] + 1);
      }
      n1[j] = static_cast<short int> (key[j] + d_);
      n2[j] = static_cast<short int> (key[j] - d_);

      //std::cout << *n1 << "  |  " << *n2 << std::endl;
				
      blur_neighborsOLD_[j*M_+i].n1 = hash_table.find(n1);
      blur_neighborsOLD_[j*M_+i].n2 = hash_table.find(n2);
/*
      std::cout << hash_table.find(n1) << "  |  " <<
      hash_table.find(n2) << std::endl;
*/    

    }
  }
  delete[] n1;
  delete[] n2;



}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void 
pcl::Permutohedral::computeOLD (std::vector<float> &out, const std::vector<float> &in, 
                                int value_size,
                                int in_offset, int out_offset,
                                int in_size, int out_size) const
{
  if (in_size == -1)  in_size = N_ -  in_offset;
  if (out_size == -1) out_size = N_ - out_offset;
		
  // Shift all values by 1 such that -1 -> 0 (used for blurring)
  float * values = new float[ (M_+2)*value_size ];
  float * new_values = new float[ (M_+2)*value_size ];
		
  for (int i=0; i<(M_+2)*value_size; i++)
    values[i] = new_values[i] = 0;
		
  // Splatting
  for (int i=0;  i<in_size; i++){
    for (int j=0; j<=d_; j++){
      int o = static_cast<int> (offsetOLD_[(in_offset+i)*(d_+1)+j]) + 1;
      float w = barycentricOLD_[(in_offset+i)*(d_+1)+j];
      for (int k=0; k<value_size; k++)
        values[ o*value_size+k ] += w * in[ i*value_size+k ];
    }
  }
		
  for (int j=0; j<=d_; j++){
    for (int i=0; i<M_; i++){
      float * old_val = values + (i+1)*value_size;
      float * new_val = new_values + (i+1)*value_size;
				
      int n1 = blur_neighborsOLD_[j*M_+i].n1+1;
      int n2 = blur_neighborsOLD_[j*M_+i].n2+1;
      float * n1_val = values + n1*value_size;
      float * n2_val = values + n2*value_size;
      for (int k=0; k<value_size; k++)
        new_val[k] = old_val[k]+0.5f * (n1_val[k] + n2_val[k]);
    }
    float * tmp = values;
    values = new_values;
    new_values = tmp;
  }
  // Alpha is a magic scaling constant (write Andrew if you really wanna understand this)
  float alpha = 1.0f / (1.0f + powf (2.0f, - float (d_)));
		
  // Slicing
  for (int i=0; i<out_size; i++)
  {
    for (int k=0; k<value_size; k++)
      out[i*value_size+k] = 0;
    for (int j=0; j<=d_; j++){
      int o = static_cast<int> (offsetOLD_[(out_offset+i)*(d_+1)+j]) + 1;
      float w = barycentricOLD_[(out_offset+i)*(d_+1)+j];
      for (int k=0; k<value_size; k++)
        out[ i*value_size+k ] += w * values[ o*value_size+k ] * alpha;
    }
  }
		
		
  delete[] values;
  delete[] new_values;
}


////////////////
// DEBUG //////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
pcl::Permutohedral::debug ()
{
  bool same = true;
  for (size_t i = 0; i < barycentric_.size (); i++)
  {
    if (barycentric_[i] != barycentricOLD_[i])
      same = false;
    if (offset_[i] != offsetOLD_[i])
    {
      same = false;
    }
    
  }

  for (size_t i = 0; i < blur_neighbors_.size (); i++)
  {
    if (blur_neighbors_[i].n1 != blur_neighborsOLD_[i].n1)
      same = false;
    if (blur_neighbors_[i].n2 != blur_neighborsOLD_[i].n2)
      same = false;
  }


  if (same)
    std::cout << "DEBUG - OK" << std::endl;
  else
    std::cout << "DEBUG - ERROR" << std::endl;
}

/*
std::vector<float>
pcl::Permuohedral::getBarycentric ()
{
  return 
}
*/
