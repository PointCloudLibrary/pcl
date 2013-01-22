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
 * $Id$
 *
 */

#include <pcl/segmentation/grabcut.h>

float 
pcl::segmentation::grabcut::colorDistance (const Color &c1, const Color &c2)
{
  return ((c1.r-c2.r)*(c1.r-c2.r)+(c1.g-c2.g)*(c1.g-c2.g)+(c1.b-c2.b)*(c1.b-c2.b));
}

void
pcl::segmentation::grabcut::GaussianFitter::add (const Color &c)
{
  sum_.r += c.r; sum_.g += c.g; sum_.b += c.b;

  accumulator_ (0,0) += c.r*c.r; accumulator_ (0,1) += c.r*c.g; accumulator_ (0,2) += c.r*c.b;
  accumulator_ (1,0) += c.g*c.r; accumulator_ (1,1) += c.g*c.g; accumulator_ (1,2) += c.g*c.b;
  accumulator_ (2,0) += c.b*c.r; accumulator_ (2,1) += c.b*c.g; accumulator_ (2,2) += c.b*c.b;

  ++count_;
}

// Build the gaussian out of all the added colors
void
pcl::segmentation::grabcut::GaussianFitter::fit (Gaussian& g, uint32_t total_count, bool compute_eigens) const
{
  if (count_==0)
  {
    g.pi = 0;
  }
  else
  {
    const float count_f = static_cast<float> (count_);
    
    // Compute mean of gaussian
    g.mu.r = sum_.r/count_f;
    g.mu.g = sum_.g/count_f;
    g.mu.b = sum_.b/count_f;

    // Compute covariance matrix
    g.covariance (0,0) = accumulator_ (0,0)/count_f - g.mu.r*g.mu.r + epsilon_;
    g.covariance (0,1) = accumulator_ (0,1)/count_f - g.mu.r*g.mu.g;
    g.covariance (0,2) = accumulator_ (0,2)/count_f - g.mu.r*g.mu.b;
    g.covariance (1,0) = accumulator_ (1,0)/count_f - g.mu.g*g.mu.r;
    g.covariance (1,1) = accumulator_ (1,1)/count_f - g.mu.g*g.mu.g + epsilon_;
    g.covariance (1,2) = accumulator_ (1,2)/count_f - g.mu.g*g.mu.b;
    g.covariance (2,0) = accumulator_ (2,0)/count_f - g.mu.b*g.mu.r;
    g.covariance (2,1) = accumulator_ (2,1)/count_f - g.mu.b*g.mu.g;
    g.covariance (2,2) = accumulator_ (2,2)/count_f - g.mu.b*g.mu.b + epsilon_;

    // Compute determinant of covariance matrix
    g.determinant = g.covariance (0,0)*(g.covariance (1,1)*g.covariance (2,2) - g.covariance (1,2)*g.covariance (2,1))
    - g.covariance (0,1)*(g.covariance (1,0)*g.covariance (2,2) - g.covariance (1,2)*g.covariance (2,0))
    + g.covariance (0,2)*(g.covariance (1,0)*g.covariance (2,1) - g.covariance (1,1)*g.covariance (2,0));

    // Compute inverse (cofactor matrix divided by determinant)
    g.inverse (0,0) =  (g.covariance (1,1)*g.covariance (2,2) - g.covariance (1,2)*g.covariance (2,1)) / g.determinant;
    g.inverse (1,0) = -(g.covariance (1,0)*g.covariance (2,2) - g.covariance (1,2)*g.covariance (2,0)) / g.determinant;
    g.inverse (2,0) =  (g.covariance (1,0)*g.covariance (2,1) - g.covariance (1,1)*g.covariance (2,0)) / g.determinant;
    g.inverse (0,1) = -(g.covariance (0,1)*g.covariance (2,2) - g.covariance (0,2)*g.covariance (2,1)) / g.determinant;
    g.inverse (1,1) =  (g.covariance (0,0)*g.covariance (2,2) - g.covariance (0,2)*g.covariance (2,0)) / g.determinant;
    g.inverse (2,1) = -(g.covariance (0,0)*g.covariance (2,1) - g.covariance (0,1)*g.covariance (2,0)) / g.determinant;
    g.inverse (0,2) =  (g.covariance (0,1)*g.covariance (1,2) - g.covariance (0,2)*g.covariance (1,1)) / g.determinant;
    g.inverse (1,2) = -(g.covariance (0,0)*g.covariance (1,2) - g.covariance (0,2)*g.covariance (1,0)) / g.determinant;
    g.inverse (2,2) =  (g.covariance (0,0)*g.covariance (1,1) - g.covariance (0,1)*g.covariance (1,0)) / g.determinant;

    // The weight of the gaussian is the fraction of the number of pixels in this Gaussian to the number
    // of pixels in all the gaussians of this GMM.
    g.pi = count_f / static_cast<float> (total_count);

    if (compute_eigens)
    {
      // Compute eigenvalues and vectors using SVD
      Eigen::JacobiSVD<Eigen::Matrix3f> svd (g.covariance, Eigen::ComputeThinU);
      // Store highest eigenvalue
      g.eigenvalue = svd.singularValues ()[0];
      // Store corresponding eigenvector
      g.eigenvector = svd.matrixU ().col (0);
    }
  }
}

float 
pcl::segmentation::grabcut::GMM::probabilityDensity (const Color &c)
{
  float result = 0;
  
  for (uint32_t i=0; i < K_; ++i)
    result += gaussians_[i].pi * probabilityDensity (i, c);

  return (result);
}

float 
pcl::segmentation::grabcut::GMM::probabilityDensity (uint32_t i, const Color &c)
{
  float result = 0;
  const pcl::segmentation::grabcut::Gaussian &G = gaussians_[i];
  if (G.pi > 0 )
  {
    if (G.determinant > 0)
    {
      float r = c.r - G.mu.r;
      float g = c.g - G.mu.g;
      float b = c.b - G.mu.b;

      float d = r * (r*G.inverse (0,0) + g*G.inverse (1,0) + b*G.inverse (2,0)) +
                g * (r*G.inverse (0,1) + g*G.inverse (1,1) + b*G.inverse (2,1)) +
                b * (r*G.inverse (0,2) + g*G.inverse (1,2) + b*G.inverse (2,2));

      result = static_cast<float> (1.0/(sqrt (G.determinant)) * exp (-0.5*d));
    }
  }

  return (result);
}

void 
pcl::segmentation::grabcut::buildGMMs (const Image& image, 
                                       const std::vector<int>& indices,
                                       const std::vector<SegmentationValue>& hard_segmentation,
                                       std::vector<uint32_t>& components,
                                       GMM& background_GMM, GMM& foreground_GMM)
{
  // Step 3: Build GMMs using Orchard-Bouman clustering algorithm

  // Set up Gaussian Fitters
  std::vector<GaussianFitter> back_fitters (background_GMM.getK ());
  std::vector<GaussianFitter> fore_fitters (foreground_GMM.getK ());

  uint32_t fore_count = 0, back_count = 0;
  const int indices_size = static_cast<int> (indices.size ());
  // Initialize the first foreground and background clusters
  for (int idx = 0; idx < indices_size; ++idx)
  {
    components [idx] = 0;
    
    if (hard_segmentation [idx] == SegmentationForeground)
    {
      fore_fitters[0].add (image[indices[idx]]);
      fore_count++;
    }
    else
    {
      back_fitters[0].add (image[indices[idx]]);
      back_count++;
    }
  }


  back_fitters[0].fit (background_GMM[0], back_count, true);
  fore_fitters[0].fit (foreground_GMM[0], fore_count, true);

  uint32_t n_back = 0, n_fore = 0;		// Which cluster will be split
  uint32_t max_K = (background_GMM.getK () > foreground_GMM.getK ()) ? background_GMM.getK () : foreground_GMM.getK ();

  // Compute clusters
  for (uint32_t i = 1; i < max_K; ++i)
  {
    // Reset the fitters for the splitting clusters
    back_fitters[n_back] = GaussianFitter ();
    fore_fitters[n_fore] = GaussianFitter ();

    // For brevity, get references to the splitting Gaussians
    Gaussian& bg = background_GMM[n_back];
    Gaussian& fg = foreground_GMM[n_fore];

    // Compute splitting points
    float split_background = bg.eigenvector[0] * bg.mu.r + bg.eigenvector[1] * bg.mu.g + bg.eigenvector[2] * bg.mu.b;
    float split_foreground = fg.eigenvector[0] * fg.mu.r + fg.eigenvector[1] * fg.mu.g + fg.eigenvector[2] * fg.mu.b;

    // Split clusters nBack and nFore, place split portion into cluster i
    for (int idx = 0; idx < indices_size; ++idx)
    {
      const Color &c = image[indices[idx]];
      
      // For each pixel
      if (i < foreground_GMM.getK () && 
          hard_segmentation[idx] == SegmentationForeground && 
          components[idx] == n_fore)
      {
        if (fg.eigenvector[0] * c.r + fg.eigenvector[1] * c.g + fg.eigenvector[2] * c.b > split_foreground)
        {
          components[idx] = i;
          fore_fitters[i].add (c);
        }
        else
        {
          fore_fitters[n_fore].add (c);
        }
      }
      else if (i < background_GMM.getK () && 
               hard_segmentation[idx] == SegmentationBackground && 
               components[idx] == n_back)
      {
        if (bg.eigenvector[0] * c.r + bg.eigenvector[1] * c.g + bg.eigenvector[2] * c.b > split_background)
        {
          components[idx] = i;
          back_fitters[i].add (c);
        }
        else
        {
          back_fitters[n_back].add (c);
        }
      }
    }

    // Compute new split Gaussians
    back_fitters[n_back].fit (background_GMM[n_back], back_count, true);
    fore_fitters[n_fore].fit (foreground_GMM[n_fore], fore_count, true);

    if (i < background_GMM.getK ())
      back_fitters[i].fit (background_GMM[i], back_count, true);
    if (i < foreground_GMM.getK ())
      fore_fitters[i].fit (foreground_GMM[i], fore_count, true);

    // Find clusters with highest eigenvalue
    n_back = 0;
    n_fore = 0;

    for (uint32_t j = 0; j <= i; ++j)
    {
      if (j < background_GMM.getK () && background_GMM[j].eigenvalue > background_GMM[n_back].eigenvalue)
        n_back = j;

      if (j < foreground_GMM.getK () && foreground_GMM[j].eigenvalue > foreground_GMM[n_fore].eigenvalue)
        n_fore = j;
    }
  }

  back_fitters.clear ();
  fore_fitters.clear ();
}

void 
pcl::segmentation::grabcut::learnGMMs (const Image& image,
                                       const std::vector<int>& indices,
                                       const std::vector<SegmentationValue>& hard_segmentation,
                                       std::vector<uint32_t>& components,
                                       GMM& background_GMM, GMM& foreground_GMM)
{
  const uint32_t indices_size = static_cast<uint32_t> (indices.size ());
  // Step 4: Assign each pixel to the component which maximizes its probability
  for (uint32_t idx = 0; idx < indices_size; ++idx)
  {
    const Color &c = image[indices[idx]];
    
    if (hard_segmentation[idx] == SegmentationForeground)
    {
      uint32_t k = 0;
      float max = 0;
      
      for (uint32_t i = 0; i < foreground_GMM.getK (); i++)
      {
        float p = foreground_GMM.probabilityDensity (i, c);
        if (p > max)
        {
          k = i;
          max = p;
        }
      }
      components[idx] = k;
    }
    else
    {
      uint32_t k = 0;
      float max = 0;
      
      for (uint32_t i = 0; i < background_GMM.getK (); i++)
      {
        float p = background_GMM.probabilityDensity (i, c);
        if (p > max)
        {
          k = i;
          max = p;
        }
      }
      components[idx] = k;
    }
  }

  // Step 5: Relearn GMMs from new component assignments

  // Set up Gaussian Fitters
  std::vector<GaussianFitter> back_fitters (background_GMM.getK ());
  std::vector<GaussianFitter> fore_fitters (foreground_GMM.getK ());

  uint32_t foreCount = 0, backCount = 0;

  for (uint32_t idx = 0; idx < indices_size; ++idx)
  {
    const Color &c = image [indices [idx]];
    
    if (hard_segmentation[idx] == SegmentationForeground)
    {
      fore_fitters[components[idx]].add (c);
      foreCount++;
    }
    else
    {
      back_fitters[components[idx]].add (c);
      backCount++;
    }
  }

  for (uint32_t i = 0; i < background_GMM.getK (); ++i)
    back_fitters[i].fit (background_GMM[i], backCount, false);

  for (uint32_t i = 0; i < foreground_GMM.getK (); ++i)
    fore_fitters[i].fit (foreground_GMM[i], foreCount, false);

  back_fitters.clear ();
  fore_fitters.clear ();
}
