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
pcl::grabcut::distance2(const Color &c1, const Color &c2)
{
  return ((c1.r-c2.r)*(c1.r-c2.r)+(c1.g-c2.g)*(c1.g-c2.g)+(c1.b-c2.b)*(c1.b-c2.b));
}

void
pcl::grabcut::GaussianFitter::add (const Color &c)
{
  sum_.r += c.r; sum_.g += c.g; sum_.b += c.b;

  accumulator_ (0,0) += c.r*c.r; accumulator_ (0,1) += c.r*c.g; accumulator_ (0,2) += c.r*c.b;
  accumulator_ (1,0) += c.g*c.r; accumulator_ (1,1) += c.g*c.g; accumulator_ (1,2) += c.g*c.b;
  accumulator_ (2,0) += c.b*c.r; accumulator_ (2,1) += c.b*c.g; accumulator_ (2,2) += c.b*c.b;

  ++count_;
}

// Build the gaussian out of all the added colors
void
pcl::grabcut::GaussianFitter::fit (Gaussian& g, uint32_t total_count, bool compute_eigens) const
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
pcl::grabcut::GMM::probabilityDensity (const Color &c)
{
  float result = 0;
  
  for (uint32_t i=0; i < K_; ++i)
    result += gaussians_[i].pi * probabilityDensity (i, c);

  return (result);
}

float 
pcl::grabcut::GMM::probabilityDensity (uint32_t i, const Color &c)
{
  float result = 0;
  const pcl::grabcut::Gaussian &G = gaussians_[i];
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
pcl::grabcut::buildGMMs (const Image& image, 
                         const pcl::PointCloud<SegmentationValue>& hard_segmentation,
                         pcl::PointCloud<uint32_t>& components,
                         GMM& background_GMM, GMM& foreground_GMM)
{
  // Step 3: Build GMMs using Orchard-Bouman clustering algorithm

  // Set up Gaussian Fitters
  std::vector<GaussianFitter> back_fitters (background_GMM.getK ());
  std::vector<GaussianFitter> fore_fitters (foreground_GMM.getK ());

  uint32_t fore_count = 0, back_count = 0;

  // Initialize the first foreground and background clusters
  for (uint32_t y = 0; y < image.height; ++y)
  {
    for (uint32_t x = 0; x < image.width; ++x)
    {
      components (x,y) = 0;

      if (hard_segmentation (x,y) == SegmentationForeground)
      {
        fore_fitters[0].add (image (x,y));
        fore_count++;
      }
      else
      {
        back_fitters[0].add (image (x,y));
        back_count++;
      }
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
    for (uint32_t y = 0; y < image.height; ++y)
    {
      for (uint32_t x = 0; x < image.width; ++x)
      {
        Color c = image (x,y);

        // For each pixel
        if (i < foreground_GMM.getK () && hard_segmentation (x,y) == SegmentationForeground && components (x,y) == n_fore)
        {
          if (fg.eigenvector[0] * c.r + fg.eigenvector[1] * c.g + fg.eigenvector[2] * c.b > split_foreground)
          {
            components (x,y) = i;
            fore_fitters[i].add (c);
          }
          else
          {
            fore_fitters[n_fore].add (c);
          }
        }
        else if (i < background_GMM.getK () && hard_segmentation (x,y) == SegmentationBackground && components (x,y) == n_back)
        {
          if (bg.eigenvector[0] * c.r + bg.eigenvector[1] * c.g + bg.eigenvector[2] * c.b > split_background)
          {
            components (x,y) = i;
            back_fitters[i].add (c);
          }
          else
          {
            back_fitters[n_back].add (c);
          }
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
pcl::grabcut::learnGMMs (const Image& image, 
                         const pcl::PointCloud<SegmentationValue>& hard_segmentation,
                         pcl::PointCloud<uint32_t>& components,
                         GMM& background_GMM, GMM& foreground_GMM)
{
  // Step 4: Assign each pixel to the component which maximizes its probability
  for (uint32_t y = 0; y < image.height; ++y)
  {
    for (uint32_t x = 0; x < image.width; ++x)
    {
      Color c = image (x,y);

      if (hard_segmentation (x,y) == SegmentationForeground)
      {
        int k = 0;
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
        components (x, y) = k;
      }
      else
      {
        int k = 0;
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

        components (x, y) = k;
      }
    }
  }

  // Step 5: Relearn GMMs from new component assignments

  // Set up Gaussian Fitters
  std::vector<GaussianFitter> back_fitters (background_GMM.getK ());
  std::vector<GaussianFitter> fore_fitters (foreground_GMM.getK ());

  uint32_t foreCount = 0, backCount = 0;

  for (uint32_t y = 0; y < image.height; ++y)
  {
    for (uint32_t x = 0; x < image.width; ++x)
    {
      Color c = image (x,y);

      if (hard_segmentation (x,y) == SegmentationForeground)
      {
        fore_fitters[components (x,y)].add (c);
        foreCount++;
      }
      else
      {
        back_fitters[components (x,y)].add (c);
        backCount++;
      }
    }
  }

  for (uint32_t i = 0; i < background_GMM.getK (); i++)
    back_fitters[i].fit (background_GMM[i], backCount, false);

  for (uint32_t i = 0; i < foreground_GMM.getK (); i++)
    fore_fitters[i].fit (foreground_GMM[i], foreCount, false);

  back_fitters.clear ();
  fore_fitters.clear ();
}

void
pcl::GrabCut::addEdge (vertex_descriptor &v1, vertex_descriptor &v2, float capacity, float rev_capacity)
{
  edge_descriptor e1 = add_edge (v1, v2, graph_).first;
  edge_descriptor e2 = add_edge (v2, v1, graph_).first;
  put (boost::edge_capacity, graph_, e1, capacity);
  put (boost::edge_capacity, graph_, e2, rev_capacity);

  rev_[e1] = e2;
  rev_[e2] = e1;
}

void
pcl::GrabCut::setTerminalWeights (vertex_descriptor& v, float source_capacity, float sink_capacity)
{
  addEdge (graph_source_, v, source_capacity, source_capacity);
  addEdge (v, graph_sink_, sink_capacity, sink_capacity);
}

bool
pcl::GrabCut::initCompute ()
{
  using namespace pcl::grabcut;
  
  width_ = image_->width;
  height_ = image_->height;

  trimap_.reset (new pcl::PointCloud<TrimapValue> (width_, height_, TrimapUnknown));

  GMM_component_.reset (new pcl::PointCloud<uint32_t> (width_, height_));

  hard_segmentation_.reset (new pcl::PointCloud<SegmentationValue> (width_, height_, SegmentationBackground));

//  soft_segmentation_ = 0;		// Not yet implemented

  t_links_image_.reset (new Image (width_, height_, Color (0,0,0)));
  // t_links_image_->fill (Color (0,0,0));
  n_links_image_.reset (new pcl::PointCloud<float> (width_, height_, 0));
  // n_links_image_->fill (0);
  GMM_image_.reset (new Image (width_, height_, Color (0,0,0)));
  // GMM_image_->fill (Color (0,0,0));
  alpha_image_.reset (new pcl::PointCloud<float> (width_, height_, 0));
  // alpha_image_->fill (0);
  foreground_GMM_.resize (K_);
  background_GMM_.resize (K_);
  //set some constants
  computeL ();
  computeBeta ();

  n_links_.reset (new pcl::PointCloud<NLinks> (width_, height_));
  computeNLinks ();
  nodes_.reset (new pcl::PointCloud<vertex_descriptor> (width_, height_));

  rev_ = get (boost::edge_reverse, graph_);
  
  return (true);
}

void 
pcl::GrabCut::initialize (uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
{
  using namespace pcl::grabcut;
  
  // Step 1: User creates inital Trimap with rectangle, Background outside, Unknown inside
  fill (*trimap_, TrimapBackground);
  fillRectangle (*trimap_, x1, y1, x2, y2, TrimapUnknown);

  // Step 2: Initial segmentation, Background where Trimap is Background, Foreground where Trimap is Unknown.
  fill (*hard_segmentation_, SegmentationBackground);
  fillRectangle (*hard_segmentation_, x1, y1, x2, y2, SegmentationForeground);
}

void 
pcl::GrabCut::fitGMMs ()
{
  // Step 3: Build GMMs using Orchard-Bouman clustering algorithm
  buildGMMs (*image_, *hard_segmentation_, *GMM_component_, background_GMM_, foreground_GMM_);

  // Initialize the graph for graphcut (do this here so that the T-Link debugging image will be initialized)
  initGraph ();

  // Build debugging images
  buildImages ();
}

int 
pcl::GrabCut::refineOnce ()
{
  float flow = 0;

  // Steps 4 and 5: Learn new GMMs from current segmentation
  learnGMMs (*image_, *hard_segmentation_, *GMM_component_, background_GMM_, foreground_GMM_);

  // Step 6: Run GraphCut and update segmentation
  initGraph ();

  flow = boost::boykov_kolmogorov_max_flow (graph_, graph_source_, graph_sink_);

  int changed = updateHardSegmentation ();
  PCL_WARN ("%d pixels changed segmentation (max flow = %f)\n", changed, flow);

  // Build debugging images
  buildImages ();

  return (changed);
}

void 
pcl::GrabCut::refine ()
{
  int changed = width_*height_;

  while (changed)
    changed = refineOnce ();
}

int 
pcl::GrabCut::updateHardSegmentation ()
{
  using namespace pcl::grabcut;
  
  int changed = 0;

  boost::property_map<Graph, boost::vertex_color_t>::type color_map = get (boost::vertex_color, graph_);

  for (uint32_t y = 0; y < height_; ++y)
  {
    for (uint32_t x = 0; x < width_; ++x)
    {
      SegmentationValue old_value = (*hard_segmentation_)(x,y);

      if ((*trimap_)(x,y) == TrimapBackground)
        (*hard_segmentation_)(x,y) = SegmentationBackground;
      else 
        if ((*trimap_)(x,y) == TrimapForeground)
          (*hard_segmentation_)(x,y) = SegmentationForeground;
        else	// TrimapUnknown
        {
          if (color_map[(*nodes_)(x,y)] == boost::black_color)
            (*hard_segmentation_)(x,y) = SegmentationForeground;
          else
            (*hard_segmentation_)(x,y) = SegmentationBackground;
        }
      
      if (old_value != (*hard_segmentation_)(x,y))
        changed++;
    }
  }
  return (changed);
}

void 
pcl::GrabCut::setTrimap (int x1, int y1, int x2, int y2, const pcl::grabcut::TrimapValue& t)
{
  using namespace pcl::grabcut;
  
  fillRectangle (*trimap_, x1, y1, x2, y2, t);

  // Immediately set the segmentation as well so that the display will update.
  if (t == TrimapForeground)
    fillRectangle (*hard_segmentation_, x1, y1, x2, y2, SegmentationForeground);
  else if (t == TrimapBackground)
    fillRectangle (*hard_segmentation_, x1, y1, x2, y2, SegmentationBackground);

  // Build debugging images
  buildImages ();
}

void 
pcl::GrabCut::initGraph ()
{
  using namespace pcl::grabcut;
  
  // Set up the graph (it can only be used once, so we have to recreate it each time the graph is updated)
  graph_.clear ();

  graph_source_ = boost::add_vertex (graph_);

  for (uint32_t y = 0; y < height_; ++y)
  {
    for (uint32_t x = 0; x < width_; ++x)
    {
      (*nodes_)(x,y) = boost::add_vertex (graph_);
    }
  }

  graph_sink_ = boost::add_vertex (graph_);

  // Set T-Link weights
  for (uint32_t y = 0; y < height_; ++y)
  {
    for (uint32_t x = 0; x < width_; ++x)
    {
      float back, fore;

      if ((*trimap_)(x,y) == TrimapUnknown )
      {
        fore = static_cast<float> (-log (background_GMM_.probabilityDensity ((*image_)(x,y))));
        back = static_cast<float> (-log (foreground_GMM_.probabilityDensity ((*image_)(x,y))));
      }
      else if ((*trimap_)(x,y) == TrimapBackground )
      {
        fore = 0;
        back = L_;
      }
      else		// TrimapForeground
      {
        fore = L_;
        back = 0;
      }

      setTerminalWeights ((*nodes_)(x,y), fore, back);

      (*t_links_image_)(x,y).r = static_cast<float> (pow (fore/L_, 0.25));
      (*t_links_image_)(x,y).g = static_cast<float> (pow (back/L_, 0.25));
    }
  }

  // Set N-Link weights from precomputed values
  for (uint32_t y = 0; y < height_; ++y)
  {
    for (uint32_t x = 0; x < width_; ++x)
    {
      if ( x > 0 && y < height_-1 )
        //graph_->add_edge ((*nodes_)(x,y), (*nodes_)(x-1,y+1), (*n_links_)(x,y).upleft, (*n_links_)(x,y).upleft);
        addEdge ((*nodes_)(x,y), (*nodes_)(x-1,y+1), (*n_links_)(x,y).upleft, (*n_links_)(x,y).upleft);

      if ( y < height_-1 )
        //graph_->add_edge ((*nodes_)(x,y), (*nodes_)(x,y+1), (*n_links_)(x,y).up, (*n_links_)(x,y).up);
        addEdge ((*nodes_)(x,y), (*nodes_)(x,y+1), (*n_links_)(x,y).up, (*n_links_)(x,y).up);

      if ( x < width_-1 && y < height_-1 )
        //graph_->add_edge ((*nodes_)(x,y), (*nodes_)(x+1,y+1), (*n_links_)(x,y).upright, (*n_links_)(x,y).upright);
        addEdge ((*nodes_)(x,y), (*nodes_)(x+1,y+1), (*n_links_)(x,y).upright, (*n_links_)(x,y).upright);

      if (x < width_-1)
        //graph_->add_edge ((*nodes_)(x,y), (*nodes_)(x+1,y), (*n_links_)(x,y).right, (*n_links_)(x,y).right);
        addEdge ((*nodes_)(x,y), (*nodes_)(x+1,y), (*n_links_)(x,y).right, (*n_links_)(x,y).right);
    }
  }
}

void 
pcl::GrabCut::computeNLinks ()
{
  for (uint32_t y = 0; y < height_; ++y )
  {
    for (uint32_t x = 0; x < width_; ++x )
    {
      if ( x > 0 && y < height_-1 )
        (*n_links_)(x,y).upleft = computeNLink ( x, y, x-1, y+1 );

      if ( y < height_-1 )
        (*n_links_)(x,y).up = computeNLink ( x, y, x, y+1 );

      if ( x < width_-1 && y < height_-1 )
        (*n_links_)(x,y).upright = computeNLink ( x, y, x+1, y+1 );

      if ( x < width_-1 )
        (*n_links_)(x,y).right = computeNLink ( x, y, x+1, y );
    }
  }
}

float 
pcl::GrabCut::computeNLink (uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2)
{
  float d = static_cast<float> (sqrt ((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2)));
  return (static_cast<float> (lambda_ * exp (-beta_ * distance2((*image_)(x1,y1), (*image_)(x2,y2))) / d));
}

void 
pcl::GrabCut::computeBeta ()
{
  float result = 0;
  int edges = 0;

  for (uint32_t y = 0; y < height_; ++y)
  {
    for (uint32_t x = 0; x < width_; ++x)
    {
      // upleft
      if (x > 0 && y < height_-1)
      {
        result += distance2( (*image_)(x,y), (*image_)(x-1,y+1) );
        edges++;
      }
      // up
      if (y < height_-1)
      {
        result += distance2( (*image_)(x,y), (*image_)(x,y+1) );
        edges++;
      }
      // upright
      if (x < width_-1 && y < height_-1)
      {
        result += distance2( (*image_)(x,y), (*image_)(x+1,y+1) );
        edges++;
      }
      // right
      if (x < width_-1)
      {
        result += distance2( (*image_)(x,y), (*image_)(x+1,y) );
        edges++;
      }
    }
  }

  beta_ = static_cast<float> (1.f/(2*result/static_cast<float> (edges)));
}

void 
pcl::GrabCut::computeL ()
{
  L_ = 8*lambda_ + 1;
}

void 
pcl::GrabCut::buildImages ()
{
  using namespace pcl::grabcut;
  
  fill (*n_links_image_, 0.f);

  for (uint32_t y = 0; y < height_; ++y)
  {
    for (uint32_t x = 0; x < width_; ++x)
    {
      // T-Links image is populated in initGraph since we have easy access to the link values there.

      // N-Links image
      if ( x > 0 && y < height_-1 )
      {
        (*n_links_image_)(x,y) += (*n_links_)(x,y).upleft/L_;
        (*n_links_image_)(x-1,y+1) += (*n_links_)(x,y).upleft/L_;
      }

      if ( y < height_-1 )
      {
        (*n_links_image_)(x,y) += (*n_links_)(x,y).up/L_;
        (*n_links_image_)(x,y+1) += (*n_links_)(x,y).up/L_;
      }

      if ( x < width_-1 && y < height_-1 )
      {
        (*n_links_image_)(x,y) += (*n_links_)(x,y).upright/L_;
        (*n_links_image_)(x+1,y+1) += (*n_links_)(x,y).upright/L_;
      }

      if ( x < width_-1 )
      {
        (*n_links_image_)(x,y) += (*n_links_)(x,y).right/L_;
        (*n_links_image_)(x+1,y) += (*n_links_)(x,y).right/L_;
      }
      
      // GMM image
      if ((*hard_segmentation_)(x,y) == SegmentationForeground)
        (*GMM_image_)(x,y) = Color ((static_cast<float> ((*GMM_component_)(x,y))+1.f)/static_cast<float> (K_),0.f,0.f);
      else
        (*GMM_image_)(x,y) = Color (0.f,(static_cast<float> ((*GMM_component_)(x,y))+1.f)/static_cast<float> (K_),0.f);

      //Alpha image
      if ((*hard_segmentation_)(x,y) == SegmentationForeground)
        (*alpha_image_)(x,y) = 0.0;
      else
        (*alpha_image_)(x,y) = 0.75;
    }
  }
}

