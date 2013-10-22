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

#include <pcl/segmentation/grabcut_segmentation.h>

#include <cstdlib>
#include <cassert>
#include <vector>
#include <map>
#include <algorithm>

pcl::segmentation::grabcut::BoykovKolmogorov::BoykovKolmogorov (std::size_t max_nodes)
  : flow_value_(0.0)
{
  if (max_nodes > 0)
  {
    source_edges_.reserve (max_nodes);
    target_edges_.reserve (max_nodes);
    nodes_.reserve (max_nodes);
  }
}

double
pcl::segmentation::grabcut::BoykovKolmogorov::operator() (int u, int v) const
{
  if ((u < 0) && (v < 0)) return flow_value_;
  if (u < 0) { return source_edges_[v]; }
  if (v < 0) { return target_edges_[u]; }
  capacitated_edge::const_iterator it = nodes_[u].find (v);
  if (it == nodes_[u].end ()) return 0.0;
  return it->second;
}

double
pcl::segmentation::grabcut::BoykovKolmogorov::getSourceEdgeCapacity (int u) const
{
  return (source_edges_[u]);
}

double
pcl::segmentation::grabcut::BoykovKolmogorov::getTargetEdgeCapacity (int u) const
{
  return (target_edges_[u]);
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::preAugmentPaths ()
{
  for (int u = 0; u < (int)nodes_.size (); u++)
  {
    // augment s-u-t paths
    if ((source_edges_[u] > 0.0) && (target_edges_[u] > 0.0))
    {
      const double cap = std::min (source_edges_[u], target_edges_[u]);
      flow_value_ += cap;
      source_edges_[u] -= cap;
      target_edges_[u] -= cap;
    }

    if (source_edges_[u] == 0.0) continue;

    // augment s-u-v-t paths
    for (std::map<int, double>::iterator it = nodes_[u].begin (); it != nodes_[u].end (); it++)
    {
      const int v = it->first;
      if ((it->second == 0.0) || (target_edges_[v] == 0.0)) continue;
      const double w = std::min (it->second, std::min (source_edges_[u], target_edges_[v]));
      source_edges_[u] -= w;
      target_edges_[v] -= w;
      it->second -= w;
      nodes_[v][u] += w;
      flow_value_ += w;
      if (source_edges_[u] == 0.0) break;
    }
  }
}

int
pcl::segmentation::grabcut::BoykovKolmogorov::addNodes (size_t n)
{
  int node_id = (int)nodes_.size ();
  nodes_.resize (nodes_.size () + n);
  source_edges_.resize (nodes_.size (), 0.0);
  target_edges_.resize (nodes_.size (), 0.0);
  return (node_id);
}

// void
// pcl::segmentation::grabcut::BoykovKolmogorov::addSourceAndTargetEdges (int u, double source_cap, double sink_cap)
// {
//   addSourceEdge (u, source_cap);
//   addTargetEdge (u, sink_cap);

// }

void
pcl::segmentation::grabcut::BoykovKolmogorov::addSourceEdge (int u, double cap)
{
  assert ((u >= 0) && (u < (int)nodes_.size ()));
  if (cap < 0.0)
  {
    flow_value_ += cap;
    target_edges_[u] -= cap;
  }
  else
    source_edges_[u] += cap;
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::addTargetEdge (int u, double cap)
{
  assert ((u >= 0) && (u < (int)nodes_.size ()));
  if (cap < 0.0)
  {
    flow_value_ += cap;
    source_edges_[u] -= cap;
  }
  else
    target_edges_[u] += cap;
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::addEdge (int u, int v, double cap_uv, double cap_vu)
{
  assert ((u >= 0) && (u < (int)nodes_.size ()));
  assert ((v >= 0) && (v < (int)nodes_.size ()));
  assert (u != v);

  capacitated_edge::iterator it = nodes_[u].find (v);
  if (it == nodes_[u].end ())
  {
    assert (cap_uv + cap_vu >= 0.0);
    if (cap_uv < 0.0)
    {
      nodes_[u].insert (std::make_pair (v, 0.0));
      nodes_[v].insert (std::make_pair (u, cap_vu + cap_uv));
      source_edges_[u] -= cap_uv;
      target_edges_[v] -= cap_uv;
      flow_value_ += cap_uv;
    }
    else
    {
      if (cap_vu < 0.0)
      {
        nodes_[u].insert (std::make_pair (v, cap_uv + cap_vu));
        nodes_[v].insert (std::make_pair (u, 0.0));
        source_edges_[v] -= cap_vu;
        target_edges_[u] -= cap_vu;
        flow_value_ += cap_vu;
      }
      else
      {
        nodes_[u].insert (std::make_pair (v, cap_uv));
        nodes_[v].insert (std::make_pair (u, cap_vu));
      }
    }
  }
  else
  {
    capacitated_edge::iterator jt = nodes_[v].find (u);
    it->second += cap_uv;
    jt->second += cap_vu;
    assert (it->second + jt->second >= 0.0);
    if (it->second < 0.0)
    {
      jt->second += it->second;
      source_edges_[u] -= it->second;
      target_edges_[v] -= it->second;
      flow_value_ += it->second;
      it->second = 0.0;
    }
    else
    {
      if (jt->second < 0.0)
      {
        it->second += jt->second;
        source_edges_[v] -= jt->second;
        target_edges_[u] -= jt->second;
        flow_value_ += jt->second;
        jt->second = 0.0;
      }
    }
  }
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::reset ()
{
  flow_value_ = 0.0;
  std::fill (source_edges_.begin (), source_edges_.end (), 0.0);
  std::fill (target_edges_.begin (), target_edges_.end (), 0.0);
  for (int u = 0; u < (int)nodes_.size (); u++)
  {
    for (capacitated_edge::iterator it = nodes_[u].begin (); it != nodes_[u].end (); it++)
    {
      it->second = 0.0;
    }
  }
  std::fill (cut_.begin (), cut_.end (), FREE);
  parents_.clear ();
  clearActive ();
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::clear ()
{
  flow_value_ = 0.0;
  source_edges_.clear ();
  target_edges_.clear ();
  nodes_.clear ();
  cut_.clear ();
  parents_.clear ();
  clearActive ();
}

double
pcl::segmentation::grabcut::BoykovKolmogorov::solve ()
{
  // initialize search tree and active set
  cut_.resize (nodes_.size ());
  std::fill (cut_.begin (), cut_.end (), FREE);
  parents_.resize (nodes_.size ());

  clearActive ();

  // pre-augment paths
  preAugmentPaths ();

  // initialize search trees
  initializeTrees ();

  std::deque<int> orphans;
  while (!isActiveSetEmpty ())
  {
    const std::pair<int, int> path = expandTrees ();
    augmentPath (path, orphans);
    if (!orphans.empty ())
    {
      adoptOrphans (orphans);
    }
  }
  return (flow_value_);
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::initializeTrees ()
{
  // initialize search tree
  for (int u = 0; u < (int)nodes_.size (); u++)
  {
    if (source_edges_[u] > 0.0)
    {
      cut_[u] = SOURCE;
      parents_[u].first = TERMINAL;
      markActive (u);
    }
    else
    {
      if (target_edges_[u] > 0.0)
      {
        cut_[u] = TARGET;
        parents_[u].first = TERMINAL;
        markActive (u);
      }
    }
  }
}

std::pair<int, int>
pcl::segmentation::grabcut::BoykovKolmogorov::expandTrees ()
{
  // expand trees looking for augmenting paths
  while (!isActiveSetEmpty ())
  {
    const int u = active_head_;

    if (cut_[u] == SOURCE) {
      for (capacitated_edge::iterator it = nodes_[u].begin (); it != nodes_[u].end (); it++)
      {
        if (it->second > 0.0)
        {
          if (cut_[it->first] == FREE)
          {
            cut_[it->first] = SOURCE;
            parents_[it->first] = std::make_pair (u, std::make_pair (it, nodes_[it->first].find (u)));
            markActive (it->first);
          }
          else
          {
            if (cut_[it->first] == TARGET)
            {
              // found augmenting path
              return (std::make_pair (u, it->first));
            }
          }
        }
      }
    }
    else
    {
      for (capacitated_edge::iterator it = nodes_[u].begin (); it != nodes_[u].end (); it++)
      {
        if (cut_[it->first] == TARGET) continue;
        if (nodes_[it->first][u] > 0.0)
        {
          if (cut_[it->first] == FREE)
          {
            cut_[it->first] = TARGET;
            parents_[it->first] = std::make_pair (u, std::make_pair (nodes_[it->first].find (u), it));
            markActive (it->first);
          }
          else
          {
            if (cut_[it->first] == SOURCE)
            {
              // found augmenting path
              return (std::make_pair (it->first, u));
            }
          }
        }
      }
    }

    // remove node from active set
    markInactive (u);
  }

  return (std::make_pair (TERMINAL, TERMINAL));
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::augmentPath (const std::pair<int, int>& path, std::deque<int>& orphans)
{
  if ((path.first == TERMINAL) && (path.second == TERMINAL))
    return;

  // find path capacity

  // backtrack
  const edge_pair e = std::make_pair (nodes_[path.first].find (path.second),
                                      nodes_[path.second].find (path.first));
  double c = e.first->second;
  int u = path.first;
  while (parents_[u].first != TERMINAL)
  {
    c = std::min (c, parents_[u].second.first->second);
    u = parents_[u].first;
    //assert (cut_[u] == SOURCE);
  }
  c = std::min (c, source_edges_[u]);

  // forward track
  u = path.second;
  while (parents_[u].first != TERMINAL)
  {
    c = std::min (c, parents_[u].second.first->second);
    u = parents_[u].first;
    //assert (cut_[u] == TARGET);
  }
  c = std::min (c, target_edges_[u]);

  // augment path
  flow_value_ += c;
  //DRWN_LOG_DEBUG ("path capacity: " << c);

  // backtrack
  u = path.first;
  while (parents_[u].first != TERMINAL)
  {
    //nodes_[u][parents_[u].first] += c;
    parents_[u].second.second->second += c;
    parents_[u].second.first->second -= c;
    if (parents_[u].second.first->second == 0.0)
    {
      orphans.push_front (u);
    }
    u = parents_[u].first;
  }
  source_edges_[u] -= c;
  if (source_edges_[u] == 0.0)
  {
    orphans.push_front (u);
  }

  // link
  e.first->second -= c;
  e.second->second += c;

  // forward track
  u = path.second;
  while (parents_[u].first != TERMINAL) {
    //nodes_[parents_[u].first][u] += c;
    parents_[u].second.second->second += c;
    parents_[u].second.first->second -= c;
    if (parents_[u].second.first->second == 0.0) {
      orphans.push_back (u);
    }
    u = parents_[u].first;
  }
  target_edges_[u] -= c;
  if (target_edges_[u] == 0.0) {
    orphans.push_back (u);
  }
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::adoptOrphans (std::deque<int>& orphans)
{
  // find new parent for orphaned subtree or free it
  while (!orphans.empty ())
  {
    const int u = orphans.front ();
    const char tree_label = cut_[u];
    orphans.pop_front ();

    // can occur if same node is inserted into orphans multiple times
    if (tree_label == FREE) continue;
    //assert (tree_label != FREE);

    // look for new parent
    bool b_free_orphan = true;
    for (capacitated_edge::iterator jt = nodes_[u].begin (); jt != nodes_[u].end (); ++jt) {
      // skip if different trees
      if (cut_[jt->first] != tree_label) continue;

      // check edge capacity
      const capacitated_edge::iterator kt = nodes_[jt->first].find (u);
      if (((tree_label == TARGET) && (jt->second <= 0.0)) ||
          ((tree_label == SOURCE) && (kt->second <= 0.0)))
        continue;

      // check that u is not an ancestor of jt->first
      int v = jt->first;
      while ((v != u) && (v != TERMINAL))
      {
        v = parents_[v].first;
      }
      if (v != TERMINAL) continue;

      // add as parent
      const edge_pair e = (tree_label == SOURCE) ? std::make_pair (kt, jt) : std::make_pair (jt, kt);
      parents_[u] = std::make_pair (jt->first, e);
      b_free_orphan = false;
      break;
    }

    // free the orphan subtree and remove it from the active set
    if (b_free_orphan)
    {
      for (capacitated_edge::const_iterator jt = nodes_[u].begin (); jt != nodes_[u].end (); ++jt)
      {
        if ((cut_[jt->first] == tree_label) && (parents_[jt->first].first == u))
        {
          orphans.push_front (jt->first);
          markActive (jt->first);
        }
        else if (cut_[jt->first] != FREE)
        {
          markActive (jt->first);
        }
      }

      // mark inactive and free
      markInactive (u);
      cut_[u] = FREE;
    }
  }
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::clearActive ()
{
  active_head_ = active_tail_ = TERMINAL;
  active_list_.resize (nodes_.size ());
  std::fill (active_list_.begin (), active_list_.end (), std::make_pair (TERMINAL, TERMINAL));
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::markActive (int u)
{
  if (isActive (u)) return;

  active_list_[u].first = active_tail_;
  active_list_[u].second = TERMINAL;
  if (active_tail_ == TERMINAL)
    active_head_ = u;
  else
    active_list_[active_tail_].second = u;
  active_tail_ = u;
}

void
pcl::segmentation::grabcut::BoykovKolmogorov::markInactive (int u)
{
  //if (!isActive (u)) return;

  if (u == active_head_)
  {
    active_head_ = active_list_[u].second;
    if (u != active_tail_)
    {
      active_list_[active_list_[u].second].first = TERMINAL;
    }
  }
  else
    if (u == active_tail_)
    {
      active_tail_ = active_list_[u].first;
      active_list_[active_list_[u].first].second = TERMINAL;
    }
    else
      if (active_list_[u].first != TERMINAL)
      {
        active_list_[active_list_[u].first].second = active_list_[u].second;
        active_list_[active_list_[u].second].first = active_list_[u].first;
      }
  //active_list_[u] = std::make_pair (TERMINAL, TERMINAL);
  active_list_[u].first = TERMINAL;
}

void
pcl::segmentation::grabcut::GaussianFitter::add (const Color &c)
{
  sum_[0] += c.r; sum_[1] += c.g; sum_[2] += c.b;
  accumulator_ (0,0) += c.r*c.r; accumulator_ (0,1) += c.r*c.g; accumulator_ (0,2) += c.r*c.b;
  accumulator_ (1,0) += c.g*c.r; accumulator_ (1,1) += c.g*c.g; accumulator_ (1,2) += c.g*c.b;
  accumulator_ (2,0) += c.b*c.r; accumulator_ (2,1) += c.b*c.g; accumulator_ (2,2) += c.b*c.b;

  ++count_;
}

// Build the gaussian out of all the added colors
void
pcl::segmentation::grabcut::GaussianFitter::fit (Gaussian& g, std::size_t total_count, bool compute_eigens) const
{
  if (count_==0)
  {
    g.pi = 0;
  }
  else
  {
    const float count_f = static_cast<float> (count_);

    // Compute mean of gaussian
    g.mu.r = sum_[0]/count_f;
    g.mu.g = sum_[1]/count_f;
    g.mu.b = sum_[2]/count_f;

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
      Eigen::JacobiSVD<Eigen::Matrix3f> svd (g.covariance, Eigen::ComputeFullU);
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

  for (std::size_t i=0; i < gaussians_.size (); ++i)
    result += gaussians_[i].pi * probabilityDensity (i, c);

  return (result);
}

float
pcl::segmentation::grabcut::GMM::probabilityDensity (std::size_t i, const Color &c)
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
                                       std::vector<std::size_t>& components,
                                       GMM& background_GMM, GMM& foreground_GMM)
{
  // Step 3: Build GMMs using Orchard-Bouman clustering algorithm

  // Set up Gaussian Fitters
  std::vector<GaussianFitter> back_fitters (background_GMM.getK ());
  std::vector<GaussianFitter> fore_fitters (foreground_GMM.getK ());

  std::size_t fore_count = 0, back_count = 0;
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

  std::size_t n_back = 0, n_fore = 0;		// Which cluster will be split
  std::size_t max_K = (background_GMM.getK () > foreground_GMM.getK ()) ? background_GMM.getK () : foreground_GMM.getK ();

  // Compute clusters
  for (std::size_t i = 1; i < max_K; ++i)
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

    for (std::size_t j = 0; j <= i; ++j)
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
                                       std::vector<std::size_t>& components,
                                       GMM& background_GMM, GMM& foreground_GMM)
{
  const std::size_t indices_size = static_cast<std::size_t> (indices.size ());
  // Step 4: Assign each pixel to the component which maximizes its probability
  for (std::size_t idx = 0; idx < indices_size; ++idx)
  {
    const Color &c = image[indices[idx]];

    if (hard_segmentation[idx] == SegmentationForeground)
    {
      std::size_t k = 0;
      float max = 0;

      for (std::size_t i = 0; i < foreground_GMM.getK (); i++)
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
      std::size_t k = 0;
      float max = 0;

      for (std::size_t i = 0; i < background_GMM.getK (); i++)
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

  std::size_t fore_counter = 0, back_counter = 0;
  for (std::size_t idx = 0; idx < indices_size; ++idx)
  {
    const Color &c = image [indices [idx]];

    if (hard_segmentation[idx] == SegmentationForeground)
    {
      fore_fitters[components[idx]].add (c);
      fore_counter++;
    }
    else
    {
      back_fitters[components[idx]].add (c);
      back_counter++;
    }
  }

  for (std::size_t i = 0; i < background_GMM.getK (); ++i)
    back_fitters[i].fit (background_GMM[i], back_counter, false);

  for (std::size_t i = 0; i < foreground_GMM.getK (); ++i)
    fore_fitters[i].fit (foreground_GMM[i], fore_counter, false);

  back_fitters.clear ();
  fore_fitters.clear ();
}
