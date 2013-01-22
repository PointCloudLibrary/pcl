///////////////////////////////////////////////////////
// grabCut->initialize (xstart, ystart, xend, yend); //
// grabCut->fitGMMs ();                              //
///////////////////////////////////////////////////////
#ifndef PCL_SEGMENTATION_IMPL_GRABCUT_HPP
#define PCL_SEGMENTATION_IMPL_GRABCUT_HPP

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

template <typename PointT> void
pcl::GrabCut<PointT>::setInputCloud (const PointCloudConstPtr &cloud)
{
  input_ = cloud;
  initCompute ();
}

template <typename PointT> bool
pcl::GrabCut<PointT>::initCompute ()
{
  using namespace pcl::segmentation::grabcut;
  if (!pcl::PCLBase<PointT>::initCompute ())
  {
    PCL_ERROR ("[pcl::GrabCut::initCompute ()] Init failed!");
    return (false);
  }
  
  if (!input_->isOrganized ())
  {
    PCL_ERROR ("[pcl::GrabCut::initCompute ()] Need an organized point cloud to proceed!");
    return (false);
  }

  std::vector<sensor_msgs::PointField> in_fields_;
  if (pcl::getFieldIndex<PointT> (*input_, "rgb", in_fields_) == -1)
  {
    PCL_ERROR ("[pcl::GrabCut::initCompute ()] No RGB data available, aborting!");
    return (false);
  }
  
  // Initialize the spatial locator
  if (!tree_)
  {
    if (input_->isOrganized ())
      tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
    else
      tree_.reset (new pcl::search::KdTree<PointT> (false));
    tree_->setInputCloud (input_);
  }
  std::cout << "size " << input_->width << "x" << input_->height << std::endl;
  const std::size_t indices_size = indices_->size ();
  image_.reset (new Image (input_->width, input_->height));
  for (std::size_t i = 0; i < indices_size; ++i)
  {
    (*image_) [i] = Color (input_->points[(*indices_)[i]]);
  }

  width_ = image_->width;
  height_ = image_->height;
  std::cout << "size " << width_ << "x" << height_ << std::endl;
  trimap_ = std::vector<segmentation::grabcut::TrimapValue> (indices_size, TrimapUnknown);

  GMM_component_.resize (indices_size);

  hard_segmentation_ = std::vector<segmentation::grabcut::SegmentationValue> (indices_size, SegmentationBackground);
  
  //  soft_segmentation_ = 0;		// Not yet implemented
  foreground_GMM_.resize (K_);
  background_GMM_.resize (K_);
  //set some constants
  computeL ();
  n_links_.resize (indices_size);
  computeBeta ();
  computeNLinks ();
  graph_nodes_.resize (indices_size);

  rev_ = get (boost::edge_reverse, graph_);
  capacity_ = get (boost::edge_capacity, graph_);
  initialized_ = false;
  return (true);
}

template <typename PointT> void
pcl::GrabCut<PointT>::addEdge (vertex_descriptor &v1, vertex_descriptor &v2, float capacity, float rev_capacity)
{
  edge_descriptor e1;
  bool e1_added;
  boost::tie (e1, e1_added) = add_edge (v1, v2, graph_);
  if (e1_added)
  {
    edge_descriptor e2 = add_edge (v2, v1, graph_).first;
    capacity_[e1] = capacity;
    capacity_[e2] = rev_capacity;
    
    rev_[e1] = e2;
    rev_[e2] = e1;
  }
}

template <typename PointT> void
pcl::GrabCut<PointT>::setTerminalWeights (vertex_descriptor& v, float source_capacity, float sink_capacity)
{
  addEdge (graph_source_, v, source_capacity, source_capacity);
  addEdge (v, graph_sink_, sink_capacity, sink_capacity);
}

template <typename PointT> void 
pcl::GrabCut<PointT>::setBackgroundPointsIndices (int x1, int y1, int x2, int y2)
{
  using namespace pcl::segmentation::grabcut;
  
  // Step 1: User creates inital Trimap with rectangle, Background outside, Unknown inside
  fill (trimap_.begin (), trimap_.end (), TrimapBackground);  
  fillRectangle (trimap_, width_, height_, x1, y1, x2, y2, TrimapUnknown);

  // Step 2: Initial segmentation, Background where Trimap is Background, Foreground where Trimap is Unknown.
  fill (hard_segmentation_.begin (), hard_segmentation_.end (), SegmentationBackground);
  fillRectangle (hard_segmentation_, width_, height_, x1, y1, x2, y2, SegmentationForeground);
  if (!initialized_)
  {
    fitGMMs ();
    initialized_ = true;
  }
}

template <typename PointT> void 
pcl::GrabCut<PointT>::setBackgroundPointsIndices (const PointIndicesConstPtr &indices)
{
  using namespace pcl::segmentation::grabcut;
  
  fill (trimap_.begin (), trimap_.end (), TrimapBackground);
  fill (hard_segmentation_.begin (), hard_segmentation_.end (), SegmentationBackground);
  for (std::vector<int>::const_iterator idx = indices->indices.begin (); idx != indices->indices.end (); ++idx)
  {
    trimap_[*idx] = TrimapUnknown;
    hard_segmentation_[*idx] = SegmentationForeground;
  }
  if (!initialized_)
  {
    fitGMMs ();
    initialized_ = true;
  }
}

template <typename PointT> void 
pcl::GrabCut<PointT>::fitGMMs ()
{
  // Step 3: Build GMMs using Orchard-Bouman clustering algorithm
  buildGMMs (*image_, *indices_, hard_segmentation_, GMM_component_, background_GMM_, foreground_GMM_);

  // Initialize the graph for graphcut (do this here so that the T-Link debugging image will be initialized)
  initGraph ();
  
  // Build debugging clouds
  if (build_clouds_)
    buildClouds ();
}

template <typename PointT> int 
pcl::GrabCut<PointT>::refineOnce ()
{
  // Steps 4 and 5: Learn new GMMs from current segmentation
  learnGMMs (*image_, *indices_, hard_segmentation_, GMM_component_, background_GMM_, foreground_GMM_);

  // Step 6: Run GraphCut and update segmentation
  initGraph ();

  float flow = boost::boykov_kolmogorov_max_flow (graph_, graph_source_, graph_sink_);

  int changed = updateHardSegmentation ();
  PCL_INFO ("%d pixels changed segmentation (max flow = %f)\n", changed, flow);

  // Build debugging clouds
  if (build_clouds_)
    buildClouds ();

  return (changed);
}

template <typename PointT> void 
pcl::GrabCut<PointT>::refine ()
{
  std::size_t changed = indices_->size ();

  while (changed)
    changed = refineOnce ();
}

template <typename PointT> int 
pcl::GrabCut<PointT>::updateHardSegmentation ()
{
  using namespace pcl::segmentation::grabcut;
  
  int changed = 0;

  boost::property_map<Graph, boost::vertex_color_t>::type color_map = get (boost::vertex_color, graph_);
  const int number_of_indices = static_cast<int> (indices_->size ());
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    SegmentationValue old_value = hard_segmentation_ [i_point];
                        
    if (trimap_ [i_point] == TrimapBackground)
      hard_segmentation_ [i_point] = SegmentationBackground;
    else 
      if (trimap_ [i_point] == TrimapForeground)
        hard_segmentation_ [i_point] = SegmentationForeground;
      else	// TrimapUnknown
      {
        if (color_map[graph_nodes_[i_point]] == boost::black_color)
          hard_segmentation_ [i_point] = SegmentationForeground;
        else
          hard_segmentation_ [i_point] = SegmentationBackground;
      }
    
    if (old_value != hard_segmentation_ [i_point])
      ++changed;
  }
  return (changed);
}

template <typename PointT> void 
pcl::GrabCut<PointT>::setTrimap (int x1, int y1, int x2, int y2, const pcl::segmentation::grabcut::TrimapValue& t)
{
  using namespace pcl::segmentation::grabcut;
  
  fillRectangle (trimap_, width_, height_, x1, y1, x2, y2, t);

  // Immediately set the segmentation as well so that the display will update.
  if (t == TrimapForeground)
    fillRectangle (hard_segmentation_, width_, height_, x1, y1, x2, y2, SegmentationForeground);
  else if (t == TrimapBackground)
    fillRectangle (hard_segmentation_, width_, height_, x1, y1, x2, y2, SegmentationBackground);

  // Build debugging images
  buildClouds ();
}

template <typename PointT> void 
pcl::GrabCut<PointT>::initGraph ()
{
  using namespace pcl::segmentation::grabcut;
  const int number_of_indices = static_cast<int> (indices_->size ());  
  // Set up the graph (it can only be used once, so we have to recreate it each time the graph is updated)
  graph_.clear ();

  graph_source_ = boost::add_vertex (graph_);

  for (int idx = 0; idx < indices_->size (); ++idx)
    graph_nodes_[idx] = boost::add_vertex (graph_);

  graph_sink_ = boost::add_vertex (graph_);

  // Set T-Link weights
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    int point_index = (*indices_) [i_point];
    float back, fore;
    
    switch (trimap_[point_index])
    {
      case TrimapUnknown :
      {
        fore = static_cast<float> (-log (background_GMM_.probabilityDensity (image_->points[point_index])));
        back = static_cast<float> (-log (foreground_GMM_.probabilityDensity (image_->points[point_index])));
        break;
      }
      case TrimapBackground :
      {
        fore = 0;
        back = L_;
        break;
      }
      default :
      {
        fore = L_;
        back = 0;
      }  
    }
    
    setTerminalWeights (graph_nodes_[i_point], fore, back);
  }

  // Set N-Link weights from precomputed values
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    const NLinks &n_link = n_links_[i_point];
    if (n_link.nb_links > 0)
    {
      int point_index = (*indices_) [i_point];
      std::vector<int>::const_iterator indices_it    = n_link.indices.begin ();
      std::vector<float>::const_iterator weights_it  = n_link.weights.begin ();
      for (; indices_it != n_link.indices.end (); ++indices_it, ++weights_it)
      {
        if (*indices_it != point_index)
        {
          addEdge (graph_nodes_[i_point], graph_nodes_[*indices_it], *weights_it, *weights_it);
        }
      }
    }
  }
}

template <typename PointT> void 
pcl::GrabCut<PointT>::computeNLinks ()
{
  const int number_of_indices = static_cast<int> (indices_->size ());
  for (int i_point = 0; i_point < number_of_indices; ++i_point)
  {
    NLinks &n_link = n_links_[i_point];
    if (n_link.nb_links > 0)
    {
      int point_index = (*indices_) [i_point];
      std::vector<int>::const_iterator indices_it = n_link.indices.begin ();
      std::vector<float>::const_iterator dists_it = n_link.dists.begin   ();
      std::vector<float>::iterator weights_it     = n_link.weights.begin ();
      for (; indices_it != n_link.indices.end (); ++indices_it, ++dists_it, ++weights_it)
      {
        if (*indices_it != point_index)
        {
          // We saved the color distance previously at the computeBeta stage for optimization purpose
          float color_distance = *weights_it;
          // Set the real weight
          *weights_it = static_cast<float> (lambda_ * exp (-beta_ * color_distance) / sqrt (*dists_it));
        }
      }
    }
  }
}

template <typename PointT> void 
pcl::GrabCut<PointT>::computeBeta ()
{
  float result = 0;
  std::size_t edges = 0;

  const int number_of_indices = static_cast<int> (indices_->size ());
  
  for (int i_point = 0; i_point < number_of_indices; i_point++)
  {
    int point_index = (*indices_)[i_point];
    const PointT& point = input_->points [point_index];
    if (pcl::isFinite (point))
    {
      NLinks &links = n_links_[i_point];
      int found = tree_->nearestKSearch (point, nb_neighbours_, links.indices, links.dists);
      if (found > 1)
      {
        links.nb_links = found - 1;
        links.weights.reserve (links.nb_links);
        for (std::vector<int>::const_iterator nn_it = links.indices.begin (); nn_it != links.indices.end (); ++nn_it)
        {
          if (*nn_it != point_index)
          {
            float color_distance = colorDistance (image_->points[point_index], image_->points[*nn_it]);
            links.weights.push_back (color_distance);
            result+= color_distance;
            ++edges;
          }
          else
            links.weights.push_back (0.f);
        }
      }
    }
  }
  
  beta_ = 1.f / (2*result / edges);
}

template <typename PointT> void 
pcl::GrabCut<PointT>::computeL ()
{
  L_ = 8*lambda_ + 1;
}

template <typename PointT> void 
pcl::GrabCut<PointT>::buildClouds ()
{
  using namespace pcl::segmentation::grabcut;
  // Initialize NLinks cloud
  n_links_cloud_.reset (new pcl::PointCloud<pcl::PointXYZI> (width_, height_));
  // Initialize TLinks cloud
  t_links_cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGB> (width_, height_));
  // n_links_cloud_->fill (0);
  GMM_cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGB> (width_, height_));
  // GMM_cloud_->fill (Color (0,0,0));
  alpha_cloud_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> (width_, height_));
  // alpha_cloud_->fill (0);
  const int number_of_indices = static_cast<int> (indices_->size ());
  for (int point_i = 0; point_i < number_of_indices; ++point_i)
  {
    Eigen::Vector3f xyz = input_->points[(*indices_)[point_i]].getVector3fMap ();
    pcl::PointXYZI &nlink_point    = n_links_cloud_->points[point_i];
    pcl::PointXYZRGB &tlink_point  = t_links_cloud_->points[point_i];
    pcl::PointXYZRGB &gmm_point    = GMM_cloud_->points[point_i];
    pcl::PointXYZRGBA &alpha_point = alpha_cloud_->points[point_i];
    const vertex_descriptor &node_point = graph_nodes_[point_i];
    nlink_point.getVector3fMap ()  = xyz;
    tlink_point.getVector3fMap ()  = xyz;
    gmm_point.getVector3fMap ()   = xyz;
    alpha_point.getVector3fMap () = xyz;

    const NLinks &link = n_links_[point_i];
    if (link.nb_links > 0)
    {
      // NLinks cloud
      std::vector<int>::const_iterator indices_it = link.indices.begin ();
      std::vector<float>::const_iterator weights_it = link.weights.begin ();
      for (; indices_it != link.indices.end (); ++indices_it, ++weights_it)
      {
        // Since we assigned 0 for auto weight there is no need for tests
        nlink_point.intensity+= *weights_it;
        n_links_cloud_->points[*indices_it].intensity+= *weights_it;
      }
      // TLinks cloud
      edge_descriptor source_point = edge (graph_source_, node_point, graph_).first;
      edge_descriptor point_sink = edge (node_point, graph_sink_, graph_).first;
      float red = static_cast<float> (pow (capacity_[source_point]/L_, 0.25)); // red
      float green = static_cast<float> (pow (capacity_[point_sink]/L_, 0.25)); // green
      gmm_point.r = static_cast<uint8_t> (red);
      gmm_point.g = static_cast<uint8_t> (green);
      // GMM cloud
      // Alpha cloud
      if (hard_segmentation_[point_i] == SegmentationForeground)
      {
        //assert (static_cast<float>(GMM_component_[point_i]+1)/static_cast<float> (K_) < 1.f);
        gmm_point.r = static_cast<uint8_t> (static_cast<float>(GMM_component_[point_i]+1)/static_cast<float> (K_));
        alpha_point.a = 0;
      }
      else
      {
        gmm_point.g = static_cast<uint8_t> (static_cast<float>(GMM_component_[point_i]+1)/static_cast<float> (K_));
        alpha_point.a = 192;
      }
    }
    else
      alpha_point.a = 255;
  }
}

#endif
