#include "typedefs.h"

#include <vector>
#include <string>
#include <sstream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

/* A few functions to help load up the points */
PointCloudPtr
loadPoints (std::string filename)
{
  PointCloudPtr output (new PointCloud);
  filename.append (".pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

PointCloudPtr
loadKeypoints (std::string filename)
{
  PointCloudPtr output (new PointCloud);
  filename.append ("_keypoints.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}

LocalDescriptorsPtr
loadLocalDescriptors (std::string filename)
{
  LocalDescriptorsPtr output (new LocalDescriptors);
  filename.append ("_localdesc.pcd");
  pcl::io::loadPCDFile (filename, *output);
  pcl::console::print_info ("Loaded %s (%lu points)\n", filename.c_str (), output->size ());
  return (output);
}


void
find_feature_correspondences (const LocalDescriptorsPtr & source_descriptors, 
                              const LocalDescriptorsPtr & target_descriptors,
                              std::vector<int> & correspondences_out, std::vector<float> & correspondence_scores_out)
{
  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<LocalDescriptorT> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);

  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i)
  {
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    correspondence_scores_out[i] = k_squared_distances[0];
  }
}


void 
visualize_correspondences (const PointCloudPtr points1, const PointCloudPtr keypoints1,
                           const PointCloudPtr points2, const PointCloudPtr keypoints2,
                           const std::vector<int> &correspondences,
                           const std::vector<float> &correspondence_scores, int max_to_display)
{ 
  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

  // Create some new point clouds to hold our transformed data
  PointCloudPtr points_left (new PointCloud);
  PointCloudPtr keypoints_left (new PointCloud);
  PointCloudPtr points_right (new PointCloud);
  PointCloudPtr keypoints_right (new PointCloud);

  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.4, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

  // Add the clouds to the visualizer
  pcl::visualization::PCLVisualizer vis;
  vis.addPointCloud (points_left, "points_left");
  vis.addPointCloud (points_right, "points_right");

  // Compute the weakest correspondence score to display
  std::vector<float> temp (correspondence_scores);
  std::sort (temp.begin (), temp.end ());
  if (max_to_display >= temp.size ())
    max_to_display = temp.size () - 1;
  float threshold = temp[max_to_display];

  std::cout << max_to_display << std::endl;

  // Draw lines between the best corresponding points
  for (size_t i = 0; i < keypoints_left->size (); ++i)
  {
    if (correspondence_scores[i] > threshold)
    {
      continue; // Don't draw weak correspondences
    }

    // Get the pair of points
    const PointT & p_left = keypoints_left->points[i];
    const PointT & p_right = keypoints_right->points[correspondences[i]];

    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;

    // Draw the line
    vis.addLine (p_left, p_right, r, g, b, ss.str ());
  }

  vis.resetCamera ();
  vis.spin ();
}

int 
main (int argc, char ** argv)
{
  if (argc < 3) 
  {
    pcl::console::print_info ("Syntax is: %s source target <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -n nr_correspondences ... The number of correspondence to draw (Default = all)\n");
    pcl::console::print_info ("Note: The inputs (source and target) must be specified without the .pcd extension\n");

    return (1);
  }

  PointCloudPtr source_points = loadPoints (argv[1]);
  PointCloudPtr source_keypoints = loadKeypoints (argv[1]);
  LocalDescriptorsPtr source_descriptors = loadLocalDescriptors (argv[1]);
  PointCloudPtr target_points = loadPoints (argv[2]);
  PointCloudPtr target_keypoints = loadKeypoints (argv[2]);
  LocalDescriptorsPtr target_descriptors = loadLocalDescriptors (argv[2]);

  int nr_correspondences = source_keypoints->size ();
  pcl::console::parse_argument (argc, argv, "-n", nr_correspondences);

  std::vector<int> correspondences;
  std::vector<float> correspondence_scores;
  find_feature_correspondences (source_descriptors, target_descriptors, correspondences, correspondence_scores);
  
  visualize_correspondences (source_points, source_keypoints, target_points, target_keypoints, 
                             correspondences, correspondence_scores, nr_correspondences);

  return (0);
}
