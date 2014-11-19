#include "surface.h"

#include <string>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int 
main (int argc, char ** argv)
{
  if (argc < 2) 
  {
    pcl::console::print_info ("Syntax is: %s input.pcd <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    --surfel radius,order............... Compute surface elements\n");
    pcl::console::print_info ("    --convex ........................... Compute convex hull\n");
    pcl::console::print_info ("    --concave alpha .................... Compute concave hull\n");
    pcl::console::print_info ("    --greedy radius,max_nn,mu,surf_angle,min_angle,max_angle ... Compute greedy triangulation\n");
    pcl::console::print_info ("    -s output.pcd ...................... Save the output cloud\n");

    return (1);
  }

  // Load the points
  PointCloudPtr cloud (new PointCloud);
  pcl::io::loadPCDFile (argv[1], *cloud);
  pcl::console::print_info ("Loaded %s (%lu points)\n", argv[1], cloud->size ());

  // Compute surface elements
  SurfaceElementsPtr surfels (new SurfaceElements);
  double mls_radius, polynomial_order;
  bool compute_surface_elements = 
    pcl::console::parse_2x_arguments (argc, argv, "--surfel", mls_radius, polynomial_order) > 0;
  if (compute_surface_elements)
  {
    surfels = computeSurfaceElements (cloud, mls_radius, polynomial_order);
    pcl::console::print_info ("Computed surface elements\n");
  }

  // Find the convex hull
  MeshPtr convex_hull;
  bool compute_convex_hull = pcl::console::find_argument (argc, argv, "--convex") > 0;
  if (compute_convex_hull)
  {
    convex_hull = computeConvexHull (cloud);
    pcl::console::print_info ("Computed convex hull\n");
  }

  // Find the concave hull
  MeshPtr concave_hull;
  double alpha;
  bool compute_concave_hull = pcl::console::parse_argument (argc, argv, "--concave", alpha) > 0;
  if (compute_concave_hull)
  {
    concave_hull = computeConcaveHull (cloud, alpha);
    pcl::console::print_info ("Computed concave hull\n");
  }

  // Compute a greedy surface triangulation
  pcl::PolygonMesh::Ptr greedy_mesh;
  std::string params_string;
  bool perform_greedy_triangulation = pcl::console::parse_argument (argc, argv, "--greedy", params_string) > 0;
  if (perform_greedy_triangulation)
  {
    assert (surfels);

    std::vector<std::string> tokens;
    boost::split (tokens, params_string, boost::is_any_of (","), boost::token_compress_on);
    assert (tokens.size () == 6);
    float radius = atof(tokens[0].c_str ());
    int max_nearest_neighbors = atoi(tokens[1].c_str ());
    float mu = atof(tokens[2].c_str ());
    float max_surface_angle = atof(tokens[3].c_str ());
    float min_angle = atof(tokens[4].c_str ());
    float max_angle = atof(tokens[5].c_str ());

    greedy_mesh = greedyTriangulation (surfels, radius, max_nearest_neighbors, mu, 
                                       max_surface_angle, min_angle, max_angle);

    pcl::console::print_info ("Performed greedy surface triangulation\n");
  }


  // Compute a greedy surface triangulation
  pcl::PolygonMesh::Ptr marching_cubes_mesh;
  double leaf_size, iso_level;
  bool perform_marching_cubes = pcl::console::parse_2x_arguments (argc, argv, "--mc", leaf_size, iso_level) > 0;
  if (perform_marching_cubes)
  {
    assert (surfels);

    marching_cubes_mesh = marchingCubesTriangulation (surfels, leaf_size, iso_level);

    pcl::console::print_info ("Performed marching cubes surface triangulation\n");
  }

  // Save output
  std::string output_filename;
  bool save_output = pcl::console::parse_argument (argc, argv, "-s", output_filename) > 0;
  if (save_output)
  {
    // Save the result
    pcl::io::savePCDFile (output_filename, *cloud);

    pcl::console::print_info ("Saved result as %s\n", output_filename.c_str ());    
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit.\n");
    pcl::visualization::PCLVisualizer vis;
    vis.addPointCloud (cloud);
    vis.resetCamera ();

    if (compute_convex_hull)
    {
      vis.addPolygonMesh<PointT> (convex_hull->points, convex_hull->faces, "convex_hull");
    }
    if (compute_concave_hull)
    {
      vis.addPolygonMesh<PointT> (concave_hull->points, concave_hull->faces, "concave_hull");
    }
    if (perform_greedy_triangulation)
    {
      vis.addPolygonMesh(*greedy_mesh, "greedy_mesh");
    }
    if (perform_marching_cubes)
    {
      vis.addPolygonMesh(*marching_cubes_mesh, "marching_cubes_mesh");
    }

    vis.spin ();
  }
}
