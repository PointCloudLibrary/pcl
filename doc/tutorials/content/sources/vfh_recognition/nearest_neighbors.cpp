#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>

typedef std::pair<std::string, std::vector<float> > vfh_model;

/** \brief Loads an n-D histogram file as a VFH signature
  * \param path the input file name
  * \param vfh the resultant VFH model
  */
bool
loadHist (const boost::filesystem::path &path, vfh_model &vfh)
{
  int vfh_idx;
  // Load the file as a PCD
  try
  {
    pcl::PCLPointCloud2 cloud;
    int version;
    Eigen::Vector4f origin;
    Eigen::Quaternionf orientation;
    pcl::PCDReader r;
    int type; unsigned int idx;
    r.readHeader (path.string (), cloud, origin, orientation, version, type, idx);

    vfh_idx = pcl::getFieldIndex (cloud, "vfh");
    if (vfh_idx == -1)
      return (false);
    if ((int)cloud.width * cloud.height != 1)
      return (false);
  }
  catch (const pcl::InvalidConversionException&)
  {
    return (false);
  }

  // Treat the VFH signature as a single Point Cloud
  pcl::PointCloud <pcl::VFHSignature308> point;
  pcl::io::loadPCDFile (path.string (), point);
  vfh.second.resize (308);

  std::vector <pcl::PCLPointField> fields;
  getFieldIndex (point, "vfh", fields);

  for (size_t i = 0; i < fields[vfh_idx].count; ++i)
  {
    vfh.second[i] = point.points[0].histogram[i];
  }
  vfh.first = path.string ();
  return (true);
}


/** \brief Search for the closest k neighbors
  * \param index the tree
  * \param model the query model
  * \param k the number of neighbors to search for
  * \param indices the resultant neighbor indices
  * \param distances the resultant neighbor distances
  */
inline void
nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
{
  // Query point
  flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
  memcpy (&p.ptr ()[0], &model.second[0], p.cols * p.rows * sizeof (float));

  indices = flann::Matrix<int>(new int[k], 1, k);
  distances = flann::Matrix<float>(new float[k], 1, k);
  index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
  delete[] p.ptr ();
}

/** \brief Load the list of file model names from an ASCII file
  * \param models the resultant list of model name
  * \param filename the input file name
  */
bool
loadFileList (std::vector<vfh_model> &models, const std::string &filename)
{
  ifstream fs;
  fs.open (filename.c_str ());
  if (!fs.is_open () || fs.fail ())
    return (false);

  std::string line;
  while (!fs.eof ())
  {
    getline (fs, line);
    if (line.empty ())
      continue;
    vfh_model m;
    m.first = line;
    models.push_back (m);
  }
  fs.close ();
  return (true);
}

int
main (int argc, char** argv)
{
  int k = 6;

  double thresh = DBL_MAX;     // No threshold, disabled by default

  if (argc < 2)
  {
    pcl::console::print_error 
      ("Need at least three parameters! Syntax is: %s <query_vfh_model.pcd> [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]);
    pcl::console::print_info ("    where [options] are:  -k      = number of nearest neighbors to search for in the tree (default: "); 
    pcl::console::print_value ("%d", k); pcl::console::print_info (")\n");
    pcl::console::print_info ("                          -thresh = maximum distance threshold for a model to be considered VALID (default: "); 
    pcl::console::print_value ("%f", thresh); pcl::console::print_info (")\n\n");
    return (-1);
  }

  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  // Load the test histogram
  std::vector<int> pcd_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  vfh_model histogram;
  if (!loadHist (argv[pcd_indices.at (0)], histogram))
  {
    pcl::console::print_error ("Cannot load test file %s\n", argv[pcd_indices.at (0)]);
    return (-1);
  }

  pcl::console::parse_argument (argc, argv, "-thresh", thresh);
  // Search for the k closest matches
  pcl::console::parse_argument (argc, argv, "-k", k);
  pcl::console::print_highlight ("Using "); pcl::console::print_value ("%d", k); pcl::console::print_info (" nearest neighbors.\n");

  std::string kdtree_idx_file_name = "kdtree.idx";
  std::string training_data_h5_file_name = "training_data.h5";
  std::string training_data_list_file_name = "training_data.list";

  std::vector<vfh_model> models;
  flann::Matrix<int> k_indices;
  flann::Matrix<float> k_distances;
  flann::Matrix<float> data;
  // Check if the data has already been saved to disk
  if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
  {
    pcl::console::print_error ("Could not find training data models files %s and %s!\n", 
        training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    return (-1);
  }
  else
  {
    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "training_data");
    pcl::console::print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", 
        (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
  }

  // Check if the tree index has already been saved to disk
  if (!boost::filesystem::exists (kdtree_idx_file_name))
  {
    pcl::console::print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
    return (-1);
  }
  else
  {
    flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
    index.buildIndex ();
    nearestKSearch (index, histogram, k, k_indices, k_distances);
  }

  // Output the results on screen
  pcl::console::print_highlight ("The closest %d neighbors for %s are:\n", k, argv[pcd_indices[0]]);
  for (int i = 0; i < k; ++i)
    pcl::console::print_info ("    %d - %s (%d) with a distance of: %f\n", 
        i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);

  // Load the results
  pcl::visualization::PCLVisualizer p (argc, argv, "VFH Cluster Classifier");
  int y_s = (int)floor (sqrt ((double)k));
  int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
  double x_step = (double)(1 / (double)x_s);
  double y_step = (double)(1 / (double)y_s);
  pcl::console::print_highlight ("Preparing to load "); 
  pcl::console::print_value ("%d", k); 
  pcl::console::print_info (" files ("); 
  pcl::console::print_value ("%d", x_s);    
  pcl::console::print_info ("x"); 
  pcl::console::print_value ("%d", y_s); 
  pcl::console::print_info (" / ");
  pcl::console::print_value ("%f", x_step); 
  pcl::console::print_info ("x"); 
  pcl::console::print_value ("%f", y_step); 
  pcl::console::print_info (")\n");

  int viewport = 0, l = 0, m = 0;
  for (int i = 0; i < k; ++i)
  {
    std::string cloud_name = models.at (k_indices[0][i]).first;
    boost::replace_last (cloud_name, "_vfh", "");

    p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
    l++;
    if (l >= x_s)
    {
      l = 0;
      m++;
    }

    pcl::PCLPointCloud2 cloud;
    pcl::console::print_highlight (stderr, "Loading "); pcl::console::print_value (stderr, "%s ", cloud_name.c_str ());
    if (pcl::io::loadPCDFile (cloud_name, cloud) == -1)
      break;

    // Convert from blob to PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromPCLPointCloud2 (cloud, cloud_xyz);

    if (cloud_xyz.points.size () == 0)
      break;

    pcl::console::print_info ("[done, "); 
    pcl::console::print_value ("%d", (int)cloud_xyz.points.size ()); 
    pcl::console::print_info (" points]\n");
    pcl::console::print_info ("Available dimensions: "); 
    pcl::console::print_value ("%s\n", pcl::getFieldsList (cloud).c_str ());

    // Demean the cloud
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (cloud_xyz, centroid);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_demean (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::demeanPointCloud<pcl::PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
    // Add to renderer*
    p.addPointCloud (cloud_xyz_demean, cloud_name, viewport);
    
    // Check if the model found is within our inlier tolerance
    std::stringstream ss;
    ss << k_distances[0][i];
    if (k_distances[0][i] > thresh)
    {
      p.addText (ss.str (), 20, 30, 1, 0, 0, ss.str (), viewport);  // display the text with red

      // Create a red line
      pcl::PointXYZ min_p, max_p;
      pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p);
      std::stringstream line_name;
      line_name << "line_" << i;
      p.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport);
      p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport);
    }
    else
      p.addText (ss.str (), 20, 30, 0, 1, 0, ss.str (), viewport);

    // Increase the font size for the score*
    p.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 18, ss.str (), viewport);

    // Add the cluster name
    p.addText (cloud_name, 20, 10, cloud_name, viewport);
  }
  // Add coordianate systems to all viewports
  p.addCoordinateSystem (0.1, "global", 0);

  p.spin ();
  return (0);
}
