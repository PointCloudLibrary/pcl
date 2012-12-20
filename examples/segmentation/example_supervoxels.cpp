#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxels.h>


using namespace pcl;

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s {-p <pcd-file> OR -r <rgb-file> -d <depth-file>} \n [-P (print graph to console) \n-o <output-file> \n-l <output-label-file> \n-v <voxel resolution> \n-s <seed resolution> \n-c <color weight> \n-z <spatial weight> \n-n <normal_weight>] \n", argv[0]);
    return (1);
  }
  
  std::string rgb_path;
  bool rgb_file_specified = pcl::console::find_switch (argc, argv, "-r");
  if (rgb_file_specified)
    pcl::console::parse (argc, argv, "-r", rgb_path);
  
  std::string depth_path;
  bool depth_file_specified = pcl::console::find_switch (argc, argv, "-d");
  if (depth_file_specified)
    pcl::console::parse (argc, argv, "-d", depth_path);
  
  PointCloud<PointXYZRGB>::Ptr cloud = boost::make_shared <PointCloud<PointXYZRGB> >();
  bool pcd_file_specified = pcl::console::find_switch (argc, argv, "-p");
  std::string pcd_path;
  if (!depth_file_specified || !rgb_file_specified)
  {
    std::cout << "Using point cloud";
    if (!pcd_file_specified)
    {
      std::cout << "No cloud specified!";
      return (1);
    }else
    {
      pcl::console::parse (argc,argv,"-p",pcd_path);
    }
  }
  
  std::string out_path;
  bool output_file_specified = pcl::console::find_switch (argc, argv, "-o");
  if (output_file_specified)
    pcl::console::parse (argc, argv, "-o", out_path);
  else
    out_path = "test_output.png";
  
  std::string out_label_path;
  bool output_label_file_specified = pcl::console::find_switch (argc, argv, "-l");
  if (output_label_file_specified)
    pcl::console::parse (argc, argv, "-l", out_label_path);
  else
    out_label_path = "test_output_labels.png";
  
  float voxel_resolution = 0.008f;
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);
    
  float seed_resolution = 0.08f;
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);
  
  float color_importance = 0.2f;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);
  
  float spatial_importance = 0.4f;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);
  
  float normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);
  
  bool print_graph = false;
  if (pcl::console::find_switch (argc, argv, "-P"))
    print_graph = true;
  
  if (!pcd_file_specified)
  {
    //Read the images
    vtkSmartPointer<vtkImageReader2Factory> reader_factory = vtkSmartPointer<vtkImageReader2Factory>::New ();
    vtkImageReader2* rgb_reader = reader_factory->CreateImageReader2 (rgb_path.c_str ());
    //qDebug () << "RGB File="<< QString::fromStdString(rgb_path);
    if ( ! rgb_reader->CanReadFile (rgb_path.c_str ()))
    {
      std::cout << "Cannot read rgb image file!";
      return (1);
    }
    rgb_reader->SetFileName (rgb_path.c_str ());
    rgb_reader->Update ();
    //qDebug () << "Depth File="<<QString::fromStdString(depth_path);
    vtkImageReader2* depth_reader = reader_factory->CreateImageReader2 (depth_path.c_str ());
    if ( ! depth_reader->CanReadFile (depth_path.c_str ()))
    {
      std::cout << "Cannot read depth image file!";
      return (1);
    }
    depth_reader->SetFileName (depth_path.c_str ());
    depth_reader->Update ();
    
    vtkSmartPointer<vtkImageFlip> flipXFilter = vtkSmartPointer<vtkImageFlip>::New();
    flipXFilter->SetFilteredAxis(0); // flip x axis
    flipXFilter->SetInputConnection(rgb_reader->GetOutputPort());
    flipXFilter->Update();
    
    vtkSmartPointer<vtkImageFlip> flipXFilter2 = vtkSmartPointer<vtkImageFlip>::New();
    flipXFilter2->SetFilteredAxis(0); // flip x axis
    flipXFilter2->SetInputConnection(depth_reader->GetOutputPort());
    flipXFilter2->Update();
    
    vtkSmartPointer<vtkImageData> rgb_image = flipXFilter->GetOutput ();
    int *rgb_dims = rgb_image->GetDimensions ();
    vtkSmartPointer<vtkImageData> depth_image = flipXFilter2->GetOutput ();
    int *depth_dims = depth_image->GetDimensions ();
    
    if (rgb_dims[0] != depth_dims[0] || rgb_dims[1] != depth_dims[1])
    {
      std::cout << "Depth and RGB dimensions to not match!";
      std::cout << "RGB Image is of size "<<rgb_dims[0] << " by "<<rgb_dims[1];
      std::cout << "Depth Image is of size "<<depth_dims[0] << " by "<<depth_dims[1];
      return (1);
    }
 
    cloud->points.reserve (depth_dims[0] * depth_dims[1]);
    cloud->width = depth_dims[0];
    cloud->height = depth_dims[1];
    cloud->is_dense = false;
    
    
    // Fill in image data
    int centerX = static_cast<int>(cloud->width / 2.0);
    int centerY = static_cast<int>(cloud->height / 2.0);
    unsigned short* depth_pixel;
    unsigned char* color_pixel;
    float scale = 1.0f/1000.0f;
    float focal_length = 525.0f;
    float fl_const = 1.0f / focal_length;
    depth_pixel = static_cast<unsigned short*>(depth_image->GetScalarPointer (depth_dims[0]-1,depth_dims[1]-1,0));
    color_pixel = static_cast<unsigned char*> (rgb_image->GetScalarPointer (depth_dims[0]-1,depth_dims[1]-1,0));
    
    for (int y=0; y<cloud->height; ++y)
    {
      for (int x=0; x<cloud->width; ++x, --depth_pixel, color_pixel-=3)
      {
        PointXYZRGB new_point;
        //  uint8_t* p_i = &(cloud_blob->data[y * cloud_blob->row_step + x * cloud_blob->point_step]);
        float depth = static_cast<float>(*depth_pixel) * scale;
        if (depth == 0.0f)
        {
          new_point.x = new_point.y = new_point.z = std::numeric_limits<float>::quiet_NaN ();
        }
        else
        {
          new_point.x = (static_cast<float>(x - centerX)) * depth * fl_const;
          new_point.y = (static_cast<float>(centerY - y)) * depth * fl_const; // vtk seems to start at the bottom left image corner
          new_point.z = depth;
        }
        
        uint32_t rgb = static_cast<uint32_t>(color_pixel[0]) << 16 |  static_cast<uint32_t>(color_pixel[1]) << 8 |  static_cast<uint32_t>(color_pixel[2]);
        new_point.rgb = *reinterpret_cast<float*> (&rgb);
        cloud->points.push_back (new_point);
        
      }
    }
  }
  else
  {
    std::cout << "Loading pointcloud...";
    pcl::io::loadPCDFile (pcd_path, *cloud);
  }
    
  std::cout << "Done making cloud!";


  
  pcl::SuperVoxels<pcl::PointXYZRGB> super (voxel_resolution, seed_resolution);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (normal_importance);
  std::vector <pcl::PointIndices> superpixel_indices;
  pcl::PointCloud<pcl::PointSuperVoxel>::Ptr supervoxel_cloud;
  
  std::cout << "Extracting superpixels!\n";
  
  super.extract (supervoxel_cloud);
  std::cout << "Getting colorized voxel cloud\n";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = super.getColoredVoxelCloud ();
  std::cout << "Getting labeled voxel cloud\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud = super.getLabeledVoxelCloud ();
  
  std::cout << "Getting colorized full cloud\n";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_colored_cloud = super.getColoredCloud ();
  std::cout << "Getting labeled full cloud\n";
  pcl::PointCloud<pcl::PointXYZL>::Ptr full_labeled_cloud = super.getLabeledCloud ();
  // THESE ONLY MAKE SENSE FOR ORGANIZED CLOUDS
  //pcl::io::savePNGFile (out_path, *full_colored_cloud);
  //pcl::io::savePNGFile (out_label_path, *full_labeled_cloud);
  
  
  typedef std::pair<uint32_t, PointSuperVoxel> LabelCenterT;
  typedef boost::adjacency_list<boost::setS, boost::setS, boost::undirectedS, PointSuperVoxel, octree::EdgeProperties> VoxelAdjacencyList;
  typedef VoxelAdjacencyList::vertex_descriptor VoxelID;
  typedef VoxelAdjacencyList::edge_descriptor EdgeID;
  std::map<uint32_t, PointSuperVoxel> label_centers;  
  VoxelAdjacencyList supervoxel_adjacency_list;
  std::cout <<"Getting Supervoxel Adjacency List\n";
  super.getSuperVoxelAdjacencyList (supervoxel_adjacency_list);
  std::cout <<"Getting Supervoxel centers\n";
  super.getSuperVoxelCenters (label_centers);
      
  
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb (colored_cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud,rgb, "colored");
  
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "colored");
 
  //The vertices in the supervoxel adjacency list are the supervoxel centroids
  //This iterates through them, finding the edges
  typedef boost::graph_traits<VoxelAdjacencyList>::vertex_iterator VertexIterator;
  typedef boost::graph_traits<VoxelAdjacencyList>::adjacency_iterator AdjacencyIterator;
     
  std::pair<VertexIterator, VertexIterator> vertex_iterator_range;
  vertex_iterator_range = boost::vertices(supervoxel_adjacency_list);
  for (VertexIterator itr=vertex_iterator_range.first ; itr != vertex_iterator_range.second; ++itr)
  {
    PointSuperVoxel label_centroid = supervoxel_adjacency_list[*itr];
    std::pair<AdjacencyIterator, AdjacencyIterator> neighbors = boost::adjacent_vertices (*itr, supervoxel_adjacency_list);
    if(print_graph)
    {
      std::cout << "Label "<<label_centroid.label<<" is at ("<<label_centroid.x<<","<<label_centroid.y<<","<<label_centroid.z<<")\n";
      std::cout << "Edges: label --- edge length :\n";
    }
    for(AdjacencyIterator itr_neighbor = neighbors.first; itr_neighbor != neighbors.second; ++itr_neighbor)
    {
      //Get the edge connecting these supervoxels
      EdgeID connecting_edge = boost::edge (*itr,*itr_neighbor, supervoxel_adjacency_list).first;
      PointSuperVoxel neighbor_point = supervoxel_adjacency_list[*itr_neighbor];
      if (print_graph)
        std::cout <<"        "<< neighbor_point.label << "  ---  "<<supervoxel_adjacency_list[connecting_edge].length<<"\n";

      std::stringstream ss;
      ss << label_centroid.label<<"-"<<neighbor_point.label;
      //Draw lines connecting supervoxel centers
      viewer->addLine (label_centroid, neighbor_point,1.0,1.0,1.0, ss.str ());
    }
    if(print_graph)
      std::cout << "\n------------------------------------\n";
  }
  
  std::cout << "Loading viewer...";
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
 
