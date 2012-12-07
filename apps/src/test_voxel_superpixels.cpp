#include <pcl/apps/test_voxel_superpixels.h>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//#include <sys/time.h>
using namespace pcl;

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s {-p <pcd-file> OR -r <rgb-file> -d <depth-file>} [-o <output-file> -l <output-label-file> -v <voxel resolution> -s <seed resolution> -c <color weight> -z <spatial weight> -f <fpfh weight> -n <normal_weight>] \n", argv[0]);
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
    qDebug () << "Using point cloud";
    if (!pcd_file_specified)
    {
      qDebug () << "No cloud specified!";
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
  
  float shape_importance = 0.1f;
  if (pcl::console::find_switch (argc, argv, "-f"))
    pcl::console::parse (argc, argv, "-f", shape_importance);
  
  float normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", normal_importance);
  
  if (!pcd_file_specified)
  {
    //Read the images
    vtkSmartPointer<vtkImageReader2Factory> reader_factory = vtkSmartPointer<vtkImageReader2Factory>::New ();
    vtkImageReader2* rgb_reader = reader_factory->CreateImageReader2 (rgb_path.c_str ());
    //qDebug () << "RGB File="<< QString::fromStdString(rgb_path);
    if ( ! rgb_reader->CanReadFile (rgb_path.c_str ()))
    {
      qCritical () << "Cannot read rgb image file!";
      return (1);
    }
    rgb_reader->SetFileName (rgb_path.c_str ());
    rgb_reader->Update ();
    //qDebug () << "Depth File="<<QString::fromStdString(depth_path);
    vtkImageReader2* depth_reader = reader_factory->CreateImageReader2 (depth_path.c_str ());
    if ( ! depth_reader->CanReadFile (depth_path.c_str ()))
    {
      qCritical () << "Cannot read depth image file!";
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
      qCritical () << "Depth and RGB dimensions to not match!";
      qDebug () << "RGB Image is of size "<<rgb_dims[0] << " by "<<rgb_dims[1];
      qDebug () << "Depth Image is of size "<<depth_dims[0] << " by "<<depth_dims[1];
      return (1);
    }
  //qDebug () << "Images loaded, making cloud";
  
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
      //  qDebug () << "Depth = "<<depth;
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
        //   qDebug () << "depth = "<<depth << "x,y,z="<<data[0]<<","<<data[1]<<","<<data[2];
        //qDebug() << "r ="<<color_pixel[0]<<" g="<<color_pixel[1]<<" b="<<color_pixel[2];
        
      }
    }
  }
  else
  {
    qDebug () << "Loading pointcloud...";
    pcl::io::loadPCDFile (pcd_path, *cloud);
  }
    
  qDebug () << "Done making cloud!";


  
  pcl::VoxelSuperpixels<pcl::PointXYZRGB> super (voxel_resolution, seed_resolution);
  super.setInputCloud (cloud);
  super.setColorImportance (color_importance);
  super.setFPFHImportance (shape_importance);
  super.setSpatialImportance (spatial_importance);
  super.setNormalImportance (spatial_importance);
  std::vector <pcl::PointIndices> superpixel_indices;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud;
  
  qDebug () << "Extracting superpixels!";
  
//  timeval t1, t2;
//  double elapsedTime;
//  gettimeofday(&t1,NULL);
  super.extract (voxel_cloud, superpixel_indices);
  qDebug () << "Num Superpixels = "<<superpixel_indices.size ();
//  gettimeofday(&t2,NULL);
//  elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
 // elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
  std::cerr << voxel_resolution << " "<<seed_resolution <<" "<< super.getVoxelCloudSize() <<"\n";

  
  std::vector <std::set<int> > superpixel_neighbors;
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > superpixel_centers;
  qDebug () << "Getting neighbors";
  super.getSuperpixelNeighbors (superpixel_neighbors);
  qDebug () << "Getting Centers";
  super.getSuperpixelCenters (superpixel_centers);
  qDebug () << "Getting Adjacency List";
  std::vector<std::vector<int> > voxel_adjacency_list;
  super.getAdjacencyList (voxel_adjacency_list);
  QString out_name;
  qDebug () << "Getting Labeled Cloud";
  pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud = super.getLabeledVoxelCloud();
  qDebug () << "Getting colorized voxel cloud";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = super.getColoredVoxelCloud ();
  qDebug () << "Getting colorized original cloud";
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_colored = super.getColoredCloud (); 
  
 // pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed_cloud = super.getSeedCloud ();
 // pcl::io::savePNGFile ("seed_cloud.png", *seed_cloud);
  
  // THESE ONLY MAKE SENSE FOR ORGANIZED CLOUDS
   qDebug () << "Writing out file to " << QString::fromStdString (out_path);
   qDebug () << "Output cloud size:"<<full_colored->width<<"x"<<full_colored->height;
  pcl::io::savePNGFile (out_path, *full_colored);
  //pcl::io::savePNGFile (out_label_path, *label_cloud);
  
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colored_cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (colored_cloud,rgb, "colored");
  
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "colored");
  //Draw lines connecting superpixel centers
  qDebug () << "Superpixel neighbor size="<<superpixel_neighbors.size ()<<"\n";
  
  int linecount = 0;
  for (int i = 0; i < superpixel_neighbors.size(); ++i)
  {
    std::set<int>::iterator it;
    for (it = superpixel_neighbors[i].begin (); it != superpixel_neighbors[i].end (); ++it)
    {
      if ( *it != i && i < *it)
      {
        pcl::PointXYZ p1 = superpixel_centers[i];
        pcl::PointXYZ p2 = superpixel_centers[*it];
        QString linename;
        linename.sprintf("%05d",linecount);
        viewer->addLine (p1, p2,1.0,1.0,1.0, linename.toStdString());
        linecount++;
      }
    }
  }
  qDebug () << "Loading viewer...";
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return (0);
}
 
