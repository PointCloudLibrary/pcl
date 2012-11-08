#include <pcl/apps/test_voxel_superpixels.h>

using namespace pcl;

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s {-r <rgb-file> -d <depth-file>} [-o <output-file> -l <output-label-file> -v <voxel resolution> -s <seed resolution>] \n", argv[0]);
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
  
  if (!depth_file_specified || !rgb_file_specified)
  {
    qCritical () << "Please specify rgb and depth file";
    return (1);
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
  
  float voxel_resolution = 0.015;
  bool voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", voxel_resolution);
    
  float seed_resolution = 0.12;
  bool seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (seed_res_specified)
    pcl::console::parse (argc, argv, "-s", seed_resolution);
  
  float color_importance = 1.0;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", color_importance);
  
  float spatial_importance = 1.0;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", spatial_importance);
  
  float shape_importance = 1.0;
  if (pcl::console::find_switch (argc, argv, "-f"))
    pcl::console::parse (argc, argv, "-f", shape_importance);
  
  
  //Read the images
  vtkSmartPointer<vtkImageReader2Factory> reader_factory = vtkSmartPointer<vtkImageReader2Factory>::New ();
  vtkImageReader2* rgb_reader = reader_factory->CreateImageReader2 (rgb_path.c_str ());
  qDebug () << "RGB File="<< QString::fromStdString(rgb_path);
  if ( ! rgb_reader->CanReadFile (rgb_path.c_str ()))
  {
    qCritical () << "Cannot read rgb image file!";
    return (1);
  }
  rgb_reader->SetFileName (rgb_path.c_str ());
  rgb_reader->Update ();
  qDebug () << "Depth File="<<QString::fromStdString(depth_path);
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
  qDebug () << "Images loaded, making cloud";
  PointCloud<PointXYZRGB>::Ptr cloud = boost::make_shared <PointCloud<PointXYZRGB> >();
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
      float depth = (float)(*depth_pixel) * scale;
    //  qDebug () << "Depth = "<<depth;
      if (depth == 0.0f)
      {
        new_point.x = new_point.y = new_point.z = std::numeric_limits<float>::quiet_NaN ();
      }
      else
      {
        new_point.x = ((float)(x - centerX)) * depth * fl_const;
        new_point.y = ((float)(centerY - y)) * depth * fl_const; // vtk seems to start at the bottom left image corner
        new_point.z = depth;
      }
      
      uint32_t rgb = (uint32_t)color_pixel[0] << 16 | (uint32_t)color_pixel[1] << 8 | (uint32_t)color_pixel[2];
      new_point.rgb = *reinterpret_cast<float*> (&rgb);
      cloud->points.push_back (new_point);
      //   qDebug () << "depth = "<<depth << "x,y,z="<<data[0]<<","<<data[1]<<","<<data[2];
      //qDebug() << "r ="<<color_pixel[0]<<" g="<<color_pixel[1]<<" b="<<color_pixel[2];
      
    }
  }
  qDebug () << "Done making cloud!"; 
  
  pcl::VoxelSuperpixels<pcl::PointXYZRGB> super;
  super.setInputCloud (cloud);
  super.setVoxelResolution (voxel_resolution);
  super.setSeedResolution (seed_resolution);
  super.setColorImportance (color_importance);
  super.setFPFHImportance (shape_importance);
  spatial_importance = 0.4 * (color_importance+shape_importance);
  super.setSpatialImportance (spatial_importance);
  std::vector <pcl::PointIndices> superpixel_indices;
  qDebug () << "Extracting superpixels!";
  super.extract (superpixel_indices);
  QString out_name;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = super.getColoredCloud ();
  pcl::PointCloud<pcl::PointXYZL>::Ptr label_cloud = super.getLabeledCloud();
 // pcl::PointCloud<pcl::PointXYZRGB>::Ptr seed_cloud = super.getSeedCloud (); 
 // pcl::io::savePNGFile ("seed_cloud.png", *seed_cloud);
  /* //Debugging code to show effect of iterations
  out_name.sprintf ("%s%04d%s","test/",0,".png");
  pcl::io::savePNGFile (out_name.toStdString (), *colored_cloud);
  float flow_r = 0.05f;
  int num_itr = 300;
  for (int i =1; i< num_itr; ++i)
  {
    super.iterateEvolvingSet (flow_r);
    colored_cloud = super.getColoredCloud ();
    QString out_name;
    out_name.sprintf ("%s%06d%s","test/",int(flow_r*i*10),".png");
    pcl::io::savePNGFile (out_name.toStdString (), *colored_cloud);
    
    
  }*/
  
 // qDebug () << "Writing out file to " << QString::fromStdString (out_path);
 // qDebug () << "Output cloud size:"<<colored_cloud->width<<"x"<<colored_cloud->height;
 pcl::io::savePNGFile (out_label_path, *label_cloud);
 pcl::io::savePNGFile (out_path, *colored_cloud);

  
  return (0);
}
 