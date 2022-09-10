#include "pcl/recognition/face_detection/face_detector_data_provider.h"
#include "pcl/recognition/face_detection/face_common.h"
#include <pcl/common/point_tests.h> // for pcl::isFinite
#include <pcl/common/time.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/io/pcd_io.h>

#include <random>

//Uncomment the following lines and set PCL_FACE_DETECTION_VIS_TRAINING_FDDP to 1
//to visualize the training process and change the CMakeLists.txt accordingly.
//#include <pcl/visualization/pcl_visualizer.h>
//#define PCL_FACE_DETECTION_VIS_TRAINING_FDDP 1
//TODO: This is not very good as it forces recognition to depend on pcl_visualization
//much better solution would be pass a visualization function from the user code to make it transparent
//to the training process

namespace pcl
{
  namespace face_detection
  {
    inline
    void showBining(int num_pitch, float res_pitch, int min_pitch, int num_yaw, float res_yaw, int min_yaw, std::vector<std::vector<int> > & yaw_pitch_bins)
    {
      std::cout << "\t\t";
      for (int j = 0; j < num_pitch; j++)
      {
        std::cout << pcl_round (static_cast<float>(min_pitch) + res_pitch * static_cast<float>(j)) << "\t";
      }

      std::cout << std::endl;

      for (int i = 0; i < num_yaw; i++)
      {
        std::cout << pcl_round (static_cast<float>(min_yaw) + res_yaw * static_cast<float>(i)) << " => \t\t";
        for (int j = 0; j < num_pitch; j++)
        {
          //std::cout <<  std::setprecision(3) << yaw_pitch_bins[i][j] / static_cast<float>(max_elems) << "\t";
          std::cout << yaw_pitch_bins[i][j] << "\t";
        }

        std::cout << std::endl;
      }
    }
  }
}

template<class FeatureType, class DataSet, class LabelType, class ExampleIndex, class NodeType>
void pcl::face_detection::FaceDetectorDataProvider<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::initialize(std::string & data_dir)
{
  std::string start;
  std::string ext = std::string ("pcd");
  bf::path dir = data_dir;

  std::vector < std::string > files;
  getFilesInDirectory (dir, start, files, ext);

  //apart from loading the file names, we will do some bining regarding pitch and yaw
  std::vector < std::vector<int> > yaw_pitch_bins;
  std::vector < std::vector<std::vector<std::string> > > image_files_per_bin;

  float res_yaw = 15.f;
  float res_pitch = res_yaw;
  int min_yaw = -75;
  int min_pitch = -60;

  int num_yaw = static_cast<int>((std::abs (min_yaw) * 2) / static_cast<int>(res_yaw + 1.f));
  int num_pitch = static_cast<int>((std::abs (min_pitch) * 2) / static_cast<int>(res_pitch + 1.f));

  yaw_pitch_bins.resize (num_yaw);
  image_files_per_bin.resize (num_yaw);
  for (int i = 0; i < num_yaw; i++)
  {
    yaw_pitch_bins[i].resize (num_pitch);
    image_files_per_bin[i].resize (num_pitch);
    for (int j = 0; j < num_pitch; j++)
    {
      yaw_pitch_bins[i][j] = 0;
    }
  }

  for (const auto &filename : files)
  {
    std::string file = data_dir + '/' + filename;

    std::string pose_file (filename);
    boost::replace_all (pose_file, ".pcd", "_pose.txt");

    Eigen::Matrix4f pose_mat;
    pose_mat.setIdentity (4, 4);

    pose_file = data_dir + '/' + pose_file;

    if (readMatrixFromFile (pose_file, pose_mat))
    {
      Eigen::Vector3f ea = pose_mat.block<3, 3> (0, 0).eulerAngles (0, 1, 2);
      ea *= 57.2957795f; //transform it to degrees to do the binning
      int y = static_cast<int>(pcl_round ((ea[0] + static_cast<float>(std::abs (min_yaw))) / res_yaw));
      int p = static_cast<int>(pcl_round ((ea[1] + static_cast<float>(std::abs (min_pitch))) / res_pitch));

      if (y < 0)
        y = 0;
      if (p < 0)
        p = 0;
      if (p >= num_pitch)
        p = num_pitch - 1;
      if (y >= num_yaw)
        y = num_yaw - 1;

      assert (y >= 0 && y < num_yaw);
      assert (p >= 0 && p < num_pitch);

      yaw_pitch_bins[y][p]++;

      image_files_per_bin[y][p].push_back (file);
    }
  }

  pcl::face_detection::showBining (num_pitch, res_pitch, min_pitch, num_yaw, res_yaw, min_yaw, yaw_pitch_bins);

  int max_elems = 0;
  int total_elems = 0;

  for (int i = 0; i < num_yaw; i++)
  {
    for (int j = 0; j < num_pitch; j++)
    {
      total_elems += yaw_pitch_bins[i][j];
      if (yaw_pitch_bins[i][j] > max_elems)
        max_elems = yaw_pitch_bins[i][j];
    }
  }

  float average = static_cast<float> (total_elems) / (static_cast<float> (num_pitch + num_yaw));
  std::cout << "The average number of image per bin is:" << average << std::endl;

  std::cout << "Total number of images in the dataset:" << total_elems << std::endl;
  //reduce unbalance from dataset by capping the number of images per bin, keeping at least a certain min
  if (min_images_per_bin_ != -1)
  {
    std::cout << "Reducing unbalance of the dataset." << std::endl;
    std::mt19937 rng((std::random_device()()));
    for (int i = 0; i < num_yaw; i++)
    {
      for (int j = 0; j < num_pitch; j++)
      {
        if (yaw_pitch_bins[i][j] >= min_images_per_bin_)
        {
          std::shuffle (image_files_per_bin[i][j].begin (), image_files_per_bin[i][j].end (), rng);
          image_files_per_bin[i][j].resize (min_images_per_bin_);
          yaw_pitch_bins[i][j] = min_images_per_bin_;
        }

        for (const auto &ii : image_files_per_bin[i][j])
        {
          image_files_.push_back (ii);
        }
      }
    }
  }

  pcl::face_detection::showBining (num_pitch, res_pitch, min_pitch, num_yaw, res_yaw, min_yaw, yaw_pitch_bins);
  std::cout << "Total number of images in the dataset:" << image_files_.size () << std::endl;
}

//shuffle file and get the first num_images_ as requested by a tree
//extract positive and negative samples
//create training examples and labels

template<class FeatureType, class DataSet, class LabelType, class ExampleIndex, class NodeType>
void pcl::face_detection::FaceDetectorDataProvider<FeatureType, DataSet, LabelType, ExampleIndex, NodeType>::getDatasetAndLabels(DataSet & data_set,
    std::vector<LabelType> & label_data, std::vector<ExampleIndex> & examples)
{
  std::mt19937 rng((std::random_device()()));
  std::shuffle (image_files_.begin (), image_files_.end (), rng);
  std::vector < std::string > files;
  files = image_files_;
  files.resize (std::min (num_images_, static_cast<int> (files.size ())));

  std::vector < TrainingExample > training_examples;
  std::vector<float> labels;

  int total_neg, total_pos;
  total_neg = total_pos = 0;

#if PCL_FACE_DETECTION_VIS_TRAINING_FDDP == 1
  pcl::visualization::PCLVisualizer vis("training");
#endif

  for (std::size_t j = 0; j < files.size (); j++)
  {

#if PCL_FACE_DETECTION_VIS_TRAINING_FDDP == 1
    vis.removeAllPointClouds();
    vis.removeAllShapes();
#endif

    if ((j % 50) == 0)
    {
      std::cout << "Loading image..." << j << std::endl;
    }
    //1. Load clouds with and without labels
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::io::loadPCDFile (files[j], *loaded_cloud);

    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_labels (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::PointCloud<pcl::PointXYZL>::Ptr loaded_cloud_labels (new pcl::PointCloud<pcl::PointXYZL>);
    pcl::io::loadPCDFile (files[j], *loaded_cloud_labels);

    //crop images to remove as many NaNs as possible and reduce the memory footprint
    {
      std::size_t min_col, min_row;
      std::size_t max_col, max_row;
      min_col = min_row = std::numeric_limits<std::size_t>::max ();
      max_col = max_row = 0;

      for (std::size_t col = 0; col < loaded_cloud->width; col++)
      {
        for (std::size_t row = 0; row < loaded_cloud->height; row++)
        {
          if (pcl::isFinite (loaded_cloud->at (col, row)))
          {
            if (row < min_row)
              min_row = row;

            if (row > max_row)
              max_row = row;

            if (col < min_col)
              min_col = col;

            if (col > max_col)
              max_col = col;
          }
        }
      }

      //std::cout << min_col << " - " << max_col << std::endl;
      //std::cout << min_row << " - " << max_row << std::endl;

      cropCloud<pcl::PointXYZ> (min_col, max_col, min_row, max_row, *loaded_cloud, *cloud);
      cropCloud<pcl::PointXYZL> (min_col, max_col, min_row, max_row, *loaded_cloud_labels, *cloud_labels);

      /*pcl::visualization::PCLVisualizer vis ("training");
       vis.addPointCloud(loaded_cloud);
       vis.spin();*/
    }

    //Compute integral image over depth
    pcl::IntegralImage2D<float, 1>::Ptr integral_image_depth;
    integral_image_depth.reset (new pcl::IntegralImage2D<float, 1> (false));

    int element_stride = sizeof(pcl::PointXYZ) / sizeof(float);
    int row_stride = element_stride * cloud->width;
    const float *data = reinterpret_cast<const float*> (&(*cloud)[0]);
    integral_image_depth->setInput (data + 2, cloud->width, cloud->height, element_stride, row_stride);

    //Compute normals and normal integral images
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    if (USE_NORMALS_)
    {
      pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> n3d;
      n3d.setNormalEstimationMethod (n3d.COVARIANCE_MATRIX);
      n3d.setInputCloud (cloud);
      n3d.setRadiusSearch (0.02);
      n3d.setKSearch (0);
      {
        pcl::ScopeTime t ("compute normals...");
        n3d.compute (*normals);
      }
    }

    int element_stride_normal = sizeof(pcl::Normal) / sizeof(float);
    int row_stride_normal = element_stride_normal * normals->width;
    pcl::IntegralImage2D<float, 1>::Ptr integral_image_normal_x;
    pcl::IntegralImage2D<float, 1>::Ptr integral_image_normal_y;
    pcl::IntegralImage2D<float, 1>::Ptr integral_image_normal_z;

    if (USE_NORMALS_)
    {
      integral_image_normal_x.reset (new pcl::IntegralImage2D<float, 1> (false));
      const float *data_nx = reinterpret_cast<const float*> (&(*normals)[0]);
      integral_image_normal_x->setInput (data_nx, normals->width, normals->height, element_stride_normal, row_stride_normal);

      integral_image_normal_y.reset (new pcl::IntegralImage2D<float, 1> (false));
      const float *data_ny = reinterpret_cast<const float*> (&(*normals)[0]);
      integral_image_normal_y->setInput (data_ny + 1, normals->width, normals->height, element_stride_normal, row_stride_normal);

      integral_image_normal_z.reset (new pcl::IntegralImage2D<float, 1> (false));
      const float *data_nz = reinterpret_cast<const float*> (&(*normals)[0]);
      integral_image_normal_z->setInput (data_nz + 2, normals->width, normals->height, element_stride_normal, row_stride_normal);
    }

    //Using cloud labels estimate a 2D window from where to extract positive samples
    //Rest can be used to extract negative samples
    std::size_t min_col, min_row;
    std::size_t max_col, max_row;
    min_col = min_row = std::numeric_limits<std::size_t>::max ();
    max_col = max_row = 0;

    //std::cout << cloud_labels->width << " " << cloud_labels->height << std::endl;

    for (std::size_t col = 0; col < cloud_labels->width; col++)
    {
      for (std::size_t row = 0; row < cloud_labels->height; row++)
      {
        if (cloud_labels->at (col, row).label == 1)
        {
          if (row < min_row)
            min_row = row;

          if (row > max_row)
            max_row = row;

          if (col < min_col)
            min_col = col;

          if (col > max_col)
            max_col = col;
        }
      }
    }

#if PCL_FACE_DETECTION_VIS_TRAINING_FDDP == 1
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_intensity->width = cloud->width;
    cloud_intensity->height = cloud->height;
    cloud_intensity->points.resize(cloud->size());
    cloud_intensity->is_dense = cloud->is_dense;

    for (int jjj = 0; jjj < static_cast<int>(cloud->size()); jjj++)
    {
      (*cloud_intensity)[jjj].getVector4fMap() = (*cloud)[jjj].getVector4fMap();
      (*cloud_intensity)[jjj].intensity = 0.f;
      int row, col;
      col = jjj % cloud->width;
      row = jjj / cloud->width;
      //std::cout << row << " " << col << std::endl;
      if (check_inside(col, row, min_col, max_col, min_row, max_row))
      {
        (*cloud_intensity)[jjj].intensity = 1.f;
      }
    }

    pcl::visualization::PointCloudColorHandlerGenericField < pcl::PointXYZI > handler(cloud_intensity, "intensity");
    vis.addPointCloud(cloud_intensity, handler, "intensity_cloud");
#endif

    std::string pose_file (files[j]);
    boost::replace_all (pose_file, ".pcd", "_pose.txt");

    Eigen::Matrix4f pose_mat;
    pose_mat.setIdentity (4, 4);
    readMatrixFromFile (pose_file, pose_mat);

    Eigen::Vector3f ea = pose_mat.block<3, 3> (0, 0).eulerAngles (0, 1, 2);
    Eigen::Vector3f trans_vector = Eigen::Vector3f (pose_mat (0, 3), pose_mat (1, 3), pose_mat (2, 3));

    pcl::PointXYZ center_point;
    center_point.x = trans_vector[0];
    center_point.y = trans_vector[1];
    center_point.z = trans_vector[2];

    //************************************************
    //2nd training style, fanelli's journal description
    //************************************************
    {
      int N_patches = patches_per_image_;
      int pos_extracted = 0;
      int neg_extracted = 0;
      int w_size_2 = static_cast<int> (w_size_ / 2);

      using pixelpair = std::pair<int, int>;
      std::vector < pixelpair > negative_p, positive_p;
      //get negative and positive indices to sample from
      for (int col = 0; col < (static_cast<int> (cloud_labels->width) - w_size_); col++)
      {
        for (int row = 0; row < (static_cast<int> (cloud_labels->height) - w_size_); row++)
        {
          if (!pcl::isFinite (cloud->at (col + w_size_2, row + w_size_2)))
            continue;

          //reject patches with more than percent invalid values
          float percent = 0.5f;
          if (static_cast<float>(integral_image_depth->getFiniteElementsCount (col, row, w_size_, w_size_)) < (percent * static_cast<float>(w_size_ * w_size_)))
            continue;

          pixelpair pp = std::make_pair (col, row);
          if (cloud_labels->at (col + w_size_2, row + w_size_2).label == 1)
            positive_p.push_back (pp);
          else
            negative_p.push_back (pp);
        }
      }

      //shuffle and resize
      std::shuffle (positive_p.begin (), positive_p.end (), rng);
      std::shuffle (negative_p.begin (), negative_p.end (), rng);
      positive_p.resize (N_patches);
      negative_p.resize (N_patches);

      //extract positive patch
      for (const auto &p : positive_p)
      {
        int col = p.first;
        int row = p.second;

        pcl::PointXYZ patch_center_point;
        patch_center_point.x = cloud->at (col + w_size_2, row + w_size_2).x;
        patch_center_point.y = cloud->at (col + w_size_2, row + w_size_2).y;
        patch_center_point.z = cloud->at (col + w_size_2, row + w_size_2).z;

        TrainingExample te;
        te.iimages_.push_back (integral_image_depth);
        if (USE_NORMALS_)
        {
          te.iimages_.push_back (integral_image_normal_x);
          te.iimages_.push_back (integral_image_normal_y);
          te.iimages_.push_back (integral_image_normal_z);
        }

        te.row_ = row;
        te.col_ = col;
        te.wsize_ = w_size_;

        te.trans_ = center_point.getVector3fMap () - patch_center_point.getVector3fMap ();
        te.trans_ *= 1000.f; //transform it to millimeters
        te.rot_ = ea;
        te.rot_ *= 57.2957795f; //transform it to degrees

        labels.push_back (1);
        pos_extracted++;
        total_pos++;

        training_examples.push_back (te);
#if PCL_FACE_DETECTION_VIS_TRAINING_FDDP == 1
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity2(new pcl::PointCloud<pcl::PointXYZI>(*cloud_intensity));
        for (int jjj = col; jjj < (col + w_size_); jjj++)
        {
          for (int iii = row; iii < (row + w_size_); iii++)
          {
            cloud_intensity2->at(jjj, iii).intensity = 2.f;
          }
        }

        vis.removeAllPointClouds();

        pcl::visualization::PointCloudColorHandlerGenericField < pcl::PointXYZI > handler(cloud_intensity2, "intensity");
        vis.addPointCloud(cloud_intensity2, handler, "cloud");
        vis.spinOnce();
        vis.spin();
        sleep(1);
#endif

      }

#if PCL_FACE_DETECTION_VIS_TRAINING_FDDP == 1
      std::cout << "Going to extract negative patches..." << std::endl;
      sleep(2);
#endif

      for (const auto &p : negative_p)
      {
        int col = p.first;
        int row = p.second;

        TrainingExample te;
        te.iimages_.push_back (integral_image_depth);
        if (USE_NORMALS_)
        {
          te.iimages_.push_back (integral_image_normal_x);
          te.iimages_.push_back (integral_image_normal_y);
          te.iimages_.push_back (integral_image_normal_z);
        }

        te.row_ = row;
        te.col_ = col;
        te.wsize_ = w_size_;
        labels.push_back (0);
        neg_extracted++;
        total_neg++;

        training_examples.push_back (te);
#if PCL_FACE_DETECTION_VIS_TRAINING_FDDP == 1
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_intensity2(new pcl::PointCloud<pcl::PointXYZI>(*cloud_intensity));
        for (int jjj = col; jjj < (col + w_size_); jjj++)
        {
          for (int iii = row; iii < (row + w_size_); iii++)
          {
            cloud_intensity2->at(jjj, iii).intensity = 2.f;
          }
        }

        vis.removeAllPointClouds();

        pcl::visualization::PointCloudColorHandlerGenericField < pcl::PointXYZI > handler(cloud_intensity2, "intensity");
        vis.addPointCloud(cloud_intensity2, handler, "cloud");
        vis.spinOnce();
        vis.spin();
        sleep(1);
#endif
      }

      if (neg_extracted != N_patches)
      {
        std::cout << "Extracted " << neg_extracted << " negative patches" << std::endl;
        std::cout << files[j] << std::endl;
      }

      if (pos_extracted != N_patches)
      {
        std::cout << "Extracted " << pos_extracted << " positive patches" << std::endl;
        std::cout << files[j] << std::endl;
      }
    }
  }

  std::cout << training_examples.size () << " " << labels.size () << " " << total_neg << " " << total_pos << std::endl;

  //train random forest and make persistent
  std::vector<int> example_indices;
  for (std::size_t i = 0; i < labels.size (); i++)
    example_indices.push_back (static_cast<int> (i));

  label_data = labels;
  data_set = training_examples;
  examples = example_indices;
}

template class pcl::face_detection::FaceDetectorDataProvider<pcl::face_detection::FeatureType, std::vector<pcl::face_detection::TrainingExample>, float, int,
    pcl::face_detection::RFTreeNode<pcl::face_detection::FeatureType> >;
