#ifndef PCL_FEATURES_IMPL_SURFLET_HPP_
#define PCL_FEATURES_IMPL_SURFLET_HPP_

#include "pcl/features/surflet.h"
#include <pcl/features/pfh.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>

//// need to be removed - just for debugging purposes
#include <iostream>
using namespace std;


//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> typename pcl::SurfletEstimation<PointInT, PointOutT>::FeatureHashMapTypePtr
pcl::SurfletEstimation<PointInT, PointOutT>::computeSurfletModel (
    const pcl::PointCloud<PointInT> &cloud /* output goes here */)
    {
  cerr << "Computing Surflet Model function called" << endl;

  cerr << "Subsampling ..." << endl;
  pcl::PointCloud<PointInT> cloud_subsampled;
  /// subsample point cloud such that min dist between points is d_dist (+parameter)
  pcl::VoxelGrid<PointInT> subsampling_filter;
  /// @todo don't make a copy here - use input_ from feature
  PointCloudIn cloud_ptr = pcl::PointCloud<PointInT> (cloud).makeShared ();
  subsampling_filter.setInputCloud (cloud_ptr);
  subsampling_filter.setLeafSize (0.01, 0.01, 0.01); /// @TODO parameter goes here
  subsampling_filter.filter (cloud_subsampled);
  cerr << "After subsampling model, points: " << cloud_subsampled.width << " / " << cloud.width << endl;

  cerr << "Estimating normals ..." << endl;
  pcl::PointCloud<Normal> cloud_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  pcl::NormalEstimation<PointInT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_subsampled.makeShared());

  typename pcl::KdTreeFLANN<PointInT>::Ptr search_tree (new pcl::KdTreeFLANN<PointInT>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch(0.05); // @TODO another parameter
  normal_estimation_filter.compute (cloud_subsampled_normals);

  cerr << "Computing feature hash map" << endl;
  /// compute feature vector for each pair of points in the subsampled point cloud
  /// @TODO currently considering only unorganized pointclouds
  float f1, f2, f3, f4;
  int d1, d2, d3, d4;
  FeatureHashMapTypePtr feature_hash_map (new FeatureHashMapType);
  for (size_t i = 0; i < cloud_subsampled.width; ++i)
    for(size_t j = 0; j < cloud_subsampled.width; ++j) if (i != j) {
      /// create Vector4f's from point clouds
      Eigen::Vector4f point1 (cloud_subsampled.points[i].x, cloud_subsampled.points[i].y, cloud_subsampled.points[i].z, 0),
          point2 (cloud_subsampled.points[j].x, cloud_subsampled.points[j].y, cloud_subsampled.points[j].z, 0),
          normal1 (cloud_subsampled_normals.points[i].normal_x, cloud_subsampled_normals.points[i].normal_y, cloud_subsampled_normals.points[i].normal_z, 0),
          normal2 (cloud_subsampled_normals.points[j].normal_x, cloud_subsampled_normals.points[j].normal_y, cloud_subsampled_normals.points[j].normal_z, 0);
      if (pcl::computePairFeatures (point1, normal1, point2, normal2, f1, f2, f3, f4)) {
        /// discretize feature vector
        d1 = floor(f1 / angle_discretization_step);
        d2 = floor(f2 / angle_discretization_step);
        d3 = floor(f3 / angle_discretization_step);
        d4 = floor(f4 / distance_discretization_step);

        /// add feature to hash map
        feature_hash_map->insert( pair <HashKeyStruct, pair <size_t, size_t> > (HashKeyStruct (d1, d2, d3, d4), pair<size_t, size_t> (i, j)));
        //     cerr << d1 << " " << d2 << " " << d3 << " " << d4 << endl;
        //     cerr << f1 << " " << f2 << " " << f3 << " " << f4 << endl << "------" << endl;
      }
      else {
        cerr << "Computing pair feature vector between points " << i << " and " << j << " went wrong." << endl;
        /// @TODO do something if fail
      }
    }

  return feature_hash_map;
    }




bool resultsCompareFunction (std::pair <Eigen::Affine3f, float> a, std::pair <Eigen::Affine3f, float> b )
{
  return (a.second > b.second);
}


//////////////////////////////////////////////////////////////////////////////////////////////
/// @TODO !!! cloud_model should be subsampled as before!!
template <typename PointInT, typename PointOutT> std::vector <std::pair <Eigen::Affine3f, float> > /// @todo very ugly hack with PointOutT = normals
pcl::SurfletEstimation<PointInT, PointOutT>::registerModelToScene (const pcl::PointCloud<PointInT> &cloud_model, const pcl::PointCloud<PointOutT> &cloud_model_normals, const pcl::PointCloud<PointInT> &cloud_scene, typename pcl::SurfletEstimation<PointInT, PointOutT>::FeatureHashMapTypePtr feature_hashmap_model)
{
  std::vector <std::pair <Eigen::Affine3f, float> > results;
  std::vector <std::vector <unsigned int> > accumulator_array;
  accumulator_array.resize (cloud_model.width);
  for (size_t i = 0; i < cloud_model.width; ++i)
  {
    std::vector <unsigned int> aux ((size_t)floor(2*M_PI / angle_discretization_step ), 0);
    accumulator_array[i] = aux;
  }
  cerr << "Accumulator array size: " << accumulator_array.size() << " x " << accumulator_array.back ().size () << endl;

  //  unsigned int accumulator_array [cloud_model.width][ (size_t)floor(2*M_PI / angle_discretization_step )];

  /// subsample scene cloud with same rate as the model cloud
  pcl::PointCloud<PointInT> cloud_scene_subsampled;
  /// subsample point cloud such that min dist between points is d_dist (+parameter)
  pcl::VoxelGrid<PointInT> subsampling_filter;
  /// @todo don't make a copy here
  PointCloudIn cloud_scene_ptr = pcl::PointCloud<PointInT> (cloud_scene).makeShared ();
  subsampling_filter.setInputCloud (cloud_scene_ptr);
  subsampling_filter.setLeafSize (0.01, 0.01, 0.01); /// @TODO parameter goes here
  subsampling_filter.filter (cloud_scene_subsampled);

  /// calculate subsampled scene cloud normals
  pcl::PointCloud<Normal> cloud_scene_subsampled_normals;
  /// recompute normals of the subsampled surfaces
  pcl::NormalEstimation<PointInT, pcl::Normal> normal_estimation_filter;
  normal_estimation_filter.setInputCloud (cloud_scene_subsampled.makeShared());
  typename pcl::KdTreeFLANN<PointInT>::Ptr search_tree (new pcl::KdTreeFLANN<PointInT>);
  normal_estimation_filter.setSearchMethod (search_tree);
  normal_estimation_filter.setRadiusSearch(0.05); // @TODO another parameter
  normal_estimation_filter.compute (cloud_scene_subsampled_normals);


  /// consider every N/5-th point in the scene as a reference point s_r @TODO parameter here
  float f1, f2, f3, f4;
  int d1, d2, d3, d4;
  for (size_t scene_reference_index = 0; scene_reference_index < cloud_scene_subsampled.width; scene_reference_index += 5)
  {
    Eigen::Vector3f scene_reference_point (cloud_scene_subsampled.points[scene_reference_index].x, cloud_scene_subsampled.points[scene_reference_index].y, cloud_scene_subsampled.points[scene_reference_index].z),
        scene_reference_normal (cloud_scene_subsampled_normals.points[scene_reference_index].normal);
    Eigen::AngleAxisf rotation_sg (acos (scene_reference_normal.dot (Eigen::Vector3f::UnitX ())), scene_reference_normal.cross (Eigen::Vector3f::UnitX ()));
    Eigen::Affine3f transform_sg = Eigen::Translation3f ( rotation_sg* ((-1)*scene_reference_point)) * rotation_sg;

    for (size_t scene_point_index = 0; scene_point_index < cloud_scene_subsampled.width; ++ scene_point_index)
      if (scene_reference_index != scene_point_index)
      {
        Eigen::Vector4f point1 (cloud_scene_subsampled.points[scene_reference_index].x, cloud_scene_subsampled.points[scene_reference_index].y, cloud_scene_subsampled.points[scene_reference_index].z, 0),
            point2 (cloud_scene_subsampled.points[scene_point_index].x, cloud_scene_subsampled.points[scene_point_index].y, cloud_scene_subsampled.points[scene_point_index].z, 0),
            normal1 (cloud_scene_subsampled_normals.points[scene_reference_index].normal_x, cloud_scene_subsampled_normals.points[scene_reference_index].normal_y, cloud_scene_subsampled_normals.points[scene_reference_index].normal_z, 0),
            normal2 (cloud_scene_subsampled_normals.points[scene_point_index].normal_x, cloud_scene_subsampled_normals.points[scene_point_index].normal_y, cloud_scene_subsampled_normals.points[scene_point_index].normal_z, 0);
        if (pcl::computePairFeatures (point1, normal1, point2, normal2, f1, f2, f3, f4)) {
          /// discretize feature vector
          d1 = floor(f1 / angle_discretization_step);
          d2 = floor(f2 / angle_discretization_step);
          d3 = floor(f3 / angle_discretization_step);
          d4 = floor(f4 / distance_discretization_step);

          HashKeyStruct key = HashKeyStruct (d1, d2, d3, d4);
          pair <typename FeatureHashMapType::iterator, typename FeatureHashMapType::iterator> map_iterator_pair = feature_hashmap_model->equal_range (key);
          for (; map_iterator_pair.first != map_iterator_pair.second; ++ map_iterator_pair.first)
          {
            size_t model_reference_index = map_iterator_pair.first->second.first,
                model_point_index = map_iterator_pair.first->second.second;
            /// calculate angle alpha
            Eigen::Vector3f model_reference_point (cloud_model.points[model_reference_index].x, cloud_model.points[model_reference_index].y, cloud_model.points[model_reference_index].z),
                model_reference_normal (cloud_model_normals.points[model_reference_index].normal),
                scene_point (cloud_scene_subsampled.points[scene_point_index].x, cloud_scene_subsampled.points[scene_point_index].y, cloud_scene_subsampled.points[scene_point_index].z),
                model_point (cloud_model.points[model_point_index].x, cloud_model.points[model_point_index].y, cloud_model.points[model_point_index].z);
            Eigen::AngleAxisf rotation_mg (acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ())), model_reference_normal.cross (Eigen::Vector3f::UnitX ()));
            Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;

            //            cerr << "Test - should be origin " << transform_mg * model_reference_point << "     " << transform_sg * scene_reference_point << endl;

            float alpha = acos ((transform_sg * scene_point).dot (transform_mg * model_point));
            Eigen::Vector3f axis_test = ((transform_sg*scene_point).normalized().cross( (transform_mg*model_point).normalized())).normalized();
            //            cerr << "axis should be UnitX: " << axis_test << endl;

            unsigned int alpha_discretized = floor(alpha) + floor(M_PI / angle_discretization_step);
            accumulator_array[model_reference_index][alpha_discretized] ++;
          }

        }
        else {
          cerr << "Computing pair feature vector between points " << scene_reference_index << " and " << scene_point_index << " went wrong." << endl;
          /// @TODO do something if fail
        }
      }

    unsigned int max_votes = 0;
    Eigen::Affine3f max_transform;
    for (size_t i = 0; i < accumulator_array.size(); ++i)
      for (size_t j = 0; j < accumulator_array.back().size(); ++j) {
        unsigned int val = accumulator_array[i][j];
        if (val > max_votes)
        {
          max_votes = val;

          Eigen::Vector3f model_reference_point (cloud_model.points[i].x, cloud_model.points[i].y, cloud_model.points[i].z),
              model_reference_normal (cloud_model_normals.points[i].normal);
          Eigen::AngleAxisf rotation_mg (acos (model_reference_normal.dot (Eigen::Vector3f::UnitX ())), model_reference_normal.cross (Eigen::Vector3f::UnitX ()));
          Eigen::Affine3f transform_mg = Eigen::Translation3f ( rotation_mg * ((-1) * model_reference_point)) * rotation_mg;
          max_transform = transform_sg.inverse () * Eigen::AngleAxisf ( (j - floor(M_PI / angle_discretization_step)) * angle_discretization_step, Eigen::Vector3f::UnitX ()) * transform_mg;
        }

        /// reset accumulator_array for the next set of iterations with a new scene reference point
        accumulator_array[i][j] = 0;
      }

//    cerr << "max_votes: " << max_votes << endl;
    results.push_back (pair <Eigen::Affine3f, float> (max_transform, max_votes));
  }


  sort(results.begin(), results.end(), resultsCompareFunction);


  /// @todo should also add pose clustering part for filtering out outliers and improving transformation


  return results;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
pcl::SurfletEstimation<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{

}




#define PCL_INSTANTIATE_SurfletEstimation(T,OutT) template class PCL_EXPORTS pcl::SurfletEstimation<T,OutT>;


#endif /* PCL_FEATURES_IMPL_SURFLET_HPP_ */
