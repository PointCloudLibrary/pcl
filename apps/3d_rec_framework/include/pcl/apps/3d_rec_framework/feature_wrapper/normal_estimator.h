/*
 * normal_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_NORMAL_ESTIMATOR_H_
#define REC_FRAMEWORK_NORMAL_ESTIMATOR_H_

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/common/time.h>

namespace pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename PointOutT>
      class PreProcessorAndNormalEstimator
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;

        float
        computeMeshResolution (PointInTPtr & input)
        {
          typedef typename pcl::KdTree<PointInT>::Ptr KdTreeInPtr;
          KdTreeInPtr tree = boost::make_shared<pcl::KdTreeFLANN<PointInT> > (false);
          tree->setInputCloud (input);

          std::vector<int> nn_indices (9);
          std::vector<float> nn_distances (9);
          std::vector<int> src_indices;

          float sum_distances = 0.0;
          std::vector<float> avg_distances (input->points.size ());
          // Iterate through the source data set
          for (size_t i = 0; i < input->points.size (); ++i)
          {
            tree->nearestKSearch (input->points[i], 9, nn_indices, nn_distances);

            float avg_dist_neighbours = 0.0;
            for (size_t j = 1; j < nn_indices.size (); j++)
              avg_dist_neighbours += std::sqrt (nn_distances[j]);

            avg_dist_neighbours /= static_cast<float> (nn_indices.size ());

            avg_distances[i] = avg_dist_neighbours;
            sum_distances += avg_dist_neighbours;
          }

          std::sort (avg_distances.begin (), avg_distances.end ());
          float avg = avg_distances[static_cast<int> (avg_distances.size ()) / 2 + 1];
          return avg;
        }

      public:

        bool compute_mesh_resolution_;
        bool do_voxel_grid_;
        bool remove_outliers_;

        //this values are used when CMR=false
        float grid_resolution_;
        float normal_radius_;

        //this are used when CMR=true
        float factor_normals_;
        float factor_voxel_grid_;
        float mesh_resolution_;

        PreProcessorAndNormalEstimator ()
        {
          remove_outliers_ = do_voxel_grid_ = compute_mesh_resolution_ = false;
        }

        void
        setFactorsForCMR (float f1, float f2)
        {
          factor_voxel_grid_ = f1;
          factor_normals_ = f2;
        }

        void
        setValuesForCMRFalse (float f1, float f2)
        {
          grid_resolution_ = f1;
          normal_radius_ = f2;
        }

        void
        setDoVoxelGrid (bool b)
        {
          do_voxel_grid_ = b;
        }

        void
        setRemoveOutliers (bool b)
        {
          remove_outliers_ = b;
        }

        void
        setCMR (bool b)
        {
          compute_mesh_resolution_ = b;
        }

        void
        estimate (PointInTPtr & in, PointInTPtr & out, pcl::PointCloud<pcl::Normal>::Ptr & normals)
        {
          if (compute_mesh_resolution_)
          {
            mesh_resolution_ = computeMeshResolution (in);
            std::cout << "compute mesh resolution:" << mesh_resolution_ << std::endl;
          }

          if (do_voxel_grid_)
          {
            pcl::ScopeTime t ("Voxel grid...");
            float voxel_grid_size = grid_resolution_;
            if (compute_mesh_resolution_)
            {
              voxel_grid_size = mesh_resolution_ * factor_voxel_grid_;
            }

            pcl::VoxelGrid<PointInT> grid_;
            grid_.setInputCloud (in);
            grid_.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
            grid_.setDownsampleAllData (true);
            grid_.filter (*out);
          }
          else
          {
            out = in;
          }

          if (out->points.size () == 0)
          {
            PCL_WARN("NORMAL estimator: Cloud has no points after voxel grid, wont be able to compute normals!\n");
            return;
          }

          if (remove_outliers_)
          {
            pcl::ScopeTime t ("remove_outliers_...");
            PointInTPtr out2 (new pcl::PointCloud<PointInT> ());
            float radius = normal_radius_;
            if (compute_mesh_resolution_)
            {
              radius = mesh_resolution_ * factor_normals_;
              if (do_voxel_grid_)
                radius *= factor_voxel_grid_;
            }

            //in synthetic views the render grazes some parts of the objects
            //thus creating a very sparse set of points that causes the normals to be very noisy
            //remove these points
            pcl::RadiusOutlierRemoval<PointInT> sor;
            sor.setInputCloud (out);
            sor.setRadiusSearch (radius);
            sor.setMinNeighborsInRadius (16);
            sor.filter (*out2);
            out = out2;

          }

          if (out->points.size () == 0)
          {
            PCL_WARN("NORMAL estimator: Cloud has no points after removing outliers...!\n");
            return;
          }

          float radius = normal_radius_;
          if (compute_mesh_resolution_)
          {
            radius = mesh_resolution_ * factor_normals_;
            if (do_voxel_grid_)
              radius *= factor_voxel_grid_;
          }

          if (out->isOrganized ())
          {
            typedef typename pcl::IntegralImageNormalEstimation<PointInT, pcl::Normal> NormalEstimator_;
            NormalEstimator_ n3d;
            n3d.setNormalEstimationMethod (n3d.COVARIANCE_MATRIX);
            //n3d.setNormalEstimationMethod (n3d.AVERAGE_3D_GRADIENT);
            n3d.setInputCloud (out);
            n3d.setRadiusSearch (radius);
            n3d.setKSearch (0);
            //n3d.setMaxDepthChangeFactor(0.02f);
            //n3d.setNormalSmoothingSize(15.0f);
            {
              pcl::ScopeTime t ("compute normals...");
              n3d.compute (*normals);
            }
          }
          else
          {

            //check nans before computing normals
            {
              pcl::ScopeTime t ("check nans...");
              int j = 0;
              for (size_t i = 0; i < out->points.size (); ++i)
              {
                if (!pcl_isfinite (out->points[i].x) || !pcl_isfinite (out->points[i].y) || !pcl_isfinite (out->points[i].z))
                  continue;

                out->points[j] = out->points[i];
                j++;
              }

              if (j != static_cast<int> (out->points.size ()))
              {
                PCL_ERROR("Contain nans...");
              }

              out->points.resize (j);
              out->width = j;
              out->height = 1;
            }

            typedef typename pcl::NormalEstimation<PointInT, pcl::Normal> NormalEstimator_;
            NormalEstimator_ n3d;
            typename pcl::search::KdTree<PointInT>::Ptr normals_tree (new pcl::search::KdTree<PointInT>);
            normals_tree->setInputCloud (out);
            n3d.setRadiusSearch (radius);
            n3d.setSearchMethod (normals_tree);
            n3d.setInputCloud (out);
            {
              pcl::ScopeTime t ("compute normals not organized...");
              n3d.compute (*normals);
            }
          }

          //check nans...
          if (!out->isOrganized ())
          {
            pcl::ScopeTime t ("check nans...");
            int j = 0;
            for (size_t i = 0; i < normals->points.size (); ++i)
            {
              if (!pcl_isfinite (normals->points[i].normal_x) || !pcl_isfinite (normals->points[i].normal_y)
                  || !pcl_isfinite (normals->points[i].normal_z))
                continue;

              normals->points[j] = normals->points[i];
              out->points[j] = out->points[i];
              j++;
            }

            normals->points.resize (j);
            normals->width = j;
            normals->height = 1;

            out->points.resize (j);
            out->width = j;
            out->height = 1;
          }
          else
          {
            //is is organized, we set the xyz points to NaN
            pcl::ScopeTime t ("check nans organized...");
            bool NaNs = false;
            for (size_t i = 0; i < normals->points.size (); ++i)
            {
              if (pcl_isfinite (normals->points[i].normal_x) && pcl_isfinite (normals->points[i].normal_y)
                  && pcl_isfinite (normals->points[i].normal_z))
                continue;

              NaNs = true;

              out->points[i].x = out->points[i].y = out->points[i].z = std::numeric_limits<float>::quiet_NaN ();
            }

            if (NaNs)
            {
              PCL_WARN("normals contain NaNs\n");
              out->is_dense = false;
            }
          }

          /*for (size_t i = 0; i < out->points.size (); i++)
          {
            int r, g, b;
            r = static_cast<int> (out->points[i].r);
            g = static_cast<int> (out->points[i].g);
            b = static_cast<int> (out->points[i].b);
            std::cout << "in normal estimator:" << r << " " << g << " " << b << std::endl;
          }*/
        }
      };
  }
}

#endif /* REC_FRAMEWORK_NORMAL_ESTIMATOR_H_ */
