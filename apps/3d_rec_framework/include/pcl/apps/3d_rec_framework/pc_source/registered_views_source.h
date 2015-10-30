/*
 * ply_source.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_MESH_SOURCE_H_
#define REC_FRAMEWORK_MESH_SOURCE_H_

#include <pcl/apps/3d_rec_framework/pc_source/source.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Data source class based on partial views from sensor.
     * In this case, the training data is obtained directly from a depth sensor.
     * The filesystem should contain pcd files (representing a view of an object in
     * camera coordinates) and each view needs to be associated with a txt file
     * containing a 4x4 matrix representing the transformation from camera coordinates
     * to a global object coordinates frame.
     * \author Aitor Aldoma
     */

    template<typename PointInT>
      class RegisteredViewsSource : public Source<PointInT>
      {
        typedef Source<PointInT> SourceT;
        typedef Model<PointInT> ModelT;

        using SourceT::path_;
        using SourceT::models_;
        using SourceT::createTrainingDir;
        using SourceT::getModelsInDirectory;
        using SourceT::model_scale_;

        std::string view_prefix_;
        int pose_files_order_; //0 is row, 1 is column

      public:
        RegisteredViewsSource ()
        {
          view_prefix_ = std::string ("view");
          pose_files_order_ = 0;
        }

        void
        setPrefix (std::string & pre)
        {
          view_prefix_ = pre;
        }

        void
        setPoseRowOrder (int o)
        {
          pose_files_order_ = o;
        }

        void
        getViewsFilenames (bf::path & path_with_views, std::vector<std::string> & view_filenames)
        {
          int number_of_views = 0;
          bf::directory_iterator end_itr;
          for (bf::directory_iterator itr (path_with_views); itr != end_itr; ++itr)
          {
            if (!(bf::is_directory (*itr)))
            {
              std::vector < std::string > strs;
              std::vector < std::string > strs_;

#if BOOST_FILESYSTEM_VERSION == 3
              std::string file = (itr->path ().filename ()).string();
#else
              std::string file = (itr->path ()).filename ();
#endif

              boost::split (strs, file, boost::is_any_of ("."));
              boost::split (strs_, file, boost::is_any_of ("_"));

              std::string extension = strs[strs.size () - 1];

              if (extension == "pcd" && (strs_[0].compare (view_prefix_) == 0))
              {
#if BOOST_FILESYSTEM_VERSION == 3
                view_filenames.push_back ((itr->path ().filename ()).string());
#else
                view_filenames.push_back ((itr->path ()).filename ());
#endif

                number_of_views++;
              }
            }
          }
        }

        void
        assembleModelFromViewsAndPoses(ModelT & model, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & poses) {
          for(size_t i=0; i < model.views_->size(); i++) {
            Eigen::Matrix4f inv = poses[i];
            typename pcl::PointCloud<PointInT>::Ptr global_cloud(new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud(*(model.views_->at(i)),*global_cloud, inv);
            *(model.assembled_) += *global_cloud;
          }
        }

        void
        loadOrGenerate (std::string & dir, std::string & model_path, ModelT & model)
        {
          std::stringstream pathmodel;
          pathmodel << dir << "/" << model.class_ << "/" << model.id_;
          bf::path trained_dir = pathmodel.str ();

          model.views_.reset (new std::vector<typename pcl::PointCloud<PointInT>::Ptr>);
          model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
          model.self_occlusions_.reset (new std::vector<float>);

          if (bf::exists (trained_dir))
          {
            //load views and poses
            std::vector < std::string > view_filenames;
            int number_of_views = 0;
            bf::directory_iterator end_itr;
            for (bf::directory_iterator itr (trained_dir); itr != end_itr; ++itr)
            {
              //check if its a directory, then get models in it
              if (!(bf::is_directory (*itr)))
              {
                //check that it is a ply file and then add, otherwise ignore..
                std::vector < std::string > strs;
                std::vector < std::string > strs_;

#if BOOST_FILESYSTEM_VERSION == 3
                std::string file = (itr->path ().filename ()).string();
#else
                std::string file = (itr->path ()).filename ();
#endif

                boost::split (strs, file, boost::is_any_of ("."));
                boost::split (strs_, file, boost::is_any_of ("_"));

                std::string extension = strs[strs.size () - 1];

                if (extension == "pcd" && strs_[0] == "view")
                {
#if BOOST_FILESYSTEM_VERSION == 3
                  view_filenames.push_back ((itr->path ().filename ()).string());
#else
                  view_filenames.push_back ((itr->path ()).filename ());
#endif

                  number_of_views++;
                }
              }
            }

            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses_to_assemble_;

            for (size_t i = 0; i < view_filenames.size (); i++)
            {
              std::stringstream view_file;
              view_file << pathmodel.str () << "/" << view_filenames[i];
              typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
              pcl::io::loadPCDFile (view_file.str (), *cloud);

              model.views_->push_back (cloud);

              std::string file_replaced1 (view_filenames[i]);
              boost::replace_all (file_replaced1, "view", "pose");
              boost::replace_all (file_replaced1, ".pcd", ".txt");

              //read pose as well
              std::stringstream pose_file;
              pose_file << pathmodel.str () << "/" << file_replaced1;
              Eigen::Matrix4f pose;
              PersistenceUtils::readMatrixFromFile (pose_file.str (), pose);

              if(pose_files_order_ != 0) {
                //std::cout << "Transpose..." << std::endl;

                Eigen::Matrix4f pose_trans = pose.transpose();
                poses_to_assemble_.push_back(pose_trans);
                //pose = pose_trans;
                //std::cout << pose << std::endl;
              }

              //std::cout << "pose being push backed to model" << std::endl;
              std::cout << pose << std::endl;

              //the recognizer assumes transformation from M to CC
              Eigen::Matrix4f inv = pose.inverse();
              model.poses_->push_back (inv);

              model.self_occlusions_->push_back (-1.f);

            }

            model.assembled_.reset (new pcl::PointCloud<PointInT>);
            assembleModelFromViewsAndPoses(model, poses_to_assemble_);

            /*pcl::visualization::PCLVisualizer vis ("results");
            pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler (model.assembled_, 255, 0, 0);
            vis.addPointCloud<PointInT> (model.assembled_, random_handler, "points");

            Eigen::Matrix4f view_transformation = model.poses_->at(0).inverse();
            typename pcl::PointCloud<PointInT>::Ptr view_trans(new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud(*(model.views_->at(0)), *view_trans, view_transformation);

            pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler2 (view_trans, 0, 255, 0);
            vis.addPointCloud<PointInT> (view_trans, random_handler2, "view");

            vis.addCoordinateSystem(0.1);
            vis.spin ();*/

          }
          else
          {

            //we just need to copy the views to the training directory
            std::stringstream direc;
            direc << dir << "/" << model.class_ << "/" << model.id_;
            createClassAndModelDirectories (dir, model.class_, model.id_);

            std::vector < std::string > view_filenames;
            bf::path model_dir = model_path;

            getViewsFilenames (model_dir, view_filenames);
            std::cout << view_filenames.size () << std::endl;

            for (size_t i = 0; i < view_filenames.size (); i++)
            {
              std::stringstream view_file;
              view_file << model_path << "/" << view_filenames[i];
              typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
              pcl::io::loadPCDFile (view_file.str (), *cloud);

              std::cout << view_file.str () << std::endl;

              std::stringstream path_view;
              path_view << direc.str () << "/view_" << i << ".pcd";
              pcl::io::savePCDFileBinary (path_view.str (), *cloud);

              std::string file_replaced1 (view_file.str ());
              boost::replace_all (file_replaced1, view_prefix_, "pose");
              boost::replace_all (file_replaced1, ".pcd", ".txt");

              Eigen::Matrix4f pose;
              PersistenceUtils::readMatrixFromFile (file_replaced1, pose);

              std::cout << pose << std::endl;

              if(pose_files_order_ == 0) {
                std::cout << "Transpose..." << std::endl;
                Eigen::Matrix4f pose_trans = pose.transpose();
                pose = pose_trans;
                std::cout << pose << std::endl;
              }

              std::stringstream path_pose;
              path_pose << direc.str () << "/pose_" << i << ".txt";
              pcl::rec_3d_framework::PersistenceUtils::writeMatrixToFile (path_pose.str (), pose);
            }

            loadOrGenerate (dir, model_path, model);

          }
        }

        bool
        isleafDirectory (bf::path & path)
        {
          bf::directory_iterator end_itr;
          bool no_dirs_inside = true;
          for (bf::directory_iterator itr (path); itr != end_itr; ++itr)
          {
            if (bf::is_directory (*itr))
            {
              no_dirs_inside = false;
            }
          }

          return no_dirs_inside;
        }

        void
        getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
        {
          bf::directory_iterator end_itr;
          for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
          {
            //check if its a directory, then get models in it
            if (bf::is_directory (*itr))
            {
#if BOOST_FILESYSTEM_VERSION == 3
              std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
#else
              std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

              bf::path curr_path = itr->path ();

              if (isleafDirectory (curr_path))
              {
#if BOOST_FILESYSTEM_VERSION == 3
                std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
                std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
                relative_paths.push_back (path);

              }
              else
              {
                getModelsInDirectory (curr_path, so_far, relative_paths);
              }
            }
          }
        }

        /**
         * \brief Creates the model representation of the training set, generating views if needed
         */
        void
        generate (std::string & training_dir)
        {

          //create training dir fs if not existent
          createTrainingDir (training_dir);

          //get models in directory
          std::vector < std::string > files;
          std::string start = "";
          bf::path dir = path_;
          getModelsInDirectory (dir, start, files);

          models_.reset (new std::vector<ModelT>);

          for (size_t i = 0; i < files.size (); i++)
          {
            ModelT m;

            std::vector < std::string > strs;
            boost::split (strs, files[i], boost::is_any_of ("/\\"));
            std::string name = strs[strs.size () - 1];

            if (strs.size () == 1)
            {
              m.id_ = strs[0];
            }
            else
            {
              std::stringstream ss;
              for (int i = 0; i < (static_cast<int> (strs.size ()) - 1); i++)
              {
                ss << strs[i];
                if (i != (static_cast<int> (strs.size ()) - 1))
                  ss << "/";
              }

              m.class_ = ss.str ();
              m.id_ = strs[strs.size () - 1];
            }

            std::cout << m.class_ << " . " << m.id_ << std::endl;
            //check which of them have been trained using training_dir and the model_id_
            //load views, poses and self-occlusions for those that exist
            //generate otherwise

            std::stringstream model_path;
            model_path << path_ << "/" << files[i];
            std::string path_model = model_path.str ();
            loadOrGenerate (training_dir, path_model, m);

            models_->push_back (m);

            //std::cout << files[i] << std::endl;
          }
        }
      };
  }
}

#endif /* REC_FRAMEWORK_MESH_SOURCE_H_ */
