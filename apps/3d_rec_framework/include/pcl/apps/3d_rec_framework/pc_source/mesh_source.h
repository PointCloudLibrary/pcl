/*
 * ply_source.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_MESH_SOURCE_H_
#define REC_FRAMEWORK_MESH_SOURCE_H_

#include <pcl/apps/3d_rec_framework/pc_source/source.h>
#include <pcl/apps/render_views_tesselated_sphere.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/apps/3d_rec_framework/utils/vtk_model_sampling.h>
#include <boost/function.hpp>
#include <vtkTransformPolyDataFilter.h>

namespace pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Data source class based on mesh models
     * \author Aitor Aldoma
     */

    template<typename PointInT>
      class MeshSource : public Source<PointInT>
      {
        typedef Source<PointInT> SourceT;
        typedef Model<PointInT> ModelT;

        using SourceT::path_;
        using SourceT::models_;
        using SourceT::createTrainingDir;
        using SourceT::getModelsInDirectory;
        using SourceT::model_scale_;

        int tes_level_;
        int resolution_;
        float radius_sphere_;
        float view_angle_;
        bool gen_organized_;
        boost::function<bool
        (const Eigen::Vector3f &)> campos_constraints_func_;

      public:

        using SourceT::setFilterDuplicateViews;

        MeshSource () :
        SourceT ()
        {
          gen_organized_ = false;
        }

        void
        setTesselationLevel (int lev)
        {
          tes_level_ = lev;
        }

        void
        setCamPosConstraints (boost::function<bool
        (const Eigen::Vector3f &)> & bb)
        {
          campos_constraints_func_ = bb;
        }

        void
        setResolution (int res)
        {
          resolution_ = res;
        }

        void
        setRadiusSphere (float r)
        {
          radius_sphere_ = r;
        }

        void
        setViewAngle (float a)
        {
          view_angle_ = a;
        }

        void
        loadOrGenerate (std::string & dir, std::string & model_path, ModelT & model)
        {
          std::stringstream pathmodel;
          pathmodel << dir << "/" << model.class_ << "/" << model.id_;
          bf::path trained_dir = pathmodel.str ();

          model.views_.reset (new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
          model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
          model.self_occlusions_.reset (new std::vector<float>);
          model.assembled_.reset (new pcl::PointCloud<pcl::PointXYZ>);
          uniform_sampling (model_path, 100000, *model.assembled_, model_scale_);

          if (bf::exists (trained_dir))
          {
            //load views, poses and self-occlusions
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

              std::string file_replaced2 (view_filenames[i]);
              boost::replace_all (file_replaced2, "view", "entropy");
              boost::replace_all (file_replaced2, ".pcd", ".txt");

              //read pose as well
              std::stringstream pose_file;
              pose_file << pathmodel.str () << "/" << file_replaced1;

              Eigen::Matrix4f pose;
              PersistenceUtils::readMatrixFromFile (pose_file.str (), pose);

              model.poses_->push_back (pose);

              //read entropy as well
              std::stringstream entropy_file;
              entropy_file << pathmodel.str () << "/" << file_replaced2;
              float entropy = 0;
              PersistenceUtils::readFloatFromFile (entropy_file.str (), entropy);
              model.self_occlusions_->push_back (entropy);

            }

          }
          else
          {
            //load PLY model and scale it
            vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New ();
            reader->SetFileName (model_path.c_str ());

            vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New ();
            trans->Scale (model_scale_, model_scale_, model_scale_);
            trans->Modified ();
            trans->Update ();

            vtkSmartPointer<vtkTransformPolyDataFilter> filter_scale = vtkSmartPointer<vtkTransformPolyDataFilter>::New ();
            filter_scale->SetTransform (trans);
            filter_scale->SetInputConnection (reader->GetOutputPort ());
            filter_scale->Update ();

            vtkSmartPointer<vtkPolyData> mapper = filter_scale->GetOutput ();
            mapper->Update ();

            //generate views
            pcl::apps::RenderViewsTesselatedSphere render_views;
            render_views.setResolution (resolution_);
            render_views.setUseVertices (false);
            render_views.setRadiusSphere (radius_sphere_);
            render_views.setComputeEntropies (true);
            render_views.setTesselationLevel (tes_level_);
            render_views.setViewAngle (view_angle_);
            render_views.addModelFromPolyData (mapper);
            render_views.setGenOrganized (gen_organized_);
            render_views.setCamPosConstraints (campos_constraints_func_);
            render_views.generateViews ();

            std::vector<typename PointCloud<PointInT>::Ptr> views_xyz_orig;
            std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
            std::vector<float> entropies;

            render_views.getViews (views_xyz_orig);
            render_views.getPoses (poses);
            render_views.getEntropies (entropies);

            model.views_.reset (new std::vector<typename PointCloud<PointInT>::Ptr> ());
            model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > ());
            model.self_occlusions_.reset (new std::vector<float> ());

            for (size_t i = 0; i < views_xyz_orig.size (); i++)
            {
              model.views_->push_back (views_xyz_orig[i]);
              model.poses_->push_back (poses[i]);
              model.self_occlusions_->push_back (entropies[i]);
            }

            std::stringstream direc;
            direc << dir << "/" << model.class_ << "/" << model.id_;
            this->createClassAndModelDirectories (dir, model.class_, model.id_);

            for (size_t i = 0; i < model.views_->size (); i++)
            {
              //save generated model for future use
              std::stringstream path_view;
              path_view << direc.str () << "/view_" << i << ".pcd";
              pcl::io::savePCDFileBinary (path_view.str (), *(model.views_->at (i)));

              std::stringstream path_pose;
              path_pose << direc.str () << "/pose_" << i << ".txt";

              pcl::rec_3d_framework::PersistenceUtils::writeMatrixToFile (path_pose.str (), model.poses_->at (i));

              std::stringstream path_entropy;
              path_entropy << direc.str () << "/entropy_" << i << ".txt";
              pcl::rec_3d_framework::PersistenceUtils::writeFloatToFile (path_entropy.str (), model.self_occlusions_->at (i));
            }

            loadOrGenerate (dir, model_path, model);

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
          std::string ext = std::string ("ply");
          bf::path dir = path_;
          getModelsInDirectory (dir, start, files, ext);

          models_.reset (new std::vector<ModelT>);

          for (size_t i = 0; i < files.size (); i++)
          {
            ModelT m;
            this->getIdAndClassFromFilename (files[i], m.id_, m.class_);

            //check which of them have been trained using training_dir and the model_id_
            //load views, poses and self-occlusions for those that exist
            //generate otherwise
            std::cout << files[i] << std::endl;
            std::stringstream model_path;
            model_path << path_ << "/" << files[i];
            std::string path_model = model_path.str ();
            loadOrGenerate (training_dir, path_model, m);

            models_->push_back (m);
          }
        }
      };
  }
}

#endif /* REC_FRAMEWORK_MESH_SOURCE_H_ */
