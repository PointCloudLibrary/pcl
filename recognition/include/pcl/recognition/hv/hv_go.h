/*
 * go.h
 *
 *  Created on: Jun 4, 2012
 *      Author: aitor
 */

#pragma once

#include <random>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

//includes required by mets.hh
#include <boost/random/linear_congruential.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <pcl/pcl_macros.h>
#include <pcl/recognition/hv/hypotheses_verification.h>
#include <pcl/recognition/3rdparty/metslib/mets.hh>
#include <pcl/features/normal_3d.h>

#include <memory>

namespace pcl
{

  /** \brief A hypothesis verification method proposed in
    * "A Global Hypotheses Verification Method for 3D Object Recognition", A. Aldoma and F. Tombari and L. Di Stefano and Markus Vincze, ECCV 2012
    * \author Aitor Aldoma
    * \ingroup recognition
    */
  template<typename ModelT, typename SceneT>
  class PCL_EXPORTS GlobalHypothesesVerification: public HypothesisVerification<ModelT, SceneT>
  {
    private:

      //Helper classes
      struct RecognitionModel
      {
        public:
          std::vector<int> explained_; //indices vector referencing explained_by_RM_
          std::vector<float> explained_distances_; //closest distances to the scene for point i
          std::vector<int> unexplained_in_neighborhood; //indices vector referencing unexplained_by_RM_neighboorhods
          std::vector<float> unexplained_in_neighborhood_weights; //weights for the points not being explained in the neighborhood of a hypothesis
          std::vector<int> outlier_indices_; //outlier indices of this model
          std::vector<int> complete_cloud_occupancy_indices_;
          typename pcl::PointCloud<ModelT>::Ptr cloud_;
          typename pcl::PointCloud<ModelT>::Ptr complete_cloud_;
          int bad_information_;
          float outliers_weight_;
          pcl::PointCloud<pcl::Normal>::Ptr normals_;
          int id_;
      };

      using RecognitionModelPtr = std::shared_ptr<RecognitionModel>;

      using SAOptimizerT = GlobalHypothesesVerification<ModelT, SceneT>;
      class SAModel: public mets::evaluable_solution
      {
        public:
          std::vector<bool> solution_;
          SAOptimizerT * opt_;
          mets::gol_type cost_;

          //Evaluates the current solution
          mets::gol_type cost_function() const override
          {
            return cost_;
          }

          void copy_from(const mets::copyable& o) override
          {
            const auto& s = dynamic_cast<const SAModel&> (o);
            solution_ = s.solution_;
            opt_ = s.opt_;
            cost_ = s.cost_;
          }

          mets::gol_type what_if(int /*index*/, bool /*val*/) const
          {
            /*std::vector<bool> tmp (solution_);
            tmp[index] = val;
            mets::gol_type sol = opt_->evaluateSolution (solution_, index); //evaluate without updating status
            return sol;*/
            return static_cast<mets::gol_type>(0);
          }

          mets::gol_type apply_and_evaluate(int index, bool val)
          {
            solution_[index] = val;
            mets::gol_type sol = opt_->evaluateSolution (solution_, index); //this will update the state of the solution
            cost_ = sol;
            return sol;
          }

          void apply(int /*index*/, bool /*val*/)
          {

          }

          void unapply(int index, bool val)
          {
            solution_[index] = val;
            //update optimizer solution
            cost_ = opt_->evaluateSolution (solution_, index); //this will update the cost function in opt_
          }
          void setSolution(std::vector<bool> & sol)
          {
            solution_ = sol;
          }

          void setOptimizer(SAOptimizerT * opt)
          {
            opt_ = opt;
          }
      };

      /*
       * Represents a move, deactivate a hypothesis
       */

      class move: public mets::move
      {
          int index_;
        public:
          move(int i) :
              index_ (i)
          {
          }

          mets::gol_type evaluate(const mets::feasible_solution& /*cs*/) const override
          {
            return static_cast<mets::gol_type>(0);
          }

          mets::gol_type apply_and_evaluate(mets::feasible_solution& cs)
          {
            auto& model = dynamic_cast<SAModel&> (cs);
            return model.apply_and_evaluate (index_, !model.solution_[index_]);
          }

          void apply(mets::feasible_solution& /*s*/) const override
          {
          }

          void unapply(mets::feasible_solution& s) const
          {
            auto& model = dynamic_cast<SAModel&> (s);
            model.unapply (index_, !model.solution_[index_]);
          }
      };

      class move_manager
      {
        public:
          std::vector<move*> moves_m;
          using iterator = typename std::vector<move *>::iterator;
          iterator begin()
          {
            return moves_m.begin ();
          }
          iterator end()
          {
            return moves_m.end ();
          }

          move_manager(int problem_size)
          {
            for (int ii = 0; ii != problem_size; ++ii)
              moves_m.push_back (new move (ii));
          }

          ~move_manager()
          {
            // delete all moves
            for (auto ii = begin (); ii != end (); ++ii)
              delete (*ii);
          }

          void refresh(mets::feasible_solution& /*s*/)
          {
            std::shuffle (moves_m.begin (), moves_m.end (), std::mt19937(std::random_device()()));
          }

      };

      //inherited class attributes
      using HypothesisVerification<ModelT, SceneT>::mask_;
      using HypothesisVerification<ModelT, SceneT>::scene_cloud_downsampled_;
      using HypothesisVerification<ModelT, SceneT>::scene_downsampled_tree_;
      using HypothesisVerification<ModelT, SceneT>::visible_models_;
      using HypothesisVerification<ModelT, SceneT>::complete_models_;
      using HypothesisVerification<ModelT, SceneT>::resolution_;
      using HypothesisVerification<ModelT, SceneT>::inliers_threshold_;

      //class attributes
      using NormalEstimator_ = pcl::NormalEstimation<SceneT, pcl::Normal>;
      pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
      pcl::PointCloud<pcl::PointXYZI>::Ptr clusters_cloud_;

      std::vector<int> complete_cloud_occupancy_by_RM_;
      float res_occupancy_grid_;
      float w_occupied_multiple_cm_;

      std::vector<int> explained_by_RM_; //represents the points of scene_cloud_ that are explained by the recognition models
      std::vector<float> explained_by_RM_distance_weighted; //represents the points of scene_cloud_ that are explained by the recognition models
      std::vector<float> unexplained_by_RM_neighboorhods; //represents the points of scene_cloud_ that are not explained by the active hypotheses in the neighboorhod of the recognition models
      std::vector<RecognitionModelPtr> recognition_models_;
      std::vector<std::size_t> indices_;

      float regularizer_;
      float clutter_regularizer_;
      bool detect_clutter_;
      float radius_neighborhood_GO_;
      float radius_normals_;

      float previous_explained_value;
      int previous_duplicity_;
      int previous_duplicity_complete_models_;
      float previous_bad_info_;
      float previous_unexplained_;

      int max_iterations_; //max iterations without improvement
      SAModel best_seen_;
      float initial_temp_;

      int n_cc_;
      std::vector<std::vector<int> > cc_;

      void setPreviousBadInfo(float f)
      {
        previous_bad_info_ = f;
      }

      float getPreviousBadInfo()
      {
        return previous_bad_info_;
      }

      void setPreviousExplainedValue(float v)
      {
        previous_explained_value = v;
      }

      void setPreviousDuplicity(int v)
      {
        previous_duplicity_ = v;
      }

      void setPreviousDuplicityCM(int v)
      {
        previous_duplicity_complete_models_ = v;
      }

      void setPreviousUnexplainedValue(float v)
      {
        previous_unexplained_ = v;
      }

      float getPreviousUnexplainedValue()
      {
        return previous_unexplained_;
      }

      float getExplainedValue()
      {
        return previous_explained_value;
      }

      int getDuplicity()
      {
        return previous_duplicity_;
      }

      int getDuplicityCM()
      {
        return previous_duplicity_complete_models_;
      }

      void updateUnexplainedVector(std::vector<int> & unexplained_, std::vector<float> & unexplained_distances, std::vector<float> & unexplained_by_RM,
          std::vector<int> & explained, std::vector<int> & explained_by_RM, float val)
      {
        {

          float add_to_unexplained = 0.f;

          for (std::size_t i = 0; i < unexplained_.size (); i++)
          {

            bool prev_unexplained = (unexplained_by_RM[unexplained_[i]] > 0) && (explained_by_RM[unexplained_[i]] == 0);
            unexplained_by_RM[unexplained_[i]] += val * unexplained_distances[i];

            if (val < 0) //the hypothesis is being removed
            {
              if (prev_unexplained)
              {
                //decrease by 1
                add_to_unexplained -= unexplained_distances[i];
              }
            } else //the hypothesis is being added and unexplains unexplained_[i], so increase by 1 unless its explained by another hypothesis
            {
              if (explained_by_RM[unexplained_[i]] == 0)
                add_to_unexplained += unexplained_distances[i];
            }
          }

          for (const int &i : explained)
          {
            if (val < 0)
            {
              //the hypothesis is being removed, check that there are no points that become unexplained and have clutter unexplained hypotheses
              if ((explained_by_RM[i] == 0) && (unexplained_by_RM[i] > 0))
              {
                add_to_unexplained += unexplained_by_RM[i]; //the points become unexplained
              }
            } else
            {
              //std::cout << "being added..." << add_to_unexplained << " " << unexplained_by_RM[explained[i]] << std::endl;
              if ((explained_by_RM[i] == 1) && (unexplained_by_RM[i] > 0))
              { //the only hypothesis explaining that point
                add_to_unexplained -= unexplained_by_RM[i]; //the points are not unexplained any longer because this hypothesis explains them
              }
            }
          }

          //std::cout << add_to_unexplained << std::endl;
          previous_unexplained_ += add_to_unexplained;
        }
      }

      void updateExplainedVector(std::vector<int> & vec, std::vector<float> & vec_float, std::vector<int> & explained_,
          std::vector<float> & explained_by_RM_distance_weighted, float sign)
      {
        float add_to_explained = 0.f;
        int add_to_duplicity_ = 0;

        for (std::size_t i = 0; i < vec.size (); i++)
        {
          bool prev_dup = explained_[vec[i]] > 1;

          explained_[vec[i]] += static_cast<int> (sign);
          explained_by_RM_distance_weighted[vec[i]] += vec_float[i] * sign;

          add_to_explained += vec_float[i] * sign;

          if ((explained_[vec[i]] > 1) && prev_dup)
          { //its still a duplicate, we are adding
            add_to_duplicity_ += static_cast<int> (sign); //so, just add or remove one
          } else if ((explained_[vec[i]] == 1) && prev_dup)
          { //if was duplicate before, now its not, remove 2, we are removing the hypothesis
            add_to_duplicity_ -= 2;
          } else if ((explained_[vec[i]] > 1) && !prev_dup)
          { //it was not a duplicate but it is now, add 2, we are adding a conflicting hypothesis for the point
            add_to_duplicity_ += 2;
          }
        }

        //update explained and duplicity values...
        previous_explained_value += add_to_explained;
        previous_duplicity_ += add_to_duplicity_;
      }

      void updateCMDuplicity(std::vector<int> & vec, std::vector<int> & occupancy_vec, float sign) {
        int add_to_duplicity_ = 0;
        for (const int &i : vec)
        {
          bool prev_dup = occupancy_vec[i] > 1;
          occupancy_vec[i] += static_cast<int> (sign);
          if ((occupancy_vec[i] > 1) && prev_dup)
          { //its still a duplicate, we are adding
            add_to_duplicity_ += static_cast<int> (sign); //so, just add or remove one
          } else if ((occupancy_vec[i] == 1) && prev_dup)
          { //if was duplicate before, now its not, remove 2, we are removing the hypothesis
            add_to_duplicity_ -= 2;
          } else if ((occupancy_vec[i] > 1) && !prev_dup)
          { //it was not a duplicate but it is now, add 2, we are adding a conflicting hypothesis for the point
            add_to_duplicity_ += 2;
          }
        }

        previous_duplicity_complete_models_ += add_to_duplicity_;
      }

      float getTotalExplainedInformation(std::vector<int> & explained_, std::vector<float> & explained_by_RM_distance_weighted, int * duplicity_)
      {
        float explained_info = 0;
        int duplicity = 0;

        for (std::size_t i = 0; i < explained_.size (); i++)
        {
          if (explained_[i] > 0)
            explained_info += explained_by_RM_distance_weighted[i];

          if (explained_[i] > 1)
            duplicity += explained_[i];
        }

        *duplicity_ = duplicity;

        return explained_info;
      }

      float getTotalBadInformation(std::vector<RecognitionModelPtr> & recog_models)
      {
        float bad_info = 0;
        for (std::size_t i = 0; i < recog_models.size (); i++)
          bad_info += recog_models[i]->outliers_weight_ * static_cast<float> (recog_models[i]->bad_information_);

        return bad_info;
      }

      float getUnexplainedInformationInNeighborhood(std::vector<float> & unexplained, std::vector<int> & explained)
      {
        float unexplained_sum = 0.f;
        for (std::size_t i = 0; i < unexplained.size (); i++)
        {
          if (unexplained[i] > 0 && explained[i] == 0)
            unexplained_sum += unexplained[i];
        }

        return unexplained_sum;
      }

      //Performs smooth segmentation of the scene cloud and compute the model cues
      void
      initialize();

      mets::gol_type
      evaluateSolution(const std::vector<bool> & active, int changed);

      bool
      addModel(typename pcl::PointCloud<ModelT>::ConstPtr & model, typename pcl::PointCloud<ModelT>::ConstPtr & complete_model, RecognitionModelPtr & recog_model);

      void
      computeClutterCue(RecognitionModelPtr & recog_model);

      void
      SAOptimize(std::vector<int> & cc_indices, std::vector<bool> & sub_solution);

    public:
      GlobalHypothesesVerification() : HypothesisVerification<ModelT, SceneT>()
      {
        resolution_ = 0.005f;
        max_iterations_ = 5000;
        regularizer_ = 1.f;
        radius_normals_ = 0.01f;
        initial_temp_ = 1000;
        detect_clutter_ = true;
        radius_neighborhood_GO_ = 0.03f;
        clutter_regularizer_ = 5.f;
        res_occupancy_grid_ = 0.01f;
        w_occupied_multiple_cm_ = 4.f;
      }

      void
      verify() override;
      
      void setResolutionOccupancyGrid(float r)
      {
        res_occupancy_grid_ = r;
      }

      void setRadiusNormals(float r)
      {
        radius_normals_ = r;
      }

      void setMaxIterations(int i)
      {
        max_iterations_ = i;
      }

      void setInitialTemp(float t)
      {
        initial_temp_ = t;
      }

      void setRegularizer(float r)
      {
        regularizer_ = r;
      }

      void setRadiusClutter(float r)
      {
        radius_neighborhood_GO_ = r;
      }

      void setClutterRegularizer(float cr)
      {
        clutter_regularizer_ = cr;
      }

      void setDetectClutter(bool d)
      {
        detect_clutter_ = d;
      }
  };
}

#ifdef PCL_NO_PRECOMPILE
#include <pcl/recognition/impl/hv/hv_go.hpp>
#endif
