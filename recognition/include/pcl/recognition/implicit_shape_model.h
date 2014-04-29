/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef	PCL_IMPLICIT_SHAPE_MODEL_H_
#define	PCL_IMPLICIT_SHAPE_MODEL_H_

#include <vector>
#include <fstream>
#include <limits>
#include <Eigen/src/Core/Matrix.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_representation.h>
#include <pcl/features/feature.h>
#include <pcl/features/spin_image.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

namespace pcl
{
  /** \brief This struct is used for storing peak. */
  struct ISMPeak
  {
    /** \brief Point were this peak is located. */
    PCL_ADD_POINT4D;

    /** \brief Density of this peak. */
    double density;

    /** \brief Determines which class this peak belongs. */
    int class_id;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;

  namespace features
  {
    /** \brief This class is used for storing, analyzing and manipulating votes
      * obtained from ISM algorithm. */
    template <typename PointT>
    class PCL_EXPORTS ISMVoteList
    {
      public:

        /** \brief Empty constructor with member variables initialization. */
        ISMVoteList ();

        /** \brief virtual descriptor. */
        virtual
        ~ISMVoteList ();

        /** \brief This method simply adds another vote to the list.
          * \param[in] in_vote vote to add
          * \param[in] vote_origin origin of the added vote
          * \param[in] in_class class for which this vote is cast
          */
        void
        addVote (pcl::InterestPoint& in_vote, const PointT &vote_origin, int in_class);

        /** \brief Returns the colored cloud that consists of votes for center (blue points) and
          * initial point cloud (if it was passed).
          * \param[in] cloud cloud that needs to be merged with votes for visualizing. */
        typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr
        getColoredCloud (typename pcl::PointCloud<PointT>::Ptr cloud = 0);

        /** \brief This method finds the strongest peaks (points were density has most higher values).
          * It is based on the non maxima supression principles.
          * \param[out] out_peaks it will contain the strongest peaks
          * \param[in] in_class_id class of interest for which peaks are evaluated
          * \param[in] in_non_maxima_radius non maxima supression radius. The shapes radius is recommended for this value.
          * \param in_sigma
          */
        void
        findStrongestPeaks (std::vector<ISMPeak, Eigen::aligned_allocator<ISMPeak> > &out_peaks, int in_class_id, double in_non_maxima_radius, double in_sigma);

        /** \brief Returns the density at the specified point.
          * \param[in] point point of interest
          * \param[in] sigma_dist
          */
        double
        getDensityAtPoint (const PointT &point, double sigma_dist);

        /** \brief This method simply returns the number of votes. */
        unsigned int
        getNumberOfVotes ();

      protected:

        /** \brief this method is simply setting up the search tree. */
        void
        validateTree ();

        Eigen::Vector3f
        shiftMean (const Eigen::Vector3f& snapPt, const double in_dSigmaDist);

      protected:

        /** \brief Stores all votes. */
        pcl::PointCloud<pcl::InterestPoint>::Ptr votes_;

        /** \brief Signalizes if the tree is valid. */
        bool tree_is_valid_;

        /** \brief Stores the origins of the votes. */
        typename pcl::PointCloud<PointT>::Ptr votes_origins_;

        /** \brief Stores classes for which every single vote was cast. */
        std::vector<int> votes_class_;

        /** \brief Stores the search tree. */
        pcl::KdTreeFLANN<pcl::InterestPoint>::Ptr tree_;

        /** \brief Stores neighbours indices. */
        std::vector<int> k_ind_;

        /** \brief Stores square distances to the corresponding neighbours. */
        std::vector<float> k_sqr_dist_;
    };
 
    /** \brief The assignment of this structure is to store the statistical/learned weights and other information
      * of the trained Implict Shape Model algorithm.
      */
    struct PCL_EXPORTS ISMModel
    {
      /** \brief Simple constructor that initializes the structure. */
      ISMModel ();

      /** \brief Copy constructor for deep copy. */
      ISMModel (ISMModel const & copy);

      /** Destructor that frees memory. */
      virtual
      ~ISMModel ();

      /** \brief This method simply saves the trained model for later usage.
        * \param[in] file_name path to file for saving model
        */
      bool
      saveModelToFile (std::string& file_name);

      /** \brief This method loads the trained model from file.
        * \param[in] file_name path to file which stores trained model
        */
      bool
      loadModelFromfile (std::string& file_name);

      /** \brief this method resets all variables and frees memory. */
      void
      reset ();

      /** Operator overloading for deep copy. */
      ISMModel & operator = (const ISMModel& other);

      /** \brief Stores statistical weights. */
      std::vector<std::vector<float> > statistical_weights_;

      /** \brief Stores learned weights. */
      std::vector<float> learned_weights_;

      /** \brief Stores the class label for every direction. */
      std::vector<unsigned int> classes_;

      /** \brief Stores the sigma value for each class. This values were used to compute the learned weights. */
      std::vector<float> sigmas_;

      /** \brief Stores the directions to objects center for each visual word. */
      Eigen::MatrixXf directions_to_center_;

      /** \brief Stores the centers of the clusters that were obtained during the visual words clusterization. */
      Eigen::MatrixXf clusters_centers_;

      /** \brief This is an array of clusters. Each cluster stores the indices of the visual words that it contains. */
      std::vector<std::vector<unsigned int> > clusters_;

      /** \brief Stores the number of classes. */
      unsigned int number_of_classes_;

      /** \brief Stores the number of visual words. */
      unsigned int number_of_visual_words_;

      /** \brief Stores the number of clusters. */
      unsigned int number_of_clusters_;

      /** \brief Stores descriptors dimension. */
      unsigned int descriptors_dimension_;

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
  }

  namespace ism
  {
    /** \brief This class implements Implicit Shape Model algorithm described in
      * "Hough Transforms and 3D SURF for robust three dimensional classication"
      * by Jan Knopp1, Mukta Prasad, Geert Willems1, Radu Timofte, and Luc Van Gool.
      * It has two main member functions. One for training, using the data for which we know
      * which class it belongs to. And second for investigating a cloud for the presence
      * of the class of interest.
      * Implementation of the ISM algorithm described in "Hough Transforms and 3D SURF for robust three dimensional classication"
      * by Jan Knopp, Mukta Prasad, Geert Willems, Radu Timofte, and Luc Van Gool
      *
      * Authors: Roman Shapovalov, Alexander Velizhev, Sergey Ushakov
      */
    template <int FeatureSize, typename PointT, typename NormalT = pcl::Normal>
    class PCL_EXPORTS ImplicitShapeModelEstimation
    {
      public:

        typedef boost::shared_ptr<pcl::features::ISMModel> ISMModelPtr;

      protected:

        /** \brief This structure stores the information about the keypoint. */
        struct PCL_EXPORTS LocationInfo
        {
          /** \brief Location info constructor.
            * \param[in] model_num number of training model.
            * \param[in] dir_to_center expected direction to center
            * \param[in] origin initial point
            * \param[in] normal normal of the initial point
            */
          LocationInfo (unsigned int model_num, const PointT& dir_to_center, const PointT& origin, const NormalT& normal) :
            model_num_ (model_num),
            dir_to_center_ (dir_to_center),
            point_ (origin),
            normal_ (normal) {};

          /** \brief Tells from which training model this keypoint was extracted. */
          unsigned int model_num_;

          /** \brief Expected direction to center for this keypoint. */
          PointT dir_to_center_;

          /** \brief Stores the initial point. */
          PointT point_;

          /** \brief Stores the normal of the initial point. */
          NormalT normal_;
        };

        /** \brief This structure is used for determining the end of the
          * k-means clustering process. */
        typedef struct PCL_EXPORTS TC
        {
          enum
          {
            COUNT = 1,
            EPS = 2
          };

          /** \brief Termination criteria constructor.
            * \param[in] type defines the condition of termination(max iter., desired accuracy)
            * \param[in] max_count defines the max number of iterations
            * \param[in] epsilon defines the desired accuracy
            */
          TC(int type, int max_count, float epsilon) :
            type_ (type),
            max_count_ (max_count),
            epsilon_ (epsilon) {};

          /** \brief Flag that determines when the k-means clustering must be stopped.
            * If type_ equals COUNT then it must be stopped when the max number of iterations will be
            * reached. If type_ eaquals EPS then it must be stopped when the desired accuracy will be reached.
            * These flags can be used together, in that case the clustering will be finished when one of these
            * conditions will be reached.
            */
          int type_;

          /** \brief Defines maximum number of iterations for k-means clustering. */
          int max_count_;

          /** \brief Defines the accuracy for k-means clustering. */
          float epsilon_;
        } TermCriteria;

        /** \brief Structure for storing the visual word. */
        struct PCL_EXPORTS VisualWordStat
        {
          /** \brief Empty constructor with member variables initialization. */
          VisualWordStat () :
            class_ (-1),
            learned_weight_ (0.0f),
            dir_to_center_ (0.0f, 0.0f, 0.0f) {};

          /** \brief Which class this vote belongs. */
          int class_;

          /** \brief Weight of the vote. */
          float learned_weight_;

          /** \brief Expected direction to center. */
          pcl::PointXYZ dir_to_center_;
        };

      public:

        /** \brief Simple constructor that initializes everything. */
        ImplicitShapeModelEstimation ();

        /** \brief Simple destructor. */
        virtual
        ~ImplicitShapeModelEstimation ();

        /** \brief This method simply returns the clouds that were set as the training clouds. */
        std::vector<typename pcl::PointCloud<PointT>::Ptr>
		getTrainingClouds ();

        /** \brief Allows to set clouds for training the ISM model.
          * \param[in] training_clouds array of point clouds for training
          */
        void
		setTrainingClouds (const std::vector< typename pcl::PointCloud<PointT>::Ptr >& training_clouds);

        /** \brief Returns the array of classes that indicates which class the corresponding training cloud belongs. */
        std::vector<unsigned int>
        getTrainingClasses ();

        /** \brief Allows to set the class labels for the corresponding training clouds.
          * \param[in] training_classes array of class labels
          */
        void
        setTrainingClasses (const std::vector<unsigned int>& training_classes);

        /** \brief This method returns the coresponding cloud of normals for every training point cloud. */
        std::vector<typename pcl::PointCloud<NormalT>::Ptr>
        getTrainingNormals ();

        /** \brief Allows to set normals for the training clouds that were passed through setTrainingClouds method.
          * \param[in] training_normals array of clouds, each cloud is the cloud of normals
          */
        void
        setTrainingNormals (const std::vector< typename pcl::PointCloud<NormalT>::Ptr >& training_normals);

        /** \brief Returns the sampling size used for cloud simplification. */
        float
        getSamplingSize ();

        /** \brief Changes the sampling size used for cloud simplification.
          * \param[in] sampling_size desired size of grid bin
          */
        void
        setSamplingSize (float sampling_size);

        /** \brief Returns the current feature estimator used for extraction of the descriptors. */
        boost::shared_ptr<pcl::Feature<PointT, pcl::Histogram<FeatureSize> > >
        getFeatureEstimator ();

        /** \brief Changes the feature estimator.
          * \param[in] feature feature estimator that will be used to extract the descriptors.
          * Note that it must be fully initialized and configured.
          */
        void
        setFeatureEstimator (boost::shared_ptr<pcl::Feature<PointT, pcl::Histogram<FeatureSize> > > feature);

        /** \brief Returns the number of clusters used for descriptor clustering. */
        unsigned int
        getNumberOfClusters ();

        /** \brief Changes the number of clusters.
          * \param num_of_clusters desired number of clusters
          */
        void
        setNumberOfClusters (unsigned int num_of_clusters);

        /** \brief Returns the array of sigma values. */
        std::vector<float>
        getSigmaDists ();

        /** \brief This method allows to set the value of sigma used for calculating the learned weights for every single class.
          * \param[in] training_sigmas new sigmas for every class. If you want these values to be computed automatically,
          * just pass the empty array. The automatic regime calculates the maximum distance between the objects points and takes 10% of
          * this value as recomended in the article. If there are several objects of the same class,
          * then it computes the average maximum distance and takes 10%. Note that each class has its own sigma value.
          */
        void
        setSigmaDists (const std::vector<float>& training_sigmas);

        /** \brief Returns the state of Nvot coeff from [Knopp et al., 2010, (4)],
          * if set to false then coeff is taken as 1.0. It is just a kind of heuristic.
          * The default behavior is as in the article. So you can ignore this if you want.
          */
        bool
        getNVotState ();

        /** \brief Changes the state of the Nvot coeff from [Knopp et al., 2010, (4)].
          * \param[in] state desired state, if false then Nvot is taken as 1.0
          */
        void
        setNVotState (bool state);

        /** \brief This method performs training and forms a visual vocabulary. It returns a trained model that
          * can be saved to file for later usage.
          * \param[out] trained_model trained model
          */
        bool
        trainISM (ISMModelPtr& trained_model);

        /** \brief This function is searching for the class of interest in a given cloud
          * and returns the list of votes.
          * \param[in] model trained model which will be used for searching the objects
          * \param[in] in_cloud input cloud that need to be investigated
          * \param[in] in_normals cloud of normals coresponding to the input cloud
          * \param[in] in_class_of_interest class which we are looking for
          */
        boost::shared_ptr<pcl::features::ISMVoteList<PointT> >
        findObjects (ISMModelPtr model, typename pcl::PointCloud<PointT>::Ptr in_cloud, typename pcl::PointCloud<Normal>::Ptr in_normals, int in_class_of_interest);

      protected:

        /** \brief Extracts the descriptors from the input clouds.
          * \param[out] histograms it will store the descriptors for each key point
          * \param[out] locations it will contain the comprehensive information (such as direction, initial keypoint)
          * for the corresponding descriptors
          */
        bool
        extractDescriptors (std::vector<pcl::Histogram<FeatureSize> >& histograms,
                            std::vector<LocationInfo, Eigen::aligned_allocator<LocationInfo> >& locations);

        /** \brief This method performs descriptor clustering.
          * \param[in] histograms descriptors to cluster
          * \param[out] labels it contains labels for each descriptor
          * \param[out] clusters_centers stores the centers of clusters
          */
        bool
        clusterDescriptors (std::vector< pcl::Histogram<FeatureSize> >& histograms, Eigen::MatrixXi& labels, Eigen::MatrixXf& clusters_centers);

        /** \brief This method calculates the value of sigma used for calculating the learned weights for every single class.
          * \param[out] sigmas computed sigmas.
          */
        void
        calculateSigmas (std::vector<float>& sigmas);

        /** \brief This function forms a visual vocabulary and evaluates weights
          * described in [Knopp et al., 2010, (5)].
          * \param[in] locations array containing description of each keypoint: its position, which cloud belongs
          * and expected direction to center
          * \param[in] labels labels that were obtained during k-means clustering
          * \param[in] sigmas array of sigmas for each class
          * \param[in] clusters clusters that were obtained during k-means clustering
          * \param[out] statistical_weights stores the computed statistical weights
          * \param[out] learned_weights stores the computed learned weights
          */
        void
        calculateWeights (const std::vector< LocationInfo, Eigen::aligned_allocator<LocationInfo> >& locations,
                          const Eigen::MatrixXi &labels,
                          std::vector<float>& sigmas,
                          std::vector<std::vector<unsigned int> >& clusters,
                          std::vector<std::vector<float> >& statistical_weights,
                          std::vector<float>& learned_weights);

        /** \brief Simplifies the cloud using voxel grid principles.
          * \param[in] in_point_cloud cloud that need to be simplified
          * \param[in] in_normal_cloud normals of the cloud that need to be simplified
          * \param[out] out_sampled_point_cloud simplified cloud
          * \param[out] out_sampled_normal_cloud and the corresponding normals
          */
        void
        simplifyCloud (typename pcl::PointCloud<PointT>::ConstPtr in_point_cloud,
                       typename pcl::PointCloud<NormalT>::ConstPtr in_normal_cloud,
                       typename pcl::PointCloud<PointT>::Ptr out_sampled_point_cloud,
                       typename pcl::PointCloud<NormalT>::Ptr out_sampled_normal_cloud);

        /** \brief This method simply shifts the clouds points relative to the passed point.
          * \param[in] in_cloud cloud to shift
          * \param[in] shift_point point relative to which the cloud will be shifted
          */
        void
        shiftCloud (typename pcl::PointCloud<PointT>::Ptr in_cloud, Eigen::Vector3f shift_point);

        /** \brief This method simply computes the rotation matrix, so that the given normal
          * would match the Y axis after the transformation. This is done because the algorithm needs to be invariant
          * to the affine transformations.
          * \param[in] in_normal normal for which the rotation matrix need to be computed
          */
        Eigen::Matrix3f
        alignYCoordWithNormal (const NormalT& in_normal);

        /** \brief This method applies transform set in in_transform to vector io_vector.
          * \param[in] io_vec vector that need to be transformed
          * \param[in] in_transform matrix that contains the transformation
          */
        void
        applyTransform (Eigen::Vector3f& io_vec, const Eigen::Matrix3f& in_transform);

        /** \brief This method estimates features for the given point cloud.
          * \param[in] sampled_point_cloud sampled point cloud for which the features must be computed
          * \param[in] normal_cloud normals for the original point cloud
          * \param[out] feature_cloud it will store the computed histograms (features) for the given cloud
          */
        void
        estimateFeatures (typename pcl::PointCloud<PointT>::Ptr sampled_point_cloud,
                          typename pcl::PointCloud<NormalT>::Ptr normal_cloud,
                          typename pcl::PointCloud<pcl::Histogram<FeatureSize> >::Ptr feature_cloud);

        /** \brief Performs K-means clustering.
          * \param[in] points_to_cluster points to cluster
          * \param[in] number_of_clusters desired number of clusters
          * \param[out] io_labels output parameter, which stores the label for each point
          * \param[in] criteria defines when the computational process need to be finished. For example if the
          * desired accuracy is achieved or the iteration number exceeds given value
          * \param[in] attempts number of attempts to compute clustering
          * \param[in] flags if set to USE_INITIAL_LABELS then initial approximation of labels is taken from io_labels
          * \param[out] cluster_centers it will store the cluster centers
          */
        double
        computeKMeansClustering (const Eigen::MatrixXf& points_to_cluster,
                                 int number_of_clusters,
                                 Eigen::MatrixXi& io_labels,
                                 TermCriteria criteria,
                                 int attempts,
                                 int flags,
                                 Eigen::MatrixXf& cluster_centers);

        /** \brief Generates centers for clusters as described in 
          * Arthur, David and Sergei Vassilvitski (2007) k-means++: The Advantages of Careful Seeding.
          * \param[in] data points to cluster
          * \param[out] out_centers it will contain generated centers
          * \param[in] number_of_clusters defines the number of desired cluster centers
          * \param[in] trials number of trials to generate a center
          */
        void
        generateCentersPP (const Eigen::MatrixXf& data,
                           Eigen::MatrixXf& out_centers,
                           int number_of_clusters,
                           int trials);

        /** \brief Generates random center for cluster.
          * \param[in] boxes contains min and max values for each dimension
          * \param[out] center it will the contain generated center
          */
        void
        generateRandomCenter (const std::vector<Eigen::Vector2f>& boxes, Eigen::VectorXf& center);

        /** \brief Computes the square distance beetween two vectors.
          * \param[in] vec_1 first vector
          * \param[in] vec_2 second vector
          */
        float
        computeDistance (Eigen::VectorXf& vec_1, Eigen::VectorXf& vec_2);

        /** \brief Forbids the assignment operator. */
        ImplicitShapeModelEstimation&
        operator= (const ImplicitShapeModelEstimation&);

      protected:

        /** \brief Stores the clouds used for training. */
        std::vector<typename pcl::PointCloud<PointT>::Ptr> training_clouds_;

        /** \brief Stores the class number for each cloud from training_clouds_. */
        std::vector<unsigned int> training_classes_;

        /** \brief Stores the normals for each training cloud. */
        std::vector<typename pcl::PointCloud<NormalT>::Ptr> training_normals_;

        /** \brief This array stores the sigma values for each training class. If this array has a size equals 0, then
          * sigma values will be calculated automatically.
          */
        std::vector<float> training_sigmas_;

        /** \brief This value is used for the simplification. It sets the size of grid bin. */
        float sampling_size_;

        /** \brief Stores the feature estimator. */
        boost::shared_ptr<pcl::Feature<PointT, pcl::Histogram<FeatureSize> > > feature_estimator_;

        /** \brief Number of clusters, is used for clustering descriptors during the training. */
        unsigned int number_of_clusters_;

        /** \brief If set to false then Nvot coeff from [Knopp et al., 2010, (4)] is equal 1.0. */
        bool n_vot_ON_;

        /** \brief This const value is used for indicating that for k-means clustering centers must
          * be generated as described in
          * Arthur, David and Sergei Vassilvitski (2007) k-means++: The Advantages of Careful Seeding. */
        static const int PP_CENTERS = 2;

        /** \brief This const value is used for indicating that input labels must be taken as the
          * initial approximation for k-means clustering. */
        static const int USE_INITIAL_LABELS = 1;
    };
  }
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::ISMPeak,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, density, ism_density)
  (float, class_id, ism_class_id)
)

#endif  //#ifndef PCL_IMPLICIT_SHAPE_MODEL_H_
