#pragma once

#include <pcl/features/integral_image2D.h>
#include <pcl/memory.h>
#include <pcl/pcl_macros.h>

#include <Eigen/Core>

namespace pcl
{
  namespace face_detection
  {
    class TrainingExample
    {
      public:
        std::vector<pcl::IntegralImage2D<float, 1>::Ptr> iimages_; //also pointer to the respective integral image
        int row_, col_;
        int wsize_;
        int label_;

        //save pose head information
        Eigen::Vector3f trans_;
        Eigen::Vector3f rot_;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };

    class FeatureType
    {
      public:
        int row1_, col1_;
        int row2_, col2_;

        int wsizex1_, wsizey1_;
        int wsizex2_, wsizey2_;

        float threshold_;
        int used_ii_;

        FeatureType()
        {
          used_ii_ = 0;
        }

        void serialize(std::ostream & stream) const
        {
          stream.write (reinterpret_cast<const char*> (&row1_), sizeof(row1_));
          stream.write (reinterpret_cast<const char*> (&col1_), sizeof(col1_));
          stream.write (reinterpret_cast<const char*> (&row2_), sizeof(row2_));
          stream.write (reinterpret_cast<const char*> (&col2_), sizeof(col2_));
          stream.write (reinterpret_cast<const char*> (&wsizex1_), sizeof(wsizex1_));
          stream.write (reinterpret_cast<const char*> (&wsizex2_), sizeof(wsizex2_));
          stream.write (reinterpret_cast<const char*> (&wsizey1_), sizeof(wsizey1_));
          stream.write (reinterpret_cast<const char*> (&wsizey2_), sizeof(wsizey2_));
          stream.write (reinterpret_cast<const char*> (&threshold_), sizeof(threshold_));
          stream.write (reinterpret_cast<const char*> (&used_ii_), sizeof(used_ii_));
        }

        inline void deserialize(std::istream & stream)
        {
          stream.read (reinterpret_cast<char*> (&row1_), sizeof(row1_));
          stream.read (reinterpret_cast<char*> (&col1_), sizeof(col1_));
          stream.read (reinterpret_cast<char*> (&row2_), sizeof(row2_));
          stream.read (reinterpret_cast<char*> (&col2_), sizeof(col2_));
          stream.read (reinterpret_cast<char*> (&wsizex1_), sizeof(wsizex1_));
          stream.read (reinterpret_cast<char*> (&wsizex2_), sizeof(wsizex2_));
          stream.read (reinterpret_cast<char*> (&wsizey1_), sizeof(wsizey1_));
          stream.read (reinterpret_cast<char*> (&wsizey2_), sizeof(wsizey2_));
          stream.read (reinterpret_cast<char*> (&threshold_), sizeof(threshold_));
          stream.read (reinterpret_cast<char*> (&used_ii_), sizeof(used_ii_));
        }
    };

    template<class FeatureType>
    class RFTreeNode
    {
      public:
        float threshold;
        FeatureType feature;
        std::vector<RFTreeNode> sub_nodes;
        float value;
        float variance;

        Eigen::Vector3d trans_mean_;
        Eigen::Vector3d rot_mean_;

        float purity_;
        Eigen::Matrix3d covariance_trans_;
        Eigen::Matrix3d covariance_rot_;

        PCL_MAKE_ALIGNED_OPERATOR_NEW

        void serialize(::std::ostream & stream) const
        {

          const int num_of_sub_nodes = static_cast<int> (sub_nodes.size ());
          stream.write (reinterpret_cast<const char*> (&num_of_sub_nodes), sizeof(num_of_sub_nodes));

          if (!sub_nodes.empty ())
          {
            feature.serialize (stream);
            stream.write (reinterpret_cast<const char*> (&threshold), sizeof(threshold));
          }

          stream.write (reinterpret_cast<const char*> (&value), sizeof(value));
          stream.write (reinterpret_cast<const char*> (&variance), sizeof(variance));

          for (std::size_t i = 0; i < 3; i++)
            stream.write (reinterpret_cast<const char*> (&trans_mean_[i]), sizeof(trans_mean_[i]));

          for (std::size_t i = 0; i < 3; i++)
            stream.write (reinterpret_cast<const char*> (&rot_mean_[i]), sizeof(rot_mean_[i]));

          for (std::size_t i = 0; i < 3; i++)
            for (std::size_t j = 0; j < 3; j++)
              stream.write (reinterpret_cast<const char*> (&covariance_trans_ (i, j)), sizeof(covariance_trans_ (i, j)));

          for (std::size_t i = 0; i < 3; i++)
            for (std::size_t j = 0; j < 3; j++)
              stream.write (reinterpret_cast<const char*> (&covariance_rot_ (i, j)), sizeof(covariance_rot_ (i, j)));

          for (int sub_node_index = 0; sub_node_index < num_of_sub_nodes; ++sub_node_index)
          {
            sub_nodes[sub_node_index].serialize (stream);
          }
        }

        inline void deserialize(::std::istream & stream)
        {
          int num_of_sub_nodes;
          stream.read (reinterpret_cast<char*> (&num_of_sub_nodes), sizeof(num_of_sub_nodes));

          if (num_of_sub_nodes > 0)
          {
            feature.deserialize (stream);
            stream.read (reinterpret_cast<char*> (&threshold), sizeof(threshold));
          }

          stream.read (reinterpret_cast<char*> (&value), sizeof(value));
          stream.read (reinterpret_cast<char*> (&variance), sizeof(variance));

          for (std::size_t i = 0; i < 3; i++)
            stream.read (reinterpret_cast<char*> (&trans_mean_[i]), sizeof(trans_mean_[i]));

          for (std::size_t i = 0; i < 3; i++)
            stream.read (reinterpret_cast<char*> (&rot_mean_[i]), sizeof(rot_mean_[i]));

          for (std::size_t i = 0; i < 3; i++)
            for (std::size_t j = 0; j < 3; j++)
              stream.read (reinterpret_cast<char*> (&covariance_trans_ (i, j)), sizeof(covariance_trans_ (i, j)));

          for (std::size_t i = 0; i < 3; i++)
            for (std::size_t j = 0; j < 3; j++)
              stream.read (reinterpret_cast<char*> (&covariance_rot_ (i, j)), sizeof(covariance_rot_ (i, j)));

          sub_nodes.resize (num_of_sub_nodes);

          if (num_of_sub_nodes > 0)
          {
            for (int sub_node_index = 0; sub_node_index < num_of_sub_nodes; ++sub_node_index)
            {
              sub_nodes[sub_node_index].deserialize (stream);
            }
          }
        }
    };
  }
}
