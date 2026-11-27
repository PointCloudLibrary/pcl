/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Point CLoud Library (PCL) - www.pointclouds.org
 * Copyright (c) 2021-, Open Perception
 *
 * All rights reserved
 */

#include <pcl/filters/experimental/cartesian_filter.h>
#include <pcl/filters/experimental/transform_filter.h>
#include <pcl/filters/filter.h>
#include <pcl/test/gtest.h>
#include <pcl/pcl_tests.h>
#include <pcl/point_types.h>

#include <boost/optional.hpp> // std::optional for C++17

#include <unordered_map>

using namespace pcl;
using namespace pcl::test;

// Grid structure with necessary declarations for CartesianFilter

struct EmptyVoxel {
  bool voxel_info = false;
};

// hash map grid
template <typename PointT>
struct EmptyMapStruct {
  using PointCloud = pcl::PointCloud<PointT>;
  using Grid = std::unordered_map<std::size_t, EmptyVoxel>;
  using GridIterator = Grid::iterator;

  EmptyMapStruct() { filter_name_ = "empty_map"; }

  std::size_t
  size() const
  {
    return grid_.size();
  }

  GridIterator
  begin()
  {
    return grid_.begin();
  }

  GridIterator
  end()
  {
    return grid_.end();
  }

  bool
  setUp(experimental::CartesianFilter<Filter, EmptyMapStruct>&)
  {
    return pass_set_up_;
  }

  void
  addPointToGrid(const PointT& pt)
  {
    grid_[pt.x].voxel_info = true;
  }

  experimental::optional<PointT>
  filterGrid(Grid::iterator grid_it)
  {

    if (return_point_) {
      // hashing index of the iterating grid
      return PointT(grid_it->first, 0, 0);
    }
    return boost::none;
  }

  std::string filter_name_;
  Grid grid_;

  // for testing
  bool pass_set_up_ = true;
  bool return_point_ = true;
};

// vector grid
template <typename PointT>
struct EmptyVecStruct {
  using PointCloud = pcl::PointCloud<PointT>;
  using Grid = std::vector<std::size_t>;
  using GridIterator = Grid::iterator;

  EmptyVecStruct() { filter_name_ = "empty_vec"; }

  std::size_t
  size() const
  {
    return grid_.size();
  }

  GridIterator
  begin()
  {
    return grid_.begin();
  }

  GridIterator
  end()
  {
    return grid_.end();
  }

  bool
  setUp(experimental::CartesianFilter<Filter, EmptyVecStruct>&)
  {
    return true;
  }

  void
  addPointToGrid(const PointT& pt)
  {
    grid_.push_back(pt.x);
  }

  experimental::optional<PointT>
  filterGrid(Grid::iterator grid_it)
  {
    // element, index of the iterating element
    return PointT(*grid_it, grid_it - grid_.begin(), 0);
  }

  std::string filter_name_;
  Grid grid_;

  // for testing
  bool pass_set_up_ = true;
  bool return_point_ = true;
};

template <typename GridStruct, typename PointT = GET_POINT_TYPE(GridStruct)>
class MockFilter : public experimental::CartesianFilter<Filter, GridStruct> {
public:
  // test passed or failed setUp
  void
  setPassSetUp(const bool pass_set_up)
  {
    this->getGridStruct().pass_set_up_ = pass_set_up;
  }

  // test optional
  void
  setReturnPoint(const bool return_point)
  {
    this->getGridStruct().return_point_ = return_point;
  }
};

TEST(ApplyFilter, GridFilter)
{
  PointCloud<PointXYZ>::Ptr input = std::make_shared<PointCloud<PointXYZ>>();
  input->push_back(PointXYZ(10, 10, 10));
  input->push_back(PointXYZ(20, 20, 20));

  PointCloud<PointXYZ> output;

  // hash map grid
  MockFilter<EmptyMapStruct<PointXYZ>> mf;
  mf.setInputCloud(input);
  mf.filter(output);

  std::unordered_map<size_t, bool> coordinates{{10, false}, {20, false}};

  for (std::size_t i = 0; i < input->size(); ++i) {
    EXPECT_FALSE(coordinates.at(static_cast<size_t>(output.at(i).x)));
    coordinates.at(static_cast<size_t>(output.at(i).x)) = true;
  }
  output.clear();

  // vector grid
  MockFilter<EmptyVecStruct<PointXYZ>> vf;
  vf.setInputCloud(input);
  vf.filter(output);

  for (std::size_t i = 0; i < input->size(); ++i) {
    EXPECT_EQ(output.at(i).x, input->at(i).x);
    EXPECT_EQ(output.at(i).y, i);
  }
}

TEST(StructMethods, GridFilter)
{
  // disable warning
  console::setVerbosityLevel(console::L_ALWAYS);

  PointCloud<PointXYZ>::Ptr input = std::make_shared<PointCloud<PointXYZ>>();
  input->push_back(PointXYZ(10, 10, 10));
  input->push_back(PointXYZ(20, 20, 20));

  PointCloud<PointXYZ> output;

  MockFilter<EmptyMapStruct<PointXYZ>> f;
  output.is_dense = false;
  f.filter(output);

  // return if input is empty
  EXPECT_FALSE(output.is_dense);
  output.clear();

  f.setInputCloud(input);
  f.setPassSetUp(false);   // setUp
  f.setReturnPoint(false); // filterGrid
  f.filter(output);

  // failed setUp
  EXPECT_TRUE(output.is_dense);
  EXPECT_EQ(output.height, 1);
  EXPECT_EQ(output.header, input->header);
  EXPECT_EQ_VECTORS(output.sensor_origin_, input->sensor_origin_);
  EXPECT_EQ_VECTORS(output.sensor_orientation_.vec(), input->sensor_orientation_.vec());
  EXPECT_EQ(output.size(), input->size());
  output.clear();

  // filterGrid return empty optional
  f.setPassSetUp(true);    // setUp
  f.setReturnPoint(false); // filterGrid
  f.filter(output);
  EXPECT_EQ(output.size(), 0);
  output.clear();

  // filterGrid return points
  f.setPassSetUp(true);   // setUp
  f.setReturnPoint(true); // filterGrid
  f.filter(output);
  EXPECT_EQ(output.size(), input->size());
}

TEST(CheckHashinghRange, GridFilter)
{
  const float min_float = std::numeric_limits<float>::min();
  const float max_float = std::numeric_limits<float>::max();

  Eigen::Vector4f min_p;
  Eigen::Vector4f max_p;
  Eigen::Array4f inv_size;

  min_p = Eigen::Array4f::Zero();
  max_p = Eigen::Array4f::Zero();
  inv_size = Eigen::Array4f::Constant(1);
  EXPECT_TRUE(experimental::checkHashRange2D(min_p, max_p, inv_size.head<2>()));
  EXPECT_TRUE(experimental::checkHashRange3D(min_p, max_p, inv_size.head<3>()));

  min_p << min_float, 0, 0, 0;
  max_p << max_float, 0, 0, 0;
  inv_size = Eigen::Array4f::Constant(1);
  EXPECT_FALSE(experimental::checkHashRange2D(min_p, max_p, inv_size.head<2>()));
  EXPECT_FALSE(experimental::checkHashRange3D(min_p, max_p, inv_size.head<3>()));

  min_p = Eigen::Array4f::Constant(min_float);
  max_p = Eigen::Array4f::Constant(max_float);
  inv_size = Eigen::Array4f::Constant(max_float);
  EXPECT_FALSE(experimental::checkHashRange2D(min_p, max_p, inv_size.head<2>()));
  EXPECT_FALSE(experimental::checkHashRange3D(min_p, max_p, inv_size.head<3>()));

  min_p = Eigen::Array4f::Zero();
  inv_size = Eigen::Array4f::Constant(1);
  const std::size_t max_size = std::numeric_limits<std::size_t>::max();

  // 2D
  const float fail_2d_size = std::ceil(sqrt(max_size));
  const float pass_2d_size = std::nexttoward(fail_2d_size, -1) - 1;
  max_p = Eigen::Array4f::Constant(pass_2d_size);
  EXPECT_TRUE(experimental::checkHashRange2D(min_p, max_p, inv_size.head<2>()));
  max_p = Eigen::Array4f::Constant(fail_2d_size);
  EXPECT_FALSE(experimental::checkHashRange2D(min_p, max_p, inv_size.head<2>()));

  // 3D
  const float fail_3d_size = std::ceil(cbrt(max_size));
  const float pass_3d_size = std::nexttoward(fail_3d_size, -1) - 1;
  max_p = Eigen::Array4f::Constant(pass_3d_size);
  EXPECT_TRUE(experimental::checkHashRange3D(min_p, max_p, inv_size.head<3>()));
  max_p = Eigen::Array4f::Constant(fail_3d_size);
  EXPECT_FALSE(experimental::checkHashRange3D(min_p, max_p, inv_size.head<3>()));
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
