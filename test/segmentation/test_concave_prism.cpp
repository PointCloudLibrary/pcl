#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <random>

using namespace pcl;
using std::vector;

TEST(ExtractPolygonalPrism, two_rings)
{
  // prepare 2 rings
  PointCloud<PointXYZ>::Ptr ring(new PointCloud<PointXYZ>);
  { // use random
    std::random_device rd;
    std::mt19937 gen(rd());
    float rMin = 0.1, rMax = 0.25f;
    std::uniform_real_distribution<float> radiusDist(rMin, rMax);
    std::uniform_real_distribution<float> radianDist(-M_PI, M_PI);
    std::uniform_real_distribution<float> zDist(-0.01f, 0.01f);
    for (size_t i = 0; i < 1000; i++) {
      float radius = radiusDist(gen);
      float angle = radianDist(gen);
      float z = zDist(gen);
      PointXYZ point(cos(angle) * radius, sin(angle) * radius, z);
      ring->push_back(point);
    }
  }

  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  cloud->reserve(ring->size() * 2);
  for (auto& point : ring->points) {
    auto left = point;
    auto rght = point;
    left.x -= 0.5f;
    rght.x += 0.5f;
    cloud->points.push_back(left);
    cloud->points.push_back(rght);
  }

  // find hull
  PointCloud<PointXYZ>::Ptr hullCloud(new PointCloud<PointXYZ>);
  vector<Vertices> rings;
  {
    ConcaveHull<PointXYZ> concaveHull;
    concaveHull.setInputCloud(cloud);
    concaveHull.setAlpha(0.05);
    concaveHull.reconstruct(*hullCloud, rings);
  }
  EXPECT_EQ(rings.size(), 4);

  // add more points before using prism
  size_t ringsPointCount = cloud->size();
  cloud->points.push_back({0, 0, 0});
  for (float a = -M_PI; a < M_PI; a += 0.05f) {
    float r = 1.f;
    cloud->points.push_back({r * cos(a), r * sin(a), 0});
  }

  // do prism
  PointIndices::Ptr inliers(new PointIndices);
  ExtractPolygonalPrismData<PointXYZ> ex;
  {
    ex.setInputCloud(cloud);
    ex.setInputPlanarHull(hullCloud);
    ex.setHeightLimits(-1, 1);
    ex.setRings(rings);
    ex.segment(*inliers);
  }

  // check that almost all of the rings are in the prism.
  // *almost* because the points on the border is not well defined,
  // and may be inside or outside.
  EXPECT_NEAR(inliers->indices.size(), ringsPointCount, 100);
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
