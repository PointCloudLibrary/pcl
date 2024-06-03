#include <pcl/test/gtest.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>

#include <random>

using namespace pcl;
using std::vector;

TEST(ExtractPolygonalPrism, two_rings)
{
  float rMin = 0.1, rMax = 0.25f;
  float dx = 0.5f; // shift the rings from [0,0,0] to [+/-dx, 0, 0]

  // prepare 2 rings
  PointCloud<PointXYZ>::Ptr ring(new PointCloud<PointXYZ>);
  { // use random
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> radiusDist(rMin, rMax);
    std::uniform_real_distribution<float> radianDist(-M_PI, M_PI);
    std::uniform_real_distribution<float> zDist(-0.01f, 0.01f);
    for (size_t i = 0; i < 1000; i++) {
      float radius = radiusDist(gen);
      float angle = radianDist(gen);
      float z = zDist(gen);
      PointXYZ point(cosf(angle) * radius, sinf(angle) * radius, z);
      ring->push_back(point);
    }
  }

  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  cloud->reserve(ring->size() * 2);
  for (auto& point : ring->points) {
    auto left = point;
    auto right = point;
    left.x -= dx;
    right.x += dx;
    cloud->push_back(left);
    cloud->push_back(right);
  }

  // create hull
  PointCloud<PointXYZ>::Ptr hullCloud(new PointCloud<PointXYZ>);
  vector<Vertices> rings(4);
  float radiuses[] = {rMin - 0.01f, rMax + 0.01f, rMin - 0.01f, rMax + 0.01f};
  float centers[] = {-dx, -dx, +dx, +dx};
  for (size_t i = 0; i < rings.size(); i++) {
    auto r = radiuses[i];
    auto xCenter = centers[i];
    for (float a = -M_PI; a < M_PI; a += 0.05f) {
      rings[i].vertices.push_back(hullCloud->size());
      PointXYZ point(xCenter + r * cosf(a), r * sinf(a), 0);
      hullCloud->push_back(point);
    }
  }

  // add more points before using prism
  size_t ringsPointCount = cloud->size();
  cloud->push_back(PointXYZ(0, 0, 0));
  for (float a = -M_PI; a < M_PI; a += 0.05f) {
    float r = 4 * rMax;
    PointXYZ point(r * cosf(a), r * sinf(a), 0);
    cloud->push_back(point);
  }

  // do prism
  PointIndices::Ptr inliers(new PointIndices);
  ExtractPolygonalPrismData<PointXYZ> ex;
  {
    ex.setInputCloud(cloud);
    ex.setInputPlanarHull(hullCloud);
    ex.setHeightLimits(-1, 1);
    ex.setPolygons(rings);
    ex.segment(*inliers);
  }

  // check that all of the rings are in the prism.
  EXPECT_EQ(inliers->indices.size(), ringsPointCount);
  for(std::size_t i=0; i<inliers->indices.size(); ++i) {
    EXPECT_EQ(inliers->indices[i], i);
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return (RUN_ALL_TESTS());
}
