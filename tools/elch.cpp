#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/elch.h>

//#include <boost/graph/graphviz.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;

#include <boost/graph/adjacency_list.hpp>
using boost::edge_weight_t;

#include <iostream>
using std::cout;
using std::endl;
#include <string>
using std::string;
#include <fstream>
using std::ofstream;

#include <vector>
using std::vector;

#include <list>
using std::list;

#include <algorithm>
using std::swap;

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
typedef std::pair<std::string, CloudPtr> CloudPair;
typedef std::vector<CloudPair> CloudVector;

typedef boost::adjacency_list<
  boost::listS, boost::vecS, boost::undirectedS,
  boost::no_property,
  boost::property< edge_weight_t, double > >
Graph;

void
elch (int start, int end, const CloudVector &clouds)
{
  CloudPtr tmp (new Cloud);
  pcl::registration::ELCH<pcl::PointXYZ> theclass;


  static Graph g;

  for (size_t i = num_vertices (g)+1; i < clouds.size (); i++)
    add_edge (i-1, i, g);

  //TODO use pose
  }
}

void
loopDetection (int end, const CloudVector &clouds, double dist)
{
  static double min_dist = -1;
  static int first, last;
  int state = 0;

  for (int i = end-1; i > 0; i--)
  {
    Eigen::Vector4f cstart, cend;
    //TODO use pose of scan
    pcl::compute3DCentroid (*(clouds[i].second), cstart);
    pcl::compute3DCentroid (*(clouds[end].second), cend);
    Eigen::Vector4f diff = cend - cstart;

    double norm = diff.norm ();

    //std::cout << "distance between " << i << " and " << end << " is " << norm << " state is " << state << std::endl;

    if (state == 0 && norm > dist)
    {
      state = 1;
      //std::cout << "state 1" << std::endl;
    }
    if (state > 0 && norm < dist)
    {
      state = 2;
      std::cout << "loop detected between scan " << i << " (" << clouds[i].first << ") and scan " << end << " (" << clouds[end].first << ")" << std::endl;
      if (min_dist < 0 || norm < min_dist)
      {
        min_dist = norm;
        first = i;
        last = end;
      }
    }
  }
  std::cout << "min_dist: " << min_dist << " state: " << state << " first: " << first << " end: " << end << std::endl;
  if (min_dist > 0 && (state < 2 || end == (int)clouds.size () - 1)) //TODO
  {
    min_dist = -1;
    std::cout << "calling elch with " << first << " and " << last << std::endl;
    elch (first, last, clouds);
    std::cout << "finished calling elch" << std::endl;
  }
}

int
main (int argc, char **argv)
{
  CloudVector clouds;
  for (int i = 1; i < argc; i++)
  {
    CloudPtr pc (new Cloud);
    pcl::io::loadPCDFile (argv[i], *pc);
    clouds.push_back (CloudPair (argv[i], pc));
    std::cout << "loading file: " << argv[i] << " size: " << pc->size () << std::endl;
  }

  for (size_t i = 0; i < clouds.size (); i++)
    loopDetection (i, clouds, 15.0);

  for (size_t i = 0; i < clouds.size (); i++)
  {
    std::string result_filename (clouds[i].first);
    result_filename = result_filename.substr (result_filename.rfind ("/") + 1);
    pcl::io::savePCDFileBinary (result_filename.c_str (), *(clouds[i].second));
    cout << "saving result to " << result_filename << endl;
  }

  return 0;
}
