.. _octree_search:

Spatial Partitioning and Search Operations with Octrees
------------------------------------------

An octree is a tree-based data structure for managing sparse 3-D data. Each internal node has exactly eight children.
In this tutorial we will learn how to use the octree for spatial partitioning and neighbor search within pointcloud data. Particularly, we explain how to perform a "Neighbors within Voxel Search", the 
"K Nearest Neighbor Search" and "Neighbors within Radius Search".


The code:
--------------
First, create a file, let's say, ``octree_search.cpp`` and place the following inside it:


.. code-block:: cpp
   :linenos:

	#include <pcl/point_cloud.h>
	#include "pcl/octree/octree.h"
	
	#include <iostream>
	#include <vector>
	
	using namespace pcl;
	using namespace pcl::octree;
	
	int
	main (int argc, char** argv)
	{
	
	  srand (time (NULL));
	
	  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
	
	  // Generate pointcloud data
	  cloud->width = 1000;
	  cloud->height = 1;
	  cloud->points.resize (cloud->width * cloud->height);
	
	  for (size_t i = 0; i < cloud->points.size (); ++i)
	  {
	    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0);
	    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0);
	    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0);
	  }
	
	  float resolution = 128.0f;
	
	  OctreePointCloud<PointXYZ> octree (resolution);
	
	  octree.setInputCloud (cloud);
	  octree.addPointsFromInputCloud ();
	
	  PointXYZ searchPoint;
	
	  searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0);
	  searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0);
	  searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0);
	
	  // Neighbors within voxel search
	
	  std::vector<int> pointIdxVec;
	
	  if (octree.voxelSearch (searchPoint, pointIdxVec))
	  {
	    std::cerr << "Neighbors within voxel search at (" << searchPoint.x 
	              << " " << searchPoint.y 
	              << " " << searchPoint.z << ")" 
	              << std::endl;
	              
	    for (size_t i = 0; i < pointIdxVec.size (); ++i)
	      std::cerr << "    " << cloud->points[pointIdxVec[i]].x 
	                << " " << cloud->points[pointIdxVec[i]].y 
	                << " " << cloud->points[pointIdxVec[i]].z << std::endl;
	  }
	
	  // K nearest neighbor search
	
	  int K = 10;
	
	  std::vector<int> pointIdxNKNSearch;
	  std::vector<float> pointNKNSquaredDistance;
	
	  std::cerr << "K nearest neighbor search at (" << searchPoint.x 
	            << " " << searchPoint.y 
	            << " " << searchPoint.z
	            << ") with K=" << K << std::endl;
	
	  if ( octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	  {
	    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	      std::cerr << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
	                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
	                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
	                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	  }
	
	  // Neighbors within radius search
	
	  std::vector<int> pointIdxRadiusSearch;
	  std::vector<float> pointRadiusSquaredDistance;
	
	  float radius = 256.0f * rand () / (RAND_MAX + 1.0);
	
	  std::cerr << "Neighbors within radius search at (" << searchPoint.x 
	            << " " << searchPoint.y 
	            << " " << searchPoint.z
	            << ") with radius=" << radius << std::endl;
	
	
	  if ( octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	  {
	    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
	      std::cerr << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
	                << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
	                << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
	                << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	  }
	
	}


The explanation
---------------

Now, let's explain the code in detail.

We fist define and instantiate a shared PointCloud structure and fill it with random points.

.. code-block:: cpp

	  PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ> ());
	
	  // Generate pointcloud data
	  cloud->width = 1000;
	  cloud->height = 1;
	  cloud->points.resize (cloud->width * cloud->height);
	
	  for (size_t i = 0; i < cloud->points.size (); ++i)
	  {
	    cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0);
	    cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0);
	    cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0);
	  }


Then we create an octree instance which is initialized with its resolution. This octree keeps a vector of point indices within its leaf nodes.
The resolution parameter describes the length of the smalles voxels at lowest octree level. The depth of the octree is therefore a function of the resolution as well as 
the spatial dimension of the pointcloud. If a bounding box of the pointcloud is know, it should be assigned to the octree by using the defineBoundingBox method. 
Then we assign a pointer to the PointCloud and add all points to the octree.

.. code-block:: cpp

	  float resolution = 128.0f;
	  
	  OctreePointCloud<PointXYZ> octree (resolution);
	
	  octree.setInputCloud (cloud);
	  octree.addPointsFromInputCloud ();


Once the PointCloud is associated with an octree, we can perform search operations. The fist search method used here is "Neighbors within Voxel Search". It assigns the search point to the corresponding 
leaf node voxel and returns a vector of point indices. These indices relate to points which fall within the same voxel. The distance between 
the search point and the search result depend therefore on the resolution parameter of the octree.

.. code-block:: cpp
	
	  std::vector<int> pointIdxVec;
	
	  if (octree.voxelSearch (searchPoint, pointIdxVec))
	  {
	    std::cerr << "Neighbors within voxel search at (" << searchPoint.x 
	              << " " << searchPoint.y 
	              << " " << searchPoint.z << ")"
	              << std::endl;
	              
	    for (size_t i = 0; i < pointIdxVec.size (); ++i)
	      std::cerr << "    " << cloud->points[pointIdxVec[i]].x 
	                << " " << cloud->points[pointIdxVec[i]].y 
	                << " " << cloud->points[pointIdxVec[i]].z << std::endl;
	  }

Next, a K nearest neighbor search is demonstrated. In this example, K is set to 10. The "K Nearest Neighbor Search" method writes the search results into two separate vectors. 
The first one, pointIdxNKNSearch, will contain the search result (indices referring to the associated PointCloud data set). The second vector holds corresponding squared distances
between the search point and the nearest neighbors.  

.. code-block:: cpp

	  // K nearest neighbor search
	
	  int K = 10;
	
	  std::vector<int> pointIdxNKNSearch;
	  std::vector<float> pointNKNSquaredDistance;
	
	  std::cerr << "K nearest neighbor search at (" << searchPoint.x 
	            << " " << searchPoint.y 
	            << " " << searchPoint.z
	            << ") with K=" << K << std::endl;
	
	  if ( octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
	  {
	    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
	      std::cerr << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x 
	                << " " << cloud->points[ pointIdxNKNSearch[i] ].y 
	                << " " << cloud->points[ pointIdxNKNSearch[i] ].z 
	                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	  }

The "Neighbors within Radius Search" works very similar to the "K Nearest Neighbor Search". Its search results are written to two separate vectors describing 
point indices and squares search point distances. 

.. code-block:: cpp

	  std::vector<int> pointIdxRadiusSearch;
	  std::vector<float> pointRadiusSquaredDistance;
	
	  float radius = 256.0f * rand () / (RAND_MAX + 1.0);
	

	  std::cerr << "Neighbors within radius search at (" << searchPoint.x 
	            << " " << searchPoint.y 
	            << " " << searchPoint.z
	            << ") with radius=" << radius << std::endl;
	
	
	  if ( octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	  {
	    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
	      std::cerr << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
	               << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
	               << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
	               << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	  }


Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (octreesearch octree_search.cpp)
   target_link_libraries (octreesearch pcl_common)

After you have made the executable, you can run it. Simply do::

  $ ./octreesearch

You will see something similar to::

	Neighbors within voxel search at (974.82 188.793 138.779)
	    903.656 82.8158 162.392
	    1007.34 191.035 61.7727
	    896.88 155.711 58.1942
	K nearest neighbor search at (974.82 188.793 138.779) with K=10
	    903.656 82.8158 162.392 (squared distance: 16853.1)
	    903.18 247.058 54.3528 (squared distance: 15655)
	    861.595 149.96 135.199 (squared distance: 14340.7)
	    896.88 155.711 58.1942 (squared distance: 13663)
	    995.889 116.224 219.077 (squared distance: 12157.9)
	    885.852 238.41 160.966 (squared distance: 10869.5)
	    900.807 220.317 77.1432 (squared distance: 10270.7)
	    1002.46 117.236 184.594 (squared distance: 7983.59)
	    1007.34 191.035 61.7727 (squared distance: 6992.54)
	    930.13 223.335 174.763 (squared distance: 4485.15)
	Neighbors within radius search at (974.82 188.793 138.779) with radius=109.783
	    1007.34 191.035 61.7727 (squared distance: 6992.54)
	    900.807 220.317 77.1432 (squared distance: 10270.7)
	    885.852 238.41 160.966 (squared distance: 10869.5)
	    1002.46 117.236 184.594 (squared distance: 7983.59)
	    930.13 223.335 174.763 (squared distance: 4485.15)


Additional Details
------------------

Several octree types are provided by the PCL octree component. They basically differ by their individual leaf node characteristics. 

* OctreePointCloudPointVector (equal to OctreePointCloud): This octree can hold a list of point indices at each leaf node.
* OctreePointCloudSinglePoint: This octree class hold only a single point indices at each leaf node. Only the most recent point index that is assigned to the leaf node is stored. 
* OctreePointCloudOccupancy: This octree does not store any point information at its leaf nodes. It can be used for spatial occupancy checks. 
* OctreePointCloudDensity: This octree counts the amount of points within each leaf node voxel. It allows for spatial density queries. 

If octrees needs to be created at high rate, please have a look at the octree double buffering implementation ( Octree2BufBase class ). This class 
keeps two parallel octree structures in the memory at the same time. In addition to search operations, this also enables spatial change detection. Furthermore, an advanced memory management reduces memory allocation 
and deallocation operations during the octree building process. The double buffering octree implementation can be assigned to all OctreePointCloud classes via the template argument "OctreeT". 

All octrees support serialization and deserialization of the octree structure and the octree data content. 

Conclusion
----------

The PCL octree implementation is a powerful tools for spatial partitioning and search operation. 

