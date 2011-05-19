.. _octree_compression:

Point Cloud Compression
--------------------------------------------------------

Point clouds consist of huge data sets describing three dimensional points associated with
additional information such as distance, color, normals, etc. Additionally, they can be created at high rate and therefore occupy a significant amount
of memory resources. Once point clouds have to be stored or transmitted over rate-limited communication channels, 
methods for compressing this kind of data become highly interesting. The Point Cloud Library provides point cloud compression functionality. It allows for encoding all kinds of point clouds including "unorganized" point clouds that are characterized by 
non-existing point references, varying point size, resolution, density and/or point ordering. Furthermore, the underlying octree data structure 
enables to efficiently merge point cloud data from several sources. 


|octreeCompression|

     .. |octreeCompression| image:: images/compression_tutorial.png

In the following, we explain how single point clouds as well
as streams of points clouds can be efficiently compressed. 
In the presented example, we capture point clouds with the OpenNIGrabber to be compressed using the PCL point cloud compression techniques.


The code:
---------

First, create a file, let's say, ``point_cloud_compression.cpp`` and place the following inside it:


.. code-block:: cpp

	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	#include <pcl/io/openni_grabber.h>
	#include <pcl/visualization/cloud_viewer.h>
	
	#include "pcl/compression/octree_pointcloud_compression.h"
	
	#include <stdio.h>
	#include <sstream>
	#include <stdlib.h>
	
	using namespace std;
	using namespace pcl;
	using namespace pcl::octree;
	
	class SimpleOpenNIViewer
	{
	public:
	  SimpleOpenNIViewer () :
	    viewer (" Point Cloud Compression Example")
	  {
	  }
	
	  void
	  cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
	  {
	    if (!viewer.wasStopped ())
	    {
	      // stringstream to store compressed point cloud
	      std::stringstream compressedData;
	      // output pointcloud
	      PointCloud<PointXYZRGB>::Ptr cloudOut (new PointCloud<PointXYZRGB> ());
	
	      // compress point cloud
	      PointCloudEncoder->encodePointCloud (cloud, compressedData);
	
	      // decompress point cloud
	      PointCloudDecoder->decodePointCloud (compressedData, cloudOut);
	
	      // show decompressed point cloud
	      viewer.showCloud (cloudOut);
	    }
	  }
	
	  void
	  run ()
	  {
	
	    bool showStatistics = true;
	
	    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
	    compression_Profiles_e compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;
	
	    // instantiate point cloud compression for encoding and decoding
	    PointCloudEncoder = new PointCloudCompression<PointXYZRGB> (compressionProfile, showStatistics);
	    PointCloudDecoder = new PointCloudCompression<PointXYZRGB> ();
	
	    // create a new grabber for OpenNI devices
	    pcl::Grabber* interface = new pcl::OpenNIGrabber ();
	
	    // make callback function from member function
	    boost::function<void
	    (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
	
	    // connect callback function for desired signal. In this case its a point cloud with color values
	    boost::signals2::connection c = interface->registerCallback (f);
	
	    // start receiving point clouds
	    interface->start ();
	
	    while (!viewer.wasStopped ())
	    {
	      sleep (1);
	    }
	
	    interface->stop ();
	
	    // delete point cloud compression instances
	    delete (PointCloudEncoder);
	    delete (PointCloudDecoder);
	
	  }
	
	  pcl::visualization::CloudViewer viewer;
	
	  PointCloudCompression<PointXYZRGB>* PointCloudEncoder;
	  PointCloudCompression<PointXYZRGB>* PointCloudDecoder;
	
	};
	
	int
	main (int argc, char **argv)
	{
	
	  SimpleOpenNIViewer v;
	  v.run ();
	
	  return 0;
	}


The explanation
---------------

Now, let's discuss the code in detail. Let's start at the main() function: First we create a new SimpleOpenNIViewer instance and call its run() method. 

.. code-block:: cpp

	int
	main (int argc, char **argv)
	{
	
	  SimpleOpenNIViewer v;
	  v.run ();
	
	  return 0;
	}


In the run() function, we create instances of the PointCloudCompression class for encoding and decoding.
They can take compression profiles as an arguments for configuring the compression algorithm. The provided compression profiles predefine 
common parameter sets for point clouds captured by openNI devices. In this example, we use the **MED_RES_ONLINE_COMPRESSION_WITH_COLOR** profile which 
applies a coordinate encoding precision of 5 cubic millimeter and enables color component encoding. It is further optimized for fast online compression. 
A full list of compression profiles including their configuration can be found in the file 
"/io/include/pcl/compression/compression_profiles.h". 
A full parametrization of the compression algorithm is also possible in the PointCloudCompression constructor using the MANUAL_CONFIGURATION profile. 
For further details on advanced parametrization, please have a look at section "Advanced Parametrization".

.. code-block:: cpp

    bool showStatistics = true;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    compression_Profiles_e compressionProfile = pcl::octree::MED_RES_ONLINE_COMPRESSION_WITH_COLOR;

    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = new PointCloudCompression<PointXYZRGB> (compressionProfile, showStatistics);
    PointCloudDecoder = new PointCloudCompression<PointXYZRGB> ();
	    
The following code instantiates a new grabber for an OpenNI device and starts the interface callback loop. 
	    
.. code-block:: cpp	 
   
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber ();

    // make callback function from member function
    boost::function<void
    (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    while (!viewer.wasStopped ())
    {
      sleep (1);
    }

    interface->stop ();	    
	    
	    
In the callback function executed by the OpenNIGrabber capture loop, we first compress the captured point cloud into a stringstream buffer. That follows a
decompression step, which decodes the compressed binary data into a new point cloud object. The decoded point cloud is then sent to the point cloud viewer.
 
.. code-block:: cpp	

  void
  cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
  {
    if (!viewer.wasStopped ())
    {
      // stringstream to store compressed point cloud
      std::stringstream compressedData;
      // output pointcloud
      PointCloud<PointXYZRGB>::Ptr cloudOut (new PointCloud<PointXYZRGB> ());

      // compress point cloud
      PointCloudEncoder->encodePointCloud (cloud, compressedData);

      // decompress point cloud
      PointCloudDecoder->decodePointCloud (compressedData, cloudOut);

      // show decompressed point cloud
      viewer.showCloud (cloudOut);
    }
  }


Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. code-block:: cmake
   
   add_executable (point_cloud_compression  point_cloud_compression.cpp)
   target_link_libraries(point_cloud_compression pcl_common pcl_io  pcl_octree  pcl_visualization )   

After you have made the executable, you can run it. Simply do::

  $ ./point_cloud_compression

You will see something similar to::

	[OpenNIGrabber] Number devices connected: 1
	[OpenNIGrabber] 1. device on bus 002:17 is a Xbox NUI Camera (2ae) from Microsoft (45e) with serial id 'B00364707960044B'
	[OpenNIGrabber] device_id is not set or has unknown format: ! Using first device.
	[OpenNIGrabber] Opened 'Xbox NUI Camera' on bus 2:17 with serial number 'B00364707960044B'
	streams alive:  image,  depth_image
	*** POINTCLOUD ENCODING ***
	Frame ID: 1
	Encoding Frame: Intra frame
	Number of encoded points: 192721
	XYZ compression percentage: 3.91049%
	XYZ bytes per point: 0.469259 bytes
	Color compression percentage: 15.4717%
	Color bytes per point: 0.618869 bytes
	Size of uncompressed point cloud: 3011.27 kBytes
	Size of compressed point cloud: 204 kBytes
	Total bytes per point: 1.08813 bytes
	Total compression percentage: 6.8008%
	Compression ratio: 14.7042
	
	*** POINTCLOUD ENCODING ***
	Frame ID: 2
	Encoding Frame: Prediction frame
	Number of encoded points: 192721
	XYZ compression percentage: 3.8132%
	XYZ bytes per point: 0.457584 bytes
	Color compression percentage: 15.5448%
	Color bytes per point: 0.62179 bytes
	Size of uncompressed point cloud: 3011.27 kBytes
	Size of compressed point cloud: 203 kBytes
	Total bytes per point: 1.07937 bytes
	Total compression percentage: 6.74609%
	Compression ratio: 14.8234
	
	*** POINTCLOUD ENCODING ***
	Frame ID: 3
	Encoding Frame: Prediction frame
	Number of encoded points: 192721
	XYZ compression percentage: 3.79962%
	XYZ bytes per point: 0.455954 bytes
	Color compression percentage: 15.2121%
	Color bytes per point: 0.608486 bytes
	Size of uncompressed point cloud: 3011.27 kBytes
	Size of compressed point cloud: 200 kBytes
	Total bytes per point: 1.06444 bytes
	Total compression percentage: 6.65275%
	Compression ratio: 15.0314
	
	...


Compression Profiles:
--------------------------------------------------------
Compression profiles define parameter sets for the PCL point cloud encoder. They are optimized for compression of 
common point clouds retrieved from the OpenNI grabber.
Please note, that the decoder does not need to be parametrized as it detects and adopts the configuration used during encoding.  
The following compression profiles are available:

	- **LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR** 1 cubic centimeter resolution, no color, fast online encoding
	
	- **LOW_RES_ONLINE_COMPRESSION_WITH_COLOR** 1 cubic centimeter resolution, color, fast online encoding
	
	- **MED_RES_ONLINE_COMPRESSION_WITHOUT_COLOR** 5 cubic milimeter resolution, no color, fast online encoding
	
	- **MED_RES_ONLINE_COMPRESSION_WITH_COLOR** 5 cubic milimeter resolution, color, fast online encoding
	
	- **HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR** 1 cubic milimeter resolution, no color, fast online encoding
	
	- **HIGH_RES_ONLINE_COMPRESSION_WITH_COLOR** 1 cubic milimeter resolution, color, fast online encoding

	- **LOW_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR** 1 cubic centimeter resolution, no color, efficient offline encoding
	
	- **LOW_RES_OFFLINE_COMPRESSION_WITH_COLOR** 1 cubic centimeter resolution, color, efficient offline encoding
	
	- **MED_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR** 5 cubic milimeter resolution, no color, efficient offline encoding
	
	- **MED_RES_OFFLINE_COMPRESSION_WITH_COLOR** 5 cubic milimeter resolution, color, efficient offline encoding
	
	- **HIGH_RES_OFFLINE_COMPRESSION_WITHOUT_COLOR** 1 cubic milimeter resolution, no color, efficient offline encoding
	
	- **HIGH_RES_OFFLINE_COMPRESSION_WITH_COLOR** 1 cubic milimeter resolution, color, efficient offline encoding
	
	- **MANUAL_CONFIGURATION** enables manual configuration for advanced parametrization
 

Advanced parametrization:
--------------------------------------------------------	

In order to have full access to all compression related parameters, the constructor of the PointCloudCompression class can initialized with additional 
compression parameters. Please note, that for enabling advanced parametrization, the compressionProfile_arg argument **needs** to be set to **MANUAL_CONFIGURATION**. 

.. code-block:: cpp

        PointCloudCompression (compression_Profiles_e compressionProfile_arg,
                               bool showStatistics_arg,
                               const double pointResolution_arg,
                               const double octreeResolution_arg,
                               bool doVoxelGridDownDownSampling_arg,
                               const unsigned int iFrameRate_arg,
                               bool doColorEncoding_arg,
                               const unsigned char colorBitResolution_arg
                               ) 
                               

The advanced parametrization is explained in the following: 

	- **compressionProfile_arg**: This parameter should be set to **MANUAL_CONFIGURATION** for enabling advanced parametrization.
	
	- **showStatistics_arg**: Print compression related statistics to stdout.
	
	- **pointResolution_arg**: Define coding precision for point coordinates. This parameter should be set to a value below the sensor noise. 
	
	- **octreeResolution_arg**: This parameter defines the voxel size of the deployed octree. A lower voxel resolution enables faster compression at, however, 
	  decreased compression performance. This enables a trade-off between high frame/update rates and compression efficiency.
	  
	- **doVoxelGridDownDownSampling_arg**: If activated, only the hierarchical octree data structure is encoded. The decoder generated points at the voxel centers. In this
	  way, the point cloud becomes downsampled during compression while archieving high compression performance. 
	  
	- **iFrameRate_arg**: The point cloud compression scheme differentially encodes point clouds.  In this way, differences between the incoming point cloud and the previously encoded pointcloud is encoded in order to archive maximum compression performance. The iFrameRate_arg allows to specify the rate of frames in the stream at which incoming point clouds are **not** differentially encoded (similar to I/P-frames in video coding).   
	    
	- **doColorEncoding_arg**: This option enables color component encoding.   	 
	
	- **colorBitResolution_arg**: This parameter defines the amount of bits per color component to be encoded. 

Command line tool for PCL point cloud stream compression
--------------------------------------------------------

The pcl visualization component contains a command line tool for point cloud compression and streaming:
Simply execute "./pcl_stream_compression -?" to see a full list of options::


  PCL point cloud stream compression

  usage: ./pcl_stream_compression [mode] [profile] [parameters]

  I/O: 
      -f file  : file name 

  file compression mode:
      -x: encode point cloud stream to file
      -d: decode from file and display point cloud stream

  network streaming mode:
      -s       : start server on localhost
      -c host  : connect to server and display decoded cloud stream

  optional compression profile: 
      -p profile : select compression profile:       
                     -"lowC"  Low resolution with color
                     -"lowNC" Low resolution without color
                     -"medC" Medium resolution with color
                     -"medNC" Medium resolution without color
                     -"highC" High resolution with color
                     -"highNC" High resolution without color

  optional compression parameters:
      -r prec  : point precision
      -o prec  : octree voxel size
      -v       : enable voxel-grid downsampling
      -a       : enable color coding
      -i rate  : i-frame rate
      -b bits  : bits/color component
      -t       : output statistics
      -e       : show input cloud during encoding

  example:
      ./pcl_stream_compression -x -p highC -t -f pc_compressed.pcc 

In order to stream compressed point cloud via TCP/IP, you can start the server with::

  $ ./point_cloud_compression -s
     
It will listen on port 6666 for incoming connections. Now start the client with::     

  $ ./point_cloud_compression -c SERVER_NAME
  
and remotely captured point clouds will be locally shown in the point cloud viewer.  
     
Conclusion
----------

This PCL point cloud compression enables to efficiently compress point clouds of any type and point cloud streams.      





     




