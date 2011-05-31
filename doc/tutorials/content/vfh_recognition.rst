.. _vfh_recognition:

Cluster Recognition and 6DOF Pose Estimation using VFH descriptors
------------------------------------------------------------------

As previously described in :ref:`vfh_estimation`, Viewpoint Feature Histograms
(VFH) are powerful *meta-local* descriptors, created for the purpose of
recognition and pose estimation for **clusters** of points. We here refer to a
**cluster** as a collection of 3D points, most of the time representing a
particular object or part of a scene, obtained through some segmentation or
detection mechanisms (please see :ref:`cluster_extraction` for an example).

Our goal here is not to provide an ultimate recognition tool, but rather a
mechanism for obtaining **candidates** that *could potentially be the
cluster/object that is searched for*, together with its 6DOF pose in space.
With this in mind, we will be formulating the *recognition* problem as a
*nearest neighbor estimation* problem. So given a set of *training data*, we
will use efficient nearest neighbor search structures such as *kd-trees* and
return a set of potential candidates with sorted distances to the query object,
rather than an absolute *"this is the object that we were searching for"* kind
of response. The reader can imagine that such a system becomes much more useful
as we can explicitly reason about failures (false positives, or true
negatives).

For the purpose of this tutorial, the application example could be formulated as follows:

 * Training stage:

   * given a scene with 1 object that is easily separable as a cluster;

   * use a ground-truth system to obtain its pose (see the discussion below);

   * rotate around the object or rotate the object with respect to the camera, and compute a VFH descriptor for each view;

   * store the views, and build a kd-tree representation.


 * Testing stage:

   * given a scene with objects that can be separated as individual clusters, first extract the clusters;

   * for each cluster, compute a VFH descriptor from the current camera position;

   * use the VFH descriptor to search for candidates in the trained kd-tree.


We hope the above makes sense. Basically we're first going to create the set of
objects that we try to later on recognize, and then we will use that to obtain
valid candidates for objects in the scene.

A good example of a ground-truth system could be a simple rotating pan-tilt
unit such as the one in the figure below. Placing an object on the unit, and
moving it with some increments in both horizontal and vertical, can result in a
perfect ground-truth system for small objects. A cheaper solution could be to
use a marker-based system (e.g., checkerboard) and rotate the camera/table
manually.

.. image:: images/vfh_recognition/pan_tilt.jpg
   :align: center


Our Kd-Tree implementation of choice for the purpose of this tutorial is of
course, `FLANN <http://www.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN>`_.


Training
--------

We begin the training by assuming that the *objects* are already separated as
individual clusters (see :ref:`cluster_extraction`), as shown in the figure
below:

.. image:: images/vfh_recognition/scene_raw.png

.. image:: images/vfh_recognition/scene_segmented.png


Since we're only trying to cover the explicity training/testing of VFH
signatures in this tutorial, we provide a set of datasets already collected at:
`vfh_recognition_tutorial_data.tbz
<http://dev.pointclouds.org/attachments/download/216/vfh_recognition_tutorial_data.tbz>`_.
The data is a subset of the objects presented in the figure below (left), and
look like the point clouds on the right. We used the pan-tilt table shown above
to acquire the data.

.. image:: images/vfh_recognition/objects.jpg

.. image:: images/vfh_recognition/training.png

Next, copy and paste the following code into your editor and save it as
``build_tree.cpp``.

.. code-block:: cpp
   :linenos:

    #include <pcl/point_types.h>
    #include <pcl/point_cloud.h>
    #include <pcl/console/parse.h>
    #include <pcl/console/print.h>
    #include <pcl/io/pcd_io.h>
    #include <boost/filesystem.hpp>
    #include <flann/flann.h>
    #include <flann/io/hdf5.h>
    #include <fstream>

    using namespace pcl;
    using namespace pcl::console;

    typedef std::pair<std::string, std::vector<float> > vfh_model;

    /** \brief Loads an n-D histogram file as a VFH signature
      * \param path the input file name
      * \param vfh the resultant VFH model
      */
    bool
    loadHist (const boost::filesystem::path &path, vfh_model &vfh)
    {
      using namespace std;

      int vfh_idx;
      // Load the file as a PCD
      try
      {
        sensor_msgs::PointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        PCDReader r;
        bool binary; int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, binary, idx);

        vfh_idx = getFieldIndex (cloud, "vfh");
        if (vfh_idx == -1)
          return (false);
        if ((int)cloud.width * cloud.height != 1)
          return (false);
      }
      catch (pcl::InvalidConversionException e)
      {
        return (false);
      }

      // Treat the VFH signature as a single Point Cloud
      PointCloud <VFHSignature308> point;
      io::loadPCDFile (path.string (), point);
      vfh.second.resize (308);

      std::vector <sensor_msgs::PointField> fields;
      getFieldIndex (point, "vfh", fields);

      for (size_t i = 0; i < fields[vfh_idx].count; ++i)
      {
        vfh.second[i] = point.points[0].histogram[i];
      }
      vfh.first = path.string ();
      return (true);
    }

    /** \brief Load a set of VFH features that will act as the model (training data)
      * \param argc the number of arguments (pass from main ())
      * \param argv the actual command line arguments (pass from main ())
      * \param extension the file extension containing the VFH features
      * \param models the resultant vector of histogram models
      */
    void
    loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, 
                       std::vector<vfh_model> &models)
    {
      if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
        return;

      for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
      {
        if (boost::filesystem::is_directory (it->status ()))
        {
          std::stringstream ss;
          ss << it->path ();
          print_highlight ("Loading %s (%lu models loaded so far).\n", ss.str ().c_str (), (unsigned long)models.size ());
          loadFeatureModels (it->path (), extension, models);
        }
        if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
        {
          vfh_model m;
          if (loadHist (base_dir / it->path ().filename (), m))
            models.push_back (m);
        }
      }
    }

    int
    main (int argc, char** argv)
    {
      if (argc < 2)
      {
        PCL_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options]\n", argv[0]);
        return (-1);
      }

      std::string extension (".pcd");
      transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

      std::string kdtree_idx_file_name = "kdtree.idx";
      std::string training_data_h5_file_name = "training_data.h5";
      std::string training_data_list_file_name = "training_data.list";

      std::vector<vfh_model> models;
      flann::Matrix<float> data;

      // Load the model histograms
      loadFeatureModels (argv[1], extension, models);
      print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n", (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());

      // Convert data into FLANN format
      data.rows = models.size ();
      data.cols = models[0].second.size (); // number of histogram bins
      data.data = (float*)malloc (data.rows * data.cols * sizeof (float)); 

      for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
          data.data[i * data.cols  + j] = models[i].second[j];

      // Save data to disk (list of models)
      flann::save_to_file (data, training_data_h5_file_name, "training_data");
      std::ofstream fs;
      fs.open (training_data_list_file_name.c_str ());
      for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
      fs.close ();
     
      // Build the tree index and save it to disk
      print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
      flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
      //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
      index.buildIndex ();
      index.save (kdtree_idx_file_name);

      return (0);
    }

In the following paragraphs we will explain what the above code does (or should
do). We'll begin with the ``main`` function.

We begin by loading a set of feature models from a directory given as the first
command line argument (see details for running the example below). The
``loadFeatureModels`` method does nothing but recursively traverse a set of
directories and subdirectories, and loads in all *.PCD* files it finds. In
``loadFeatureModels``, we call ``loadHist``, which will attempt to open each
PCD file found, read its header, and check whether it contains a VFH signature
or not. Together with the VFH signature we also store the PCD file name into a
``vfh_model`` pair.  

Once all VFH features have been loaded, we convert them to FLANN format, using:

.. code-block:: cpp

      // Convert data into FLANN format
      data.rows = models.size ();
      data.cols = models[0].second.size (); // number of histogram bins
      data.data = (float*)malloc (data.rows * data.cols * sizeof (float)); 

      for (size_t i = 0; i < data.rows; ++i)
        for (size_t j = 0; j < data.cols; ++j)
          data.data[i * data.cols  + j] = models[i].second[j];

Since we're lazy, and we want to use this data (and not reload it again by crawling the directory structure in the testing phase), we dump the data to disk:

.. code-block:: cpp

      // Save data to disk (list of models)
      flann::save_to_file (data, training_data_h5_file_name, "training_data");
      std::ofstream fs;
      fs.open (training_data_list_file_name.c_str ());
      for (size_t i = 0; i < models.size (); ++i)
        fs << models[i].first << "\n";
      fs.close ();

Finally, we create the KdTree, and save its structure to disk:

.. code-block:: cpp
   :linenos:

      flann::Index<flann::ChiSquareDistance<float> > index (data, flann::LinearIndexParams ());
      //flann::Index<flann::ChiSquareDistance<float> > index (data, flann::KDTreeIndexParams (4));
      index.buildIndex ();
      index.save (kdtree_idx_file_name);

Here we will use a ``LinearIndex``, which does a brute-force search using a
Chi-Square distance metric (see [VFH]_ for more information). For building a
proper kd-tree, comment line 1 and uncomment line 2 in the code snippet above.
The most important difference between a LinearIndex and a KDTreeIndex in FLANN
is that the KDTree will be much faster, while producing approximate nearest
neighbor results, rather than absolute.

So, we're done with training. To summarize:

  1. we crawled a directory structure, looked at all the .PCD files we found, tested them whether they are VFH signatures and loaded them in memory;

  2. we converted the data into FLANN format and dumped it to disk;

  3. we built a kd-tree structure and dumped it to disk.

Testing
-------

In the testing phase, we will illustrate how the system works by randomly
loading one of the files used in the training phase (feel free to supply your
own file here!), and checking the results of the tree.

Begin by copying and pasting the following code into your editor and save it as
``nearest_neighbors.cpp``.

.. code-block:: cpp
   :linenos:

    #include <pcl/point_types.h>
    #include <pcl/point_cloud.h>
    #include <pcl/common/common.h>
    #include <pcl/registration/transforms.h>
    #include <pcl/visualization/pcl_visualizer.h>
    #include <pcl/console/parse.h>
    #include <pcl/console/print.h>
    #include <pcl/io/pcd_io.h>
    #include <iostream>
    #include <flann/flann.h>
    #include <flann/io/hdf5.h>
    #include <boost/filesystem.hpp>

    using namespace std;
    using namespace pcl;
    using namespace pcl::console;
    using namespace pcl::visualization;

    typedef std::pair<std::string, std::vector<float> > vfh_model;

    /** \brief Loads an n-D histogram file as a VFH signature
      * \param path the input file name
      * \param vfh the resultant VFH model
      */
    bool
    loadHist (const boost::filesystem::path &path, vfh_model &vfh)
    {
      using namespace std;

      int vfh_idx;
      // Load the file as a PCD
      try
      {
        sensor_msgs::PointCloud2 cloud;
        int version;
        Eigen::Vector4f origin;
        Eigen::Quaternionf orientation;
        PCDReader r;
        bool binary; int idx;
        r.readHeader (path.string (), cloud, origin, orientation, version, binary, idx);

        vfh_idx = getFieldIndex (cloud, "vfh");
        if (vfh_idx == -1)
          return (false);
        if ((int)cloud.width * cloud.height != 1)
          return (false);
      }
      catch (pcl::InvalidConversionException e)
      {
        return (false);
      }

      // Treat the VFH signature as a single Point Cloud
      PointCloud <VFHSignature308> point;
      io::loadPCDFile (path.string (), point);
      vfh.second.resize (308);

      std::vector <sensor_msgs::PointField> fields;
      getFieldIndex (point, "vfh", fields);

      for (size_t i = 0; i < fields[vfh_idx].count; ++i)
      {
        vfh.second[i] = point.points[0].histogram[i];
      }
      vfh.first = path.string ();
      return (true);
    }


    /** \brief Search for the closest k neighbors
      * \param index the tree
      * \param model the query model
      * \param k the number of neighbors to search for
      * \param indices the resultant neighbor indices
      * \param distances the resultant neighbor distances
      */
    inline void
    nearestKSearch (flann::Index<flann::ChiSquareDistance<float> > &index, const vfh_model &model, 
                    int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances)
    {
      // Query point
      flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
      memcpy (&p.data[0], &model.second[0], p.cols * p.rows * sizeof (float));

      indices = flann::Matrix<int>(new int[k], 1, k);
      distances = flann::Matrix<float>(new float[k], 1, k);
      index.knnSearch (p, indices, distances, k, flann::SearchParams (512));
      p.free();
    }

    /** \brief Load the list of file model names from an ASCII file
      * \param models the resultant list of model name
      * \param filename the input file name
      */
    bool
    loadFileList (std::vector<vfh_model> &models, const std::string &filename)
    {
      ifstream fs;
      fs.open (filename.c_str ());
      if (!fs.is_open () || fs.fail ())
        return (false);

      std::string line;
      while (!fs.eof ())
      {
        getline (fs, line);
        if (line.empty ())
          continue;
        vfh_model m;
        m.first = line;
        models.push_back (m);
      }
      fs.close ();
      return (true);
    }

    int
    main (int argc, char** argv)
    {
      int k = 6;

      double thresh = DBL_MAX;     // No threshold, disabled by default

      if (argc < 2)
      {
        print_error ("Need at least three parameters! Syntax is: %s <query_vfh_model.pcd> [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]);
        print_info ("    where [options] are:  -k      = number of nearest neighbors to search for in the tree (default: "); print_value ("%d", k); print_info (")\n");
        print_info ("                          -thresh = maximum distance threshold for a model to be considered VALID (default: "); print_value ("%f", thresh); print_info (")\n\n");
        return (-1);
      }

      // this won't be needed for flann > 1.6.10
      flann::ObjectFactory<flann::IndexParams, flann_algorithm_t>::instance().register_<flann::LinearIndexParams>(FLANN_INDEX_LINEAR);

      string extension (".pcd");
      transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

      // Load the test histogram
      vector<int> pcd_indices = parse_file_extension_argument (argc, argv, ".pcd");
      vfh_model histogram;
      if (!loadHist (argv[pcd_indices.at (0)], histogram))
      {
        print_error ("Cannot load test file %s\n", argv[pcd_indices.at (0)]);
        return (-1);
      }

      parse_argument (argc, argv, "-thresh", thresh);
      // Search for the k closest matches
      parse_argument (argc, argv, "-k", k);
      print_highlight ("Using "); print_value ("%d", k); print_info (" nearest neighbors.\n");

      string kdtree_idx_file_name = "kdtree.idx";
      string training_data_h5_file_name = "training_data.h5";
      string training_data_list_file_name = "training_data.list";

      vector<vfh_model> models;
      flann::Matrix<int> k_indices;
      flann::Matrix<float> k_distances;
      flann::Matrix<float> data;
      // Check if the data has already been saved to disk
      if (!boost::filesystem::exists ("training_data.h5") || !boost::filesystem::exists ("training_data.list"))
      {
        print_error ("Could not find training data models files %s and %s!\n", training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
        return (-1);
      }
      else
      {
        loadFileList (models, training_data_list_file_name);
        flann::load_from_file (data, training_data_h5_file_name, "training_data");
        print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
      }

      // Check if the tree index has already been saved to disk
      if (!boost::filesystem::exists (kdtree_idx_file_name))
      {
        print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
        return (-1);
      }
      else
      {
        flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
        index.buildIndex ();
        nearestKSearch (index, histogram, k, k_indices, k_distances);
      }

      // Output the results on screen
      print_highlight ("The closest %d neighbors for %s are:\n", k, argv[pcd_indices[0]]);
      for (int i = 0; i < k; ++i)
        print_info ("    %d - %s (%d) with a distance of: %f\n", i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);

      // Load the results
      PCLVisualizer p (argc, argv, "VFH Cluster Classifier");
      int y_s = (int)floor (sqrt ((double)k));
      int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
      double x_step = (double)(1 / (double)x_s);
      double y_step = (double)(1 / (double)y_s);
      print_highlight ("Preparing to load "); print_value ("%d", k); print_info (" files ("); 
      print_value ("%d", x_s);    print_info ("x"); print_value ("%d", y_s); print_info (" / ");
      print_value ("%f", x_step); print_info ("x"); print_value ("%f", y_step); print_info (")\n");

      int viewport = 0, l = 0, m = 0;
      for (int i = 0; i < k; ++i)
      {
        string cloud_name = models.at (k_indices[0][i]).first;
        boost::replace_last (cloud_name, "_vfh", "");

        p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);
        l++;
        if (l >= x_s)
        {
          l = 0;
          m++;
        }

        sensor_msgs::PointCloud2 cloud;
        print_highlight (stderr, "Loading "); print_value (stderr, "%s ", cloud_name.c_str ());
        if (pcl::io::loadPCDFile (cloud_name, cloud) == -1)
          break;

        // Convert from blob to PointCloud
        PointCloud<PointXYZ> cloud_xyz;
        fromROSMsg (cloud, cloud_xyz);

        if (cloud_xyz.points.size () == 0)
          break;

        print_info ("[done, "); print_value ("%d", (int)cloud_xyz.points.size ()); print_info (" points]\n");
        print_info ("Available dimensions: "); print_value ("%s\n", getFieldsList (cloud).c_str ());

        // Demean the cloud
        Eigen::Vector4f centroid;
        compute3DCentroid (cloud_xyz, centroid);
        PointCloud<PointXYZ>::Ptr cloud_xyz_demean (new PointCloud<PointXYZ>);
        demeanPointCloud<PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
        // Add to renderer*
        p.addPointCloud (cloud_xyz_demean, cloud_name, viewport);
        
        // Check if the model found is within our inlier tolerance
        stringstream ss;
        ss << k_distances[0][i];
        if (k_distances[0][i] > thresh)
        {
          p.addText (ss.str (), 20, 30, 1, 0, 0, ss.str (), viewport);  // display the text with red

          // Create a red line
          PointXYZ min_p, max_p;
          pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p);
          stringstream line_name;
          line_name << "line_" << i;
          p.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport);
          p.setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport);
        }
        else
          p.addText (ss.str (), 20, 30, 0, 1, 0, ss.str (), viewport);

        // Increase the font size for the score*
        p.setShapeRenderingProperties (PCL_VISUALIZER_FONT_SIZE, 18, ss.str (), viewport);

        // Add the cluster name
        p.addText (cloud_name, 20, 10, cloud_name, viewport);
      }
      // Add coordianate systems to all viewports
      p.addCoordinateSystem (0.1, 0);

      p.spin ();
      return (0);
    }


The above code snippet is slightly larger, because we also included some
visualization routines and some other "eye candy" stuff.

In lines:

.. code-block:: cpp

      vector<int> pcd_indices = parse_file_extension_argument (argc, argv, ".pcd");
      vfh_model histogram;
      if (!loadHist (argv[pcd_indices.at (0)], histogram))
      {
        print_error ("Cannot load test file %s\n", argv[pcd_indices.at (0)]);
        return (-1);
      }

we load the first given user histogram (and ignore the rest). Then we proceed
at checking two command line parameters, namely ``-k`` which will define how
many nearest neighbors to check and display on screen, and ``-thresh`` which
defines a maximum distance metric after which we will start displaying red
lines (i.e., crossing) over the **k** models found on screen (eye candy!).

In lines:

.. code-block:: cpp

        loadFileList (models, training_data_list_file_name);
        flann::load_from_file (data, training_data_h5_file_name, "training_data");

we load the training data from disk, together with the list of file names that
we previously stored in ``build_tree.cpp``. Then, we read the kd-tree and rebuild the index:

.. code-block:: cpp

        flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams ("kdtree.idx"));
        index.buildIndex ();

Here we need to make sure that we use the **exact** distance metric
(``ChiSquareDistance`` in this case), as the one that we used while creating
the tree. The most important part of the code comes here:

.. code-block:: cpp

        nearestKSearch (index, histogram, k, k_indices, k_distances);

Inside ``nearestKSearch``, we first convert the query point to FLANN format:

.. code-block:: cpp

      flann::Matrix<float> p = flann::Matrix<float>(new float[model.second.size ()], 1, model.second.size ());
      memcpy (&p.data[0], &model.second[0], p.cols * p.rows * sizeof (float));

Followed by obtaining the resultant nearest neighbor indices and distances for the query in:

.. code-block:: cpp

      indices = flann::Matrix<int>(new int[k], 1, k);
      distances = flann::Matrix<float>(new float[k], 1, k);
      index.knnSearch (p, indices, distances, k, flann::SearchParams (512));

Lines:

.. code-block:: cpp

      PCLVisualizer p (argc, argv, "VFH Cluster Classifier");
      int y_s = (int)floor (sqrt ((double)k));
      int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
      double x_step = (double)(1 / (double)x_s);
      double y_step = (double)(1 / (double)y_s);
 

create a ``PCLVisualizer`` object, and sets up a set of different viewports (e.g., splits the screen into different chunks), which will be enabled in:

.. code-block:: cpp

        p.createViewPort (l * x_step, m * y_step, (l + 1) * x_step, (m + 1) * y_step, viewport);

Using the file names representing the models that we previously obtained in
``loadFileList``, we proceed at loading the model file names using:

.. code-block:: cpp

        sensor_msgs::PointCloud2 cloud;
        ...
        if (pcl::io::loadPCDFile (cloud_name, cloud) == -1)
          break;

        // Convert from blob to PointCloud
        PointCloud<PointXYZ> cloud_xyz;
        fromROSMsg (cloud, cloud_xyz);

For visualization purposes, we demean the point cloud by computing its centroid and then subtracting it:

.. code-block:: cpp

        Eigen::Vector4f centroid;
        compute3DCentroid (cloud_xyz, centroid);
        PointCloud<PointXYZ>::Ptr cloud_xyz_demean (new PointCloud<PointXYZ>);
        demeanPointCloud<PointXYZ> (cloud_xyz, centroid, *cloud_xyz_demean);
        // Add to renderer*
        p.addPointCloud (cloud_xyz_demean, cloud_name, viewport);

Finally we check if the distance obtained by ``nearestKSearch`` is larger than the user given threshold, and if it is, we display a red line over the cloud that is being rendered in the viewport:

.. code-block:: cpp

          // Create a red line
          PointXYZ min_p, max_p;
          pcl::getMinMax3D (*cloud_xyz_demean, min_p, max_p);
          stringstream line_name;
          line_name << "line_" << i;
          p.addLine (min_p, max_p, 1, 0, 0, line_name.str (), viewport);
          p.setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, line_name.str (), viewport);


Compiling and running the code
------------------------------

Create a new ``CMakeLists.txt`` file, and put the following content into it::

  cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
  project(vfh_cluster_classifier)
  list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

  find_package(Boost 1.40.0 COMPONENTS system filesystem thread date_time)
  include_directories(${Boost_INCLUDE_DIRS})

  find_package(PCL 1.0 REQUIRED COMPONENTS common io features visualization)
  include_directories(${PCL_INCLUDE_DIRS})

  find_package(Flann)
  include_directories(${FLANN_INCLUDE_DIRS})
  
  find_package(HDF5)
  include_directories(${HDF5_INCLUDE_DIR})

  add_executable(build_tree src/build_tree.cpp)
  target_link_libraries(build_tree ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY})

  add_executable(nearest_neighbors src/nearest_neighbors.cpp)
  target_link_libraries(nearest_neighbors ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${Boost_LIBRARIES} ${FLANN_LIBRARIES} ${HDF5_hdf5_LIBRARY} vtkCommon vtkRendering)

.. note::
	
	If you are running this tutorial on Windows, you have to install (`HDF5 1.8.7 Shared Library<http://www.hdfgroup.org/ftp/HDF5/hdf5-1.8.7/bin/windows/`_). If CMake is not able to find HDF5, 
	you can manually supply the include directory in HDF5_INCLUDE_DIR variable and the full path of **hdf5dll.lib** in HDF5_hdf5_LIBRARY variable. 
	Make sure that the needed dlls are in the same folder as the executables.

The above assumes that your two source files (``build_tree.cpp`` and ``nearest_neighbors.cpp``) are stored into the *src/* subdirectory.

Then, make sure that the datasets you downloaded (`vfh_recognition_tutorial_data.tbz
<http://dev.pointclouds.org/attachments/download/216/vfh_recognition_tutorial_data.tbz>`_) are unpacked in this directory, thus creating a *data/* subdirectory.

After you have made the executable, you can run them like so::

  $ ./build/build_tree data/

You should see the following output on screen::

  > Loading data/001.324.25 (0 models loaded so far).
  > Loading data/800.919.49 (13 models loaded so far).
  > Loading data/100.922.16 (27 models loaded so far).
  > Loading data/901.125.07 (47 models loaded so far).
  > Loading data/000.580.67 (65 models loaded so far).
  > Loading data/463.156.00 (81 models loaded so far).
  > Loading data/401.431.44 (97 models loaded so far).
  > Loading data/100.919.00 (113 models loaded so far).
  > Loading data/401.324.52 (134 models loaded so far).
  > Loading data/201.327.78 (150 models loaded so far).
  > Loading data/300.151.23 (166 models loaded so far).
  > Loading data/200.921.07 (180 models loaded so far).
  > Loaded 195 VFH models. Creating training data training_data.h5/training_data.list.
  Building the kdtree index (kdtree.idx) for 195 elements...

The above crawled the *data/* subdirectory, and created a kd-tree with 195 entries. To run the nearest neighbor testing example, you have two options:

  1. Either run the following command manually, and select one of the datasets that we provided as a testing sample, like this::

        ./build/nearest_neighbors -k 16 -thresh 50 data/000.580.67/1258730231333_cluster_0_nxyz_vfh.pcd

  2. Or, if you are on a linux system, you can place the following on a bash script file (e.g., ``test.sh``)::

        #!/bin/bash

        # Example directory containing _vfh.pcd files
        DATA=data

        # Inlier distance threshold
        thresh=50

        # Get the closest K nearest neighbors
        k=16

        for i in `find $DATA -type d -name "*"`
        do
          echo $i
          for j in `find $i -type f \( -iname "*cluster*_vfh.pcd" \) | sort -R`
          do
            echo $j
            ./build/nearest_neighbors -k $k -thresh $thresh $j -cam "0.403137,0.868471/0,0,0/-0.0932051,-0.201608,-0.518939/-0.00471487,-0.931831,0.362863/1464,764/6,72"
          done
        done

    and run the script like this::

        bash test.sh

You should see *recognition* examples like the ones shown below:

.. image:: images/vfh_recognition/vfh_example1.jpg
.. image:: images/vfh_recognition/vfh_example2.jpg
.. image:: images/vfh_recognition/vfh_example3.jpg

