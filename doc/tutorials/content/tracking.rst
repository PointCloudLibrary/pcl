.. _tracking:

Tracking object in real time
----------------------------

The pcl_tracking library contains data structures and mechanism for 3D tracking which uses Particle Filter Algorithm. This tracking will enable you to implement 6D-pose (position and rotation) tracking which is optimized to run in real time.  In order to make reference model, the tracker at first separates point cloud of the target object from others. Then it starts tracking the object with Particles at each loop of function.


At each loop, tracking program proceeds along with following algorythm.
	1. (At  t = t - 1) At first, using previous Pariticle's information about position and rotation, it will predict each position and rotation of them at the next frame.


	2. Next, we calculate weights of those particles with the likelihood formula below.(you can select whick likelihood function you use)


	3. Finally, we use the evaluate function which compares real point cloud data from depth sensor  with the predicted particles, and resample particles.

.. math::

	L_j = L_distance ( \times L_color )

	w = \sum_ L_j



.. image:: images/tracking/slideCapture.png
  :height: 400

Following figure shows how  looks like when trakcing works successfully.

.. image:: images/tracking/mergePicture.png
  :height: 600

The code
--------

Create two files,  paste following code with your editor and save each file as segment_reference.h, segment_reference.cpp and tracking_sample.cpp.

segment_reference.h

.. literalinclude:: sources/tracking/segment_reference.h
   :language: cpp
   :linenos:

segment_reference.cpp

.. literalinclude:: sources/tracking/segment_reference.cpp
   :language: cpp
   :linenos:

tracking_sample.cpp

.. literalinclude:: sources/tracking/tracking_sample.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece. We focus on openni_tracking_sample.cpp which will teach you how to use tracking library. segment_reference.cpp is just for extracting tracking object from other point cloud and planes.

.. code-block:: cpp
   :linenos:

	//Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
	tracker->setMaximumParticleNum (500);
	tracker->setDelta (0.99);
	tracker->setEpsilon (0.2);
	tracker->setBinSize (bin_size);

	//Set all parameters for  ParticleFilter
	tracker_ = tracker;
	tracker_->setTrans (Eigen::Affine3f::Identity ());
	tracker_->setStepNoiseCovariance (default_step_covariance);
	tracker_->setInitialNoiseCovariance (initial_noise_covariance);
	tracker_->setInitialNoiseMean (default_initial_mean);
	tracker_->setIterationNum (1);
	tracker_->setParticleNum (400);
	tracker_->setResampleLikelihoodThr(0.00);
	tracker_->setUseNormal (false);

First, in main function, these lines set the parameters for tracking. 

.. code-block:: cpp
   :linenos:

	//Setup coherence object for tracking
	ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence =  ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr(new ApproxNearestPairPointCloudCoherence<RefPointType> ());
    
	boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
    = boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
    coherence->addPointCoherence (distance_coherence);

	boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
	coherence->setSearchMethod (search);
	coherence->setMaximumDistance (0.01);

	tracker_->setCloudCoherence (coherence);

Here, we set likelihood function which tracker use when calculate weights. You can add more likelihood function as you like. By default, there are normals likelihood and color likelihood functions.

.. code-block:: cpp
   :linenos:

	if (counter_ < 10)
	{
	  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled\_, downsampling_grid_size_);
	}
	//Make model reference and set it to tracker_
	else if (counter_ == 10)
	{
	  cloud_pass_downsampled_ = cloud_pass_;
	  CloudPtr target_cloud;

	  PCL_INFO ("segmentation, please wait...\n");
	  //remove Plane's point cloud
	  extractPlanes(target_cloud);

	  if (target_cloud != NULL)
	  {
	    Eigen::Vector4f c;
	    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
	    CloudPtr transed_ref_downsampled (new Cloud);
					
	    //Make model reference
	    initialize_segemnted_reference(target_cloud, transed_ref_downsampled, trans);
					
	    tracker_->setReferenceCloud (transed_ref_downsampled);
	    tracker_->setTrans (trans);
	  }
	  PCL_INFO ("segmentation Complete!\n");
	  PCL_INFO ("Start tracking\n");
	}
 	//Track the object
	else
	{
	  gridSampleApprox (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);
	  tracker_->setInputCloud (cloud_pass_downsampled_);
	  tracker_->compute ();
	}

Until the counter\_ variable become equal to 10, we ignore the input point cloud, because the point cloud at first few frames often have noise. At 10 frame, in order to make reference point cloud, we remove plane's point cloud from other, downsample  and extract the segment of model. We give the model segment and segment's centroid  to tracker\_. At each loop, we set input point cloud to tracker and the tracker will compute particles movement. 

	ParticleFilter::PointCloudStatePtr particles = tracker\_->getParticles ();

Finallly, in drawParticles function, you can get particles's positions by calling getParticles().


Compiling and running the program
---------------------------------

Create a CmakeLists.txt file and add the following lines into it.

.. literalinclude:: sources/tracking/CMakeLists.txt
   :language: cmake
   :linenos:


After you created the executable, you can then launch it. 
Before you start tracking, we recommend that you should put the object you want to track in the center of screen on a plane where there is nothing without the object.(Please specify the device_id as second argument.):

	$ ./openni_tracking sample “#1”

After few seconds, you may see “segmentation Complete!” in terminal and you can move tracking object around. As you can see in following pictures, the blue point cloud is reference model segmentation's cloud and the red one is particles' cloud.

.. image:: images/tracking/redone.png
  :height: 400

.. image:: images/tracking/blueone.png
  :height: 400























