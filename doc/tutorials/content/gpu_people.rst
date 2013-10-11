.. _gpu_people:

Detecting people and their poses using PointCloud Library
---------------------------------------------------------
In this tutorial we will learn how detect a person and its pose in a pointcloud. 
This is based on work from Koen Buys, Cedric Cagniart, Anatoly Bashkeev and Caroline Pantofaru, this
has been presented on ICRA2012 and IROS2012 and an official reference for a journal paper is in progress. A coarse outline of how it works can be seen in the following video.


  .. raw:: html

     <iframe width="560" height="315" src="http://www.youtube.com/embed/Wd4OM8wOO1E?rel=0" frameborder="0" allowfullscreen></iframe>

This shows how to detect people with an Primesense device, the full version 
working on oni and pcd files can be found in the git master.
The code assumes a organised and projectable pointcloud, and should work with other 
sensors then the Primesense device.

  .. image:: images/gpu/people/ss26_1.jpg
    :width: 400 pt
    :height: 372 pt
  .. image:: images/gpu/people/ss26_2.jpg
    :width: 400 pt
    :height: 372 pt

In order to run the code you'll need a decent Nvidia GPU with Fermi or Kepler architecture, have a look
at the GPU installation tutorial to get up and running with your GPU installation.

The code
--------
The full version of this code can be found in PCL gpu/people/tools,
the following is a reduced version for the tutorial.
This version can be found in doc/tutorials/content/sources/gpu/people_detect.

The explanation
---------------

Now, let's break down the code piece by piece. Starting from the main routine.

.. literalinclude:: sources/gpu/people_detect/src/people_detect.cpp
   :language: cpp
   :lines: 317-368

First the GPU device is set, by default this is the first GPU found in the bus, but if you have multiple GPU's in
your system, this allows you to select a specific one.
Then a OpenNI Capture is made, see the OpenNI Grabber tutorial for more info on this. (TODO add link)

.. literalinclude:: sources/gpu/people_detect/src/people_detect.cpp
   :language: cpp
   :lines: 330-339

The implementation is based on a similar approach as Shotton et al. and thus needs off-line learned random
decision forests for labeling. The current implementation allows up to 4 decision trees to be loaded into
the forest. This is done by giving it the names of the text files to load.

.. literalinclude:: sources/gpu/people_detect/src/people_detect.cpp
   :language: cpp
   :lines: 341-342

An additional parameter allows you to configure the number of trees to be loaded. 

.. literalinclude:: sources/gpu/people_detect/src/people_detect.cpp
   :language: cpp
   :lines: 351-353

Then the RDF object is created, loading the trees upon creation.

.. literalinclude:: sources/gpu/people_detect/src/people_detect.cpp
   :language: cpp
   :lines: 355-360

Now we create the application object, give it the pointer to the RDF object and start the loop.
Now we'll have a look at the main loop.


.. literalinclude:: sources/gpu/people_detect/src/people_detect.cpp
   :language: cpp
   :lines: 238-286

This routine first connects a callback routine to the grabber and waits for valid data to arrive.
Each time the data arrives it will call the process function of the people detector, this is a
fully encapsulated method and will call the complete pipeline. 
Once the pipeline completed processing, the results can be fetched as public structs or methods from the
people detector object. Have a look at doc.pointclouds.org for more documentation on the
available structs and methods.
The visualizeAndWrite method will illustrate one of the available methods of the people detector object:

.. literalinclude:: sources/gpu/people_detect/src/people_detect.cpp
   :language: cpp
   :lines: 141-178

Line 144 calls the RDF getLabels method which returns the labels on the device, these however
are a discrete enum of the labels and are visually hard to recognize, so these are converted to
colors that illustrate each body part in line 145. 
At this point the results are still stored in the device memory and need to be copied to the CPU
host memory, this is done in line 151. Afterwards the images are shown and stored to disk.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/gpu/people_detect/CMakeLists.txt
   :language: cmake
   :linenos:

After you have made the executable, you can run it. Simply do:
  $ ./people_detect
