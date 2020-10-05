.. _using_kinfu_large_scale:

Using Kinfu Large Scale to generate a textured mesh
---------------------------------------------------

This tutorial demonstrates how to use KinFu Large Scale to produce a mesh (in meters) from a room, and apply texture information in post-processing for a more appealing visual result. The first part of this tutorial shows how to obtain the TSDF cloud from KinFu Large Scale. The second part shows how to convert the TSDF cloud into a uniform mesh. The third part shows how to texture the obtained mesh using the RGB images and poses we obtained from KinFu Large Scale.

Part 1: Running pcl_kinfu_largeScale to obtain a TSDF cloud
-----------------------------------------------------------

*TSDF Cloud*

This section describes the TSDF Cloud, which is the expected output of KinFu Large Scale. A TSDF cloud looks like the one in the following video.

  .. raw:: html
  
     <iframe width="420" height="315" src="https://www.youtube.com/embed/AjjSZufyprU" frameborder="0" allowfullscreen></iframe>

You may be wondering: *"What is the difference between a TSDF cloud and a normal point cloud?"* Well, a TSDF cloud *is* a point cloud. However, the TSDF cloud makes use of how the data is stored within GPU at KinFu runtime. 

  .. image:: images/using_kinfu_large_scale/11.jpg
    :align: center
    :width: 696 pt
    :height: 326 pt

  *Figure 1: The cube is subdivided into a set of Voxels. These voxels are equal in size. The default size in meters for the cube is 3 meters per axis. The default voxel size is 512 per axis. Both the number of voxels and the size in meters give the amount of detail of our model.*
 
As you may already know, the way in which the TSDF volume is stored in GPU is a voxel grid. KinFu subdivides the physical space of the cube (e.g. 3 meters) into a voxel grid with a certain number of voxels per axis (say, 512 voxels per axis). The size in meters of the cube and the number of voxels give us the resolution of our cube. The quality of the model is proportional to these two parameters. However, modifying them affects directly the memory footprint for our TSDF volume in GPU. Further information on these properties can be found in the relevant papers.
  
At the time of data extraction, the grid is traversed from front to back, and the TSDF values are checked for each voxel. In the figure below, you may notice that the values range from -1 to 1. 

  .. image:: images/using_kinfu_large_scale/12.jpg
    :align: center
    :width: 400 pt
    :height: 350 pt

  *Figure 2: A representation of the TSDF Volume grid in the GPU. Each element in the grid represents a voxel, and the value inside it represents the TSDF value. The TSDF value is the distance to the nearest isosurface. The TSDF has a positive value whenever we are "in front" of the surface, whereas it has a negative value when inside the isosurface. At the time of extraction, we avoid extracting the voxels with a value of 1, since they represent empty space, and are therefore of no use to our model.*

Since we want to minimize the required bandwidth between GPU and CPU, we will only extract the voxels with a TSDF value in the range [-1, 0.98]. We avoid extracting voxels with a value of 1 because they represent empty space. In this way we ensure that we only extract those voxels that are close to the isosurface. The TSDF cloud is not in meters. The X,Y,Z coordinates for each of the extracted points correspond to the voxel indices with respect to the world model. 
  
As mentioned above, the TSDF cloud is a section of the TSDF volume grid; which is why the points are equally-spaced and uniformly-distributed. This can be observed when we zoom in the point cloud.

*Running pcl_kinfu_largeScale*

Finally, we are ready to start KinFu Large Scale. After building the git master, we will call the application::
  
  $ ./bin/pcl_kinfu_largeScale -r -et
  
The *-r* parameter enables registration, which is used for texture extraction. In particular, it allows us to extract the correct focal length. The *-et* parameter enables the texture extraction. By enabling this option, we will extract RGB images at the same time that we are scanning. All the RGB snapshots are saved in the KinFuSnapshots folder. Each RGB image will be saved with its corresponding camera pose. It is suggested to empty this directory before starting the scan, in this way we avoid using textures that do not correspond to our latest scan. 

The video below shows the process of scanning a large area. Notice the smooth movements at the time of scanning. Furthermore, notice how a complex object (e.g. chair) is kept within sight at the time of shifting so that tracking does not get lost.

  -	The shifting can be triggered by rotation or translation.

  -	Every time we shift out part of the cube,  four main things happen: 1)We save the data in the slice that is shifted out and send it to the world model, which is stored in CPU. 2) We clear that slice to allow for new data to be added. 3) We shift the cube's origin. 4) We retrieve existing data (if any) from the world model and load it to the TSDF volume. This is only present when we return to areas that we previously scanned.

  -	Whenever we are satisfied with the area that we have scanned, we press the "L" key to let KinFu know that we are ready to perform the exit routine. However, the routine is not executed until we shift again. 

What the exit routine will do is to get all the information regarding our model, comprise it in a point cloud and save it to disk as *world.pcd* The PCD file is saved in the same directory from where we run KinFu Large Scale.

Since we used the *-et* option, you will also find a folder called KinFuSnapshots, which contains all the RGB images and its corresponding poses for this scan. The following video demonstrates the scanning process and the generated output:

  .. raw:: html
  
    <iframe width="420" height="315" src="https://www.youtube.com/embed/rF1N-EEIJao" frameborder="0" allowfullscreen></iframe>

The next part of this tutorial will demonstrate how to get a mesh from the TSDF cloud.

Part 2: Running pcl_kinfu_largeScale_mesh_output to convert the TSDF cloud into a mesh
--------------------------------------------------------------------------------------

This section describes how to convert the TSDF Cloud, which is the expected output of KinFu Large Scale, into a mesh. For this purpose we will use the meshing application in KinFu Large Scale. The input for this application is the world model as a PCD file. The output is a set of meshes, since the world model is processed as a set of cubes. 

The reason why we load the world model in cubes is because we have the limitation of memory in the GPU. A point of improvement for the meshing application could be to return the complete mesh instead of a set of meshes. Contributions welcome! 

After we obtain a set of meshes, we process them in Meshlab in order to merge them as a single mesh. At this point it is important to mention that we need to save the mesh as a ply file without binary encoding. 

The mesh is also simplified using quadric edge decimation. The reason for doing this is to reduce the time it takes to perform the UV mapping in the next step. The UV mapping is done for each face in the mesh. Therefore, by reducing the number of faces we reduce the time it takes to generate the texture. 

We run this application with the command::

  $ ./bin/pcl_kinfu_largeScale_mesh_output world.pcd

where *world.pcd* is the world model we obtained from KinFu Large Scale. The following video shows the process of creating, merging, and simplifying the meshes into a single mesh which we will use for texturing. 

  .. raw:: html
  
    <iframe width="420" height="315" src="https://www.youtube.com/embed/XMJ-ikSZAOE" frameborder="0" allowfullscreen></iframe>

The next part of this tutorial will demonstrate how to generate the texture for the mesh we have just created.

Part 3: Running pcl_kinfu_largeScale_texture_output to generate the texture
----------------------------------------------------------------------------

This section describes how to generate the textures for the mesh we created in the previous step. The input for this application is the merged mesh, as well as the RGB captures and poses we saved during the scanning in part 1. The RGB captures and poses should be in the KinFuSnapshots folder. We select the most representative snapshots for the sake of time. Each snapshot must have its corresponding camera pose in a text file in the same folder. 

The generated PLY mesh must be in the same folder as the snapshots and camera poses. The output will be generated as an OBJ file with its corresponding MTL file. The former contains data about the mesh, whereas the latter contains information about the texture. Unfortunately at this point some of the generated textures may seen patched, this is based on how the RGB camera in the Kinect adapts to light. A potential area of improvement could be to equalize the color tones in the images. Contributions welcome! 
  
In order to run the texturing application, we use the following command::
  
  $ ./bin/pcl_kinfu_largeScale_texture_output path/to/merged_mesh.ply

The following video shows the process in detail. It also shows the final output for this tutorial.

  .. raw:: html
  
     <iframe width="420" height="315" src="https://www.youtube.com/embed/7S7Jj-4cKHs" frameborder="0" allowfullscreen></iframe>

Output
-------

The viewer below displays a sample of the output obtained after the entire pipeline. The mesh was decimated, and the faces were removed so that only the points remained. So, the output mesh was converted from mesh (.ply) to point cloud (.pcd) to show it in this tutorial. The vertex count is ~900k points.

.. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=https://raw.github.com/PointCloudLibrary/data/master/tutorials/kinfu_large_scale/Tutorial_Cloud_Couch_bin_compressed.pcd&scale=0.004&psize=1" align="center" width="600" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>

To further demonstrate the capabilities of KinFu Large Scale, we made another example with a room.

.. raw:: html

    <iframe src="http://pointclouds.org/assets/viewer/pcl_viewer.html?load=https://raw.github.com/PointCloudLibrary/data/master/tutorials/kinfu_large_scale/using_kinfu_large_scale_output.pcd&scale=0.004&psize=1" align="center" width="600" height="400" marginwidth="0" marginheight="0" frameborder='no' allowfullscreen mozallowfullscreen webkitallowfullscreen style="max-width: 100%;"></iframe>


General Recommendations
-----------------------

There is a set of recommendations that we want to mention regarding the use of KinFu Large Scale. These recommendations are listed below:
 
  1) **Scan scenes with enough details for ICP:** It is a known fact that ICP does not perform well in scenes with few details, or where there are a lot of co-planer surfaces. In other words, if the only thing you have is a wall and floor, most probably the tracking will not perform well.

  2) **Frame rate is less than original KinFu:** The code in Kinfu largescale is experimental. There are still many areas in which the performance can be optimized to provide a faster execution. In our tests, the obtained frame rate is around 20 fps. We are using a GTX480 and 4GB of RAM. The decrease in frame rate is mainly because of two things. First, that the code has not yet been completely optimized. Second, that additional operations are taking place in the frame processing loop as a result of the large scale implementation. 

  3) **Scan smoothly:** Since there are more things happening per frame, KinFu Large Scale may not respond as fast as the original KinFu. Data is exchanged between GPU and CPU especially at the time of shifting. Performing smooth movements, in particular at the time of shifting, decreases the risk of losing the camera pose tracking. Be patient and you will get good results. 

Related Executables
-------------------
  
There are three executables related to this tutorial:
  
  -	**pcl_kinfu_largeScale:** In charge of obtaining the scan of the room. Its functionality is almost the same as KinFu, except that it includes the capability of shifting the cube that is being scanned to allow for large area 3D reconstruction. The output from this application is the world reconstructed model as a TSDF cloud. The concept of TSDF cloud will be explained better below. Another output from this application is a set of RGB screenshots and their corresponding camera poses.

  -	**pcl_kinfu_largeScale_mesh_output:** This application is in charge of generating a set of meshes from the extracted TSDF world cloud. The TSDF world model is processed as cubes of points and generates a mesh for each of these cubes.  

  -	As an additional processing step, the current state of the implementation requires that the output meshes are merged in the software of your preference. In other words, the output of the meshing application is given as a set of mesh cubes. This tutorial has been done using with Meshlab (*merge visible layers* function in Meshlab). Since the following step is performed on a per-face basis, it is also optional to decimate the mesh in order to decrease the time it takes to generate the texture.

  -	**pcl_kinfu_largeScale_texture_output:** After the meshes are generated and merged into one, this application is in charge of using the RGB screenshots and their corresponding camera poses taken during the scan to perform UV mapping in order to reconstruct the texture of the model.
  
Conclusion
----------
In this tutorial we have shown the pipeline from scanning to final texturing using KinFu Large Scale. The - *experimental* - code is available in the master branch of PCL.
