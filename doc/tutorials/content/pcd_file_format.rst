.. _pcd_file_format:

The PCD (**P**\oint **C**\loud **D**\ata) file format
-----------------------------------------------------

This document describes the PCD (Point Cloud Data) file format, and the way it
is used inside Point Cloud Library (PCL).

.. image:: images/PCD_icon.png
   :alt: PCD file format icon
   :align: right

Why a new file format?
----------------------

The PCD file format is not meant to reinvent the wheel, but rather to
complement existing file formats that for one reason or another did not/do not
support some of the extensions that PCL brings to n-D point cloud processing.

PCD is not the first file type to support 3D point cloud data. The computer
graphics and computational geometry communities in particular, have created
numerous formats to describe arbitrary polygons and point clouds acquired using
laser scanners. Some of these formats include:

* `PLY <http://en.wikipedia.org/wiki/PLY_(file_format)>`_ - a polygon file format, developed at Stanford University by Turk et al

* `STL <http://en.wikipedia.org/wiki/STL_(file_format)>`_ - a file format native to the stereolithography CAD software created by 3D Systems

* `OBJ <http://en.wikipedia.org/wiki/Wavefront_.obj_file>`_ - a geometry definition file format first developed by Wavefront Technologies 

* `X3D <http://en.wikipedia.org/wiki/X3D>`_ - the ISO standard XML-based file format for representing 3D computer graphics data

* `and many others <http://en.wikipedia.org/wiki/Category:Graphics_file_formats>`_

All the above file formats suffer from several shortcomings, as explained in
the next sections -- which is natural, as they were created for a different
purpose and at different times, before today's sensing technologies and
algorithms have been invented. 

PCD versions
------------

PCD file formats might have different revision numbers, prior to the release of
Point Cloud Library (PCL) version 1.0. These are numbered with PCD_Vx (e.g.,
PCD_V5, PCD_V6, PCD_V7, etc) and represent version numbers 0.x for the PCD
file.

The official entry point for the PCD file format in PCL however should be
version **0.7 (PCD_V7)**.

File format header
------------------

Each PCD file contains a header that identifies and declares certain properties
of the point cloud data stored in the file.

As of version 0.7, the PCD header contains the following entries:

* **FIELDS** - specifies the name of each dimension/field that a point can
  have. Examples::

    FIELDS x y z                                # XYZ data
    FIELDS x y z rgb                            # XYZ + colors
    FIELDS x y z normal_x normal_y normal_z     # XYZ + surface normals
    FIELDS j1 j2 j3                             # moment invariants 
    ...

* **SIZE** - specifies the size of each dimension in bytes. Examples: 
  
  * *unsigned char*/*char* has 1 byte
  * *unsigned short*/*short* has 2 bytes
  * *unsigned int*/*int*/*float* has 4 bytes
  * *double* has 8 bytes

* **TYPE** - specifies the type of each dimension as a char. The current accepted types are:

  * **I** - represents signed types int8 (*char*), int16 (*short*), and int32 (*int*)
  * **U** - represents unsigned types uint8 (*unsigned char*), uint16 (*unsigned short*), uint32 (*unsigned int*)
  * **F** - represents float types

* **COUNT** - specifies how many elements does each dimension have. For
  example, *x* data usually has 1 element, but a feature descriptor like the
  *VFH* has 308. Basically this is a way to introduce n-D histogram descriptors
  at each point, and treating them as a single contiguous block of memory. By
  default, if *COUNT* is not present, all dimensions' count is set to 1.


* **WIDTH** - specifies the width of the point cloud dataset in the number of
  points. *WIDTH* has two meanings:

  * it can specify the total number of points in the cloud (equal with **POINTS** see below) for unorganized datasets;
  * it can specify the width (total number of points in a row) of an organized point cloud dataset.

  Also see **HEIGHT**.

  .. note::

     An **organized point cloud** dataset is the name given to point clouds that
     resemble an organized image (or matrix) like structure, where the data is
     split into rows and columns. Examples of such point clouds include data
     coming from stereo cameras or Time Of Flight cameras. The advantages of a
     organized dataset is that by knowing the relationship between adjacent
     points (e.g. pixels), nearest neighbor operations are much more efficient,
     thus speeding up the computation and lowering the costs of certain
     algorithms in PCL.

  Examples::

    WIDTH 640     # there are 640 points per line

* **HEIGHT** - specifies the height of the point cloud dataset in the number of points. *HEIGHT* has two meanings:

  * it can specify the height (total number of rows) of an organized point cloud dataset;
  * it is set to **1** for unorganized datasets (*thus used to check whether a dataset is organized or not*).

  Example::

    WIDTH 640       # Image-like organized structure, with 640 rows and 480 columns,
    HEIGHT 480      # thus 640*480=307200 points total in the dataset

  Example::

    WIDTH 307200
    HEIGHT 1        # unorganized point cloud dataset with 307200 points


* **POINTS** - specifies the total number of points in the cloud. As of version
  0.7, its purpose is a bit redundant, so we're expecting this to be removed in
  future versions.

  Example::

    POINTS 307200   # the total number of points in the cloud


* **DATA** - specifies the data type that the point cloud data is stored in. As
  of version 0.7, two data types are supported: *ascii* and *binary*. See the
  next section for more details.


.. note::

  The next bytes directly after the header's last line (**DATA**) are
  considered part of the point cloud data, and will be interpreted as such.

Data storage types
------------------

As of version 0.7, the **.PCD** file format uses two different modes for storing data:

* in **ASCII** form, with each point on a new line::

    p_1
    p_2
    p_3
    p_4
    ...

    p_n

* in **binary** form, where the data is a complete memory copy of the
  `pcl::PointCloud.points` array/vector. On Linux systems, we use `mmap`/`munmap`
  operations for the fastest possible read/write access to the data.


Storing point cloud data in both a simple ascii form with each point on a line,
space or tab separated, without any other characters on it, as well as in a
binary dump format, allows us to have the best of both worlds: simplicity and
speed, depending on the underlying application. The ascii format allows users
to open up point cloud files and plot them using standard software tools like
`gnuplot` or manipulate them using tools like `sed`, `awk`, etc.



Advantages over other file formats
----------------------------------

Having PCD as (yet another) file format can be seen as PCL suffering from the `not invented here` syndrome. In reality, this is not the case, as none of the above mentioned file formats offers the flexibility and speed of PCD files. Some of the clearly stated advantages include:

* the ability to store and process organized point cloud datasets -- this is of
  extreme importance for real time applications, and research areas such as
  augmented reality, robotics, etc;

* binary `mmap`/`munmap` data types are the fastest possible way of loading and
  saving data to disk. 

* storing different data types (all primitives supported: char, short, int,
  float, double) allows the point cloud data to be flexible and efficient with
  respect to storage and processing. Invalid point dimensions are usually
  stored as NAN types.

* n-D histograms for feature descriptors -- very important for 3D
  perception/computer vision applications


An additional advantage is that by controlling the file format, we can best
adapt it to PCL, and thus obtain the highest performance with respect to PCL
applications, rather than adapting a different file format to PCL as the native
type and inducing additional delays through conversion functions. 


.. note::

  Though PCD (Point Cloud Data) is the *native* file format in PCL, the
  `pcl_io` library should offer the possibility to save and load data in all
  the other aforementioned file formats too.


Example
-------

A snippet of a PCD file is attached below. It is left to the reader to
interpret the data and see what it means. :) Have fun!


