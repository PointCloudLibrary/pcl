.. _how_to_write_a_tutorial:

How to write a good tutorial
----------------------------

No matter how many tutorials we create and upload at
www.pointclouds.org/documentation/tutorials, there are never going to be
enough. :) As our code base and user base are growing, so is the demand for
detailed explanations or step-by-step/how-to documentation increasing. This
short guide will help you understand how **you** can contribute documentation
and help improve the project.


The Point Cloud Library (PCL) documentation infrastructure has two distinct
parts:

1. `API documentation <http://docs.pointclouds.org/>`_ -- we are using
`Doxygen <http://www.doxygen.org/>`_ to automatically generate the best
possible API documentation, directly from our source files;

2. `Tutorials and HowTo documents <http://www.pointclouds.org/documentation>`_
-- we are using `Restructured Text <http://docutils.sourceforge.net/rst.html>`_
via `Sphinx <http://sphinx.pocoo.org>`_ to transform simple **reST** files into
beautiful HTML documents.


Both documentation sources are stored in our `Source repository
<https://github.com/PointCloudLibrary/pcl>`_ and the web pages are generated
hourly by our server via `crontab` jobs.

In the next two sections we will address both of the above, and present a small
example for each. We'll begin with the easiest of the two: adding a new
tutorial.

Creating a new tutorial
-----------------------

As already mentioned, we make use of Sphinx to generate HTML files from reST
(restructured text) documents. If you want to add a new tutorial, we suggest
you read the following resources:

 * http://sphinx.pocoo.org/rest.html - official Sphinx documentation
 * http://docutils.sourceforge.net/rst.html - official RST documentation
 * http://www.siafoo.net/help/reST - has a nice tutorial/set of examples

Once you understand how reST works, look over our current set of tutorials for
examples at https://github.com/PointCloudLibrary/pcl/tree/master/doc/tutorials/content.

To add a new tutorial, simply create a new file, and send it to us together
with the images/videos that you want included in the tutorial. The best way to
do this is to login to https://github.com/PointCloudLibrary/pcl and send it as
a pull request.


Improving the API documentation
-------------------------------

Providing a good API documentation is not easy -- as finding a balance between
the amount of information that you present for each function, versus keeping it
clean and simple is ermmm, a challenge in itself. Differently said, it's hard
to know what sort of people will look at the API: hardcore developers or first
time users. 

Our solution is to document the API as best as possible, but leave certain more
complex details such as application examples for the tutorials. However, while
this is a nice goal, it's very improbable that our documentation is perfect.

To help us improve the API documentation, all that you need to do is simply
check out the source code of PCL (we recommend trunk if you're going to start
editing the sources), like::

  git clone https://github.com/PointCloudLibrary/pcl

Then, edit the file containing the function/class that you want to improve the
documentation for, say *common/include/pcl/point_cloud.h*, and go to the
element that you want to improve. Let's take *points* for example::

  /** \brief The point data. */
  std::vector<PointT, Eigen::aligned_allocator<PointT> > points;

What you have to modify is the Doxygen-style comment starting with /\*\* and
ending with \*/. See http://www.doxygen.org for more information.

To send us the modification, please send a pull request through Github.

Testing the modified API documentation
--------------------------------------

If you want to test it locally on your machine, make sure you have Doxygen
installed, and go into the build system (here we assume that you followed the
source installation instructions from
http://www.pointclouds.org/downloads) and run::

  make doc

This will create a set of html files containing the API documentation for PCL,
in **build/html/**

