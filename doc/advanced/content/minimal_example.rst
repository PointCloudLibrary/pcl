.. _minimal_example:

How to build a minimal example
------------------------------

First of all make a backup of your current state or start a new project for the
minimal example. Than there are basically two ways: strip down your program or
start from scratch.

Method 1: Strip down your program
=================================

This method has the advantage that you start with your actual problem and you
can test all the time if you are on the right track. First make sure that the
program actually compiles without the problematic code by commenting it. Then
start removing unneeded code until the bare minimum and make sure that it's
still showing the error by compiling it with and without the problematic line
(make sure it still emits the same error message).

Method 2: Start from scratch
============================

If your program is to big to strip it down, it's maybe easier to start from
scratch by building a small project that only includes the problematic code. 
Again make sure that it actually compiles without the erroneous code and emits
the same error with it.

How to deal with input data (e.g. point clouds)
-----------------------------------------------

If you fear that your problem is connected to the input data (either if you
have a problem with pcl/io or the error depends on the input data) you should
include the input with your minimal example. If the file is to big and a
stripped down version doesn't work, you should upload it somewhere and only
provide a link to the data. If you can't include the data or don't know a way
to provide it, add a remark to your mail and we will contact you to find a
solution.

If the input data is not so important it is best to generate fake data:

.. code-block:: cpp
   :linenos:

   pcl::PointCloud<pcl::PointCloudXYZ> cloud;
   cloud.insert (cloud.end (), PointXYZ (1, 1, 1));

I'm linking against other libraries, what to do?
------------------------------------------------

Normally other libraries should not interfere, so try to build a minimal
example using PCL (and it's dependencies) first. If your problems is gone
without the other library please make sure that it's not actually a problem
with one of the other libraries and add a comment in your minimal example.

Final Make
----------
Please put only one error into the minimal example as well as include all
necessary files to build it.

References
----------
- `Latex minimal example <http://www.minimalbeispiel.de/mini-en.html>`_
- `How to Report Bugs Effectively <http://www.chiark.greenend.org.uk/~sgtatham/bugs.html>`_
