.. _model_outlier_removal:

Filtering a PointCloud using ModelOutlierRemoval
------------------------------------------------

This tutorial demonstrates how to extract parametric models for example for planes or spheres 
out of a PointCloud by using SAC_Models with known coefficients.
If you don't know the models coefficients take a look at the :ref:`random_sample_consensus` tutorial. 

The code
--------

First, create a file, let's call it ``model_outlier_removal.cpp``, in your favorite
editor, and place the following inside it:

.. literalinclude:: sources/model_outlier_removal/model_outlier_removal.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

In the following lines, we define the PointClouds structures, fill in noise, random points 
on a plane as well as random points on a sphere and display its content to screen.

.. literalinclude:: sources/model_outlier_removal/model_outlier_removal.cpp
   :language: cpp
   :lines: 7-45

Finally we extract the sphere using ModelOutlierRemoval.

.. literalinclude:: sources/model_outlier_removal/model_outlier_removal.cpp
   :language: cpp
   :lines: 50-61

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/model_outlier_removal/CMakeLists.txt
   :language: cmake
   :linenos:


After you have made the executable, you can run it. Simply do::

  $ ./model_outlier_removal

You will see something similar to::

  Cloud before filtering: 
    0.352222 -0.151883 -0.106395
    -0.397406 -0.473106 0.292602
    -0.731898 0.667105 0.441304
    -0.734766 0.854581 -0.0361733
    -0.4607 -0.277468 -0.916762
    -0.82 -0.341666 0.4592
    -0.728589 0.667873 0.152
    -0.3134 -0.873043 -0.3736
    0.62553 0.590779 0.5096
    -0.54048 0.823588 -0.172
    -0.707627 0.424576 0.5648
    -0.83153 0.523556 0.1856
    -0.513903 -0.719464 0.4672
    0.291534 0.692393 0.66
    0.258758 0.654505 -0.7104
  Sphere after filtering: 
    -0.82 -0.341666 0.4592
    -0.728589 0.667873 0.152
    -0.3134 -0.873043 -0.3736
    0.62553 0.590779 0.5096
    -0.54048 0.823588 -0.172
    -0.707627 0.424576 0.5648
    -0.83153 0.523556 0.1856
    -0.513903 -0.719464 0.4672
    0.291534 0.692393 0.66
    0.258758 0.654505 -0.7104

