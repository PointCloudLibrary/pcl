.. _registration_api:

The PCL Registration API
------------------------

The problem of consistently aligning various 3D point cloud data views into a
complete model is known as **registration**. Its goal is to ﬁnd the relative
positions and orientations of the separately acquired views in a global
coordinate framework, such that the intersecting areas between them overlap
perfectly. For every set of point cloud datasets acquired from different views,
we therefore need a system that is able to align them together into a single
point cloud model, so that subsequent processing steps such as segmentation and
object reconstruction can be applied. 

.. image:: images/registration/scans.png
    :align: center

A motivation example in this sense is given in the figure above, where a set of
six individual datasets has been acquired using a tilting 2D laser unit. Since
each individual scan represents only a small part of the surrounding world, it
is imperative to find ways to register them together, thus creating the complete
point cloud model as shown in the figure below.

.. image:: images/registration/s1-6.png
    :align: center

The algorithmic work in the PCL registration library is motivated by finding
correct point correspondences in the given input datasets, and estimating rigid
transformations that can rotate and translate each individual dataset into a
consistent global coordinate framework. This registration paradigm becomes
easily solvable if the point correspondences are perfectly known in the input
datasets. This means that a selected list of points in one dataset have to
"coincide" from a feature representation point of view with a list of points
from another dataset. Additionally, if the correspondences estimated are
"perfect", then the registration problem has a closed form solution.


PCL contains a set of powerful algorithms that allow the estimation of multiple
sets of correspondences, as well as methods for rejecting bad correspondences,
and estimating transformations in a robust manner from them. The following
sections will describe each of them individually.


An overview of pairwise registration
------------------------------------

.. image:: images/registration/block_diagram_single_iteration.png
    :align: center
