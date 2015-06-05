.. _writing_new_classes:

Writing a new PCL class
-----------------------

Converting code to a PCL-like mentality/syntax for someone that comes in
contact for the first time with our infrastructure might appear difficult, or
raise certain questions. 

This short guide is to serve as both a HowTo and a FAQ for writing new PCL
classes, either from scratch, or by adapting old code.

Besides converting your code, this guide also explains some of the advantages
of contributing your code to an already existing open source project. Here, we
advocate for PCL, but you can certainly apply the same ideology to other
similar projects.

.. contents::

Advantages: Why contribute?
---------------------------

The first question that someone might ask and we would like to answer is:

*Why contribute to PCL, as in what are its advantages?*

This question assumes you've already identified that the set of tools and
libraries that PCL has to offer are useful for your project, so you have already
become an *user*. 

Because open source projects are mostly voluntary efforts, usually with
developers geographically distributed around the world, it's very common that
the development process has a certain *incremental*, and *iterative* flavor.
This means that:

 * it's impossible for developers to think ahead of all the possible uses a new
   piece of code they write might have, but also...

 * figuring out solutions for corner cases and applications where bugs might
   occur is hard, and might not be desirable to tackle at the beginning, due to
   limited resources (mostly a cost function of free time).


In both cases, everyone has definitely encountered situations where either an
algorithm/method that they need is missing, or an existing one is buggy.
Therefore the next natural step is obvious: 

*change the existing code to fit your application/problem*.

While we're going to discuss how to do that in the next sections, we would
still like to provide an answer for the first question that we raised, namely
"why contribute?".

In our opinion, there are many advantages. To quote Eric Raymond's *Linus's
Law*: **"given enough eyeballs, all bugs are shallow"**. What this means is
that by opening your code to the world, and allowing others to see it, the
chances of it getting fixed and optimized are higher, especially in the
presence of a dynamic community such as the one that PCL has.

In addition to the above, your contribution might enable, amongst many things:

  * others to create new work based on your code;
  * you to learn about new uses (e.g., thinks that you haven't thought it could be used when you designed it);
  * worry-free maintainership (e.g., you can go away for some time, and then return and see your code still working. Others will take care of adapting it to the newest platforms, newest compilers, etc);
  * your reputation in the community to grow - everyone likes free stuff (!).

For most of us, all of the above apply. For others, only some (your mileage
might vary). 

.. _bilateral_filter_example:

Example: a bilateral filter
---------------------------

To illustrate the code conversion process, we selected the following example:
apply a bilateral filter over intensity data from a given input point cloud,
and save the results to disk. 

.. code-block:: cpp
   :linenos:

    #include <pcl/point_types.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/kdtree/kdtree_flann.h>

    typedef pcl::PointXYZI PointT;

    float
    G (float x, float sigma)
    {
      return exp (- (x*x)/(2*sigma*sigma));
    }

    int
    main (int argc, char *argv[])
    {
      std::string incloudfile = argv[1];
      std::string outcloudfile = argv[2];
      float sigma_s = atof (argv[3]);
      float sigma_r = atof (argv[4]);

      // Load cloud
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);
      int pnumber = (int)cloud->size ();

      // Output Cloud = Input Cloud
      pcl::PointCloud<PointT> outcloud = *cloud;

      // Set up KDTree
      pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);
      tree->setInputCloud (cloud);

      // Neighbors containers
      std::vector<int> k_indices;
      std::vector<float> k_distances;

      // Main Loop
      for (int point_id = 0; point_id < pnumber; ++point_id)
      {
        float BF = 0;
        float W = 0;

        tree->radiusSearch (point_id, 2 * sigma_s, k_indices, k_distances);

        // For each neighbor
        for (size_t n_id = 0; n_id < k_indices.size (); ++n_id)
        {
          float id = k_indices.at (n_id);
          float dist = sqrt (k_distances.at (n_id));
          float intensity_dist = abs (cloud->points[point_id].intensity - cloud->points[id].intensity);

          float w_a = G (dist, sigma_s);
          float w_b = G (intensity_dist, sigma_r);
          float weight = w_a * w_b;

          BF += weight * cloud->points[id].intensity;
          W += weight;
        }

        outcloud.points[point_id].intensity = BF / W;
      }

      // Save filtered output
      pcl::io::savePCDFile (outcloudfile.c_str (), outcloud);
      return (0);
    }

The presented code snippet contains:
 * an I/O component: lines 21-27 (reading data from disk), and 64 (writing data to disk)
 * an initialization component: lines 29-35 (setting up a search method for nearest neighbors using a KdTree)
 * the actual algorithmic component: lines 7-11 and 37-61

Our goal here is to convert the algorithm given into an useful PCL class so that it can be reused elsewhere. 

Setting up the structure
------------------------

.. note::

  If you're not familiar with the PCL file structure already, please go ahead
  and read the `PCL C++ Programming Style Guide
  <http://www.pointclouds.org/documentation/advanced/pcl_style_guide.php>`_ to
  familiarize yourself with the concepts. 

There's two different ways we could set up the structure: i) set up the code
separately, as a standalone PCL class, but outside of the PCL code tree; or ii)
set up the files directly in the PCL code tree. Since our assumption is that
the end result will be contributed back to PCL, it's best to concentrate on the
latter, also because it is a bit more complex (i.e., it involves a few
additional steps). You can obviously repeat these steps with the former case as
well, with the exception that you don't need the files copied in the PCL tree,
nor you need the fancier *cmake* logic.

Assuming that we want the new algorithm to be part of the PCL Filtering library, we will begin by creating 3 different files under filters:

 * *include/pcl/filters/bilateral.h* - will contain all definitions;
 * *include/pcl/filters/impl/bilateral.hpp* - will contain the templated implementations;
 * *src/bilateral.cpp* - will contain the explicit template instantiations [*]_.


We also need a name for our new class. Let's call it `BilateralFilter`.

.. [*] The PCL Filtering API specifies that two definitions and implementations must be available for every algorithm: one operating on PointCloud<T> and another one operating on PCLPointCloud2. For the purpose of this tutorial, we will concentrate only on the former.

bilateral.h
===========

As previously mentioned, the *bilateral.h* header file will contain all the
definitions pertinent to the `BilateralFilter` class. Here's a minimal
skeleton:

.. code-block:: cpp
   :linenos:

    #ifndef PCL_FILTERS_BILATERAL_H_
    #define PCL_FILTERS_BILATERAL_H_

    #include <pcl/filters/filter.h>

    namespace pcl
    {
      template<typename PointT>
      class BilateralFilter : public Filter<PointT>
      {
      };
    }

    #endif // PCL_FILTERS_BILATERAL_H_

bilateral.hpp
=============

While we're at it, let's set up two skeleton *bilateral.hpp* and
*bilateral.cpp* files as well. First, *bilateral.hpp*:

.. code-block:: cpp
   :linenos:

    #ifndef PCL_FILTERS_BILATERAL_IMPL_H_
    #define PCL_FILTERS_BILATERAL_IMPL_H_

    #include <pcl/filters/bilateral.h>
    
    #endif // PCL_FILTERS_BILATERAL_H_

This should be straightforward. We haven't declared any methods for
`BilateralFilter` yet, therefore there is no implementation. 

bilateral.cpp
=============

Let's write *bilateral.cpp* too:

.. code-block:: cpp
   :linenos:

    #include <pcl/filters/bilateral.h>
    #include <pcl/filters/impl/bilateral.hpp>
    
Because we are writing templated code in PCL (1.x) where the template parameter
is a point type (see :ref:`adding_custom_ptype`), we want to explicitely
instantiate the most common use cases in *bilateral.cpp*, so that users don't
have to spend extra cycles when compiling code that uses our
`BilateralFilter`. To do this, we need to access both the header
(*bilateral.h*) and the implementations (*bilateral.hpp*).

CMakeLists.txt
==============

Let's add all the files to the PCL Filtering *CMakeLists.txt* file, so we can
enable the build.

.. code-block:: cmake
   :linenos:

    # Find "set (srcs", and add a new entry there, e.g.,
    set (srcs
         src/conditional_removal.cpp
         # ...
         src/bilateral.cpp)
         )

    # Find "set (incs", and add a new entry there, e.g.,
    set (incs
         include pcl/${SUBSYS_NAME}/conditional_removal.h
         # ...
         include pcl/${SUBSYS_NAME}/bilateral.h
         )

    # Find "set (impl_incs", and add a new entry there, e.g., 
    set (impl_incs
         include/pcl/${SUBSYS_NAME}/impl/conditional_removal.hpp
         # ...
         include/pcl/${SUBSYS_NAME}/impl/bilateral.hpp
         )

.. _filling:

Filling in the class structure
------------------------------

If you correctly edited all the files above, recompiling PCL using the new
filter classes in place should work without problems. In this section, we'll
begin filling in the actual code in each file. Let's start with the
*bilateral.cpp* file, as its content is the shortest.

bilateral.cpp
=============

As previously mentioned, we're going to explicitely instantiate and
*precompile* a number of templated specializations for the `BilateralFilter`
class. While this might lead to an increased compilation time for the PCL
Filtering library, it will save users the pain of processing and compiling the
templates on their end, when they use the class in code they write. The
simplest possible way to do this would be to declare each instance that we want
to precompile by hand in the *bilateral.cpp* file as follows:

.. code-block:: cpp
   :linenos:
    
    #include <pcl/point_types.h>
    #include <pcl/filters/bilateral.h>
    #include <pcl/filters/impl/bilateral.hpp>

    template class PCL_EXPORTS pcl::BilateralFilter<pcl::PointXYZ>;
    template class PCL_EXPORTS pcl::BilateralFilter<pcl::PointXYZI>;
    template class PCL_EXPORTS pcl::BilateralFilter<pcl::PointXYZRGB>;
    // ...
 
However, this becomes cumbersome really fast, as the number of point types PCL
supports grows. Maintaining this list up to date in multiple files in PCL is
also painful. Therefore, we are going to use a special macro called
`PCL_INSTANTIATE` and change the above code as follows:

.. code-block:: cpp
   :linenos:
    
    #include <pcl/point_types.h>
    #include <pcl/impl/instantiate.hpp>
    #include <pcl/filters/bilateral.h>
    #include <pcl/filters/impl/bilateral.hpp>

    PCL_INSTANTIATE(BilateralFilter, PCL_XYZ_POINT_TYPES);

This example, will instantiate a `BilateralFilter` for all XYZ point types
defined in the *point_types.h* file (see
:pcl:`PCL_XYZ_POINT_TYPES<PCL_XYZ_POINT_TYPES>` for more information).

By looking closer at the code presented in :ref:`bilateral_filter_example`, we
notice constructs such as `cloud->points[point_id].intensity`. This indicates
that our filter expects the presence of an **intensity** field in the point
type. Because of this, using **PCL_XYZ_POINT_TYPES** won't work, as not all the
types defined there have intensity data present. In fact, it's easy to notice
that only two of the types contain intensity, namely:
:pcl:`PointXYZI<pcl::PointXYZI>` and
:pcl:`PointXYZINormal<pcl::PointXYZINormal>`. We therefore replace
**PCL_XYZ_POINT_TYPES** and the final *bilateral.cpp* file becomes:

.. code-block:: cpp
   :linenos:
    
    #include <pcl/point_types.h>
    #include <pcl/impl/instantiate.hpp>
    #include <pcl/filters/bilateral.h>
    #include <pcl/filters/impl/bilateral.hpp>

    PCL_INSTANTIATE(BilateralFilter, (pcl::PointXYZI)(pcl::PointXYZINormal));

Note that at this point we haven't declared the PCL_INSTANTIATE template for
`BilateralFilter`, nor did we actually implement the pure virtual functions in
the abstract class :pcl:`pcl::Filter<pcl::Filter>` so attemping to compile the
code will result in errors like::

  filters/src/bilateral.cpp:6:32: error: expected constructor, destructor, or type conversion before ‘(’ token

bilateral.h
===========

We begin filling the `BilateralFilter` class by first declaring the
constructor, and its member variables. Because the bilateral filtering
algorithm has two parameters, we will store these as class members, and
implement setters and getters for them, to be compatible with the PCL 1.x API
paradigms. 

.. code-block:: cpp
   :linenos:

    ...
    namespace pcl
    {
      template<typename PointT>
      class BilateralFilter : public Filter<PointT>
      {
        public:
          BilateralFilter () : sigma_s_ (0),
                               sigma_r_ (std::numeric_limits<double>::max ())
          {
          }

          void
          setSigmaS (const double sigma_s)
          {
            sigma_s_ = sigma_s;
          }

          double
          getSigmaS ()
          {
            return (sigma_s_);
          }
          
          void
          setSigmaR (const double sigma_r)
          {
            sigma_r_ = sigma_r;
          }

          double
          getSigmaR ()
          {
            return (sigma_r_);
          }

        private:
          double sigma_s_;
          double sigma_r_;
      };
    }

    #endif // PCL_FILTERS_BILATERAL_H_

Nothing out of the ordinary so far, except maybe lines 8-9, where we gave some
default values to the two parameters. Because our class inherits from
:pcl:`pcl::Filter<pcl::Filter>`, and that inherits from
:pcl:`pcl::PCLBase<pcl::PCLBase>`, we can make use of the
:pcl:`setInputCloud<pcl::PCLBase::setInputCloud>` method to pass the input data
to our algorithm (stored as :pcl:`input_<pcl::PCLBase::input_>`). We therefore
add an `using` declaration as follows:

.. code-block:: cpp
   :linenos:

    ...
      template<typename PointT>
      class BilateralFilter : public Filter<PointT>
      {
        using Filter<PointT>::input_;
        public:
          BilateralFilter () : sigma_s_ (0),
    ...

This will make sure that our class has access to the member variable `input_`
without typing the entire construct. Next, we observe that each class that
inherits from :pcl:`pcl::Filter<pcl::Filter>` must inherit a
:pcl:`applyFilter<pcl::Filter::applyFilter>` method. We therefore define:

.. code-block:: cpp
   :linenos:

    ...
        using Filter<PointT>::input_;
        typedef typename Filter<PointT>::PointCloud PointCloud;

        public:
          BilateralFilter () : sigma_s_ (0),
                               sigma_r_ (std::numeric_limits<double>::max ())
          {
          }

          void
          applyFilter (PointCloud &output);
    ...

The implementation of `applyFilter` will be given in the *bilateral.hpp* file
later. Line 3 constructs a typedef so that we can use the type `PointCloud`
without typing the entire construct. 

Looking at the original code from section :ref:`bilateral_filter_example`, we
notice that the algorithm consists of applying the same operation to every
point in the cloud. To keep the `applyFilter` call clean, we therefore define
method called `computePointWeight` whose implementation will contain the corpus
defined in between lines 45-58:

.. code-block:: cpp
   :linenos:

    ... 
          void
          applyFilter (PointCloud &output);

          double
          computePointWeight (const int pid, const std::vector<int> &indices, const std::vector<float> &distances);
    ...


In addition, we notice that lines 29-31 and 43 from section
:ref:`bilateral_filter_example` construct a :pcl:`KdTree<pcl::KdTree>`
structure for obtaining the nearest neighbors for a given point. We therefore
add:

.. code-block:: cpp
   :linenos:

    #include <pcl/kdtree/kdtree.h>
    ...
        using Filter<PointT>::input_;
        typedef typename Filter<PointT>::PointCloud PointCloud;
        typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;

      public:
    ...

        void
        setSearchMethod (const KdTreePtr &tree)
        {
          tree_ = tree;
        }

      private:
    ...
        KdTreePtr tree_;
    ...


Finally, we would like to add the kernel method (`G (float x, float sigma)`)
inline so that we speed up the computation of the filter. Because the method is
only useful within the context of the algorithm, we will make it private. The
header file becomes:

.. code-block:: cpp
   :linenos:

    #ifndef PCL_FILTERS_BILATERAL_H_
    #define PCL_FILTERS_BILATERAL_H_

    #include <pcl/filters/filter.h>
    #include <pcl/kdtree/kdtree.h>

    namespace pcl
    {
      template<typename PointT>
      class BilateralFilter : public Filter<PointT>
      {
        using Filter<PointT>::input_;
        typedef typename Filter<PointT>::PointCloud PointCloud;
        typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;

        public:
          BilateralFilter () : sigma_s_ (0), 
                               sigma_r_ (std::numeric_limits<double>::max ())
          {
          }


          void
          applyFilter (PointCloud &output);

          double 
          computePointWeight (const int pid, const std::vector<int> &indices, const std::vector<float> &distances);

          void 
          setSigmaS (const double sigma_s)
          {
            sigma_s_ = sigma_s;
          }

          double 
          getSigmaS ()
          {
            return (sigma_s_);
          }

          void
          setSigmaR (const double sigma_r)
          {
            sigma_r_ = sigma_r;
          }

          double 
          getSigmaR ()
          {
            return (sigma_r_);
          }

          void
          setSearchMethod (const KdTreePtr &tree)
          {
            tree_ = tree;
          }


        private:

          inline double
          kernel (double x, double sigma)
          {
            return (exp (- (x*x)/(2*sigma*sigma)));
          }

          double sigma_s_;
          double sigma_r_;
          KdTreePtr tree_;
      };
    }

    #endif // PCL_FILTERS_BILATERAL_H_

bilateral.hpp
=============

There's two methods that we need to implement here, namely `applyFilter` and
`computePointWeight`. 

.. code-block:: cpp
   :linenos:

    template <typename PointT> double
    pcl::BilateralFilter<PointT>::computePointWeight (const int pid, 
                                                      const std::vector<int> &indices,
                                                      const std::vector<float> &distances)
    {
      double BF = 0, W = 0;

      // For each neighbor
      for (size_t n_id = 0; n_id < indices.size (); ++n_id)
      {
        double id = indices[n_id];
        double dist = std::sqrt (distances[n_id]);
        double intensity_dist = abs (input_->points[pid].intensity - input_->points[id].intensity);

        double weight = kernel (dist, sigma_s_) * kernel (intensity_dist, sigma_r_);

        BF += weight * input_->points[id].intensity;
        W += weight;
      }
      return (BF / W);
    }

    template <typename PointT> void
    pcl::BilateralFilter<PointT>::applyFilter (PointCloud &output)
    {
      tree_->setInputCloud (input_);

      std::vector<int> k_indices;
      std::vector<float> k_distances;

      output = *input_;

      for (size_t point_id = 0; point_id < input_->points.size (); ++point_id)
      {
        tree_->radiusSearch (point_id, sigma_s_ * 2, k_indices, k_distances);

        output.points[point_id].intensity = computePointWeight (point_id, k_indices, k_distances);
      }
      
    }

The `computePointWeight` method should be straightforward as it's *almost
identical* to lines 45-58 from section :ref:`bilateral_filter_example`. We
basically pass in a point index that we want to compute the intensity weight
for, and a set of neighboring points with distances.

In `applyFilter`, we first set the input data in the tree, copy all the input
data into the output, and then proceed at computing the new weighted point
intensities.

Looking back at :ref:`filling`, it's now time to declare the `PCL_INSTANTIATE`
entry for the class:

.. code-block:: cpp
   :linenos:

    #ifndef PCL_FILTERS_BILATERAL_IMPL_H_
    #define PCL_FILTERS_BILATERAL_IMPL_H_

    #include <pcl/filters/bilateral.h>

    ...

    #define PCL_INSTANTIATE_BilateralFilter(T) template class PCL_EXPORTS pcl::BilateralFilter<T>;

    #endif // PCL_FILTERS_BILATERAL_H_

One additional thing that we can do is error checking on:

 * whether the two `sigma_s_` and `sigma_r_` parameters have been given;
 * whether the search method object (i.e., `tree_`) has been set.

For the former, we're going to check the value of `sigma_s_`, which was set to
a default of 0, and has a critical importance for the behavior of the algorithm
(it basically defines the size of the support region). Therefore, if at the
execution of the code, its value is still 0, we will print an error using the
:pcl:`PCL_ERROR<PCL_ERROR>` macro, and return.

In the case of the search method, we can either do the same, or be clever and
provide a default option for the user. The best default options are:

 * use an organized search method via :pcl:`pcl::OrganizedNeighbor<pcl::OrganizedNeighbor>` if the point cloud is organized;
 * use a general purpose kdtree via :pcl:`pcl::KdTreeFLANN<pcl::KdTreeFLANN>` if the point cloud is unorganized.

.. code-block:: cpp
   :linenos:

    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/kdtree/organized_data.h>

    ...
    template <typename PointT> void
    pcl::BilateralFilter<PointT>::applyFilter (PointCloud &output)
    {
      if (sigma_s_ == 0)
      {
        PCL_ERROR ("[pcl::BilateralFilter::applyFilter] Need a sigma_s value given before continuing.\n");
        return;
      }
      if (!tree_)
      {
        if (input_->isOrganized ())
          tree_.reset (new pcl::OrganizedNeighbor<PointT> ());
        else
          tree_.reset (new pcl::KdTreeFLANN<PointT> (false));
      }
      tree_->setInputCloud (input_);
    ...

The implementation file header thus becomes:

.. code-block:: cpp
   :linenos:

    #ifndef PCL_FILTERS_BILATERAL_IMPL_H_
    #define PCL_FILTERS_BILATERAL_IMPL_H_

    #include <pcl/filters/bilateral.h>
    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/kdtree/organized_data.h>

    template <typename PointT> double
    pcl::BilateralFilter<PointT>::computePointWeight (const int pid, 
                                                      const std::vector<int> &indices,
                                                      const std::vector<float> &distances)
    {
      double BF = 0, W = 0;

      // For each neighbor
      for (size_t n_id = 0; n_id < indices.size (); ++n_id)
      {
        double id = indices[n_id];
        double dist = std::sqrt (distances[n_id]);
        double intensity_dist = abs (input_->points[pid].intensity - input_->points[id].intensity);

        double weight = kernel (dist, sigma_s_) * kernel (intensity_dist, sigma_r_);

        BF += weight * input_->points[id].intensity;
        W += weight;
      }
      return (BF / W);
    }

    template <typename PointT> void
    pcl::BilateralFilter<PointT>::applyFilter (PointCloud &output)
    {
      if (sigma_s_ == 0)
      {
        PCL_ERROR ("[pcl::BilateralFilter::applyFilter] Need a sigma_s value given before continuing.\n");
        return;
      }
      if (!tree_)
      {
        if (input_->isOrganized ())
          tree_.reset (new pcl::OrganizedNeighbor<PointT> ());
        else
          tree_.reset (new pcl::KdTreeFLANN<PointT> (false));
      }
      tree_->setInputCloud (input_);

      std::vector<int> k_indices;
      std::vector<float> k_distances;

      output = *input_;

      for (size_t point_id = 0; point_id < input_->points.size (); ++point_id)
      {
        tree_->radiusSearch (point_id, sigma_s_ * 2, k_indices, k_distances);

        output.points[point_id].intensity = computePointWeight (point_id, k_indices, k_distances);
      }
    }
     
    #define PCL_INSTANTIATE_BilateralFilter(T) template class PCL_EXPORTS pcl::BilateralFilter<T>;

    #endif // PCL_FILTERS_BILATERAL_H_


Taking advantage of other PCL concepts
--------------------------------------

Point indices
=============

The standard way of passing point cloud data into PCL algorithms is via
:pcl:`setInputCloud<pcl::PCLBase::setInputCloud>` calls. In addition, PCL also
defines a way to define a region of interest / *list of point indices* that the
algorithm should operate on, rather than the entire cloud, via
:pcl:`setIndices<pcl::PCLBase::setIndices>`.

All classes inheriting from :pcl:`PCLBase<pcl::PCLBase>` exhbit the following
behavior: in case no set of indices is given by the user, a fake one is created
once and used for the duration of the algorithm. This means that we could
easily change the implementation code above to operate on a *<cloud, indices>*
tuple, which has the added advantage that if the user does pass a set of
indices, only those will be used, and if not, the entire cloud will be used.

The new *bilateral.hpp* class thus becomes:

.. code-block:: cpp
   :linenos:

    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/kdtree/organized_data.h>

    ...
    template <typename PointT> void
    pcl::BilateralFilter<PointT>::applyFilter (PointCloud &output)
    {
      if (sigma_s_ == 0)
      {
        PCL_ERROR ("[pcl::BilateralFilter::applyFilter] Need a sigma_s value given before continuing.\n");
        return;
      }
      if (!tree_)
      {
        if (input_->isOrganized ())
          tree_.reset (new pcl::OrganizedNeighbor<PointT> ());
        else
          tree_.reset (new pcl::KdTreeFLANN<PointT> (false));
      }
      tree_->setInputCloud (input_);
    ...

The implementation file header thus becomes:

.. code-block:: cpp
   :linenos:

    #ifndef PCL_FILTERS_BILATERAL_IMPL_H_
    #define PCL_FILTERS_BILATERAL_IMPL_H_

    #include <pcl/filters/bilateral.h>
    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/kdtree/organized_data.h>

    template <typename PointT> double
    pcl::BilateralFilter<PointT>::computePointWeight (const int pid, 
                                                      const std::vector<int> &indices,
                                                      const std::vector<float> &distances)
    {
      double BF = 0, W = 0;

      // For each neighbor
      for (size_t n_id = 0; n_id < indices.size (); ++n_id)
      {
        double id = indices[n_id];
        double dist = std::sqrt (distances[n_id]);
        double intensity_dist = abs (input_->points[pid].intensity - input_->points[id].intensity);

        double weight = kernel (dist, sigma_s_) * kernel (intensity_dist, sigma_r_);

        BF += weight * input_->points[id].intensity;
        W += weight;
      }
      return (BF / W);
    }

    template <typename PointT> void
    pcl::BilateralFilter<PointT>::applyFilter (PointCloud &output)
    {
      if (sigma_s_ == 0)
      {
        PCL_ERROR ("[pcl::BilateralFilter::applyFilter] Need a sigma_s value given before continuing.\n");
        return;
      }
      if (!tree_)
      {
        if (input_->isOrganized ())
          tree_.reset (new pcl::OrganizedNeighbor<PointT> ());
        else
          tree_.reset (new pcl::KdTreeFLANN<PointT> (false));
      }
      tree_->setInputCloud (input_);

      std::vector<int> k_indices;
      std::vector<float> k_distances;

      output = *input_;

      for (size_t i = 0; i < indices_->size (); ++i)
      {
        tree_->radiusSearch ((*indices_)[i], sigma_s_ * 2, k_indices, k_distances);

        output.points[(*indices_)[i]].intensity = computePointWeight ((*indices_)[i], k_indices, k_distances);
      }
    }
     
    #define PCL_INSTANTIATE_BilateralFilter(T) template class PCL_EXPORTS pcl::BilateralFilter<T>;

    #endif // PCL_FILTERS_BILATERAL_H_

To make :pcl:`indices_<pcl::PCLBase::indices_>` work without typing the full
construct, we need to add a new line to *bilateral.h* that specifies the class
where `indices_` is declared:

.. code-block:: cpp
   :linenos:

    ...
      template<typename PointT>
      class BilateralFilter : public Filter<PointT>
      {
        using Filter<PointT>::input_;
        using Filter<PointT>::indices_;
        public:
          BilateralFilter () : sigma_s_ (0),
    ...



Licenses
========

It is advised that each file contains a license that describes the author of
the code. This is very useful for our users that need to understand what sort
of restrictions are they bound to when using the code. PCL is 100% **BSD
licensed**, and we insert the corpus of the license as a C++ comment in the
file, as follows:

.. code-block:: cpp
   :linenos:

    /*
     * Software License Agreement (BSD License)
     *
     *  Point Cloud Library (PCL) - www.pointclouds.org
     *  Copyright (c) 2010-2011, Willow Garage, Inc.
     *
     *  All rights reserved.
     *
     *  Redistribution and use in source and binary forms, with or without
     *  modification, are permitted provided that the following conditions
     *  are met:
     *
     *   * Redistributions of source code must retain the above copyright
     *     notice, this list of conditions and the following disclaimer.
     *   * Redistributions in binary form must reproduce the above
     *     copyright notice, this list of conditions and the following
     *     disclaimer in the documentation and/or other materials provided
     *     with the distribution.
     *   * Neither the name of Willow Garage, Inc. nor the names of its
     *     contributors may be used to endorse or promote products derived
     *     from this software without specific prior written permission.
     *
     *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
     *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
     *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     *  POSSIBILITY OF SUCH DAMAGE.
     *
     */

An additional like can be inserted if additional copyright is needed (or the
original copyright can be changed):

.. code-block:: cpp
   :linenos:

    * Copyright (c) XXX, respective authors.

Proper naming
=============

We wrote the tutorial so far by using *silly named* setters and getters in our
example, like `setSigmaS` or `setSigmaR`. In reality, we would like to use a
better naming scheme, that actually represents what the parameter is doing. In
a final version of the code we could therefore rename the setters and getters
to `set/getHalfSize` and `set/getStdDev` or something similar.

Code comments
=============

PCL is trying to maintain a *high standard* with respect to user and API
documentation. This sort of Doxygen documentation has been stripped from the
examples shown above. In reality, we would have had the *bilateral.h* header
class look like:

.. code-block:: cpp
   :linenos:

    /*
     * Software License Agreement (BSD License)
     *
     *  Point Cloud Library (PCL) - www.pointclouds.org
     *  Copyright (c) 2010-2011, Willow Garage, Inc.
     *
     *  All rights reserved.
     *
     *  Redistribution and use in source and binary forms, with or without
     *  modification, are permitted provided that the following conditions
     *  are met:
     *
     *   * Redistributions of source code must retain the above copyright
     *     notice, this list of conditions and the following disclaimer.
     *   * Redistributions in binary form must reproduce the above
     *     copyright notice, this list of conditions and the following
     *     disclaimer in the documentation and/or other materials provided
     *     with the distribution.
     *   * Neither the name of Willow Garage, Inc. nor the names of its
     *     contributors may be used to endorse or promote products derived
     *     from this software without specific prior written permission.
     *
     *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
     *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
     *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     *  POSSIBILITY OF SUCH DAMAGE.
     *
     */

    #ifndef PCL_FILTERS_BILATERAL_H_
    #define PCL_FILTERS_BILATERAL_H_

    #include <pcl/filters/filter.h>
    #include <pcl/kdtree/kdtree.h>

    namespace pcl
    {
      /** \brief A bilateral filter implementation for point cloud data. Uses the intensity data channel.
        * \note For more information please see 
        * <b>C. Tomasi and R. Manduchi. Bilateral Filtering for Gray and Color Images.
        * In Proceedings of the IEEE International Conference on Computer Vision,
        * 1998.</b>
        * \author Luca Penasa
        */
      template<typename PointT>
      class BilateralFilter : public Filter<PointT>
      {
        using Filter<PointT>::input_;
        using Filter<PointT>::indices_;
        typedef typename Filter<PointT>::PointCloud PointCloud;
        typedef typename pcl::KdTree<PointT>::Ptr KdTreePtr;

        public:
          /** \brief Constructor. 
            * Sets \ref sigma_s_ to 0 and \ref sigma_r_ to MAXDBL
            */
          BilateralFilter () : sigma_s_ (0), 
                               sigma_r_ (std::numeric_limits<double>::max ())
          {
          }


          /** \brief Filter the input data and store the results into output
            * \param[out] output the resultant point cloud message
            */
          void
          applyFilter (PointCloud &output);

          /** \brief Compute the intensity average for a single point
            * \param[in] pid the point index to compute the weight for
            * \param[in] indices the set of nearest neighor indices 
            * \param[in] distances the set of nearest neighbor distances
            * \return the intensity average at a given point index
            */
          double 
          computePointWeight (const int pid, const std::vector<int> &indices, const std::vector<float> &distances);

          /** \brief Set the half size of the Gaussian bilateral filter window.
            * \param[in] sigma_s the half size of the Gaussian bilateral filter window to use
            */
          inline void 
          setHalfSize (const double sigma_s)
          {
            sigma_s_ = sigma_s;
          }

          /** \brief Get the half size of the Gaussian bilateral filter window as set by the user. */
          double 
          getHalfSize ()
          {
            return (sigma_s_);
          }

          /** \brief Set the standard deviation parameter
            * \param[in] sigma_r the new standard deviation parameter
            */
          void
          setStdDev (const double sigma_r)
          {
            sigma_r_ = sigma_r;
          }

          /** \brief Get the value of the current standard deviation parameter of the bilateral filter. */
          double 
          getStdDev ()
          {
            return (sigma_r_);
          }

          /** \brief Provide a pointer to the search object.
            * \param[in] tree a pointer to the spatial search object.
            */
          void
          setSearchMethod (const KdTreePtr &tree)
          {
            tree_ = tree;
          }

        private:

          /** \brief The bilateral filter Gaussian distance kernel.
            * \param[in] x the spatial distance (distance or intensity)
            * \param[in] sigma standard deviation
            */
          inline double
          kernel (double x, double sigma)
          {
            return (exp (- (x*x)/(2*sigma*sigma)));
          }

          /** \brief The half size of the Gaussian bilateral filter window (e.g., spatial extents in Euclidean). */
          double sigma_s_;
          /** \brief The standard deviation of the bilateral filter (e.g., standard deviation in intensity). */
          double sigma_r_;

          /** \brief A pointer to the spatial search object. */
          KdTreePtr tree_;
      };
    }

    #endif // PCL_FILTERS_BILATERAL_H_

And the *bilateral.hpp* like:

.. code-block:: cpp
   :linenos:

    /*
     * Software License Agreement (BSD License)
     *
     *  Point Cloud Library (PCL) - www.pointclouds.org
     *  Copyright (c) 2010-2011, Willow Garage, Inc.
     *
     *  All rights reserved.
     *
     *  Redistribution and use in source and binary forms, with or without
     *  modification, are permitted provided that the following conditions
     *  are met:
     *
     *   * Redistributions of source code must retain the above copyright
     *     notice, this list of conditions and the following disclaimer.
     *   * Redistributions in binary form must reproduce the above
     *     copyright notice, this list of conditions and the following
     *     disclaimer in the documentation and/or other materials provided
     *     with the distribution.
     *   * Neither the name of Willow Garage, Inc. nor the names of its
     *     contributors may be used to endorse or promote products derived
     *     from this software without specific prior written permission.
     *
     *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
     *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
     *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
     *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
     *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
     *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
     *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
     *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
     *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
     *  POSSIBILITY OF SUCH DAMAGE.
     *
     */

    #ifndef PCL_FILTERS_BILATERAL_IMPL_H_
    #define PCL_FILTERS_BILATERAL_IMPL_H_

    #include <pcl/filters/bilateral.h>
    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/kdtree/organized_data.h>

    //////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT> double
    pcl::BilateralFilter<PointT>::computePointWeight (const int pid, 
                                                      const std::vector<int> &indices,
                                                      const std::vector<float> &distances)
    {
      double BF = 0, W = 0;

      // For each neighbor
      for (size_t n_id = 0; n_id < indices.size (); ++n_id)
      {
        double id = indices[n_id];
        // Compute the difference in intensity
        double intensity_dist = abs (input_->points[pid].intensity - input_->points[id].intensity);

        // Compute the Gaussian intensity weights both in Euclidean and in intensity space
        double dist = std::sqrt (distances[n_id]);
        double weight = kernel (dist, sigma_s_) * kernel (intensity_dist, sigma_r_);

        // Calculate the bilateral filter response
        BF += weight * input_->points[id].intensity;
        W += weight;
      }
      return (BF / W);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    template <typename PointT> void
    pcl::BilateralFilter<PointT>::applyFilter (PointCloud &output)
    {
      // Check if sigma_s has been given by the user
      if (sigma_s_ == 0)
      {
        PCL_ERROR ("[pcl::BilateralFilter::applyFilter] Need a sigma_s value given before continuing.\n");
        return;
      }
      // In case a search method has not been given, initialize it using some defaults
      if (!tree_)
      {
        // For organized datasets, use an OrganizedNeighbor
        if (input_->isOrganized ())
          tree_.reset (new pcl::OrganizedNeighbor<PointT> ());
        // For unorganized data, use a FLANN kdtree
        else
          tree_.reset (new pcl::KdTreeFLANN<PointT> (false));
      }
      tree_->setInputCloud (input_);

      std::vector<int> k_indices;
      std::vector<float> k_distances;

      // Copy the input data into the output
      output = *input_;

      // For all the indices given (equal to the entire cloud if none given)
      for (size_t i = 0; i < indices_->size (); ++i)
      {
        // Perform a radius search to find the nearest neighbors
        tree_->radiusSearch ((*indices_)[i], sigma_s_ * 2, k_indices, k_distances);

        // Overwrite the intensity value with the computed average
        output.points[(*indices_)[i]].intensity = computePointWeight ((*indices_)[i], k_indices, k_distances);
      }
    }
     
    #define PCL_INSTANTIATE_BilateralFilter(T) template class PCL_EXPORTS pcl::BilateralFilter<T>;

    #endif // PCL_FILTERS_BILATERAL_H_


Testing the new class
---------------------

Testing the new class is easy. We'll take the first code snippet example as
shown above, strip the algorithm, and make it use the `pcl::BilateralFilter`
class instead:

.. code-block:: cpp
   :linenos:

    #include <pcl/point_types.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/kdtree/kdtree_flann.h>
    #include <pcl/filters/bilateral.h>

    typedef pcl::PointXYZI PointT;

    int
    main (int argc, char *argv[])
    {
      std::string incloudfile = argv[1];
      std::string outcloudfile = argv[2];
      float sigma_s = atof (argv[3]);
      float sigma_r = atof (argv[4]);

      // Load cloud
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile (incloudfile.c_str (), *cloud);

      pcl::PointCloud<PointT> outcloud;

      // Set up KDTree
      pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);

      pcl::BilateralFilter<PointT> bf;
      bf.setInputCloud (cloud);
      bf.setSearchMethod (tree);
      bf.setHalfSize (sigma_s);
      bf.setStdDev (sigma_r);
      bf.filter (outcloud);

      // Save filtered output
      pcl::io::savePCDFile (outcloudfile.c_str (), outcloud);
      return (0);
    }

