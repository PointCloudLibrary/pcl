.. _custom_executor:

Creating Custom Executors
--------------------------------------------------

Since executors have a standard interface, you can create custom executors
and use them.
PCL offers support for a few executors, namely:

1. Inline Executor
2. SSE Executor
3. OMP Executor
4. CUDA Executor

You can create a specialized version of these executors i.e., executors derived from
the one supported by PCL that adds some additional functionality.

In this tutorial, we will learn how to create an executor derived from the OMP executor,
that measures and reports the time taken by a functor filter to execute.

.. note::
   This tutorial is for advanced users and requires some knowledge on working of executors
   and its implementation in PCL.

The code
--------

First, create a file, let's say, ``custom_executor.cpp``, and place the following inside it:

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :linenos:

The explanation
---------------

Now, let's break down the code piece by piece.

Here, we forward declare our custom executor struct called `omp_benchmark_executor`.
This is required for the trait `is_executor_available` declared in the next few lines,
which is used inside our custom executor's definition.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 10-12

We mark the executor as available, by creating a specialization of `omp_benchmark_executor`
which inherits from `std::true_type`. This acts as an indicator that the system supports the
executor. You can wrap this inside a `#ifdef` and check for certain macros, which
indicate the presence of certain features like `_OPENMP` is used for OpenMP.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 14-18

Here, we define the structure for our custom executor, which we had forward declared earlier.
It is templated with the two properties it supports, which are `blocking_t` and `allocator_t`.
Since this is a specialization of OMP executor we inherit from `omp_executor`, this allows
us to use our custom executor wherever the OpenMP executor provided in PCL is supported.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 20-23

We need to introduce the base struct, i.e., `omp_executor` members into
our current struct. You can read more on why this is needed over
`here <https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members>`_.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 24-29

Our custom executor's special feature is the ability to time functions executed
by `bulk_execute`, so we need to override the function. We also perform checks on the executor's
availability.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 31-37

This is where we define what happens before and after we invoke our callable.
We measure the time before and after all the thread executes the code. We enclose all our code
in a parallel region with the specified maximum number of threads.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 39-43

We then measure the time taken by each thread. The callable is invoked in a loop which is
automatically parallelized using OpenMP.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 44-51

In the following lines, we create a Point Cloud structure for the input and output point clouds,
then fill the input cloud using `CloudGenerator`. The generator uses 128 as a seed value to uniformly
fill the input cloud with a point cloud having width & height as 200. Each point is generated
with x,y & z coordinates in the range [-20, 20].

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 59-65

We then create a FunctorFilter called `positive_filter` that filters out
any points which have negative coordinates.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 67-76

Finally, we create an instance of our custom executor `omp_benchmark_executor` and limit
the max number of threads to four. Then we call the `filter` function of `positive_filter` with
our custom executor. We repeat the same and limit the number of threads to one the
second time.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 78-85

.. note::
   Not all code inside the functor filter is executed by the executor. So it does not measure the
   time taken by the entire filter to execute and only measures the execution time, of the portion of
   the program that was executed by the executor.

   Refer to the implementation of FunctorFilter for more insight.

Compiling and running the program
---------------------------------

Add the following lines to your CMakeLists.txt file:

.. literalinclude:: sources/custom_executor/CMakeLists.txt
   :language: cmake
   :linenos:


After you have made the executable, you can run it. Simply do:

  $ ./custom_executor

You should get an output similar to this (the time taken will change based
on your system configuration):

.. code-block:: bash

   Filtering using 4 Threads
   Time taken by thread 0: took 2.97464ms.
   Time taken by thread 3: took 4.24397ms.
   Time taken by thread 2: took 4.26735ms.
   Time taken by thread 1: took 5.19864ms.
   Total time taken: took 5.44704ms.

   Filtering using 1 Thread
   Time taken by thread 0: took 9.38748ms.
   Total time taken: took 9.39384ms.

