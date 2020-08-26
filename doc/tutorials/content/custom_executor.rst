.. _custom_executor:

Creating Custom Executors
--------------------------------------------------

Since executors have a standard interface, you can create your own executors
and use them.
PCL offers support for a few executors, namely:

1. Inline Executor
2. SSE Executor
3. OMP Executor
4. CUDA Executor

You can create specialized version of these executors i.e. executors derived from
the executors supported by PCL, that add some additional functionality.

In this tutorial we will learn how to create an executor derived from OMP executor,
that measures and reports the time taken by a functor filter to execute.

.. note::
   This is an advanced topic and requires some knowledge on working of executors
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
This is required se the trait `is_executor_available` declared in the next few lines,
is needed inside the definition of our custom executor.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 8-10


We mark the executor as available, by creating a specialization of `omp_benchmark_executor`
which inherits from `std::true_type`. This acts as an indicator that the system supports the
executor. You can wrap this inside a `#ifdef` and check for the presence of certain macros which
indicate the presence of certain features, like `_OPENMP` is used for OpenMP.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 12-16

Here, we define the structure for our custom executor which we had forward declared earlier.
It is templated with the 2 properties it supports which are `blocking_t` and `allocator_t`.
Since this is a specialization of OMP executor we inherit from `omp_executor`, this allows
us to use our custom executor wherever the OpenMP executor provided in PCL is supported.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 18-21

We need to introduce the base struct i.e. `omp_executor` members into
our current struct. You can read more on why this is needed over
`here <https://isocpp.org/wiki/faq/templates#nondependent-name-lookup-members>`_.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 22-26

The special feature that our custom executor offers is the ability to time functions executed
by `bulk_execute`, so we need to override the function. We also perform checks on the availability of
the executor and limiting the number of threads to the max limit defined in the executor.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 28-35

This is the where we define what happens before and after we invoke our callable.
First, we measure the time, before executing any code. Then we create a parallel region
with the specified number of threads.
We then measure the time at the beginning of execution in each thread and then measure the time
after invoking the callable and print the difference.
While invoking the callable we pass the index, which contains the total number of threads and
the index of the thread on which the code is being executed.


.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 37-51

We measure the time after all the threads have finished executing and print
the difference from the initially measured time.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 53-55

In the following lines, we create a Point Cloud structure, then fill the
input cloud using `CloudGenerator`. The generator uses 128 as seed value to uniformly fill
the input cloud with a point cloud having width & height as 200. Each point is generated
with x,y & z co-ordinates in range [-20, 20].

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 62-65

We then create a FunctorFilter called `positive_filter` that filters out
any points which have negative co-ordinates.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 67-74

Finally, we create in instance of our custom executor `omp_benchmark_executor` and limit
the max number of threads to 4. Then we call the `filter` function of `positive_filter` with
the our custom executor.

.. literalinclude:: sources/custom_executor/custom_executor.cpp
   :language: cpp
   :lines: 77-80

.. note::
   Not all code inside the functor filter is executed by the executor, only the loop
   which filters the points is executed by the executor.
   So it does not measure the time take by the entire filter to execute, and only measures
   the time of the portion of the program which the code was executed by the executor.
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

   Time taken by Thread: 0 is 3531028 ns
   Time taken by Thread: 2 is 3652374 ns
   Time taken by Thread: 1 is 3700053 ns
   Time taken by Thread: 3 is 4297594 ns
   Total time taken: 4446868 ns

