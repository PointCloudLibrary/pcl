.. toctree::

The following presents a set of advanced topics regarding PCL.

Compiling PCL
-------------

PCL uses modern C++ template programming in order to achieve maximum generality
and reusability of its components. Due to intricate details of the current
generation of C++ compilers however, the usage of templated code introduces
additional compile-time delays. We present a series of tricks that, if used
appropriately, will save you a lot of headaches and will speed up the
compilation of your project.

.. Some useful references:
.. * http://developers.sun.com/solaris/articles/CC_perf/content.html
.. * http://www.drdobbs.com/blog/archives/2010/08/c_compilation_s.html
.. * http://gamesfromwithin.com/physical-structure-and-c-part-2-build-times

* :ref:`c_cache`

CCache is a compiler cache. It speeds up recompilation by caching previous
compilations and detecting when the same compilation is being done again.
Supported languages are C, C++, Objective-C and Objective-C++.
  
* :ref:`compiler_optimizations`

Depending on what compiler optimizations you use, your code might behave
differently, both at compile time and at run time.

* :ref:`single_compile_unit`

In certain cases, it's better to concatenate source files into single
compilation units to speed up compiling.

..
  Profiling your code


Contents
--------

.. toctree::
  
   c_cache
   compiler_optimizations
   single_compile_unit

