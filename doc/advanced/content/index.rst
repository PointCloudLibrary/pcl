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

.. // commented out
.. // Some useful references:
.. // * http://developers.sun.com/solaris/articles/CC_perf/content.html
.. // * http://www.drdobbs.com/blog/archives/2010/08/c_compilation_s.html
.. // * http://gamesfromwithin.com/physical-structure-and-c-part-2-build-times

:ref:`c_cache`

CCache is a compiler cache. It speeds up recompilation by caching previous
compilations and detecting when the same compilation is being done again.
Supported languages are C, C++, Objective-C and Objective-C++.
  
:ref:`compiler_optimizations`

Depending on what compiler optimizations you use, your code might behave
differently, both at compile time and at run time.

:ref:`single_compile_unit`

In certain cases, it's better to concatenate source files into single
compilation units to speed up compiling.

Developing PCL code
-------------------

To make our lives easier, and to be able to read and integrate code from each
other without causing ourselves headaches, we assembled a set of rules for PCL
development that everyone should follow:

 * if you make any commits, please add the commit log or something similar to
   the changelist page
   (http://dev.pointclouds.org/projects/pcl/wiki/ChangeList);
   
 * if you change anything in an existing algorithm, make sure that there are
   unit tests for it and make sure that they pass before you commit the code;

 * if you add a new algorithm or method, please document the code in a similar
   manner to the existing PCL code (or better!), and add some minimal unit
   tests before you commit it;

 * method definitions go into (include/.h), templated implementations go into
   (include/impl/.hpp), non-templated implementations go into (src/.cpp), and
   unit tests go in (test/.cpp);

 * last but not least, please respect the same naming and indentation
   guidelines as you see in the :ref:`pcl_style_guide`.


:ref:`pcl_style_guide`

Please follow the following naming and indentation rules when developing code
for PCL.


..
  Profiling your code


Contents
--------

.. toctree::
  
   c_cache
   compiler_optimizations
   single_compile_unit
   pcl_style_guide

