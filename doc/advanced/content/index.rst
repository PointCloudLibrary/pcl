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

* :ref:`c_cache`

  `CCache` is a compiler cache. It speeds up recompilation by caching previous
  compilations and detecting when the same compilation is being done again.
  Supported languages are C, C++, Objective-C and Objective-C++.

  .. image:: images/ccache.png
     :height: 75px


* :ref:`distc`

  `distcc` is a program to distribute builds of C, C++, Objective C or
  Objective C++ code across several machines on a network. distcc should always
  generate the same results as a local build, is simple to install and use, and
  is usually much faster than a local compile. 
    
  .. image:: images/distcc.png
     :height: 75px


* :ref:`compiler_optimizations`

  Depending on what compiler optimizations you use, your code might behave
  differently, both at compile time and at run time.

  .. image:: images/optimize.png
     :height: 75px


* :ref:`single_compile_unit`

  In certain cases, it's better to concatenate source files into single
  compilation units to speed up compiling.

  .. image:: images/unitybuild.jpg
     :height: 75px

Developing PCL code
-------------------

To make our lives easier, and to be able to read and integrate code from each
other without causing ourselves headaches, we assembled a set of rules for PCL
development that everyone should follow:

.. topic:: Rules

 * if you make important commits, please **_add the commit log_** or something similar **_to
   the changelist page_**
   (https://github.com/PointCloudLibrary/pcl/blob/master/CHANGES.md);
   
 * if you change anything in an existing algorithm, **_make sure that there are
   unit tests_** for it and **_make sure that they pass before you commit_** the code;

 * if you add a new algorithm or method, please **_document the code in a similar
   manner to the existing PCL code_** (or better!), and **_add some minimal unit
   tests_** before you commit it;

 * method definitions go into (include/.h), templated implementations go into
   (include/impl/.hpp), non-templated implementations go into (src/.cpp), and
   unit tests go in (test/.cpp);

 * last but not least, please **_respect the same naming and indentation
   guidelines_** as you see in the :ref:`pcl_style_guide`.


* :ref:`pcl_style_guide`

  Please follow the following naming and indentation rules when developing code for PCL.

* :ref:`exceptions_guide`

  Short documentation on how to add new, throw and handle exceptions in PCL.

* :ref:`pcl2`

  An in-depth discussion about the PCL 2.x API can be found here.

Commiting changes to the git master
-----------------------------------
In order to oversee the commit messages more easier and that the changelist looks homogenous please keep the following format:

"* <fixed|bugfix|changed|new> X in @<classname>@ (#<bug number>)" 

Improving the PCL documentation
-------------------------------

* :ref:`how_to_write_a_tutorial`

  In case you want to contribute/help PCL by improving the existing
  documentation and tutorials/examples, please read our short guide on how to
  start.

How to build a minimal example
------------------------------

* :ref:`minimal_example`

  In case you need help to debug your code, please follow this guidelines to write a minimal example.

