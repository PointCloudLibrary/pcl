.. _pcl_style_guide:

PCL C++ Programming Style Guide
-------------------------------

To make sure that all code in PCL is coherent and easily understood by other
developers and users, we follow a set of strict rules that everyone should
adopt. These rules are not to be broken unless there is a very good reason to
do so. Changes to these rules are always possible, but the person proposing and
changing a rule will have the unfortunate task to go and apply the rule change
to all the existing code.

.. contents:: Table of Contents
   :backlinks: none
   :local:

1. Naming
=========

1.1. Files
^^^^^^^^^^

All files should be **under_scored**.

* Header files have the extension **.h**
* Templated implementation files have the extension **.hpp**
* Source files have the extension **.cpp**

1.2. Directories
^^^^^^^^^^^^^^^^

All directories and subdirectories should be **under_scored**.

* Header files should go under **include/**
* Templated implementation files should go under **include/impl/**
* Source files should go under **src/**

1.3. Includes
^^^^^^^^^^^^^

Include statements are made with **"quotes"** only if the file is in the
same directory, in any other case the include statement is made with
**<chevron_brackets>**, e.g.:

 .. code-block:: cpp

  #include <pcl/module_name/file_name.h>
  #include <pcl/module_name/impl/file_name.hpp>

1.4. Defines & Macros
^^^^^^^^^^^^^^^^^^^^^

Macros should all be **ALL_CAPITALS_AND_UNDERSCORED**.

Include guards are not implemented with defines, instead ``#pragma once`` should be used.

 .. code-block:: cpp

  // the license

  #pragma once

  // the code

1.5. Namespaces
^^^^^^^^^^^^^^^

Namespaces should be **under_scored**, e.g.:

 .. code-block:: cpp

  namespace pcl_io
  {
    ...
  }

1.6. Classes / Structs
^^^^^^^^^^^^^^^^^^^^^^

Class names (and other type names) should be **CamelCased**.
Exception: if the class name contains a short acronym, the acronym itself
should be all capitals. Class and struct names are preferably **nouns**:
PFHEstimation instead of EstimatePFH.

Correct examples:

 .. code-block:: cpp

  class ExampleClass;
  class PFHEstimation;

1.7. Functions / Methods
^^^^^^^^^^^^^^^^^^^^^^^^

Functions and class method names should be **camelCased**, and arguments are
**under_scored**. Function and method names are preferably **verbs**, and the name
should make clear what it does: checkForErrors() instead of errorCheck(),
dumpDataToFile() instead of dataFile().

Correct usage:

 .. code-block:: cpp

  int 
  applyExample (int example_arg);

1.8. Variables
^^^^^^^^^^^^^^

Variable names should be **under_scored**.

 .. code-block:: cpp

  int my_variable;

1.8.1. Iterators
""""""""""""""""

Iterator variables should indicate what they're iterating over, e.g.:

 .. code-block:: cpp

  std::list<int> pid_list;
  std::list<int>::iterator pid_it;

1.8.2. Constants
""""""""""""""""

Constants should be **ALL_CAPITALS**, e.g.:

 .. code-block:: cpp

  const static int MY_CONSTANT = 1000;

1.8.3. Member variables
"""""""""""""""""""""""

Variables that are members of a class are **under_scored_**, with a trailing
underscore added, e.g.:

 .. code-block:: cpp

  int example_int_;

1.9. Return statements
^^^^^^^^^^^^^^^^^^^^^^

Return statements should have their values in parentheses, e.g.:

 .. code-block:: cpp

  int
  main ()
  {
    return (0);
  }

|

2. Indentation and Formatting
=============================

The standard indentation for each block in PCL is **2 spaces**. Under no
circumstances, tabs or other spacing measures should be used. PCL uses a
variant of the GNU style formatting. 

2.1. Namespaces
^^^^^^^^^^^^^^^

In both header and implementation files, namespaces are to be explicitly
declared, and their contents should not be indented, like clang-format
enforces in the Formatting CI job, e.g.:

.. code-block:: cpp

  namespace pcl
  {

  class Foo
  {
    ...
  };

  }


2.2. Classes
^^^^^^^^^^^^

The template parameters of a class should be declared on a different line,
e.g.:

.. code-block:: cpp

   template <typename T>
   class Foo
   {
     ...
   }

2.3. Functions / Methods
^^^^^^^^^^^^^^^^^^^^^^^^

The return type of each function declaration must be placed on a different
line, e.g.:

.. code-block:: cpp

   void
   bar ();

Same for the implementation/definition, e.g.:

.. code-block:: cpp

   void
   bar ()
   {
     ...
   }

or

.. code-block:: cpp

   void
   Foo::bar ()
   {
     ...
   }

or

.. code-block:: cpp

   template <typename T> void
   Foo<T>::bar ()
   {
     ...
   }

2.4. Braces
^^^^^^^^^^^

Braces, both open and close, go on their own lines, e.g.:

.. code-block:: cpp

   if (a < b)
   {
     ...
   }
   else
   {
     ...
   }

Braces can be omitted if the enclosed block is a single-line statement, e.g.:

.. code-block:: cpp

   if (a < b)
     x = 2 * a;

2.5. Spacing
^^^^^^^^^^^^

We'll say it again: the standard indentation for each block in PCL is **2
spaces**. We also include a space before the bracketed list of arguments to a
function/method, e.g.:

.. code-block:: cpp

   int 
   exampleMethod (int example_arg);


Class and struct members are indented by **2 spaces**. Access qualifiers (public, private and protected) are put at the
indentation level of the class body and members affected by these qualifiers are indented by one more level, i.e. 2 spaces. E.g.:

.. code-block:: cpp

   namespace foo
   {

   class Bar
   {
     int i;
     public:
       int j;
     protected:
       void
       baz ();
   };
   }


2.6. Automatic code formatting
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

We currently use clang-format-10 as the tool for auto-formatting our C++ code.
Please note that different versions of clang-format can result in slightly different outputs.

The style rules mentioned in this document are enforced via `PCL's .clang-format file
<https://github.com/PointCloudLibrary/pcl/blob/master/.clang-format>`_.
The style files which were previously distributed should now be considered deprecated.

For the integration of clang-format with various text editors and IDE's, refer to this `page
<https://clang.llvm.org/docs/ClangFormat.html>`_.

Details about the style options used can be found `here
<https://clang.llvm.org/docs/ClangFormatStyleOptions.html>`_.

2.6.1. Script usage
"""""""""""""""""""

PCL also creates a build target 'format' to format the whitelisted directories using clang-format.

Command line usage:

.. code-block:: shell

   $ make format


2.7. Includes
^^^^^^^^^^^^^

For consistent usage, headers should be included in the following order with alphabetical grouping ensured:

1.  PCL headers

    i.  All modular PCL includes, except main includes of common module.
        
        Examples:

        .. code-block:: cpp

           #include <pcl/common/common.h>
           #include <pcl/simulation/camera.h>
           #include <pcl/ml/dt/decision_forest.h>

    #.  The main PCL includes of common module. These are the header files in the ``pcl/common/include/pcl/`` directory.
    
        Examples:

        .. code-block:: cpp

           #include <pcl/memory.h>
           #include <pcl/pcl_macros.h>
           #include <pcl/point_cloud.h>

2.  Major 3rd-Party components of tests and modules

    i.  gtest
    #.  boost
    #.  Eigen
    #.  flann
3.  Major 3rd-Party components of apps

    i.  Qt
    #.  ui-files
    #.  vtk
4.  Minor 3rd-Party components

    i.  librealsense
    #.  ros/message_filters
    #.  opencv/opencv2
    #.  tide
    #.  thrust
    #.  OpenGL, GL & GLUT
5.  C++ standard library headers (alphabetical)
6.  Others

This style can also be enforced via clang-format. For usage instructions, refer `2.6. Automatic code formatting`_.


3. Structuring
==============

3.1. Classes and API
^^^^^^^^^^^^^^^^^^^^

For most classes in PCL, it is preferred that the interface (all public
members) does not contain variables and only two types of methods:

* The first method type is the get/set type that allows to manipulate the
  parameters and input data used by the class.
* The second type of methods is actually performing the class functionality
  and produces output, e.g. compute, filter, segment.

3.2. Passing arguments
^^^^^^^^^^^^^^^^^^^^^^

For get/set type methods the following rules apply:

* If large amounts of data needs to be set (usually the case with input data
  in PCL) it is preferred to pass a boost shared pointer instead of the actual
  data.
* Getters always need to pass exactly the same types as their repsective setters
  and vice versa.
* For getters, if only one argument needs to be passed this will be done via
  the return keyword. If two or more arguments need to be passed they will
  all be passed by reference instead.

For the compute, filter, segment, etc. type methods the following rules apply:

* The output arguments are preferably non-pointer type, regardless of data
  size.
* The output arguments will always be passed by reference.

3.3. Object declaration
^^^^^^^^^^^^^^^^^^^^^^^

3.3.1 Use of auto
"""""""""""""""""
* For Iterators auto must be used as much as possible 
* In all the other cases auto can be used at the author's discretion
* Use const auto references by default in range loops. Drop the const if the item needs to be modified.

3.3.2 Type qualifiers of variables
""""""""""""""""""""""""""""""""""
* Declare variables const when they don't need to be modified.
* Use const references whenever you don't need a copy of the variable. 
* Use of unsigned variables if the value is sure to not go negative by 
  use and by definition of the variable
