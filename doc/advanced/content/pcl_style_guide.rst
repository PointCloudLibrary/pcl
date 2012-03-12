.. _pcl_style_guide:

PCL C++ Programming Style Guide
-------------------------------

To make sure that all code in PCL is coherent and easily understood by other
developers and users, we follow a set of strict rules that everyone should
adopt. These rules are not to be broken unless there is a very good reason to
do so. Changes to these rules are always possible, but the person proposing and
changing a rule will have the unfortunate task to go and apply the rule change
to all the existing code.

Chapter 1 -- Naming
===================

1.1 Files
^^^^^^^^^

All files should be **under_scored**.

 * Header files have the extension **.h**
 * Templated implementation files have the extension **.hpp**
 * Source files have the extension **.cpp**


1.2 Directories
^^^^^^^^^^^^^^^

All directories and subdirectories should be **under_scored**.

 * Header files should go under **include/**
 * Templated implementation files should go under **include/impl/**
 * Source files should go under **src/**


1.3 Classes / Structs
^^^^^^^^^^^^^^^^^^^^^

Class names (and other type names) should be **CamelCased**, e.g.:

.. code-block:: cpp

  class ExampleClass;

Exception: if the class name contains a short acronym, the acronym itself
should be all capitals, e.g.:

.. code-block:: cpp

  class PFHEstimation;


1.4 Functions / Methods
^^^^^^^^^^^^^^^^^^^^^^^

Functions and class method names should be **camelCased**, and arguments are
**under_scored**, e.g.:

.. code-block:: cpp

  int 
  exampleMethod (int example_arg);

Functions and methods usually performs an action, so the name should make clear
what it does: checkForErrors() instead of errorCheck(), dumpDataToFile()
instead of dataFile(). Classes are often nouns. By making function names verbs
and following other naming conventions programs can be read more naturally.


1.5 Variables
^^^^^^^^^^^^^

Variable names should be **under_scored**.

.. code-block:: cpp

  int my_variable;

1.5.1 Iterators
"""""""""""""""

Iterator variables should indicate what they're iterating over, e.g.:

.. code-block:: cpp

  std::list<int> pid_list;
  std::list<int>::iterator pid_it;

1.5.2 Constants
"""""""""""""""

Constants should be **ALL_CAPITALS**, e.g.:

.. code-block:: cpp

  const static int MY_CONSTANT = 1000;

1.5.3 Member variables
""""""""""""""""""""""

Variables that are members of a class are **under_scored**, with a trailing
underscore added, e.g.:

.. code-block:: cpp

  int example_int_;


1.6 Namespaces
^^^^^^^^^^^^^^

Namespaces should be **under_scored**, e.g.:

.. code-block:: cpp

  namespace pcl_io
  {
    ...
  }

1.7 Return statements
^^^^^^^^^^^^^^^^^^^^^

Return statements should have their values in parentheses, e.g.:

.. code-block:: cpp

  int
  main ()
  {
    return (0);
  }


1.8 Includes
^^^^^^^^^^^^

Include statements are made with quotes only if the header file is in the same directory, in any other case the include statement is
made as:

.. code-block:: cpp

  #include <pcl/library/file.h>
  #incluce <pcl/library/impl_file.hpp>

Chapter 2 - Indentation and Formatting
======================================

The standard indentation for each block in PCL is **2 spaces**. Under no
circumstances, tabs or other spacing measures should be used. PCL uses a
variant of the GNU style formatting. 

2.1 Namespaces
^^^^^^^^^^^^^^

In a header file, the contets of a namespace should be indented, e.g.:

.. code-block:: cpp

  namespace pcl
  {
    class Foo
    {
      ...
    };
  }

In an implementation file, the namespace must be added to each individual method or function definition, e.g.:

.. code-block:: cpp

  void
  pcl::Foo::bar ()
  {
    ...
  }


2.2 Classes
^^^^^^^^^^^

The template parameters of a class should be declared on a different line,
e.g.:

.. code-block:: cpp

   template <typename T>
   class Foo
   {
     ...
   }

2.3 Functions / Methods
^^^^^^^^^^^^^^^^^^^^^^^

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

2.4 Braces
^^^^^^^^^^

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

2.5 Spacing
^^^^^^^^^^^

We'll say it again: the standard indentation for each block in PCL is **2
spaces**. We also include a space before the bracketed list of arguments to a
function/method, e.g.:

.. code-block:: cpp

   int 
   exampleMethod (int example_arg);


If multiple namespaces are declared within header files, always use **2
spaces** to indent them, e.g.:

.. code-block:: cpp

   namespace foo
   {
     namespace bar
     {
        void
        method (int my_var);
      }
   }


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
     }
   }


2.6 Automatic code indentation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The following set of rules can be automatically used by various different IDEs,
editors, etc.

2.6.1 Eclipse
"""""""""""""

Please us the following `formatting XML file
<http://dev.pointclouds.org/attachments/download/25/pcl_eclipse_formatting.xml>`_,
download it to some known location, and then:

* start Eclipse
* select **Window -> Preferences -> C/C++ -> Code Style**
* click **Import...**
* select ``pcl_eclipse_formatting.xml`` from the location you saved it to
* click **OK**

As you edit a file, Eclipse should use this new profile to format your code
following the PCL conventions. To reformat an entire file, select **Edit ->
Format**.

2.6.2 Uncrustify
""""""""""""""""

You can find a config for `Uncrustify <http://uncrustify.sourceforge.net/>`_
`here <http://dev.pointclouds.org/attachments/download/537/uncrustify.cfg>`_

2.6.3 Emacs
"""""""""""
You can use the following `PCL C/C++ style file
<http://dev.pointclouds.org/attachments/download/748/pcl-c-style.el>`_,
download it to some known location and then:

* open .emacs 
* add the following before any C/C++ custom hooks

.. code-block:: lisp

   (load-file "/location/to/pcl-c-style.el")
   (add-hook 'c-mode-common-hook 'pcl-set-c-style)

2.6.4 UniversalIndent
"""""""""""""""""""""

TBD...

