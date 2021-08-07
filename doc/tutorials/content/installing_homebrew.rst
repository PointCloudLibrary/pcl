.. _installing_homebrew:

Installing on Mac OS X using Homebrew
=====================================

This tutorial explains how to install the Point Cloud Library on Mac OS
X using Homebrew.

.. image:: images/macosx_logo.png
   :alt: Mac OS X logo
   :align: right

.. _homebrew_preqs:

Prerequisites
=============

You will need to have Homebrew installed. If you do not already have a Homebrew installation, see the
`Homebrew homepage`_ for installation instructions.

.. _`Homebrew homepage`:
   http://brew.sh/

.. _homebrew_all:

Using the formula
=================

The PCL formula is in the Homebrew official repositories.
This will automatically install all necessary dependencies and provides options for controlling
which parts of PCL are installed.

.. note::

   To prepare it, follow these steps:


   #. Install Homebrew. See the Homebrew website for instructions.
   #. Execute ``brew update``.

To install the latest version using the formula, execute the following command::

  $ brew install pcl

You can specify options to control which parts of PCL are installed. For
example, to build just the libraries without extra dependencies, execute the following command::

  $ brew install pcl --without-apps --without-tools --without-vtk --without-qt

For a full list of the available options, see the formula's help::

  $ brew options pcl

Once PCL is installed, you may wish to periodically upgrade it. Update
Homebrew and, if a PCL update is available, upgrade::

  $ brew update
  $ brew upgrade pcl

Using PCL
---------

Now that PCL in installed, you can start using the library in your own
projects by following the :ref:`using_pcl_pcl_config` tutorial.
