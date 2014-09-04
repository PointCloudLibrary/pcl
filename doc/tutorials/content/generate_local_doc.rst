.. _generate_local_doc:

======================================
Generate a local documentation for PCL
======================================

For practical reasons you might want to have a local documentation which corresponds to your 
PCL version. In this tutorial you will learn how to generate it and how to set up Apache so that 
the search bar works.

This tutorial was written for Ubuntu 12.04 and 14.04, feel free to edit it on GitHub to add your platform. 

Dependencies
============

You need to install a few dependencies in order to be able to generate the documentation::

  $ sudo apt-get install doxygen graphviz sphinx3 python-pip
  $ sudo pip install sphinxcontrib-doxylink

Generate the documentation
==========================

Go into the build folder of PCL where you've configured it (`see tutorial <http://www.pointclouds.org/downloads/source.html>`_) and enter::

  $ make doc

Then you can open the documentation with your browser, for example::

  $ firefox doc/doxygen/html/index.html 

The documentation has been generated in your PCL build directory but it is not installed; if you wish to install it just do::

  $ sudo make install

The default PCL ``CMAKE_INSTALL_PREFIX`` is ``/usr/local``, this means the documentation will be located in ``/usr/local/share/doc/pcl-1.7/html/index.html``

.. note::
  You will quickly notice that the search bar doesn't work! (searching opens "search.php" instead of searching)

Installing and configuring Apache
=================================

Apache (`The Apache HTTP Server <https://en.wikipedia.org/wiki/Apache_HTTP_Server>`_) is a web server application, in this section you will 
learn how to configure Apache in order to be able to use the search feature within your offline documentation.

First you need to install Apache and php::

  $ sudo apt-get install apache2 php5 libapache2-mod-php5

Then you need to edit the default website location::

  $ sudo gedit /etc/apache2/sites-available/000-default.conf

Change ``DocumentRoot`` (default = ``/var/www/html``) to ``/usr/local/share/doc/pcl-1.7/html/`` (or your local PCL doc build path) 

After that change the Apache directory options::

  $ sudo gedit +153 /etc/apache2/apache2.conf

Replace the paragraph at line 153 with::

  <Directory />
      #Options FollowSymLinks
      Options Indexes FollowSymLinks Includes ExecCGI
      AllowOverride All
      Order deny,allow
      Allow from all
  </Directory>

Restart Apache and the search bar will now work if you open ``localhost``::

  $ sudo /etc/init.d/apache2 restart
  $ firefox localhost
