.. _c_cache:

Using CCache to speed up compilation
------------------------------------

`CCache <http://ccache.samba.org/>`_ is nothing more than a cache for your
compiler. ccache is usually very easy to install. Here's an example for Ubuntu
systems::

  sudo apt-get install ccache


``ccache`` will cache previous compilations, detect when the same compilation
is being done again, and reuse its cache instead of recompiling the source code
again. This can speed up your compilation by many orders of magnitude,
especially in those situations where your file timestamps change, and ``make``
is triggering a recompile.

To enable ccache, simply add '/usr/lib/ccache' to the beginning of your PATH.
This directory contains symlinks to ccache, and ccache is smart enough to
look at the name of the calling executable to determine which real executable
to run. I.e. there is a symlink from '/usr/lib/ccache/g++' to just 'ccache',
but it actually runs the equivalent of 'ccache g++'.

Using colorgcc to colorize output
---------------------------------
`colorgcc <https://github.com/johannes/colorgcc>`_ is a colorizer for the output
of GCC, and allows you to better interpret the compiler warnings/errors.

To enable both colorgcc and ccache, perform the following steps:

Install ``colorgcc`` on an Ubuntu system with ::

  sudo apt-get install colorgcc

To enable colorgcc, perform the following steps:

.. code-block:: cmake

  cp /etc/colorgcc/colorgccrc $HOME/.colorgccrc


* edit the $HOME/.colorgccrc file, search for the following lines:

.. code-block:: cmake

    g++: /usr/bin/g++
    gcc: /usr/bin/gcc
    c++: /usr/bin/g++
    cc:  /usr/bin/gcc
    g77: /usr/bin/g77
    f77: /usr/bin/g77
    gcj: /usr/bin/gcj
    
and replace them with:

.. code-block:: cmake

    g++: ccache /usr/bin/g++
    gcc: ccache /usr/bin/gcc
    c++: ccache /usr/bin/g++
    cc:  ccache /usr/bin/gcc
    g77: ccache /usr/bin/g77
    f77: ccache /usr/bin/g77
    gcj: ccache /usr/bin/gcj

* create a $HOME/bin or $HOME/sbin directory, and create the following softlinks in it

.. code-block:: cmake

    ln -s /usr/bin/colorgcc c++
    ln -s /usr/bin/colorgcc cc
    ln -s /usr/bin/colorgcc g++
    ln -s /usr/bin/colorgcc gcc

make sure that $HOME/bin or $HOME/sbin is the first directory in your $PATH, e.g.::

    export PATH=$HOME/bin:$PATH

or::

    export PATH=$HOME/sbin:$PATH

depending on where you stored the ``colorgcc`` softlinks, so that when
cc/gcc/g++/c++ is invoked the freshly created softlinks get activated first and
not the global /usr/bin/{cc,gcc,g++,c++}.

