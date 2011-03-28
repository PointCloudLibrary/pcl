.. _compiler_optimizations:

Compiler optimizations
----------------------

Using excessive compiler optimizations can really hurt your compile-time
performance, and there's a question whether you really need these optimizations
everytime you recompile to prototype something new, or whether you can live
with a less optimal binary for testing things. Obviously once your tests
succeed and you want to deploy your project, you can simply re-enable the
compiler optimizations. Here's a few tests that we did a while back with
`pcl_ros <http://pcl.ros.org/>`_::

  -j1, RelWithDebInfo + O3 : 3m20.376s -j1, RelWithDebInfo : 2m48.064s
  -j1, Debug : 2m0.452s
  -j2, Debug : 1m8.151s
  -j4, Debug : 0m42.846s

In general, we got used to enable all compiler optimizations possible. In PCL
pre-0.4, this is how the CMakeLists.txt file looked like::

  add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
  #add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
  add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)

Obviously, not all those flags were enabled by default, but we were definitely
playing around with them, and sometimes committing them to the repository,
which led to increase compilation times for some of the projects that needed to
precompile/use PCL.

In general there is no good rule of thumb here, but we decided to disable these
excessive optimizations by default, and rely on CMake's *RelWithDebInfo* by
default. You should do the same too when you prototype.

