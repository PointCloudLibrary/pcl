The OpenNURBS library uses zlib for mesh and bitmap compression.
The zlib source code distributed with OpenNURBS is a subset of what
is available from zlib.  The zlib code itself has not been modified.
See http://www.zlib.net/ for more details.

If you are using opennurbs as a statically linked library, then
you may make calls to the same zlib that opennurbs uses.  All the
necessary header files are included by opennurbs.h.

If you are using opennurbs as a DLL or writing a Rhino plug-in
and you want to use the same zlib that opennurbs uses, then
compile opennurbs_zlib_memory.cpp into your application
and statically link with the same zlib that opennurbs.dll
links with. All the necessary header files are included by
opennurbs.h.

This zlib opennurbs uses is compiled with z_ symbol projectection
and compiles zlib with MY_ZCALLOC and Z_PREFIX defined.  This means
the zlib functions have z_ symbol protection and the zcalloc()
and zcfree() functions in opennurbs_zlib_memory.cpp are used.
See opennurbs_zlib.h for details.

Zlib has a generous license that is similar to the one for OpenNURBS.
The zlib license shown below was copied from the zlib web page
http://www.zlib.net/zlib_license.html on 6 October 2005.


  /* 
    zlib.h -- interface of the 'zlib' general purpose compression library
    version 1.2.2, October 3rd, 2004

    Copyright (C) 1995-2004 Jean-loup Gailly and Mark Adler

    This software is provided 'as-is', without any express or implied
    warranty.  In no event will the authors be held liable for any damages
    arising from the use of this software.

    Permission is granted to anyone to use this software for any purpose,
    including commercial applications, and to alter it and redistribute it
    freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not
       claim that you wrote the original software. If you use this software
       in a product, an acknowledgment in the product documentation would be
       appreciated but is not required.
    2. Altered source versions must be plainly marked as such, and must not be
       misrepresented as being the original software.
    3. This notice may not be removed or altered from any source distribution.

    Jean-loup Gailly jloup@gzip.org
    Mark Adler madler@alumni.caltech.edu
  */
