/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
 *  Copyright (c) 2018 Fizyr BV. - https://fizyr.com
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * This file defines compatibility wrappers for low level I/O functions.
 * Implemented as inlinable functions to prevent any performance overhead.
 */

#ifndef __PCL_IO_LOW_LEVEL_IO__
#define __PCL_IO_LOW_LEVEL_IO__

#ifdef _WIN32
# ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
# endif
# ifndef NOMINMAX
#  define NOMINMAX
# endif
# include <io.h>
# include <windows.h>
# include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#else
# include <unistd.h>
# include <sys/mman.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <sys/fcntl.h>
#endif

namespace pcl
{
  namespace io
  {
#ifdef _WIN32
    inline int raw_open(const char * pathname, int flags, int mode)
    {
      return ::_open(pathname, flags, mode);
    }

    inline int raw_open(const char * pathname, int flags)
    {
      return ::_open(pathname, flags);
    }

    inline int raw_close(int fd)
    {
      return ::_close(fd);
    }

    inline int raw_lseek(int fd, long offset, int whence)
    {
      return ::_lseek(fd, offset, whence);
    }

    inline int raw_read(int fd, void * buffer, size_t count)
    {
      return ::_read(fd, buffer, count);
    }

    inline int raw_write(int fd, const void * buffer, size_t count)
    {
      return ::_write(fd, buffer, count);
    }

    inline int raw_fallocate(int fd, long len)
    {
      return ::_chsize(fd, len);
    }
#else
    inline int raw_open(const char * pathname, int flags, int mode)
    {
      return ::open(pathname, flags, mode);
    }

    inline int raw_open(const char * pathname, int flags)
    {
      return ::open(pathname, flags);
    }

    inline int raw_close(int fd)
    {
      return ::close(fd);
    }

    inline off_t raw_lseek(int fd, off_t offset, int whence)
    {
      return ::lseek(fd, offset, whence);
    }

    inline ssize_t raw_read(int fd, void * buffer, size_t count)
    {
      return ::read(fd, buffer, count);
    }

    inline ssize_t raw_write(int fd, const void * buffer, size_t count)
    {
      return ::write(fd, buffer, count);
    }

# ifndef __APPLE__
    inline int raw_fallocate(int fd, off_t len)
    {
      return ::posix_fallocate(fd, 0, len);
    }
# else
    inline int raw_fallocate(int fd, off_t len)
    {
      // Try to allocate contiguous space first.
      ::fstore_t store = {F_ALLOCATEALL | F_ALLOCATECONTIG, F_PEOFPOSMODE, 0, len};
      if (::fcntl(fd, F_PREALLOCATE, &store) < 0)
      {
        // Try fragmented if that failed.
        store.fst_flags = F_ALLOCATEALL;
        int ret = ::fcntl(fd, F_PREALLOCATE, &store);

        // Bail if it still failed.
        if (ret < 0) {
          return ret;
        }
      }

      // File could be larger than requested, so truncate.
      return ::ftruncate(fd, len);
    }
# endif // __APPLE__
#endif // _WIN32

  }
}
#endif // __PCL_IO_LOW_LEVEL_IO__
