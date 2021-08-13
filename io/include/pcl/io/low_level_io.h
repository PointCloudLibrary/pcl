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

#pragma once

#ifdef _WIN32
# ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
# endif
# ifndef NOMINMAX
#  define NOMINMAX
# endif
# include <io.h>
# include <windows.h>
# ifdef _MSC_VER
// ssize_t is already defined in MinGW and its definition conflicts with that of
// SSIZE_T on a 32-bit target, so do this only for MSVC.
#  include <basetsd.h>
using ssize_t = SSIZE_T;
# endif /* _MSC_VER */
#else
# include <unistd.h>
# include <sys/mman.h>
# include <sys/types.h>
# include <sys/stat.h>
# include <sys/fcntl.h>
# include <cerrno>
#endif
#include <cstddef>

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

    inline int raw_read(int fd, void * buffer, std::size_t count)
    {
      return ::_read(fd, buffer, count);
    }

    inline int raw_write(int fd, const void * buffer, std::size_t count)
    {
      return ::_write(fd, buffer, count);
    }

    inline int raw_ftruncate(int fd, long length)
    {
      return ::_chsize(fd, length);
    }

    inline int raw_fallocate(int fd, long length)
    {
      // Doesn't actually allocate, but best we can do?
      return raw_ftruncate(fd, length);
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

    inline ssize_t raw_read(int fd, void * buffer, std::size_t count)
    {
      return ::read(fd, buffer, count);
    }

    inline ssize_t raw_write(int fd, const void * buffer, std::size_t count)
    {
      return ::write(fd, buffer, count);
    }

    inline int raw_ftruncate(int fd, off_t length)
    {
      return ::ftruncate(fd, length);
    }

# ifdef __APPLE__
    inline int raw_fallocate(int fd, off_t length)
    {
      // OS X doesn't have posix_fallocate, but it has a fcntl that does the job.
      // It may make the file too big though, so we truncate before returning.

      // Try to allocate contiguous space first.
      ::fstore_t store = {F_ALLOCATEALL | F_ALLOCATECONTIG, F_PEOFPOSMODE, 0, length, 0};
      if (::fcntl(fd, F_PREALLOCATE, &store) != -1)
        return raw_ftruncate(fd, length);

      // Try fragmented if it failed.
      store.fst_flags = F_ALLOCATEALL;
      if (::fcntl(fd, F_PREALLOCATE, &store) != -1)
        return raw_ftruncate(fd, length);

      // Fragmented also failed.
      return -1;
    }

# else // __APPLE__
    inline int raw_fallocate(int fd, off_t length)
    {
#  ifdef ANDROID
      // Android's libc doesn't have posix_fallocate.
      if (::fallocate(fd, 0, 0, length) == 0)
        return 0;
#  else
      // Conforming POSIX systems have posix_fallocate.
      if (::posix_fallocate(fd, 0, length) == 0)
        return 0;
#  endif

      // EINVAL should indicate an unsupported filesystem.
      // All other errors are passed up.
      if (errno != EINVAL)
        return -1;

      // Try to deal with unsupported filesystems by simply seeking + writing.
      // This may not really allocate space, but the file size will be set.
      // Writes to the mmapped file may still trigger SIGBUS errors later.

      // Remember the old position and seek to the desired length.
      off_t old_pos = raw_lseek(fd, 0, SEEK_CUR);
      if (old_pos == -1)
        return -1;
      if (raw_lseek(fd, length - 1, SEEK_SET) == -1)
        return -1;

      // Write a single byte to resize the file.
      char buffer = 0;
      ssize_t written = raw_write(fd, &buffer, 1);

      // Seek back to the old position.
      if (raw_lseek(fd, old_pos, SEEK_SET) == -1)
        return -1;

      // Fail if we didn't write exactly one byte,
      if (written != 1)
        return -1;

      return 0;
    }
# endif // __APPLE
#endif // _WIN32

  }
}
