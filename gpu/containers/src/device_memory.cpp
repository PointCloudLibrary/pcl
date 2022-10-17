/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#include <pcl/gpu/containers/device_memory.h>
#include <pcl/gpu/utils/safe_call.hpp>
#include <pcl/pcl_config.h> // used for HAVE_CUDA
#include <pcl/pcl_macros.h> // used for PCL_DEPRECATED

#include <cuda_runtime_api.h>

#include <cassert>

#if !defined(HAVE_CUDA)

void
throw_nogpu()
{
  throw "PCL 2.0 exception";
}

pcl::gpu::DeviceMemory::DeviceMemory() { throw_nogpu(); }

pcl::gpu::DeviceMemory::DeviceMemory(void*, std::size_t) { throw_nogpu(); }

pcl::gpu::DeviceMemory::DeviceMemory(std::size_t) { throw_nogpu(); }

pcl::gpu::DeviceMemory::~DeviceMemory() { throw_nogpu(); }

pcl::gpu::DeviceMemory::DeviceMemory(const DeviceMemory&) { throw_nogpu(); }

pcl::gpu::DeviceMemory&

pcl::gpu::DeviceMemory::operator=(const pcl::gpu::DeviceMemory&)
{
  throw_nogpu();
  return *this;
}

void
pcl::gpu::DeviceMemory::create(std::size_t)
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory::release()
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory::copyTo(DeviceMemory&) const
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory::upload(const void*, std::size_t)
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory::download(void*) const
{
  throw_nogpu();
}

bool
pcl::gpu::DeviceMemory::empty() const
{
  throw_nogpu();
}

pcl::gpu::DeviceMemory2D::DeviceMemory2D() { throw_nogpu(); }

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int, int) { throw_nogpu(); }

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int, int, void*, std::size_t)
{
  throw_nogpu();
}

pcl::gpu::DeviceMemory2D::~DeviceMemory2D() { throw_nogpu(); }

pcl::gpu::DeviceMemory2D::DeviceMemory2D(const DeviceMemory2D&) { throw_nogpu(); }

pcl::gpu::DeviceMemory2D&

pcl::gpu::DeviceMemory2D::operator=(const pcl::gpu::DeviceMemory2D&)
{
  throw_nogpu();
  return *this;
}

void
pcl::gpu::DeviceMemory2D::create(int, int)
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory2D::release()
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory2D::copyTo(DeviceMemory2D&) const
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory2D::upload(const void*, std::size_t, int, int)
{
  throw_nogpu();
}

void
pcl::gpu::DeviceMemory2D::download(void*, std::size_t) const
{
  throw_nogpu();
}

bool
pcl::gpu::DeviceMemory2D::empty() const
{
  throw_nogpu();
}

#else

//////////////////////////    XADD    ///////////////////////////////

template <typename _Tp>
PCL_DEPRECATED(1, 16, "Removed in favour of c++11 atomics")
static inline _Tp CV_XADD(std::atomic<_Tp>* addr, std::atomic<_Tp> delta)
{
  _Tp tmp = addr->fetch_add(delta);
  return tmp;
}

////////////////////////    DeviceArray    /////////////////////////////

pcl::gpu::DeviceMemory::DeviceMemory()
: data_(nullptr), sizeBytes_(0), refcount_(nullptr)
{}

pcl::gpu::DeviceMemory::DeviceMemory(void* ptr_arg, std::size_t sizeBytes_arg)
: data_(ptr_arg), sizeBytes_(sizeBytes_arg), refcount_(nullptr)
{}

pcl::gpu::DeviceMemory::DeviceMemory(std::size_t sizeBtes_arg)
: data_(nullptr), sizeBytes_(0), refcount_(nullptr)
{
  create(sizeBtes_arg);
}

pcl::gpu::DeviceMemory::~DeviceMemory() { release(); }

pcl::gpu::DeviceMemory::DeviceMemory(const DeviceMemory& other_arg)
: data_(other_arg.data_)
, sizeBytes_(other_arg.sizeBytes_)
, refcount_(other_arg.refcount_)
{
  if (refcount_)
    refcount_->fetch_add(1);
}

pcl::gpu::DeviceMemory&
pcl::gpu::DeviceMemory::operator=(const pcl::gpu::DeviceMemory& other_arg)
{
  if (this != &other_arg) {
    if (other_arg.refcount_)
      other_arg.refcount_->fetch_add(1);
    release();

    data_ = other_arg.data_;
    sizeBytes_ = other_arg.sizeBytes_;
    refcount_ = other_arg.refcount_;
  }
  return *this;
}

void
pcl::gpu::DeviceMemory::create(std::size_t sizeBytes_arg)
{
  if (sizeBytes_arg == sizeBytes_)
    return;

  if (sizeBytes_arg > 0) {
    if (data_)
      release();

    sizeBytes_ = sizeBytes_arg;

    cudaSafeCall(cudaMalloc(&data_, sizeBytes_));

    refcount_ = new std::atomic<int>(1);
  }
}

void
pcl::gpu::DeviceMemory::copyTo(DeviceMemory& other) const
{
  if (empty())
    other.release();
  else {
    other.create(sizeBytes_);
    cudaSafeCall(cudaMemcpy(other.data_, data_, sizeBytes_, cudaMemcpyDeviceToDevice));
    cudaSafeCall(cudaDeviceSynchronize());
  }
}

void
pcl::gpu::DeviceMemory::release()
{
  if (refcount_ && refcount_->fetch_sub(1) == 1) {
    delete refcount_;
    cudaSafeCall(cudaFree(data_));
  }
  data_ = nullptr;
  sizeBytes_ = 0;
  refcount_ = nullptr;
}

void
pcl::gpu::DeviceMemory::upload(const void* host_ptr_arg, std::size_t sizeBytes_arg)
{
  create(sizeBytes_arg);
  cudaSafeCall(cudaMemcpy(data_, host_ptr_arg, sizeBytes_, cudaMemcpyHostToDevice));
  cudaSafeCall(cudaDeviceSynchronize());
}

bool
pcl::gpu::DeviceMemory::upload(const void* host_ptr_arg,
                               std::size_t device_begin_byte_offset,
                               std::size_t num_bytes)
{
  if (device_begin_byte_offset + num_bytes > sizeBytes_) {
    return false;
  }
  void* begin = static_cast<char*>(data_) + device_begin_byte_offset;
  cudaSafeCall(cudaMemcpy(begin, host_ptr_arg, num_bytes, cudaMemcpyHostToDevice));
  cudaSafeCall(cudaDeviceSynchronize());
  return true;
}

void
pcl::gpu::DeviceMemory::download(void* host_ptr_arg) const
{
  cudaSafeCall(cudaMemcpy(host_ptr_arg, data_, sizeBytes_, cudaMemcpyDeviceToHost));
  cudaSafeCall(cudaDeviceSynchronize());
}

bool
pcl::gpu::DeviceMemory::download(void* host_ptr_arg,
                                 std::size_t device_begin_byte_offset,
                                 std::size_t num_bytes) const
{
  if (device_begin_byte_offset + num_bytes > sizeBytes_) {
    return false;
  }
  const void* begin = static_cast<char*>(data_) + device_begin_byte_offset;
  cudaSafeCall(cudaMemcpy(host_ptr_arg, begin, num_bytes, cudaMemcpyDeviceToHost));
  cudaSafeCall(cudaDeviceSynchronize());
  return true;
}

void
pcl::gpu::DeviceMemory::swap(DeviceMemory& other_arg)
{
  std::swap(data_, other_arg.data_);
  std::swap(sizeBytes_, other_arg.sizeBytes_);
  std::swap(refcount_, other_arg.refcount_);
}

bool
pcl::gpu::DeviceMemory::empty() const
{
  return !data_;
}
size_t
pcl::gpu::DeviceMemory::sizeBytes() const
{
  return sizeBytes_;
}

////////////////////////    DeviceArray2D    /////////////////////////////

pcl::gpu::DeviceMemory2D::DeviceMemory2D()
: data_(nullptr), step_(0), colsBytes_(0), rows_(0), refcount_(nullptr)
{}

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int rows_arg, int colsBytes_arg)
: data_(nullptr), step_(0), colsBytes_(0), rows_(0), refcount_(nullptr)
{
  create(rows_arg, colsBytes_arg);
}

pcl::gpu::DeviceMemory2D::DeviceMemory2D(int rows_arg,
                                         int colsBytes_arg,
                                         void* data_arg,
                                         std::size_t step_arg)
: data_(data_arg)
, step_(step_arg)
, colsBytes_(colsBytes_arg)
, rows_(rows_arg)
, refcount_(nullptr)
{}

pcl::gpu::DeviceMemory2D::~DeviceMemory2D() { release(); }

pcl::gpu::DeviceMemory2D::DeviceMemory2D(const DeviceMemory2D& other_arg)
: data_(other_arg.data_)
, step_(other_arg.step_)
, colsBytes_(other_arg.colsBytes_)
, rows_(other_arg.rows_)
, refcount_(other_arg.refcount_)
{
  if (refcount_)
    refcount_->fetch_add(1);
}

pcl::gpu::DeviceMemory2D&
pcl::gpu::DeviceMemory2D::operator=(const pcl::gpu::DeviceMemory2D& other_arg)
{
  if (this != &other_arg) {
    if (other_arg.refcount_)
      other_arg.refcount_->fetch_add(1);
    release();

    colsBytes_ = other_arg.colsBytes_;
    rows_ = other_arg.rows_;
    data_ = other_arg.data_;
    step_ = other_arg.step_;

    refcount_ = other_arg.refcount_;
  }
  return *this;
}

void
pcl::gpu::DeviceMemory2D::create(int rows_arg, int colsBytes_arg)
{
  if (colsBytes_ == colsBytes_arg && rows_ == rows_arg)
    return;

  if (rows_arg > 0 && colsBytes_arg > 0) {
    if (data_)
      release();

    colsBytes_ = colsBytes_arg;
    rows_ = rows_arg;

    cudaSafeCall(cudaMallocPitch((void**)&data_, &step_, colsBytes_, rows_));

    refcount_ = new std::atomic<int>(1);
  }
}

void
pcl::gpu::DeviceMemory2D::release()
{
  if (refcount_ && refcount_->fetch_sub(1) == 1) {
    delete refcount_;
    cudaSafeCall(cudaFree(data_));
  }

  colsBytes_ = 0;
  rows_ = 0;
  data_ = nullptr;
  step_ = 0;
  refcount_ = nullptr;
}

void
pcl::gpu::DeviceMemory2D::copyTo(DeviceMemory2D& other) const
{
  if (empty())
    other.release();
  else {
    other.create(rows_, colsBytes_);
    cudaSafeCall(cudaMemcpy2D(other.data_,
                              other.step_,
                              data_,
                              step_,
                              colsBytes_,
                              rows_,
                              cudaMemcpyDeviceToDevice));
    cudaSafeCall(cudaDeviceSynchronize());
  }
}

void
pcl::gpu::DeviceMemory2D::upload(const void* host_ptr_arg,
                                 std::size_t host_step_arg,
                                 int rows_arg,
                                 int colsBytes_arg)
{
  create(rows_arg, colsBytes_arg);
  cudaSafeCall(cudaMemcpy2D(data_,
                            step_,
                            host_ptr_arg,
                            host_step_arg,
                            colsBytes_,
                            rows_,
                            cudaMemcpyHostToDevice));
  cudaSafeCall(cudaDeviceSynchronize());
}

void
pcl::gpu::DeviceMemory2D::download(void* host_ptr_arg, std::size_t host_step_arg) const
{
  cudaSafeCall(cudaMemcpy2D(host_ptr_arg,
                            host_step_arg,
                            data_,
                            step_,
                            colsBytes_,
                            rows_,
                            cudaMemcpyDeviceToHost));
  cudaSafeCall(cudaDeviceSynchronize());
}

void
pcl::gpu::DeviceMemory2D::swap(DeviceMemory2D& other_arg)
{
  std::swap(data_, other_arg.data_);
  std::swap(step_, other_arg.step_);

  std::swap(colsBytes_, other_arg.colsBytes_);
  std::swap(rows_, other_arg.rows_);
  std::swap(refcount_, other_arg.refcount_);
}

bool
pcl::gpu::DeviceMemory2D::empty() const
{
  return !data_;
}

int
pcl::gpu::DeviceMemory2D::colsBytes() const
{
  return colsBytes_;
}

int
pcl::gpu::DeviceMemory2D::rows() const
{
  return rows_;
}

size_t
pcl::gpu::DeviceMemory2D::step() const
{
  return step_;
}

#endif
