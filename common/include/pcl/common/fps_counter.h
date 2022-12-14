/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception Inc.
 *
 *  All rights reserved
 */

#pragma once

#include <pcl/common/time.h>

namespace pcl {

class fpsCounter {

  unsigned tick_;
  double start_time_;

  struct ret {
    double fps = 0, duration = 0;
  };

  void
  reset()
  {
    tick_ = start_time_ = 0;
  };

  double
  now() const
  {
    return pcl::getTime();
  };

protected:
  std::string what_;

public:
  fpsCounter(const std::string what) : what_(what), tick_(0) {}

  unsigned
  tick()
  {
    if (tick_ == 0) {
      start_time_ = now();
    }

    tick_ += 1;
    if (resetCondition()) {
      printCounterDetails();
      reset();
    }

    return tick_;
  };

  virtual bool
  resetCondition()
  {
    return getAvgFps().duration >= 1.0;
  };

  virtual void
  printCounterDetails()
  {
    std::cout << "Average framerate(" << what_ << "): " << getAvgFps().fps << " Hz"
              << std::endl;
  };

  unsigned
  getCount() const
  {
    return tick_;
  };

  ret
  getAvgFps() const
  {
    const double delta_t = now() - start_time_;
    const double fps = (tick_ - 1) / delta_t;
    return {fps, delta_t};
  };
};

} // namespace pcl

namespace pcl {

class fpsCounterConditional : public fpsCounter {

  unsigned reset_iter_;
  bool stop_computing_;

  bool
  resetCondition()
  {
    return getCount() >= reset_iter_;
  };

  void
  printCounterDetails()
  {
    std::cout << "Average framerate(" << what_ << "): " << getAvgFps().fps << " Hz"
              << std::endl;

    if (stop_computing_)
      std::cout << "Press 's' to start computing!\n";
  };

public:
  fpsCounterConditional(const std::string what,
                        const unsigned& reset_iter = 10,
                        const bool& stop_computing = false)
  : fpsCounter(what), reset_iter_(reset_iter), stop_computing_(stop_computing)
  {}
};

} // namespace pcl
