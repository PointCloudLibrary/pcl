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

#include <functional>

namespace pcl {

struct FpsCounterDetails {

  unsigned counter_type_;
  unsigned reset_iter_;
  double fps_;
  unsigned count_;
  double start_time_;
  double duration_;
  double last_reset_;
  FpsCounterDetails(unsigned counter_type = 0, unsigned reset_iter = 10)
  : counter_type_(counter_type)
  , reset_iter_(reset_iter)
  , start_time_(pcl::getTime())
  , last_reset_(pcl::getTime())
  {}
};

class FpsCounter {
public:
  FpsCounter() {}
  FpsCounter(FpsCounterDetails& details) : details_(&details) {}

  FpsCounterDetails* details_;

  void
  calculateFps(const unsigned& ticks, const double& elapsed_time)
  {
    details_->fps_ = double(ticks) / double(elapsed_time);
  };

  unsigned
  getCount()
  {
    return details_->count_;
  }; // returns the number of frames counted till now
  double
  getFps()
  {
    return details_->fps_;
  };
  void
  tick(); // add a frame
  double
  now()
  {
    return pcl::getTime();
  }; // appropriate return for the time
};

class FpsCounterConditional : public FpsCounter {
public:
  FpsCounterConditional(pcl::FpsCounterDetails& fps_counter_details,
                        std::function<void()> print_counter_details)
  : FpsCounter(fps_counter_details), print_counter_details(print_counter_details)
  {}

  void
  tick()
  {

    double time_now = now();

    details_->count_++;

    if (details_->counter_type_ != 2)
      calculateFps(details_->count_, time_now - details_->last_reset_);
    else
      calculateFps(details_->count_ - 1, details_->duration_);

    if (resetCondition(time_now)) {
      print_counter_details();
      resetFpsCounter(time_now);
    }
    else {
      updateElapsedTime(time_now);
    }
  };

private:
  std::function<void()> print_counter_details;

  bool
  resetCondition(const double& now)
  {
    if (details_->counter_type_ == 0)
      return (now - details_->last_reset_) >= 1.0;
    else
      return details_->count_ == details_->reset_iter_;
  };

  void
  resetFpsCounter(const double& now)
  {
    details_->count_ = 0;
    details_->last_reset_ = now;
    details_->duration_ = 0.0;
  };

  void
  updateElapsedTime(const double& now)
  {
    details_->duration_ += now - details_->start_time_;
  };
};

auto simpleFpsCounter = [](pcl::FpsCounterDetails& fps_counter_details,
                           const std::string& what,
                           const bool& stop_computing = false) {
  if (fps_counter_details.counter_type_ == 2 && what == "start") {
    fps_counter_details.start_time_ = pcl::getTime();
    return;
  }

  auto print_counter_details = [&]() {
    std::cout << "Average framerate(" << what << "): " << fps_counter_details.fps_
              << " Hz" << std::endl;

    if (stop_computing)
      std::cout << "Press 's' to start computing!\n";
  };

  return FpsCounterConditional(fps_counter_details, print_counter_details).tick();
};

} // namespace pcl
