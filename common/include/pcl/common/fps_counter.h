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

class FpsCounter {
public:
  FpsCounter(unsigned counter_type = 0, unsigned reset_iter = 10)
  : counter_type_(counter_type)
  , reset_iter_(reset_iter)
  , count_(0)
  , start_time_(now())
  , duration_(0.0)
  , last_reset_(now())
  {}

  unsigned counter_type_;
  unsigned reset_iter_;
  unsigned count_;
  double start_time_;
  double duration_;
  double last_reset_;

  unsigned
  getCount()
  {
    return count_;
  }; // returns the number of frames counted till now
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
  FpsCounterConditional(auto fps_calc,
                        auto reset_condition,
                        auto print_details,
                        auto reset,
                        auto update_duration)
  : fps_calc(fps_calc)
  , reset_condition(reset_condition)
  , print_details(print_details)
  , reset(reset)
  , update_duration(update_duration)
  {}

  void
  tick()
  {

    double time_now = now();
    double fps = fps_calc(time_now);

    if (reset_condition(time_now)) {
      print_details(fps);
      reset();
    }
    else {
      update_duration(time_now);
    }
  };

private:
  std::function<double(const double&)> fps_calc;
  std::function<bool(const double&)> reset_condition;
  std::function<void(const float&)> print_details;
  std::function<void()> reset;
  std::function<void(const double&)> update_duration;
};

auto simpleFpsCounter = [](pcl::FpsCounter& fps_counter,
                           const std::string& what,
                           const bool& stop_computing = false) {
  if (fps_counter.counter_type_ == 2 && what == "start") {
    fps_counter.start_time_ = fps_counter.now();
    return;
  }

  auto fps_calc = [&](const double& now) {
    fps_counter.count_++;

    if (fps_counter.counter_type_ != 2)
      return double(fps_counter.count_) / double(now - fps_counter.last_reset_);
    else
      return double(fps_counter.count_ - 1) / double(fps_counter.duration_);
  };

  auto print_details = [&](const float& fps) {
    std::cout << "Average framerate(" << what << "): " << fps << " Hz" << std::endl;

    if (stop_computing)
      std::cout << "Press 's' to start computing!\n";
  };

  auto reset_condition = [&](const double& now) {
    if (fps_counter.counter_type_ == 0)
      return (now - fps_counter.last_reset_) >= 1.0;
    else
      return fps_counter.count_ == fps_counter.reset_iter_;
  };

  auto reset = [&]() {
    fps_counter.count_ = 0;
    fps_counter.last_reset_ = fps_counter.now();
    fps_counter.duration_ = 0.0;
  };

  auto update_duration = [&](const double& now) {
    fps_counter.duration_ += now - fps_counter.start_time_;
  };

  return FpsCounterConditional(
             fps_calc, reset_condition, print_details, reset, update_duration)
      .tick();
};

} // namespace pcl
