#ifndef TIMER_H_
#define TIMER_H_

#include <ctime>

template <int bins>
class Timer {
public:

  Timer();

  /** mark the beginning of the timing */
  void start();

  /** the amount of time between now and the last call to start() is added to the specified bin */
  void stop(int bin);

  /** access the bin total, in seconds */
  double operator[](int bin);

  /** storage of bin totals */
  clock_t total[bins];

  /** time when start() was called */
  clock_t mark;

};

#include "proctor/impl/timer.hpp"

#endif //#ifndef TIMER_H_
