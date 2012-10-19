#include <time.h>

#include "stopwatch.h"

long long Stopwatch::getTime()
{
  timespec ts;
  clock_gettime (CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

Stopwatch::Stopwatch()
{
  start_time = getTime ();
}

int Stopwatch::elapsedMs()
{
  return (getTime () - start_time) / 1000000;
}
