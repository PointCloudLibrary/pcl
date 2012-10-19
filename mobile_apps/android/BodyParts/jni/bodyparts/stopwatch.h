#ifndef STOPWATCH_H_
#define STOPWATCH_H_

class Stopwatch
{
private:
  long long start_time;
  static long long getTime();

public:
  Stopwatch();
  int elapsedMs();
};

#endif // STOPWATCH_H_
