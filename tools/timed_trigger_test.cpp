#include <iostream>
#include <thread>
#include <pcl/common/time_trigger.h>
#include <pcl/common/time.h>

using namespace std::chrono_literals;
using namespace pcl;

double global_time;

void callback ()
{
  static double last_time = pcl::getTime ();
  double elapsed = pcl::getTime () - last_time;
  last_time = pcl::getTime ();
  std::cout << "global fn: " << pcl::getTime () - global_time << " :: " << elapsed << std::endl;

  std::this_thread::sleep_for(1ms);
}

class Dummy
{
  public:
    void myTimer ()
    {
      static double last_time = pcl::getTime ();
      double elapsed = pcl::getTime () - last_time;
      last_time = pcl::getTime ();
      std::cout << "member fn: " << pcl::getTime () - global_time << " :: " << elapsed << std::endl;
    }
};

int main ()
{
  TimeTrigger trigger (10.0, callback);
  Dummy dummy;
  global_time = pcl::getTime ();
  trigger.start ();
  std::this_thread::sleep_for(2s);
  trigger.registerCallback ([&]{ dummy.myTimer (); });
  std::this_thread::sleep_for(3s);
  trigger.setInterval (0.2);
  std::this_thread::sleep_for(2s);
  return 0;
}
