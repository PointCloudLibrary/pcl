#include <iostream>
#include <pcl/common/time_trigger.h>
#include <pcl/common/time.h>
#include <pcl/visualization/boost.h>

using namespace std;
using namespace pcl;

double global_time;

void callback ()
{
  static double last_time = pcl::getTime ();
  double elapsed = pcl::getTime () - last_time;
  last_time = pcl::getTime ();
  cout << "global fn: " << pcl::getTime () - global_time << " :: " << elapsed << endl;
  boost::this_thread::sleep(boost::posix_time::microseconds(1000));
}

class Dummy
{
  public:
    void myTimer ()
    {
      static double last_time = pcl::getTime ();
      double elapsed = pcl::getTime () - last_time;
      last_time = pcl::getTime ();
      cout << "member fn: " << pcl::getTime () - global_time << " :: " << elapsed << endl;
    }
};

int main ()
{
  TimeTrigger trigger (10.0, callback);
  Dummy dummy;
  global_time = pcl::getTime ();
  trigger.start ();
  boost::this_thread::sleep(boost::posix_time::seconds(2));
  trigger.registerCallback ( boost::bind(&Dummy::myTimer, dummy));
  boost::this_thread::sleep(boost::posix_time::seconds(3));
  trigger.setInterval (0.2);
  boost::this_thread::sleep(boost::posix_time::seconds(2));
  return 0;
}
