#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

using namespace std::chrono_literals;

class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
    static unsigned count = 0;
    static double last = pcl::getTime ();
    if (++count == 30)
    {
      double now = pcl::getTime ();
      std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      count = 0;
      last = now;
    }
  }
  
  void run ()
  {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    std::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      [this] (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud) { cloud_cb_ (cloud); };

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (true)
      std::this_thread::sleep_for(1s);

    // stop the grabber
    interface->stop ();
  }
};

int main ()
{
  SimpleOpenNIProcessor v;
  v.run ();
  return (0);
}
