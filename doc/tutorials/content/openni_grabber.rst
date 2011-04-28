.. _openni_grabber:

The OpenNI Grabber Framework in PCL
-----------------------------------

As of PCL 1.0, we offer a new generic grabber interface to provide a smooth and
convenient access to different devices and their drivers, file formats and
other sources of data. 

The first driver that we incorporated is the new OpenNI Grabber, which makes it
a breeze to request data streams from OpenNI compatible cameras. This tutorial
presents how to set up and use the grabber, and since it's so simple, we can
keep it short :).

The cameras that we have tested so far are the `Primesense Reference Design <http://www.primesense.com/?p=514>`_, `Microsoft Kinect <http://www.xbox.com/kinect/>`_ and `Asus Xtion Pro <http://event.asus.com/wavi/product/WAVI_Pro.aspx>`_ cameras:


.. image:: images/openni_cams.png
   :height: 390px

Simple Example
--------------

In *visualization*, there is a very short piece of code which contains all that
is required to set up a *pcl::PointCloud<XYZ>* or *pcl::PointCloud<XYZRGB>*
cloud callback.

Here is a screenshot and a video of the PCL OpenNI Viewer in action, which uses
the OpenNI Grabber.

.. image:: images/pcl_openni_viewer.jpg
   :height: 390px
   :target: _images/pcl_openni_viewer.jpg

.. raw:: html
  
  <iframe title="PCL OpenNI Viewer example" width="480" height="390" src="http://www.youtube.com/embed/x3SaWQkPsPI?rel=0" frameborder="0" allowfullscreen></iframe>

So let's look at the code. From *visualization/tools/openni_viewer_simple.cpp*

.. code-block:: cpp
   :linenos:

    class SimpleOpenNIViewer
    {
      public:
        SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

        void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
        {   
          if (!viewer.wasStopped())
            viewer.showCloud (cloud);
        }   

        void run ()
        {   
          pcl::Grabber* interface = new pcl::OpenNIGrabber();

          boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f = 
            boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

          interface->registerCallback (f);
              
          interface->start (); 
              
          while (!viewer.wasStopped())
          {   
            sleep (1);
          }   

          interface->stop (); 
        }   

        pcl::visualization::CloudViewer viewer;
    };

    int main ()
    {
      SimpleOpenNIViewer v;
      v.run (); 
      return 0;
    }


As you can see, the *run ()* function of *SimpleOpenNIViewer* first creates a
new *OpenNIGrabber* interface. The next line might seem a bit intimidating at
first, but it's not that bad. We create a *boost::bind* object with the address
of the callback *cloud_cb_*, we pass a reference to our *SimpleOpenNIViewer*
and the argument palce holder *_1*.

The *bind* then gets casted to a *boost::function* object which is templated on
the callback function type, in this case *void (const
pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)*. The resulting function object can
the be registered with the *OpenNIGrabber* and subsequently started.  Note that
the *stop ()* method does not necessarily need to be called, as the destructor
takes care of that.

Additional Details
------------------

The *OpenNIGrabber* offers more than one datatype, which is the reason we made
the *Grabber* interface so generic, leading to the relatively complicated
*boost::bind* line. In fact, we can register the following callback types as of
this writing:

* `void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGB> >&)`

* `void (const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZ> >&)`

* `void (const boost::shared_ptr<openni_wrapper::Image>&)`

  This provides just the RGB image from the built-in camera.

* `void (const boost::shared_ptr<openni_wrapper::DepthImage>&)`

  This provides the depth image, without any color or intensity information

* `void (const boost::shared_ptr<openni_wrapper::Image>&, const boost::shared_ptr<openni_wrapper::DepthImage>&, float constant)`
    
  When a callback of this type is registered, the grabber sends both RGB
  image and depth image and the constant (*1 / focal length*), which you need
  if you want to do your own disparity conversion. 

.. note::
  All callback types that need a depth _and_ image stream have a
  synchronization mechanism enabled which ensures consistent depth and image
  data. This introduces a small lag, since the synchronizer needs to wait at
  least for one more set of images before sending the first ones. 

Starting and stopping streams
-----------------------------

The *registerCallback* call returns a *boost::signals2::connection* object,
which we ignore in the above example. However, if you want to interrupt or
cancel one or more of the registered data streams, you can call disconnect the
callback without stopping the whole grabber:

.. code-block:: cpp

   boost::signals2::connection = interface (registerCallback (f));

   // ...

   if (c.connected ())
     c.disconnect ();

Benchmark
---------

The following code snippet will attempt to subscribe to both the *depth* and
*color* streams, and is provided as a way to benchmark your system. If your
computer is too slow, and you might not be able to get ~29Hz+, please contact
us. We might be able to optimize the code even further.

.. code-block:: cpp

  #include <pcl/point_cloud.h>
  #include <pcl/point_types.h>
  #include <pcl/io/openni_grabber.h>
  #include <pcl/common/time.h>

  class SimpleOpenNIProcessor
  {
    public:
      void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud)
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
        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =
          boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

        // connect callback function for desired signal. In this case its a point cloud with color values
        boost::signals2::connection c = interface->registerCallback (f);

        // start receiving point clouds
        interface->start ();

        // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
        while (true)
          sleep(1);

        // stop the grabber
        interface->stop ();
      }
  };

  int main ()
  {
    SimpleOpenNIProcessor v;
    v.run ();
    return 0;
  }

Conclusion
----------

The Grabber interface is very powerful and general and makes it a breeze to
connect to OpenNI compatible cameras in your code. We are in the process of
writing a FileGrabber which can be used using the same interface, and can e.g.
load all Point Cloud files from a directory and provide them to the callback at
a certain rate. The only change required is
the allocation of the Grabber Object (*pcl::Grabber *g = new ...;*).

If you have a sensor which you would like to have available within PCL, just
let us know at *pcl-developers@pointclouds.org*, and we will figure something
out.
