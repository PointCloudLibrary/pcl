.. _mobile_streaming:

Point Cloud Streaming to Mobile Devices with Real-time Visualization
--------------------------------------------------------------------

This tutorial describes how to send point cloud data over the network from a desktop server to a client running on a mobile
device.  The tutorial describes an example app, *PointCloudStreaming*, for the Android
operating system that receives point clouds over a TCP socket and renders them
using the VES and Kiwi mobile visualization framework.  The *PointCloudStreaming*
app acts as a client, and it connects to the server program *pcl_openni_mobile_server*.
The server program uses the ``pcl::OpenNIGrabber`` to generate point clouds from an
OpenNI compatible camera.  The tutorial :ref:`openni_grabber` provides a background
for working with the ``pcl::OpenNIGrabber``.  This tutorial describes the client and server
programs and how to run them.

  .. raw:: html

    <iframe title="Point cloud streaming" width="500" height="281" src="http://player.vimeo.com/video/41377003" frameborder="0" webkitAllowFullScreen mozallowfullscreen allowfullscreen></iframe>

Building and running the server
-------------------------------

The server program, *pcl_openni_mobile_server*, is included with PCL as an
example app.  Build PCL with the **BUILD_apps** option enabled, then run the
server program from the PCL build directory::

  $ ./bin/pcl_openni_mobile_server -p 11111

The server will start and listen on port 11111.  You must have an OpenNI compatible
camera connected in order to run the server, otherwise the program will abort
with an error message that the OpenNI grabber could not be initialized.  When the server
starts, it will open a visualization window.  The visualization window will refresh
once after each new point cloud is sent to the client.  The server runs until the
client disconnects.  The server uses a voxel grid filter and a bounding box region
to limit the number of points sent to the client.

Building and running the client
-------------------------------

The client is an Android app named *Point Cloud Streaming*.  The app
is implemented using the Android `NativeActivity <https://developer.android.com/reference/android/app/NativeActivity.html>`_.
Using *NativeActivity*, an Android app can be implemented in pure
C++ code without writing components in Java.  The app uses APIs provided by the `Android
NDK <http://developer.android.com/tools/sdk/ndk/index.html>`_ to handle touch events
and app life cycle events.  While this is suitable for an example app, apps that
demand extra features and user interface elements will require implementations that mix
native code and Java components and APIs.

To build the *PointCloudStreaming* app, first build its main dependency, VES and Kiwi.
Follow the `VES Developer's Guide <http://vtk.org/Wiki/VES/Developers_Guide>`_ for
instructions on setting up your environment for compiling Android applications and
how to build VES and Kiwi.  Next, edit the file *mobile_apps/android/PointCloudStreaming/tools.sh*
and enter the correct paths for your environment.  Set the ANDROID_NDK environment
variable to the location of your Android NDK installation, and then run the bash
scripts in order::

  $ ./configure_cmake.sh
  $ ./configure_ant.sh
  $ ./compile.sh
  $ ./install.sh

Make sure your device is connected before running install.sh.  After running
install.sh, the *Point Cloud Streaming* app will be found on your device.  When
you start the app, it will automatically attempt to connect to the server program.
The app uses a text file to read the server host and port information that will
be used.  The first time the app runs, it will write a text file to the Android
device's SD card at */mnt/sdcard/PointCloudStreaming/appConfig.txt*.  You can edit this file to input the correct server host and
port information, or modify the file *mobile_apps/android/PointCloudStreaming/assets/appConfig.txt*
in the PCL source code repository and recompile the app.

.. note::
   Note, the app will not overwrite *appConfig.txt* on the SD card if it already exists.

When the app runs, it will display text on the screen indicating whether or not
the server connection was successful.  Upon successful connection to the server,
the app will begin receiving and rendering point clouds.  The camera can be
repositioned using single-touch and two-touch gestures.


Server program implementation
-----------------------------

The server is provided by the program pcl_openni_mobile_server and implemented
by the source file  *apps/src/openni_mobile_server.cpp*.  The program's entry
point is the *main()* function.  It parses command line arguments and then creates
a *PCLMobileServer* object, passing the command line arguments as parameters to
the constructor.  The remainder of the program is handled by the *PCLMobileServer*
object.

.. code-block:: cpp

    PCLMobileServer<pcl::PointXYZRGBA> server (device_id, port, leaf_x, leaf_y, leaf_z);
    server.run ();

The *run()* method initializes some objects before entering the main server loop.
The first object to be initialized is the ``pcl::OpenNIGrabber``.  The grabber is
used to generate point clouds from an OpenNI compatible camera.  Here are the first
few lines of the *run()* method:

.. code-block:: cpp

    pcl::OpenNIGrabber grabber (device_id_);
    std::function<void (const CloudConstPtr&)> handler_function = [this] (const CloudConstPtr& cloud) { handleIncomingCloud (cloud); };
    grabber.registerCallback (handler_function);
    grabber.start ();

The grabber is constructed and then the *handleIncomingCloud* method is bound and
registered as a callback on grabber.  This callback method is called for each new
point cloud that is generated.  The OpenNIGrabber runs in a separate thread, and
the *handleIncomingCloud* method is called on that thread.  This allows the
grabber is generate and process point clouds continuously while the server
loop runs in the main thread.  Here is the implementation of the *handleIncomingCloud()*
method:

.. code-block:: cpp

    void
    handleIncomingCloud (const CloudConstPtr& new_cloud)
    {
      CloudPtr temp_cloud (new Cloud);
      voxel_grid_filter_.setInputCloud (new_cloud);
      voxel_grid_filter_.filter (*temp_cloud);

      PointCloudBuffers::Ptr new_buffers = PointCloudBuffers::Ptr (new PointCloudBuffers);
      CopyPointCloudToBuffers (temp_cloud, *new_buffers);

      std::lock_guard<std::mutex> lock (mutex_);
      filtered_cloud_ = temp_cloud;
      buffers_ = new_buffers;
    }

The new cloud is filtered through a voxel grid filter.  The result of the voxel
grid filter is then copied into a *PointCloudBuffers* object.  This object
is a struct that contains the buffers that will be sent over the TCP
socket to the client:

.. code-block:: cpp

    struct PointCloudBuffers
    {
      typedef pcl::shared_ptr<PointCloudBuffers> Ptr;
      std::vector<short> points;
      std::vector<unsigned char> rgb;
    };

The *PointCloudBuffers* struct contains two vectors, one for points and one
for rgb colors.  The points vector is defined using short.  Each xyz point
coordinate of the point cloud is converted from float to short in order to
reduce the number of bytes required to represent the coordinate.  This conversion
results in a loss of precision, but the assumption is that the point clouds generated
by the ``pcl::OpenNIGrabber`` will have units in meters and the extent of the point
cloud will be limited to only several meters.  The short data type contains
enough bits to acceptably represent such value ranges for the purposes of
visualization.

The conversion from float to short is performed by the *CopyPointCloudToBuffers*
function.  The function also defines a fixed, axis aligned bounding box, outside
of which points will be culled.  The function loops over all the points in the
point cloud and copies the xyz and rgb values into buffers, while skipping points
that lie outside of the predefined bounding box or contain NaN values.

.. code-block:: cpp

    void
    CopyPointCloudToBuffers (pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, PointCloudBuffers& cloud_buffers)
    {
      const std::size_t nr_points = cloud->size ();

      cloud_buffers.resize (nr_points*3);
      cloud_buffers.rgb.resize (nr_points*3);

      const pcl::PointXYZ  bounds_min (-0.9, -0.8, 1.0);
      const pcl::PointXYZ  bounds_max (0.9, 3.0, 3.3);

      std::size_t j = 0;
      for (std::size_t i = 0; i < nr_points; ++i)
      {

        const pcl::PointXYZRGBA& point = (*cloud)[i];

        if (!pcl_isfinite (point.x) || 
            !pcl_isfinite (point.y) || 
            !pcl_isfinite (point.z))
          continue;

        if (point.x < bounds_min.x ||
            point.y < bounds_min.y ||
            point.z < bounds_min.z ||
            point.x > bounds_max.x ||
            point.y > bounds_max.y ||
            point.z > bounds_max.z)
          continue;

        const int conversion_factor = 500;

        cloud_buffers[j*3 + 0] = static_cast<short> (point.x * conversion_factor);
        cloud_buffers[j*3 + 1] = static_cast<short> (point.y * conversion_factor);
        cloud_buffers[j*3 + 2] = static_cast<short> (point.z * conversion_factor);

        cloud_buffers.rgb[j*3 + 0] = point.r;
        cloud_buffers.rgb[j*3 + 1] = point.g;
        cloud_buffers.rgb[j*3 + 2] = point.b;

        j++;
      }

      cloud_buffers.resize (j * 3);
      cloud_buffers.rgb.resize (j * 3);
    }

The server program opens a TCP socket and waits for a client connection using
APIs provided by boost::asio and boost::asio::tcp.

.. code-block:: cpp

    boost::asio::io_service io_service;
    tcp::endpoint endpoint (tcp::v4 (), static_cast<unsigned short> (port_));
    tcp::acceptor acceptor (io_service, endpoint);
    tcp::socket socket (io_service);

    std::cout << "Listening on port " << port_ << "..." << std::endl;
    acceptor.accept (socket);

    std::cout << "Client connected." << std::endl;

After a successful connection, the program enters the main server loop:

.. code-block:: cpp

      while (!viewer_.wasStopped ())
      {

        // wait for client
        unsigned int nr_points = 0;
        boost::asio::read (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

        PointCloudBuffers::Ptr buffers_to_send = getLatestBuffers ();

        nr_points = static_cast<unsigned int> (buffers_to_send->size()/3);
        boost::asio::write (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

        if (nr_points)
        {
          boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->points.front(), nr_points * 3 * sizeof (short)));
          boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->rgb.front(), nr_points * 3 * sizeof (unsigned char)));
        }

        counter++;

        double new_time = pcl::getTime ();
        double elapsed_time = new_time - start_time;
        if (elapsed_time > 1.0)
        {
          double frames_per_second = counter / elapsed_time;
          start_time = new_time;
          counter = 0;
          std::cout << "fps: " << frames_per_second << std::endl;
        }

        viewer_.showCloud (getLatestPointCloud ());
      }

The first part of the loop waits for a message from the client.  It reads 4 bytes from
the client, but does not actually read the value sent.

.. code-block:: cpp

    // wait for client
    unsigned int nr_points = 0;
    boost::asio::read (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

You could extend the example code so that the client actually sends some usable
information to the server, such as new leaf size parameters to set on the voxel grid filter.

Next, the loop gets the latest point cloud buffers that were generated by the OpenNI grabber
callback function, and sends information about the buffer's number of points to the client:

.. code-block:: cpp

    PointCloudBuffers::Ptr buffers_to_send = getLatestBuffers ();

    nr_points = static_cast<unsigned int> (buffers_to_send->size()/3);
    boost::asio::write (socket, boost::asio::buffer (&nr_points, sizeof (nr_points)));

Next, if there is a non-zero number of points, the server sends the xyz and rgb
buffers to the client:

.. code-block:: cpp

    if (nr_points)
    {
      boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->points.front(), nr_points * 3 * sizeof (short)));
      boost::asio::write (socket, boost::asio::buffer (&buffers_to_send->rgb.front(), nr_points * 3 * sizeof (unsigned char)));
    }

The remainder of the code in the server loop is responsible for refreshing the
server's visualization window and incrementing a counter for tracking the number
of point clouds per second that are transferred.  The server runs indefinitely until
it is terminated or the connection drops.


Client app implementation
-------------------------

The client application, an Android app named *PointCloudStreaming* is implemented
in a single C++ file, *mobile_apps/android/PointCloudStreaming/jni/PointCloudStreaming.cpp*.
The app implementation contains a lot of boiler plate code for initializing the OpenGL ES 2.0
rendering context, managing application life cycle using the Android NDK APIs, and
converting touch events into high level gestures.  Most of this code is outside of the
scope of this tutorial.  This tutorial will focus on the code in the client app that is
responsible for handling point cloud streaming.  In fact, the majority of the code that
handles point cloud streaming is contained in a class named *vesKiwiStreamingDataRepresentation*
found in the *kiwi* library, part of the VES and Kiwi mobile visualization framework.  
The *vesKiwiStreamingDataRepresentation* is usable by any mobile application.

The *PointCloudStreaming* app, in *PointCloudStreaming.cpp* instantiates the
*vesKiwiStreamingDataRepresentation* in a function named *connect()* like this:

.. code-block:: cpp

  bool connect(const std::string& host, int port)
  {
    mIsConnected = false;

    std::stringstream hostPort;
    hostPort << host << ":" << port;
    this->showText("Connecting to " + hostPort.str());

    if (!mDataRep) {
      mDataRep = vesKiwiStreamingDataRepresentation::Ptr(new vesKiwiStreamingDataRepresentation);
    }

    if (!mDataRep->connectToServer(host, port)) {
      this->showText("Connection failed to " + hostPort.str());
      return false;
    }

    this->showText("Connected to " + hostPort.str());
    mIsConnected = true;
    mDataRep->initializeWithShader(mShader);
    mDataRep->addSelfToRenderer(this->renderer());
    this->resetView();
    return true;
  }

A new instance is lazy constructed and stored in *mDataRep*.  The *mDataRep* object provides
functionality for initializing the connection to the server and managing
the point cloud streaming after a successful connection.  After a successful connection
is made, the *connect()* function does not need to be called again.  The *mDataRep*
object starts a new thread which reads point cloud xyz and rgb values from the TCP socket
and converts them into VES data structures that are used for rendering.  The primary
data structure used is a *vesGeometryData* which will be described in more detail later.

At each render loop, the *willRender()* function is called:

.. code-block:: cpp

  void willRender()
  {
    this->Superclass::willRender();

    if (mIsConnected) {
      this->mDataRep->willRender(this->renderer());
    }
    else {
      this->connect(mHost, mPort);
    }
  }

If there is not a valid connection to the server, then a connection is attempted,
otherwise the *willRender()* method of *mDataRep* is called.  The *mDataRep* object
uses this opportunity to swap in the most recent *vesGeometryData* data structure
in order to update the point cloud visualization before rendering the new frame.

Let's now examine some of the code in *vesKiwiStreamingDataRepresentation*.  This class
is derived from *vesKiwiDataRepresentation*.  In kiwi, a *data representation*
is a high level class that contains all the custom logic required to render
a piece of data and control its appearance.  The *data representation* ties together
many different classes from VTK and VES to accomplish its task.  For example,
it may use VTK filters and data objects, convert VTK data objects into VES data structures,
and use VES rendering classes for managing shaders, textures, and appearance details.
Advanced *data representations*, such as those derived from *vesKiwiWidgetRepresentation*
use touch events and gestures to update the data object visualization.

In the case of *vesKiwiStreamingDataRepresentation*, it uses a TCP socket and a
thread in order to manage a real-time visualization of a point cloud stream sent
from the server.  The server connection is established in the *connectToServer()*
method:

.. code-block:: cpp

    bool vesKiwiStreamingDataRepresentation::connectToServer(const std::string& host, int port)
    {
      return (this->Internal->Comm->ConnectToServer(host.c_str(), port) == 0);
    }

In the above code, *this->Internal->Comm* is an instance of a *vtkClientSocket*.
Rather than use *boost::asio::tcp*, kiwi makes use of the networking classes
provided by VTK.  After the connection is established, the client loop is started
in a new thread:

.. code-block:: cpp

    this->Internal->ClientThreadId = this->Internal->MultiThreader->SpawnThread(ClientLoop, this->Internal);

The client loop is implemented by the *ClientLoop* function.  The *this->Internal* pointer
is passed to the *ClientLoop* function as an argument.  The client loop runs in a
new thread and uses the *this->Internal* pointer to communicate with the main thread.
Communication is performed safely using a mutex lock.  Here is the implementation of
the client loop:

.. code-block:: cpp

    VTK_THREAD_RETURN_TYPE ClientLoop(void* arg)
    {
      vtkMultiThreader::ThreadInfo* threadInfo = static_cast<vtkMultiThreader::ThreadInfo*>(arg);

      vesKiwiStreamingDataRepresentation::vesInternal* selfInternal =
        static_cast<vesKiwiStreamingDataRepresentation::vesInternal*>(threadInfo->UserData);

      bool shouldQuit = false;
      while (!shouldQuit) {

          vesGeometryData::Ptr geometryData = ReceiveGeometryData(selfInternal->Comm.GetPointer());

          if (!geometryData) {
            break;
          }

          selfInternal->Lock->Lock();
          selfInternal->GeometryData = geometryData;
          selfInternal->HaveNew = true;
          shouldQuit = selfInternal->ShouldQuit;
          selfInternal->Lock->Unlock();
      }

      return VTK_THREAD_RETURN_VALUE;
    }

The bulk of the work is carried out by *ReceiveGeometryData()*.  This function
is responsible for receiving point cloud xyz and rgb buffers over the TCP
socket and copying them into a new *vesGeometryData* object that is used for
rendering.  *ReceiveGeometryData()* is implemented like this:

.. code-block:: cpp

    vesGeometryData::Ptr ReceiveGeometryData(vtkClientSocket* comm)
    {
      vtkNew<vtkShortArray> points;
      vtkNew<vtkUnsignedCharArray> colors;
      double startTime = vtkTimerLog::GetUniversalTime();

      int numberOfPoints = 0;

      if (!comm->Send(&numberOfPoints, 4)) {
        return vesGeometryData::Ptr();
      }
      if (!comm->Receive(&numberOfPoints, 4)) {
        return vesGeometryData::Ptr();
      }

      if (!numberOfPoints) {
        return vesGeometryData::Ptr(new vesGeometryData);
      }

      points->SetNumberOfTuples(numberOfPoints*3);
      colors->SetNumberOfComponents(3);
      colors->SetNumberOfTuples(numberOfPoints);

      if (!comm->Receive(points->GetVoidPointer(0), numberOfPoints * 3 * 2)) {
        return vesGeometryData::Ptr();
      }
      if (!comm->Receive(colors->GetVoidPointer(0), numberOfPoints * 3)) {
        return vesGeometryData::Ptr();
      }

      double elapsed = vtkTimerLog::GetUniversalTime() - startTime;
      double kb = points->GetActualMemorySize() + colors->GetActualMemorySize();
      double mb = kb/1024.0;

      std::cout << numberOfPoints << " points in " << elapsed << " seconds "
                << "(" << mb/ elapsed << "mb/s)" << std::endl;


      return CreateGeometryData(points.GetPointer(), colors.GetPointer());
    }

The network communication code in *ReceiveGeometryData()* is written to match the
communication code in the server program.  First, a ready signal is sent from
the client to the server.  This signal is 4 bytes and is not actually used for
anything on the server side.

.. code-block:: cpp

    int numberOfPoints = 0;

    if (!comm->Send(&numberOfPoints, 4)) {
      return vesGeometryData::Ptr();
    }

The return value of *Send()* is checked to determine whether or not the
communication was successful.  If the connection was dropped then the function
aborts by returning a null *vesGeometryData* pointer.  The client loop is designed
to break out of the loop in the case of a null pointer, indicating a dropped
connection.  If the connection is still valid, but the incoming point cloud
contains zero points, then an empty *vesGeometryData* object is returned:

.. code-block:: cpp

    if (!numberOfPoints) {
      return vesGeometryData::Ptr(new vesGeometryData);
    }

If there is a non-zero number of points to receive, then the xyz and rgb data
is received into buffers:

.. code-block:: cpp

    points->SetNumberOfTuples(numberOfPoints*3);
    colors->SetNumberOfComponents(3);
    colors->SetNumberOfTuples(numberOfPoints);

    if (!comm->Receive(points->GetVoidPointer(0), numberOfPoints * 3 * 2)) {
      return vesGeometryData::Ptr();
    }
    if (!comm->Receive(colors->GetVoidPointer(0), numberOfPoints * 3)) {
      return vesGeometryData::Ptr();
    }

The points object is a *vtkShortArray* and colors is a *vtkUnsignedCharArray*.
These types are analogous to std::vector<short> and std::vector<unsigned char>.
Finally, the buffers are copied into a new *vesGeometryData* object which will
be used for rendering.  The copy is performed by *CreateGeometryData()*:

.. code-block:: cpp

    vesGeometryData::Ptr CreateGeometryData(vtkShortArray* points, vtkUnsignedCharArray* colors)
    {
      const int numberOfPoints = points->GetNumberOfTuples()*points->GetNumberOfComponents() / 3;

      vesSharedPtr<vesGeometryData> output(new vesGeometryData());
      vesSourceDataP3f::Ptr sourceData(new vesSourceDataP3f());

      vesVertexDataP3f vertexData;
      for (int i = 0; i < numberOfPoints; ++i) {
        vertexData.m_position[0] = points->GetValue(i*3 + 0);
        vertexData.m_position[1] = points->GetValue(i*3 + 1);
        vertexData.m_position[2] = points->GetValue(i*3 + 2);
        sourceData->pushBack(vertexData);
      }

      output->addSource(sourceData);
      output->setName("PolyData");

      vesPrimitive::Ptr pointPrimitive (new vesPrimitive());
      pointPrimitive->setPrimitiveType(vesPrimitiveRenderType::Points);
      pointPrimitive->setIndexCount(1);
      output->addPrimitive(pointPrimitive);


      vesKiwiDataConversionTools::SetVertexColors(colors, output);
      return output;
    }

Remember, the network communication and construction of *vesGeometryData* occurs
on a thread.  The main thread is used by the application for rendering.  A
mutex lock is used to update the pointer to the most recent *vesGeometryData*
object constructed.  On the main thread, before each frame to be rendered, the
*vesKiwiStreamingDataRepresentation* has the opportunity to swap the
current *vesGeometryData* pointer with a new one.  This occurs in *willRender()*:

.. code-block:: cpp

    void vesKiwiStreamingDataRepresentation::willRender(vesSharedPtr<vesRenderer> renderer)
    {
      vesNotUsed(renderer);

      this->Internal->Lock->Lock();

      if (this->Internal->HaveNew) {
        this->Internal->PolyDataRep->mapper()->setGeometryData(this->Internal->GeometryData);
        this->Internal->HaveNew = false;
      }

      this->Internal->Lock->Unlock();
    }

By using threads, the network communication of the client loop is decoupled from
the application's rendering loop.  The app is able to render the point cloud and
handle touch events to move the camera at interactive frame rates, even if the
network communication runs at a slower rate.

Conclusion
----------

This tutorial has described client and server programs for streaming point
clouds to mobile devices.  The example client program runs on Android, but it is implemented
in native C++ code that is runnable on other mobile operating systems such
as iOS.

If one wants to develop their own streaming point cloud apps, a good starting
point would be to copy and rename the *vesKiwiStreamingDataRepresentation* class (instead of
deriving from it) to create a new class that can be modified to implement the client side communication.
The new class can be compiled directly with the new Android and iOS app being developed.
The source code of the VES and Kiwi mobile visualization framework contains additional examples of
Android and iOS apps.  These examples can also be used as starting points for developing new apps.
For more information, see the `VES and Kiwi homepage <http://vtk.org/Wiki/VES>`_.
