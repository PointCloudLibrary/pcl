.. _pcl_painter2D:

PCLPainter2D
============

PCLPainter2D class provides a very simple interface (just like PCLPlotter) to draw 2D figures in a canvas or a view. One can add figures by simple *add\*()* methods and in the end, show the canvas by simple *display\*()* methods.

Basic structure
---------------

Following is the usual way of using PCLPainter2D class.


.. code-block:: cpp

    //1. declare a Painter2D class
    PCLPainter2D painter;
	
    //2. add figures to the canvas by simple add*() methods. Use transform*() functions if required.
    painter.addCircle (0,0,5);
    painter.addLine (0,0, 5,0);
	
    //3. call a display*() (display (), spin (), spinOnce ()) method for the display of the canvas
    painter.display ();
	
  
Discussions
-----------

I am keeping this discussion here so that the design decision gets highlighted and is not lost in an unnoticed blog. Users who just want to learn this class can safely go ahead to the next section showing a complete example.

So, Lets see how 2D drawing works in VTK! The VTK user needs to first:
	
1) Make a subclass of vtkContextItem
2) Re-implement (override) Paint () of vtkContextItem. (shown in the figure)
		
.. image:: images/pcl_painter2D_contextItem.png
  :width: 350px
  :align: center

It would be really nice to have a vtkContextItem class which cuts off the overhead of subclassing and allows user to draw directly from the function calls. Unfortunately, we don't have any (out of vtkChart, vtkPlot, vtkAxis,..., etc.) vtkContextItem class with that kind of behavior. 

Thus, it maybe wise to have a class like Painter2D which can avoid subclassing in PCL and its rendering could be further optimized in the future.

A complete example
==================
Following is a complete example depcting many usage of the Plotter. Copy it into a file named ``pcl_painter2D_demo.cpp``.

.. literalinclude:: sources/pcl_painter2D/pcl_painter2D_demo.cpp
    :language: cpp
    :linenos:

Compiling and running the program
---------------------------------

Add the following lines to your `CMakeLists.txt` file:

.. literalinclude:: sources/pcl_painter2D/CMakeLists.txt
   :language: cmake
   :linenos:

Compile and run the code by the following commands ::

  $ cmake .
  $ make
  $ ./pcl_painter2D_demo
  
  
Video
-----

The following video shows the the output of the demo.  


.. raw:: html
    
    <iframe width="420" height="315" src="http://www.youtube.com/embed/0kPwTds7HSk" frameborder="0" allowfullscreen></iframe>
