#include <iostream>
#include <map>
#include <vector>
#include <pcl/visualization/pcl_painter2D.h>
//----------------------------------------------------------------------------

int main (int argc, char * argv [])
{
  pcl::visualization::PCLPainter2D *painter = new pcl::visualization::PCLPainter2D();
  
  int winw = 800, winh = 600;
  painter->setWindowSize (winw, winh);
  int xpos = 0;
  int r = winw;
  int R = 50;
  int noc = winw/R;
  
  while (1)
  {
    painter->addLine (0, winh/2, winw, winh/2);
    painter->addLine (winw/2, 0, winw/2, winh);
  
    //draw noc no of circles
    for (int i = 0; i < noc; i++)
    {
      if (i % 2) 
        painter->setBrushColor (0, 0, 0, 200);
      else
        painter->setBrushColor (255, 255, 255, 200);
      
      int rad = r - i*R;
      if (rad < 0) { cout << "yaaaarp"; rad = winw + rad;}
      
      painter->addCircle (winw/2, winh/2, rad);
    }
    
    r -= 5;
    if (r < winw-R) r = winw + R;

    painter->setBrushColor (255,0,0,100);
    painter->addRect ((xpos += 5) % winw, 100, 100, 100);

    //display
    painter->spinOnce ();
    painter->clearFigures ();
  }


  return 0;
}