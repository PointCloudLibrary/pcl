/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
 *  Author: Qinghua Li, Yan Zhuang, Xuedong Wang
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Intelligent Robotics Lab, DLUT. nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
  * \Test demo: Create scene database
  * \Created on: July 07, 2013
  * \Author: Qinghua Li
  */

#include <QFileDialog>
#include <QTextStream>
#include <QApplication>
#include <opencv/highgui.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/place_recognition/scene_cognition.h>
#include "../include/scene_cognition.h"

int
main (int argc, char** argv)
{
  QApplication app(argc, argv);

  int cloud_width;
  int cloud_height;
  std::vector< std::vector<pcl::PointXYZ> > point_cloud;
  QString fileName;
  fileName = QFileDialog::getOpenFileName (0, "Open", ".", "PCD Files (*.pcd);; TXT Files (*.txt)");
  if (!fileName.isNull ())
  {
    std::string file_name = std::string ((const char*) fileName.toLocal8Bit ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1)  // Load the file
    {
      PCL_ERROR ("Error!\nCouldn't read PCD file.\n");
      return (-1);
    }
    cloud_width = cloud->width;
    cloud_height = cloud->height;
    int points_number = cloud_width * cloud_height;
    std::cout<<"Loaded "<<points_number<<" data points from PCD file Successfully"<<std::endl;

    pcl::PointXYZ temp_point;
    std::vector < pcl::PointXYZ > temp_vector;

    /** \Push the raw point cloud data to the 2-D vector(point_cloud) */
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      temp_point.x = cloud->points[i].x;
      temp_point.y = cloud->points[i].y;
      temp_point.z = cloud->points[i].z;
      temp_vector.push_back (temp_point);

      if (!((i + 1) % cloud->width))
      {
        point_cloud.push_back (temp_vector);
        temp_vector.clear ();
      }
    }
  }

  /** \generate and show bearing-angle image. */
  pcl::BearingAngle ba;
  IplImage* BA_image;
  if (point_cloud.size () > 0)
  {
    BA_image = ba.generateBAImage (point_cloud, cloud_width, cloud_height);
    cvNamedWindow ("Bearing-Angle image", 0);
    cvShowImage ("Bearing-Angle image", BA_image);
    cvWaitKey (0);
    cvDestroyWindow ("Bearing-Angle image");
  }
  else
  {
    std::cout<<"Error! \n"<<"NO available data, please load data at first!"<<std::endl;
    return (-1);
  }

  pcl::SceneCognition sc;
  GlobalFeature global_feature;
  if (sc.extractGlobalFeature (point_cloud, &global_feature)) // Extract global spatial features
  {
    CvMemStorage* storage = cvCreateMemStorage (0);
    sc.cvSURFInitialize ();

    CvSeq* model_descriptors = sc.cvSURFDescriptor (BA_image, storage, 4., 1);
    int n = model_descriptors->total;  // Total number of features

    if (sc.drawSURFFeatures (BA_image, model_descriptors, n)) // Mark features on the Bearing-Angle image
    {
      cvNamedWindow ("Bearing-Angle features image", 0);
      cvShowImage ("Bearing-Angle features image", BA_image);
      cvWaitKey (0);
      cvDestroyWindow ("Bearing-Angle features image");

      std::cout<<"The number of features is "<<n<<". \n"
               <<"Please save these global spatial features and local SURF features to database files(*.txt) correctly!"<<std::endl;

      /** \Save features to database file */
      QString fileName;
      fileName = QFileDialog::getSaveFileName (0, "Save", ".", "TXT Files (*.txt)");
      if (!fileName.isNull ())
      {
        QFile file (fileName);
        if (file.open (QIODevice::WriteOnly | QIODevice::Text))
        {
          QTextStream stream (&file);
          /** \Write the global spatial features to file */
          stream << global_feature.area << "  " << global_feature.duty_cycle << "\n";

          /** Write SURF features to file */
          SURFDescriptor* tmp_feature;
          for (int i = 0; i < n; i++)
          {
            tmp_feature = (SURFDescriptor*) cvGetSeqElem (model_descriptors, i);
            tmp_feature->laser_neighbors.resize (NEIGHBOR);  // 24 neighborhoods of laser point

            /** \Get the corresponding position(row and column) of laser point */
            int x = (int) tmp_feature->x;
            int y = (int) tmp_feature->y;

            if (x < 0)
            {
              x = 0;
            }
            else if (x > cloud_width - 1)
            {
              x = cloud_width - 1;
            }

            if (y < 0)
            {
              y = 0;
            }
            else if (y > cloud_height - 1)
            {
              y = cloud_height - 1;
            }
            x = cloud_width - 1 - x;
            y = cloud_height - 1 - y;

            tmp_feature->cor_point = point_cloud[y][x];  // laser data point

            stream << tmp_feature->x << " "
              << tmp_feature->y << " "
              << tmp_feature->laplacian << " "
              << tmp_feature->s << " "
              << tmp_feature->dir << " "
              << tmp_feature->mod << " "
              << tmp_feature->cor_point.x << " "
              << tmp_feature->cor_point.y << " " << tmp_feature->cor_point.z << " ";

            for (int j = 0; j < 128; j++)
            {
              stream << tmp_feature->vector[j] << " ";
            }
            stream << "\n";
          }
          file.close ();
        }
        std::cout<<"\nSave global spatial features and local SURF features to database files successfully!"<<std::endl;
      }
      else
        std::cout<<"\n~Failed~\n Invalid file name, please input file name correctly!"<<std::endl;
    }   // if (sc.drawSURFFeatures(BA_iplimage, model_descriptors, n))
  }     // if (sc.extractGlobalFeature(point_cloud, &global_feature))

  return 0;
}
