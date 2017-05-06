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
  * \Test demo: Place Recognition
  * \Created on: July 07, 2013
  * \Author: Qinghua Li
  */

#include <QApplication>
#include <QFileDialog>
#include <opencv/highgui.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/place_recognition/scene_cognition.h>
#include "../include/scene_cognition.h"

#define MIN_VALUE 0.0
#define MIN_NUMBER 0  // The minimum requirement for matching pairs

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

    // Push the raw point cloud data to the 2-D vector(point_cloud)
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

    // Add the character of laser data
    for (int i = 0; i < n; i++)
    {
      SURFDescriptor* temp_feature = (SURFDescriptor*) cvGetSeqElem (model_descriptors, i);
      temp_feature->laser_neighbors.resize (NEIGHBOR);  // 24 neighborhoods of laser point

      // Get the corresponding position(row and column) of laser point
      int x = temp_feature->x;
      int y = temp_feature->y;

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

      temp_feature->cor_point = point_cloud[y][x];  // laser data point
    }

    if (sc.drawSURFFeatures (BA_image, model_descriptors, n)) // Mark features on the Bearing-Angle image
    {
      cvNamedWindow ("Bearing-Angle features image", 0);
      cvShowImage ("Bearing-Angle features image", BA_image);
      cvWaitKey (0);
      cvDestroyWindow ("Bearing-Angle features image");

      std::cout<<"The number of features is "<<n<<". \n"
               <<"Please select database files what you want to load"<<std::endl;

      /** \Load the SURF features from database files */
      sc.surf_descriptors.clear ();
      sc.global_descriptors.clear ();
      QStringList files;
      files = QFileDialog::getOpenFileNames (0, "Load database files", ".", "TXT Files (*.txt)");
      if (!files.isEmpty ())
      {
        int file_number = files.size ();
        BOOST_FOREACH (QString s, files)
        {
          QFile file (s);
          if (file.open (QIODevice::ReadOnly | QIODevice::Text))
          {
            GlobalFeature global_feature;
            SURFDescriptor tmp_feature;
            std::vector< SURFDescriptor > tmp_surf_descriptors;
            tmp_surf_descriptors.clear ();

            // Read global features
            QByteArray line = file.readLine ();
            char* sep_str = line.data ();
            char sepr[] = " \t\n";

            char* str = strtok (sep_str, sepr);
            if (str != NULL)
              global_feature.area = atof (str);
            str = strtok (NULL, sepr);
            if (str != NULL)
              global_feature.duty_cycle = atof (str);
            sc.global_descriptors.push_back (global_feature);

            /** \Read SURF features */
            while (!file.atEnd ())
            {
              line = file.readLine ();
              sep_str = line.data ();

              //Read the pixel position(x,y)
              str = strtok (sep_str, sepr);
              if (str != NULL)
                tmp_feature.x = atof (str);
              str = strtok (NULL, sepr);
              if (str != NULL)
                tmp_feature.y = atof (str);

              str = strtok (NULL, sepr);
              if (str != NULL)
                tmp_feature.laplacian = atof (str);

              // Read the scale
              str = strtok (NULL, sepr);
              if (str != NULL)
                tmp_feature.s = atof (str);

              // Read the principal direction
              str = strtok (NULL, sepr);
              if (str != NULL)
                tmp_feature.dir = atof (str);

              str = strtok (NULL, sepr);
              if (str != NULL)
                tmp_feature.mod = atof (str);

              // Read the corresponding laser point for this feature
              str = strtok (NULL, sepr);
              if (str != NULL)
                tmp_feature.cor_point.x = atof (str);
              str = strtok (NULL, sepr);
              if (str != NULL)
                tmp_feature.cor_point.y = atof (str);
              str = strtok (NULL, sepr);
              if (str != NULL)
              tmp_feature.cor_point.z = atof (str);

              for (int i = 0; i < 128; i++)
              {
                str = strtok (NULL, sepr);
                if (str != NULL)
                tmp_feature.vector[i] = atof (str);
              }

              tmp_surf_descriptors.push_back (tmp_feature);
            }

            sc.surf_descriptors.push_back (tmp_surf_descriptors);
            file.close ();
          }
        }
      }
      else
      {
        std::cout<<"Failed to load database files!"<<std::endl;
        return (-1);
      }

      int file_num = sc.global_descriptors.size ();
      std::cout<<"Select "<<file_num<<" database files totally \n"<<std::endl;

      if (file_num > 0)
      {
        int check_number = 0;
        std::vector < bool > if_select (file_num, false);

        while (check_number < file_num)
        {
          // Locate the unchecked matching scene
          int uncheck = -1;
          do
          {
            uncheck++;
          }
          while (if_select[uncheck] == true && uncheck < file_num);

          int best = uncheck;
          double flag_dis = 0;
          // Select the closest scene with query one from unchecked matching database scenes
          for (int next = uncheck + 1; next < file_num; next++)
          {
            double area_distance1 = (sc.global_descriptors[next].area - global_feature.area) *
                                    (sc.global_descriptors[next].area - global_feature.area);
            double duty_cycle_distance1 = (sc.global_descriptors[next].duty_cycle - global_feature.duty_cycle) *
                                          (sc.global_descriptors[next].duty_cycle - global_feature.duty_cycle);
            double area_distance2 = (sc.global_descriptors[best].area - global_feature.area) *
                                    (sc.global_descriptors[best].area - global_feature.area);
            double duty_cycle_distance2 = (sc.global_descriptors[best].duty_cycle - global_feature.duty_cycle) *
                                          (sc.global_descriptors[best].duty_cycle - global_feature.duty_cycle);

            double distance1 = 0.8 * area_distance1;
            double distance2 = 0.8 * area_distance2;

            flag_dis = distance2;
            if (distance1 < distance2 && if_select[next] == false)
            {
              best = next;
              flag_dis = distance1;
            }
          }

          if_select[best] = true;

          // KD-Tree matching
          CvSeq* seq = sc.findSURFMatchPairs (model_descriptors, n, sc.surf_descriptors[best], storage);

          // Get the matching degree
          double match_degree = sc.getMatchDegree (seq);
          // The determined factor for whether query scene is one of database scenes
          if (sc.eff_match_pairs >= MIN_NUMBER && match_degree >= MIN_VALUE)
          {
            QString file_full_name = files[best];
            QFileInfo info (file_full_name);
            QString file_name = info.baseName ();

            std::cout<<"Place Recognition is accomplished Successfully! \n"
                     <<"The query place should be "<<(const char*) file_name.toLocal8Bit ()<<". \n"
                     <<"    Matching degree: "<<match_degree<<"\n"
                     <<"    Effective matching pairs: "<<sc.eff_match_pairs<<"\n"<<std::endl;
            break;
          }

          check_number++;
          if (check_number == file_num)
          {
            std::cout<<"Sorry, there is not available matching place in the database! \n"
                     <<"You may add the query scene to database in test_2."<<std::endl;
          }
        }  // End of while
      }    // if (file_num > 0)
    }      // if (sc.drawSURFFeatures(BA_iplimage, model_descriptors, n))
  }        // if (sc.extractGlobalFeature(point_cloud, &global_feature))

  return 0;
}
