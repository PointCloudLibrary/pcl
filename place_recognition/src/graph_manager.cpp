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

#include "../include/graph_manager.h"

#define MIN_VALUE 0.0
#define MIN_NUMBER 0  // The minimum requirement for matching pairs

std::vector < std::vector < pcl::PointXYZ > >GraphManager::lib_cloud;
std::vector < std::vector < pcl::PointXYZ > >GraphManager::query_cloud;


GraphManager::GraphManager ()
  : cloud_width (361), cloud_height (361)
{
}

GraphManager::~GraphManager ()
{
}


void
GraphManager::loadData ()
{
  Q_EMIT reset();
  QString message;
  QString fileName;
  fileName = QFileDialog::getOpenFileName (0, tr ("Open"), ".", tr ("PCD Files (*.pcd);; TXT Files (*.txt)"));
  if (!fileName.isNull ())
  {
    std::string file_name = std::string ((const char*) fileName.toLocal8Bit ());

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_name, *cloud) == -1)  // Load the file
    {
      PCL_ERROR ("Error!\nCouldn't read PCD file.\n");
      switch (QMessageBox::critical (NULL, tr ("Critical"), "Load data failed!\n"
                                     "Would you like to reload data?",
                                     QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))
      {
        case QMessageBox::Yes:
          loadData ();
          break;

        case QMessageBox::No:
          break;
      }
      return;
    }

    int points_number = cloud->width * cloud->height;
    Q_EMIT setGUIStatus(message.sprintf ("Loaded %d data points from PCD file Successfully", points_number));

    cloud_width = cloud->width;
    cloud_height = cloud->height;

    pcl::PointXYZ temp_point;
    std::vector < pcl::PointXYZ > temp_vector;

    // Push the raw point cloud data to the 2-D vector(query_cloud)
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      temp_point.x = cloud->points[i].x;
      temp_point.y = cloud->points[i].y;
      temp_point.z = cloud->points[i].z;
      temp_vector.push_back (temp_point);

      if (!((i + 1) % cloud->width))
      {
        query_cloud.push_back (temp_vector);
        temp_vector.clear ();
      }
    }
  }
  else
  {
    switch (QMessageBox::critical (NULL, tr ("Critical"), "Failed!\n Please open file again!",
                                   QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))
    {
      case QMessageBox::Yes:
        loadData ();
        break;

      case QMessageBox::No:
        break;
    }
  }
}

void
GraphManager::showBAImage ()
{
  if (query_cloud.size () == 0)
  {
    switch (QMessageBox::critical (NULL, tr ("Critical"), "Failed!\n"
                                   "NO available data, please load data at first!",
                                   QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))
    {
      case QMessageBox::Yes:
        loadData ();
        showBAImage ();
        break;

      case QMessageBox::No:
        break;
    }
  }
  else
  {
    BearingAngle ba;
    BA_iplimage = ba.generateBAImage (query_cloud, cloud_width, cloud_height);
    QImage BA_qimage = ba.cvIplImage2QImage (ba.getChannelsImage (BA_iplimage));
    drawQImageText (BA_qimage, "Query scene BA image");
    Q_EMIT newQueryBAImage(BA_qimage);
  }
}

void
GraphManager::extractFeatures ()
{
  SceneRecognition sr;
  GlobalFeature global_feature;
  if (sr.extractGlobalFeature (query_cloud, &global_feature)) // Extract global spatial features
  {
    BearingAngle ba;
    BA_iplimage = ba.generateBAImage (query_cloud, cloud_width, cloud_height);
    QImage BA_qimage = ba.cvIplImage2QImage (ba.getChannelsImage (BA_iplimage));
    cvReleaseImage (&ba.channels_image);
    drawQImageText (BA_qimage, "Query scene BA image");
    Q_EMIT setGUIStatus("Show Bearing-Angle image and Features image of Query scene");
    Q_EMIT newQueryBAImage(BA_qimage);

    CvMemStorage* storage = cvCreateMemStorage (0);
    sr.cvSURFInitialize ();

    CvSeq* model_descriptors = sr.cvSURFDescriptor (BA_iplimage, storage, 4., 1);
    int n = model_descriptors->total;  // Total number of features

    if (sr.drawSURFFeatures (BA_iplimage, model_descriptors, n)) // Mark features on the Bearing-Angle image
    {
      QImage features_qimage = ba.cvIplImage2QImage (ba.getChannelsImage (BA_iplimage));
      drawQImageText (features_qimage, "Query scene features image");
      Q_EMIT newFeaturesImage(features_qimage);

      QString message;
      QMessageBox::about (NULL, tr ("Tips"), message.sprintf ("The number of features is %d.\n"
                          "Please save these global spatial features and local SURF features to database files", n));
      while (true)
      {
        // Save features to database file
        QString fileName;
        fileName = QFileDialog::getSaveFileName (0, tr ("Save"), ".", tr ("TXT Files (*.txt)"));
        Q_EMIT setGUIStatus("Please input file name(*.txt) correctly");
        if (!fileName.isNull ())
        {
          QFile file (fileName);
          if (file.open (QIODevice::WriteOnly | QIODevice::Text))
          {
            QTextStream stream (&file);
            // Write the global spatial features to file
            stream << global_feature.area << "  " << global_feature.duty_cycle << "\n";

            // Write SURF features to file
            SURFDescriptor* tmp_feature;
            for (int i = 0; i < n; i++)
            {
              tmp_feature = (SURFDescriptor*) cvGetSeqElem (model_descriptors, i);
              tmp_feature->laser_neighbors.resize (NEIGHBOR);  // 24 neighborhoods of laser point

              // Get the corresponding position(row and column) of laser point
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

              tmp_feature->cor_point = query_cloud[y][x];  // laser data point

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

          Q_EMIT setGUIStatus("Save global spatial features and local SURF features to database files successfully");
          break;
        }
        else
        {
          switch (QMessageBox::critical (NULL, tr ("Critical"), "~Failed~\n Invalid file name, please input file name correctly!",
                                         QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))
          {
            case QMessageBox::Yes:
              break;

            case QMessageBox::No:
              return;
          }
        }
      }  // End of while
    }    // if (sr.drawSURFFeatures (BA_iplimage, model_descriptors, n))
  }      // if (sr.extractGlobalFeature (query_cloud, &global_feature))
}

void
GraphManager::placeRecognition ()
{
  SceneRecognition sr;
  GlobalFeature global_feature;
  if (sr.extractGlobalFeature (query_cloud, &global_feature)) // Extract global spatial features
  {
    BearingAngle ba;
    BA_iplimage = ba.generateBAImage (query_cloud, cloud_width, cloud_height);
    QImage BA_qimage = ba.cvIplImage2QImage (ba.getChannelsImage (BA_iplimage));
    cvReleaseImage (&ba.channels_image);
    drawQImageText (BA_qimage, "Query scene BA image");
    Q_EMIT setGUIStatus("Show Bearing-Angle image and Features image of Query scene");
    Q_EMIT newQueryBAImage(BA_qimage);

    CvMemStorage* storage = cvCreateMemStorage (0);
    sr.cvSURFInitialize ();

    CvSeq* model_descriptors = sr.cvSURFDescriptor (BA_iplimage, storage, 4., 1);
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

      temp_feature->cor_point = query_cloud[y][x];  // laser data point
    }

    if (sr.drawSURFFeatures (BA_iplimage, model_descriptors, n)) // Mark features on the Bearing-Angle image
    {
      QImage features_qimage = ba.cvIplImage2QImage (ba.getChannelsImage (BA_iplimage));
      drawQImageText (features_qimage, "Query scene features image");
      Q_EMIT newFeaturesImage(features_qimage);

      QString message;
      QMessageBox::about (NULL, tr ("Tips"), message.sprintf ("The number of features is %d.\n"
                                             "Please select database files what you want to load!", n));
      // Load the SURF features from database files
      sr.readFeaturesFromFile ();

      int file_num = sr.global_descriptors.size ();
      Q_EMIT setGUIStatus(message.sprintf ("Select %d database files totally", file_num));

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
            double area_distance1 = (sr.global_descriptors[next].area - global_feature.area) *
                                    (sr.global_descriptors[next].area - global_feature.area);
            double duty_cycle_distance1 = (sr.global_descriptors[next].duty_cycle - global_feature.duty_cycle) *
                                          (sr.global_descriptors[next].duty_cycle - global_feature.duty_cycle);
            double area_distance2 = (sr.global_descriptors[best].area - global_feature.area) *
                                    (sr.global_descriptors[best].area - global_feature.area);
            double duty_cycle_distance2 = (sr.global_descriptors[best].duty_cycle - global_feature.duty_cycle) *
                                          (sr.global_descriptors[best].duty_cycle - global_feature.duty_cycle);

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
          CvSeq* seq = sr.findSURFMatchPairs (model_descriptors, n, sr.surf_descriptors[best], storage);

          // Get the matching degree
          double match_degree = sr.getMatchDegree (seq);
          // The determined factor for whether query scene is one of database scenes
          if (sr.eff_match_pairs >= MIN_NUMBER && match_degree >= MIN_VALUE)
          {
            QString file_full_name = sr.files[best];
            QFileInfo info (file_full_name);
            QString file_name = info.baseName ();
            QString place_name = file_name;

            Q_EMIT setGUIStatus("Place Recognition is accomplished Successfully");
            QMessageBox::about (NULL, tr ("Tips"), message.sprintf ("Place recognition is accomplished successfully!\n\n"
                                "The query place should be %s .\n"
                                "    Matching degree: %lf \n"
                                "    Effective matching pairs: %d",
                                (const char*) file_name.toLocal8Bit (), match_degree, sr.eff_match_pairs));

            file_name = info.path () + "/" + info.baseName () + ".pcd";
            std::string lib_file_name = std::string ((const char*) file_name.toLocal8Bit ());

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            /* Load the corresponding scene in database */
            if (pcl::io::loadPCDFile<pcl::PointXYZ> (lib_file_name, *cloud) == -1)
            {
              PCL_ERROR ("Error!\nCouldn't read PCD file.\n");
              return;
            }

            int points_number = cloud->width * cloud->height;
            Q_EMIT setGUIStatus(message.sprintf ("Loaded %d data points from PCD file", points_number));

            int lib_cloud_width = cloud->width;
            int lib_cloud_height = cloud->height;

            lib_cloud.clear ();
            pcl::PointXYZ temp_point;
            std::vector < pcl::PointXYZ > temp_vector;

            // Push the raw point cloud data to the 2-D vector(lib_cloud)
            for (size_t i = 0; i < cloud->points.size (); ++i)
            {
              temp_point.x = cloud->points[i].x;
              temp_point.y = cloud->points[i].y;
              temp_point.z = cloud->points[i].z;
              temp_vector.push_back (temp_point);

              if (!((i + 1) % cloud->width))
              {
                lib_cloud.push_back (temp_vector);
                temp_vector.clear ();
              }
            }

            Q_EMIT setGUIStatus("Show 3D laser scanning data and Bearing-Angle image of " + place_name + " room");

            if (lib_cloud.size () > 0)
            {
              cvReleaseImage (&BA_iplimage);
              cvReleaseImage (&ba.channels_image);
              BA_iplimage = ba.generateBAImage (lib_cloud, lib_cloud_width, lib_cloud_height);
              QImage lib_BA_qimage = ba.cvIplImage2QImage (ba.getChannelsImage (BA_iplimage));
              drawQImageText (lib_BA_qimage, "Corresponding scene BA image");
              Q_EMIT newLibBAImage(lib_BA_qimage);
            }
            break;
          }

          check_number++;
          if (check_number == file_num)
          {
            switch (QMessageBox::question (NULL, tr ("Question"), "Sorry, there is not available matching place in the database!\n"
                                           "Would you like to add the query scene to database?",
                                           QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes))
            {
              case QMessageBox::Yes:
                extractFeatures ();
                break;

              case QMessageBox::No:
                break;
            }
          }
        }  // End of while
      }    // if (file_num > 0)
    }      // if (sr.drawSURFFeatures(BA_iplimage, model_descriptors, n))
  }        // if (sr.extractGlobalFeature(query_cloud, &global_feature))
}

void
GraphManager::drawQImageText (QImage & image, const QString & str)
{
  // Construct QPainter for the QImage
  QPainter painter (&image);
  painter.setCompositionMode (QPainter::CompositionMode_SourceIn);

  // set the pen and font for painter
  painter.setPen (Qt::blue);
  painter.setFont (QFont ("Times", 16, QFont::Normal, true));

  painter.drawText (QPoint (5, 20), str);
}

