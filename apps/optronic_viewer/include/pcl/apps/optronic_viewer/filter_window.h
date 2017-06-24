/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright (c) 2012-, Open Perception, Inc.
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
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES  (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef PCL_APPS_OPTRONIC_VIEWER_FILTER_WINDOW_H_
#define PCL_APPS_OPTRONIC_VIEWER_FILTER_WINDOW_H_

#include <boost/shared_ptr.hpp>

#include <pcl/apps/optronic_viewer/qt.h>
#include <pcl/apps/optronic_viewer/openni_grabber.h>
#include <pcl/apps/optronic_viewer/cloud_filter.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <fz_api.h>


namespace pcl
{
  namespace apps
  {
    namespace optronic_viewer
    {
      
      /** \brief Window class for wizards to create new filters. */
      class FilterWindow : public QWizard
      {
        Q_OBJECT

      public:
        /** \brief Creates a new wizard for creating a filter. The filter to be
         *         created can be selected from the specified list of filters.
         *         The supplied factories are used to create the corresponding
         *         filters.
         */
        FilterWindow (
          std::vector<CloudFilterFactory*> & filter_factories, 
          std::vector<CloudFilter*> & filter_list);
        virtual ~FilterWindow ();

      public Q_SLOTS:
        /** \brief Called if a different item in the filter list is selected. */
        virtual void itemSelected (int id);
        /** \brief Called when the 'finish' button is pressed. */
        virtual void finished ();
        /** \brief Called when the 'next' button is pressed. */
        virtual void next ();

      Q_SIGNALS:
        /** \brief Ommitted when a filter is created. */
        void filterCreated ();

      protected:
        virtual void cleanupPage (int id)
        {
          std::cerr << "cleanup page" << std::endl;
        }

      private:
        /** \brief Creates the page for selecting filters. */
        void createFilterSelectionPage ();
        /** \brief Fills the combo box used for selecting filters. */
        void fillFilterSelectionComboBox (QComboBox * combo_box);

      private:
        /** \brief List of filter factories used to create the available filters. */
        std::vector<CloudFilterFactory*> filter_factories_;

        /** \brief Combo box that holds the names of the available filters. */
        QComboBox * filter_selection_combo_box_;
        /** \brief Line edit used to specify the name of the created filter. */
        QLineEdit * filter_name_line_edit_;

        int last_added_page_id_;

        /** \brief The destination for the newly created filter. */
        std::vector<CloudFilter*> * filter_list_;

        /** \brief The filter to be created. */
        CloudFilter * filter_;
      };
    }
  }
}

#endif // PCL_APPS_OPTRONIC_VIEWER_FILTER_WINDOW_H_
