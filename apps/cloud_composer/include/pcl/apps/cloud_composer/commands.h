/*
 * Software License Agreement  (BSD License)
 *
 *  Point Cloud Library  (PCL) - www.pointclouds.org
 *  Copyright  (c) 2012, Jeremie Papon.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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

#pragma once

#include <pcl/apps/cloud_composer/items/cloud_item.h>

#include <QUndoCommand>

namespace pcl
{
  namespace cloud_composer
  {
    class AbstractTool;
    class ProjectModel;
    struct OutputPair
    {
      QList <const CloudComposerItem*> input_items_;
      QList <CloudComposerItem*> output_items_;
    };


    
    class CloudCommand : public QUndoCommand
    {
      public: 
        CloudCommand (ConstItemList input_data, QUndoCommand* parent = nullptr);
        
        
        ~CloudCommand ();
        
        virtual bool
        runCommand (AbstractTool* tool) = 0;

        void 
        undo ()  override = 0;
        
        void
        redo () override = 0;
        
        //QList <CloudComposerItem*> 
       // executeToolOnTemplateCloud (AbstractTool* tool, ConstItemList &input_data);
        
        void 
        setProjectModel (ProjectModel* model);
        
        inline void
        setInputData (ConstItemList input_data)
        {
          original_data_ = std::move(input_data);
        }
      protected:
        /** \brief Removes the original item(s) from the model and replaces with the replacement(s)
         *  Replacements are only inserted once, original items must have same parent
         *  This stores the removed items in removed_items_
         */
        bool 
        replaceOriginalWithNew (const QList <const CloudComposerItem*>& originals, const QList <CloudComposerItem*>& new_items);
        
        /** \brief This removes new_items from the model and restores originals */
        bool
        restoreOriginalRemoveNew (const QList <const CloudComposerItem*>& originals, const QList <CloudComposerItem*>& new_items);
        
        ConstItemList original_data_;
        
        QMap <QStandardItem*, QStandardItem*> removed_to_parent_map_;
        QList <OutputPair> output_data_;
        ProjectModel* project_model_;
       
        /** \brief This determines if we delete original items or not on destruction 
         * If the command is being deleted because stack is at limit, then we want
         * to only delete the originals, since the command is staying for good (new items shouldn't be deleted)
         * On the other hand, if we destruct after an undo, then we want to delete the new items (but not the originals)
         */
        bool last_was_undo_;
        
        /** \brief This is used to check if a templated version of a tool can be used
         *  For this to return true, all items must be clouds, and must have the same template type 
         */
        bool 
        canUseTemplates (ConstItemList &input_data);
        
        bool can_use_templates_;
        int template_type_;
    };
    
    class ModifyItemCommand : public CloudCommand
    {
      public: 
        ModifyItemCommand (ConstItemList input_data, QUndoCommand* parent = nullptr);
    
        bool
        runCommand (AbstractTool* tool) override;
        
        void
        undo () override;
      
        void
        redo () override;
      private: 
        
      
      
    };
    
    class NewItemCloudCommand : public CloudCommand
    {
      public: 
        NewItemCloudCommand (ConstItemList input_data, QUndoCommand* parent = nullptr);
      
        bool
        runCommand (AbstractTool* tool) override;
        
        void
        undo () override;
      
        void
        redo () override;

    };
    

    class SplitCloudCommand : public CloudCommand
    {
      public: 
        SplitCloudCommand (ConstItemList input_data, QUndoCommand* parent = nullptr);
      
        bool
        runCommand (AbstractTool* tool) override;
        
        void
        undo () override;
      
        void
        redo () override;
      private:

    };  
    
    class DeleteItemCommand : public CloudCommand
    {
      public: 
        DeleteItemCommand (ConstItemList input_data, QUndoCommand* parent = nullptr);
      
        bool
        runCommand (AbstractTool* tool) override;
        
        void
        undo () override;
      
        void
        redo () override;
      private:
    };
    
    class MergeCloudCommand : public CloudCommand
    {
      public: 
        /** \brief Construct for a merge command
         *  \param[in] input_data Input list of CloudItem s from the project model which will be merged
         *  \param[in] temporary_clouds Input list of CloudItems which 
         */
        MergeCloudCommand (ConstItemList input_data, QUndoCommand* parent = nullptr);
      
        bool
        runCommand (AbstractTool* tool) override;
        
        void
        undo () override;
      
        void
        redo () override;
        
        inline void
        setSelectedIndicesMap( const QMap <CloudItem*, pcl::PointIndices::Ptr >& selected_item_index_map)
        {
          selected_item_index_map_ = selected_item_index_map;
        }
          
      private:
        QMap <CloudItem*, pcl::PointIndices::Ptr > selected_item_index_map_;
    };
  }
} 

Q_DECLARE_METATYPE (ConstItemList);
