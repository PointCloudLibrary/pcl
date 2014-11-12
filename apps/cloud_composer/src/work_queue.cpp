/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
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
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */


#include <pcl/apps/cloud_composer/qt.h>
#include <pcl/apps/cloud_composer/work_queue.h>
#include <pcl/apps/cloud_composer/tool_interface/abstract_tool.h>

pcl::cloud_composer::WorkQueue::WorkQueue (QObject* parent)
  : QObject (parent)
{
    
    
    
}


pcl::cloud_composer::WorkQueue::~WorkQueue ( )
{
  
  
}

void
pcl::cloud_composer::WorkQueue::enqueueNewAction (AbstractTool* new_tool, ConstItemList input_data)
{
  ActionPair new_action;
  //Create a command which will manage data for the tool
  new_action.command = new_tool->createCommand (input_data);
  new_action.tool = new_tool;
 
  work_queue_.enqueue (new_action);
  checkQueue ();
}

void
pcl::cloud_composer::WorkQueue::actionFinished (ActionPair finished_action)
{
  //Signal the project model that the command is done 
  emit commandComplete (finished_action.command);
  
  //Queue the tool for deletion
  finished_action.tool->deleteLater ();
  //Check if there are any remaining commands in queue
  checkQueue ();
  
}

void
pcl::cloud_composer::WorkQueue::checkQueue ( )
{
  if (work_queue_.length () > 0)
  {
    ActionPair action_to_execute = work_queue_.dequeue ();
    if (action_to_execute.command->runCommand (action_to_execute.tool))
    {
      //Success, send the command back to the main thread
      actionFinished (action_to_execute);
    }
    else
    {
      qDebug () << "FAILED TO EXECUTE COMMAND";
     //Failure, what to do with data?? 
    }
      
      
  }
    
  
}
