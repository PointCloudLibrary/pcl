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
