#include "../../include/baxter_ig_interface/baxter_communication_interface.hpp"

#include <thread>
#include <boost/thread/thread.hpp>
#include <boost/chrono/include.hpp>

namespace baxter_ig_interface
{
  
  BaxterCommunicationInterface::BaxterCommunicationInterface( ros::NodeHandle nh, std::shared_ptr<BaxterController> controller, std::string in_name, std::string out_name )
  : baxter_controller_(controller)
  , pcl_rerouter_(nh,in_name,out_name)
  {
    
  }
  
  BaxterCommunicationInterface::View BaxterCommunicationInterface::getCurrentView()
  {
    View current_view;
    
    try
    {
      current_view.pose() = baxter_controller_->currentPose();
    }
    catch(...)
    {
      current_view.bad() = true;
    }
    current_view.nonViewSpace() = true;
    
    return current_view;
  }
  
  BaxterCommunicationInterface::ReceptionInfo BaxterCommunicationInterface::retrieveData()
  {
    if( pcl_rerouter_.rerouteOneToSrv() )
    {
      return ReceptionInfo::SUCCEEDED;
    }
    return ReceptionInfo::FAILED;
  }
  
  BaxterCommunicationInterface::MovementCost BaxterCommunicationInterface::movementCost( View& target_view )
  {
    MovementCost cost;
    
    movements::Pose current;
    auto target_pos = target_view.pose().position;
    
    try
    {
      current = baxter_controller_->currentPose();
      auto distance = current.position - target_pos;
      cost.cost = distance.norm();
    }
    catch(...)
    {
      cost.exception = MovementCost::Exception::COST_UNKNOWN;
    }
    
    return cost;
  }
  
  BaxterCommunicationInterface::MovementCost BaxterCommunicationInterface::movementCost( View& start_view, View& target_view, bool fill_additional_information  )
  {
    MovementCost cost;
    
    auto distance = start_view.pose().position - target_view.pose().position;
    cost.cost = distance.norm();
    
    return cost;
  }
  
  bool BaxterCommunicationInterface::moveTo( View& target_view )
  {
    bool success = baxter_controller_->moveTo(target_view.pose());
    
    return success;
  }
  
  
}
