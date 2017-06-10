#pragma once

#include <ros/ros.h>
#include "ig_active_reconstruction/robot_communication_interface.hpp"

#include "../baxter_ig_interface/pcl_rerouter.hpp"
#include "baxter_controller.hpp"

namespace baxter_ig_interface
{
  
  class BaxterCommunicationInterface: public ig_active_reconstruction::robot::CommunicationInterface
  {
  public:
    typedef ig_active_reconstruction::views::View View;
    typedef ig_active_reconstruction::robot::MovementCost MovementCost;
    
  public:
    /*! Constructor
     * @param nh Sets the ros node handle used for data subscription and publication.
     * @param controller Sets the controller of the camera that will be moved.
     * @param in_name Input name (Where pcl input is retrieved, relative to node namespace)
     * @param out_name How outputs (srv/topic) are advertised (relative to node namespace)
     */
    BaxterCommunicationInterface( ros::NodeHandle nh, std::shared_ptr<BaxterController> controller, std::string in_name="in", std::string out_name="out" );
    
    /*! returns the current view */
    virtual View getCurrentView();
    
    /*! Commands robot to retrieve new data.
    * @return information about what happened (data received, receival failed )
    */
    virtual ReceptionInfo retrieveData();
    
    /*! Returns the cost to move from the current view to the indicated view
    * @param target_view the next view
    * @return cost to move to that view: Simple distance
    */
    virtual MovementCost movementCost( View& target_view );
    
    /*! returns the cost to move from start view to target view
    * @param start_view the start view
    * @param target_view the target view
    * @param fill_additional_information if true then the different parts of the cost will be included in the additional fields as well
    * @return cost for the movement: Simple distance.
    */
    virtual MovementCost movementCost( View& start_view, View& target_view, bool fill_additional_information  );
    
    /*! Tells the robot to get the camera to a new view
    * @param target_view where to move to
    * @return false if the operation failed
    */
    virtual bool moveTo( View& target_view );
    
  private:
    std::shared_ptr<BaxterController> baxter_controller_; //! For movements etc.
    ros_tools::PclRerouter pcl_rerouter_; //! Since the gazebo stereo camera outputs a continuous stream of data but we are only interested in on dataset at a particular time, data retrieval consists in rerouting one data packet to the correct output where it is processed further.
  };
  
}
