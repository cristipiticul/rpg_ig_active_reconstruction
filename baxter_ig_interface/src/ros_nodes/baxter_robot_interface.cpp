#include <ros/ros.h>

#include <ig_active_reconstruction_ros/param_loader.hpp>
#include <ig_active_reconstruction_ros/robot_ros_server_ci.hpp>

#include "../../include/baxter_ig_interface/baxter_communication_interface.hpp"


/*! Implements a ROS node interface to a "flying" (staticly placed) gazebo stereo camera.
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_interface");
  ros::NodeHandle nh;
  
  // Load parameters
  //------------------------------------------------------------------
  std::string model_name;
  std::string camera_optical_frame_name, robot_base_frame_name, end_effector_frame_name;
  std::string sensor_in_topic, sensor_out_name;
  ros_tools::getExpParam(camera_optical_frame_name, "camera_optical_frame_name");
  ros_tools::getExpParam(robot_base_frame_name, "robot_base_frame_name");
  ros_tools::getExpParam(end_effector_frame_name, "end_effector_frame_name");
  ros_tools::getExpParam(sensor_in_topic, "sensor_in_topic");
  ros_tools::getExpParam(sensor_out_name, "sensor_out_name");
  
  using namespace baxter_ig_interface;
  
  // Controller
  //------------------------------------------------------------------
  std::shared_ptr<BaxterController> controller = std::make_shared<BaxterController>(robot_base_frame_name, camera_optical_frame_name, end_effector_frame_name);
  
  // Iar communication interface
  //------------------------------------------------------------------
  boost::shared_ptr<BaxterCommunicationInterface> robot_interface = boost::make_shared<BaxterCommunicationInterface>(nh,controller,sensor_in_topic,sensor_out_name);
  
  // Expose communication interface to ROS
  //------------------------------------------------------------------
  ig_active_reconstruction::robot::RosServerCI comm_unit(nh,robot_interface);
  
  
  // spin...
  ROS_INFO_STREAM("Baxter robot interface is setup.");
  ros::spin();
  
  return 0;
}
