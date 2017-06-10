#include "../../include/baxter_ig_interface/baxter_controller.hpp"

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <movements/ros_movements.h>
#include <stdexcept>
#include <tf_conversions/tf_eigen.h>

namespace baxter_ig_interface
{

  BaxterController::BaxterController(std::string robot_base_frame_name, std::string camera_optical_frame_name)
  : has_moved_(false)
  , keepPublishing_(false)
  , cam_to_image_(0.5, 0.5, -0.5, 0.5)
  , robot_base_frame_name_(robot_base_frame_name)
  , camera_optical_frame_name_(camera_optical_frame_name)
  {

  }

  BaxterController::~BaxterController()
  {
    keepPublishing_ = false;
    if (publisher_.joinable())
      publisher_.join();
  }

  bool BaxterController::moveTo(movements::Pose pose)
  {
    {
      std::lock_guard < std::mutex > guard(protector_);
      has_moved_ = true;
    }

    ROS_INFO("baxter_ig_interface::BaxterController::moveTo: we were asked to move to:");
    std::cout << pose << '\n';

    return true;
  }

  movements::Pose BaxterController::currentPose()
  {
    movements::Pose result;

    ros::Time now = ros::Time::now();
    tf::StampedTransform transform;
    try
    {
      tf_listener_.waitForTransform(robot_base_frame_name_, camera_optical_frame_name_, now, ros::Duration(1.0));
      tf_listener_.lookupTransform(robot_base_frame_name_, camera_optical_frame_name_, now, transform);

      tf::vectorTFToEigen(transform.getOrigin(), result.position);
      tf::quaternionTFToEigen(transform.getRotation(), result.orientation);
    }
    catch (tf::TransformException &t)
    {
      ROS_WARN(
          "baxter_ig_interface::BaxterController::currentPose: Could not read transform between frames \"%s\" and \"%s\"",
          robot_base_frame_name_.c_str(), camera_optical_frame_name_.c_str());
    }

    return result;
  }

}
