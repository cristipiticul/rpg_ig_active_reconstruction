#include "../../include/baxter_ig_interface/baxter_controller.hpp"

#include "ros/ros.h"
#include "gazebo_msgs/SetModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <movements/ros_movements.h>
#include <stdexcept>
// For tf <-> eigen conversions
#include <tf_conversions/tf_eigen.h>
// For tf <-> ROS msg conversions
#include <tf/transform_datatypes.h>
// For eigen <-> ROS msg conversions

namespace baxter_ig_interface
{
  typedef ig_active_reconstruction::views::View View;

  /** TODO:
   * Move additional fields functions to a "view utils" source file.
   */

  static bool getAdditionalFieldValueFromView( const View& view, std::string field_name, double& value )
  {
    bool found = false;
    for( unsigned int i = 0; i < view.additionalFieldsNames().size() && !found; i++ )
    {
      if (view.additionalFieldsNames()[i] == field_name)
      {
        value = view.additionalFieldsValues()[i];
        found = true;
      }
    }
    return found;
  }

  static bool getPoseFromViewAdditionalFields( const View& view, std::string frame_name, geometry_msgs::Pose& result )
  {
    bool success = true;

    success = success && (getAdditionalFieldValueFromView(view, frame_name + "/position/x", result.position.x));
    success = success && (getAdditionalFieldValueFromView(view, frame_name + "/position/y", result.position.y));
    success = success && (getAdditionalFieldValueFromView(view, frame_name + "/position/z", result.position.z));
    success = success && (getAdditionalFieldValueFromView(view, frame_name + "/orientation/x", result.orientation.x));
    success = success && (getAdditionalFieldValueFromView(view, frame_name + "/orientation/y", result.orientation.y));
    success = success && (getAdditionalFieldValueFromView(view, frame_name + "/orientation/z", result.orientation.z));
    success = success && (getAdditionalFieldValueFromView(view, frame_name + "/orientation/w", result.orientation.w));

    if( !success )
    {
      ROS_WARN("BaxterController: could not read %s pose from View.", frame_name.c_str());
      std::cout << "Known field names:";
      for( unsigned int i = 0; i < view.additionalFieldsNames().size(); i++ )
      {
        std::cout << '\n' << view.additionalFieldsNames()[i];
      }
      std::cout << '\n' << '\n';
    }
    return success;
  }

  static bool getCameraPoseFromViewAdditionalFields( const View& view, geometry_msgs::Pose& result )
  {
    return getPoseFromViewAdditionalFields(view, "camera", result);
  }

  static bool getObjectEefPoseFromViewAdditionalFields( const View& view, geometry_msgs::Pose& result )
  {
    return getPoseFromViewAdditionalFields(view, "object_eef", result);
  }

  BaxterController::BaxterController(std::string robot_base_frame_name, std::string camera_optical_frame_name, std::string camera_eef_frame_name, std::string object_eef_frame_name)
  : has_moved_(false)
  , keepPublishing_(false)
  //, cam_to_image_(0.5, 0.5, -0.5, 0.5)
  , robot_base_frame_name_(robot_base_frame_name)
  , camera_optical_frame_name_(camera_optical_frame_name)
  , camera_eef_frame_name_(camera_eef_frame_name)
  , object_eef_frame_name_(object_eef_frame_name)
  , mover_(BAXTER_MOVER_ACTION_SERVICE_NAME)
  {
    ROS_INFO("Waiting for %s action server...", BAXTER_MOVER_ACTION_SERVICE_NAME);
    mover_.waitForServer();
    ROS_INFO("Done.");
  }

  BaxterController::~BaxterController()
  {
    keepPublishing_ = false;
    if (publisher_.joinable())
      publisher_.join();
  }

  bool BaxterController::moveTo(View& new_view)
  {
    {
      std::lock_guard < std::mutex > guard(protector_);
      has_moved_ = true;
    }

    geometry_msgs::Pose camera_pose;
    if( !getCameraPoseFromViewAdditionalFields(new_view, camera_pose))
    {
      return false;
    }
    geometry_msgs::Pose camera_eef_pose;
    if( !viewpointToEndEffectorPose(camera_pose, camera_eef_pose) )
    {
      return false;
    }

    geometry_msgs::Pose object_eef_pose;
    if( !getObjectEefPoseFromViewAdditionalFields(new_view, object_eef_pose) )
    {
      return false;
    }


    if( !moveEndEffectorTo("right", camera_eef_pose) )
    {
      return false;
    }

    if( !moveEndEffectorTo("left", object_eef_pose) )
    {
      return false;
    }

    ROS_INFO("Movement was executed successfully!");
    return true;
  }

  bool BaxterController::moveEndEffectorTo(std::string limb, geometry_msgs::Pose pose)
  {
    bool success = false;
    baxter_arm_movement::MoveArmToPoseGoal goal;
    goal.limb = limb;
    goal.pose = pose;
    ROS_INFO("baxter_ig_interface::BaxterController: Sending goal and waiting for result...");
    mover_.sendGoal(goal);
    bool finishedBeforeTimeout = mover_.waitForResult(ros::Duration(20.0));
    ROS_INFO("baxter_ig_interface::BaxterController: Got result");
    if (finishedBeforeTimeout)
    {
      success = mover_.getResult()->success;
      if (!success)
      {
        ROS_WARN("baxter_ig_interface::BaxterController::moveEndEffectorTo: action server failed to execute the movement");
      }
    }
    else
    {
      ROS_WARN("baxter_ig_interface::BaxterController::moveEndEffectorTo: action server timed out");
      success = false;
    }
    return success;
  }

  /**
   * TODO: Refactor - Move this to another place. It is not the "job" of the BaxterController to make conversions.
   * Viewpoint represents the desired camera to base transform.
   * In order to move the arm, we need an end-effector pose.
   * Knowing that the end-effector-to-camera transform is static,
   * we can find the end-effector pose: apply the eef_to_camera
   * transform to the camera_to_base transform.
   */
  bool BaxterController::viewpointToEndEffectorPose(geometry_msgs::Pose viewpoint, geometry_msgs::Pose &end_effector_pose)
  {
    tf::StampedTransform eef_to_camera;

    if( !getTransform(camera_optical_frame_name_, camera_eef_frame_name_, eef_to_camera) )
    {
      ROS_WARN("baxter_ig_interface::BaxterController::viewpointToEndEffectorPose: could not convert camera pose to end-effector pose");
      return false;
    }

    tf::Pose camera_to_base;
    tf::poseMsgToTF(viewpoint, camera_to_base);

    tf::Pose eef_to_base = camera_to_base * eef_to_camera;

    tf::poseTFToMsg(eef_to_base, end_effector_pose);
    return true;
  }

  movements::Pose BaxterController::currentPose()
  {
    movements::Pose result;

    tf::StampedTransform transform;
    if( getTransform(object_eef_frame_name_, camera_optical_frame_name_, transform) )
    {
      tf::vectorTFToEigen(transform.getOrigin(), result.position);
      tf::quaternionTFToEigen(transform.getRotation(), result.orientation);
    }
    return result;
  }

  bool BaxterController::getTransform(const std::string target_frame, const std::string source_frame, tf::StampedTransform &result)
  {
    bool success = false;
    ros::Time now = ros::Time::now();
    tf::StampedTransform transform;
    try
    {
      tf_listener_.waitForTransform(target_frame, source_frame, now, ros::Duration(1.0));
      tf_listener_.lookupTransform(target_frame, source_frame, now, result);

      success = true;
    }
    catch (tf::TransformException &t)
    {
      ROS_WARN(
          "baxter_ig_interface::BaxterController::currentPose: Could not read transform between frames \"%s\" and \"%s\"",
          source_frame.c_str(), target_frame.c_str());
    }
    return success;
  }


}
