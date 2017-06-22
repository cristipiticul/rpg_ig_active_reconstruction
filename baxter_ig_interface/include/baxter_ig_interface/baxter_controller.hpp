#pragma once

#include <Eigen/Core>
#include <thread>
#include <mutex>
#include <movements/core>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include "ig_active_reconstruction/view.hpp"
#include "baxter_arm_movement/MoveArmToPoseAction.h"

#define BAXTER_MOVER_ACTION_SERVICE_NAME "/move_arm_to_pose"

namespace baxter_ig_interface
{
  /*! Simple ROS interface for moving a stereo camera within gazebo. Poses are assumed exactly without added noise.
   * It also features a TF information broadcaster.
   */
  class BaxterController
  {
  public:
    typedef ig_active_reconstruction::views::View View;

    /*! Constructor.
     * @param robot_base_frame_name: the frame name of the base (fixed). Used to retrieve the currentPose
     * @param camera_optical_frame_name: the frame name of the camera (optical means the frame which has Z pointing forwards).
     *                 Used to retrieve the currentPose
     * @param camera_eef_frame_name: the frame name of the end-effector to which the camera is attached.
     *                 (The transform between end_effector_frame_name and camera_optical_frame_name should be static/constant)
     * @param object_eef_frame_name: the frame name of the end-effector that holds the object.
     */
    BaxterController(std::string robot_base_frame_name, std::string camera_optical_frame_name, std::string camera_eef_frame_name, std::string object_eef_frame_name);
    
    /*! Stops the thread on destruction.*/
    virtual ~BaxterController();
    
    /*! Tells the robot to get the camera to a new view
    * @param new_pose where to move to
    * @return false if the operation failed
    */
    virtual bool moveTo( View& new_view );
    
    /*! Returns the current camera pose.
     * @throws std::runtime_error If reception failed.
     */
    virtual movements::Pose currentPose();

  private:
    /*!
     * Moves the end-effector (gripper) of the arm to
     * the specified pose.
     * @param limb "left" or "right", depending on which arm we want to move
     * @param pose The desired end-effector pose
     */
    bool moveEndEffectorTo(std::string limb, geometry_msgs::Pose pose);

    /*! Converts from viewpoint pose to end-effector pose.
     * @param viewpoint the input viewpoint
     * @param end_effector_pose (OUTPUT) the result
     * @return true on success
     */
    bool viewpointToEndEffectorPose(geometry_msgs::Pose viewpoint, geometry_msgs::Pose &end_effector_pose);

    /*! Reads a transform using tf_listener
     * @param target_frame The base frame (the one to which we want to convert points to)
     * @param source_frame The frame whose origin and rotation will be found by this call
     */
    bool getTransform(std::string target_frame, std::string source_frame, tf::StampedTransform &result);

    bool has_moved_;
    
    bool keepPublishing_; //! Thread runs as long as this is true
    std::thread publisher_;
    std::mutex protector_;
    
    // Used only for Gazebo, when the x axis points forward (not the z axis)
    // Eigen::Quaterniond cam_to_image_; //! Transformation from camera model to image frame.
    
    actionlib::SimpleActionClient<baxter_arm_movement::MoveArmToPoseAction> mover_;

    std::string robot_base_frame_name_;
    std::string object_eef_frame_name_;
    std::string camera_optical_frame_name_;
    std::string camera_eef_frame_name_;
    tf::TransformListener tf_listener_;
  };
  
}
