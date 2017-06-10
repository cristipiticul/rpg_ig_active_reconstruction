#pragma once

#include <Eigen/Core>
#include <thread>
#include <mutex>
#include <movements/core>
#include <tf/transform_listener.h>
#include "ig_active_reconstruction/view.hpp"

namespace baxter_ig_interface
{
  /*! Simple ROS interface for moving a stereo camera within gazebo. Poses are assumed exactly without added noise.
   * It also features a TF information broadcaster.
   */
  class BaxterController
  {
  public:
    /*! Constructor.
     * @param robot_base_frame_name: the frame name of the base (fixed). Used to retrieve the currentPose
     * @param camera_optical_frame_name: the frame name of the camera (optical means the frame which has Z pointing forwards).
     *                 Used to retrieve the currentPose
     */
    BaxterController(std::string robot_base_frame_name, std::string camera_optical_frame_name);
    
    /*! Stops the thread on destruction.*/
    virtual ~BaxterController();
    
    /*! Tells the robot to get the camera to a new view
    * @param new_pose where to move to
    * @return false if the operation failed
    */
    virtual bool moveTo( movements::Pose new_pose );
    
    /*! Returns the current camera pose.
     * @throws std::runtime_error If reception failed.
     */
    virtual movements::Pose currentPose();

  private:
    bool has_moved_;
    
    bool keepPublishing_; //! Thread runs as long as this is true
    std::thread publisher_;
    std::mutex protector_;
    
    Eigen::Quaterniond cam_to_image_; //! Transformation from camera model to image frame.
    
    std::string robot_base_frame_name_;
    std::string camera_optical_frame_name_;
    tf::TransformListener tf_listener_;
  };
  
}
