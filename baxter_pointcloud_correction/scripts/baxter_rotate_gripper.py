#!/usr/bin/env python

"""
Rotates the gripper and saves point clouds for
model reconstruction.

Required service:
rosrun pointcloud_pcd_binary with_tf /camera/depth_registered/points /left_gripper_base
"""

import rospy
import numpy as np
from std_srvs import srv
from baxter_arm_movement.baxter_helper import Limb

node_name="baxter_rotate_gripper"
save_pcd_service_name='save_pcd'

def main():
    rospy.init_node(node_name)
    limb = Limb('left')
    
    rospy.loginfo("Waiting for service %s", save_pcd_service_name)
    rospy.wait_for_service(save_pcd_service_name)
    save_pcd_service = rospy.ServiceProxy(save_pcd_service_name, srv.Empty)
    
    angles = limb.joint_angles()
    for angle in np.linspace(-3.0, 3.0, 30):
        angles['left_w2'] = angle
        limb.move_to_joint_positions(angles)
        save_pcd_service()
        print(angle)
        
    
if __name__=="__main__":
    main()