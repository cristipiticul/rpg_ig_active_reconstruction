#!/usr/bin/env python
import rospy
import rospkg
from ig_active_reconstruction_msgs.srv import *
from ig_active_reconstruction_msgs.msg import *

node_name="viewspace_ui"
get_current_view_service_name="robot/current_view"
add_to_viewspace_service_name="views/add"
save_viewspace_service_name="views/save"

def main():
    rospy.init_node(node_name)
    rospack = rospkg.RosPack()
    save_dir = rospack.get_path("baxter_ig_interface") + "/config/"

    rospy.loginfo("baxter_ig_interface::viewspace_ui: Waiting for service %s", get_current_view_service_name)
    rospy.wait_for_service(get_current_view_service_name)

    rospy.loginfo("baxter_ig_interface::viewspace_ui: Waiting for service %s", add_to_viewspace_service_name)
    rospy.wait_for_service(add_to_viewspace_service_name)
    
    rospy.loginfo("baxter_ig_interface::viewspace_ui: Waiting for service %s", save_viewspace_service_name)
    rospy.wait_for_service(save_viewspace_service_name)
    
    rospy.loginfo("baxter_ig_interface::viewspace_ui: done")
    
    get_pose_service = rospy.ServiceProxy(get_current_view_service_name, ViewRequest)
    add_to_viewspace = rospy.ServiceProxy(add_to_viewspace_service_name, ViewSpaceUpdate)
    save_viewspace = rospy.ServiceProxy(save_viewspace_service_name, ViewSpaceSave)

    while True:
        command = raw_input('Enter command (a->add view, s->save, q->quit): ')
        if command == 'a':
            current_view = get_pose_service().view
            result = add_to_viewspace([current_view]).update_result
            if result != 0: # 0 is success (see ViewSpaceUpdateResult in views_communication_interface.hpp)
                rospy.logerr("The add_to_viewspace service did not update successfully. Response code: %s" % response)
        elif command == 's':
            filename = raw_input('Enter the desired filename (it will be saved in the baxter_ig_interface/config folder: ')
            path_to_file = save_dir + filename
            save_viewspace(path_to_file)
        elif command == 'q':
            break
        else:
            rospy.logerr("Unrecognized command %s!" % command)
    return None
    
if __name__=="__main__":
    main()
