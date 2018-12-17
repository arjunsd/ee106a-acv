#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from baxter_interface import gripper as robot_gripper
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg

def main():
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():
        raw_input('Press [ Enter ]: ')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.808
        request.ik_request.pose_stamped.pose.position.y = 0.259
        request.ik_request.pose_stamped.pose.position.z = -0.113      
        request.ik_request.pose_stamped.pose.orientation.x = -0.097
        request.ik_request.pose_stamped.pose.orientation.y = 0.995
        request.ik_request.pose_stamped.pose.orientation.z = 0.016
        request.ik_request.pose_stamped.pose.orientation.w = 0.025


        #Construct the request2
        request2 = GetPositionIKRequest()
        request2.ik_request.group_name = "left_arm"
        request2.ik_request.ik_link_name = "left_gripper"
        request2.ik_request.attempts = 20
        request2.ik_request.pose_stamped.header.frame_id = "base"
        
        #Set the desired orientation for the end effector HERE
        request2.ik_request.pose_stamped.pose.position.x = 0.798
        request2.ik_request.pose_stamped.pose.position.y = -0.005
        request2.ik_request.pose_stamped.pose.position.z = -0.104       
        request2.ik_request.pose_stamped.pose.orientation.x = -0.053
        request2.ik_request.pose_stamped.pose.orientation.y = 0.999
        request2.ik_request.pose_stamped.pose.orientation.z = 0.009
        request2.ik_request.pose_stamped.pose.orientation.w = -0.011

        #setup the right_gripper
        right_gripper = robot_gripper.Gripper('left')
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            response2 = compute_ik(request2)
            
            #Print the response HERE
            print(response)
            print(response2)
            group = MoveGroupCommander("left_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)
            group.go()
            raw_input('Press [ Enter ]: ')
            right_gripper.close()
            rospy.sleep(1.0)

            group.set_pose_target(request2.ik_request.pose_stamped)
            group.go()
            right_gripper.open()
            rospy.sleep(1.0)




            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute

            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
    main()

