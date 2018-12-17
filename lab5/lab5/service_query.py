#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import lab3_skeleton as lab3


def main(x,y,z):
    #Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    
    #Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    
    while not rospy.is_shutdown():


        raw_input('Press enter to compute an IK solution:')
        
        #Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "left_arm"
        request.ik_request.ik_link_name = "left_gripper"
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"

        #Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = x
        request.ik_request.pose_stamped.pose.position.y = y
        request.ik_request.pose_stamped.pose.position.z = z
        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            #Send the request to the service
            response = compute_ik(request)
            message = response.solution.joint_state
			####
            #Print the contents of the message to the console
            theta = []
            theta.append(message.position[1])
            theta.append(message.position[2])
            theta.append(message.position[3])
            theta.append(message.position[4])
            theta.append(message.position[5])
            theta.append(message.position[6])
            theta.append(message.position[7])
            print("theta = ")
            print(theta)
            print("\n")
            print("------")
            print("gst = ")
            print(lab3.lab3(theta))
            print("--------------")
            print(response)
			#Print the response HERE
			####

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

#Python's syntax for a main() method
if __name__ == '__main__':
	x = input('Type X then press enter to compute an IK solution:')
	y = input('Type Y then press enter to compute an IK solution:')
	z = input('Type Z then press enter to compute an IK solution:')
	main(x,y,z)
