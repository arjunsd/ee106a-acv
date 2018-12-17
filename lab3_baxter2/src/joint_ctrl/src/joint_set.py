#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""
import argparse

import rospy
import numpy as np
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

def set_joints():

    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()
    curr_angle = {}
    joint_command = {}
    limb = left

    def dict_add(dict, joint_name, location):
        #current_position = limb.joint_angle(joint_name)
        dict[joint_name] = location

    def set_j(limb, joint_command):
        limb.set_joint_positions(joint_command)

    def curr_angle_update(limbJointSide):
        dict_add(curr_angle, limbJointSide[0], limb.joint_angle(limbJointSide[0]))
        dict_add(curr_angle, limbJointSide[1], limb.joint_angle(limbJointSide[1]))
        dict_add(curr_angle, limbJointSide[2], limb.joint_angle(limbJointSide[2]))
        dict_add(curr_angle, limbJointSide[3], limb.joint_angle(limbJointSide[3]))
        dict_add(curr_angle, limbJointSide[4], limb.joint_angle(limbJointSide[4]))
        dict_add(curr_angle, limbJointSide[5], limb.joint_angle(limbJointSide[5]))
        dict_add(curr_angle, limbJointSide[6], limb.joint_angle(limbJointSide[6]))



    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        print("Press F to continue, Esc to quit.")
        c = baxter_external_devices.getch()
            #catch Esc or ctrl-c
        if c in ['\x1b', '\x03']:
            done = True
            rospy.signal_shutdown("test finished.")
        elif c in ['f']:
            left_s0 = input("What should left_s0 be? :")
            left_s1 = input("What should left_s1 be? :")
            left_e0 = input("What should left_e0 be? :")
            left_e1 = input("What should left_e1 be? :")
            left_w0 = input("What should left_w0 be? :")
            left_w1 = input("What should left_w1 be? :")
            left_w2 = input("What should left_w2 be? :")

            if limb is left:

                dict_add(joint_command, lj[0], left_s0)
                dict_add(joint_command, lj[1], left_s1)
                dict_add(joint_command, lj[2], left_e0)
                dict_add(joint_command, lj[3], left_e1)
                dict_add(joint_command, lj[4], left_w0)
                dict_add(joint_command, lj[5], left_w1)
                dict_add(joint_command, lj[6], left_w2)
                curr_angle_update(lj)
                set_j(limb, joint_command)

                np.array([left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2])
                tolerance = 1
                while joint_command != curr_angle:
                    print("_______________")
                    print("Executing...")
                    set_j(limb, joint_command)
                    curr_angle_update(lj)
                    print("curr_angle =" )
                    print(curr_angle)
                    print("joint command")
                    print(joint_command)
                    for joint in curr_angle.keys(): 
                        curr_angle[joint] = round(curr_angle[joint],tolerance)
                print("done?")
                #while ((not (joint_command['left_s0']-tolerance < curr_angle['left_s0'] < joint_command['left_s0']+tolerance)) and (not (joint_command['left_s1']-tolerance < curr_angle['left_s1'] < joint_command['left_s1']+tolerance)) and (not (joint_command['left_e0']-tolerance < curr_angle['left_e0'] < joint_command['left_e0']+tolerance)) and (not (joint_command['left_e1']-tolerance < curr_angle['left_e1'] < joint_command['left_e1']+tolerance)) and (not (joint_command['left_w1']-tolerance < curr_angle['left_w1'] < joint_command['left_w1']+tolerance))  and (not (joint_command['left_w0']-tolerance < curr_angle['left_w0'] < joint_command['left_w0']+tolerance)) and (not (joint_command['left_s1']-tolerance < curr_angle['left_s1'] < joint_command['left_s1']+tolerance)) and (not (joint_command['left_s0']-tolerance < curr_angle['left_s0'] < joint_command['left_s0']+tolerance)) and (not (joint_command['left_w2']-tolerance < curr_angle['left_w2'] < joint_command['left_w2']+tolerance))):
					# print("Executing...")
					# set_j(limb, joint_command)
					# curr_angle_update(lj)
					# time.sleep(.01) #delay 10 ms

            if limb is right:

				dict_add(joint_command, rj[0], left_s0)
				dict_add(joint_command, rj[1], left_s1)
				dict_add(joint_command, rj[2], left_e0)
				dict_add(joint_command, rj[3], left_e1)
				dict_add(joint_command, rj[4], left_w0)
				dict_add(joint_command, rj[5], left_w1)
				dict_add(joint_command, rj[6], left_w2)
				curr_angle_update(rj)
				set_j(limb, joint_command)

				while ((curr_angle['left_s0'] not in range(joint_command['left_s0']-0.5, joint_command['left_s0']+0.5)) and (curr_angle['left_s1'] not in range(joint_command['left_s1']-0.5, joint_command['left_s1']+0.5)) and (curr_angle['left_e0'] not in range(joint_command['left_e0']-0.5, joint_command['left_e0']+0.5)) and (curr_angle['left_e1'] not in range(joint_command['left_e1']-0.5, joint_command['left_e1']+0.5)) and (curr_angle['left_w0'] not in range(joint_command['left_w0']-0.5, joint_command['left_w0']+0.5)) and (curr_angle['left_w1'] not in range(joint_command['left_w1']-0.5, joint_command['left_w1']+0.5)) and (curr_angle['left_w2'] not in range(joint_command['left_w2']-0.5, joint_command['left_w2']+0.5))):
					print("Executing...")
					set_j(limb, joint_command)
					curr_angle_update(rj)
				# while joint_command != curr_angle:
                #     print("Executing...")
                #     set_j(limb, joint_command)
                #     curr_angle_update(rj)




        else:
            print("invalid command...try again")



                
                


                    




def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)

    bindings = {
    #   key: (function, args, description)
        '9': (set_j, [left, lj[0], 0.1], "left_s0 increase"),
        '6': (set_j, [left, lj[0], -0.1], "left_s0 decrease"),
        '8': (set_j, [left, lj[1], 0.1], "left_s1 increase"),
        '7': (set_j, [left, lj[1], -0.1], "left_s1 decrease"),
        'o': (set_j, [left, lj[2], 0.1], "left_e0 increase"),
        'y': (set_j, [left, lj[2], -0.1], "left_e0 decrease"),
        'i': (set_j, [left, lj[3], 0.1], "left_e1 increase"),
        'u': (set_j, [left, lj[3], -0.1], "left_e1 decrease"),
        'l': (set_j, [left, lj[4], 0.1], "left_w0 increase"),
        'h': (set_j, [left, lj[4], -0.1], "left_w0 decrease"),
        'k': (set_j, [left, lj[5], 0.1], "left_w1 increase"),
        'j': (set_j, [left, lj[5], -0.1], "left_w1 decrease"),
        '.': (set_j, [left, lj[6], 0.1], "left_w2 increase"),
        'n': (set_j, [left, lj[6], -0.1], "left_w2 decrease"),
        ',': (grip_left.close, [], "left: gripper close"),
        'm': (grip_left.open, [], "left: gripper open"),
        '/': (grip_left.calibrate, [], "left: gripper calibrate"),

        # '4': (set_j, [right, rj[0], 0.1], "right_s0 increase"),
        # '1': (set_j, [right, rj[0], -0.1], "right_s0 decrease"),
        # '3': (set_j, [right, rj[1], 0.1], "right_s1 increase"),
        # '2': (set_j, [right, rj[1], -0.1], "right_s1 decrease"),
        # 'r': (set_j, [right, rj[2], 0.1], "right_e0 increase"),
        # 'q': (set_j, [right, rj[2], -0.1], "right_e0 decrease"),
        # 'e': (set_j, [right, rj[3], 0.1], "right_e1 increase"),
        # 'w': (set_j, [right, rj[3], -0.1], "right_e1 decrease"),
        # 'f': (set_j, [right, rj[4], 0.1], "right_w0 increase"),
        # 'a': (set_j, [right, rj[4], -0.1], "right_w0 decrease"),
        # 'd': (set_j, [right, rj[5], 0.1], "right_w1 increase"),
        # 's': (set_j, [right, rj[5], -0.1], "right_w1 decrease"),
        # 'v': (set_j, [right, rj[6], 0.1], "right_w2 increase"),
        # 'z': (set_j, [right, rj[6], -0.1], "right_w2 decrease"),
        # 'c': (grip_right.close, [], "right: gripper close"),
        # 'x': (grip_right.open, [], "right: gripper open"),
        # 'b': (grip_right.calibrate, [], "right: gripper calibrate"),
     }
    done = False
    print("Controlling joints. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                done = True
                rospy.signal_shutdown("Example finished.")
            elif c in bindings:
                cmd = bindings[c]
                #expand binding to something like "set_j(right, 's0', 0.1)"
                cmd[0](*cmd[1])
                print("command: %s" % (cmd[2],))
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    set_joints()
    print("Done.")


if __name__ == '__main__':
    main()