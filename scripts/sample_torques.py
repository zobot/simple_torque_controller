#!/usr/bin/env python  
import roslib
roslib.load_manifest('simple_torque_controller')
import rospy
from std_msgs.msg import Float64MultiArray

import numpy as np
from subprocess import Popen, check_output
from IPython import embed
import time

def main():
    rospy.init_node('sample_torque_publisher')
    output = check_output("rosrun pr2_controller_manager pr2_controller_manager list", shell=True)
    output_lines = output.split('\n')
    to_kill_l_arm = False
    to_load_controller = True
    to_start_controller = True
    for line in output_lines: 
        if 'l_arm_controller' in line:
            print line
            to_kill_l_arm = True
        if 'simple_torque_controller' in line:
            print line
            to_load_controller = False
            if 'running' in line:
                to_start_controller = False
    if to_kill_l_arm:
        kill_l_arm = Popen("rosrun pr2_controller_manager pr2_controller_manager " \
                + "kill l_arm_controller", shell=True)
        rospy.sleep(0.2)
    if to_load_controller:
        load_simple_torque = Popen("rosrun pr2_controller_manager pr2_controller_manager " \
                + "load simple_torque_controller/SimpleTorqueController", shell=True)
        rospy.sleep(0.2)
    if to_start_controller:
        start_simple_torque = Popen("rosrun pr2_controller_manager pr2_controller_manager " \
                + "start simple_torque_controller/SimpleTorqueController", shell=True)
        rospy.sleep(0.2)
    torque_array_pub = rospy.Publisher('/rllab_torque', Float64MultiArray)

    current_msg = Float64MultiArray(data=np.zeros((7,)))

    while not rospy.is_shutdown():
        cur_time = time.clock()
        for i in range(7):
            current_msg.data[i] = (8.0 - i) * np.sin(12 * cur_time)
        torque_array_pub.publish(current_msg)
        rospy.sleep(0.05)


if __name__ == '__main__':
    main()
