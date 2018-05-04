#!/usr/bin/env python

import rospy
from cartesian_impedance_msgs.srv import *
from cartesian_impedance_msgs.msg import *


if __name__ == '__main__':
    try:
        rospy.init_node("contact_control_sevice_caller")

        rate = rospy.Rate(20)
        rospy.loginfo("[contact_control_sevice_caller] started")


        rospy.wait_for_service('configure_cartesian_impedance')
        try:
            configure_cartesian_impedance = rospy.ServiceProxy('configure_cartesian_impedance_blocking', ConfigureForceControl)
            set_cartesian_impedance = SetCartesianImpedance()

            #Disables the direction, when damping is set to 0.0
            set_cartesian_impedance.damping.translational.x = 80.0
            set_cartesian_impedance.damping.translational.y = 0.0
            set_cartesian_impedance.damping.translational.z = 0.0

            set_cartesian_impedance.max_cart_vel.set.linear.x = -0.05
            set_cartesian_impedance.max_cart_vel.set.linear.y = 0.0
            set_cartesian_impedance.max_cart_vel.set.linear.z = 0.0
            set_cartesian_impedance.max_cart_vel.set.angular.x = 0.0
            set_cartesian_impedance.max_cart_vel.set.angular.y = 0.0
            set_cartesian_impedance.max_cart_vel.set.angular.z = 0.0

            set_cartesian_impedance.max_ctrl_force.set.force.x = 5.0
            set_cartesian_impedance.max_ctrl_force.set.force.y = 5.0
            set_cartesian_impedance.max_ctrl_force.set.force.z = 5.0
            set_cartesian_impedance.max_ctrl_force.set.torque.x = 5.0
            set_cartesian_impedance.max_ctrl_force.set.torque.y = 5.0
            set_cartesian_impedance.max_ctrl_force.set.torque.z = 5.0

            set_cartesian_impedance.max_path_deviation.translation.x = 0.2
            set_cartesian_impedance.max_path_deviation.translation.y = 0.05
            set_cartesian_impedance.max_path_deviation.translation.z = 0.05
            set_cartesian_impedance.max_path_deviation.rotation.z = 0.05
            set_cartesian_impedance.max_path_deviation.rotation.z = 0.05
            set_cartesian_impedance.max_path_deviation.rotation.z = 0.05

            set_cartesian_force_control = SetCartesianForceCtrl()
            response = configure_cartesian_impedance(set_cartesian_impedance, set_cartesian_force_control)

            rospy.loginfo("Got move completed!")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


        while not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo("[contact_control_sevice_caller] stopped")

    except rospy.ROSInterruptException:
        pass
