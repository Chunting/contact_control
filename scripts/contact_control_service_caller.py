#!/usr/bin/env python

import rospy
from cartesian_impedance_msgs.srv import *
from cartesian_impedance_msgs.msg import *
import franka_control.srv as franka_ctrl_srv
import actionlib
import kinova_msgs.msg


def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/m1n6s300_driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None


def open_fingers():
    gripper_client([3400.0, 3400.0, 3400.0])


def close_fingers():
    gripper_client([5440.0, 5440.0, 5440.0])

def close_fingers_around_motor():
    gripper_client([5000.0, 5000.0, 5000.0])

def open_door():
    open_fingers()

    rospy.loginfo("[contact_control_sevice_caller] fingers opened")

    # Move to the door ==============================

    configure_cartesian_impedance = rospy.ServiceProxy('configure_cartesian_impedance_blocking',
                                                       ConfigureForceControl)
    direction_control_laws = DirectionControlLaws()
    # Set type TYPE_UNKNOWN to disable dimension
    direction_control_laws.x.type = ControlLaw.TYPE_UNKNOWN
    direction_control_laws.y.type = ControlLaw.TYPE_UNKNOWN
    direction_control_laws.z.type = ControlLaw.TYPE_COMPLIANT_MOVE
    direction_control_laws.rx.type = ControlLaw.TYPE_UNKNOWN
    direction_control_laws.ry.type = ControlLaw.TYPE_UNKNOWN
    direction_control_laws.rz.type = ControlLaw.TYPE_UNKNOWN

    set_cartesian_impedance = SetCartesianImpedance()

    set_cartesian_impedance.damping.translational.x = 50.0
    set_cartesian_impedance.damping.translational.y = 50.0
    set_cartesian_impedance.damping.translational.z = 50.0

    set_cartesian_impedance.max_cart_vel.set.linear.x = 0.0
    set_cartesian_impedance.max_cart_vel.set.linear.y = 0.0
    set_cartesian_impedance.max_cart_vel.set.linear.z = 0.05
    set_cartesian_impedance.max_cart_vel.set.angular.x = 0.0
    set_cartesian_impedance.max_cart_vel.set.angular.y = 0.0
    set_cartesian_impedance.max_cart_vel.set.angular.z = 0.0

    set_cartesian_impedance.max_ctrl_force.set.force.x = 15.0
    set_cartesian_impedance.max_ctrl_force.set.force.y = 20.0
    set_cartesian_impedance.max_ctrl_force.set.force.z = 5.0
    set_cartesian_impedance.max_ctrl_force.set.torque.x = 15.0
    set_cartesian_impedance.max_ctrl_force.set.torque.y = 20.0
    set_cartesian_impedance.max_ctrl_force.set.torque.z = 5.0

    set_cartesian_impedance.max_path_deviation.translation.x = 0.0
    set_cartesian_impedance.max_path_deviation.translation.y = 0.0
    set_cartesian_impedance.max_path_deviation.translation.z = 0.1  # Change this to edit distance from door
    set_cartesian_impedance.max_path_deviation.rotation.z = 0.0
    set_cartesian_impedance.max_path_deviation.rotation.z = 0.0
    set_cartesian_impedance.max_path_deviation.rotation.z = 0.0

    set_cartesian_force_control = SetCartesianForceCtrl()
    response = configure_cartesian_impedance(set_cartesian_impedance, set_cartesian_force_control,
                                             direction_control_laws)

    # Grab the handle ==============================

    close_fingers()

    # Open the door ==============================

    direction_control_laws.x.type = ControlLaw.TYPE_UNKNOWN
    direction_control_laws.y.type = ControlLaw.TYPE_FOLLOWER  # TODO: which way the door opens?
    direction_control_laws.z.type = ControlLaw.TYPE_COMPLIANT_MOVE
    direction_control_laws.rx.type = ControlLaw.TYPE_UNKNOWN
    direction_control_laws.ry.type = ControlLaw.TYPE_UNKNOWN
    direction_control_laws.rz.type = ControlLaw.TYPE_UNKNOWN

    set_cartesian_impedance.max_cart_vel.set.linear.x = 0.0
    set_cartesian_impedance.max_cart_vel.set.linear.y = 0.1  # TODO: which way the door opens?
    set_cartesian_impedance.max_cart_vel.set.linear.z = -0.05  # Opposite of moving to door

    response = configure_cartesian_impedance(set_cartesian_impedance, set_cartesian_force_control,
                                             direction_control_laws)

    rospy.loginfo("Got move completed!")


if __name__ == '__main__':

    try:
        rospy.init_node("contact_control_sevice_caller")

        rate = rospy.Rate(20)
        rospy.loginfo("[contact_control_sevice_caller] started")

        rospy.wait_for_service('configure_cartesian_impedance')
        rospy.wait_for_service('/franka_control/set_full_collision_behavior')
        try:

            close_fingers_around_motor()

            configure_franka_collision = rospy.ServiceProxy('/franka_control/set_full_collision_behavior',
                                                            franka_ctrl_srv.SetFullCollisionBehavior)

            lower_torque_thresholds_acceleration = [20.0, 20.0, 19.0, 19.0, 16.0, 14.0, 13.0]
            upper_torque_thresholds_acceleration = [45.0, 45.0, 42.0, 40.0, 39.0, 37.0, 34.0]
            lower_torque_thresholds_nominal = [20.0, 20.0, 18.0, 17.0, 16.0, 15.0, 12.0]
            upper_torque_thresholds_nominal = [45.0, 45.0, 42.0, 40.0, 39.0, 37.0, 34.0]
            lower_force_thresholds_acceleration = [25.0, 25.0, 25.0, 20.0, 20.0, 20.0]
            upper_force_thresholds_acceleration = [50.0, 50.0, 50.0, 45.0, 45.0, 45.0]
            lower_force_thresholds_nominal = [20.0, 20.0, 20.0, 15.0, 15.0, 15.0]
            upper_force_thresholds_nominal = [50.0, 50.0, 50.0, 45.0, 45.0, 45.0]

            response = configure_franka_collision(lower_torque_thresholds_acceleration,
                                                  upper_torque_thresholds_acceleration,
                                                  lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
                                                  lower_force_thresholds_acceleration,
                                                  upper_force_thresholds_acceleration,
                                                  lower_force_thresholds_nominal,
                                                  upper_force_thresholds_nominal)
            rospy.loginfo(response)

            configure_cartesian_impedance = rospy.ServiceProxy('configure_cartesian_impedance_blocking',
                                                               ConfigureForceControl)

            direction_control_laws = DirectionControlLaws()
            # Set type TYPE_UNKNOWN to disable dimension
            direction_control_laws.x.type = ControlLaw.TYPE_UNKNOWN
            direction_control_laws.y.type = ControlLaw.TYPE_COMPLIANT_MOVE
            direction_control_laws.z.type = ControlLaw.TYPE_SPRING
            direction_control_laws.rx.type = ControlLaw.TYPE_UNKNOWN
            direction_control_laws.ry.type = ControlLaw.TYPE_UNKNOWN
            direction_control_laws.rz.type = ControlLaw.TYPE_UNKNOWN

            direction = -1
            set_cartesian_impedance = SetCartesianImpedance()

            set_cartesian_impedance.damping.translational.x = 35.0
            set_cartesian_impedance.damping.translational.y = 35.0
            set_cartesian_impedance.damping.translational.z = 20.0

            set_cartesian_impedance.max_cart_vel.set.linear.x = 0.05
            set_cartesian_impedance.max_cart_vel.set.linear.y = 0.05 * direction
            set_cartesian_impedance.max_cart_vel.set.linear.z = 0.05
            set_cartesian_impedance.max_cart_vel.set.angular.x = 0.0
            set_cartesian_impedance.max_cart_vel.set.angular.y = 0.0
            set_cartesian_impedance.max_cart_vel.set.angular.z = 0.0

            set_cartesian_impedance.max_ctrl_force.set.force.x = 25.0
            set_cartesian_impedance.max_ctrl_force.set.force.y = 25.0
            set_cartesian_impedance.max_ctrl_force.set.force.z = 25.0
            set_cartesian_impedance.max_ctrl_force.set.torque.x = 25.0
            set_cartesian_impedance.max_ctrl_force.set.torque.y = 25.0
            set_cartesian_impedance.max_ctrl_force.set.torque.z = 25.0

            set_cartesian_impedance.max_path_deviation.translation.x = 0.02
            set_cartesian_impedance.max_path_deviation.translation.y = 0.15
            set_cartesian_impedance.max_path_deviation.translation.z = 0.2  # Change this to edit distance from door
            set_cartesian_impedance.max_path_deviation.rotation.z = 0.0
            set_cartesian_impedance.max_path_deviation.rotation.z = 0.0
            set_cartesian_impedance.max_path_deviation.rotation.z = 0.0

            set_cartesian_impedance.stiffness.translational.x = 10.0
            set_cartesian_impedance.stiffness.translational.y = 0.0
            set_cartesian_impedance.stiffness.translational.z = 10.0
            set_cartesian_impedance.stiffness.rotational.x = 0.0
            set_cartesian_impedance.stiffness.rotational.y = 0.0
            set_cartesian_impedance.stiffness.rotational.z = 0.0


            while not rospy.is_shutdown():
                set_cartesian_impedance.max_cart_vel.set.linear.y = 0.05 * direction
                set_cartesian_force_control = SetCartesianForceCtrl()
                response = configure_cartesian_impedance(set_cartesian_impedance, set_cartesian_force_control,
                                                         direction_control_laws)
                direction = direction * -1

        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

        while not rospy.is_shutdown():
            rate.sleep()

        rospy.loginfo("[contact_control_sevice_caller] stopped")

    except rospy.ROSInterruptException:
        pass
