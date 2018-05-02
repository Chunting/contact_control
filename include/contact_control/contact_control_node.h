#pragma once

#include <ros/ros.h>
#include <contact_control/contact_control.h>
#include <geometry_msgs/WrenchStamped.h>
#include "cartesian_impedance_msgs/ConfigureForceControl.h"

class ContactControlNode {
private:
    ros::NodeHandle nh_;
    ContactControl contact_control_ = ContactControl();
    std::vector<std::future<Contact::EndCondition>> pending_futures_;

    bool configure_cartesian_impedance(
        cartesian_impedance_msgs::ConfigureForceControl::Request &req,
        cartesian_impedance_msgs::ConfigureForceControl::Response &res);

    double get_stiffness(cartesian_impedance_msgs::ConfigureForceControl::Request &req, Contact::Dimension dim);

    double get_damping(cartesian_impedance_msgs::ConfigureForceControl::Request &req, Contact::Dimension dim);

    double
    get_max_path_deviation(cartesian_impedance_msgs::ConfigureForceControl::Request &req, Contact::Dimension dim);

    double get_ctrl_force(cartesian_impedance_msgs::ConfigureForceControl::Request &req, Contact::Dimension dim);

    double get_ctrl_torque(cartesian_impedance_msgs::ConfigureForceControl::Request &req, Contact::Dimension dim);

    double get_cart_vel(cartesian_impedance_msgs::ConfigureForceControl::Request &req, Contact::Dimension dim);

public:
    ContactControlNode(ros::NodeHandle nh) {
      this->nh_ = nh;
    };

    void init();
};
