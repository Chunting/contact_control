#include "contact_control/contact_control_node.h"

void ContactControlNode::init() {
  ros::ServiceServer service = nh_.advertiseService("configure_cartesian_impedance",
                                                    &ContactControlNode::configure_cartesian_impedance, this);

  //Contact control initial configuration
  //TODO: make configurable
  contact_control_.setFTTopic("wrench");
  contact_control_.setVelTopic("jog_arm_server/delta_jog_cmds");
  contact_control_.initialize("manipulator", "base_link", "base_link", "base_link", "base_link");
  ROS_INFO("ContactControlNode initialized successfully!");

  while (ros::ok()) {
    ros::Duration(0.2).sleep();
    if (!pending_futures_.empty()) {
      pending_futures_.back().wait();
      Contact::EndCondition end_condition = pending_futures_.back().get();
      pending_futures_.pop_back();
      ROS_INFO("EndCondition: %i", end_condition);
    }
  }
}

bool ContactControlNode::configure_cartesian_impedance(
    cartesian_impedance_msgs::ConfigureForceControl::Request &req,
    cartesian_impedance_msgs::ConfigureForceControl::Response &res) {
  contact_control_.stopMove();
  ROS_INFO("Previous move stopped");

  std::map<std::string, Contact::Dimension> string_to_dimensions;
  string_to_dimensions.insert(std::pair<std::string, Contact::Dimension>("X", Contact::Dimension::DIM_X));
  string_to_dimensions.insert(std::pair<std::string, Contact::Dimension>("Y", Contact::Dimension::DIM_Y));
  string_to_dimensions.insert(std::pair<std::string, Contact::Dimension>("Z", Contact::Dimension::DIM_Z));
  string_to_dimensions.insert(std::pair<std::string, Contact::Dimension>("RX", Contact::Dimension::DIM_RX));
  string_to_dimensions.insert(std::pair<std::string, Contact::Dimension>("RY", Contact::Dimension::DIM_RX));
  string_to_dimensions.insert(std::pair<std::string, Contact::Dimension>("RZ", Contact::Dimension::DIM_RX));

  for (auto const &s_to_d : string_to_dimensions) {
    Contact::Dimension dim = s_to_d.second;
    double stiffness = get_stiffness(req, dim);
    double damping = get_damping(req, dim);
    double max_path_deviation = get_max_path_deviation(req, dim);
    double max_ctrl_force = get_ctrl_force(req, dim);
    double max_ctrl_torque = get_ctrl_torque(req, dim);
    double max_ctrl_force_torque = std::min(max_ctrl_force, max_ctrl_torque);
    double max_cart_vel = get_cart_vel(req, dim); //TODO not used

    contact_control_.setFollower(dim, damping, max_ctrl_force_torque);
    //contact_control_.setSpring(dim, stiffness, damping, max_path_deviation, max_ctrl_force_torque);
    ROS_INFO(
        "Set spring rule with dimension: %s, "
            "stiffness: %f, damping: %f, "
            "max deviation: %f, max_ctrl_force_torque: %f", s_to_d.first.c_str(), stiffness,
        damping,
        max_path_deviation,
        max_ctrl_force_torque);
  }

  double global_max_force = 300.0;
  double global_max_torque = 300.0;
  double max_velocity = 0.2; // 0.0 - 1.0

  pending_futures_.push_back(contact_control_.moveAsync(global_max_force, global_max_torque, max_velocity));

  ROS_INFO("New move started");
  return true;
}

double ContactControlNode::get_stiffness(cartesian_impedance_msgs::ConfigureForceControl::Request &req,
                                            Contact::Dimension dim) {
  switch (dim) {
    case Contact::Dimension::DIM_X:
      return req.cart_impedance_params.stiffness.translational.x;
    case Contact::Dimension::DIM_Y:
      return req.cart_impedance_params.stiffness.translational.y;
    case Contact::Dimension::DIM_Z:
      return req.cart_impedance_params.stiffness.translational.z;
    case Contact::Dimension::DIM_RX:
      return req.cart_impedance_params.stiffness.rotational.x;
    case Contact::Dimension::DIM_RY:
      return req.cart_impedance_params.stiffness.rotational.y;
    case Contact::Dimension::DIM_RZ:
      return req.cart_impedance_params.stiffness.rotational.z;
    default:
      return 0.0;
  }
}

double
ContactControlNode::get_damping(cartesian_impedance_msgs::ConfigureForceControl::Request &req,
                                   Contact::Dimension dim) {
  switch (dim) {
    case Contact::Dimension::DIM_X:
      return req.cart_impedance_params.damping.translational.x;
    case Contact::Dimension::DIM_Y:
      return req.cart_impedance_params.damping.translational.y;
    case Contact::Dimension::DIM_Z:
      return req.cart_impedance_params.damping.translational.z;
    case Contact::Dimension::DIM_RX:
      return req.cart_impedance_params.damping.rotational.x;
    case Contact::Dimension::DIM_RY:
      return req.cart_impedance_params.damping.rotational.y;
    case Contact::Dimension::DIM_RZ:
      return req.cart_impedance_params.damping.rotational.z;
    default:
      return 0.0;
  }
}

double ContactControlNode::get_max_path_deviation(cartesian_impedance_msgs::ConfigureForceControl::Request &req,
                                                     Contact::Dimension dim) {
  switch (dim) {
    case Contact::Dimension::DIM_X:
      return req.cart_impedance_params.max_path_deviation.translation.x;
    case Contact::Dimension::DIM_Y:
      return req.cart_impedance_params.max_path_deviation.translation.y;
    case Contact::Dimension::DIM_Z:
      return req.cart_impedance_params.max_path_deviation.translation.z;
    case Contact::Dimension::DIM_RX:
      return req.cart_impedance_params.max_path_deviation.rotation.x;
    case Contact::Dimension::DIM_RY:
      return req.cart_impedance_params.max_path_deviation.rotation.y;
    case Contact::Dimension::DIM_RZ:
      return req.cart_impedance_params.max_path_deviation.rotation.z;
    default:
      return 0.0;
  }
}

double ContactControlNode::get_ctrl_force(cartesian_impedance_msgs::ConfigureForceControl::Request &req,
                                             Contact::Dimension dim) {
  switch (dim) {
    case Contact::Dimension::DIM_X:
      return req.cart_impedance_params.max_ctrl_force.set.force.x;
    case Contact::Dimension::DIM_Y:
      return req.cart_impedance_params.max_ctrl_force.set.force.y;
    case Contact::Dimension::DIM_Z:
      return req.cart_impedance_params.max_ctrl_force.set.force.z;
    case Contact::Dimension::DIM_RX:
      return req.cart_impedance_params.max_ctrl_force.set.force.x;
    case Contact::Dimension::DIM_RY:
      return req.cart_impedance_params.max_ctrl_force.set.force.y;
    case Contact::Dimension::DIM_RZ:
      return req.cart_impedance_params.max_ctrl_force.set.torque.z;
    default:
      return 0.0;
  }
}

double ContactControlNode::get_ctrl_torque(cartesian_impedance_msgs::ConfigureForceControl::Request &req,
                                              Contact::Dimension dim) {
  switch (dim) {
    case Contact::Dimension::DIM_X:
      return req.cart_impedance_params.max_ctrl_force.set.torque.x;
    case Contact::Dimension::DIM_Y:
      return req.cart_impedance_params.max_ctrl_force.set.torque.y;
    case Contact::Dimension::DIM_Z:
      return req.cart_impedance_params.max_ctrl_force.set.torque.z;
    case Contact::Dimension::DIM_RX:
      return req.cart_impedance_params.max_ctrl_force.set.torque.x;
    case Contact::Dimension::DIM_RY:
      return req.cart_impedance_params.max_ctrl_force.set.torque.y;
    case Contact::Dimension::DIM_RZ:
      return req.cart_impedance_params.max_ctrl_force.set.torque.z;
    default:
      return 0.0;
  }
}

double ContactControlNode::get_cart_vel(cartesian_impedance_msgs::ConfigureForceControl::Request &req,
                                           Contact::Dimension dim) {
  switch (dim) {
    case Contact::Dimension::DIM_X:
      return req.cart_impedance_params.max_cart_vel.set.linear.x;
    case Contact::Dimension::DIM_Y:
      return req.cart_impedance_params.max_cart_vel.set.linear.y;
    case Contact::Dimension::DIM_Z:
      return req.cart_impedance_params.max_cart_vel.set.linear.z;
    case Contact::Dimension::DIM_RX:
      return req.cart_impedance_params.max_cart_vel.set.linear.x;
    case Contact::Dimension::DIM_RY:
      return req.cart_impedance_params.max_cart_vel.set.linear.y;
    case Contact::Dimension::DIM_RZ:
      return req.cart_impedance_params.max_cart_vel.set.linear.z;
    default:
      return 0.0;
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "contact_control_node");
  ros::NodeHandle nh;

  ros::Rate r(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /*RobotMover robotMover = RobotMover();
  robotMover.moveToHome();
  ROS_INFO("Robot moved to home!");*/

  ContactControlNode contactControlNode = ContactControlNode(nh);
  contactControlNode.init();

  return 0;
}
