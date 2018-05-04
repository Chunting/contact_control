#include "contact_control/contact_direction.h"

ContactDirection::ContactDirection() :
    isInitialized(false),
    direction(Contact::DIM_X),
    hasStart(false),
    velocityMax(0.0),
    forceTorqueMax(0.0),
    displacementMax(0.0),
    endOnDiff(false),
    ftDiffMax(0.0),
    springConstant(0.0),
    dampingCoeff(0.0),
    positionOffset(0.0),
    movementType(Contact::DIRECTION),
    travelMax(0.0),
    isReady(false),
    controlFrame(""),
    velFrame("") {
  // Do nothing
}

ContactDirection::~ContactDirection() {
  // delete listener;
}

void ContactDirection::initialize(Contact::Dimension dim, std::string vf, std::string cf, tf::TransformListener *list) {
  direction = dim;
  velFrame = vf;
  controlFrame = cf;
  listener = list;
  startFrame.frame_id_ = "start_frame";
  startFrame.setIdentity();
  isInitialized = true;
}

void ContactDirection::adjustSpringConstant(double k) {
  if (fabs(k) <= 1e-3) {
    ROS_ERROR("[ContactDirection] Spring constant must be non-zero");
  } else {
    springConstant = k;
  }
}

void
ContactDirection::setMovement(double vMax, double ftStall, double dMax, double ftMax, geometry_msgs::PoseStamped pose) {
  movementType = Contact::DIRECTION;
  velocityMax = vMax;
  stallForceTorque = ftStall;
  forceTorqueMax = ftMax;
  displacementMax = dMax;
  setStart(pose);
  if (fabs(velocityMax) <= 1e-3) {
    ROS_ERROR("[ContactDirection] Max velocity must be non zero");
    isReady = false;
  } else if (displacementMax <= 1e-3) {
    ROS_ERROR("[ContactDirection] Displacement max must be greater than zero");
    isReady = false;
  } else if (forceTorqueMax <= 1e-3 || stallForceTorque <= 1e-3) {
    ROS_ERROR("[ContactDirection] Force Torque max and stall force torque must be greater than zero");
    isReady = false;
  } else {
    springConstant = stallForceTorque / fabs(velocityMax); //TODO: kahtlane koht
    isReady = true;
  }
}

void ContactDirection::setSpring(double k, double b, double posOffset, double ftMax, geometry_msgs::PoseStamped pose) {
  displacementMax = 0.0;
  movementType = Contact::SPRING;
  setStart(pose);
  if (fabs(k) <= 1e-3 || fabs(b) <= 1e-3) {
    ROS_ERROR("[ContactDirection] Spring and damping constants must be non zero");
    isReady = false;
  } else {
    springConstant = k;
    dampingCoeff = b;
    positionOffset = posOffset;
    isReady = true;
  }
  forceTorqueMax = ftMax;
}

void ContactDirection::setSpring(double k, double b, double posOffset, double dMax, double ftMax,
                                 geometry_msgs::PoseStamped pose) {
  if (dMax <= -1e-3) {
    ROS_ERROR("[ContactDirection] Displacement max must be positive");
    isReady = false;
    return;
  } else {
    setSpring(k, b, posOffset, ftMax, pose);
    displacementMax = dMax;
  }
}

void ContactDirection::setFollower(double b, double ftMax, geometry_msgs::PoseStamped pose) {
  movementType = Contact::FOLLOWER;
  setStart(pose);
  if (fabs(b) <= 1e-3) {
    ROS_ERROR("[ContactDirection] Damping constant must be non zero");
    isReady = false;
  } else {
    ROS_INFO("[ContactDirection] Setting follower for direction %i", direction);
    dampingCoeff = b;
    isReady = true;
  }
  forceTorqueMax = ftMax;
}

double ContactDirection::getVelocity(double ft, geometry_msgs::PoseStamped pose) {
  if (endOnDiff) {
    writeFT(ft);
  }
  if (isReady) {
    if (Contact::SPRING == movementType) {
      //dampingCoeff - mida suurem, seda aeglasemalt liigub. Esialgu katsetada 1'ga, siis saab võrdustada springContanti max ft'ga
      //springConstant - mida suurem, seda
      /*if (fabs(getTravel(pose)) >= displacementMax) {
        //TODO jätta meelde, kuhu poole liikus (velocity märk) ning kui see muutub, siis lahti lasta...
      }*/
      //velocity = (1 / dampingCoeff * ft) - (springConstant * (getTravel(pose) /*+ positionOffset*/));
      velocity = (springConstant * -(getTravel(pose) + positionOffset) + (1 / dampingCoeff) * ft);
    } else if (Contact::DIRECTION == movementType) {

      double remainingTravel = displacementMax - fabs(getTravel(pose));
      ROS_INFO_THROTTLE(1,"[ContactDirection] remaining travel: %f", remainingTravel);

      if ((remainingTravel <= 0.02 && fabs(velocity) >= 0.02) || ft >= stallForceTorque) {
        if (velocity > 0) {
          velocity = velocity - deceleration;
        } else {
          velocity = velocity + deceleration;
        }
      } else {
        velocity = velocityMax;
      }

    } else if (Contact::FOLLOWER == movementType) {
      //getTravel(pose);
      //ROS_INFO_STREAM("FT for dim: " << direction << ". " << ft);
      if (std::abs(ft) > 2.5) {
        velocity = (1 / dampingCoeff) * ft;
      } else {//Slow down, if small enough force
        //Otherwise decelrate
        if (velocity > 0) {
          velocity = velocity - deceleration;
        } else {
          velocity = velocity + deceleration;
        }
      }

    } else {
      ROS_ERROR("[ContactDirection] Movement type is incorrect.");
      velocity = 0.0;
    }

    //If velocity already small enough, set it to 0
    if (std::abs(velocity) <= std::abs(deceleration)) {
      velocity = 0.0;
    }

  } else {
    velocity = 0.0;
  }


  return velocity;
}

void ContactDirection::reset() {
  isReady = false;
  hasStart = false;
  clearFT();
}

Contact::EndCondition ContactDirection::getCondition(double ft, geometry_msgs::PoseStamped pose) {
  Contact::EndCondition endCondition = Contact::NO_CONDITION;

  if (isReady) {
    if (fabs(ft) >= forceTorqueMax) {
      endCondition = Contact::FT_VIOLATION;
    }
    if (Contact::DIRECTION == movementType) {
      if (fabs(getTravel(pose)) >= displacementMax || checkDiff()) {
        endCondition = Contact::MAX_TRAVEL;
      } else if (checkDiff())
        endCondition = Contact::DIFF_VIOLATION;
    } else if (Contact::SPRING == movementType && displacementMax >= 1e-3) {
      if (fabs(getTravel(pose)) >= displacementMax) {
        endCondition = Contact::MAX_TRAVEL;
      }
    }
  }
  //ROS_INFO_STREAM("Dimension: " << direction << " FT: " << ft << " Travel: " << fabs(getTravel(pose)) << " Max: " << displacementMax);
  return endCondition;
}

void ContactDirection::writeFT(double ft) {
  ftHistory.add(ros::Time::now().toSec(), ft);
}

void ContactDirection::clearFT() {
  ftHistory.clear();
}

void ContactDirection::setMaxDiff(double diffMax, double time) {
  if (diffMax != 0.0) {
    endOnDiff = true;
    ftDiffMax = diffMax;
    ftHistory.initialize(time);
  } else {
    endOnDiff = false;
  }
}

bool ContactDirection::checkDiff() {
  if (endOnDiff)
    return ftHistory.getDiff() > ftDiffMax;
  return false;
}

void ContactDirection::setStart(geometry_msgs::PoseStamped pose) {
  // Find transform from world frame to the starting control frame
  startFrame.frame_id_ = "start_frame";
  startFrame.child_frame_id_ = velFrame;
  startFrame.stamp_ = pose.header.stamp;
  startFrame.setIdentity();
  if (velFrame.compare(controlFrame) != 0) {
    if (listener->waitForTransform(controlFrame, velFrame, pose.header.stamp, ros::Duration(1.0))) {
      listener->lookupTransform(controlFrame, velFrame, pose.header.stamp, startFrame);
    } else {
      ROS_ERROR("[ContactDirection] Could not find transform from world to control frame. Wait for transform timed out.");
    }
  }

  // Convert the start pose into the starting control frame.
  tf::poseStampedMsgToTF(pose, startPose);
  if (startPose.frame_id_.compare("start_frame") != 0) {
    toStartFrame(startPose);
  }
  travelMax = 0.0;
  hasStart = true;
}

bool ContactDirection::hasStartInfo() {
  return hasStart;
}

tf::StampedTransform ContactDirection::getStart() {
  return startFrame;
}

void ContactDirection::toStartFrame(geometry_msgs::PoseStamped &pose) {
  // Convert the geometry msg to tf
  tf::Stamped <tf::Pose> tfPose;
  tf::poseStampedMsgToTF(pose, tfPose);
  // Transform the frame
  toStartFrame(tfPose);
  // Convert back to geometry msgs
  tf::poseStampedTFToMsg(tfPose, pose);
}

void ContactDirection::toStartFrame(tf::Stamped <tf::Pose> &pose) {
  // First use TF listener to convert the pose to world frame.
  pose.stamp_ = ros::Time(0);
  if (velFrame.compare(pose.frame_id_) != 0) {
    if (listener->waitForTransform(velFrame, pose.frame_id_, ros::Time(0), ros::Duration(1.0))) {
      try {
        listener->transformPose(velFrame, pose, pose);
      }
      catch (tf::TransformException ex) {
        ROS_ERROR_STREAM("[ContactDirection] Could not transform pose to world frame. " << ex.what());
      }
    } else {
      ROS_ERROR("[ContactDirection] Could not transform pose to world frame. Wait for transform timed out.");
    }
  }
  // Next use the transform we found to convert to the starting frame.
  pose.setData(startFrame * pose);
  pose.frame_id_ = "start_frame";
}

double ContactDirection::getTravel(geometry_msgs::PoseStamped pose) {
  double travel = 0.0;
  tf::Stamped <tf::Pose> endPose;
  tf::poseStampedMsgToTF(pose, endPose);

  if (startPose.frame_id_.compare("start_frame") != 0) {
    toStartFrame(endPose);
  }
  //ROS_INFO_STREAM("Travel x: " << endPose.getOrigin().getX() - startPose.getOrigin().getX());
  //ROS_INFO_STREAM("Travel y: " << endPose.getOrigin().getY() - startPose.getOrigin().getY());
  //ROS_INFO_STREAM("Travel z: " << endPose.getOrigin().getZ() - startPose.getOrigin().getZ());
  ros::Time time = ros::Time::now();
  switch (direction) {
    case Contact::DIM_X:
      travel = endPose.getOrigin().getX() - startPose.getOrigin().getX();
      break;
    case Contact::DIM_Y:
      travel = endPose.getOrigin().getY() - startPose.getOrigin().getY();
      break;
    case Contact::DIM_Z:
      travel = endPose.getOrigin().getZ() - startPose.getOrigin().getZ();
      break;
    case Contact::DIM_RX: {
      tf::Stamped <tf::Pose> diff(endPose.inverseTimes(startPose), startPose.stamp_, startPose.frame_id_);
      double rX, rY, rZ;
      diff.getBasis().getRPY(rX, rY, rZ);
      travel = rX;
    }
      break;
    case Contact::DIM_RY: {
      tf::Stamped <tf::Pose> diff(endPose.inverseTimes(startPose), startPose.stamp_, startPose.frame_id_);
      double rX, rY, rZ;
      diff.getBasis().getRPY(rX, rY, rZ);
      travel = rY;
    }
      break;
    case Contact::DIM_RZ: {
      tf::Stamped <tf::Pose> diff(endPose.inverseTimes(startPose), startPose.stamp_, startPose.frame_id_);
      double rX, rY, rZ;
      diff.getBasis().getRPY(rX, rY, rZ);
      travel = rZ;
    }
      break;
    default:
      ROS_ERROR("[ContactDirection] Direction class initialized with incorrect direction variable");
      break;
  }
  //ROS_INFO_STREAM("Get travel for all dims. " << diff.getOrigin().getX() << " " << diff.getOrigin().getY() << " " << diff.getOrigin().getZ());
  //ROS_INFO_STREAM("Get travel for dim: 0. Travel: " << diff.getOrigin().getX());
  //ROS_INFO_STREAM("Get travel for dim: 1. Travel: " << diff.getOrigin().getY());
  //ROS_INFO_STREAM("Get travel for dim: 2. Travel: " << diff.getOrigin().getZ());
  return travel;
}
