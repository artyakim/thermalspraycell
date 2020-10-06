//
// Created by profanter on 8/13/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <ros/ros.h>
#include <EdoControlSim.h>

#include <moveit/move_group/capability_names.h>


#include <tf/tf.h>

// This is the height of the base
#define OFFSET_BASE_BOTTOM_EDO_BOTTOM  0.135

EdoControlSim::EdoControlSim(const char* nodeHandleNamespace, const char* robotDescriptionParam) : EdoControl(true, nodeHandleNamespace), currentGoal(), previousGoal(), goalExecuted(true) {
    nh.param<std::string>("move_group", movegroup_name, "manipulator");
    nh.param<std::string>("ee_link", ee_link, "tool_link_ee");
    nh.param<std::string>("gripper_get_span_topic", gripper_get_span_topic, "edo_gripper_span");
    nh.param<std::string>("gripper_set_span_topic", gripper_set_span_topic, "set_gripper_span");
    nh.param<std::string>("gripper_state_topic", gripper_state_topic, "edo_gripper_state");

    group = new moveit::planning_interface::MoveGroupInterface(
            moveit::planning_interface::MoveGroupInterface::Options(movegroup_name, robotDescriptionParam, nh));

    group->setPlanningTime(0.5);
    group->setPlannerId("RRTConnectkConfigDefault");
    group->setEndEffectorLink(ee_link);

    myplan = moveit::planning_interface::MoveGroupInterface::Plan();

    lastGripperSpan = -1;
    isGripperMoving = false;
    waitForGripperUpdate = false;

    execute_action_goal_sub = nh.subscribe(move_group::EXECUTE_ACTION_NAME + "/goal", 1, &EdoControlSim::onExecuteActionGoal, this);
    execute_action_result_sub = nh.subscribe(move_group::EXECUTE_ACTION_NAME + "/result", 1, &EdoControlSim::onExecuteActionResult, this);

    gripper_sub = nh.subscribe(gripper_get_span_topic, 1, &EdoControlSim::getGripperSpanCallback, this);
    gripper_state_sub = nh.subscribe(gripper_state_topic, 1, &EdoControlSim::getGripperStateCallback, this);
    gripper_pub = nh.advertise<std_msgs::Float32>(gripper_set_span_topic,1, true);
}

EdoControlSim::~EdoControlSim() {
    delete group;
}


std::string getMoveItErrorStr(moveit::planning_interface::MoveItErrorCode errorCode) {
    switch (errorCode.val) {
        case moveit::planning_interface::MoveItErrorCode::SUCCESS:
            return "SUCCESS";
        case moveit::planning_interface::MoveItErrorCode::FAILURE:
            return "FAILURE";
        case moveit::planning_interface::MoveItErrorCode::PLANNING_FAILED:
            return "PLANNING_FAILED";
        case moveit::planning_interface::MoveItErrorCode::INVALID_MOTION_PLAN:
            return "INVALID_MOTION_PLAN";
        case moveit::planning_interface::MoveItErrorCode::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
            return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
        case moveit::planning_interface::MoveItErrorCode::CONTROL_FAILED:
            return "CONTROL_FAILED";
        case moveit::planning_interface::MoveItErrorCode::UNABLE_TO_AQUIRE_SENSOR_DATA:
            return "UNABLE_TO_AQUIRE_SENSOR_DATA";
        case moveit::planning_interface::MoveItErrorCode::TIMED_OUT:
            return "TIMED_OUT";
        case moveit::planning_interface::MoveItErrorCode::PREEMPTED:
            return "PREEMPTED";
        case moveit::planning_interface::MoveItErrorCode::START_STATE_IN_COLLISION:
            return "START_STATE_IN_COLLISION";
        case moveit::planning_interface::MoveItErrorCode::START_STATE_VIOLATES_PATH_CONSTRAINTS:
            return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
        case moveit::planning_interface::MoveItErrorCode::GOAL_IN_COLLISION:
            return "GOAL_IN_COLLISION";
        case moveit::planning_interface::MoveItErrorCode::GOAL_VIOLATES_PATH_CONSTRAINTS:
            return "GOAL_VIOLATES_PATH_CONSTRAINTS";
        case moveit::planning_interface::MoveItErrorCode::GOAL_CONSTRAINTS_VIOLATED:
            return "GOAL_CONSTRAINTS_VIOLATED";
        case moveit::planning_interface::MoveItErrorCode::INVALID_GROUP_NAME:
            return "INVALID_GROUP_NAME";
        case moveit::planning_interface::MoveItErrorCode::INVALID_GOAL_CONSTRAINTS:
            return "INVALID_GOAL_CONSTRAINTS";
        case moveit::planning_interface::MoveItErrorCode::INVALID_ROBOT_STATE:
            return "INVALID_ROBOT_STATE";
        case moveit::planning_interface::MoveItErrorCode::INVALID_LINK_NAME:
            return "INVALID_LINK_NAME";
        case moveit::planning_interface::MoveItErrorCode::INVALID_OBJECT_NAME:
            return "INVALID_OBJECT_NAME";
        case moveit::planning_interface::MoveItErrorCode::FRAME_TRANSFORM_FAILURE:
            return "FRAME_TRANSFORM_FAILURE";
        case moveit::planning_interface::MoveItErrorCode::COLLISION_CHECKING_UNAVAILABLE:
            return "COLLISION_CHECKING_UNAVAILABLE";
        case moveit::planning_interface::MoveItErrorCode::ROBOT_STATE_STALE:
            return "ROBOT_STATE_STALE";
        case moveit::planning_interface::MoveItErrorCode::SENSOR_INFO_STALE:
            return "SENSOR_INFO_STALE";
        case moveit::planning_interface::MoveItErrorCode::NO_IK_SOLUTION:
            return "NO_IK_SOLUTION";
        default:
            return "UNKNOWN";
    }
}

bool EdoControlSim::moveJoint(double j1, double j2, double j3, double j4, double j5, double j6, bool blocking) {

    ROS_INFO_NAMED("EdoControlSim", "moveJoint:\n\tRAD: %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f\n\tDEG: %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\n\tBlocking: %d",
            j1, j2, j3, j4, j5, j6, j1*RAD_TO_DEG, j2*RAD_TO_DEG, j3*RAD_TO_DEG, j4*RAD_TO_DEG, j5*RAD_TO_DEG, j6*RAD_TO_DEG, blocking);

    group->setStartStateToCurrentState();

    std::vector<double> jointRadiants;
    jointRadiants.push_back(j1);
    jointRadiants.push_back(j2);
    jointRadiants.push_back(j3);
    jointRadiants.push_back(j4);
    jointRadiants.push_back(j5);
    jointRadiants.push_back(j6);

    //group->setGoalTolerance(0.2);
    group->setJointValueTarget(jointRadiants);

    moveit::planning_interface::MoveItErrorCode planErrorCode = group->plan(myplan);

    if (planErrorCode != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR_STREAM_NAMED("EdoControlSim", "Failed to plan Path. " << getMoveItErrorStr(planErrorCode));
        return false;
    }

    moveit::planning_interface::MoveItErrorCode execErrorCode;
    goalExecuted = false;
    execErrorCode = group->asyncExecute(myplan);


    if (execErrorCode != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR_STREAM_NAMED("EdoControlSim", "Failed to execute Path. " << getMoveItErrorStr(execErrorCode));
        return false;
    } else if (blocking) {
        waitRobotStationary();
    }

    return true;
}

bool EdoControlSim::moveCartesian(geometry_msgs::Pose newPose, bool blocking) {

    tf::Quaternion rot;
    tf::quaternionMsgToTF(newPose.orientation, rot);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(rot).getRPY(roll, pitch, yaw);

    ROS_INFO_NAMED("EdoControlSim", "moveCartesian:\n\tPosition (m): %2.3f %2.3f %2.3f\n\tRotation (RAD): %2.3f %2.3f %2.3f\n\tRotation (DEG): %4.2f %4.2f %4.2f\n\tBlocking: %d",
            newPose.position.x, newPose.position.y, newPose.position.z,
            roll, pitch, yaw, roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG , blocking);

    geometry_msgs::PoseStamped command_cartesian_position;

    // swap X and Y
    command_cartesian_position.pose.position.x = newPose.position.x;
    command_cartesian_position.pose.position.y = newPose.position.y;
    command_cartesian_position.pose.position.z = newPose.position.z;
    command_cartesian_position.pose.orientation = newPose.orientation;
    //command_cartesian_position.header.frame_id = "edo_base_link";

    // goal tolerance sphere radius
    //group->setGoalTolerance(0.01);
    group->setStartStateToCurrentState();
    group->setPoseTarget(command_cartesian_position, ee_link);
    group->setPoseReferenceFrame(transformListener.resolve("edo_base_link"));
    group->startStateMonitor();

    robot_state::RobotStatePtr state = group->getCurrentState();

    // set waypoints for which to compute path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(command_cartesian_position.pose);

    // compute cartesian path
    moveit::planning_interface::MoveItErrorCode planErrorCode;
    double ret = group->computeCartesianPath(waypoints, 0.01, 0, myplan.trajectory_, false, &planErrorCode);
    if(ret < 0){
        // no path could be computed
        ROS_ERROR_STREAM_NAMED("EdoControlSim", "Failed to plan Path. " << getMoveItErrorStr(planErrorCode));
        return false;
    } else if (ret < 1){
        // path started to be computed, but did not finish
        ROS_ERROR_STREAM_NAMED("EdoControlSim", "Failed to plan Path. Cartesian path computation finished" << ret * 100 << "% only!" << getMoveItErrorStr(planErrorCode));
        return false;
    }

    moveit::planning_interface::MoveItErrorCode execErrorCode;

    goalExecuted = false;
    execErrorCode = group->asyncExecute(myplan);

    if (execErrorCode != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_ERROR_STREAM_NAMED("EdoControlSim", "Failed to execute Path. " << getMoveItErrorStr(execErrorCode));
        return false;
    } else if (blocking) {
        waitRobotStationary();
    }

    return true;
}

bool EdoControlSim::moveGripper(float span, bool blocking) {

    if (abs(span-lastGripperSpan) < 0.0005) {
        return true;
    }
    lastGripperSpan = span;

    this->waitForGripperUpdate = true;
    std_msgs::Float32 msg;
    msg.data = span;
    ROS_INFO_NAMED("EdoControlSim", "Set gripper span: %f (%s)", span, gripper_pub.getTopic().c_str());
    this->gripper_pub.publish(msg);

    if (blocking) {
        return this->waitGripperStationary(true);
    }

    return true;
}

bool EdoControlSim::setSpeed(double speed) {
    group->setMaxVelocityScalingFactor(speed);
    return true;
}

bool EdoControlSim::waitRobotStationary() {

    if (goalExecuted)
        return true;

    ROS_INFO_STREAM_NAMED("EdoControlSim", "Waiting for execution result");

    // wait a bit to get update from robot
    usleep(100000);

    while (!goalExecuted && ros::ok()) {
        usleep(1000);
    }


    if (this->goalResult == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO_STREAM_NAMED("EdoControlSim", "Executed Path. " << getMoveItErrorStr(goalResult));
        return true;
    } else {
        ROS_ERROR_STREAM_NAMED("EdoControlSim", "Failed to execute path. " << getMoveItErrorStr(goalResult));
        return false;
    }
}

bool EdoControlSim::waitGripperStationary(bool waitForStateChange = false) {

    // wait a bit to get update from gripper
    ROS_INFO_STREAM_NAMED("EdoControlSim", "Waiting for gripper to move");
    if (waitForStateChange) {
        while ((waitForGripperUpdate || !isGripperMoving) && ros::ok()) {
            usleep(1000);
        }
    }

    ROS_INFO_STREAM_NAMED("EdoControlSim", "Waiting for gripper idle");
    while((waitForGripperUpdate || isGripperMoving) && ros::ok()) {
        usleep(1000);
    }
    ROS_INFO_STREAM_NAMED("EdoControlSim", "Griper is done moving");
    return true;
}


void EdoControlSim::getGripperSpanCallback(const std_msgs::Float32ConstPtr &msg) {

}

void EdoControlSim::getGripperStateCallback(const std_msgs::Int8ConstPtr &msg) {
    isGripperMoving = (msg->data == 1);
    waitForGripperUpdate = false;
}

void EdoControlSim::onExecuteActionResult(const moveit_msgs::ExecuteTrajectoryActionResultConstPtr &msg) {
    if (msg->status.goal_id.id != currentGoal)
        return;

    goalResult = msg->result.error_code;
    previousGoal = currentGoal;
    goalExecuted = true;

}

void EdoControlSim::onExecuteActionGoal(const moveit_msgs::ExecuteTrajectoryActionGoalConstPtr &msg) {
    if (currentGoal != msg->goal_id.id && previousGoal != msg->goal_id.id) {
        currentGoal = msg->goal_id.id;
        ROS_INFO_STREAM_NAMED("EdoControlSim", "Tracking new goal: " << currentGoal);
    }
}
