//
// Created by profanter on 8/13/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_EDOCONTROLSIM_H
#define PROJECT_EDOCONTROLSIM_H

#include "EdoControl.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>

#include <actionlib/client/simple_action_client.h>

/**
 * Interface for controlling the simulated robot in gazebo.
 *
 * Additional documentation for the control methods can be found in the base class.
 */
class EdoControlSim : public EdoControl {


public:
    /**
     * Creates an instance of the robot control interface for the simulated robot..
     * @param nodeHandleNamespace This is the node handle namespace. It should typically be set to '/edo'
     * @param robotDescriptionParam The name of the ros parameter where the robot description is stored. Typically '/edo/robot_description'
     */
    explicit EdoControlSim(const char* nodeHandleNamespace, const char* robotDescriptionParam);

    ~EdoControlSim() override;

    bool moveJoint(double j1, double j2, double j3, double j4, double j5, double j6, bool blocking) override;
    bool moveCartesian(geometry_msgs::Pose newPose, bool blocking) override;
    bool moveGripper(float span, bool blocking) override;
    bool setSpeed(double speed) override;

    bool waitRobotStationary() override;
    bool waitGripperStationary(bool waitForStateChange) override;
    double getGripperZOffset() override {
        return 0.12;
    }

private:
    std::string movegroup_name, ee_link, gripper_get_span_topic, gripper_set_span_topic, gripper_state_topic;

    moveit::planning_interface::MoveGroupInterface *group;
    moveit::planning_interface::MoveGroupInterface::Plan myplan;

    ros::Subscriber execute_action_goal_sub;
    ros::Subscriber execute_action_result_sub;
    std::string currentGoal;
    std::string previousGoal;
    bool goalExecuted;
    moveit::planning_interface::MoveItErrorCode goalResult;

    bool isGripperMoving;
    double lastGripperSpan;
    bool waitForGripperUpdate;
    ros::Publisher gripper_pub;
    ros::Subscriber gripper_sub;
    ros::Subscriber gripper_state_sub;
    void getGripperSpanCallback(const std_msgs::Float32ConstPtr &msg);
    void getGripperStateCallback(const std_msgs::Int8ConstPtr &msg);

    void onExecuteActionGoal(const moveit_msgs::ExecuteTrajectoryActionGoalConstPtr &msg);
    void onExecuteActionResult(const moveit_msgs::ExecuteTrajectoryActionResultConstPtr &msg);
};


#endif //PROJECT_EDOCONTROLSIM_H
