//
// Created by profanter on 8/13/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_EDOCONTROLREAL_H
#define PROJECT_EDOCONTROLREAL_H

#include "EdoControl.h"

#include "edo_core_msgs/JointStateArray.h"
#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/MovementFeedback.h"

struct jointState{
    float position;
    float velocity;
    float current;
    uint8_t commandFlag;
};

/**
 * Interface for controlling the real robot.
 *
 * Additional documentation for the control methods can be found in the base class.
 */
class EdoControlReal: public EdoControl {

public:
    /**
     * Creates an instance of the real robot control interface.
     * @param nodeHandleNamespace This is the node handle namespace. It should typically be set to '/edo'
     */
    explicit EdoControlReal(const char* nodeHandleNamespace);

    ~EdoControlReal() override;

    bool moveJoint(double j1, double j2, double j3, double j4, double j5, double j6, bool blocking) override;
    bool pauseJointMovement() { return sendEmptyMoveCommand(80); };
    bool resumeJointMovement() { return sendEmptyMoveCommand(82); };
    bool haltJointMovement() { return sendEmptyMoveCommand(67); };

    bool moveCartesian(geometry_msgs::Pose newPose, bool blocking) override;
    bool moveGripper(float span, bool blocking) override;
    bool setSpeed(double speed) override;

    bool waitRobotStationary() override;
    bool waitGripperStationary(bool waitForStateChange) override;
    double getGripperZOffset() override {
        return 0.13;
    }

    void sendMovementCommand(edo_core_msgs::MovementCommand cmd);
    void sendResetMovementCommand();
    void getUsbJointStateCallback(const edo_core_msgs::JointStateArray msg);
    void moveAckCallback(const edo_core_msgs::MovementFeedback msg);

private:
    bool sendEmptyMoveCommand(uint8_t moveCommand);

    uint m_speed;
    ros::Subscriber usb_jnt_state_sub;
    ros::Subscriber move_ack_sub;
    ros::Publisher move_ctrl_pub;
    uint8_t number_of_joints;
    jointState jointStateArray[7];
    bool jointStatesAvailable;
    double lastTimeJointState, oneBeforeLastTimeJointState;
    bool goalExecuted;
    int8_t goalExecutedSuccess;

    bool resetMovementCommandSent;
};



#endif //PROJECT_EDOCONTROLREAL_H
