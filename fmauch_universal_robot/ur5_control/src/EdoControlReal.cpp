//
// Created by profanter on 8/13/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#include <math.h>

#include <ros/ros.h>

#include "tf/transform_datatypes.h"
#include <tf/tf.h>
#include <EdoControlReal.h>


#include "edo_core_msgs/MovementCommand.h"
#include "edo_core_msgs/JointStateArray.h"

#include "EdoControlReal.h"
#include "EdoCodesReal.h"

#define METER_TO_MILIMETER 1000
// unit meter
#define ROBOT_BOX_HEIGHT 0.135

//TODO add precedure/thread/Timer to kill the program if there is no communication with a robot


EdoControlReal::EdoControlReal(const char* nodeHandleNamespace) : EdoControl(false, nodeHandleNamespace), m_speed(20) {

    move_ack_sub = nh.subscribe("/machine_movement_ack", 10, &EdoControlReal::moveAckCallback, this);
    usb_jnt_state_sub = nh.subscribe("/usb_jnt_state", 10, &EdoControlReal::getUsbJointStateCallback, this);

    move_ctrl_pub = nh.advertise<edo_core_msgs::MovementCommand>("/bridge_move", 10, true);

    // default values for getUsbJointStateCallback
    lastTimeJointState = -1;
    oneBeforeLastTimeJointState = -2;
    jointStatesAvailable = false;

    goalExecuted = true;
    resetMovementCommandSent = false;
}

EdoControlReal::~EdoControlReal(){
// delete jointStateArray ??
}

// Do not remove these three functions
static
tf::Quaternion quaternionFromAER(double a, double e, double r) {
    // Comau is using ZYZ Euler angles (Z = a, Y = e, Z = r)

    tf::Matrix3x3 matZ1;
    matZ1.setRPY(0,0,a);

    tf::Matrix3x3 matY;
    matY.setRPY(0,e,0);

    tf::Matrix3x3 matZ2;
    matZ2.setRPY(0,0,r);

    tf::Matrix3x3 rot = matZ1 * matY * matZ2;

    tf::Quaternion q;

    rot.getRotation(q);

    return q;
}

static
void twoaxisrot(double r11, double r12, double r21, double r31, double r32, double res[]){
    res[0] = atan2( r11, r12 );
    res[1] = acos ( r21 );
    res[2] = atan2( r31, r32 );
}

static
tf::Vector3 aerFromQuaternion(const tf::Quaternion q) {
    double res[3];
    twoaxisrot( 2*(q.y()*q.z() - q.w()*q.x()),
                2*(q.x()*q.z() + q.w()*q.y()),
                q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(),
                2*(q.y()*q.z() + q.w()*q.x()),
                -2*(q.x()*q.z() - q.w()*q.y()),
                res);

    return tf::Vector3(res[0], res[1], res[2]);
}

edo_core_msgs::MovementCommand createMove(int type, int delay, int speed){
    // delay: 255-flyby, 0-254 - seconds
    edo_core_msgs::MovementCommand msg;
    // Joint movement to joint point
    if(type == 0){
        msg.move_command = 77;
        msg.move_type = 74;
        msg.ovr = speed;
        msg.delay = delay;
        msg.target.data_type = 74;
        msg.target.joints_mask = 127;
        msg.target.joints_data.resize(7, 0.0);
    }
        // Joint movement to cartesian point
    else if(type == 1){
        msg.move_command = 77;
        msg.move_type = 74;
        msg.ovr = speed;
        msg.delay = delay;
        msg.target.data_type = 88;
        msg.target.joints_mask = 127;
        msg.target.joints_data.resize(7, 0.0);
    }
        // Linear movement to joint point
    else if(type == 10){
        msg.move_command = 77;
        msg.move_type = 76;
        msg.ovr = speed;
        msg.delay = delay;
        msg.target.data_type = 74;
        msg.target.joints_mask = 127;
        msg.target.joints_data.resize(7, 0.0);
    }
        // Linear movement to cartesian point
    else if(type == 11){
        msg.move_command = 77;
        msg.move_type = 76;
        msg.ovr = speed;
        msg.delay = delay;
        msg.target.data_type = 88;
        msg.target.joints_mask = 127;
        msg.target.joints_data.resize(7, 0.0);
    }
        // Reset
    else if(type == -1){
        msg.move_command = 67;
        msg.target.joints_data.clear();
    }
    return msg;
}  // createMove()

void printJointStates(const std::string &strMsg, jointState * array)
{
    // print out joint positions
    std::cout << strMsg << ": "
              << array[0].position << " "
              << array[1].position << " "
              << array[2].position << " "
              << array[3].position << " "
              << array[4].position << " "
              << array[5].position << " "
              << array[6].position << std::endl;
}

void EdoControlReal::getUsbJointStateCallback(const edo_core_msgs::JointStateArray msg)
{
    // joint position data comes in degrees, griper span is in mm
    try
    {
        for (std::size_t i = 0; i < msg.joints.size() && ros::ok(); i++) {
            jointStateArray[i].position = msg.joints[i].position;
            jointStateArray[i].velocity = msg.joints[i].velocity;
            jointStateArray[i].current = msg.joints[i].current;
            jointStateArray[i].commandFlag = msg.joints[i].commandFlag;
        }
        if (not ros::ok())
        {
            // we should not used joint data anymore if ros::ok() fails
            lastTimeJointState = -1;
            oneBeforeLastTimeJointState = -2;
            jointStatesAvailable = false;
        }

        oneBeforeLastTimeJointState = lastTimeJointState;
        lastTimeJointState = ros::Time::now().toSec();
        //ROS_INFO("%.5f", fabs(oneBeforeLastTimeJointState -  lastTimeJointState));
        if( fabs(oneBeforeLastTimeJointState -  lastTimeJointState) < 0.015) {
            jointStatesAvailable = true;
            //printJointStates("true getUsbJointStateCallback", jointStateArray);

        }else {
            jointStatesAvailable = false;
            //printJointStates("false getUsbJointStateCallback", jointStateArray);
        }
    }
    catch (int e)
    {
        ROS_INFO_STREAM_NAMED("EdoControlReal", "No value from a topic /usb_jnt_state");
    }
}

// TODO check state of robot - callback

void EdoControlReal::moveAckCallback(const edo_core_msgs::MovementFeedback msg)
{
    //std::cout << "MoveAckCallback: " << std::to_string(msg.type).c_str() << ", " <<
        //std::to_string(msg.data).c_str() << std::endl;
    switch(msg.type) {
        case 0:
            // TODO This is happenih multiple time for one command, how to resolve this?
            //ROS_INFO_STREAM_NAMED("EdoControlReal", "FB: Command was received");
            break;
        case 1:
            // TODO Ignore this for now...
            //ROS_INFO_STREAM_NAMED("EdoControlReal", "FB: Send next comamnd if available");
            //goalExecuted = true;
            //goalExecutedSuccess = -1;
            break;
        case 2:
            ROS_INFO_STREAM_NAMED("EdoControlReal", "FB: Command executed and send next one if available");
            goalExecuted = true;
            goalExecutedSuccess = 1;
            break;
        case -1:
            ROS_INFO_STREAM_NAMED("EdoControlReal", "FB: Error: " << EdoCodesReal::getErrorMessages(msg.data));
            goalExecuted = true;
            goalExecutedSuccess = 0;
            break;
        default:
            ROS_INFO_STREAM_NAMED("EdoControlReal", "Feedback from a robot is not specified!!!: Type: " << std::to_string(msg.type).c_str());
            ROS_INFO_STREAM_NAMED("EdoControlReal", "Data: " << EdoCodesReal::getErrorMessages(msg.data));
    }
}

void EdoControlReal::sendMovementCommand(edo_core_msgs::MovementCommand cmd)
{
    goalExecuted = false;
    goalExecutedSuccess = -2;
    move_ctrl_pub.publish(cmd);
    if(resetMovementCommandSent)
        ROS_INFO_STREAM_NAMED("EdoControlReal", "Movement Command message was published");
    else
        ROS_INFO_STREAM_NAMED("EdoControlReal", "Reset Command message was published");
}

void EdoControlReal::sendResetMovementCommand()
{
    if(!resetMovementCommandSent)
    {
        edo_core_msgs::MovementCommand preMsg = createMove(-1, -1, -1);
        waitRobotStationary();
        sendMovementCommand(preMsg);
        resetMovementCommandSent = true;
        usleep(10000);
    }
}

bool EdoControlReal::moveJoint(double j1, double j2, double j3, double j4, double j5, double j6, bool blocking) {
    // joints comes in radians

    ROS_INFO_NAMED("EdoControlReal", "moveJoint:\n\tRAD: %2.3f %2.3f %2.3f %2.3f %2.3f %2.3f\n\tDEG: %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f\n\tBlocking: %d",
                   j1, j2, j3, j4, j5, j6, j1*RAD_TO_DEG, j2*RAD_TO_DEG, j3*RAD_TO_DEG, j4*RAD_TO_DEG, j5*RAD_TO_DEG, j6*RAD_TO_DEG, blocking);

    // do not execute movement if there is long delay from joint's states or none at all
    while( !jointStatesAvailable && ros::ok() )
    {
        ROS_WARN_ONCE("Waiting for first joint state message ...");
        ros::spinOnce();
    }
    if (! ros::ok())
    {
        ROS_INFO_NAMED("EdoControlReal", "ROS exited prematurely, joint movement did not finished.");
        return false;
    }

    sendResetMovementCommand();

    edo_core_msgs::MovementCommand msg = createMove(0, 255, this->m_speed);
    // msg is accepting joint data in degress
    msg.target.joints_data[0] = j1 * RAD_TO_DEG;
    msg.target.joints_data[1] = j2 * RAD_TO_DEG;
    msg.target.joints_data[2] = j3 * RAD_TO_DEG;
    msg.target.joints_data[3] = j4 * RAD_TO_DEG;
    msg.target.joints_data[4] = j5 * RAD_TO_DEG;
    msg.target.joints_data[5] = j6 * RAD_TO_DEG;
    // joint 7 is in milimeters
    msg.target.joints_data[6] = jointStateArray[6].position;


    if(!waitRobotStationary())
        return false;

    sendMovementCommand(msg);

    if (blocking) {
        if(!waitRobotStationary())
            return false;
    }

    return true;
}

bool EdoControlReal::moveCartesian(geometry_msgs::Pose newPose, bool blocking) {
    // position data in meters and, orientation in quaternion

    tf::Quaternion rot;
    tf::quaternionMsgToTF(newPose.orientation, rot);
    double roll = 0, pitch = 0, yaw = 0;
    tf::Matrix3x3(rot).getRPY(roll, pitch, yaw);
    ROS_INFO_NAMED("EdoControlReal", "moveCartesian:\n\tPosition (m): %2.3f %2.3f %2.3f\n\tRotation (RAD): %2.3f %2.3f %2.3f\n\tRotation (DEG): %4.2f %4.2f %4.2f\n\tBlocking: %d",
                   newPose.position.x, newPose.position.y, newPose.position.z,
                   roll, pitch, yaw, roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG , blocking);

    // do not execute movement if there is long delay from joint's states or none at all
    while( !jointStatesAvailable && ros::ok())
    {
        ROS_WARN_ONCE("Waiting for first joint state message ...");
        ros::spinOnce();
    }
    if (! ros::ok())
    {
        ROS_INFO_NAMED("EdoControlReal", "ROS exited prematurely, cartesian movement did not finished.");
        return false;
    }
    sendResetMovementCommand();

    edo_core_msgs::MovementCommand msg = createMove(11, 255, this->m_speed);
    msg.target.cartesian_data.x = newPose.position.x * METER_TO_MILIMETER;
    msg.target.cartesian_data.y = newPose.position.y * METER_TO_MILIMETER;
    msg.target.cartesian_data.z = newPose.position.z * METER_TO_MILIMETER;

    tf::Vector3 zyz = aerFromQuaternion(rot);

    msg.target.cartesian_data.a = zyz[0] * RAD_TO_DEG;
    msg.target.cartesian_data.e = zyz[1] * RAD_TO_DEG;
    msg.target.cartesian_data.r = zyz[2] * RAD_TO_DEG;

    // joint position for gripper also needs to be set
    msg.target.joints_data[6] = jointStateArray[6].position;

    if(!waitRobotStationary())
        return false;

    std::cout << msg.target.cartesian_data;
    sendMovementCommand(msg);

    if (blocking) {
        if(!waitRobotStationary())
            return false;
    }

    return true;

}

bool EdoControlReal::moveGripper(float span, bool blocking) {
    // do not execute movement if there is long delay from joint's states
    ROS_INFO("Move Gripper: %f, %d", span, blocking);
    while( !jointStatesAvailable && ros::ok() )
    {
        ROS_WARN_ONCE("Waiting for first joint state message ...");
        ros::spinOnce();
    }
    if (! ros::ok())
    {
        ROS_INFO_NAMED("EdoControlReal", "ROS exited prematurely, gripper movement did not finished.");
        return false;
    }
    sendResetMovementCommand();

    edo_core_msgs::MovementCommand msg = createMove(0, 255, this->m_speed);
    // msg is accepting joint data in degress
    msg.target.joints_data[0] = jointStateArray[0].position;
    msg.target.joints_data[1] = jointStateArray[1].position;
    msg.target.joints_data[2] = jointStateArray[2].position;
    msg.target.joints_data[3] = jointStateArray[3].position;
    msg.target.joints_data[4] = jointStateArray[4].position;
    msg.target.joints_data[5] = jointStateArray[5].position;
    // joint 7 is in milimeters, span is in meters
    msg.target.joints_data[6] = span * METER_TO_MILIMETER;

    //printJointStates("Joint values in moveGripper()", jointStateArray);

    if(!waitGripperStationary(true))
        return false;

    sendMovementCommand(msg);

    if (blocking) {
        if(!waitGripperStationary(true))
            return false;
    }

    return true;
}

bool EdoControlReal::setSpeed(double speed) {
    if (speed >= 0 && speed <= 1)
    {
        m_speed = round(speed*100);
        if (m_speed < 0 || m_speed > 100)
        {
            // This should never happen
            m_speed = 10;
            ROS_INFO_NAMED("EdoControlReal", "Wrong value for speed parameter after casting. Speed parameter is set to: %.2f", m_speed/100.0);
        }
    }
    else
    {
        m_speed = 10;
        ROS_INFO_NAMED("EdoControlReal", "Value for speed parameter is outside of [0.1 ... 1.0]. Speed parameter is set to: %.2f", m_speed/100.0);
    }
    return true;
}

bool EdoControlReal::waitRobotStationary() {

    if (goalExecuted)
        return true;

    ROS_INFO_STREAM_NAMED("EdoControlReal", "Waiting for execution result...");

    // wait a bit to get update from robot
    usleep(100000);

    while (!goalExecuted && goalExecutedSuccess < 0 && ros::ok()) {
        usleep(1000);
        ros::spinOnce();
        if (-2 != goalExecutedSuccess)
            std::cout << "goalExecuted: " << goalExecuted << ", goalExecutedSuccess: " << std::to_string(goalExecutedSuccess).c_str() << std::endl;
        //TODO add ros warning, print once
    }

    if (! ros::ok())
    {
        ROS_INFO_NAMED("EdoControlReal", "ROS exited prematurely, unable to wait robot to stationary.");
        return false;
    }

    // TODO remove this sleep with checking if joint are not moving anymore/moving inside of some epsilon value
    usleep(300000);

    //if(goalExecutedSuccess == -1){ // TODO this value is ignored for now
        //return true;
    //}else
    if(goalExecutedSuccess == 1){
        ROS_INFO_STREAM_NAMED("EdoControlReal", "Executed Path successfuly.");
        return true;
    }else if(goalExecutedSuccess == 0) {
        ROS_INFO_STREAM_NAMED("EdoControlReal", "Failed to execute path.");
        return false;
    }
}

bool EdoControlReal::waitGripperStationary(bool waitForStateChange = false) {
    // TODO how to use waitForStateChange ??
    // check few last topics if gripper is still moving
    return waitRobotStationary();
}

bool EdoControlReal::sendEmptyMoveCommand(const uint8_t moveCommand) {
    edo_core_msgs::MovementCommand msg;
    msg.move_command = moveCommand;

    sendMovementCommand(msg);
    return waitRobotStationary();
}
