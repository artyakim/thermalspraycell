//
// Created by profanter on 8/13/18.
// Copyright (c) 2018 fortiss GmbH. All rights reserved.
//

#ifndef PROJECT_EDOCONTROL_H
#define PROJECT_EDOCONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>

#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD (M_PI/180.0)

class EdoControl {

public:

    /**
     * Move robot to given joint values. The values should be given in radiants.
     * You can use the DEG_TO_RAD define to convert degress to rad.
     * @param j1 radiants for joint 1
     * @param j2 radiants for joint 2
     * @param j3 radiants for joint
     * @param j4 radiants for joint
     * @param j5 radiants for joint
     * @param j6 radiants for joint
     * @param blocking if set to true, the method is blocking and it waits until the movement finished.
     * @return true on success, false if the robot did not move to the position.
     */
    virtual bool moveJoint(double j1, double j2, double j3, double j4, double j5, double j6, bool blocking) = 0;

    /**
     * Move the robot to the given cartesian position.
     * The position is relative to the robot base (i.e. edo_base_link).
     * @param newPose The new cartesian pose relative to edo_base_link. It has to be in meters.
     * @param blocking if set to true, the method is blocking and it waits until the movement finished.
     * @return true on success, false if the robot did not move to the position.
     */
    virtual bool moveCartesian(geometry_msgs::Pose newPose, bool blocking) = 0;

    /**
     * Move the gripper to the given gripper span.
     *
     * @param span the new span for the gripper in meters. For the default e.DO gripper, the span is between [0, 0.08]
     * * @param blocking if set to true, the method is blocking and it waits until the gripper reached its position
     * @return true on success, false if the gripper could not be moved
     */
    virtual bool moveGripper(float span, bool blocking) = 0;

    /**
     * Set the robot speed in percentage. The value must be between 0 and 1.
     *
     * @param speed in percentage between 0 and 1
     * @return true if the speed was changed successfully.
     */
    virtual bool setSpeed(double speed) = 0;

    /**
     * Waits until the robot is stationary, i.e., it finished the current movement.
     * If the robot is already stationary, this method returns immediately.
     *
     * @return true if the robot is stationary, false if there was an error while waiting.
     */
    virtual bool waitRobotStationary() = 0;

    /**
     * Waits until the gripper is stationary, i.e., it finished the current movement.
     * If the gripper is already stationary, this method returns immediately.
     *
     * @param waitForStateChange if set to true, this method will first wait until the gripper
     * is changing to moving, and then wait until the gripper is stationary again.
     * Most of the time this parameter should be set to true.
     * @return true if the robot is stationary, false if there was an error while waiting.
     */
    virtual bool waitGripperStationary(bool waitForStateChange) = 0;

    /**
     * Returns the Z offset of the gripper pick position to the robot flange.
     * This value may slightly differ between simulation and real robot.
     *
     * @return the z offset in meters
     */
    virtual double getGripperZOffset() = 0;

    virtual ~EdoControl();
protected:
    explicit EdoControl(bool _isSimulation, const char* nodeHandleNamespace);
    const bool isSimulation;

    ros::NodeHandle nh;

    tf::TransformListener transformListener;

private:
    static bool initialized;

};


#endif //PROJECT_EDOCONTROL_H
