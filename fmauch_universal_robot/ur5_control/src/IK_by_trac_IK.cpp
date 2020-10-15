//
// Created by artemyakimchuk on 20/11/2019.
//

#include "ros/ros.h"
#include "EdoControlSim.h"
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <math.h>
#include "sensor_msgs/JointState.h"
#include <tf_conversions/tf_eigen.h>
#include <trac_ik/trac_ik.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <pilz_control/pilz_joint_trajectory_controller.h>
#include <pilz_msgs/GetMotionSequenceRequest.h>
#define DEG_TO_RAD (M_PI/180.0)

#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <math.h>


std::vector<geometry_msgs::Pose> createWaypoints (geometry_msgs::Pose primitive, double dim_a, double dim_b)
{
    tf::Transform rot_tcp;
    double radius = 0.005;
    double offset = 0.15;
    double delta = 0.001;
    double n = ceil(dim_b/(2*((2*radius)-delta)));
    double r,p,y;
    tf::poseMsgToTF(primitive, rot_tcp);
    rot_tcp.getBasis().getRPY(p,r,y); // roll and pitch are different cause base_link do not match world
    //y-= M_PI/2 ;
    //y-= 0 ;
    //p +=M_PI/2;
    r +=M_PI/2;

    std::vector<geometry_msgs::Pose> waypoints;
    tf::Transform tf_of_object, tf_current;
    geometry_msgs::Pose point;
    tf::poseMsgToTF(primitive, tf_of_object);

    tf_of_object.getBasis().setRPY(r,p,y);

    tf_current.getOrigin().setX(- dim_b/2-(2*radius));
    tf_current.getOrigin().setY(+ dim_a/2+(radius));
    tf_current.getOrigin().setZ( -offset);
    tf_current.getBasis().setRPY(0,0,0);

    tf_of_object *= tf_current;

    tf::poseTFToMsg(tf_of_object, point);
    waypoints.push_back(point);

    for(int i =0; i<n; ++i)
    {
        //tf_of_object.getOrigin().setZ((tf_of_object.getOrigin().z() - dim_a - 2*(2*radius))) ;

        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setX(-(- dim_b - 2*(2*radius)));
        tf_of_object *= tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

        //tf_of_object.getOrigin().setX((tf_of_object.getOrigin().x() + (2*radius)-delta)) ;
        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setY(-( (2*radius)-delta));
        tf_of_object *=tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

        //tf_of_object.getOrigin().setZ((tf_of_object.getOrigin().z() + dim_a + 2*(2*radius))) ;
        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setX(-( + dim_b + 2*(2*radius)));
        tf_of_object *=tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

        //tf_of_object.getOrigin().setX((tf_of_object.getOrigin().x() + (2*radius)-delta)) ;
        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setY(-( + (2*radius)-delta));
        tf_of_object *=tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

    }
    return waypoints;

}
void addPrimitiveBox (moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                      moveit::planning_interface::MoveGroupInterface &group,
                      std::string box_id,
                      tf::Vector3 box_size,
                      tf::Transform box_tf
                      )

{
    //Add primitive object for process
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();
    collision_object.id = box_id;
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = 0.3;
//    primitive.dimensions[1] = 0.15;
//    primitive.dimensions[2] = 0.3;
    primitive.dimensions[0] = box_size.getX();
    primitive.dimensions[1] = box_size.getY();
    primitive.dimensions[2] = box_size.getZ();


    //pose of the box relative to the frame id
//    tf::Transform box_rot;
//    box_rot.getBasis().setRPY(1.57,0.0,0.0);
    geometry_msgs::Pose box_pose;
    tf::poseTFToMsg(box_tf, box_pose);
//    box_pose.position.x = 0.4;
//    box_pose.position.y = 0.0;
//    box_pose.position.z = 0.075;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    //Add collision object to the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

}

std::vector<geometry_msgs::Pose> Interpolate (geometry_msgs::Pose n, geometry_msgs::Pose m )
{
	//Define the biggest shortcut of xyz.
	std::vector<geometry_msgs::Pose> InterWayPoints;
	InterWayPoints.clear();
	//step size is 1mm;
	float delta = 0.01;
	float d_x = m.position.x - n.position.x;
	float d_y = m.position.y - n.position.y;
	float d_z = m.position.z - n.position.z;
	float d_max = std::max({abs(d_x), abs(d_y), abs(d_z)});
	int steps = d_max/delta; //define number of steps in the biggest shortcut
	geometry_msgs::Pose current_pose = n;

	for (int i =0; i<steps; ++i)
	{
		current_pose.position.x += d_x/steps;
		current_pose.position.y += d_y/steps;
		current_pose.position.z += d_z/steps;
		InterWayPoints.push_back(current_pose);
	}
	return InterWayPoints;
}

std::vector<geometry_msgs::Pose> InterWayPoints (std::vector<geometry_msgs::Pose> waypoints)
{
	std::vector<geometry_msgs::Pose> inter_waypoints, new_points;
	inter_waypoints.clear();
	for (int n = 0; n<waypoints.size()-1; ++n)
	{
		//Here we attach interpolated trajectory to existing position vector by insert function
		new_points = Interpolate(waypoints[n], waypoints[n+1]);
		inter_waypoints.insert(std::end(inter_waypoints), std::begin(new_points), std::end(new_points));
		new_points.clear();
	}
	return inter_waypoints;
}


int main(int argc, char **argv) {
    
    //moveit::planning_interface::MoveGroupInterface *group;


    ros::init(argc, argv, "ur5_control");
    ros::NodeHandle n("~");
    if (n.ok()) {

        ROS_INFO("hey");
    }

	tf::StampedTransform object_transform;
	tf::TransformListener listener_for_tf;
	tf::Vector3 vec(0,0,0);
	object_transform.setOrigin(vec);
	if(listener_for_tf.waitForTransform("base_link","object_pose", ros::Time(), ros::Duration(5.0)))
	{
		listener_for_tf.lookupTransform("base_link", "object_pose", ros::Time(0), object_transform);
		ros::spinOnce();
	}
    ros::AsyncSpinner spinner(2);
    spinner.start();


    //Initialize MoveGroup
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPlanningTime(20.5);
    group.setPlannerId("RRTConnectkConfigDefault");
    //group.setPlannerId("PTP");
    group.setEndEffectorLink("tcp");
    const:: robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //init rviz topic for markers
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 0.5;
    visual_tools.publishText(text_pose, "ThermalSpray Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", group.getEndEffectorLink().c_str());
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(group.getJointModelGroupNames().begin(), group.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    visual_tools.prompt("Press NEXT to start ");

    group.setStartStateToCurrentState();
    group.setMaxVelocityScalingFactor(0.9);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    geometry_msgs::Pose current_pose;
//    current_pose.position = group.getCurrentPose().pose.position;
//    current_pose.orientation = group.getCurrentPose().pose.orientation;
/////////////////////////////////////////////////////////////////////////////////////////////

    // Cartesian Paths
    //Create basic parameters for traj
    std::vector<geometry_msgs::Pose> waypoints;
    std::vector<geometry_msgs::Pose> inter_waypoints;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.05;
    robot_trajectory::RobotTrajectory rt(group.getRobotModel(), group.getName());
    trajectory_processing::TimeOptimalTrajectoryGeneration time_param;

//    //Add primitive object for process
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group.getPlanningFrame();
//    collision_object.id = "box1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;

    tf::Vector3 box_size;
    box_size.setX(0.09);
    box_size.setY(0.14);
    box_size.setZ(0.05);
    tf::Transform box_tf;
    ///////////////////////////////////////////////////////////////ADD TF HERE

    box_tf.setRotation(object_transform.getRotation());
    box_tf.setOrigin(object_transform.getOrigin());
    tf::Quaternion q;
    q.setX(0.373491);
    q.setY(0.615548);
    q.setZ(0.603583);
    q.setW(0.342479);
    box_tf.setRotation(q);
    box_tf.getBasis().setRPY(1.57,0.0,0.0); /// commented
    box_tf.getOrigin().setX(0.3737);
    box_tf.getOrigin().setY(-0.396672);
    box_tf.getOrigin().setZ(0.273011);
    addPrimitiveBox (planning_scene_interface,
                          group,
                          "box1",
                          box_size,
                          box_tf);

//    //Add collision object to the world
//    ROS_INFO_NAMED("tutorial", "Add an object into the world");
//    planning_scene_interface.addCollisionObjects(collision_objects);
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

//    tf::Vector3 wall_back;
//    wall_back.setX(0.01);
//    wall_back.setY(2);
//    wall_back.setZ(2);
//    tf::Transform wallback_tf;
//    wallback_tf.getBasis().setRPY(1.57,0.0,0.0);
//    wallback_tf.getOrigin().setX(-0.5);
//    wallback_tf.getOrigin().setY(0.0);
//    wallback_tf.getOrigin().setZ(1);
//    addPrimitiveBox (planning_scene_interface,
//                     group,
//                     "wallback",
//                     wall_back,
//                     wallback_tf);
//    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
//    visual_tools.trigger();



    visual_tools.prompt("Press NEXT to get the ENVIRONMENT");
    //Add floor as a primitive object to avoid collision
    moveit_msgs::CollisionObject collision_object1, wall_one, wall_two;
    collision_object1.header.frame_id = group.getPlanningFrame();
    wall_one.header.frame_id = group.getPlanningFrame();
    wall_two.header.frame_id = group.getPlanningFrame();

    collision_object1.id = "floorPrimitive";
    wall_one.id = "wallonePrimitive";
    wall_two.id = "walltwoPrimitive";

    shape_msgs::SolidPrimitive primitive1, wall1_primitive, wall2_primitive;
    primitive1.type = primitive1.BOX;
    wall1_primitive.type = primitive1.BOX;
    wall2_primitive.type = primitive1.BOX;

    primitive1.dimensions.resize(3);
    primitive1.dimensions[0] = 2;
    primitive1.dimensions[1] = 0.01;
    primitive1.dimensions[2] = 2;

    wall1_primitive.dimensions.resize(3);
    wall1_primitive.dimensions[0] = 2;
    wall1_primitive.dimensions[1] = 2;
    wall1_primitive.dimensions[2] = 0.01;

    wall2_primitive.dimensions.resize(3);
    wall2_primitive.dimensions[0] = 2;
    wall2_primitive.dimensions[1] = 2;
    wall2_primitive.dimensions[2] = 0.01;

    //Param of the floorPrimitive relative to the frame id
    tf::Transform box_rot1, wall1_rot, wall2_rot;
    box_rot1.getBasis().setRPY(M_PI/2,0,0);
    wall1_rot.getBasis().setRPY(M_PI/2,0,0);
    wall2_rot.getBasis().setRPY(M_PI/2,0,0);

    geometry_msgs::Pose box_pose1, wall1_pose, wall2_pose;
    tf::poseTFToMsg(box_rot1, box_pose1);
    tf::poseTFToMsg(wall1_rot, wall1_pose);
    tf::poseTFToMsg(wall2_rot, wall2_pose);
    box_pose1.position.x = 0;
    box_pose1.position.y = 0;
    box_pose1.position.z = -0.01;

    wall1_pose.position.x = 0;
    wall1_pose.position.y = -0.45;
    wall1_pose.position.z = 1;

    wall2_pose.position.x = 0;
    wall2_pose.position.y = 0.45;
    wall2_pose.position.z = 1;


    //Add floor to the RViz
    collision_object1.primitives.push_back(primitive1);
    //collision_object1.primitives.push_back(wall1_primitive);
    //collision_object1.primitives.push_back(wall2_primitive);

    collision_object1.primitive_poses.push_back(box_pose1);
    //collision_object1.primitive_poses.push_back(wall1_pose);
    //collision_object1.primitive_poses.push_back(wall2_pose);

    collision_object1.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects1;
    collision_objects1.push_back(collision_object1);
    //Add collision object to the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects1);
    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.prompt("Press NEXT to plan to HOME position");


    //joint_group_positions[0]= - (90*DEG_TO_RAD + atan2(box_tf.getOrigin().getX(), box_tf.getOrigin().getY()));
    joint_group_positions[0]=5*DEG_TO_RAD;
    joint_group_positions[1]=5*DEG_TO_RAD;
    joint_group_positions[2]=5*DEG_TO_RAD;
    joint_group_positions[3]=5*DEG_TO_RAD;
    joint_group_positions[4]=5*DEG_TO_RAD;
    joint_group_positions[5]=5*DEG_TO_RAD;

//    joint_group_positions[0]= 0;
//    joint_group_positions[1]= -90*DEG_TO_RAD;
//    joint_group_positions[2]= 0*DEG_TO_RAD;
//    joint_group_positions[3]= 0*DEG_TO_RAD;
//    joint_group_positions[4]= 0*DEG_TO_RAD;
//    joint_group_positions[5]= 90*DEG_TO_RAD;

    group.setJointValueTarget(joint_group_positions);
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    visual_tools.prompt("Press NEXT to MOVE to HOME position");
    group.move();

    joint_group_positions[1]=6*DEG_TO_RAD;
    joint_group_positions[2]=6*DEG_TO_RAD;
    joint_group_positions[3]=6*DEG_TO_RAD;
    joint_group_positions[4]=6*DEG_TO_RAD;

    group.setJointValueTarget(joint_group_positions);
     success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    visual_tools.prompt("Press NEXT to MOVE to CLOSER position");
    group.move();
/*

//    //Add floor as a primitive object to avoid collision
//    moveit_msgs::CollisionObject collision_object1, wall_one, wall_two;
//    collision_object1.header.frame_id = group.getPlanningFrame();
//    wall_one.header.frame_id = group.getPlanningFrame();
//    wall_two.header.frame_id = group.getPlanningFrame();
//
//    collision_object1.id = "floorPrimitive";
//    wall_one.id = "wallonePrimitive";
//    wall_two.id = "walltwoPrimitive";
//
//    shape_msgs::SolidPrimitive primitive1, wall1_primitive, wall2_primitive;
//    primitive1.type = primitive1.BOX;
//    wall1_primitive.type = primitive1.BOX;
//    wall2_primitive.type = primitive1.BOX;
//
//    primitive1.dimensions.resize(3);
//    primitive1.dimensions[0] = 2;
//    primitive1.dimensions[1] = 0.01;
//    primitive1.dimensions[2] = 2;
//
//    wall1_primitive.dimensions.resize(3);
//    wall1_primitive.dimensions[0] = 2;
//    wall1_primitive.dimensions[1] = 2;
//    wall1_primitive.dimensions[2] = 0.01;
//
//    wall2_primitive.dimensions.resize(3);
//    wall2_primitive.dimensions[0] = 2;
//    wall2_primitive.dimensions[1] = 2;
//    wall2_primitive.dimensions[2] = 0.01;
//
//    //Param of the floorPrimitive relative to the frame id
//    tf::Transform box_rot1, wall1_rot, wall2_rot;
//    box_rot1.getBasis().setRPY(M_PI/2,0,0);
//    wall1_rot.getBasis().setRPY(M_PI/2,0,0);
//    wall2_rot.getBasis().setRPY(M_PI/2,0,0);
//
//    geometry_msgs::Pose box_pose1, wall1_pose, wall2_pose;
//    tf::poseTFToMsg(box_rot1, box_pose1);
//    tf::poseTFToMsg(wall1_rot, wall1_pose);
//    tf::poseTFToMsg(wall2_rot, wall2_pose);
//    box_pose1.position.x = 0;
//    box_pose1.position.y = 0;
//    box_pose1.position.z = -0.01;
//
//    wall1_pose.position.x = 0;
//    wall1_pose.position.y = -0.8;
//    wall1_pose.position.z = 1;
//
//    wall2_pose.position.x = 0;
//    wall2_pose.position.y = 0.8;
//    wall2_pose.position.z = 1;
//
//
//    //Add floor to the RViz
//    collision_object1.primitives.push_back(primitive1);
//    collision_object1.primitives.push_back(wall1_primitive);
//    collision_object1.primitives.push_back(wall2_primitive);
//
//    collision_object1.primitive_poses.push_back(box_pose1);
//    collision_object1.primitive_poses.push_back(wall1_pose);
//    collision_object1.primitive_poses.push_back(wall2_pose);
//
//    collision_object1.operation = collision_object.ADD;
//    std::vector<moveit_msgs::CollisionObject> collision_objects1;
//    collision_objects1.push_back(collision_object1);
//    //Add collision object to the world
//    ROS_INFO_NAMED("tutorial", "Add an object into the world");
//    planning_scene_interface.addCollisionObjects(collision_objects1);
//    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    visual_tools.trigger();
    visual_tools.prompt("Press to form Spraying TRAJECTORY");

    geometry_msgs::Pose box_pose;
    tf::poseTFToMsg(box_tf, box_pose);

    waypoints.clear();
    inter_waypoints.clear();

    //waypoints = createWaypoints(box_pose, primitive.dimensions[2], primitive.dimensions[0]);
    waypoints = createWaypoints(box_pose, box_size.getY(), box_size.getX());

	inter_waypoints = InterWayPoints(waypoints);

    moveit_msgs::RobotTrajectory trajectory_paint;
    double fraction = group.computeCartesianPath(inter_waypoints, eef_step, jump_threshold, trajectory_paint);

//    Use of TimeOptimalTrajectoryGeneration
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_paint);
    success = time_param.computeTimeStamps(rt, 0.91,0.9);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
    rt.getRobotTrajectoryMsg(trajectory_paint);
    my_plan.trajectory_ = trajectory_paint;


    //Visualize traj in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(inter_waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < inter_waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(inter_waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
    visual_tools.trigger();

    visual_tools.prompt("PRESS NEXT TO EXECUTE TRAJ");
    group.execute(my_plan);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    visual_tools.trigger();
    visual_tools.prompt("Press NEXT to attach object to robot");

    ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
    group.attachObject(collision_object.id);
    visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
                        "robot");
    ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
    group.detachObject(collision_object.id);
    visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

/* Wait for MoveGroup to recieve and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
                        "robot");

    ROS_INFO_NAMED("tutorial", "Remove the object from the world");
    std::vector<std::string> object_ids;
    object_ids.push_back(collision_object.id);
    planning_scene_interface.removeCollisionObjects(object_ids);

    visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

/* Wait for MoveGroup to recieve and process the attached collision object message */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");

    ros::shutdown();
    return 0;
}
