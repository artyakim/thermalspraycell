//
// Created by artemyakimchuk on 20/11/2019.
//

#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#define DEG_TO_RAD (M_PI/180.0)


std::vector<geometry_msgs::Pose> createWaypoints(geometry_msgs::Pose primitive, double dim_a, double dim_b) {
    tf::Transform rot_tcp;
    double radius = 0.005;
    double offset = 0.15;
    double delta = 0.001;
    double n = ceil(dim_b / (2 * ((2 * radius) - delta)));
    double r, p, y;
    tf::poseMsgToTF(primitive, rot_tcp);
    //rot_tcp.getBasis().getRPY(p,r,y); // roll and pitch are different cause base_link do not match world
    rot_tcp.getBasis().getRPY(r, p, y); // roll and pitch are different cause base_link do not match world
    //y-= M_PI/2 ;
    //y-= 0 ;

    //p +=M_PI/2;
    //r +=M_PI/2;

    std::vector<geometry_msgs::Pose> waypoints;
    tf::Transform tf_of_object, tf_current;
    geometry_msgs::Pose point;
    tf::poseMsgToTF(primitive, tf_of_object);

    tf_of_object.getBasis().setRPY(r, p, y);

//    tf_current.getOrigin().setX(- dim_b/2-(2*radius));
//    tf_current.getOrigin().setY(+ dim_a/2+(radius));
    tf_current.getOrigin().setX(-dim_b / 2);
    tf_current.getOrigin().setY(+dim_a / 2);
    tf_current.getOrigin().setZ(-offset);
    tf_current.getBasis().setRPY(0, 0, 0);

    tf_of_object *= tf_current;

    tf::poseTFToMsg(tf_of_object, point);
    waypoints.push_back(point);

    for (int i = 0; i < n; ++i) {
        //tf_of_object.getOrigin().setZ((tf_of_object.getOrigin().z() - dim_a - 2*(2*radius))) ;

        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setX(-(-dim_b - 2 * (2 * radius)));
        tf_of_object *= tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

        //tf_of_object.getOrigin().setX((tf_of_object.getOrigin().x() + (2*radius)-delta)) ;
        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setY(-((2 * radius) - delta));
        tf_of_object *= tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

        //tf_of_object.getOrigin().setZ((tf_of_object.getOrigin().z() + dim_a + 2*(2*radius))) ;
        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setX(-(+dim_b + 2 * (2 * radius)));
        tf_of_object *= tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

        //tf_of_object.getOrigin().setX((tf_of_object.getOrigin().x() + (2*radius)-delta)) ;
        tf_current.getOrigin().setZero();
        tf_current.getOrigin().setY(-(+(2 * radius) - delta));
        tf_of_object *= tf_current;
        tf::poseTFToMsg(tf_of_object, point);
        waypoints.push_back(point);

    }
    return waypoints;

}

void addPrimitiveBox(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface,
                     moveit::planning_interface::MoveGroupInterface &group,
                     std::string box_id,
                     tf::Vector3 box_size,
                     tf::Transform box_tf
) {
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

    geometry_msgs::Pose box_pose;
    tf::poseTFToMsg(box_tf, box_pose);


    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    //Add collision object to the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

}

std::vector<geometry_msgs::Pose> Interpolate(geometry_msgs::Pose n, geometry_msgs::Pose m) {
    //Define the biggest shortcut of xyz.
    std::vector<geometry_msgs::Pose> InterWayPoints;
    InterWayPoints.clear();
    //step size is 1mm;
    float delta = 0.005;
    float d_x = m.position.x - n.position.x;
    float d_y = m.position.y - n.position.y;
    float d_z = m.position.z - n.position.z;
    float d_max = std::max({abs(d_x), abs(d_y), abs(d_z)});
    int steps = d_max / delta; //define number of steps in the biggest shortcut
    geometry_msgs::Pose current_pose = n;

    for (int i = 0; i < steps; ++i) {
        current_pose.position.x += d_x / steps;
        current_pose.position.y += d_y / steps;
        current_pose.position.z += d_z / steps;
        InterWayPoints.push_back(current_pose);
    }
    return InterWayPoints;
}

std::vector<geometry_msgs::Pose> InterWayPoints(std::vector<geometry_msgs::Pose> waypoints) {
    std::vector<geometry_msgs::Pose> inter_waypoints, new_points;
    inter_waypoints.clear();
    for (int n = 0; n < waypoints.size() - 1; ++n) {
        //Here we attach interpolated trajectory to existing position vector by insert function
        new_points = Interpolate(waypoints[n], waypoints[n + 1]);
        inter_waypoints.insert(std::end(inter_waypoints), std::begin(new_points), std::end(new_points));
        new_points.clear();
    }
    return inter_waypoints;
}


int main(int argc, char **argv) {

    ROS_INFO_NAMED("tutorial", "trajectory execution node has started");
    ros::init(argc, argv, "ur5_control");
    ros::NodeHandle n("~");
    if (n.ok()) {
        ROS_INFO("ok status was received from n.ok() check");
    }

    tf::StampedTransform object_transform;
    tf::TransformListener listener_for_tf;
    tf::Vector3 vec(0, 0, 0);
    object_transform.setOrigin(vec);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    //Initialize MoveGroup
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPlanningTime(20.5);
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setEndEffectorLink("tcp");
    const ::robot_state::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(
            "manipulator");
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

    visual_tools.prompt("Press NEXT to start and load transform ");

    if (n.ok()) {
        ROS_INFO("Waiting for transform....");
    }
    if (listener_for_tf.waitForTransform("base_link", "object_pose", ros::Time(), ros::Duration(10.0))) {
        listener_for_tf.lookupTransform("base_link", "object_pose", ros::Time(0), object_transform);
        ros::spinOnce();
    }

    group.setStartStateToCurrentState();
    group.setMaxVelocityScalingFactor(0.9);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    moveit::core::RobotStatePtr current_state = group.getCurrentState();
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    geometry_msgs::Pose current_pose;
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
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;

    tf::Vector3 box_size;
    box_size.setX(0.105);
    box_size.setY(0.115);
    box_size.setZ(0.05);
    tf::Transform box_tf;
    ///////////////////////////////////////////////////////////////ADD TF HERE

    box_tf.setRotation(object_transform.getRotation());
    box_tf.setOrigin(object_transform.getOrigin());

    ros::NodeHandle nh;
    std::string stl_path;
    if (nh.getParam("/trajectory_node/stl_path", stl_path)) {
        ROS_INFO_NAMED("cv node", "Path to stl file: %s", stl_path.c_str());
    }
    Eigen::Vector3d vectorScale(1, 1, 1);
    std::string stl_path_with_prefix = "file://" + stl_path;
    shapes::Mesh *meshObject = shapes::createMeshFromResource(stl_path_with_prefix, vectorScale);
    shape_msgs::Mesh mesh;
    shapes::ShapeMsg meshMessage;
    shapes::constructMsgFromShape(meshObject, meshMessage);
    // Now, let's add the collision object into the world
    ROS_INFO_NAMED("tutorial", "Add an object into the world");
    mesh = boost::get<shape_msgs::Mesh>(meshMessage);
    collision_object.meshes.push_back(mesh);
    geometry_msgs::Pose box_pose;
    tf::poseTFToMsg(box_tf, box_pose);
    collision_object.mesh_poses.push_back(box_pose);

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);
    planning_scene_interface.addCollisionObjects(collision_objects);

    visual_tools.publishAxisLabeled(box_pose, "object", rvt::SMALL);
    //Add collision object to the world

    visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    visual_tools.prompt("Press NEXT to plan to HOME position");

    const float small_deviation = 5 * DEG_TO_RAD;
    joint_group_positions = {small_deviation, small_deviation, small_deviation, small_deviation, small_deviation,
                             small_deviation};

    group.setJointValueTarget(joint_group_positions);
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    visual_tools.prompt("Press NEXT to MOVE to HOME position");
    group.move();

    joint_group_positions[1] = 6 * DEG_TO_RAD;
    joint_group_positions[2] = 6 * DEG_TO_RAD;
    joint_group_positions[3] = 6 * DEG_TO_RAD;
    joint_group_positions[4] = 6 * DEG_TO_RAD;

    group.setJointValueTarget(joint_group_positions);
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    visual_tools.prompt("Press NEXT to MOVE to CLOSER position");
    group.move();
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    visual_tools.trigger();
    visual_tools.prompt("Press to form Spraying TRAJECTORY");

    tf::Transform translate_to_complex_obj_oY, translate_to_complex_obj_oX, obj_tf_final, translate_to_complex_obj_oZ;//Create tf to translate trajectory from box to complex object
    ///////////////////////////////////////////////////////////////ADD TF HERE

    translate_to_complex_obj_oY.getBasis().setRPY(0.0, -90 * DEG_TO_RAD, 0.0);
    translate_to_complex_obj_oX.getBasis().setRPY(39.85 * DEG_TO_RAD, 0.0, 0.0);
    translate_to_complex_obj_oZ.getBasis().setRPY(0.0, 0.0, -90 * DEG_TO_RAD);
    translate_to_complex_obj_oX.getOrigin().setZero();
    translate_to_complex_obj_oY.getOrigin().setZero();
    translate_to_complex_obj_oZ.getOrigin().setX(0.0525);
    translate_to_complex_obj_oZ.getOrigin().setY(-0.0007);
    translate_to_complex_obj_oZ.getOrigin().setZ(-0.112);
    obj_tf_final = box_tf * translate_to_complex_obj_oY * translate_to_complex_obj_oX * translate_to_complex_obj_oZ;
    tf::poseTFToMsg(obj_tf_final, box_pose);

    waypoints.clear();
    inter_waypoints.clear();

    waypoints = createWaypoints(box_pose, box_size.getY(), box_size.getX());

    inter_waypoints = InterWayPoints(waypoints);

    moveit_msgs::RobotTrajectory trajectory_paint;
    double fraction = group.computeCartesianPath(inter_waypoints, eef_step, jump_threshold, trajectory_paint);

//    Use of TimeOptimalTrajectoryGeneration
    rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_paint);
    success = time_param.computeTimeStamps(rt, 0.91, 0.9);
    ROS_INFO("Computed time stamp %s", success ? "SUCCEDED" : "FAILED");
    rt.getRobotTrajectoryMsg(trajectory_paint);
    my_plan.trajectory_ = trajectory_paint;


    //Visualize traj in RViz
    visual_tools.deleteAllMarkers();
    visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishPath(inter_waypoints, rvt::LIME_GREEN, rvt::SMALL);
    for (std::size_t i = 0; i < inter_waypoints.size(); ++i)
        visual_tools.publishAxisLabeled(inter_waypoints[i], "pt" + std::to_string(i), rvt::SMALL);

    visual_tools.publishAxisLabeled(box_pose, "object", rvt::SMALL);
    visual_tools.trigger();

    visual_tools.prompt("PRESS NEXT TO EXECUTE TRAJ");
    group.execute(my_plan);

    visual_tools.prompt("Press NEXT to plan to HOME position");

    joint_group_positions[0] = 5 * DEG_TO_RAD;
    joint_group_positions[1] = 5 * DEG_TO_RAD;
    joint_group_positions[2] = 5 * DEG_TO_RAD;
    joint_group_positions[3] = 5 * DEG_TO_RAD;
    joint_group_positions[4] = 5 * DEG_TO_RAD;
    joint_group_positions[5] = 5 * DEG_TO_RAD;

    group.setJointValueTarget(joint_group_positions);
    success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    visual_tools.prompt("Press NEXT to MOVE to HOME position");
    group.move();

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Wait for MoveGroup to recieve and process the attached collision object message */

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
