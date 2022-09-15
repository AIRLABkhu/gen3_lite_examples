/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const double pi=M_PI;

void go_to_pose_goal(moveit::planning_interface::MoveGroupInterface &move_group_interface,
geometry_msgs::Pose &target_pose) {
  // .. _move_group_interface-planning-to-pose-goal:

  move_group_interface.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;


  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Finally, to execute the trajectory stored in my_plan, you could use the following method call:
  // Note that this can lead to problems if the robot moved in the meanwhile.
  move_group_interface.execute(my_plan);

}
// for gripper handling
void  controll_gripper 
(moveit::planning_interface::MoveGroupInterface &move_group_interface,double value)

{
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;

  joint_group_positions=move_group_interface.getCurrentJointValues();
  joint_group_positions[0]=0;
  joint_group_positions[1]=0;
  joint_group_positions[2]=value;
  joint_group_positions[3]=0;
  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  move_group_interface.setJointValueTarget(joint_group_positions);
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  move_group_interface.execute(my_plan);
}



geometry_msgs::Pose list_to_pose(double x,double y,double z,double roll,double pitch,double yaw) {

geometry_msgs::Pose target_pose;
tf2::Quaternion orientation;
orientation.setRPY(roll,pitch, yaw);
target_pose.orientation= tf2::toMsg(orientation);
target_pose.position.x=x;
target_pose.position.y=y;
target_pose.position.z=z;

return target_pose;
}

void pick_cube_1(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);

geometry_msgs::Pose target_pose1,target_pose2;
target_pose1=list_to_pose(0.35, 0, 0.28, 0, -pi, -pi/2);
target_pose2=list_to_pose(0.35, 0, 0.20, 0, -pi, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);

 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
move_group.attachObject("cube_1");

}


void place_cube_1(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.35, 0, 0.23, 0, -pi, -pi/2);
target_pose2=list_to_pose(0.35, 0.2, 0.23, 0, -pi, -pi/2);
target_pose3=list_to_pose(0.35, 0.2, 0.13, 0, -pi, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);

 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
move_group.detachObject("cube_1");
controll_gripper(gripper,0.9);
}

void go_up(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

geometry_msgs::Pose target_pose1,target_pose2;
target_pose1=list_to_pose(0.35, 0.2, 0.13, 0, -pi, -pi/2);
target_pose2=list_to_pose(0.35, 0.2, 0.28, 0, -pi, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);


 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);

}

void pick_cube_0(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

controll_gripper(gripper,0.5);
geometry_msgs::Pose target_pose1,target_pose2;
target_pose1=list_to_pose(0.35, 0, 0.28, 0, -pi, -pi/2);
target_pose2=list_to_pose(0.35, 0, 0.15, 0, -pi, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);

 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
move_group.attachObject("cube_0");

}


void place_cube_0(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

geometry_msgs::Pose target_pose1,target_pose2,target_pose3;
target_pose1=list_to_pose(0.35, 0, 0.28, 0, -pi, -pi/2);
target_pose2=list_to_pose(0.35, 0.2, 0.28, 0, -pi, -pi/2);
target_pose3=list_to_pose(0.35, 0.2, 0.18, 0, -pi, -pi/2);
moveit_msgs::RobotTrajectory trajectory;
std::vector<geometry_msgs::Pose> waypoints;
waypoints.push_back(target_pose1);
waypoints.push_back(target_pose2);
waypoints.push_back(target_pose3);

 // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
move_group.execute(trajectory);
move_group.detachObject("cube_0");
controll_gripper(gripper,0.9);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
 const std::string & planning_frame)
{
  
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table_0";
  collision_objects[0].header.frame_id = planning_frame;

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.5;
  collision_objects[0].primitives[0].dimensions[1] = 0.5;
  collision_objects[0].primitives[0].dimensions[2] = 0.1;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0.0;
  collision_objects[0].primitive_poses[0].position.z = 0.05;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "cube_0";
  collision_objects[1].header.frame_id = planning_frame;

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.05;
  collision_objects[1].primitives[0].dimensions[1] = 0.05;
  collision_objects[1].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.35;
  collision_objects[1].primitive_poses[0].position.y = 0.0;
  collision_objects[1].primitive_poses[0].position.z = 0.13;

  collision_objects[1].operation = collision_objects[1].ADD;

  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = planning_frame;
  collision_objects[2].id = "cube_1";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.05;
  collision_objects[2].primitives[0].dimensions[1] = 0.05;
  collision_objects[2].primitives[0].dimensions[2] = 0.05;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.35;
  collision_objects[2].primitive_poses[0].position.y = 0.0;
  collision_objects[2].primitive_poses[0].position.z = 0.18;

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  arm.setPlanningTime(45.0);

  moveit::core::RobotStatePtr initial_pose = arm.getCurrentState();
  const moveit::core::JointModelGroup* joint_model_group =
      arm.getCurrentState()->getJointModelGroup("arm");
  const std::string & planning_frame=arm.getPlanningFrame();
  addCollisionObjects(planning_scene_interface,planning_frame);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick_cube_1(arm,gripper);

  ros::WallDuration(1.0).sleep();

  place_cube_1(arm,gripper);

  ros::WallDuration(1.0).sleep();

  go_up(arm,gripper);

  ros::WallDuration(1.0).sleep();

  pick_cube_0(arm,gripper);

  ros::WallDuration(1.0).sleep();

  place_cube_0(arm,gripper);

  ros::WallDuration(1.0).sleep();

  ros::shutdown(); 
  return 0;
}

