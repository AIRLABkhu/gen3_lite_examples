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
void  go_to_joint_state 
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

void pick(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::MoveGroupInterface& gripper)
{

//go_to_pose_goal(move_group,target_pose);
go_to_joint_state(gripper,0.15);

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  grasps[0].grasp_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, 0, 0);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.05;
  grasps[0].grasp_pose.pose.position.y = 0.35;
  grasps[0].grasp_pose.pose.position.z = 0.3;



//geometry_msgs::Pose target_pose;
//tf2::Quaternion orientation;
//orientation.setRPY(-M_PI / 2, 0, 0);
//target_pose.orientation= tf2::toMsg(orientation);
//target_pose.position.x=0.05;
//target_pose.position.y=0.35-0.1;
//target_pose.position.z=0.3;

//go_to_pose_goal(move_group,target_pose);
//go_to_joint_state(gripper,0.15);

//target_pose.position.y+=0.1;
//go_to_pose_goal(move_group,target_pose);
//go_to_joint_state(gripper,0.15);

move_group.setSupportSurfaceName("table1");
move_group.pick("object",grasps);
}


void place(moveit::planning_interface::MoveGroupInterface& move_group,
moveit::planning_interface::MoveGroupInterface& gripper)
{

//geometry_msgs::Pose target_pose;
//tf2::Quaternion orientation;
//orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);
//target_pose.orientation= tf2::toMsg(orientation);
//target_pose.position.x=0.35;
//target_pose.position.y=0.5;
//target_pose.position.z=0.3;

//go_to_pose_goal(move_group,target_pose);


std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, 0, -M_PI / 2);  // A quarter turn about the z-axis
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* For place location, we set the value to the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0.35;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.3;


move_group.setSupportSurfaceName("table2");
move_group.place("object",place_location);
go_to_joint_state(gripper,0.9);
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
 const std::string & planning_frame)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = planning_frame;

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.3;
  collision_objects[0].primitives[0].dimensions[1] = 0.6;
  collision_objects[0].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.45;
  collision_objects[0].primitive_poses[0].position.y = 0.3;
  collision_objects[0].primitive_poses[0].position.z = 0.1;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = planning_frame;

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.3;
  collision_objects[1].primitives[0].dimensions[1] = 0.3;
  collision_objects[1].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.15;
  collision_objects[1].primitive_poses[0].position.y = 0.45;
  collision_objects[1].primitive_poses[0].position.z = 0.1;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = planning_frame;
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.05;
  collision_objects[2].primitive_poses[0].position.y = 0.35;
  collision_objects[2].primitive_poses[0].position.z = 0.3;
  // END_SUB_TUTORIAL

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
  //planning_frame is "base_link" I just show how to get planningframe from MoveGroupInterface  
  const std::string & planning_frame=arm.getPlanningFrame();
  addCollisionObjects(planning_scene_interface,planning_frame);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(arm,gripper);

  ros::WallDuration(1.0).sleep();

  place(arm,gripper);

  ros::waitForShutdown();
  return 0;
}

