/* Author: Hyoseok Hwang & Jungwoon Lee @ AIRLAB, KHU*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>
#include <iostream>

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

geometry_msgs::Pose list_to_pose(double x,double y,double z,double roll,double pitch,double yaw)
{

  geometry_msgs::Pose target_pose;
  tf2::Quaternion orientation;
  orientation.setRPY(roll,pitch, yaw);
  target_pose.orientation= tf2::toMsg(orientation);
  target_pose.position.x=x;
  target_pose.position.y=y;
  target_pose.position.z=z;

  return target_pose;
}


void move_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  
  geometry_msgs::Pose target_pose;
  target_pose=list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  ROS_INFO("Move along Y-axis");
  target_pose.position.y+=0.05;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.y-=0.1;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.y+=0.05;
  go_to_pose_goal(move_group_interface,target_pose);

  ROS_INFO("Move along x-axis");
  target_pose.position.x+=0.02;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.x-=0.05;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.x+=0.02;
  go_to_pose_goal(move_group_interface,target_pose);

  ROS_INFO("Move along z-axis");
  target_pose.position.z+=0.1;
  go_to_pose_goal(move_group_interface,target_pose);
  target_pose.position.z-=0.1;
  go_to_pose_goal(move_group_interface,target_pose);

}

void rotation_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  geometry_msgs::Pose target_pose;
  target_pose=list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  target_pose=list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0.3, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  target_pose=list_to_pose(0.34, 0.2, 0.3, -M_PI/2, -0.3, 0);
  go_to_pose_goal(move_group_interface,target_pose);

  target_pose=list_to_pose(0.34, 0.2, 0.3, -M_PI/2 ,0, 0);
  go_to_pose_goal(move_group_interface,target_pose);

}

void gripper_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface,double value)
{
  ROS_INFO("gripper sample"); 
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

void waypoint_sample(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{
  ROS_INFO("way_point");
  moveit_msgs::RobotTrajectory trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose start = move_group_interface.getCurrentPose().pose;

  start.position.y-=0.1;
  waypoints.push_back(start);
  start.position.x+=0.1;
  waypoints.push_back(start);

  // with end points of end-effector in waypoints and computerCartesianPath function, you can plan Cartesian path.
  double fraction = move_group_interface.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
  move_group_interface.execute(trajectory);
}

void draw_rectangle(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{


}

void draw_circle(moveit::planning_interface::MoveGroupInterface &move_group_interface)
{


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "khu_moveit_sameple");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm("arm");
  moveit::planning_interface::MoveGroupInterface gripper("gripper");
  arm.setPlanningTime(45.0);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();
  
    // place(arm,gripper);
	// pick(arm,gripper);

  bool loop = true;
  while(loop)
  {
    int cmd;
    std::cout << "Menu? (0:exit / 1:move / 2:rotation / 3:gripper / 4:waypoint / 5:draw rectangle / 6:draw circle) ";
    std::cin >> cmd;
    switch(cmd) {
      case 1:
        move_sample(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 2:
        rotation_sample(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 3:
        gripper_sample(gripper, 0.16);
        ros::WallDuration(1.0).sleep();
        gripper_sample(gripper, 0.0);
        ros::WallDuration(1.0).sleep();
        break;
      case 4:
        waypoint_sample(arm);
        ros::WallDuration(1.0).sleep();
        break;
      case 0:
        loop = false;
        ROS_INFO("Quit");
        break;
    }
  }
  ros::waitForShutdown();
  return 0;
}

