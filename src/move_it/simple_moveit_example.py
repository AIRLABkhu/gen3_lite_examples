#!/usr/bin/env python

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math
from std_srvs.srv import Empty
import copy
from moveit_commander.conversions import pose_to_list, list_to_pose

class ExampleMoveItTrajectories(object):

    def __init__(self):

        #Initialize the node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('example_move_it_trajectories')

        #Create the MoveItInterface necessary objects
        arm_group_name="arm"
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        #Create arm group module
        self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
        display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
        #Create gripper group module
        gripper_group_name="gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
	gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]




    
    def reach_named_position(self,target):
        # [home,vertical]
        arm_group=self.arm_group

        # Going to one of those targets
        rospy.loginfo("Going to named target " + target)
        # Set the target
        arm_group.set_named_target(target)
        # Plan the trajectory
        planned_path1 = arm_group.plan()
        # Execute the trajectory and block while it's not finished
        arm_group.execute(planned_path1, wait=True)
            
    def reach_joint_angles(self,tolerance):

        arm_group = self.arm_group
        success = True

        # Get the current joint positions
        joint_positions = arm_group.get_current_joint_values()
        rospy.loginfo("Printing current joint positions before movement :")

        # Set the goal joint tolerance
        self.arm_group.set_goal_joint_tolerance(tolerance)

        # Set the joint target configuration
        joint_positions[0] = 0
        joint_positions[1] = 0
        joint_positions[2] = pi/2
        joint_positions[3] = pi/4
        joint_positions[4] = 0
        joint_positions[5] = pi/2
        arm_group.set_joint_value_target(joint_positions)
        
        # Plan and execute in one command
        arm_group.go(wait=True)

    def get_cartesian_pose(self):
        arm_group = self.arm_group

        # Get the current pose
        pose = arm_group.get_current_pose()

        return pose.pose


    def reach_gripper_position(self, relative_position):
            gripper_group = self.gripper_group
            
            # We only have to move this joint because all others are mimic!
            gripper_joint = self.robot.get_joint(self.gripper_joint_name)
            gripper_max_absolute_pos = gripper_joint.max_bound()
            gripper_min_absolute_pos = gripper_joint.min_bound()
            try:
                val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
                return val
            except:
                return False 

    def compute_cartesian_path(self,waypoints):
            arm_group=self.arm_group
            (plan,fraction)=arm_group.compute_cartesian_path(waypoints,0.01,0.0)
            arm_group.execute(plan,wait=True)
            rospy.sleep(3)

            
    def draw_circle(self):
            rospy.loginfo("Circle")
            waypoints = []

            # start with the current pose
            start_pose=self.get_cartesian_pose()
            start_pose.orientation.w=1.0

            for i in range(36):
              wpose=copy.deepcopy(start_pose)
              wpose.position.x=start_pose.position.x+0.1*math.cos(i*0.2)
              wpose.position.y=start_pose.position.y+0.1*math.sin(i*0.2)
              waypoints.append(copy.deepcopy(wpose))

            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")
      
    def draw_rectangle(self):

            rospy.loginfo("Rectangle")
            waypoints=[]

            #start with the current pose
            start_pose=self.get_cartesian_pose()
            start_pose.orientation.w=1.0

            wpose=copy.deepcopy(start_pose)
            wpose.position.x-=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.y-=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.y+=0.2
            waypoints.append(copy.deepcopy(wpose))

            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")

    def draw_triangle(self):
            #Triangle
            rospy.loginfo("Triangle")
            waypoints=[]

            #start with the current pose
            start_pose=self.get_cartesian_pose()
            start_pose.orientation.w=1.0

            wpose=copy.deepcopy(start_pose)
            wpose.position.x-=0.1
            wpose.position.y-=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.2
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x-=0.1
            wpose.position.y+=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")

    def draw_a(self):
            #Draw A
            #start with the current pose
            rospy.loginfo("Draw Alphabet A")
            waypoints=[]
            center_pose=self.get_cartesian_pose()
            center_pose.orientation.w=1.0

            wpose=copy.deepcopy(center_pose)
            wpose.position.x-=0.1
            wpose.position.y-=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))


            wpose.position.x+=0.1
            wpose.position.y+=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.1
            wpose.position.y-=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))


            wpose.position.x-=0.1
            wpose.position.y+=0.1*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x-=0.05
            wpose.position.y-=0.05*math.sqrt(3)
            waypoints.append(copy.deepcopy(wpose))

            wpose.position.x+=0.1
            waypoints.append(copy.deepcopy(wpose))
            self.compute_cartesian_path(waypoints)
            self.reach_named_position("home")







def main():

  example=ExampleMoveItTrajectories()
  raw_input()
  example.reach_gripper_position(0.5)


  #For testing purposes
  example.reach_named_position("home")
  

  #Circle
  example.draw_circle()

  # Rectangle
  example.draw_rectangle()  

  # Triangle
  example.draw_triangle()

  # Draw A
  example.draw_a()  





if __name__=='__main__':
  main()


