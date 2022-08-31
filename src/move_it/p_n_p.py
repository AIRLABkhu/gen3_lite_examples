#!/usr/bin/env python


# We use MoveIT::Grasping

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
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf.transformations import quaternion_from_euler

(X, Y, Z, W) = (0, 1, 2, 3)
OPEN = 0.9
CLOSE = 0.15
PICK_ORIENTATION_EULER = [-math.pi / 2, 0, 0]
PLACE_ORIENTATION_EULER = [-math.pi / 2, 0, -math.pi / 2]

def create_collision_object(id, dimensions, pose):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = 'base_link'

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[X]
    object_pose.position.y = pose[Y]
    object_pose.position.z = pose[Z]

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    return object


def main():

    #Initialize the node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    #Create the MoveItInterface necessary objects
    arm_group_name="arm"
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    #Create arm group module
    arm_group=moveit_commander.MoveGroupCommander(arm_group_name)
    display_trajectory_publisher = rospy.Publisher('move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

    #Create gripper group module
    gripper_group_name="gripper"
    gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name)
    gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
    gripper_joint_name = gripper_joint_names[0]



    
    #Setting Environment
    table_0=create_collision_object(id='table_0',
                                    dimensions=[0.5,0.5,0.1],
                                    pose=[0.5,0,0.05])
                
    cube_0=create_collision_object(id='cube_0',
                                    dimensions=[0.05,0.05,0.05],
                                    pose=[0.35,0,0.13])

    cube_1=create_collision_object(id='cube_1',
                                    dimensions=[0.05,0.05,0.05],
                                    pose=[0.35,0,0.18])

                 

    scene.add_object(table_0)
    scene.add_object(cube_0)
    scene.add_object(cube_1)

    # move to pose goal
    rospy.loginfo("Going to home")
    arm_group.set_named_target("home")
    plan=arm_group.plan()
    arm_group.execute(plan,wait=True)


    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.28, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)



    # Gripper Open    
    # We only have to move this joint because all others are mimic!
    gripper_joint = robot.get_joint(gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    gripper_joint.move(0.5 * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)

    #list_to_pose function transforms 6DOF input to pose data for calculating path.
    #Move for grasping cube0
    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.20, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)
    
    #Grasping
    touch_links=['left_finger_dist_link','left_finger_dist_link']
    gripper_group.attach_object('cube_1','gripper_base_link',touch_links=touch_links)

    #Move for detaching
    waypoints=[]
    pose_goal1=list_to_pose([0.35, 0, 0.23, 0, -pi, -pi/2])
    pose_goal2=list_to_pose([0.35, 0.2, 0.23, 0, -pi, -pi/2])
    pose_goal3=list_to_pose([0.35, 0.2, 0.13, 0, -pi, -pi/2])
    waypoints.append(pose_goal1)
    waypoints.append(pose_goal2)
    waypoints.append(pose_goal3)
    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)


    #Detach Object
    gripper_group.detach_object('cube_1')


    rospy.loginfo("Going to home")
    arm_group.set_named_target("home")
    plan=arm_group.plan()
    arm_group.execute(plan,wait=True)

    #Move for grasping cube1
    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.28, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)

    waypoints=[]
    pose_goal=list_to_pose([0.35, 0, 0.15, 0, -pi, -pi/2])
    waypoints.append(pose_goal)

    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)

    #Grasping
    touch_links=['left_finger_dist_link','left_finger_dist_link']
    gripper_group.attach_object('cube_0','gripper_base_link',touch_links=touch_links)

    #Move for detaching
    waypoints=[]
    pose_goal1=list_to_pose([0.35, 0, 0.28, 0, -pi, -pi/2])
    pose_goal2=list_to_pose([0.35, 0.2, 0.28, 0, -pi, -pi/2])
    pose_goal3=list_to_pose([0.35, 0.2, 0.18, 0, -pi, -pi/2])
    waypoints.append(pose_goal1)
    waypoints.append(pose_goal2)
    waypoints.append(pose_goal3)
    plan, _ = arm_group.compute_cartesian_path(
                                waypoints,   # waypoints to follow
                                0.01,        # eef_step
                                0.0)         # jump_threshold
    arm_group.execute(plan,wait=True)


    #Detach Object
    gripper_group.detach_object('cube_0')

    rospy.loginfo("Going to home")
    arm_group.set_named_target("home")
    plan=arm_group.plan()
    arm_group.execute(plan,wait=True)







if __name__=='__main__':
  main()
