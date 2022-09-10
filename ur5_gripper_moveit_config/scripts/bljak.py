#!/usr/bin/env python

from shutil import move
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


group_name_arm = "ur5_arm"
group_name_hand = "gripper"




moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_py_interface', anonymous=True)

# Mozda ne treba
robot = moveit_commander.RobotCommander()

# Za kolizije i slicne stvari oko okoline robota
scene = moveit_commander.PlanningSceneInterface()

#Inicijalizacija komandera za grupe artikuliranih zglobova
move_group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
move_group_hand = moveit_commander.MoveGroupCommander(group_name_hand)

# ROS Publisher trajektorije u RViz-u
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)



# PLACE GOAL, dignuti iznad, samo ga spusti nakon te tocke malo dolje jos i tjt
joint_goal = move_group_arm.get_current_joint_values()
joint_goal[0] = 2.96
joint_goal[1] = -1.13
joint_goal[2] = 1.38
joint_goal[3] = -1.81
joint_goal[4] = -1.57
joint_goal[5] = 0

move_group_arm.go(joint_goal, wait=True)
move_group_arm.stop()

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4):

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
        # Test if the box is in attached objects
        attached_objects = scene.get_attached_objects([box_name])
        is_attached = len(attached_objects.keys()) > 0

        # Test if the box is in the scene.
        # Note that attaching the box will remove it from known_objects
        is_known = box_name in scene.get_known_object_names()

        # Test if we are in the expected state
        if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

        # Sleep so that we give other threads time on the processor
        rospy.sleep(0.1)
        seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False

print move_group_arm.get_planning_frame()
# Dodavanje collision objekta (stol) u scenu; TO SUTRA
table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = 'base_link'
table_pose.pose.orientation.w = 1.0
table_pose.pose.position.z = -0.297
box_name = "table"

scene.add_box(box_name, table_pose, size=(1, 1, 0.297))
print wait_for_state_update(box_is_known=True, timeout = 4)
