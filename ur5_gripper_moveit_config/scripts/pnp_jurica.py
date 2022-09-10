#!/usr/bin/env python

import time

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32 as float32
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_matrix, quaternion_about_axis

# gpd_ros is name of the GPD ROS Wrapper package in workspace, imports message from there
from gpd_ros.msg import GraspConfigList

# Global variable for storing (current) grasp info
grasps = []
pcd = PointCloud2()

def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


"""PickAndPlace"""
class PickAndPlace(object):

  def __init__(self):
    super(PickAndPlace, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('PickAndPlace', anonymous=True)

    group_name_arm = "ur5_arm"
    group_name_hand = "gripper"

    self.group_arm = moveit_commander.MoveGroupCommander(group_name_arm)
    self.group_hand = moveit_commander.MoveGroupCommander(group_name_hand)

    self.scene = moveit_commander.PlanningSceneInterface()
    self.planning_frame_arm = self.group_arm.get_planning_frame()
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    
    self.pcd_pub = rospy.Publisher('/gpd_input',
                                               PointCloud2,
                                               queue_size=1)

    # Time is required to pass before you can call scene object (it needs to connect to ROS Master)
    rospy.sleep(0.5)

    self.planning_frame_hand = self.group_hand.get_planning_frame()
    
    self.box_name = 'box'

    self.add_table()

    self.robot = moveit_commander.RobotCommander()

    self.group_arm.set_goal_tolerance(0.03)
    self.group_hand.set_goal_tolerance(0.03)

    self.group_arm.allow_replanning(True)
    self.group_hand.allow_replanning(True)

    self.group_names = self.robot.get_group_names()

  # ROS types <--> Numpy types
  def from_point(self, msg):
    """
    Converts a C{geometry_msgs/Point} ROS message into a numpy array.
    @type  msg: geometry_msgs/Point
    @param msg: The ROS message to be converted
    @rtype: np.array
    @return: The resulting numpy array
    """
    return self.from_vector3(msg)

  @staticmethod
  def to_point(array):
    """
    Converts a numpy array into a C{geometry_msgs/Point} ROS message.
    @type  array: np.array
    @param array: The position as numpy array
    @rtype: geometry_msgs/Point
    @return: The resulting ROS message
    """
    return geometry_msgs.msg.Point(*array)
  
  @staticmethod
  def from_vector3(msg):
    """
    Converts a C{geometry_msgs/Vector3} ROS message into a numpy array.
    @type  msg: geometry_msgs/Vector3
    @param msg: The ROS message to be converted
    @rtype: np.array
    @return: The resulting numpy array
    """
    return np.array([msg.x, msg.y, msg.z])

  @staticmethod
  def point_displacement(point, vector, displacement):
    unit_vector = vector / np.linalg.norm(vector)
    return point + displacement * unit_vector


  """Method for robot joint goal above place table; calling the 
     method executes the action of robot and returns if the desired joint
     pose goal is within specified tolerance"""
  def place_location_up(self):

    # Location above the place table; still has to go down to place

    joint_goal = self.group_arm.get_current_joint_values()
    joint_goal[0] = 2.96
    joint_goal[1] = -1.13
    joint_goal[2] = 1.38
    joint_goal[3] = -1.81
    joint_goal[4] = -1.57
    joint_goal[5] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the self.group_arm
    self.group_arm.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    self.group_arm.stop()

    # Gripper open
    self.group_hand.set_named_target ('open')
    self.group_hand.go(wait=True)
    self.group_hand.stop()
  
  """Robot plans and executes to (named) target goal which was specified in moveit_setup_assistant"""
  def gpd_home(self):

    # Go to optimal camera location
    self.group_arm.set_named_target('gpd_home')
    self.group_arm.go(wait=True) # Maybe add try: ... except: here if the plan fails to execute
    self.group_arm.stop()

    # Open gripper fully
    self.group_hand.set_named_target ('open')
    self.group_hand.go(wait=True)
    self.group_hand.stop()


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    start = rospy.get_time()
    seconds = rospy.get_time()

    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])

      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def attach_box(self, timeout=4):

    touch_links = ['robotiq_85_left_finger_tip_link', 'robotiq_85_right_finger_tip_link', 'robotiq_85_base_link', 'robotiq_85_left_finger_link', 'robotiq_85_left_inner_knuckle_link',
    'robotiq_85_left_knuckle_link', 'robotiq_85_right_finger_link', 'robotiq_85_right_inner_knuckle_link', 'robotiq_85_right_knuckle_link', 'robotiq_coupler' ]

    self.scene.attach_box('ee_link', self.box_name, touch_links=touch_links)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=True, timeout=timeout)

  def add_table(self, timeout=4):
    
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = 'base_link'
    table_pose.pose.orientation.w = 1.0

    table_pose.pose.position.x = 0.95
    table_pose.pose.position.z = -0.3

    self.scene.add_box('table', table_pose, size = (2, 3, 0.593))

    return self.wait_for_state_update(box_is_known=True, timeout = timeout)

  def add_box(self, grasp, timeout = 4):
    
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = 'base_link'
    box_pose.pose.position = grasp.sample
    box_pose.pose.orientation = self.goal_pose_hand.orientation

    self.scene.add_box(self.box_name, box_pose, size = (0.045, 0.045, 0.045))

    return self.wait_for_state_update(box_is_known=True, timeout = timeout)

  def callback(self, msg):

    global grasps
    grasps = msg.grasps

  # Listens until exactly one message is received from gpd 
  def gpd_listener(self):
    callback = self.callback

    grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps',GraspConfigList, callback)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():    
        if len(grasps) > 0:
            rospy.loginfo('Received %d grasps.', len(grasps))
            break
        rate.sleep()

  def callback_pcd(self, msg):
    global pcd
    pcd = msg

  # Listens until exactly one message is received from gpd 
  def pcd_listener(self):

    pcd_sub = rospy.Subscriber('/cloud_stitched', PointCloud2, self.callback_pcd)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():    
        if  len(pcd.fields) > 0:
          rospy.loginfo('Message received.')
          rospy.loginfo(pcd.header)
          break
        rate.sleep()

  def pcd_publish(self):

    self.pcd_pub.publish(pcd)

  def grasp_approach(self, grasp_pose):

    unit_vector = self.approach / np.linalg.norm(self.approach)
    
    waypoints = []
    wpose = grasp_pose

    wpose.position.x += unit_vector[0] * 0.05
    wpose.position.y += unit_vector[1] * 0.05
    wpose.position.z += unit_vector[2] * 0.05

    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += unit_vector[0] * 0.05
    wpose.position.y += unit_vector[1] * 0.05
    wpose.position.z += unit_vector[2] * 0.05

    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += unit_vector[0] * 0.05
    wpose.position.y += unit_vector[1] * 0.05
    wpose.position.z += unit_vector[2] * 0.05

    waypoints.append(copy.deepcopy(wpose))


    (plan, fraction) = self.group_arm.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0,
                                      avoid_collisions = True)         # jump_threshold

    return plan, fraction

  def execute_plan(self, plan):
      self.group_arm.execute(plan, wait=True)

  # First version, tries to go directly without goind back 10cm and then executing trajectory
  def pick(self, grasp):
    
    # Orientation matrix obtained from the 3 orientation vectors of GraspConfig message
    M = np.array([[grasp.approach.x, grasp.binormal.x, grasp.axis.x, 0],
                 [grasp.approach.y, grasp.binormal.y, grasp.axis.y, 0],
                 [grasp.approach.z, grasp.binormal.z, grasp.axis.z, 0],
                 [0                ,0               ,0            , 1]])
    q = quaternion_from_matrix(M)

    # Position and orientation of grasp pose, received from GPD
    goal_pose = geometry_msgs.msg.Pose()
    goal_pose.orientation = q
    goal_pose.position = grasp.position

    # Since self.group_arm commander is executing to goal position of 'ee_link', we have to translate the goal position
    # so it corresponds to the position base of hand should achieve
    approach = self.from_vector3(grasp.approach)
    self.approach = approach
    goal_pose_position = self.from_point(goal_pose.position)

    # Distance between 'ee_link' and base of robot hand
    displacement = 0.09592
    goal_pose_hand = geometry_msgs.msg.Pose()
    goal_pose_hand.orientation.x = q[0]
    goal_pose_hand.orientation.y = q[1]
    goal_pose_hand.orientation.z = q[2]
    goal_pose_hand.orientation.w = q[3]

    self.goal_pose_hand = goal_pose_hand

    goal_pose_hand.position = self.to_point(self.point_displacement(goal_pose_position, approach, -displacement))
    goal_pose_hand_position = self.from_point(goal_pose_hand.position)

    pre_grasp_pose_hand = goal_pose_hand
    pre_grasp_pose_hand.position = self.to_point(self.point_displacement(goal_pose_hand_position, approach, -0.15))

    # First it goes into pre grasp pose which is 15 cm away in approach vector direction
    self.group_arm.set_pose_target(pre_grasp_pose_hand)
    self.group_arm.go(wait=True)
    self.group_arm.stop()
    self.group_arm.clear_pose_targets()

    print 'Is the pre_grasp pose within tolerance:', all_close(pre_grasp_pose_hand, self.group_arm.get_current_pose().pose, 0.01)
    print 'Goal pose:', pre_grasp_pose_hand
    print '\n actual pose:', self.group_arm.get_current_pose().pose

    # Execute to grasp pose
    plan, fraction = self.grasp_approach(pre_grasp_pose_hand)

    time.sleep(2)

    self.execute_plan(plan)
    print 'Is the grasp pose within tolerance:', all_close(goal_pose_hand, self.group_arm.get_current_pose().pose, 0.01)
    print 'Goal pose:', goal_pose_hand
    print '\n actual pose:', self.group_arm.get_current_pose().pose

    # Gripper close

    hand_close_width = self.group_hand.get_current_joint_values()

    hand_close_width[0] = 0.9 - grasp.width.data*10
    rospy.loginfo('Closing to width %s.', hand_close_width)


    self.group_hand.go(hand_close_width, wait=True)
    self.group_hand.stop()

    self.goal_state = goal_pose_hand

  def detach_box(self, timeout=4):

    self.scene.remove_attached_object('ee_link', name=self.box_name)


  def retreat(self):
    '''Retreat from the pick location; straight up 15cm'''

    retreat_pose = self.group_arm.get_current_pose().pose
    retreat_pose.position.z += 0.15

    self.group_arm.set_pose_target(retreat_pose)
    self.group_arm.go(wait=True)
    self.group_arm.stop()

    self.group_arm.set_named_target ('gpd_home')
    self.group_arm.go(wait=True)
    self.group_arm.stop()

def main():

  # Create object and initialize everything
  pnp = PickAndPlace()
  global pcd

  try:
    while True:
      
      # First go to home pose and open gripper
      pnp.gpd_home()

      input = raw_input("Press <Enter> to start/continue grasping or q to quit: ")

      if input == '':
        # Reset the values before next cycle
        pcd = PointCloud2()
        del grasps[:]



        pnp.pcd_listener()

        pnp.pcd_publish()

        # Subscribes to '/detect_grasps/clustered_grasps'
        print 'Waiting for grasps.'
        pnp.gpd_listener()

        while len(grasps) < 1:
          rospy.sleep(1)

        # Tu napravi da planira prvo putanje i ak je izvedivo moze, a ak ne da izabere drugi grasp
        grasp = grasps[0] # napravi bolju heuristiku za odabir graspa, nekaj ak ne uspije plan ili collision i slicno

        pnp.pick(grasp)

        pnp.retreat()

        pnp.place_location_up()

        pnp.gpd_home()

        pass
      elif input == 'q':
        break
      else:
        print 'Sorry, couldnt understand that. Exiting!'
        break
  except rospy.ROSInterruptException:
      return
  except KeyboardInterrupt:
      return


if __name__ == '__main__':
  main()