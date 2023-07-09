import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, OrientationConstraint

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    
    ## Nimrod ##
    args = sys.argv
    args.append("__ns:=my_gen3")  
    moveit_commander.roscpp_initialize(args)
    ## End of Nimrod ##


    #moveit_commander.roscpp_initialize(sys.argv) <-------- the original
    #rospy.init_node('example_move_it_trajectories')
    #rospy.init_node('move_class')
    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      # if self.is_gripper_present:
      #   gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
      #   self.gripper_joint_name = gripper_joint_names[0]
      # else:
      #   gripper_joint_name = ""
      self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      joint_positions[0] = pi/2
      joint_positions[1] = 0
      joint_positions[2] = pi/4
      joint_positions[3] = -pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
      joint_positions[6] = 0.2
    elif self.degrees_of_freedom == 6:
      joint_positions[0] = 0
      joint_positions[1] = 0
      joint_positions[2] = pi/2
      joint_positions[3] = pi/4
      joint_positions[4] = 0
      joint_positions[5] = pi/2
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

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
  
  def init_revolving_joint_constraints(self):
    self.not_revolving = Constraints()
    joint_constraint = JointConstraint()

    self.not_revolving.name = "not_revolving"

    joint_constraint.position = 0
    joint_constraint.tolerance_above = 0.1
    joint_constraint.tolerance_below = 0.1
    joint_constraint.weight = 1

    joint_constraint.joint_name = "joint_1"
    self.not_revolving.joint_constraints.append(joint_constraint)

  def init_revolving_path_constraints(self,pose):
    self.not_revolving = Constraints()
    self.not_revolving.name = "not_revolving"
    orientation_constraint = OrientationConstraint()
    #orientation_constraint.header = pose.header
    orientation_constraint.link_name = self.arm_group.get_end_effector_link()
    orientation_constraint.orientation = pose.orientation
    orientation_constraint.absolute_x_axis_tolerance = 0.4
    orientation_constraint.absolute_y_axis_tolerance = 0.4
    orientation_constraint.absolute_z_axis_tolerance = 0.4
    #orientation_constraint.absolute_z_axis_tolerance = 3.14 #ignore this axis
    orientation_constraint.weight = 1

    self.not_revolving.orientation_constraints.append(orientation_constraint)

  def enable_revolving_path_constraints(self):
    self.arm_group.set_path_constraints(self.not_revolving)

  def disable_upright_path_constraints(self):
    self.arm_group.set_path_constraints(None)

# import sys
# import time
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from math import pi
# from std_srvs.srv import Empty

# class ExampleMoveItTrajectories(object):
#   """ExampleMoveItTrajectories"""
#   def __init__(self):

#     # Initialize the node
#     super(ExampleMoveItTrajectories, self).__init__()
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('example_move_it_trajectories')

#     try:
#       self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
#       if self.is_gripper_present:
#         gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
#         self.gripper_joint_name = gripper_joint_names[0]
#       else:
#         gripper_joint_name = ""
#       self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

#       # Create the MoveItInterface necessary objects
#       arm_group_name = "arm"
#       self.robot = moveit_commander.RobotCommander("robot_description")
#       self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
#       self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
#       self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
#                                                     moveit_msgs.msg.DisplayTrajectory,
#                                                     queue_size=20)

#       if self.is_gripper_present:
#         gripper_group_name = "gripper"
#         self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

#       rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
#     except Exception as e:
#       print (e)
#       self.is_init_success = False
#     else:
#       self.is_init_success = True


#   def reach_named_position(self, target):
#     arm_group = self.arm_group
    
#     # Going to one of those targets
#     rospy.loginfo("Going to named target " + target)
#     # Set the target
#     arm_group.set_named_target(target)
#     # Plan the trajectory
#     planned_path1 = arm_group.plan()
#     # Execute the trajectory and block while it's not finished
#     return arm_group.execute(planned_path1, wait=True)

#   def reach_joint_angles(self, tolerance):
#     arm_group = self.arm_group
#     success = True

#     # Get the current joint positions
#     joint_positions = arm_group.get_current_joint_values()
#     rospy.loginfo("Printing current joint positions before movement :")
#     for p in joint_positions: rospy.loginfo(p)

#     # Set the goal joint tolerance
#     self.arm_group.set_goal_joint_tolerance(tolerance)

#     # Set the joint target configuration
#     if self.degrees_of_freedom == 7:
#       joint_positions[0] = pi/2
#       joint_positions[1] = 0
#       joint_positions[2] = pi/4
#       joint_positions[3] = -pi/4
#       joint_positions[4] = 0
#       joint_positions[5] = pi/2
#       joint_positions[6] = 0.2
#     elif self.degrees_of_freedom == 6:
#       joint_positions[0] = 0
#       joint_positions[1] = 0
#       joint_positions[2] = pi/2
#       joint_positions[3] = pi/4
#       joint_positions[4] = 0
#       joint_positions[5] = pi/2
#     arm_group.set_joint_value_target(joint_positions)
    
#     # Plan and execute in one command
#     success &= arm_group.go(wait=True)

#     # Show joint positions after movement
#     new_joint_positions = arm_group.get_current_joint_values()
#     rospy.loginfo("Printing current joint positions after movement :")
#     for p in new_joint_positions: rospy.loginfo(p)
#     return success

#   def get_cartesian_pose(self):
#     arm_group = self.arm_group

#     # Get the current pose and display it
#     pose = arm_group.get_current_pose()
#     rospy.loginfo("Actual cartesian pose is : ")
#     rospy.loginfo(pose.pose)

#     return pose.pose

#   def reach_cartesian_pose(self, pose, tolerance, constraints):
#     arm_group = self.arm_group
    
#     # Set the tolerance
#     arm_group.set_goal_position_tolerance(tolerance)

#     # Set the trajectory constraint if one is specified
#     if constraints is not None:
#       arm_group.set_path_constraints(constraints)

#     # Get the current Cartesian Position
#     arm_group.set_pose_target(pose)

#     # Plan and execute
#     rospy.loginfo("Planning and going to the Cartesian Pose")
#     return arm_group.go(wait=True)

#   def reach_gripper_position(self, relative_position):
#     gripper_group = self.gripper_group
    
#     # We only have to move this joint because all others are mimic!
#     gripper_joint = self.robot.get_joint(self.gripper_joint_name)
#     gripper_max_absolute_pos = gripper_joint.max_bound()
#     gripper_min_absolute_pos = gripper_joint.min_bound()
#     try:
#       val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
#       return val
#     except:
#       return False 