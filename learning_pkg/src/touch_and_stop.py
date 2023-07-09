#!/usr/bin/env python

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import rospy
import numpy as np
from kinova_class import ExampleMoveItTrajectories
from math import pi
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt
from learning_pkg.srv import SensorMeanMessage, SensorMeanMessageRequest, MoveCommandMessage, MoveCommandMessageRequest
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler


#---------------------------------------------------------Most of the code was moved to service move command--------------------------------------------
# def move_down():
#     #example = ExampleMoveItTrajectories()
#     #example.init_revolving_path_constraints()
#     #example.enable_revolving_path_constraints()
#   # For testing purposes
#     success = example.is_init_success
#     try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#     except:
#       pass

#     if success:
#         rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     example.init_revolving_path_constraints(actual_pose)
#     example.enable_revolving_path_constraints()

#     actual_pose.position.z = actual_pose.position.z-0.0001
#     actual_pose.orientation.x = actual_pose.orientation.x
#     actual_pose.orientation.y = actual_pose.orientation.y
#     actual_pose.orientation.z = actual_pose.orientation.z
#     actual_pose.orientation.w = actual_pose.orientation.w
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.00005, constraints=example.not_revolving)
    
#     rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

#     if not success:
#       rospy.logerr("The example encountered an error.")
#     return

# def move_down_some():
#     #example = ExampleMoveItTrajectories()
#     #example.init_revolving_path_constraints()
#     #example.enable_revolving_path_constraints()

#   # For testing purposes
#     success = example.is_init_success
#     try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#     except:
#       pass

#     if success:
#         rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     example.init_revolving_path_constraints(actual_pose)
#     example.enable_revolving_path_constraints()

#     actual_pose.position.z = actual_pose.position.z - 0.0005
#     actual_pose.orientation.x = actual_pose.orientation.x
#     actual_pose.orientation.y = actual_pose.orientation.y
#     actual_pose.orientation.z = actual_pose.orientation.z
#     actual_pose.orientation.w = actual_pose.orientation.w
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.00005, constraints=example.not_revolving)
    
#     rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

#     if not success:
#       rospy.logerr("The example encountered an error.")
#     return

# def move_down_alot():
#     #example = ExampleMoveItTrajectories()
#     #example.init_revolving_path_constraints()
#     #example.enable_revolving_path_constraints()

#   # For testing purposes
#     success = example.is_init_success
#     try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#     except:
#       pass

#     if success:
#         rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     example.init_revolving_path_constraints(actual_pose)
#     example.enable_revolving_path_constraints()
#     actual_pose_orien_array=np.array([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
#     pose_angles=qua_to_eul(actual_pose_orien_array)
#     print(pose_angles)

#     actual_pose.position.z = actual_pose.position.z - 0.005
#     actual_pose.orientation.x = actual_pose.orientation.x
#     actual_pose.orientation.y = actual_pose.orientation.y
#     actual_pose.orientation.z = actual_pose.orientation.z
#     actual_pose.orientation.w = actual_pose.orientation.w
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.0005, constraints=example.not_revolving)
    
#     rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

#     if not success:
#       rospy.logerr("The example encountered an error.")
#     return

# def move_down_alot_more():
#     #example = ExampleMoveItTrajectories()
#     #example.init_revolving_path_constraints()
#     #example.enable_revolving_path_constraints()

#   # For testing purposes
#     success = example.is_init_success
#     try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#     except:
#       pass

#     if success:
#         rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     example.init_revolving_path_constraints(actual_pose)
#     example.enable_revolving_path_constraints()

#     actual_pose.position.z = actual_pose.position.z - 0.01
#     actual_pose.orientation.x = actual_pose.orientation.x
#     actual_pose.orientation.y = actual_pose.orientation.y
#     actual_pose.orientation.z = actual_pose.orientation.z
#     actual_pose.orientation.w = actual_pose.orientation.w
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.001, constraints=example.not_revolving)
    
#     rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

#     if not success:
#       rospy.logerr("The example encountered an error.")
#     return

# def move_up(pitch):
#     #example = ExampleMoveItTrajectories()
#     #example.init_revolving_path_constraints()
#     #example.enable_revolving_path_constraints()

#   # For testing purposes
#     success = example.is_init_success
#     try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#     except:
#       pass

#     if success:
#         rospy.loginfo("Reaching Cartesian Pose...")
    
#     actual_pose = example.get_cartesian_pose()
#     example.init_revolving_path_constraints(actual_pose)
#     example.enable_revolving_path_constraints()
#     #print("actual pose:")
#     #print(actual_pose)
#     actual_pose_orien_array=np.array([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orienta1tion.w])
#     pose_angles=qua_to_eul(actual_pose_orien_array)
#     print(pose_angles)
#     pose_angles[1]=pitch
#     pose_angles_rad=np.array([pose_angles[0]*pi/180,pose_angles[1]*pi/180,pose_angles[2]*pi/180])
#     new_quaternion=eul_to_qua(pose_angles_rad)
#     actual_pose.position.z = actual_pose.position.z+0.011
#     # actual_pose.orientation.x = new_quaternion[0]
#     # actual_pose.orientation.y = new_quaternion[1]
#     # actual_pose.orientation.z = new_quaternion[2]
#     # actual_pose.orientation.w = new_quaternion[3]
#     actual_pose.orientation.x = actual_pose.orientation.x
#     actual_pose.orientation.y = actual_pose.orientation.y
#     actual_pose.orientation.z = pitch

#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.001, constraints=example.not_revolving)
    
#     rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

#     if not success:
#       rospy.logerr("The example encountered an error.")
#     return

# def choose_angle():
#   print("choose angle (1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17)")
#   choise_2 = int(input())
#   if choise_2 == 1:
#     pitch=0.00
#     move_up(pitch)
#   if choise_2 == 2:
#     pitch=0.015
#     move_up(pitch)
#   if choise_2 == 3:
#     pitch=0.03
#     move_up(pitch) 
#   if choise_2 == 4:
#     pitch=0.045
#     move_up(pitch)
#   if choise_2 == 5:
#     pitch=0.06
#     move_up(pitch)
#   if choise_2 == 6:
#     pitch=0.075
#     move_up(pitch)
#   if choise_2 == 7:
#     pitch=0.09
#     move_up(pitch)
#   if choise_2 == 8:
#     pitch=0.1
#     move_up(pitch) 
#   if choise_2 == 9:
#     pitch=0.115
#     move_up(pitch) 
#   if choise_2 == 10:
#     pitch=0.130
#     move_up(pitch) 
#   if choise_2 == 11:
#     pitch=0.145
#     move_up(pitch) 
#   if choise_2 == 12:
#     pitch=0.160
#     move_up(pitch) 
#   if choise_2 == 13:
#     pitch=0.175
#     move_up(pitch) 
#   if choise_2 == 14:
#     pitch=0.190
#     move_up(pitch) 
#   if choise_2 == 15:
#     pitch=0.2
#     move_up(pitch) 
#   if choise_2 == 16:
#     pitch=0.215
#     move_up(pitch) 
#   if choise_2 == 17:
#     pitch=0.23
#     move_up(pitch) 

# def choose_angle_reverse():
#   print("choose angle (1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17)")
#   choise_2 = int(input())
#   if choise_2 == 1:
#     pitch=0.00
#     move_up(pitch)
#   if choise_2 == 2:
#     pitch=-3.5
#     move_up(pitch)
#   if choise_2 == 3:
#     pitch=-7
#     move_up(pitch) 
#   if choise_2 == 4:
#     pitch=-10.5
#     move_up(pitch)
#   if choise_2 == 5:
#     pitch=-14
#     move_up(pitch)
#   if choise_2 == 6:
#     pitch=-17.5
#     move_up(pitch)
#   if choise_2 == 7:
#     pitch=-21
#     move_up(pitch)
#   if choise_2 == 8:
#     pitch=-24.5
#     move_up(pitch) 
#   if choise_2 == 9:
#     pitch=-28
#     move_up(pitch) 
#   if choise_2 == 10:
#     pitch=-31.5
#     move_up(pitch) 
#   if choise_2 == 11:
#     pitch=-35
#     move_up(pitch) 
#   if choise_2 == 12:
#     pitch=-38.5
#     move_up(pitch) 
#   if choise_2 == 13:
#     pitch=-42
#     move_up(pitch) 
#   if choise_2 == 14:
#     pitch=-45.5
#     move_up(pitch) 
#   if choise_2 == 15:
#     pitch=-49
#     move_up(pitch) 
#   if choise_2 == 16:
#     pitch=-52.5
#     move_up(pitch) 
#   if choise_2 == 17:
#     pitch=-56
#     move_up(pitch) 

# def qua_to_eul(quaternion):
#    x,y,z,w = quaternion[0],quaternion[1],quaternion[2],quaternion[3]
#    t0= +2.0*(w*x+y*z)
#    t1= +1.0-2.0*(x*x+y*y)
#    roll_x=np.arctan2(t0,t1)
#    t2= +2.0*(w*y-z*x)
#    t2= +1.0 if t2>+1.0 else t2
#    t2= -1.0 if t2<-1.0 else t2
#    pitch_y=np.arcsin(t2)
#    t3= +2.0*(w*z+x*y)
#    t4= +1.0-2.0*(y*y+z*z)
#    yaw_z=np.arctan2(t3,t4)
#    eulerAngles=np.array([roll_x*180/pi,pitch_y*180/pi,yaw_z*180/pi])
#    #eulerAngles=np.degrees(np.array([2*np.arctan2(quaternion[0],quaternion[3]),2*np.arcsin(quaternion[1]),2*np.arctan2(quaternion[2],quaternion[3])]))
#    return eulerAngles

# def eul_to_qua(euler):
#    qx=np.sin(euler[0]/2)*np.cos(euler[1]/2)*np.cos(euler[2]/2)-np.cos(euler[0]/2)*np.sin(euler[1]/2)*np.sin(euler[2]/2)
#    qy=np.cos(euler[0]/2)*np.sin(euler[1]/2)*np.cos(euler[2]/2)+np.sin(euler[0]/2)*np.cos(euler[1]/2)*np.sin(euler[2]/2)
#    qz=np.cos(euler[0]/2)*np.cos(euler[1]/2)*np.sin(euler[2]/2)-np.sin(euler[0]/2)*np.sin(euler[1]/2)*np.cos(euler[2]/2)
#    qw=np.cos(euler[0]/2)*np.cos(euler[1]/2)*np.cos(euler[2]/2)+np.sin(euler[0]/2)*np.sin(euler[1]/2)*np.sin(euler[2]/2)
#    quaternions=np.array([qx,qy,qz,qw])
#    return quaternions

# def angle():
#   print("choose angle (1-roll 2-pitch 3-yaw)")
#   choise_2 = int(input())
#   if choise_2 == 1:
#     change_angle(10,0,0)
#   if choise_2 == 2:
#     change_angle(0,10,0)
#   if choise_2 == 3:
#     change_angle(0,0,10)

# def change_angle(roll,pitch,yaw):
#     success = example.is_init_success
#     try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#     except:
#       pass
#     if success:
#         rospy.loginfo("Reaching Cartesian Pose...")
#     actual_pose = example.get_cartesian_pose()
#     example.init_revolving_path_constraints(actual_pose)
#     example.enable_revolving_path_constraints()
#     actual_pose_orien_array=np.array([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
#     pose_angles=qua_to_eul(actual_pose_orien_array)
#     print(pose_angles)
#     pose_angles[0]=pose_angles[0]+roll
#     pose_angles[1]=pose_angles[1]+pitch
#     pose_angles[2]=pose_angles[2]+yaw
#     pose_angles_rad=np.array([pose_angles[0]*pi/180,pose_angles[1]*pi/180,pose_angles[2]*pi/180])
#     new_quaternion=eul_to_qua(pose_angles_rad)
#     actual_pose.position.z = actual_pose.position.z
#     actual_pose.orientation.x = new_quaternion[0]
#     actual_pose.orientation.y = new_quaternion[1]
#     actual_pose.orientation.z = new_quaternion[2]
#     actual_pose.orientation.w = new_quaternion[3]
#     success &= example.reach_cartesian_pose(pose=actual_pose, tolerance=0.001, constraints=example.not_revolving)    
#     rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)
#     if not success:
#       rospy.logerr("The example encountered an error.")
#     return

# def print_angle():
#     success = example.is_init_success
#     try:
#       rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
#     except:
#       pass
#     if success:
#         rospy.loginfo("Reaching Cartesian Pose...")
#     actual_pose = example.get_cartesian_pose()
#     example.init_revolving_path_constraints(actual_pose)
#     example.enable_revolving_path_constraints()
#     actual_pose_orien_array=np.array([actual_pose.orientation.x,actual_pose.orientation.y,actual_pose.orientation.z,actual_pose.orientation.w])
#     pose_angles=qua_to_eul(actual_pose_orien_array)
#     print('angles:')
#     print(pose_angles)
#------------------main------------------
mean_arr = []
diff_arr = []
rospy.init_node('touch_and_stop', anonymous=True)

rospy.wait_for_service('/sensor_mean')
#rospy.wait_for_service('/my_gen3/move_group/trajectory_execution/set_parameters')

service = rospy.ServiceProxy('/sensor_mean', SensorMeanMessage)
service2 = rospy.ServiceProxy('/move_command', MoveCommandMessage)

example = ExampleMoveItTrajectories()

while not rospy.is_shutdown():
  mean_obj=SensorMeanMessageRequest() 
  mean = service()
  print("mean:")
  print(mean)
  mean_arr.append(mean.mean)
  if(len(mean_arr)>1):
    diff_arr.append(mean_arr[-2]-mean_arr[-1])
    # print("different in force:")
    # print(diff_arr[-1])
    print("do you want to lower (1-down/2-down some/3-down alot/4-down alot more/5-up/6-up negetive/7-angle/ 8-print angles)")
    choise = int(input())
    #print(choise)
    success2 = MoveCommandMessageRequest(choise)
#      if choise == 1:
#          move_down()
#      if choise == 2:
#         move_down_some()
#      if choise == 3:
#         move_down_alot()
#      if choise == 4:
#         move_down_alot_more()
#      if choise == 5:
#          choose_angle()
#      if choise == 6:
#          choose_angle_reverse()
#      if choise == 7:
#          angle()
#      if choise == 8:
#          print_angle()          
  print("mean array:")
  print(mean_arr)  
  # print("diff array:")
  # print(diff_arr)
  rospy.sleep(0.1)
    
#rospy.spin()

# def main():
#     # Initialize the node
#     mean_arr = []
#     diff_arr = []
#     rospy.init_node('touch_and_stop', anonymous=True)

#     rospy.wait_for_service('service_sensor_mean')
#     service = rospy.ServiceProxy('service_sensor_mean', SensorMeanMessage)
#     mean = service().result
#     mean_arr.append(mean)
#     if(len(mean_arr)>1):
#         diff_arr.append(mean_arr[-2]-mean_arr[-1])
#         print(diff_arr[-1])
#         choise = input("do you want to lower (y/n)")
#         if choise == "y":
#             move_down()
#     print(diff_arr)
    
#     rospy.spin()

# if __name__ == '__main__':
#   main()
