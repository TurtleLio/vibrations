#! /usr/bin/env python

#import time
import rospy
import actionlib
from learning_pkg.msg import MeanActionFeedback, MeanActionResult, MeanActionAction, MeanActionGoal
from learning_pkg.msg import ContinuousMoveActionFeedback, ContinuousMoveActionResult, ContinuousMoveActionAction, ContinuousMoveActionGoal
import numpy as np
from kinova_class import ExampleMoveItTrajectories
from math import pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion

class MeanPosePoseActionServer(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer('/action_continuous_move_server',ContinuousMoveActionAction,execute_cb=self.execute_callback,auto_start=False)
        self._as.start()

    def execute_callback(self, goal):
        # Perform some action here based on the goal received
        moving_force_diff = goal.mean_diff_goal
        mean_diff_arr=[]
        #rospy.init_node('mean_action_client')
        #rospy.init_node('service_move_command')
        #my_service = rospy.Service('/move_command', Quaternion, my_callback)
        result = self.action_client(moving_force_diff)
        #result = MyActionResult()
        # Set the result of the action
        self._as.set_succeeded(result)

    def feedback_callback(self,feedback):
        mean_diff_arr.append(feedback)

    def action_client(self, moving_force_diff):
        mean_client=actionlib.SimpleActionClient("/action_mean_server", MeanActionAction,feedback_cb=self.feedback_callback)
        rospy.loginfo("Waiting for action server...")
        mean_client.wait_for_server()
        mean_goal = MeanActionGoal()
        mean_goal.mean_diff_goal=moving_force_diff
        rospy.loginfo("Sending goal mean...")
        mean_client.send_goal(mean_goal)
        if len(mean_diff_arr) > 0: 
            while mean_diff_arr[-1] < 0.1:  # shoule be action from point 1-2, action from point 2-3, action mean
                print(mean_diff_arr[-1])
                #pose1=pose
                #pose2=pose1
                #pose2.position.z=pose2.position.z-0.0001
                #action_pose1(pose1)
                #action_pose2(pose2)
                #action_mean()

        rospy.loginfo("Finished action")
        success = 1
        return ContinuousMoveActionResult(success)

# if __name__ == '__main__':
#     # moving_force_diff = 0.1
#     # mean_diff_arr=[]
#     # rospy.init_node('mean_pose_pose_client')
#     # rospy.init_node('service_move_command')
#     #my_service = rospy.Service('/move_command', Quaternion, my_callback)
#     # action_client()
#     # create the Service called my_service with the defined callback
#     mean_diff_arr=[]
#     rospy.init_node('action_continuous_move')
#     server = MeanPosePoseActionServer()
#     rate = rospy.Rate(1)
#     rospy.loginfo("Service /service_move_command Ready")
#     rospy.spin()  # maintain the service open.

    # moving_force_diff = 0.1
    # mean_diff_arr=[]
    # rospy.init_node('mean_pose_pose_client')
    # rospy.init_node('service_move_command')
    #my_service = rospy.Service('/move_command', Quaternion, my_callback)
    # action_client()
    # create the Service called my_service with the defined callback
mean_diff_arr=[]
rospy.init_node('action_continuous_move')
server = MeanPosePoseActionServer()
rate = rospy.Rate(1)
rospy.loginfo("Service /service_move_command Ready")
rospy.spin()  # maintain the service open.
