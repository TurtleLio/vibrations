#! /usr/bin/env python
import rospy

import actionlib
from geometry_msgs.msg import WrenchStamped
from learning_pkg.msg import MeanActionFeedback, MeanActionResult, MeanActionAction
from learning_pkg.srv import SensorMeanMessage, SensorMeanMessageResponse

class MyActionServer(object):
    def __init__(self):
        self._as = actionlib.SimpleActionServer('/action_mean_server',MeanActionAction,execute_cb=self.execute_callback,auto_start=False)
        self._as.start()

    def execute_callback(self, goal):
        # Perform some action here based on the goal received
        feedback=MeanActionFeedback()
        while not rospy.is_shutdown():
            mean = mean_service()
            mean_arr.append(mean)
            if(len(mean_arr)>1):
                diff_arr.append(mean_arr[-2]-mean_arr[-1])
            feedback.current_mean_diff=diff_arr[-1]
            self._as.publish_feedback(feedback)
        result = MeanActionResult()
        # Set the result of the action
        result.success = True
        # Publish the result
        self._as.set_succeeded(result)

# if __name__ == '__main__':
#     temp_array=[]
#     mean_arr=[]
#     diff_arr=[]
#     rospy.init_node('action_mean')
#     mean_service = rospy.ServiceProxy('/sensor_mean', SensorMeanMessage)
#     rospy.wait_for_service('/sensor_mean')
#     server = MyActionServer()
#     rospy.spin()


temp_array=[]
mean_arr=[]
diff_arr=[]
rospy.init_node('action_mean')
mean_service = rospy.ServiceProxy('/sensor_mean', SensorMeanMessage)
rospy.wait_for_service('/sensor_mean')
server = MyActionServer()
rospy.spin()


# class MeanClass():
#   # create messages that are used to publish feedback/result
#     _feedback = FibonacciFeedback()
#     _result   = FibonacciResult()

#     def __init__(self):
#     # creates the action server
#         self._as = actionlib.SimpleActionServer("fibonacci_as", MeanAction, self.goal_callback, False)
#         self._as.start()
    
#   def goal_callback(self, goal):
#     # this callback is called when the action server is called.
#     # this is the function that computes the Fibonacci sequence
#     # and returns the sequence to the node that called the action server
    
#     # helper variables
#     r = rospy.Rate(1)
#     success = True
    
#     # append the seeds for the fibonacci sequence
#     self._feedback.sequence = []
#     self._feedback.sequence.append(0)
#     self._feedback.sequence.append(1)
    
#     # publish info to the console for the user
#     rospy.loginfo('"fibonacci_as": Executing, creating fibonacci sequence of order %i with seeds %i, %i' % ( goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))
    
#     # starts calculating the Fibonacci sequence
#     fibonacciOrder = goal.order
#     for i in range(1, fibonacciOrder):
    
#       # check that preempt (cancelation) has not been requested by the action client
#       if self._as.is_preempt_requested():
#         rospy.loginfo('The goal has been cancelled/preempted')
#         # the following line, sets the client in preempted state (goal cancelled)
#         self._as.set_preempted()
#         success = False
#         # we end the calculation of the Fibonacci sequence
#         break
      
#       # builds the next feedback msg to be sent
#       self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])
#       # publish the feedback
#       self._as.publish_feedback(self._feedback)
#       # the sequence is computed at 1 Hz frequency
#       r.sleep()
    
#     # at this point, either the goal has been achieved (success==true)
#     # or the client preempted the goal (success==false)
#     # If success, then we publish the final result
#     # If not success, we do not publish anything in the result
#     if success:
#       self._result.sequence = self._feedback.sequence
#       rospy.loginfo('Succeeded calculating the Fibonacci of order %i' % fibonacciOrder )
#       self._as.set_succeeded(self._result)
      
# if __name__ == '__main__':
#   rospy.init_node('mean_action')
#   MeanClass()
#   rospy.spin()