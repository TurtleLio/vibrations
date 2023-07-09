import rospy
import actionlib
from learning_pkg.msg import ContinuousMoveActionAction, ContinuousMoveActionGoal, ContinuousMoveActionFeedback, ContinuousMoveActionResult

def action_client():
    # Create an instance of the SimpleActionClient
    client = actionlib.SimpleActionClient('/action_continuous_move_server', ContinuousMoveActionAction)

    # Wait for the action server to start
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()

    # Create a goal message
    goal = ContinuousMoveActionGoal()
    goal.move=1
    # Set goal parameters if required

    # Send the goal to the action server
    rospy.loginfo("Sending goal...")
    client.send_goal(goal)

    # Wait for the action to complete or be preempted
    rospy.loginfo("Waiting for result...")
    client.wait_for_result()

    # Get the result of the action
    result = client.get_result()

    # Process the result
    if result.success:
        rospy.loginfo("Action completed successfully: %s", result.message)
    else:
        rospy.loginfo("Action failed: %s", result.message)

# if __name__ == '__main__':
#     rospy.init_node('continuous_move_client')
#     action_client()    
#     while not rospy.is_shutdown():
#         rospy.spin()

rospy.init_node('continuous_move_client')
rospy.loginfo("init node start")
action_client()    
while not rospy.is_shutdown():
    rospy.spin()