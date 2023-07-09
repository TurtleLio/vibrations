#! /usr/bin/env python

#import time
import rospy
from geometry_msgs.msg import WrenchStamped
from learning_pkg.srv import SensorMeanMessage, SensorMeanMessageResponse

# you import the service message python classes generated from Empty.srv.

def callback(data, temp_array):
    temp_array.append(data.wrench.force.z)

def my_callback(request):
    temp_array=[]
    subscriber = rospy.Subscriber("/bus0/ft_sensor0/ft_sensor_readings/wrench", WrenchStamped, callback,temp_array)
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < 1:
        rospy.sleep(0.1)
    subscriber.unregister()
    sum=0
    for i in range(len(temp_array)):
        sum = sum+temp_array[i]
    mean = sum/(len(temp_array))
    rospy.loginfo("Finished service service_sensor_mean")
    return SensorMeanMessageResponse(mean)

temp_array=[]
rospy.init_node('service_sensor_mean')
my_service = rospy.Service('/sensor_mean', SensorMeanMessage, my_callback)
# create the Service called my_service with the defined callback
rate = rospy.Rate(1)
rospy.loginfo("Service /service_sensor_mean Ready")
rospy.spin()  # maintain the service open.
