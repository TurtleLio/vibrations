#!/usr/bin/env python

import rospy
import numpy as np
from numpy import save
from numpy.fft import fft
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt

# Define the callback function to be called when a message is received
def callback(data,force_torque):
    if len(force_torque) != 0:
        new_force_x = float("{:.2f}".format(data.wrench.force.x - force_torque[-1][1]))
        new_force_y = float("{:.2f}".format(data.wrench.force.y - force_torque[-1][2]))
        new_force_z = float("{:.2f}".format(data.wrench.force.z - force_torque[-1][3]))
        new_torque_x = float("{:.2f}".format(data.wrench.torque.x - force_torque[-1][4]))
        new_torque_y = float("{:.2f}".format(data.wrench.torque.y - force_torque[-1][5]))
        new_torque_z = float("{:.2f}".format(data.wrench.torque.z - force_torque[-1][6]))

    else:
        new_force_x = float("{:.2f}".format(data.wrench.force.x))
        new_force_y = float("{:.2f}".format(data.wrench.force.y))
        new_force_z = float("{:.2f}".format(data.wrench.force.z))
        new_torque_x = float("{:.2f}".format(data.wrench.torque.x))
        new_torque_y = float("{:.2f}".format(data.wrench.torque.y))
        new_torque_z = float("{:.2f}".format(data.wrench.torque.z))
    # Extract the 6-dimensional force-torque vector from the message
    #data_norm = [data.header.seq , float("{:.2f}".format(data.wrench.force.x)), float("{:.2f}".format(data.wrench.force.y)), float("{:.2f}".format(data.wrench.force.z)), float("{:.2f}".format(data.wrench.torque.x)), float("{:.2f}".format(data.wrench.torque.y)), float("{:.2f}".format(data.wrench.torque.z))]
    data_norm = [data.header.seq , data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    #data_norm = [data.header.seq , new_force_x, new_force_y, new_force_z, new_torque_x, new_torque_y, new_torque_z]
    # Add the force-torque vector to the dictionary
    force_torque.append(data_norm)

def nice_plot(array_wrench):
        plt.figure()
        #plt.plot(array_wrench[:, 0],array_wrench[:, 1], label='Fx')    #without vibrations 0.09 diff, with vibration 0.14 diff
        # plt.plot(array_wrench[:, 0],array_wrench[:, 2], label='Fy')    #without vibrations 0.11 diff ,with vibration 0.12 diff
        plt.plot(array_wrench[:, 0],array_wrench[:, 3], label='Fz')      #without vibrations 0.10 diff ,with vibration 0.10 diff
        # plt.plot(array_wrench[:, 0],array_wrench[:, 4], label='Tx')
        # plt.plot(array_wrench[:, 0],array_wrench[:, 5], label='Ty')
        # plt.plot(array_wrench[:, 0],array_wrench[:, 6], label='Tz')
        plt.legend()
        plt.title('Force-torque over time')
        plt.xlabel('Time (ms)')
        plt.ylabel('Force/Torque (N/Nm)')
        plt.show()

def rearrange(arr,index):
    new_arr=[]
    for i in range(len(arr)):
        if (i%index)==0:
            new_arr.append(arr[i])
    for i in range(1,len(new_arr)):
        for j in range(1,7):
            new_arr[i][j]=new_arr[i][j]-new_arr[i-1][j]
    return new_arr

def main():
    # Initialize the node
    rospy.init_node('ft_sensor_reader', anonymous=True)

    # Create a dictionary to store the force-torque vectors
    force_torque = []
    # Subscribe to the "/bus0/ft_sensor0/ft_sensoreadings/wrench" topic
    reading_index=rospy.Subscriber("/bus0/ft_sensor0/ft_sensor_readings/wrench", WrenchStamped, callback, force_torque)

    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < 5:
        rospy.sleep(0.1)
    #print(force_torque)
    index_number=6
    force_torque=rearrange(force_torque,index_number)
    force_torque = np.array(force_torque)
    print(force_torque[:, 3])
    nice_plot(force_torque)

    






if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass