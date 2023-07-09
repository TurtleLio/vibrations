#!/usr/bin/env python

import rospy
import numpy as np
from numpy import save
from numpy.fft import fft
from geometry_msgs.msg import WrenchStamped
import matplotlib.pyplot as plt

# Define the callback function to be called when a message is received
def callback(data,force_torque):
    #print(data)
    # Extract the 6-dimensional force-torque vector from the message
    data_norm = [data.header.seq , data.wrench.force.x, data.wrench.force.y, data.wrench.force.z, data.wrench.torque.x, data.wrench.torque.y, data.wrench.torque.z]
    # Add the force-torque vector to the dictionary
    force_torque.append(data_norm)

def nice_plot(array_wrench):
        plt.figure()
        plt.plot(array_wrench[:, 0],array_wrench[:, 1], label='Fx')    #without vibrations 0.09 diff, with vibration 0.14 diff
        # plt.plot(array_wrench[:, 0],array_wrench[:, 2], label='Fy')    #without vibrations 0.11 diff ,with vibration 0.12 diff
        # plt.plot(array_wrench[:, 0],array_wrench[:, 3], label='Fz')      #without vibrations 0.10 diff ,with vibration 0.10 diff
        # plt.plot(array_wrench[:, 0],array_wrench[:, 4], label='Tx')
        # plt.plot(array_wrench[:, 0],array_wrench[:, 5], label='Ty')
        # plt.plot(array_wrench[:, 0],array_wrench[:, 6], label='Tz')
        plt.legend()
        plt.title('Force-torque over time')
        plt.xlabel('Time (ms)')
        plt.ylabel('Force/Torque (N/Nm)')
        plt.show()

def nice_plot_fft(array_wrench_fft,freq):
        #N=len(array_wrench_fft[0])
        N=len(array_wrench_fft)
        n=np.arange(N)
        sp=1/400
        T=N/sp
        freq_array=n/T

        #n_oneside = N//2
        #f_oneside = freq_array[:n_oneside]
        plt.figure()
        plt.plot(freq_array,np.abs(array_wrench_fft),'b')
        #plt.plot(freq_array[-300:],array_wrench_fft[1][-300:], label='Fx_fft')
        #plt.plot(array_wrench[:, 0],array_wrench[:, 2], label='Fy')
        #plt.plot(array_wrench[:, 0],array_wrench[:, 3], label='Fz')
        #plt.plot(array_wrench[:, 0],array_wrench[:, 4], label='Tx')
        #plt.plot(array_wrench[:, 0],array_wrench[:, 5], label='Ty')
        #plt.plot(array_wrench[:, 0],array_wrench[:, 6], label='Tz')
        plt.legend()
        plt.title('Force-torque fft')
        plt.xlabel('Freq (Hz)')
        plt.ylabel('Amplitude')
        plt.show()

def make_fft(array_wrench):

     Fx_fft=fft(array_wrench[:,1])
     Fy_fft=fft(array_wrench[:,2])
     Fz_fft=fft(array_wrench[:,3])
     Tx_fft=fft(array_wrench[:,4])
     Ty_fft=fft(array_wrench[:,5])
     Tz_fft=fft(array_wrench[:,6])
     array_wrench_fft=[array_wrench[:,0],Fx_fft,Fy_fft,Fz_fft,Tx_fft,Ty_fft,Tz_fft]
     return array_wrench_fft

def main():
    # Initialize the node
    rospy.init_node('ft_sensor_reader', anonymous=True)

    # Create a dictionary to store the force-torque vectors
    force_torque = []

    # Subscribe to the "/bus0/ft_sensor0/ft_sensoreadings/wrench" topic
    rospy.Subscriber("/bus0/ft_sensor0/ft_sensor_readings/wrench", WrenchStamped, callback, force_torque)

    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < 10:
        rospy.sleep(0.1)
    #print(force_torque)
    force_torque = np.array(force_torque)
    #nice_plot(force_torque)
    #print(force_torque[:, 3])
    save('force_torque_10sec.npy',force_torque)

    freq = 400
    force_torque_fft=make_fft(force_torque)
    nice_plot_fft(force_torque_fft[1],freq)





if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass