# You can use this file to plot the loged sensor data
# Note that you need to modify/adapt it to your own files
# Feel free to make any modifications/additions here

import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader

def plot_imu(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    fig, ax1 = plt.subplots()
    fig.set_size_inches(8,6)
    
    ax1.set_xlabel("Timestamp (sec)")
    ax1.set_ylabel("Linear Acceleration (m/s\u00b2)")
    ax2 = ax1.twinx()
    ax2.set_ylabel("Angular Velocity (Rad/s)")
    
    for val in values:
        time_list.append((val[-1] - first_stamp)/1e9) #convert from nano to seconds

    
    legend = ["X Acceleration", "Y Acceleration", "Angular Velocity"]
    for i in range(0, len(headers) - 2):
        ax1.plot(time_list, [lin[i] for lin in values], label= legend[i])
        
        
    for i in range(len(headers) - 2, len(headers) - 1):
        ax2.plot(time_list, [lin[i] for lin in values], 'g-', label= legend[i])
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    plt.title(filename[:-4])
   
    ax1.legend(loc='upper left')
    ax2.legend(loc='upper right')
    ax1.grid(True)
    plt.savefig(filename[:-4]+".svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
    plt.show()
    
def vel_to_pos(v_x, v_y, w_z, time_stamps):

    vx = np.array(v_x)
    vy = np.array(v_y)
    wz = np.array(w_z)
    timestamps = np.array(time_stamps)
    
    # Initialize position and orientation
    x, y, theta = [0.0], [0.0], [0.0]

    for i in range(1, len(timestamps)):
        dt = timestamps[i] - timestamps[i-1]
        
        # Update orientation
        theta_new = theta[-1] + wz[i-1] * dt
        
        # Update position (convert from robot to world frame)
        dx = (vx[i-1] * np.cos(theta[-1]) - vy[i-1] * np.sin(theta[-1])) * dt
        dy = (vx[i-1] * np.sin(theta[-1]) + vy[i-1] * np.cos(theta[-1])) * dt
        
        x.append(x[-1] + dx)
        y.append(y[-1] + dy)
        theta.append(theta_new)
    return x,y,theta
    
def plot_odom(filename):
    
    headers, values=FileReader(filename).read_file() 
    time_list=[]
    first_stamp=values[0][-1]
    
    fig, ax1 = plt.subplots(1,2)
    fig.set_size_inches(12,6)
    
    ax1[0].set_xlabel("Timestamp (sec)")
    ax1[0].set_ylabel("Linear Position (m)")
    ax2 = ax1[0].twinx()
    ax2.set_ylabel("Angular Position (Rad)")
    
    ax1[1].set_xlabel("X Position (m)")
    ax1[1].set_ylabel("Y Position (m)")
    
    for val in values:
        time_list.append((val[-1] - first_stamp)/1e9) #convert from nano to seconds
    
    dim = [0,0,0]
    for i in range(0, len(headers)-1):
        dim[i] = [lin[i] for lin in values]
    
    x, y, theta = vel_to_pos(dim[0], dim[1], dim[2], time_list)
    # x = dim[0]
    # y = dim[1]
    # theta = dim[2]

    # for i in range(0, len(headers) - 2):
    #     ax1[0].plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    #     dim[i] = [lin[i] for lin in values]
        
    # for i in range(len(headers) - 2, len(headers) - 1):
    #     ax2.plot(time_list, [lin[i] for lin in values], 'g-', label= headers[i]+ " linear")
    
    #plt.plot([lin[0] for lin in values], [lin[1] for lin in values])
    legend = ["X Position", "Y Position", "Theta"]
    ax1[0].plot(time_list, x, label= legend[0])
    ax1[0].plot(time_list, y, label= legend[1])
    ax2.plot(time_list, theta, 'g-', label= legend[2]) 
    
    ax1[0].set_title(filename[:-4], loc='center')
    
    ax1[1].set_title("Odometry X/Y Position Plot")
   
    ax1[1].plot(x, y, 'b-')
    ax1[0].legend(loc='upper left')
    ax2.legend(loc='upper right')
    ax1[0].grid(True)
    ax1[1].grid(True)
    plt.tight_layout()
    plt.savefig(filename[:-4]+".svg", bbox_inches='tight', pad_inches=0.1, dpi=300)
    plt.show()
    
import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--imu_files', nargs='+', required=False, help='List of files to process')
    parser.add_argument('--odom_files', nargs='+', required=False, help='List of files to process')
    
    args = parser.parse_args()
    
    # print("plotting the files", args.files)

    imu_files=args.imu_files
    if imu_files:
        for filename in imu_files:
            plot_imu(filename)
        
    odom_files = args.odom_files
    
    if odom_files:
        for filename in odom_files:
            plot_odom(filename)
