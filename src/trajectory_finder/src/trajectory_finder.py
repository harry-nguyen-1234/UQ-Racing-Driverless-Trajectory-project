#!/usr/bin/env python
import rospy
from fssim_messages.msg import Map, Cone
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
import numpy as np

blue_x_coords = []
blue_y_coords = []
yellow_x_coords = []
yellow_y_coords = []

def callback(data):
    for blue_cones in data.cones_blue:
        blue_x_coords.append(blue_cones.position.x)
        blue_y_coords.append(blue_cones.position.y)

    for yellow_cones in data.cones_yellow:
        yellow_x_coords.append(yellow_cones.position.x)
        yellow_y_coords.append(yellow_cones.position.y)

    spline_resolution = len(blue_x_coords) * len(yellow_x_coords)

    # Create a spline interpolation for the blue cones.
    tck, u = splprep([blue_x_coords, blue_y_coords], s=0)
    u_new = np.linspace(u.min(), u.max(), spline_resolution)
    spline_blue_x, spline_blue_y = splev(u_new, tck)
    plt.plot(spline_blue_x, spline_blue_y, c='#0000ff')

    # Create a spline interpolation for the yellow cones.
    tck, u = splprep([yellow_x_coords, yellow_y_coords], s=0)
    u_new = np.linspace(u.min(), u.max(), spline_resolution)
    spline_yellow_x, spline_yellow_y = splev(u_new, tck)
    plt.plot(spline_yellow_x, spline_yellow_y, c='#ffcc00')

    # Loop over the spline data points
    for i in range(0, len(spline_blue_x), len(blue_x_coords)):
        x = [
            spline_blue_x[i], spline_yellow_x[i]
        ]
        y = [
            spline_blue_y[i], spline_yellow_y[i]
        ]
        # Draw a line between two points along each spline.
        # Draw the midpoint of two points along each spline.
        plt.plot(x, y, c='#ff0000')
        plt.scatter(np.mean(x), np.mean(y), c='#ff00ff')

    # Draw the original raw data.
    plt.scatter(blue_x_coords, blue_y_coords, c='#000000')
    plt.scatter(yellow_x_coords, yellow_y_coords, c='#000000')
    plt.axes().set_aspect('equal')
    plt.grid(True)
    plt.show()
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chicken/map", Map, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()