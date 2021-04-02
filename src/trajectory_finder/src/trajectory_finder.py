#!/usr/bin/env python
import rospy
from fssim_messages.msg import Map, Cone
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import numpy as np

def callback(data):
    blue_x_coords = []
    blue_y_coords = []
    yellow_x_coords = []
    yellow_y_coords = []

    for blue_cones in data.cones_blue:
        blue_x_coords.append(blue_cones.position.x)
        blue_y_coords.append(blue_cones.position.y)
    blue_coords = zip(blue_x_coords, blue_y_coords)

    for yellow_cones in data.cones_yellow:
        yellow_x_coords.append(yellow_cones.position.x)
        yellow_y_coords.append(yellow_cones.position.y)
    yellow_coords = zip(yellow_x_coords, yellow_y_coords)

    tree = KDTree(blue_coords)
    indexes = tree.query(yellow_coords)[1]

    for count, value in enumerate(indexes):
        x = [
        blue_x_coords[value],
        yellow_x_coords[count]
        ]

        y = [
        blue_y_coords[value],
        yellow_y_coords[count]
        ]
        plt.plot(x, y, c='#ff0000')
        plt.scatter(np.mean(x), np.mean(y), c='#ff00ff')

    # Draw the original raw data.
    plt.scatter(blue_x_coords, blue_y_coords, c='#0000ff')
    plt.scatter(yellow_x_coords, yellow_y_coords, c='#ffcc00')
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