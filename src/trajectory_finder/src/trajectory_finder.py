#!/usr/bin/env python
import rospy
from fssim_messages.msg import Map, Cone
from geometry_msgs.msg import PolygonStamped, Point32
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import numpy as np

publisher = rospy.Publisher("chicken/trajectory", PolygonStamped, queue_size=1)

def callback(data):
    blue_x_coords = []
    blue_y_coords = []
    yellow_x_coords = []
    yellow_y_coords = []
    middle_points = []
    trajectory_polygon = PolygonStamped()

    for blue_cones in data.cones_blue:
        blue_x_coords.append(blue_cones.position.x)
        blue_y_coords.append(blue_cones.position.y)
    blue_coords = zip(blue_x_coords, blue_y_coords)

    for yellow_cones in data.cones_yellow:
        yellow_x_coords.append(yellow_cones.position.x)
        yellow_y_coords.append(yellow_cones.position.y)
    yellow_coords = zip(yellow_x_coords, yellow_y_coords)

    tree = KDTree(blue_coords)
    indexes_of_closest = tree.query(yellow_coords)[1]

    for count, value in enumerate(indexes_of_closest):
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

        middle_points.append(Point32(np.mean(x), np.mean(y), 0))

    trajectory_polygon.polygon.points = middle_points
    trajectory_polygon.header.frame_id = "map"
    trajectory_polygon.header.stamp = rospy.get_rostime()
    publisher.publish(trajectory_polygon)

if __name__ == '__main__':

    rospy.init_node('trajectory_finder', anonymous=True)
    rospy.Subscriber("chicken/map", Map, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()