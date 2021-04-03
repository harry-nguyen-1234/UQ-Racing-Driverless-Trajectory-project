#!/usr/bin/env python
import rospy
from fssim_messages.msg import Map, Cone
from geometry_msgs.msg import PolygonStamped, Point32
from scipy.spatial import KDTree
from scipy.interpolate import splprep, splev
import numpy as np
import matplotlib.pyplot as plt

ENABLE_PLOTTING = False

publisher = None

def find_cone_coords(data):
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

    if ENABLE_PLOTTING:
        # Plot the blue cones in blue and yellow cones in yellow.
        plt.scatter(blue_x_coords, blue_y_coords, c='#0000ff')
        plt.scatter(yellow_x_coords, yellow_y_coords, c='#ffcc00')

    return blue_coords, yellow_coords

def find_middle_points(blue_coords, yellow_coords):
    tree = KDTree(blue_coords)
    _, indexes_of_closest = tree.query(yellow_coords)

    middle_points = []
    for count, value in enumerate(indexes_of_closest):
        x = [
        blue_coords[value][0],
        yellow_coords[count][0]
        ]

        y = [
        blue_coords[value][1],
        yellow_coords[count][1]
        ]

        if ENABLE_PLOTTING:
            # Draw a line between the two sampled cones in red.
            # Draw the midpoint of the line in magenta.
            plt.plot(x, y, c='#ff0000')
            plt.scatter(np.mean(x), np.mean(y), c='#ff00ff')

        middle_points.append([np.mean(x), np.mean(y)])

    if ENABLE_PLOTTING:
        # Draw a line between each midpoint in red.
        x = [pair[0] for pair in middle_points]
        y = [pair[1] for pair in middle_points]
        plt.plot(x, y, c='#ff0000')
    
    return middle_points

def interpolate_middle_points(middle_points):
    middle_x = [pair[0] for pair in middle_points]
    middle_y = [pair[1] for pair in middle_points]

    # Create a spline interpolation 
    scale_factor = 10
    spline_resolution = len(middle_points) * scale_factor
    tck, u = splprep([middle_x, middle_y], s=0)
    u_new = np.linspace(u.min(), u.max(), spline_resolution)
    spline_x, spline_y = splev(u_new, tck)

    if ENABLE_PLOTTING:
        # Draw the spline in green
        plt.plot(spline_x, spline_y, c='#00cc00')

    middle_points_interpolated = []
    for x,y in zip(spline_x, spline_y):
        middle_points_interpolated.append(Point32(x, y, 0))

    return middle_points_interpolated

def callback(data):
    blue_coords, yellow_coords = find_cone_coords(data)
    middle_points = find_middle_points(blue_coords, yellow_coords)
    middle_points_interpolated = interpolate_middle_points(middle_points)

    if ENABLE_PLOTTING:
        plt.axes().set_aspect('equal')
        plt.grid(True)
        plt.show()

    trajectory_polygon = PolygonStamped()
    trajectory_polygon.polygon.points = middle_points_interpolated
    trajectory_polygon.header.frame_id = "map"
    trajectory_polygon.header.stamp = rospy.get_rostime()
    publisher.publish(trajectory_polygon)

if __name__ == '__main__':

    rospy.init_node('trajectory_finder', anonymous=True)
    rospy.Subscriber("chicken/map", Map, callback)
    publisher = rospy.Publisher("chicken/trajectory", PolygonStamped, 
        latch=True, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()