from __future__ import print_function

import yaml

from math import pi

import rospy
import actionlib
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import quaternion_from_euler

from waypoint_dispatcher.msg import FollowWaypointsAction, FollowWaypointsGoal


def load_route(file_name):
    rospy.loginfo("Loading route")
    with open(file_name, "r") as yaml_file:
        file_contents = yaml.load(yaml_file, Loader=yaml.FullLoader)

    return file_contents


def create_client(service_name="waypoint_dispatcher"):
    # Creates the SimpleActionClient
    client = actionlib.SimpleActionClient(service_name, FollowWaypointsAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for {} server".format(service_name))
    client.wait_for_server()

    return client


def waypoint_client():
    # Creates the SimpleActionClient
    client = actionlib.SimpleActionClient("waypoint_dispatcher", FollowWaypointsAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("Waiting for waypoint_dispatcher server")
    client.wait_for_server()

    waypoint_list = load_route(rospy.get_param("~route_file"))
    goal = create_goal(waypoint_list["waypoints"])

    # Sends the goal to the action server.
    rospy.loginfo("Sending route")
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()


def create_goal(waypoint_list):
    # Creates a goal to send to the action server.
    goal = FollowWaypointsGoal()
    for w in waypoint_list:
        pose = PoseStamped()
        g = MoveBaseActionGoal()

        orientation = [w.get("roll", 0), w.get("pitch", 0), w.get("yaw", 0)]
        orientation = [x * pi / 180.0 for x in orientation]
        yaw_deg = quaternion_from_euler(*orientation, axes="sxyz")
        q = Quaternion(*yaw_deg)
        waypoint = Pose(Point(w["x"], w["y"], w.get("z", 0.0)), q)

        pose.pose = waypoint

        g.goal.target_pose = pose

        goal.goal.append(g)

    return goal
