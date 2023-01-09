#!/usr/bin/env python3

import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Path
from tars.srv import RobotGoalSrvRequest, RobotGoalSrv
from tars.msg import AgentsMsg
from sklearn.preprocessing import normalize
from planner import Planner

class Robot:

    def __init__(self):
        # Launch Params
        self.max_vel = rospy.get_param("~max_linear_vel", default=0.4)
        self.min_vel = rospy.get_param("~min_vel", default=0.1)
        self.frame_id = rospy.get_param("~frame_id", default="map")
        self.base_frame_id = rospy.get_param("~base_frame_id", default="09/base_link")
        self.lookahead = rospy.get_param("~lookahead", default=0.3)
        self.dist_tolerance = rospy.get_param("~dist_tolerance", default=0.1)
        self.robot_id = rospy.get_param("~robot_id", default="9")
        # Publisher topics 
        self.cmd_vel_pub = rospy.Publisher("/tars/09/cmd_vel", Twist, queue_size=10)
        self.marker_pub = rospy.Publisher("~marker_next_point", Marker, queue_size=10, latch=True)
        # Subscriber topics
        rospy.Subscriber("path", Path, self.path_callback)
        rospy.Subscriber("/tars/09/agents_tracking", AgentsMsg, self.agents_callback)
        rospy.Subscriber("/tars/agents", AgentsMsg, self.objetive_tracking)
        # Services
        self.robot_goal_service = rospy.ServiceProxy("/tars/robot_goal", RobotGoalSrv)
        # Global variables
        self.listener = tf.TransformListener()
        self.path = None
        self.current_point = PointStamped()
        self.goal_point = PointStamped()
        self.goal_point_index = 0
        self.last_next_point_index = -1 
        self.robot_data = None
        self.robot_globalForce = None
        self.objetive_found = False
        self.time_objetive_not_found = 0
        self.objetive = None
        self.planner = Planner()
        self.init = True
        self.time_follow_objetive = 0
        self.real_objetive = None

    # Función callback del camino a seguir
    def path_callback(self, path):
        self.path = path
        self.actual_point_index = 0
    
    # Función callback del tracking de personas del robot incluido él mismo
    def agents_callback(self, agents_data):
        for agent in agents_data.agents:
            if agent.id == "09":
                self.robot_data = agent
                self.robot_globalForce = np.array((agent.forces.globalForce.x, agent.forces.globalForce.y))
            if agent.id == "Alumno10":
                self.objetive = agent
                self.objetive_found = True
    
    # Función callback del tracking de objetivo incluido él mismo. Se utiliza en caso de que el objetivo quede ocluido por un periodo de tiempo
    def objetive_tracking(self, agentss_data):
        for agent in agentss_data.agents:
            if agent.id == "Alumno10":
                self.real_objetive = agent

    # Función que simula el comportamiento del robot en cada uno de los casos posibles
    def command(self):
        if self.robot_data is not None:
            if self.objetive_found == True and self.path is None:
                current_robot_pos = np.array((self.robot_data.position.x, self.robot_data.position.y))
                current_objetive_pos = np.array((self.objetive.position.x, self.objetive.position.y))
                self.path = self.planner.find_path(current_robot_pos, current_objetive_pos)
                self.time_objetive_not_found = 0
            elif (self.objetive_found == True and self.path is not None) or (self.objetive_found == False and self.path is not None):
                self.current_point.header.frame_id = self.base_frame_id
                self.current_point.header.stamp = rospy.Time()
                self.current_point.point.x = 0
                self.current_point.point.y = 0
                self.current_point.point.z = 0

                try:
                    global_current_point = self.listener.transformPoint(
                        self.frame_id, self.current_point
                    )
                except (
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                ):
                    rospy.loginfo("Problem TF")
                    return
                
                nearest_point_index = self.calculate_nearest_point(global_current_point)
                next_point_index = self.calculate_next_point(nearest_point_index)

                self.goal_point_index = next_point_index
                base_goal = PointStamped()
                self.goal_point.header.frame_id = self.frame_id
                self.goal_point.header.stamp = rospy.Time()
                self.goal_point.point.x = self.path.poses[
                    self.goal_point_index
                ].pose.position.x
                self.goal_point.point.y = self.path.poses[
                    self.goal_point_index
                ].pose.position.y
                self.goal_point.point.z = 0

                try:
                    base_goal = self.listener.transformPoint(
                        self.base_frame_id, self.goal_point
                    )
                except (
                    tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException,
                ):
                    rospy.loginfo("Problem TF")
                    return
                
                if self.last_next_point_index != next_point_index:
                    robot_goal_request = RobotGoalSrvRequest()
                    robot_goal_request.id = "09"
                    robot_goal_request.gx = self.goal_point.point.x
                    robot_goal_request.gy = self.goal_point.point.y
                    self.robot_goal_service(robot_goal_request)
                
                self.last_next_point_index = next_point_index

                distance = math.sqrt(math.pow(base_goal.point.x, 2) + math.pow(base_goal.point.y, 2))
                angular = math.atan2(base_goal.point.y, base_goal.point.x)
                linear_vel = 0.1 * distance
                angular_vel = 0.5 * angular
                if self.robot_globalForce is not None:
                    v = self.conversion_to_cartesian(linear_vel, angular_vel) + self.robot_globalForce * 0.05
                    linear_vel, angular_vel = self.conversion_to_polar(v)
                    if linear_vel > self.max_vel:
                        linear_vel = self.max_vel
                    print("linear_vel:", linear_vel)
                    print("angular_vel:", angular_vel)
                    self.publish_vel(linear_vel, angular_vel)

                if distance < self.dist_tolerance:
                    self.publish_vel(0, 0)
                    self.path = None
                    self.last_next_point_index = -1

                if self.objetive_found == True and self.path is not None and self.time_follow_objetive < 100:
                    self.time_follow_objetive += 1
                elif self.objetive_found == True:
                    current_robot_pos = np.array((self.robot_data.position.x, self.robot_data.position.y))
                    current_objetive_pos = np.array((self.objetive.position.x, self.objetive.position.y))
                    self.path = self.planner.find_path(current_robot_pos, current_objetive_pos)
                    self.time_objetive_not_found = 0
                
            elif self.objetive_found == False and self.time_objetive_not_found < 400 and self.init == False:
                if self.time_follow_objetive < 100:
                    self.time_follow_objetive += 1
                else:
                    objetive = self.real_objetive
                    current_robot_pos = np.array((self.robot_data.position.x, self.robot_data.position.y))
                    current_objetive_position = np.array((objetive.position.x, objetive.position.y))
                    self.planner.find_path(current_robot_pos, current_objetive_position)

                if self.objetive_found:
                    self.time_objetive_not_found = 0
                else:
                    self.time_objetive_not_found += 1
            elif (self.objetive_found == False and self.time_objetive_not_found == 400) or self.init == True:
                current_robot_pos = np.array((self.robot_data.position.x, self.robot_data.position.y))
                self.planner.find_random_path(current_robot_pos)
                self.init = False
                self.time_objetive_not_found = 0

    # Función para convertir coordenadas polares en cartesianas
    def conversion_to_cartesian(self, linear_vel, angular_vel):
        v_x = linear_vel * np.cos(angular_vel)
        v_y = linear_vel * np.sin(angular_vel)
        v = [v_x, v_y]
        return v

    # Función para convertir coordenadas cartesianas en polares
    def conversion_to_polar(self, v):
        lineal_vel = np.sqrt(np.power(v[0], 2) + np.power(v[1], 2))
        angular_vel = np.arctan2(v[1], v[0])
        return lineal_vel, angular_vel

    # Función para calcular el punto más cerrcano del camino al robot
    def calculate_nearest_point(self, global_current_point):
        distances = []
        for i in range(0, len(self.path.poses)):
            dist = math.sqrt(
                math.pow(
                    self.path.poses[i].pose.position.x - global_current_point.point.x, 2
                )
                + math.pow(
                    self.path.poses[i].pose.position.y - global_current_point.point.y, 2
                )
            )
            distances.append(dist)
        min_dist_index = np.argmin(distances, axis=0)
        return min_dist_index
    
    # Función para calcular el siguiente punto a visitar del camino 
    def calculate_next_point(self, nearest_point_index):
        next_point_index = nearest_point_index
        stop = False
        nearest_point = PointStamped()
        nearest_point.header.frame_id = self.frame_id
        nearest_point.header.stamp = rospy.Time()
        nearest_point = self.path.poses[nearest_point_index]
        next_point = PointStamped()
        next_point.header.frame_id = self.frame_id
        next_point.header.stamp = rospy.Time()
        while (next_point_index < len(self.path.poses)) and (not stop):
            dist = math.sqrt(
                math.pow(
                    self.path.poses[next_point_index].pose.position.x
                    - nearest_point.pose.position.x,
                    2,
                )
                + math.pow(
                    self.path.poses[next_point_index].pose.position.y
                    - nearest_point.pose.position.y,
                    2,
                )
            )
            if dist >= self.lookahead:
                stop = True
            else:
                next_point_index += 1

        if next_point_index > len(self.path.poses) - 1:
            next_point.point.x = self.path.poses[
                len(self.path.poses) - 1
            ].pose.position.x
            next_point.point.y = self.path.poses[
                len(self.path.poses) - 1
            ].pose.position.y
            next_point_index = len(self.path.poses) - 1

        if next_point_index < self.goal_point_index:
            next_point_index = self.goal_point_index

        next_point.point.x = self.path.poses[next_point_index].pose.position.x
        next_point.point.y = self.path.poses[next_point_index].pose.position.y
        self.publish_marker(next_point.point)
        return next_point_index

    # Función para publicar el marcador del punto a visitar del camino 
    def publish_marker(self, next_point):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()

        marker.id = 0
        marker.ns = "scan_ds"
        marker.lifetime.secs = 100

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.type = marker.POINTS
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.b = 0.0
        marker.color.g = 0.0
        marker.points.append(next_point)
        self.marker_pub.publish(marker)

    # Función para publicar la velocidad que debe de seguir al robot
    def publish_vel(self, linear_vel, angular_vel):
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel
        self.cmd_vel_pub.publish(cmd_vel)

if __name__ == "__main__":
    rospy.init_node("control_node")
    robot = Robot()
    r =rospy.Rate(20)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        robot.command()
        r.sleep