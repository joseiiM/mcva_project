#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from thetaStar_algorithm import thetaStarAlgorithm
from nav_msgs.msg import Odometry
import random

class Planner:
    def __init__(self):
        # Launch params
        self.base_frame_id = rospy.get_param("~base_frame_id", default="09/base_link")
        self.global_frame_id = rospy.get_param("~global_frame_id", default="map")
        # Publisher topics
        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10, latch=True)
        # Subscriber topics
        rospy.Subscriber("/clicked_point", PointStamped, self.goal_callback)
        rospy.Subscriber("goal_pose", PoseStamped, self.goal_callback)
        rospy.Subscriber("map", OccupancyGrid, self.map_callback)
        self.listener = tf.TransformListener()
        self.map = None
        self.goal = PointStamped()
        self.initial_pos = PointStamped()
    
    # Función callback del goal en caso de que publiquemos un objetivo en rviz a mano
    def goal_callback(self, goal):
        self.goal = goal

        print("goalx:", self.goal.point.x)
        print("goaly:", self.goal.point.y)
        self.goalx = self.goal.point.x
        self.goaly = self.goal.point.y
        base_pos = PointStamped()
        base_pos.header.frame_id = self.base_frame_id
        base_pos.header.stamp = rospy.Time()
        if self.initial_pos is None:
            try:
                self.initial_pos = self.listener.transformPoint(self.global_frame_id, base_pos)
                self.initial_pos.point.x = 9
                self.initial_pos.point.y = 1
            except:
                self.initial_pos = None

        self.initx = self.initial_pos.point.x
        self.inity = self.initial_pos.point.y
        self.calculate_route()

    # Función callback del mapa
    def map_callback(self, map):
        self.map = map

    #Función que dadas las coordenadas (x,y) del punto de origen y destino llama a la clase 
    #thetaStarAlgorithm que calcula la ruta a seguir por el robot
    def calculate_path(self, ix, iy, gx, gy):
        self.thetaStar = thetaStarAlgorithm(self.map)
        print("init")
        rx, ry, total_cost = self.thetaStar.planning(ix, iy, gx, gy)
        print("finish")
        path = rx, ry
        return path, total_cost

    # Función que llama a la función calculate_path para calcular una ruta y monta el mensaje de tipo Path
    # para posteriormente publicarlo en el topic /path y que el nodo de control lo reciva para comandar al robot
    def calculate_route(self):
        if self.map is not None or self.goal is not None:
            path, total_cost = self.calculate_path(self.initx, self.inity, self.goalx, self.goaly)

            xpath = path[0]
            ypath = path[1]

            path_m = Path()
            path_m.header.stamp = rospy.Time.now()
            path_m.header.frame_id = "map"
            path_m.header.seq = 0

            for i in range(len(xpath)):
                point = PoseStamped()
                point.pose.position.x = xpath[i]
                point.pose.position.y = ypath[i]
                point.pose.position.z = 0

                path_m.poses.append(point)
            
            self.initial_pos.point.x = path_m.poses[len(path_m.poses) - 1].pose.position.x
            self.initial_pos.point.y = path_m.poses[len(path_m.poses) - 1].pose.position.y
            
            print("Number of nodes created: ", len(path_m.poses))
            print("Total path cost: ", total_cost)

            self.path_publisher.publish(path_m)
    
    # Función para buscar una ruta dado un punto de inicio y un punto objetivo
    def find_path(self, current_robot_pos, current_objetive_pos):
        self.initx = current_robot_pos[0]
        self.inity = current_robot_pos[1]
        self.goalx = current_objetive_pos[0]
        self.goaly = current_objetive_pos[1]
        self.calculate_route()
        rospy.sleep(2)
    
    # Función para buscar una ruta dado un punto de inicio y un punto objetivo aleatorio del mapa 
    def find_random_path(self, current_robot_pos):
        resolution = self.map.info.resolution
        min_x = self.map.info.origin.position.x
        min_y = self.map.info.origin.position.y
        y_width = self.map.info.height
        x_width = self.map.info.width
        max_x = min_x + x_width * resolution
        max_y = min_y + y_width * resolution
        random_goal_x = random.randrange(int(max_x)-2)
        random_goal_y = random.randrange(int(max_y)-2)
        self.initx = current_robot_pos[0]
        self.inity = current_robot_pos[1]
        self.goalx = random_goal_x
        self.goaly = random_goal_y
        print("looking for route")
        self.calculate_route()
        print("route found")
        rospy.sleep(2)

if __name__ == "__main__":
    try:
        rospy.init_node("path_publisher", anonymous=False)
        planner = Planner()
        r = rospy.Rate(200)
        while not rospy.is_shutdown():
            r.sleep()
    except:
        rospy.loginfo("Planning node terminated.")