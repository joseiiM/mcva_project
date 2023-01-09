import matplotlib.pyplot as plt
import math
from nav_msgs.msg import OccupancyGrid
import numpy as np 

class thetaStarAlgorithm:
    def __init__(self, costmap):
        # Copy the map metadata
        self.resolution = costmap.info.resolution
        self.min_x = costmap.info.origin.position.x
        self.min_y = costmap.info.origin.position.y
        self.y_width = costmap.info.height
        self.x_width = costmap.info.width
        self.max_x = self.min_x + self.x_width *self.resolution
        self.max_y = self.min_y + self.y_width *self.resolution
        print(self.min_x, self.min_y)
        print(self.max_x, self.max_y)
        print("Resolution: ", self.resolution)
        print(self.x_width, self.y_width)
        

        self.motion = self.get_motion_model()
        
        # Copy the actual map data from the map
        x = 0
        y = 0
        ox = list()
        oy = list()
         # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        obstacles = 0
        for value in costmap.data:
            if value > 80:            # This 80 value could change depending on the map
                obstacles += 1
                self.obstacle_map[x][y] = True
                ox.append(float(x)*self.resolution +self.min_x)
                oy.append(float(y)*self.resolution +self.min_y)
            # Update the iterators
            x += 1
            if x == self.x_width:
                x = 0
                y += 1

    class Node:
        def __init__(self, x, y, cost_f, parent, goal_node, closed_set, obstacle_map, real_cost):
            self.x = int(x)  # index of grid
            self.y = int(y) # index of grid 
            if goal_node is not None:
                self.heuristic_cost = self.calculate_heuristic_cost(goal_node, x, y)
            else:
                self.heuristic_cost = 0.0
            self.cost_f = cost_f
            self.cost = self.cost_f + self.heuristic_cost
            self.parent = parent  
            self.real_cost = real_cost
            self.last_parent = None 
            if closed_set is not None:
                parent2 = closed_set[parent].parent
                if parent2 != -1:
                    parent2_cost_1 = math.sqrt(math.pow(self.x - closed_set[parent2].x, 2) + math.pow(self.y - closed_set[parent2].y, 2))
                    parent2_cost_2 = parent2_cost_1 + self.calculate_heuristic_cost(goal_node, closed_set[parent2].x, closed_set[parent2].y) 
                    if parent2_cost_2 < self.cost and self.lineOfSight(closed_set[parent2], self.x, self.y, obstacle_map):
                        self.cost = parent2_cost_2
                        self.real_cost = parent2_cost_1 + closed_set[parent2].real_cost
                        self.last_parent = self.parent
                        self.parent = parent2

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent)
        
        def calculate_heuristic_cost(self, goal_node, x, y):
            return np.sqrt(np.power(goal_node.x - x, 2) + np.power(goal_node.y - y, 2))
        
        def lineOfSight(self, last_node, x, y, obstacle_map):
            y_size = len(obstacle_map)
            x_size = len(obstacle_map)
            x1 = last_node.x
            x2 = x
            y1 = last_node.y 
            y2 = y

            #Distance
            dy=y2-y1
            dx=x2-x1

            if dy < 0:
                dy = -dy
                sy = -1
            else:
                sy = 1

            if dx < 0:
                dx = -dx
                sx = -1
            else:
                sx = 1

            f = 0

            if dx >= dy:
                while x1 != x2:
                    f = f + dy
                    if f >= dx and 0 < y1+(sy-1)/2 and y1+(sy-1)/2 < y_size and 0 < x1+(sx-1)/2 and x1+(sx-1)/2 < x_size:
                        if obstacle_map[int(x1+int((sx-1))/2)][int(y1+int((sy-1))/2)]:

                            return False
                        y1 = y1 + sy
                        f  = f  - dx

                    elif 0 < y1+(sy-1)/2 and y1+(sy-1)/2 < y_size and 0 < x1+(sx-1)/2 and x1+(sx-1)/2 < x_size:
                        if f != 0 and obstacle_map[int(x1+(sx-1)/2)][int(y1+(sy-1)/2)]:

                            return False

                    elif 1<y1 and y1<y_size and 0 < x1+(sx-1)/2 and x1+(sx-1)/2 < x_size:
                        if dy==0 and obstacle_map[int(x1+int((sx-1))/2)][y1] and obstacle_map[int(x1+int((sx-1))/2)][y1-1] :

                            return False
                    x1 = x1 + sx

            else:

                while y1 != y2:
                    f = f + dx
                    if f >= dy and 0 < y1+(sy-1)/2 and y1+(sy-1)/2 < y_size and 0< x1+(sx-1)/2 and x1+(sx-1)/2 < x_size:
                        if obstacle_map[int(x1+int((sx-1))/2)][int(y1+int((sy-1))/2)]:

                            return False
                        x1 = x1 + sx
                        f = f - dy
                    elif 0 < y1+(sy-1)/2 and y1+(sy-1)/2 < y_size and 0 < x1+(sx-1)/2 and x1+(sx-1)/2 < x_size:
                        if f !=0 and obstacle_map[int(x1+int((sx-1))/2)][int(y1+int((sy-1))/2)]:

                            return False

                    elif 0 < y1+(sy-1)/2 and y1+(sy-1)/2 < y_size and 1 < x1 and x1 < x_size:       
                        if dx == 0 and obstacle_map[x1][int(y1+ int((sy-1))/2)] and obstacle_map[x1-1][int(y1+int((sy-1))/2)]:

                            return False

                    y1=y1+sy

            return True

        
        def verify_node(self, x, y, obstacle_map):

            if obstacle_map[int(x)][int(y)]:
                return False

            return True
        

    def planning(self, start_x, start_y, goal_x, goal_y):
        """
        input:
            sx: start x position [m]
            sy: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                            self.calc_xy_index(start_y, self.min_y), 0.0, -1, None, None, self.obstacle_map, 0.0)

        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                            self.calc_xy_index(goal_y, self.min_y), 0.0, -1, None ,None, self.obstacle_map, 0.0)

        if (not self.verify_node(start_node)):
            print("Error: init not valid")
            return (0,0)
        
        if (not self.verify_node(goal_node)):
            print("Error: goal not valid")
            return (0,0)
        
        print(start_node, goal_node)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while 1:
            if len(open_set) != 0:
                c_id = min(open_set, key=lambda o: open_set[o].cost) 
                current = open_set[c_id]
            else:
                print("Goal found")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                goal_node.real_cost = current.real_cost
                break
            
            if current.x == goal_node.x and current.y == goal_node.y:
                print("Goal found")
                goal_node.parent = current.parent
                goal_node.cost = current.cost
                goal_node.real_cost = current.real_cost
                break

            # Remove the item from the open set
            del open_set[c_id]
                
            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                current.y + move_y,
                                current.cost + move_cost, c_id, goal_node, closed_set, self.obstacle_map, current.real_cost + move_cost) 

                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        open_set[n_id] = node

        rx, ry, total_cost = self.calc_final_path(goal_node, closed_set)

        return rx, ry, total_cost     
        
    # Function to calculate and return the final path
    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent = goal_node.parent
        while parent != -1:
            n = closed_set[parent]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent = n.parent
        rx.reverse()
        ry.reverse()
        return rx, ry, goal_node.real_cost

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    # Function to calculate the index of the input node
    def calc_index(self, node):
        return (node.x - self.min_x) * self.y_width + (node.y - self.min_y)
    
    # Function to calculate the position of the input node 
    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    # Function to verify that the input node is valid
    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[int(node.x)][int(node.y)]:
            return False

        return True

    # Function to get the motion model
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                [0, 1, 1],
                [-1, 0, 1],
                [0, -1, 1],
                [-1, -1, math.sqrt(2)],
                [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)],
                [1, 1, math.sqrt(2)]]

        return motion