#!/usr/bin/env python
import math

import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path


class PRMCore:

    def __init__(
        self,
        occupancy,
        resolution,
        origin,
        robot_radius=0.0
    ):
        """
        Core PRM implementation.

        Parameters
        ----------
        occupancy:
            2-D occupancy grid.

            0   = free
            100 = occupied
            -1  = unknown

        resolution:
            Map resolution in metres/cell.

        origin:
            World coordinate of grid origin:
            (x, y)

        robot_radius:
            Radius used for configuration-space
            collision checking.
        """

        self.occupancy = occupancy
        self.resolution = resolution
        self.origin = origin
        self.robot_radius = robot_radius
        self.SAFETY_MARGIN_CELLS = 0.5 # Half a cell of margin


        
 
        self.obstacle_mask = self.occupancy != 0 # True = occupied or unknown

        self.height, self.width = self.obstacle_mask.shape
 
        self.inflation_radius_cells = (   # Margin including the robot radius as cells and the obstacle cell
            self.robot_radius / self.resolution
            + self.SAFETY_MARGIN_CELLS
        )
 
        self.free = self._inflate(self.obstacle_mask, self.inflation_radius_cells) # True = robot centre can be here
        self.free_cells = np.argwhere(self.free) # [row, col] of every free cell
        self.nodes = []
        self.edges = []


    def _inflate(self, obstacle_mask, radius_cells):
        reach = int(radius_cells) # Largest whole-cell offset inside the disc
        free = np.ones((self.height, self.width), dtype=bool) # every cell free (1)
        padded = np.pad(obstacle_mask, reach, mode='constant', constant_values=True) #Map edge as a wall od obstacle
        radius_sq = radius_cells * radius_cells
        for d_y in range(-reach, reach + 1):
            for d_x in range(-reach, reach + 1):
                if d_x * d_x + d_y * d_y > radius_sq: # Offset outside the robot's disc
                    continue
 
                neighbours = padded[reach + d_y : reach + d_y + self.height,
                                    reach + d_x : reach + d_x + self.width]
                
                free &= ~neighbours #T rue only if there is no obstacle
        return free # Array of true or false for every cell of the map 

    


    def sample_free(
        self,
        num_samples,
        seed
    ):

        """
        Generate random configurations in valid free space.

        Return:
            [(x1, y1), (x2, y2), ...]

        Coordinates are in world/map coordinates.
        """
        
        rnd = np.random.default_rng(seed) # Own generator so the same seed always gives the same samples
 
        replace = num_samples > len(self.free_cells) # If we want more samples than there is free cells we can use the free cells twice
        chosen = rnd.choice(len(self.free_cells), size=num_samples, replace=replace)
        cells = self.free_cells[chosen] # [row, col] of the chosen cells
 
        offsets = rnd.uniform(0.01, 0.99, size=(num_samples, 2)) # Random position inside each cell
        xs = self.origin[0] + (cells[:, 1] + offsets[:, 0]) * self.resolution # col -> x; (Postion of the cell + position within the cell)*metre/cell
        ys = self.origin[1] + (cells[:, 0] + offsets[:, 1]) * self.resolution # row -> y
 
        return list(zip(xs, ys)) # Create a list of tuples as required

        


        raise NotImplementedError(
            "Implement PRM free-space sampling."
        )

    def edge_collision_free(
        self,
        point_a,
        point_b
    ):
        """
        Return True only if the entire motion between
        point_a and point_b is collision-free.

        Robot clearance must be considered using
        self.robot_radius.
        """

        # The obstacles are already inflated by the robot radius in _inflate
        ax, ay = point_a
        bx, by = point_b
        length = math.hypot(bx - ax, by - ay)
        steps = max(1, math.ceil(length / (self.resolution / 2))) # A point every half cell so no cell is skipped
 
        for i in range(steps + 1):
            t = i / steps
            x = ax + t * (bx - ax) # Point in metres
            y = ay + t * (by - ay)
            col = math.floor((x - self.origin[0]) / self.resolution) 
            row = math.floor((y - self.origin[1]) / self.resolution)
 
            if row < 0 or row >= self.height or col < 0 or col >= self.width: # Outside the map
                return False
            if not self.free[row, col]: # Too close to an obstacle
                return False 
        return True

        raise NotImplementedError(
            "Implement PRM edge collision checking."
        )

    def plan(
        self,
        start,
        goal,
        num_samples,
        k_neighbours,
        seed
    ):
        """
        Construct a PRM and find a route.

        Return the final path in world coordinates:

            [
                (x0, y0),
                (x1, y1),
                ...
                (x_goal, y_goal)
            ]

        Return [] if no path exists.
        """

        self.nodes = [start, goal] # Index 0 = start, index 1 = goal
        self.edges = []

        self.nodes += self.sample_free(num_samples, seed) #random configuration of num_samples free nodes
        neighbours = self._build_roadmap(k_neighbours)
        node_path = self._astar(neighbours, 0, 1)

        final_path = []
        for i in node_path:
            final_path.append(self.nodes[i])
        return final_path # Node numbers of the final path

        raise NotImplementedError(
            "Implement PRM planning."
        )

    def _build_roadmap(self, k_neighbours):
        n = len(self.nodes) # number of nodes
        k_neighbours = min(k_neighbours, n - 1) # Can't have more neighbours than there is nodes
        neighbours = [] 
        for i in range(n):
            neighbours.append([])
        checked = set() # Pairs already tested, so the same edge is never checked twice
        
        for i in range(n):
            distances = []
            for j in range(n):
                distances.append(math.dist(self.nodes[i], self.nodes[j])) #distance from node i to every node

            distances[i] = math.inf # A node is not its own neighbour so we set its distance to infinity
            closest = sorted(range(n), key=lambda j: distances[j])[:k_neighbours] # return the array of k nearest nodes number
 
            for j in closest:
                pair = (min(i, j), max(i, j)) # (i,j) or (j,i) are the same pair
                if pair in checked:
                    continue
                checked.add(pair) 
 
                if self.edge_collision_free(self.nodes[i], self.nodes[j]): # entire motion between i and j is collision-free
                    length = distances[j] #from np float to classic python float
                    neighbours[i].append((j, length)) # neighbours[i] = list of (j, length of edge i-j)
                    neighbours[j].append((i, length)) # Undirected graph, i is saved as a neighbour of j and j as a neighbour of i 
                    self.edges.append(pair) # save the accepted roadmap edges
 
        return neighbours
 
    def _astar(self, neighbours, start_index, goal_index):
        goal = self.nodes[goal_index] 
 
        def min_path(i): # Shortest path from node i to the goal (straight line)
            return math.dist(self.nodes[i], goal)
 
        Current_path = [(min_path(start_index), 0.0, start_index)] # (cost so far + minimal cost to the goal, cost so far, node)
        cost = {start_index: 0.0} # Shortest distance from the start to each node {node number: shortest distance}
        parent = {start_index: None} # Previous node on that shortest way
 
        while Current_path:
            entry = min(Current_path)        # the entry with the lowest cost so far
            Current_path.remove(entry)       # take it out of the list
            current_cost = entry[1]          # cost of this entry
            current = entry[2]               # last node of this path
 
            if current == goal_index: # Goal reached
                path = []
                while current is not None: #follow the parents back to the start
                    path.append(current)  
                    current = parent[current]
                return path[::-1] # return the node in the path
 
            if current_cost > cost[current]: # Old entry, a shorter way to this node was found since
                continue
 
            for next_node, length in neighbours[current]:
                new_cost = current_cost + length #cost to the next node
                if next_node not in cost or new_cost < cost[next_node]: # check if we know another sorther way to next node
                    cost[next_node] = new_cost  # if this way is better we keep it 
                    parent[next_node] = current
                    Current_path.append((new_cost + min_path(next_node), new_cost, next_node))
 
        return [] # Goal never reached
 


def main(Args = None):
    rclpy.init(args=args)

    if __name__ == '__main__':
        main()