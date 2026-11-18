#!/usr/bin/env python3
"""PRM (Probabilistic Roadmap) planner core, adapted from CW1 to plan on a
SLAM map built with slam_toolbox.

Changes from the CW1 version are marked "CW2:" in the comments:

* Map interpretation: occupied cells are inflated by robot radius +
  safety margin; unknown areas by the robot radius only (the whole robot
  must be on known ground), after removing tiny unknown specks that SLAM
  leaves inside rooms, so they do not block corridors and doorways.
* Safety distance: a clearance in metres (safety_margin) is added on top
  of the robot radius, instead of half a cell.
* Multi-query: the roadmap is built once per map (build_roadmap) and
  reused for every goal (query). Start and goal are connected to it per
  query. plan() still does both in one call, as in CW1.
* Start/goal snapping: a start or goal inside the safety zone (e.g. the
  robot parked close to a wall) is moved to the nearest safe cell.
* Path shortcutting: the A* path through the roadmap is shortened by
  skipping waypoints whenever the direct segment is collision-free.
* k-nearest neighbours computed with numpy (the SLAM map is much larger
  than the CW1 map, so the pure-Python version was slow).
* Statistics (nodes, edges, timings, path length) kept in self.stats.
"""
import math
import time

import numpy as np
from scipy import ndimage


class PRMCore:

    def __init__(
        self,
        occupancy,
        resolution,
        origin,
        robot_radius=0.0,
        safety_margin=0.10,
        occupied_threshold=65,
        free_threshold=25,
        min_unknown_area=0.05
    ):
        """
        Core PRM implementation.

        Parameters
        ----------
        occupancy:
            2-D occupancy grid (nav_msgs/OccupancyGrid values).

            0..100 = probability of occupation (map_server gives 0 or 100)
            -1     = unknown

        resolution:
            Map resolution in metres/cell.

        origin:
            World coordinate of grid origin:
            (x, y)

        robot_radius:
            Radius used for configuration-space
            collision checking.

        safety_margin:
            CW2: extra clearance (m) kept between the robot and obstacles.

        occupied_threshold / free_threshold:
            CW2: cells >= occupied_threshold are obstacles, cells between
            0 and free_threshold are known free space (same thresholds as
            the map saver's .yaml).

        min_unknown_area:
            CW2: unknown patches smaller than this (m^2) are SLAM noise
            inside explored space and are ignored.
        """

        self.occupancy = occupancy
        self.resolution = resolution
        self.origin = origin
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin

        # CW2: a SLAM map has three kinds of cells, handled differently:
        # * occupied (walls, furniture): inflated by robot radius + safety
        #   margin, so paths keep a safe distance from them.
        # * unknown: the whole robot must be on known ground, so unknown
        #   areas are inflated by the robot radius only. This removes the
        #   one-cell-wide "fingers" of free space where single laser rays
        #   went through gaps (unusable, and they waste samples).
        #   Tiny unknown specks inside explored rooms are ignored, so they
        #   do not block corridors.
        # * free: everything else.
        self.obstacle_mask = self.occupancy >= occupied_threshold
        unknown = (self.occupancy < 0) | (
            (self.occupancy > free_threshold) & ~self.obstacle_mask)

        self.height, self.width = self.obstacle_mask.shape

        labels, count = ndimage.label(unknown)
        if count:
            sizes = ndimage.sum(unknown, labels, index=np.arange(1, count + 1))
            min_cells = min_unknown_area / (self.resolution ** 2)
            small = np.zeros(count + 1, dtype=bool)
            small[1:] = sizes < min_cells
            unknown &= ~small[labels]  # drop the small specks

        self.inflation_radius_cells = (   # Robot radius + safety distance, in cells
            (self.robot_radius + self.safety_margin) / self.resolution
        )

        # True = robot centre can be here: far from walls AND on known ground
        self.free = (
            self._inflate(self.obstacle_mask, self.inflation_radius_cells)
            & self._inflate(unknown, self.robot_radius / self.resolution)
            & ~unknown
        )
        self.free_cells = np.argwhere(self.free)  # [row, col] of every free cell

        # Roadmap (built once by build_roadmap, reused by every query)
        self.roadmap_nodes = []
        self.roadmap_neighbours = []
        self.roadmap_edges = []

        # Result of the last query (used for RViz markers)
        self.nodes = []      # roadmap nodes + start + goal
        self.edges = []
        self.start = None
        self.goal = None
        self.raw_path = []   # path through the roadmap before shortcutting
        self.stats = {}

    # ------------------------------------------------------------------
    # Configuration space
    # ------------------------------------------------------------------
    def _inflate(self, obstacle_mask, radius_cells):
        reach = int(radius_cells)  # Largest whole-cell offset inside the disc
        free = np.ones((self.height, self.width), dtype=bool)  # every cell free (1)
        padded = np.pad(obstacle_mask, reach, mode='constant', constant_values=True)  # Map edge as a wall or obstacle
        radius_sq = radius_cells * radius_cells
        for d_y in range(-reach, reach + 1):
            for d_x in range(-reach, reach + 1):
                if d_x * d_x + d_y * d_y > radius_sq:  # Offset outside the robot's disc
                    continue

                neighbours = padded[reach + d_y: reach + d_y + self.height,
                                    reach + d_x: reach + d_x + self.width]

                free &= ~neighbours  # True only if there is no obstacle
        return free  # Array of true or false for every cell of the map

    def world_to_cell(self, point):
        col = math.floor((point[0] - self.origin[0]) / self.resolution)
        row = math.floor((point[1] - self.origin[1]) / self.resolution)
        return row, col

    def cell_to_world(self, row, col):
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def is_free(self, point):
        row, col = self.world_to_cell(point)
        if row < 0 or row >= self.height or col < 0 or col >= self.width:
            return False
        return bool(self.free[row, col])

    def snap_to_free(self, point, max_distance=0.6):
        """CW2: return point if it is safe, otherwise the centre of the
        nearest safe cell within max_distance metres, otherwise None."""
        if self.is_free(point):
            return point
        if len(self.free_cells) == 0:
            return None
        row, col = self.world_to_cell(point)
        d2 = ((self.free_cells[:, 0] - row) ** 2
               + (self.free_cells[:, 1] - col) ** 2)
        best = int(np.argmin(d2))
        if math.sqrt(d2[best]) * self.resolution > max_distance:
            return None
        return self.cell_to_world(*self.free_cells[best])

    # ------------------------------------------------------------------
    # Sampling and collision checking (unchanged from CW1)
    # ------------------------------------------------------------------
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

        rnd = np.random.default_rng(seed)  # Own generator so the same seed always gives the same samples

        replace = num_samples > len(self.free_cells)  # If we want more samples than there is free cells we can use the free cells twice
        chosen = rnd.choice(len(self.free_cells), size=num_samples, replace=replace)
        cells = self.free_cells[chosen]  # [row, col] of the chosen cells

        offsets = rnd.uniform(0.01, 0.99, size=(num_samples, 2))  # Random position inside each cell
        xs = self.origin[0] + (cells[:, 1] + offsets[:, 0]) * self.resolution  # col -> x; (Position of the cell + position within the cell)*metre/cell
        ys = self.origin[1] + (cells[:, 0] + offsets[:, 1]) * self.resolution  # row -> y

        return [(float(x), float(y)) for x, y in zip(xs, ys)]

    def edge_collision_free(
        self,
        point_a,
        point_b
    ):
        """
        Return True only if the entire motion between
        point_a and point_b is collision-free.

        Robot clearance is already in self.free (obstacles inflated by
        robot_radius + safety_margin).
        """
        ax, ay = point_a
        bx, by = point_b
        length = math.hypot(bx - ax, by - ay)
        steps = max(1, math.ceil(length / (self.resolution / 2)))  # A point every half cell so no cell is skipped

        for i in range(steps + 1):
            t = i / steps
            x = ax + t * (bx - ax)  # Point in metres
            y = ay + t * (by - ay)
            col = math.floor((x - self.origin[0]) / self.resolution)
            row = math.floor((y - self.origin[1]) / self.resolution)

            if row < 0 or row >= self.height or col < 0 or col >= self.width:  # Outside the map
                return False
            if not self.free[row, col]:  # Too close to an obstacle or unknown
                return False
        return True

    # ------------------------------------------------------------------
    # Roadmap (built once per map)
    # ------------------------------------------------------------------
    def build_roadmap(self, num_samples, k_neighbours, seed):
        """CW2: sample the free space and connect each sample to its k
        nearest collision-free neighbours. Reused by every query."""
        t0 = time.perf_counter()
        nodes = self.sample_free(num_samples, seed)
        n = len(nodes)
        k = min(k_neighbours, n - 1)
        pts = np.array(nodes)

        # CW2: all pairwise distances at once with numpy
        dist = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
        np.fill_diagonal(dist, np.inf)  # A node is not its own neighbour
        nearest = np.argsort(dist, axis=1)[:, :k]  # k nearest node numbers for every node

        neighbours = [[] for _ in range(n)]
        edges = []
        checked = set()  # Pairs already tested, so the same edge is never checked twice
        for i in range(n):
            for j in nearest[i]:
                j = int(j)
                pair = (min(i, j), max(i, j))  # (i,j) or (j,i) are the same pair
                if pair in checked:
                    continue
                checked.add(pair)
                if self.edge_collision_free(nodes[i], nodes[j]):
                    length = float(dist[i, j])
                    neighbours[i].append((j, length))  # Undirected graph
                    neighbours[j].append((i, length))
                    edges.append(pair)

        self.roadmap_nodes = nodes
        self.roadmap_neighbours = neighbours
        self.roadmap_edges = edges
        self.stats = {
            'roadmap_nodes': n,
            'roadmap_edges': len(edges),
            'build_time': time.perf_counter() - t0,
        }
        return len(edges)

    # ------------------------------------------------------------------
    # Query (one per goal)
    # ------------------------------------------------------------------
    def query(self, start, goal, k_neighbours, shortcut=True):
        """CW2: connect start and goal to the roadmap and search it.

        Returns the path [(x0, y0), ..., (x_goal, y_goal)], or [] if no
        path exists. self.start / self.goal hold the (possibly snapped)
        start and goal actually used."""
        t0 = time.perf_counter()
        self.raw_path = []
        self.start = self.snap_to_free(start)
        self.goal = self.snap_to_free(goal)
        self.stats.update({
            'start_snapped': self.start is not None and tuple(self.start) != tuple(start),
            'goal_snapped': self.goal is not None and tuple(self.goal) != tuple(goal),
        })
        if self.start is None or self.goal is None:
            self.stats['failure'] = (
                'start not reachable' if self.start is None else 'goal not reachable'
            )
            self.stats['query_time'] = time.perf_counter() - t0
            return []

        n = len(self.roadmap_nodes)
        start_index, goal_index = n, n + 1
        self.nodes = list(self.roadmap_nodes) + [self.start, self.goal]
        neighbours = [list(nb) for nb in self.roadmap_neighbours] + [[], []]
        self.edges = list(self.roadmap_edges)

        # Direct connection if the straight line is safe
        if self.edge_collision_free(self.start, self.goal):
            d = math.dist(self.start, self.goal)
            neighbours[start_index].append((goal_index, d))
            neighbours[goal_index].append((start_index, d))
            self.edges.append((start_index, goal_index))

        # Connect start and goal to their k nearest roadmap nodes
        pts = np.array(self.roadmap_nodes)
        for index in (start_index, goal_index):
            p = self.nodes[index]
            d = np.linalg.norm(pts - np.array(p), axis=1)
            for j in np.argsort(d)[:k_neighbours]:
                j = int(j)
                if self.edge_collision_free(p, self.nodes[j]):
                    neighbours[index].append((j, float(d[j])))
                    neighbours[j].append((index, float(d[j])))
                    self.edges.append((index, j))

        node_path = self._astar(neighbours, start_index, goal_index)
        path = [self.nodes[i] for i in node_path]
        self.raw_path = path
        if path and shortcut:
            path = self.shortcut(path)

        self.stats.update({
            'query_time': time.perf_counter() - t0,
            'raw_waypoints': len(self.raw_path),
            'waypoints': len(path),
            'raw_length': self.path_length(self.raw_path),
            'path_length': self.path_length(path),
            'failure': '' if path else 'no connection through the roadmap',
        })
        return path

    def plan(
        self,
        start,
        goal,
        num_samples,
        k_neighbours,
        seed
    ):
        """
        Construct a PRM and find a route (CW1 interface, single query).

        Return the final path in world coordinates:

            [
                (x0, y0),
                (x1, y1),
                ...
                (x_goal, y_goal)
            ]

        Return [] if no path exists.
        """
        self.build_roadmap(num_samples, k_neighbours, seed)
        return self.query(start, goal, k_neighbours)

    # ------------------------------------------------------------------
    # Path post-processing
    # ------------------------------------------------------------------
    def shortcut(self, path):
        """CW2: greedy shortcutting. From each waypoint, jump to the
        furthest later waypoint that can be reached in a straight,
        collision-free line. Keeps the same safety distance."""
        result = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1 and not self.edge_collision_free(path[i], path[j]):
                j -= 1
            result.append(path[j])
            i = j
        return result

    @staticmethod
    def path_length(path):
        return sum(math.dist(path[i], path[i + 1]) for i in range(len(path) - 1))

    # ------------------------------------------------------------------
    # Graph search (unchanged from CW1)
    # ------------------------------------------------------------------
    def _astar(self, neighbours, start_index, goal_index):
        goal = self.nodes[goal_index]

        def min_path(i):  # Shortest path from node i to the goal (straight line)
            return math.dist(self.nodes[i], goal)

        Current_path = [(min_path(start_index), 0.0, start_index)]  # (cost so far + minimal cost to the goal, cost so far, node)
        cost = {start_index: 0.0}  # Shortest distance from the start to each node {node number: shortest distance}
        parent = {start_index: None}  # Previous node on that shortest way

        while Current_path:
            entry = min(Current_path)        # the entry with the lowest cost so far
            Current_path.remove(entry)       # take it out of the list
            current_cost = entry[1]          # cost of this entry
            current = entry[2]               # last node of this path

            if current == goal_index:  # Goal reached
                path = []
                while current is not None:  # follow the parents back to the start
                    path.append(current)
                    current = parent[current]
                return path[::-1]  # return the node in the path

            if current_cost > cost[current]:  # Old entry, a shorter way to this node was found since
                continue

            for next_node, length in neighbours[current]:
                new_cost = current_cost + length  # cost to the next node
                if next_node not in cost or new_cost < cost[next_node]:  # check if we know another shorter way to next node
                    cost[next_node] = new_cost  # if this way is better we keep it
                    parent[next_node] = current
                    Current_path.append((new_cost + min_path(next_node), new_cost, next_node))

        return []  # Goal never reached
