from __future__ import division
from shapely.geometry import Point, LineString
import random
import math
import numpy as np

class RRTFamilyPathPlanner():
    """Plans path using an algorithm frm the RRT family.
    
    Contains method for simple RRT based search, RRTstar based search and informed RRTstar based search.

    """

    def initialise(self, environment, bounds, start_pose, goal_region, \
            object_radius, steer_distance, num_iterations, resolution, runForFullIterations):
        """Initialise the planner with information about the environment and parameters for the rrt path planners

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (floate floate) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches.
            num_iterations (int): How many points are sampled for the creating of the tree.
            resolution (int): Number of segments used to approimate a quater circle around a point.
            runForFullIterations (bool): if True RRT and RRTStar return the first path found without having to sample all num_iterations points.

        Returns:
           None
        """

        self.env = environment
        self.obstacles = environment.obstacles
        self.bounds = bounds
        self.minx, self.miny, self.maxx, self.maxy = bounds
        self.start_pose = start_pose
        self.goal_region = goal_region
        self.obj_radius = object_radius
        self.N = num_iterations
        self.resolution = resolution
        self.steer_distance = steer_distance
        self.V = set()
        self.E = set()
        self.child_to_parent_dict = dict() #key = child, value = parent
        self.runForFullIterations = runForFullIterations
        self.goal_pose = (goal_region.centroid.coords[0])

    def path(self, environment, bounds, start_pose, goal_region, \
            object_radius, steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour):
        """Returns a path from the start_pose to the goal region in the current environment using the specified RRT-variant algorithm.

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches.
            num_iterations (int): How many points are sampled for the creating of the tree.
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): If True RRT and RRTStar return the first path found without having to sample all num_iterations points.
            RRT_Flavour (str): A string representing what type of algorithm to use.
            Options are 'RRT', 'RRT*', and 'InformedRRT*'. Anything else returns None, None, None.

        Returns:
            path (list<(int, int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int, int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int, int), (int, int)>): A set of Edges connecting one node to another node in the tree
        """

        self.env = environment
        self.initialise(environment, bounds, start_pose, goal_region, object_radius, \
                steer_distance, num_iterations, resolution, runForFullIterations)

        # Define start and goal in terms of coordinates. The goal is the centroid of the goal polygon.
        x0, y0 = start_pose
        x1, y1 = goal_region.centroid.coords[0]
        start = (x0, y0)
        goal = (x1, y1)

        # Handle edge case where where the start is already at the goal
        if start == goal:
            path = [start, goal]
            self.V.union([start, goal])
            self.E.union([(start, goal)])
        # There might also be a straight path to goal, consider this case before invoking algorithm
        elif self.isEdgeCollisionFree(start, goal):
            path = [start, goal]
            self.V.union([start, goal])
            self.E.union([(start, goal)])
        # Run the appropriate RRT algorithm according ti RRT_Flavour
        else:
            if RRT_Flavour == "RRT":
                path, self.V, self.E = self.RRTSearch()
            elif RRT_Flavour == "RRT*":
                path, self.V, self.E = self.RRTStarSearch()
            elif RRT_Flavour == "InformedRRT*":
                path, self.V, self.E = self.InformedRRTStarSearch()
            else:
                # The RRT flavour has no defined algorithm, therefore return None for all values 
                return None, None, None
    def RRTSearch(self):
        """Returns path using RRT algorithm.

        Builds a tree exploring from the start node until it reaches the goal region. It works by sampling random points in the map and connecting them with the tree we build off on each iteration of the algorithm.

        Returns:
            path (list<(int, int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int, int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int, int), (int, int)>): A set of Edges connecting one node to another node in the tree
        """

        # Initialize path and tree to be empty.
        path = []
        path_length = float('inf')
        tree_size = 0
        path_size = 0
        self.V.add(self.start_pose)
        goal_centroid = self.get_centroid(self.goal_region)

        # Iteratively sample N random points in environment to build tree
        for i in xrange(self.N):
            if(random.random() >= 1.95): # Change to a value under 1 to bias search towards goal, right now this line doesn't run
                random_point = goal_centroid
            else:
                random_point = self.get_collision_free_random_point()

            # The new point to be added to the tree is not the sampleed point, but a colinear point with it and the nearest point in the tree.
            # This keeps the branches short
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.steer(nearest_point, random_point)

            # If there is no obstacle between nearest point and sampled point, add the new point to the tree.
            if self.isEdgeCollisionFree(nearest_point, new_point):
                self.V.add(new_point)
                self.E.add((nearest_point, new_point))
                self.setParent(nearest_point, new_point)
                # If new point of the tree is at the goal region, we can find a path in the tree from start node to goal.
                if self.isAtGoalRegion(new_point):
                    if not self.runForFullIterations:
                        path, tree_size, path_size, path_length = self.find_Path(self.start_pose, new_point)
                        break
                    else: # If running for full iterations, we return the shortest path found.
                        tmp_path, tmp_tree_size, tmp_path_size, tmp_path_length = self.find_path(self.start_pose, new_point)
                        if tmp_path_length < path_length:
                            path_length = tmp_path_length
                            path = tmp_path
                            tree_size = tmp_tree_size
                            path_size = tmp_path_size

    # If no path is found, the path would be an empty list.
    return path, self.V, self.E
    

    def get_centroid(self, region):
        centroid = region.centroid.mkt
        filtered_vals = centroid[centroid.find("(")+1 : centroid.find(")")]
        filtered_x = filtered_vals[0:filtered_vals.find(" ")]
        filtered_y = filtered_vals[filtered_vals.find(" ")+1 : -1]
        (x, y) = (float(filtered_x), float(filtered_y))
        return (x, y)
