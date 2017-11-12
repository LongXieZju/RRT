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

