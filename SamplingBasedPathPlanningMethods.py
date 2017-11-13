from RRTFamilyOfPlanners import RRTFamilyPathPlanner
from drawer import draw_results
import time

class SamplingBasedPathPlanner():
    """Plans path using a sampling based algorithm on a 2D environment.
    
    Contains method for simple RRT based search, RRTStar based search, informed RRTStar based search, and PRM based search, all in 2D space.
    Methods also have the option to draw the results.
    
    """
    
    def __init__(self):
        """
        The planner contains two objects. One for planning using RRT algorithms and another for using a PRM planner.
        """
        self.RRTFamilySolver = RRTFamilyPathPlanner()
#         self.PRMSolver = PRMPathPlanner()

    def RRT(self, environment, bounds, start_pose, goal_region, object_radius, \
            steer_distance, num_iterations, resolution=3, runForFullIterations=False, drawResults=False):
        """Returns a path from the start_pose to the goal region in the current environment using RRT.
        
        Args:
            environment (A yaml environment): Environment where the planner will run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches.
            num_iterations (int): How many points are sampled for the creating of the tree.
            resolution (int): Number of segments used to appromixate a quater circle around a point.
            runForFullIterations (bool): Optional, if True return the first path found without having to sample all num_iterations points.
            drawResults (bool): Optional, if set to True it plots the path and environment using a matplotlib plot.
            
        Returns:
            path (list<(int, int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region.
            self.V (set<(int, int)>): A list of Vertices (coordinates) of nodes in the tree.
            self.E (set<(int, int),(int, int)>): A set of Edges connecting one node to another node in the tree.
            
        """
        start_time = time.time()
        path, V, E = self.RRTFamilySolver.path(environment, bounds, start_pose, goal_region, \
                                               object_radius, steer_distance, num_iterations, \
                                               resolution, runForFullIterations, RRT_Flavour="RRT")
        elapsed_time = time.time() - start_time
        if path and drawResults:
            draw_results("RRT", path, V, E, environment, bounds, object_radius, resolution, star_pose, goal_region, elapsed_time)
        return path, V, E