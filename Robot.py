"""
Robot Class

Author: Yasim Ahmad(yaaximus)

Email: yasim.ahmed63@yahoo.com
"""
import math as ma
import numpy as np
from utils.positional import Position


class Robot(object):
    """
    This class encorporates the capability of defining Robot object.
    """

    def __init__(self, Position, sensor_range=2.0, num_of_artif_pts=60):
        """
        This is the constructor of Robot class

        Parameters
        ----------
        Position : [Position]
            [Position of object], by default is 1.0 in both x and y axis
        sensor_range : float, optional
            [Distance from robot centre where artificial points will be defined], by default 2.0
        num_of_artif_pts : int, optional
            [Number of artificial points], by default 60
        """

        self.position = Position
        self.__sensor_range = sensor_range
        self.__step_size = 0.4*self.__sensor_range
        self.__num_of_artif_pts = num_of_artif_pts
        self.__step_degree = 360.0/self.__num_of_artif_pts
        self.__artif_pts_x = np.zeros((1, self.__num_of_artif_pts))
        self.__artif_pts_y = np.zeros((1, self.__num_of_artif_pts))
        self.__theta = np.zeros((1, self.__num_of_artif_pts))
        self.__update_theta()
        self.__artif_pts_cost_obst = np.zeros((1, self.__num_of_artif_pts))
        self.__artif_pts_cost_goal = np.zeros((1, self.__num_of_artif_pts))
        self.__artif_pts_cost = np.zeros((1, self.__num_of_artif_pts))
        self.__dist_to_goal_artif_pts = np.zeros((1, self.__num_of_artif_pts))
        self.__err_cost_func = np.zeros((1, self.__num_of_artif_pts))
        self.__err_dist_to_goal = np.zeros((1, self.__num_of_artif_pts))
        self.__fitness = np.zeros((1, self.__num_of_artif_pts))

    def __update_theta(self):
        """
        This function is used for computing angle theta's which are responsible for 
        finding x and y position of artif points x = xprev + cos(theta),
        y = yprev + sin(theta).
        """

        self.__theta[0][0] = self.__step_degree

        for num in range(1, self.__num_of_artif_pts):
            self.__theta[0][num] = self.__theta[0][num-1] + self.__step_degree

    def __cost_function_for_obstacles(self, obstacles, pt_x=None, pt_y=None):
        """
        This function is responsible for calculating potential of current 
        point w.r.t all obstacle. If pt_x and pt_y are not passed as a argument
        then x and y coordinate of robot will be used.

        Parameters
        ----------
        obstacles : [list]
            [list of obstalce objects]
        pt_x : [float], optional
            [Position in x-axis], by default None
        pt_y : [float], optional
            [Position in y-axis], by default None

        Returns
        -------
        [float]
            [Potential of current point w.r.t to obstacles]
        """

        cost_obst = 0.0

        if pt_x is None:
            for obstacle in obstacles:
                cost_obst = cost_obst + obstacle.get_alpha()*ma.exp(-0.5 *
                                                                    ma.pow((self.position.calculate_distance(other=obstacle) / obstacle.get_sigma()), 2.0))
        else:
            for obstacle in obstacles:
                cost_obst = cost_obst + obstacle.get_alpha()*ma.exp(-0.5 *
                                                                    ma.pow((self.position.calculate_distance(other=obstacle, x=pt_x, y=pt_y) / obstacle.get_sigma()), 2.0))

        return cost_obst

    def __cost_function_for_goal(self, goal, pt_x=None, pt_y=None):
        """
        This function is responsible for calculating potential of current 
        point w.r.t goal. If pt_x and pt_y are not passed as a argument
        then x and y coordinate of robot will be used.

        Parameters
        ----------
        goal : [Goal]]
            [Object of class Goal]
        pt_x : [float], optional
            [Position in x-axis], by default None
        pt_y : [float], optional
            [Position in y-axis], by default None

        Returns
        -------
        [float]
            [Potential of current point w.r.t to goal]
        """

        if pt_x is None:
            return -goal.get_alpha()*ma.exp(-0.5*ma.pow((self.position.calculate_distance(other=goal) / goal.get_sigma()), 2.0))
        else:
            return -goal.get_alpha()*ma.exp(-0.5*ma.pow((self.position.calculate_distance(other=goal, x=pt_x, y=pt_y) / goal.get_sigma()), 2.0))

    def cost_function(self, goal, obstacles, pt_x=None, pt_y=None):
        """
        This function is responsible for calculating total potential of a point 
        w.r.t goal and obstacles.If pt_x and pt_y are not passed as a argument
        then x and y coordinate of robot will be used.

        Parameters
        ----------
        goal : [Goal]
            [Object of class Goal]
        obstacles : [list]
            [List of obstalce objects]
        pt_x : [float], optional
            [Position in x-axis], by default None
        pt_y : [float], optional
            [Position in y-axis], by default None

        Returns
        -------
        [float]
            [total potential of a point w.r.t goal and obstacles]
        """

        if pt_x is None:
            return self.__cost_function_for_goal(goal) + self.__cost_function_for_obstacles(obstacles)

        else:
            return self.__cost_function_for_goal(goal, pt_x=pt_x, pt_y=pt_y) + self.__cost_function_for_obstacles(obstacles, pt_x=pt_x, pt_y=pt_x)

    def __update_artif_pts_coords(self):
        """
        This function is responsible for updating artif points of robot.
        """

        for num in range(1, self.__num_of_artif_pts+1):
            self.__artif_pts_x[0][num-1] = self.position.x + \
                (self.__step_size*ma.cos(ma.pi*self.__theta[0][num-1]/180.0))
            self.__artif_pts_y[0][num-1] = self.position.y + \
                (self.__step_size*ma.sin(ma.pi*self.__theta[0][num-1]/180.0))

    def __update_artif_pts(self, goal, obstacles):
        """
        This function is responsible for updating potential, distance to goal
        , error calculation, and fitness of artif points.

        Parameters
        ----------
        goal : [Goal]
            [Object of class Goal]
        obstacles : [list]
            [List of obstalce objects]
        """

        for num in range(0, self.__num_of_artif_pts):
            self.__artif_pts_cost_obst[0][num] = self.__cost_function_for_obstacles(
                obstacles=obstacles, pt_x=self.__artif_pts_x[0][num], pt_y=self.__artif_pts_y[0][num])
            self.__artif_pts_cost_goal[0][num] = self.__cost_function_for_goal(
                goal=goal, pt_x=self.__artif_pts_x[0][num], pt_y=self.__artif_pts_y[0][num])
            self.__artif_pts_cost[0][num] = self.__artif_pts_cost_obst[0][num] + \
                self.__artif_pts_cost_goal[0][num]
            self.__dist_to_goal_artif_pts[0][num] = self.position.calculate_distance(
                other=goal, x=self.__artif_pts_x[0][num], y=self.__artif_pts_y[0][num])

        self.__calculate_error(goal=goal, obstacles=obstacles)
        self.__calculate_fitness()

    def __calculate_error(self, goal, obstacles):
        """
        This function is responsible for calculating error in potential and
        distance to goal of artif points w.r.t goal and obstacles.

        Parameters
        ----------
        goal : [Goal]
            [Object of class Goal]
        obstacles : [list]
            [List of obstalce objects]
        """

        JT = self.__cost_function_for_obstacles(
            obstacles) + self.__cost_function_for_goal(goal)
        DTG = self.position.calculate_distance(other=goal)

        for num in range(0, self.__num_of_artif_pts):
            self.__err_cost_func[0][num] = self.__artif_pts_cost[0][num] - JT
            self.__err_dist_to_goal[0][num] = self.__dist_to_goal_artif_pts[0][num] - DTG

    def __calculate_fitness(self):
        """
        This function is responsible for calculating fitness based on error
        of artif points w.r.t goal and obstacles.
        """

        for num in range(0, self.__num_of_artif_pts):
            self.__fitness[0][num] = - self.__err_dist_to_goal[0][num]

    def decide_next_move(self, goal, obstacles):
        """
        This function is responsible for deciding next move of robot based on
        potential, and distance to goal of all artif potential points w.r.t
        goal and obstacles.

        Parameters
        ----------
        goal : [type]
            [description]
        obstacles : [type]
            [description]
        """

        self.__update_artif_pts_coords()

        self.__update_artif_pts(goal=goal, obstacles=obstacles)

    def take_next_move(self):
        """
        This function is responsible for checking criteria for moving robot
        to next position based on a point for which the distance error is 
        negative and the potential error is smallest and negative.
        """

        check = 0
        for num in range(0, self.__num_of_artif_pts):
            k = np.argmax(self.__fitness[0])
            if (self.__err_cost_func[0][k] < 0):
                check = check + 1
                self.position.x = self.__artif_pts_x[0][k]
                self.position.y = self.__artif_pts_y[0][k]
            else:
                self.__fitness[0][k] = 0.0

        if check == 0:
            print("No Solution Exists")
        else:
            print("Solution Exists")