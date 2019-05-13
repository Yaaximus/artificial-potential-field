import math as ma
import numpy as np

class Robot(object):
    """
    This class encorporates the capability of defining Robot object.
    """

    def __init__(self, pos_x=3.0, pos_y=3.0, sensor_range=2.0, npts=60):
        """
        This is the constructor of Robot class
        
        Parameters
        ----------
        pos_x : float, optional
            [Position in x-axis of Robot], by default 3.0
        pos_y : float, optional
            [Position in y-axis of Robot], by default 3.0
        sensor_range : float, optional
            [Distance from robot centre where artificial points will be defined], by default 2.0
        npts : int, optional
            [Number of artificial points], by default 60
        """

        self._pos_x = pos_x
        self._pos_y = pos_y
        self._sensor_range = sensor_range
        self._step_size = 0.4*self._sensor_range
        self._npts = npts
        self._step_degree = 360.0/self._npts
        self._artificial_point_x = np.zeros((1, self._npts))
        self._artificial_point_y = np.zeros((1, self._npts))
        self._theta = np.zeros((1, self._npts))
        self._update_theta()
        self._confirm_message = ["Solution Exists"]
        self._error_message = ["No Solution Exists"]
        self._j_obst_artificial_points = np.zeros((1, self._npts))
        self._j_goal_artificial_points = np.zeros((1, self._npts))
        self._jt_artificial_points = np.zeros((1, self._npts))
        self._dtg_artificial_points = np.zeros((1, self._npts))
        self._err_jt = np.zeros((1, self._npts))
        self._err_dtg = np.zeros((1, self._npts))
        self._fitness = np.zeros((1, self._npts))

    def _update_theta(self):
        """
        This function is used for computing angle theta's which are responsible for 
        finding x and y position of artificial points x = xprev + cos(theta),
        y = yprev + sin(theta).
        """

        self._theta[0][0] = self._step_degree

        for num in range(1, self._npts):
            self._theta[0][num] = self._theta[0][num-1] + self._step_degree

    def get_coordinate(self, coordinate=None):
        """
        This function returns a specified coordinate of Robot.
        
        Parameters
        ----------
        coordinate : [char], optional
            "x" or "y" position of robot], by default None
        
        Returns
        -------
        [float]
            [position of robot in coordinate frame]
        """

        return self._pos_x if coordinate == "x" else self._pos_y

    def _cost_function_for_obstacles(self, obstacles, pt_x=None, pt_y=None):
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

        J_Obst = 0.0

        if pt_x is None:
            for obstacle in obstacles:
                J_Obst = J_Obst + obstacle.get_alpha()*ma.exp(-0.5*ma.pow( (self.get_distance_to_object(goal=obstacle) / obstacle.get_sigma()),2.0))
        else:
            for obstacle in obstacles:
                J_Obst = J_Obst + obstacle.get_alpha()*ma.exp(-0.5*ma.pow( (self.get_distance_to_object(goal=obstacle, pt_x=pt_x, pt_y=pt_y) / obstacle.get_sigma()),2.0))

        return J_Obst

    def _cost_function_for_goal(self, goal, pt_x=None, pt_y=None):
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
            return -goal.get_alpha()*ma.exp(-0.5*ma.pow( (self.get_distance_to_object(goal=goal) / goal.get_sigma()),2.0))
        else:
            return -goal.get_alpha()*ma.exp(-0.5*ma.pow( (self.get_distance_to_object(goal=goal, pt_x=pt_x, pt_y=pt_y) / goal.get_sigma()),2.0))

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
            return self._cost_function_for_goal(goal) + self._cost_function_for_obstacles(obstacles)

        else:
            return self._cost_function_for_goal(goal, pt_x=pt_x, pt_y=pt_y) + self._cost_function_for_obstacles(obstacles, pt_x=pt_x, pt_y=pt_x)

    def get_distance_to_object(self, goal, pt_x=None, pt_y=None):
        """
        This function is responsible for calculating distance to goal of a
        specified point. If pt_x and pt_y are not passed as a argument
        then x and y coordinate of robot will be used.
        
        Parameters
        ----------
        goal : [Goal]
            [Object of class Goal]
        pt_x : [float], optional
            [Position in x-axis], by default None
        pt_y : [float], optional
            [Position in y-axis], by default None
        
        Returns
        -------
        [float]
            [distance of a point from goal]
        """

        if pt_x is None:
            return ma.sqrt(ma.pow((self.get_coordinate("x")-goal.get_coordinate("x")),2)+ma.pow((self.get_coordinate("y")-goal.get_coordinate("y")),2))

        else:
            return ma.sqrt(ma.pow((pt_x-goal.get_coordinate("x")),2)+ma.pow((pt_y-goal.get_coordinate("y")),2))

    def _update_artificial_points_coordinates(self):
        """
        This function is responsible for updating artificial points of robot.
        """

        for num in range(1, self._npts+1):
            self._artificial_point_x[0][num-1] = self.get_coordinate("x") + (self._step_size*ma.cos(ma.pi*self._theta[0][num-1]/180.0))
            self._artificial_point_y[0][num-1] = self.get_coordinate("y") + (self._step_size*ma.sin(ma.pi*self._theta[0][num-1]/180.0))

    def _update_artificial_points(self, goal, obstacles):
        """
        This function is responsible for updating potential, distance to goal
        , error calculation, and fitness of artificial points.
        
        Parameters
        ----------
        goal : [Goal]
            [Object of class Goal]
        obstacles : [list]
            [List of obstalce objects]
        """

        for num in range(0, self._npts):
            self._j_obst_artificial_points[0][num] = self._cost_function_for_obstacles(obstacles=obstacles, pt_x=self._artificial_point_x[0][num], pt_y=self._artificial_point_y[0][num])
            self._j_goal_artificial_points[0][num] = self._cost_function_for_goal(goal=goal, pt_x=self._artificial_point_x[0][num], pt_y=self._artificial_point_y[0][num])
            self._jt_artificial_points[0][num] = self._j_obst_artificial_points[0][num] + self._j_goal_artificial_points[0][num]
            self._dtg_artificial_points[0][num] = self.get_distance_to_object(goal=goal, pt_x=self._artificial_point_x[0][num], pt_y=self._artificial_point_y[0][num])
        
        self._calculate_error(goal=goal, obstacles=obstacles)
        self._calculate_fitness()

    def _calculate_error(self, goal, obstacles):
        """
        This function is responsible for calculating error in potential and
        distance to goal of artificial points w.r.t goal and obstacles.
        
        Parameters
        ----------
        goal : [Goal]
            [Object of class Goal]
        obstacles : [list]
            [List of obstalce objects]
        """

        JT = self._cost_function_for_obstacles(obstacles) + self._cost_function_for_goal(goal)
        DTG = self.get_distance_to_object(goal=goal)

        for num in range(0, self._npts):
            self._err_jt[0][num] = self._jt_artificial_points[0][num] - JT
            self._err_dtg[0][num] = self._dtg_artificial_points[0][num] - DTG

    def _calculate_fitness(self):
        """
        This function is responsible for calculating fitness based on error
        of artificial points w.r.t goal and obstacles.
        """
        
        for num in range(0, self._npts):
            self._fitness[0][num] = - self._err_dtg[0][num]

    def decide_next_move(self, goal, obstacles):
        """
        This function is responsible for deciding next move of robot based on
        potential, and distance to goal of all artificial potential points w.r.t
        goal and obstacles.
        
        Parameters
        ----------
        goal : [type]
            [description]
        obstacles : [type]
            [description]
        """

        self._update_artificial_points_coordinates()

        self._update_artificial_points(goal=goal, obstacles=obstacles)

    def take_next_move(self):
        """
        This function is responsible for checking criteria for moving robot
        to next position based on a point for which the distance error is 
        negative and the potential error is smallest and negative.
        """

        check = 0
        for num in range(0, self._npts):
            k = np.argmax(self._fitness[0])
            if (self._err_jt[0][k]<0):
                check = check + 1
                self._pos_x = self._artificial_point_x[0][k]
                self._pos_y = self._artificial_point_y[0][k]
            else:
                self._fitness[0][k] = 0.0

        if check == 0:
            print(self._error_message)
        else:
            print(self._confirm_message)