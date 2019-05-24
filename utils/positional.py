"""
Position Class

Author: Yasim Ahmad(yaaximus)

Email: yasim.ahmed63@yahoo.com
"""
import math as ma


class Position:
    """
    This class encapsulates the capability of defining position of object in
    coordinate system and calculating distance b/w two objects
    """

    def __init__(self, x=1.0, y=1.0):
        """
        This is the constructor of Position class
        
        Parameters
        ----------
        x : float, optional
            [Position in x-axis of Object], by default 1.0
        y : float, optional
            [Position in y-axis of Object], by default 1.0
        """

        self.x = x
        self.y = y

    def calculate_distance(self, other, x=None, y=None):
        """
        This function is responsible for calculating distance b/w two 
        points. If x and y are not passed as a argument
        then x and y coordinate of Position class object will be used.

        Parameters
        ----------
        other : [Object]
            [object of class Object]
        x : [float], optional
            [Position in x-axis], by default None
        y : [float], optional
            [Position in y-axis], by default None

        Returns
        -------
        [float]
            [distance b/w two points]
        """

        if x is None:
            return ma.sqrt(ma.pow((self.x-other.position.x), 2)+ma.pow((self.y-other.position.y), 2))

        else:
            return ma.sqrt(ma.pow((x-other.position.x), 2)+ma.pow((y-other.position.y), 2))
