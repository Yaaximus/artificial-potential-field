"""
Object class

Author: Yasim Ahmad(yaaximus)

Email: yasim.ahmed63@yahoo.com
"""
import math as ma
from utils.positional import Position


class Object(object):
    """
    This class encorporates the capability of defining Goal/Obstacle object.
    """

    def __init__(self, Position, sigma=4.0):
        """
        This is the constructor of Object class.

        Parameters
        ----------
        position : [position]
            [x and y coordinates of object]
        sigma : float, optional
            [Width of Attractant/Repellant], by default 4.0
        """

        self.position = Position
        self._sigma = sigma
        self._alpha = 1.0 / (self._sigma*ma.sqrt(2.0*ma.pi))

    def get_alpha(self):
        """
        This function is responsible for returning Depth of attractant/repellant

        Returns
        -------
        [float]
            [Depth of attractant]
        """

        return self._alpha

    def get_sigma(self):
        """
        This function is responsible for returning Width of attractan

        Returns
        -------
        [float]
            [Width of attractant]
        """

        return self._sigma
