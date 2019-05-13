import math as ma

class Obstacle(object):
    """
    This class encorporates the capability of defining Obstacle object.
    """

    def __init__(self, pos_x=6.0, pos_y=6.0, sigma=4.0):
        """
        This is the constructor of  obstacle class.
        
        Parameters
        ----------
        pos_x : float, optional
            [Position in x-axis of Obstacle], by default 6.0
        pos_y : float, optional
            [Position in y-axis of Obstacle], by default 6.0
        sigma : float, optional
            [Width of Repellant], by default 4.0
        """

        self._pos_x = pos_x
        self._pos_y = pos_y
        self._sigma = sigma
        self._alpha = 1.0 / (self._sigma*ma.sqrt(2.0*ma.pi))

    def get_coordinate(self, coordinate=None):
        """
        This function returns a specified coordinate of Repellant.
        
        Parameters
        ----------
        coordinate : [char], optional
            ["x" or "y" position of repellant], by default None
        
        Returns
        -------
        [float]
            [position of repellant in coordinate frame]
        """

        return self._pos_x if coordinate == "x" else self._pos_y

    def get_alpha(self):
        """
        This function is responsible for returning Depth of repellant
        
        Returns
        -------
        [float]
            [Depth of repellant]
        """

        return self._alpha

    def get_sigma(self):
        """
        This function is responsible for returning Width of repellant
        
        Returns
        -------
        [float]
            [Width of repellant]
        """
    
        return self._sigma