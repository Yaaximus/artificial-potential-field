import math as ma

class Goal(object):
    """
    This class encorporates the capability of defining Goal object.
    """

    def __init__(self, pos_x=9.0, pos_y=9.0, sigma=4.0):
        """
        This is the constructor of Goal class.
        
        Parameters
        ----------
        pos_x : float, optional
            [Position in x-axis of Goal], by default 9.0
        pos_y : float, optional
            [Position in y-axis of Goal], by default 9.0
        sigma : float, optional
            [Width of Attractant], by default 4.0
        """

        self._pos_x = pos_x
        self._pos_y = pos_y
        self._sigma = sigma
        self._alpha = 1.0 / (self._sigma*ma.sqrt(2.0*ma.pi))

    def get_coordinate(self, coordinate=None):
        """
        This function returns a specified coordinate of Attractant.
        
        Parameters
        ----------
        coordinate : [char], optional
            ["x" or "y" position of attractant], by default None
        
        Returns
        -------
        [float]
            [position of attractant in coordinate frame]
        """

        return self._pos_x if coordinate == "x" else self._pos_y

    def get_alpha(self):
        """
        This function is responsible for returning Depth of attractant
        
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