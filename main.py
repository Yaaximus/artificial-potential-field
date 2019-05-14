#!/usr/bin/env python

"""
Main script

Contains: main function

Author: Yasim Ahmad(mostwanted999)

Email: yasim.ahmed63@yahoo.com
"""

from Robot import Robot
from Object import Object
import matplotlib.pyplot as plt

def main():
    """
    This is the function for instantiating object of Robot, Goal, and Obstacles.
    This programs runs until distance of robot is less then 0.9 to goal and 
    during this process it keeps on deciding next move and then exectuing it.
    """

    obstacle1 = Object(pos_x=9.0, pos_y=5.0, sigma=1.0)
    obstacle2 = Object(pos_x=9.0, pos_y=8.0, sigma=1.0)
    obstacle3 = Object(pos_x=14.0, pos_y=15.0, sigma=1.0)
    obstacle4 = Object(pos_x=14.0, pos_y=18.0, sigma=1.0)
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]
    goal = Object(pos_x=18.0, pos_y=12.0, sigma=2.0)
    robot = Robot(pos_x=5.0, pos_y=5.0, sensor_range=2.0, npts=60)

    plt.figure()
    plt.axis([0, 20, 0, 20])
    
    iter = 1
    while robot.get_distance_to_object(goal=goal) > 0.9:

        plt.plot(robot.get_coordinate("x"), robot.get_coordinate("y"),"o")
        plt.plot(obstacle1.get_coordinate("x"), obstacle1.get_coordinate("y"),"o")
        plt.plot(obstacle2.get_coordinate("x"), obstacle2.get_coordinate("y"),"o")
        plt.plot(obstacle3.get_coordinate("x"), obstacle3.get_coordinate("y"),"o")
        plt.plot(obstacle4.get_coordinate("x"), obstacle4.get_coordinate("y"),"o")
        plt.plot(goal.get_coordinate("x"), goal.get_coordinate("y"),"o")

        robot.decide_next_move(goal=goal, obstacles=obstacles)
        robot.take_next_move()
        plt.draw()
        plt.savefig("./images_for_gif/"+str(iter)+".png")
        iter += 1
        plt.pause(0.1)

if __name__ == "__main__":

    main()