#!/usr/bin/env python

"""
Main script

Contains: main function

Author: Yasim Ahmad(yaaximus)

Email: yasim.ahmed63@yahoo.com
"""

from Robot import Robot
from Object import Object
import matplotlib.pyplot as plt
from utils.positional import Position


def main():
    """
    This is the function for instantiating object of Robot, Goal, and Obstacles.
    This programs runs until distance of robot is less then 0.9 to goal and 
    during this process it keeps on deciding next move and then exectuing it.
    """

    iteration_no = 1.0
    x_limit = 20.0
    y_limit = 20.0

    obstacle1 = Object(Position(x=9.0, y=5.0), sigma=1.0)
    obstacle2 = Object(Position(x=9.0, y=8.0), sigma=1.0)
    obstacle3 = Object(Position(x=14.0, y=15.0), sigma=1.0)
    obstacle4 = Object(Position(x=14.0, y=18.0), sigma=1.0)
    obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]
    goal = Object(Position(x=18.0, y=12.0), sigma=2.0)
    robot = Robot(Position(x=5.0, y=5.0),
                  sensor_range=2.0, num_of_artif_pts=60)

    plt.figure()
    plt.axis([0.0, x_limit, 0.0, y_limit])

    while robot.position.calculate_distance(other=goal) > 0.9:

        plt.plot(robot.position.x, robot.position.y,
                 "bo", markersize=x_limit/1.5)
        plt.plot(goal.position.x, goal.position.y, "go",
                 markersize=x_limit*goal.get_sigma())
        plt.plot(obstacle1.position.x, obstacle1.position.y,
                 "ro", markersize=x_limit*obstacle1.get_sigma())
        plt.plot(obstacle2.position.x, obstacle2.position.y,
                 "ro", markersize=x_limit*obstacle2.get_sigma())
        plt.plot(obstacle3.position.x, obstacle3.position.y,
                 "ro", markersize=x_limit*obstacle3.get_sigma())
        plt.plot(obstacle4.position.x, obstacle4.position.y,
                 "ro", markersize=x_limit*obstacle4.get_sigma())

        plt.legend(('Robot', 'Goal', 'Obstacle'), loc='lower right',
                   fontsize='small', numpoints=1, markerscale=0.5, labelspacing=1)

        robot.decide_next_move(goal=goal, obstacles=obstacles)
        robot.take_next_move()
        plt.draw()
        plt.savefig("./docs/images_for_gif/"+str(iteration_no)+".png")
        iteration_no += 1.0
        plt.pause(0.02)


if __name__ == "__main__":

    main()
