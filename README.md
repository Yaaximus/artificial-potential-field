# Python implmentation of Artificial Potential Field

## Running instruction:
Run the ArtificialPotentialField.py file from directory. User can defined
objects like Robot, Goal, Obstacles in this file.

## Requirements
- python
- numpy

## Features
- Supports multiple obstacle objects 
- Single robot and goal object

## Fixed issues:
- bug fixed in take_next_move function of Robot class

## Example for usage: 
#### You can define objects, robot, goal and obstacle's in main function like:
obstacle1 = Obstacle(pos_x=9.0, pos_y=5.0, sigma=1.0)
obstacle2 = Obstacle(pos_x=9.0, pos_y=8.0, sigma=1.0)
obstacle3 = Obstacle(pos_x=14.0, pos_y=15.0, sigma=1.0)
obstacle4 = Obstacle(pos_x=14.0, pos_y=18.0, sigma=1.0)
    
obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]

goal = Goal(pos_x=18.0, pos_y=12.0, sigma=2.0)

my_robot = Robot(pos_x=5.0, pos_y=5.0, sensor_range=2.0, npts=60)