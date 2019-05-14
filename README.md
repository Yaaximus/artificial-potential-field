# Python implmentation of Artificial Potential Field

## Running instruction:
- Run the main.py file from directory. 
- User can defined objects like Robot, Goal, Obstacles in this file.
- Check demo.gif for output

## Requirements
- python
- numpy

## Features
- Supports multiple obstacle objects 
- Single robot and goal object
- image saving capability added

## Fixed issues:
- bug fixed in take_next_move function of Robot class

## Example for usage: 
#### You can define objects, robot, goal and obstacle's in main function like:
- obstacle1 = Object(pos_x=9.0, pos_y=5.0, sigma=1.0)
- obstacle2 = Object(pos_x=9.0, pos_y=8.0, sigma=1.0)
- obstacle3 = Object(pos_x=14.0, pos_y=15.0, sigma=1.0)
- obstacle4 = Object(pos_x=14.0, pos_y=18.0, sigma=1.0)
    
- goal = Object(pos_x=18.0, pos_y=12.0, sigma=2.0)

- robot = Robot(pos_x=5.0, pos_y=5.0, sensor_range=2.0, npts=60)

## List of Obstacles:
- obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]