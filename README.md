# Python implmentation of Artificial Potential Field

- Check demo_1.gif to see the output with 4 obstacles b/w robot and goal
- Check demo_2.gif to see the output with 6 obstacles b/w robot and goal

## Running instruction:
- Run the main.py file from directory. 
- User can defined objects like Robot, Goal, Obstacles in this file.

## Requirements
- python
- numpy

## Features
- Supports multiple obstacle objects 
- Single robot and goal object
- image saving capability added
- positional class encorporated to remove duplicate code in all objects

## Fixed issues:
- bug fixed in take_next_move function of Robot class

## Example for usage: 
#### You can define objects, robot, goal and obstacle's in main function like:
- obstacle1 = Object(position(x=9.0, y=5.0), sigma=1.0)
- obstacle2 = Object(position(x=9.0, y=8.0), sigma=1.0)
- obstacle3 = Object(position(x=14.0, y=15.0), sigma=1.0)
- obstacle4 = Object(position(x=14.0, y=18.0), sigma=1.0)
    
- goal = Object(position(x=18.0, y=12.0), sigma=2.0)

- robot = Robot(position(x=5.0, y=5.0), sensor_range=2.0, npts=60)

## List of Obstacles:
- obstacles = [obstacle1, obstacle2, obstacle3, obstacle4]
