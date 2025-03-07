<p align="center">
	<img src="https://github.com/RohitMovva/VexAutonomousPlanner/blob/main/assets/flip_logo.png?raw=true" alt="IntroIcon" width="125">
</p>

# VexAutonomousPlanner

An application for easy, action oriented, generation of paths for VEX robotics.

Created by team 77717F Flip

## Project Description
This application acts a path planner through which you can draw out paths for the robot to take. The path creation process is broken down below:
- User inputs nodes (shift+click to create node)
- User marks certain nodes with actions (right click node to edit actions)
- Nodes are auto connected with Quintic Hermite Splines
- Upon saving the path, a motion profile is auto generated and a trajectory is stored

## Features
- Create, move, and delete nodes
- Automatically generated quintic Hermviite splines (no need to deal with control points)
- Assign custom actions to nodes, such as turn, wait, reverse, and more
- Generate a 2d motion profile of the path
- View graphs generated by the 2d motion profile including position, velocity, heading, etc...
- Store the motion profile to a header file to use in your own codebase
- Preview robot on the path
- Customize robot constraints
- Mirror path to easily mirror autons
- Save and acess paths locally
- Convert path to string to easily send to others

<p align="center">
  <img src="https://github.com/RohitMovva/VexAutonomousPlanner/blob/main/assets/demo_images/full_view.png?raw=true" alt="Full Application Mockup" width="700">
  <br>
  <em>The main interface showing the path planning workspace</em>
</p>

<p align="center">
  <img src="https://github.com/RohitMovva/VexAutonomousPlanner/blob/main/assets/demo_images/node_settings.png?raw=true" alt="Node Creation" width="300">
  <br>
  <em>Creating and connecting nodes with auto-generated splines</em>
</p>

<p align="center">
  <img src="https://github.com/RohitMovva/VexAutonomousPlanner/blob/main/assets/demo_images/velocity_profile.png?raw=true" alt="Motion Profile" width="700">
  <br>
  <em>Visualizing the generated motion profile</em>
</p>


## Installation
The project can be accessed either by cloning or by downloading the latest excecutable from the Releases tab on the side. (Project is still in heavy development)

On the first run of the application you will be prompted to select a "routes" folder, this is just a folder where paths you generate will be stored. You will also be prompted to select another folder to save the trajectories generated based on the path, which can be used directly within your codebase.

## Contribution and Collaboration
Currently the project is currently solely maintained and written by team 77717F. If interested in contributing or if you have any other questions or suggestions feel free to reach out to me on Discord (communistpoultry).

## Credits
This project was made possible by:

- [Jonathan Bayless](https://github.com/baylessj/robotsquiggles)
- [Jerry Lum](https://github.com/Jerrylum/path.jerryio)
- [FRC Team 254: The Cheesy Poofs](https://www.youtube.com/watch?v=8319J1BEHwM)
