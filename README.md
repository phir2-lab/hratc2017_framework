# HRATC2017 Framework

This repository contains the framework used in the [HRATC2017](http://inf.ufrgs.br/hratc2017/HRATC2017/Welcome.html) robot programming competition, with all description, simulation using Gazebo and judge components.

## Organization

- **description**

The complete description of the robot with all its sensors (urdf files) and the description of the environment (world files), along with all models and meshes used in the descriptions.

- **launch**

All launch files for executing the competition.

- **settings**

All parameters of the judge (such as number of mines), along with minefield corners and obstacles positions in GPS coordinates.

- **src**

Source code of all simulation and evaluation (judge) components.

- hratc.rviz

rviz configuration file showing the minefield, the robot, its sensor measurements, and all metrics used by the judge of the challenge (mines positions, true and false detections, etc).

## How to run the HRATC framework?

For complete installation instructions check [here](http://inf.ufrgs.br/hratc2017/HRATC2017/Simulator.html).
But if you already installed the framework, you just need to run the file "run_simulation.launch" to start the simulation of the robot in gazebo, the simulation of the metal detector, the judge, and the visualization in rviz.

`roslaunch hratc2017_framework run_simulation.launch`

You can change the type of the simulated scenario to increase or decrease the difficulty of the robot control, and better prepare for the real environment. See [here](https://github.com/ras-sight/hratc2017_framework/wiki/Minefield-configuration).

## Contributions

 - Robot and sensors configuration by Renan Maffei and Gonçalo Cabrita;
 - World modelling by Mathias Mantelli, Renata Neuland and Diego Pittol;
 - Configuration of minefield by Mathias Mantelli and Renan Maffei;
 - Simulation of the metal detector signal by Renan Maffei;
 - Judge by Renan Maffei; 
 - Transformation of GPS coordinates and UTM by Gonçalo Cabrita;
 - Occupancy grid for coverage area by Vitor Jorge;
