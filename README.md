# SpaceRyde ATC Challenge

This repo is my submission for SpaceRyde's ATC challenge for the software internship opening. It has been great fun programming this, and it was a great and rigorous endeavor in many respects.

## Summary
The challenge is to program a ATC (air traffic control) system in a 2D environment that can meet the following requirements:
- Keeps track of all airplanes’ positions (latitude, longitude) within the traffic control zone.
The traffic control zone is a circle of radius 10km.
- Queues airplanes for landing based on time of arrival into the traffic control zone.
- Commands airplanes to go for landing (when it is their turn) to one of two runways.
- Otherwise commands airplanes to fly in a circular “holding pattern” which is a circle with
a radius of 1km around a suitable point.
- The ATC must never allow for planes to come within 100m of each other , whether
flying or landing.

My system is implemented in Python using PyGame to simulate the environment. The code is made to run in a modular, object oriented approach with attention to flexibility to allow for possible changes in system paramenters and conditions.

## Installation and Running
The repo includes three relevant .py files: 
- **env.py**: initializes the PyGame UI to visualize the ATC system, runs the control algorithm and system simulation
- **control.py**: includes all classes and functions pertinent to the control of the system
- **astar.py**: A* path planning algorithm class based on code from [this A* implementation template](https://gist.github.com/Nicholas-Swift/003e1932ef2804bebef2710527008f44)

To see the system running, simply clone/download this repo and run the **env.py** file. 

Prior to running, please make sure the machine is running Python3 and the following modules are installed:
- numpy
- queue
- pygame
- random
- scipy
- skimage

## Implementation Details
The approach taken to this problem was to cleanly modularize all the components of the system, moving or still, and to have a centralized ATC that stores information, issues commands and deals with system states. This allows for the components in the system to be fully decoupled during testing, as the ATC facilitates all communication.

A brief original attempt of implementation with ROS was made. The two strategies attempted was to have:

1. All airplanes and airstrips be ROS nodes that subscribe to an ATC node, which publishes information to the planes. This ran into issues with continually initializing new nodes while keeping track of existing ones, and destorying ones that were no longer required.
2. All airplnes and airstrips belong to a single data-storing node that communicates with an ATC node. This ran into issues with communications arms races, where the order in which certain data were not to be guaranteed to always occur in the same way.

Instead, a class-based implmentation was chosen. The idea here is to mimic the idealized ROS architecture using classes isntead of ROS nodes. Hence, in the control.py file, we have classes for:
- Planes: with parameters for a unique identifier (6 digit alphanumeric string), current location, and given constraints such as speed, holding radius and safety radius.
- Airfields: with geometric parameters
- Parking (holding) spots: with geometric parameters
- ATC: with interprets the system states and give paths and commands to plane objects

The system procedure is roughly as follows:
1. Spawn world with set physical parameters, visualizing with our PyGame UI
2. 


