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

My system is implemented in Python using PyGame to simulate the environment. The code is "supposed to" made to run in a modular, object oriented approach with attention to flexibility to allow for possible changes in system paramenters and conditions.

## Installation and Running
The repo includes three relevant .py files: 
- **env.py**: initializes the PyGame UI to visualize the ATC system, runs the control algorithm and system simulation
- **control.py**: includes all classes and functions pertinent to the control of the system
- **astar.py**: A* path planning algorithm class based on code from [this A* implementation template](https://gist.github.com/Nicholas-Swift/003e1932ef2804bebef2710527008f44)

To see the system running, simply clone/download this repo and run the **env.py** file. Note that parts of the code may run modularly, but will not run as an entire entity unfortunately.

The following Python3 modules are used in the code:
- numpy
- queue
- pygame
- random
- scipy
- skimage

## Implementation Details
The approach taken to this problem was to cleanly modularize all the components of the system, moving or still, and to have a centralized ATC that stores information, issues commands and deals with system states. This allows for the components in the system to be fully decoupled during testing, as the ATC facilitates all communication. The decoupling ended up being extremely important, as the code unfortunately does not run as a whole, but the classes more or less behave correctly in their own environments.

A brief original attempt of implementation with ROS was made. The two strategies attempted was to have:

1. All airplanes and airstrips be ROS nodes that subscribe to an ATC node, which publishes information to the planes. This ran into issues with continually initializing new nodes while keeping track of existing ones, and destorying ones that were no longer required.
2. All airplnes and airstrips belong to a single data-storing node that communicates with an ATC node. This ran into issues with communications arms races, where the order in which certain data were not to be guaranteed to always occur in the same way.

Instead, a class-based implmentation was chosen. The idea here is to mimic the idealized ROS architecture using classes instead of ROS nodes. Hence, in the control.py file, we have classes for:
- Planes: with parameters for a unique identifier (6 digit alphanumeric string), current location, and given constraints such as speed, holding radius and safety radius.
- Airfields: with geometric parameters
- Parking (holding) spots: with geometric parameters
- ATC: with interprets the system states and give paths and commands to plane objects

The system procedure is roughly as follows:
1. Spawn world with set physical parameters, visualizing with our PyGame UI
2. Run our main PyGame loop at 10 Hz (or 10 clock ticks for 60 FPS) since this is our frequency of communication
3. Spawn planes at some chosen rate (up to the max number of planes allowable in the area)
4. Queue the planes based on spawn, and let our ATC make the following decision for the plane at the front of the queue:
    1. Land planes, if possible
    2. If landing not possible, send plane to the closest available holding spot
5. Repeat for planes at the front of the queue

Note on the "holding spot" mentioned in 4(ii): the optimal approach would be to solve the circle packing problem to fill our active area with as many holding circles as possible. An even better extended approach would be to allow the circles to overlap, and make the planes fly in such a ways such that they are out of sync and will not collide on the perimeter. Both of these approaches are significantly difficult to implement, so holding spots were simply made along concentric circles of the active area, at a safe distance from each other and disallowing overlaps.

Path planning was done using the A* search algorithm, which is a greedy-based algorithm for graph traversal, as an extension to Dijkstra's algorithm. To be able to use this, we need to turn our active area into a "graph". This can be achieved by viewing the active area as a numpy array of 0's. Then, we find all our obstacles/boundaries, such as pther planes, the occupied holding circles and the airstrips (the radius of holding circles were increased by 50m to avoid collisions). Passing this, as well as our starting and desired ending coordinates into our path planning algorithm, we can receive a list of coordinates for the optimal path. To make moving simpler, this list was changed into a FIFO queue, so that at every time step we can pop a coordinate, and change the plane's current location.

The integration of the path planning ultimately did not work. The classes behave correctly as their own, and applying the A* algorithm on smaller test mazes/maps gets the correct output. However, spatial bound and communication issues arose during integration, and as today is the 7th day, there is no more time to fix this unfortunately. Chances are I will come back to this after midterms and attempt to make this work properly. 

## Final words
Thank you for the oppotunity to complete this challenge! It was a lot of fun and definitely took some mental capacity and flexibility. Due to timing constraints and conflicts with school and exams, the integration of modules ultimately was not complete, but I tried my best to develop the system with some *theoretical* strengths and resilience, which I would be happy to talk about or explain. I would also love to know how this is achieved in ROS, or using other and more efficient methods. I hope to have the chance to discuss this further, thank you!

