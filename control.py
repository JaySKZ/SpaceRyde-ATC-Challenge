#!/usr/bin/env python
import pygame
import queue
import numpy as np
import random
from scipy.stats import uniform
from skimage import draw

from astar import *

class circle:
    def __init__(self, origin, radius):
        self.origin = origin
        self.radius = radius

class rectangle:
    def __init__(self, top_left, bottom_right):
        self.top_left = top_left
        self.bottom_right = bottom_right

class airfield:
    def __init__(self, id, top_left, bottom_right):
        self.id = id
        self.vacant = True
        self.top_left = top_left
        self.bottom_right = bottom_right

    def get_status(self):
        return self.vacant

    def set_status(self, status):
        self.vacant = status

    def get_hitbox(self):
        return rectangle(self.top_left, self.bottom_right)

class plane:
    #Maybe add a landing flag later if needed
    def __init__(self, id, location, holding_rad = 1050*0.05, safety_rad = 50*0.05):
        self.id = id
        self.location = location
        self.holding = False
        self.holding_rad = holding_rad
        self.safety_rad = safety_rad

        self.path = queue.SimpleQueue()

    def get_location(self):
        return self.location

    def get_hitbox(self):
        if (self.holding):
            return circle(self.location, self.holding_rad)
        else:
            return circle(self.location, self.safety_rad)

    def move_plane(self):
        if not self.holding:
            if not self.path.empty():
                self.location = self.path.get()
                return True
            else:
                return False


class parking_spots:
    def __init__(self, airfields, active_rad = 10000*0.05, holding_rad = 1050*0.05):
        self.airfields = airfields
        self.active_rad = active_rad
        self.holding_rad = holding_rad

        self.parking_spots = {}

        #Create holding spots on edge of smaller concentric circles to active area
        for i in range(int(np.round(active_rad / (2*holding_rad)))):
            #Special case for holding circle in center of active area
            if i == 0:
                if self.check_spot((0,0),holding_rad, airfields):
                    self.parking_spots.update({(0,0): True})
                continue

            #General case
            radius = i*2*holding_rad
            circum = np.pi*2*radius

            num_spots = circum // (2*holding_rad)
            arc_length = circum / num_spots

            dTheta = arc_length / radius
            print(i,num_spots)
            for j in range(int(num_spots)):
                if self.check_spot((radius*np.cos(j*dTheta), radius*np.sin(j*dTheta)),holding_rad, airfields):
                    self.parking_spots.update({(radius*np.cos(j*dTheta), radius*np.sin(j*dTheta)): True})

    def check_spot(self, origin, holding_rad, airfields):
        #Remove parking spots that overlap airfields
        square_circle_topleft = (origin[0]-holding_rad,origin[1]-holding_rad)
        square_circle_bottomright = (origin[0]+holding_rad,origin[1]+holding_rad)
        for airfield in airfields:
            hitbox = airfield.get_hitbox()
            cond_1 = hitbox.top_left[0] < square_circle_bottomright[0] and hitbox.top_left[1] < square_circle_bottomright[1]
            cond_2 = hitbox.bottom_right[0] > square_circle_topleft[0] and hitbox.bottom_right[1] > square_circle_topleft[1]
            if cond_1 and  cond_2:
                return False
            else:
                return True
        return True

    def parking_list(self):
        return self.parking_spots

    def get_spots(self):
        return self.parking_spots.keys()

    def set_status(self, spot, status):
        self.parking_spots[spot] = status


class ATC:
    def __init__(self, airfields, planes = {}):
        self.FPS = 60
        self.SCALING = 0.05
        self.VEL = 140
        self.SCALED_VEL = self.VEL * self.SCALING
        self.SCREENWIDTH  = int(21000*self.SCALING)
        self.SCREENHEIGHT = int(21000*self.SCALING)
        self.RUNWAY_WIDTH = 100
        self.RUNWAY_LENGTH = 500

        self.airfields = airfields
        self.planes = planes
        self.plane_queue = queue.SimpleQueue()

        #Technically a flight log, keeps track of which planes landed at which airports
        self.avail_airports = {}

    #Add an active plane to the queue
    def add_plane(self, id, location):
        self.plane_queue.put(id)
        self.planes[id] = plane(id, location)

    #Find a valid holding spot for a plane (closest to it)
    def find_parking(self, location, parking_spots):
        min_dist = 1000000
        spot = (2000,2000)

        for k in parking_spots.keys():
            if parking_spots[k]:
                dist = np.sqrt((location[0][0]-k[0])**2 + (location[1][0]-k[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    spot = k

        return spot

    #Direct plane to holding spot
    def hold_plane(self, plane, parking_spots):
        plane_location = self.planes[plane.id].get_location()
        plane.holding = True

        holding_spot = self.find_parking(plane_location, parking_spots)

        parking_spots.set_status[holding_spot] = False

        holding_path = self.plan_path(plane_location, parking_spots, holding_spot)

        for i in range(len(holding_path)):
            if i % np.round(self.SCALED_VEL) == 0:
                self.planes[plane.id].path.put(holding_path[i])

    #Attempt landing
    def try_landing(self, parking_spots):
        for airfield in self.airfields:
            if not (self.plane_queue.empty()):
                if (airfield.get_status()):
                    self.land(airfield, parking_spots)
                    return True

        return False

    #Landing sequence
    def land(self, airfield, parking_spots):
        landing_airfield = self.airfields[airfield.id]
        landing_airfield.set_status(False)

        plane_id = self.plane_queue.get()
        self.planes[plane_id].holding = True
        plane_location = self.planes[plane_id].get_location()

        int(plane_location[0])
        int(plane_location[1])

        self.avail_airports.update({plane_id: airfield})

        hitbox = landing_airfield.get_hitbox()
        destination = (hitbox.top_left[0] + 50*self.SCALING, hitbox.top_left[1] - 1)

        landing_path = self.plan_path(plane_location, parking_spots, destination)

        runway_seq = [(int(hitbox.top_left[0] + self.RUNWAY_WIDTH*self.SCALING), int(hitbox.top_left[1]+y)) for y in range(0,int(self.RUNWAY_LENGTH*self.SCALING))]

        landing_path.extend(runway_seq)

        for i in range(len(landing_path)):
            if i % np.round(self.SCALED_VEL) == 0:
                self.planes[plane_id].path.put(landing_path[i])


    #Uses the A* algorithm to plan a path for plane given collisions and destination
    def plan_path(self, location, parking_spots, destination):
        #We make a "maze", or a collision map for our active area
        nav_map = np.zeros((self.SCREENWIDTH, self.SCREENHEIGHT))

        #Make active holding zones in navigation map
        for k in parking_spots.keys():
            if not parking_spots[k]:
                rr, cc = draw.circle(int(k[0]), int(k[1]), radius=int(1050*0.05), shape=nav_map.shape)
                nav_map[rr,cc] = 1

        #Make airfields in navigation map
        for airfield in self.airfields:
            hitbox = airfield.get_hitbox()
            start = hitbox.top_left
            extent = (int(start[0]+500*self.SCALING), int(start[1]+100*self.SCALING))
            rr, cc = draw.rectangle(start, extent=extent, shape=nav_map.shape)
            nav_map[int(rr),int(cc)] = 1
        int(location[0])
        int(location[1])
        int(destination[0])
        int(destination[1])
        return astar(nav_map, location, destination)

    def move_timestep(self):
        for plane in self.planes:
            plane_id = plane.id
            if plane.move_plane() == False:
                airfield = self.avail_airports[plane_id]
                airfield.set_status(True)

                del self.planes[plane.id]
            else:
                plane.move_plane()

    def decision(self, parking_spots):
        if self.try_landing:
            self.try_landing(parking_spots)
        else:
            self.hold_plane(plane, parking_spots)

        self.move_timestep()
        state = [plane.location for plane in self.planes]
        return state
