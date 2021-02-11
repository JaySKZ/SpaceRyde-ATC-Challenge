#!/usr/bin/env python
import pygame
import queue
import numpy as np
import random
from scipy.stats import uniform
from skimage import draw

from astar import astar

class circle:
    def __init__(self, origin, radius):
        self.origin = origin
        self.radius = radius

class rectangle:
    def __init__(self, top_left, bottom_right):
        self.top_left = top_left
        self.bottom_right = bottom_right

class airfield:
    def __init__(self, top_left, bottom_right):
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
    def __init__(self, id, location, path = [], holding_rad = 1050*0.05, safety_rad = 50*0.05):
        self.id = id
        self.location = location
        self.path = path
        self.holding = False
        self.holding_rad = holding_rad
        self.safety_rad = safety_rad

    def get_location(self):
        return self.location

    def set_path(self, path):
        self.path = path

    def get_path(self):
        return self.path

    def get_hitbox(self):
        if (self.holding):
            return circle(self.location, self.holding_rad)
        else:
            return circle(self.location, self.safety_rad)

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
        self.VEL = 140*1000/3600
        self.SCALED_VEL = self.VEL * self.SCALING
        self.SCREENWIDTH  = int(21000*self.SCALING)
        self.SCREENHEIGHT = int(21000*self.SCALING)

        self.airfields = airfields
        self.planes = planes
        self.plane_queue = queue.SimpleQueue()

    def add_plane(self, id, location):
        self.plane_queue.put(id)
        self.planes[id] = plane(id, location)

    def find_parking(self, id, parking_spots):
        location = self.planes[id].get_location()

        min_dist = 1000000
        spot = (2000,2000)

        for k in parking_spots.keys():
            if parking_spots[k]:
                dist = np.sqrt((location[0][0]-k[0])**2 + (location[1][0]-k[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    spot = k

        return spot

    def land(self, airfield):
        airfield.set_status(False)
        plane_id = self.plane_queue.get()

        # find a path to airport
        # set path of plane
        # land

        del self.planes[plane_id]
        airfield.set_status(True)

    def try_landing(self):
        for airfield in self.airfields:
            if not (plane_queue.empty()):
                if (airfield.get_status()):
                    self.land(airfield)
                    pass

    def plan_path(self, id, parking_spots, destination):
        holding_rad = 1050*0.05
        location = self.planes[id].get_location()

        nav_map = np.zeros(self.SCREENWIDTH, self.SCREENHEIGHT)

        for k in parking_spots.keys():
            if not parking_spots[k]:
                rr, cc = draw.circle(int(k[0]), int(k[1]), radius=1050*0.05, shape=arr.shape)
                nav_map[rr,cc] = 1

        return astar(nav_map, location, destination)

    def map_hitbox(self):
        hitboxes = [plane.get_hitbox() for plane in self.planes].extend([airfield.get_hitbox() for airfield in self.airfields])



def main():
    pass

# if __name__ = "__main__":
#     main()


