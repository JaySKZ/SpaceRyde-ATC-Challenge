#!/usr/bin/env python
import pygame
import numpy as np
import random
import control
from scipy.stats import uniform

FPS = 60
SCALING = 0.05 # 10:1 pixel -> meter conversion

SCREENWIDTH  = int(21000*SCALING)
SCREENHEIGHT = int(21000*SCALING)

SIZE = (SCREENWIDTH, SCREENHEIGHT)

VEL = 140*1000/3600
SCALEDVEL = VEL / (SCALING*FPS)

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

def main():
    global SCREEN, FPSCLOCK
    pygame.init()
    FPSCLOCK = pygame.time.Clock()
    SCREEN = pygame.display.set_mode(SIZE)
    pygame.display.set_caption('ATC')

    # airfields = [airfield(1), airfield(2)]
    # planes = {'1': plane('1', (1000,1000))}

    draw_world()

    airfield_1 = control.airfield((-350*0.05,-250*0.05), (-250*0.05,750*0.05))

    ATC = control.ATC([airfield_1])

    new_id, new_loc = spawn()

    ATC.add_plane(new_id,new_loc)

    parking_spots = control.parking_spots([airfield_1])

    print(new_loc)
    print(ATC.find_parking(new_id, parking_spots.parking_list()))

    # diction = parking_spots.parking_list()

    # print(diction.keys())

    n = 10

    #Draw parking spots
    for spot in parking_spots.get_spots():
        pygame.draw.circle(SCREEN, GREEN, (spot[0]+SCREENWIDTH/2, spot[1]+SCREENHEIGHT/2), 1000*SCALING, 1)

    while True:
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            break

        if (n == 10):
            #new_id = spawn()[0]
            n = 0
        else:
            n += 1

        pygame.display.flip()
        # Loop at 10 Hz
        FPSCLOCK.tick(10)

    pygame.quit()

def draw_world():
    SCREEN.fill(BLACK)
    #Draw active area
    pygame.draw.circle(SCREEN, WHITE, (SCREENWIDTH/2,SCREENHEIGHT/2), 10000*SCALING, 1)

    #Draw airfields
    pygame.draw.rect(SCREEN, WHITE, [SCREENWIDTH/2-(350*SCALING), SCREENHEIGHT/2-(250*SCALING), 100*SCALING, 500*SCALING],1)
    pygame.draw.rect(SCREEN, WHITE, [SCREENWIDTH/2+(250*SCALING), SCREENHEIGHT/2-(250*SCALING), 100*SCALING, 500*SCALING],1)

    # TODO init classes for airfields

def spawn():
    R = 10000*SCALING
    theta = uniform.rvs(0, 2*np.pi, size=1)

    # x = int(SCREENWIDTH/2 + R*np.cos(theta))
    # y = int(SCREENHEIGHT/2 + R*np.sin(theta))

    x = R*np.cos(theta)
    y = R*np.sin(theta)

    loc = (x,y)

    id = ''.join(random.choice('0123456789ABCDEF') for i in range(6))

    return id, loc







main()