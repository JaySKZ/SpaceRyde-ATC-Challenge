#!/usr/bin/env python
import pygame
import numpy as np
import random
from scipy.stats import uniform

FPS = 60
SCALING = 0.1 # 10:1 pixel -> meter conversion

SCREENWIDTH  = 2100
SCREENHEIGHT = 2100

SIZE = (SCREENWIDTH, SCREENHEIGHT)

VEL = 140*1000/3600
SCALEDVEL = VEL / (SCALING*FPS)

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

plane_dict={}

def main():
    global SCREEN, FPSCLOCK
    pygame.init()
    FPSCLOCK = pygame.time.Clock()
    SCREEN = pygame.display.set_mode(SIZE)
    pygame.display.set_caption('ATC')

    draw_world()

    while True:
        event = pygame.event.poll()
        if event.type == pygame.QUIT:
            break
        spawn()

        pygame.display.flip()
        # Loop at 10 Hz
        FPSCLOCK.tick(10)

    pygame.quit()

def draw_world():
    SCREEN.fill(BLACK)
    #Draw active area
    pygame.draw.circle(SCREEN, WHITE, (SCREENWIDTH/2,SCREENHEIGHT/2), 10000*SCALING, 1)
    pygame.draw.rect(SCREEN, WHITE, [SCREENWIDTH/2-35, SCREENHEIGHT/2-25, 100*SCALING, 500*SCALING],1)
    pygame.draw.rect(SCREEN, WHITE, [SCREENWIDTH/2+35, SCREENHEIGHT/2-25, 100*SCALING, 500*SCALING],1)
    
def spawn():
    R = 10000*SCALING
    theta = uniform.rvs(0, 2*np.pi, size=1)

    x = int(SCREENWIDTH/2 + R*np.cos(theta))
    y = int(SCREENHEIGHT/2 + R*np.sin(theta))

    id = ''.join(random.choice('0123456789ABCDEF') for i in range(6))

    data = {id: {'Location': (x,y), 'Holding': False, 'Landing': False}}
    plane_dict.update(data)

    #draw plane
    pygame.draw.circle(SCREEN, RED, (x,y), 50*SCALING, 1)
    
    return id, plane_dict

main()