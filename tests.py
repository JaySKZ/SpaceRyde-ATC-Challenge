#!/usr/bin/env python
import random
import numpy as np

from scipy.stats import uniform
import matplotlib.pyplot as plt
from skimage import draw

arr = np.zeros((10, 10))
rr, cc = draw.circle(3, 3, radius=2, shape=arr.shape)
arr[rr, cc] = 1

print(arr)

# d = {(10,10): 'hi'}

# for k in d:
#     print(k)



# SCALING = 0.1

# SCREENWIDTH  = 2100
# SCREENHEIGHT = 2100

# plane_dict = {}

# def spawnCallback():
#     R = 10000*SCALING
#     theta = uniform.rvs(0, 2*np.pi, size=1)

#     x = int(SCREENWIDTH/2 + R*np.cos(theta))
#     y = int(SCREENHEIGHT/2 + R*np.sin(theta))

#     id = ''.join(random.choice('0123456789ABCDEF') for i in range(6))

#     data = {id: {'Location': (x,y), 'Holding': False, 'Landing': False}}
#     plane_dict.update(data)

#     message = message_converter.convert_dictionary_to_ros_message('std_msgs/String', plane_dict)
#     return message

# def updateCallback():
#     rospy.init_node('/')

# def plane():
#     rospy.init_node('/plane', anonymous=False)

#     sub = rospy.Subscriber('/plane/query', Bool)

#     if sub == True:


#     global SPAWNRATE = rospy.Rate(2) #2Hz i.e. spawn a plane every 0.5 seconds

#     rospy.spin()