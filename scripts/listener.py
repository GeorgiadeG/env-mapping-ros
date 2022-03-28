#!/usr/bin/env python

import rospy
import pygame
import sys
import math
from sensor_msgs.msg import LaserScan
from pygame.locals import *

SCREEN_SIZE = 1000
SIZE = SCREEN_SIZE, SCREEN_SIZE
RED = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY = (150, 150, 150)
WHITE = (255, 255, 255)

#define PI
PI = 3.14159265358979

squareSize = 6
samplingSize = 5
currentSampleIter =0

pygame.init()
screen = pygame.display.set_mode(SIZE)
screen.fill(WHITE)

def callback(data):
	currAngle = data.angle_min
	
	# If this is larger than zero that means we have to decrease
	# and if it is negative we have to increase the values
	angleIncr = data.angle_increment
	for l in data.ranges:
		if (l >= data.range_min and l <= data.range_max):
			x = (math.cos(currAngle) * l * 100) + SCREEN_SIZE/2
			y = (math.sin(currAngle) * l * 100) + SCREEN_SIZE/2		
			rect = Rect(x,y,squareSize,squareSize)
			currAngle+=angleIncr
			pygame.draw.rect(screen, RED, rect)
	
	currentSampleIter = (currentSampleIter + 1) % samplingSize
	if (currentSampleIter == 0):
		pygame.display.flip()
		screen.fill(WHITE)
	#pygame.display.flip()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
   listener()