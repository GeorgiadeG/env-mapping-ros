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
dataTable = [(0,0)]*760
counterTable = [0]*760
pygame.init()
screen = pygame.display.set_mode(SIZE)
screen.fill(WHITE)

def callback(data):
	currAngle = data.angle_min
	
	#if the size of data.ranges is not 760, then it is not a full 360 degree scan so exit the function
	if len(data.ranges) != 760:
		return
	
	# If this is larger than zero that means we have to decrease
	# and if it is negative we have to increase the values
	angleIncr = data.angle_increment
	
	# loop from 0 to data.ranges.length
	for i in range(0, len(data.ranges)):
		if (data.ranges[i] >= data.range_min and data.ranges[i] <= data.range_max):
			x = (math.cos(currAngle) * data.ranges[i] * 100) + SCREEN_SIZE/2
			y = (math.sin(currAngle) * data.ranges[i] * 100) + SCREEN_SIZE/2

			#increment the counter for the current angle
			counterTable[i] += 1
			
			#take the values x and y from the dataTable
			temp_x = dataTable[i][0]
			temp_y = dataTable[i][1]

			#add the temp and x,y and puth them back to the table
			dataTable[i] = (temp_x + x, temp_y + y)

			currAngle+=angleIncr
	
	global currentSampleIter
	currentSampleIter = (currentSampleIter + 1) % samplingSize
	if (currentSampleIter == 0):
		#average the values in the table with the corresponding counter
		for i in range(0,760):
			if counterTable[i] != 0:
				dataTable[i] = (dataTable[i][0]/counterTable[i], dataTable[i][1]/counterTable[i])
				# draw rectangle from dataTable[i]
				pygame.draw.rect(screen, GRAY, (dataTable[i][0] - squareSize, dataTable[i][1] - squareSize, squareSize*2, squareSize*2))
				dataTable[i] = (0,0)
				counterTable[i] = 0
			else:
				dataTable[i] = (0,0)
				counterTable[i] = 0

		pygame.display.flip()
		screen.fill(WHITE)
	#pygame.display.flip()

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
   listener()