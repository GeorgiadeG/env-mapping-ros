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
samplingSize = 2
currentSampleIter =0
dataTable = []
averageInitialAngle = 0
averageIncrement = 0
averagedMinRange =0
averagedMaxRange = 0
counterTable = [0]*760
pygame.init()
screen = pygame.display.set_mode(SIZE)

def plotData():
	# Data is a 2d table, we want to average the values of each column and then plot them
	averagedRanges = [0] * 760
	counterTable = [0] * 760
	screen.fill(WHITE)

	for i in range (0, samplingSize):
		for j in range (0, 760):
			if dataTable[i][j] <= 12 and dataTable[i][j] >= 0.15:
				averagedRanges[j] += dataTable[i][j]
				counterTable[j] += 1
			
	# Starting from the average initial angle and the average increment, we calculate the angle of each point
	# and then we plot the points
	for i in range (0, 760):
		if counterTable[i] > 0:
			angle = -PI + (i * averageIncrement)
			x = math.cos(angle) * averagedRanges[i] / counterTable[i]
			y = math.sin(angle) * averagedRanges[i] / counterTable[i]
			
			pygame.draw.rect(screen, RED, (x * 100 + SCREEN_SIZE/2, y * 100  + SCREEN_SIZE/2, squareSize, squareSize))

	# Draw the initial point of the lidar
	pygame.draw.rect(screen, GREEN, (SCREEN_SIZE/2, SCREEN_SIZE/2, squareSize, squareSize))
	# Plot the data
	pygame.display.flip()



def callback(data):

	# If the data doesnt have 760 entries we exit
	if len(data.ranges) != 760:
		return

	#currAngle = data.angle_min
	global dataTable
	global currentSampleIter
	global averageInitialAngle
	global averageIncrement
	if currentSampleIter == samplingSize:
		plotData()
		currentSampleIter = 0
		averageInitialAngle = 0
		averageIncrement = 0
		dataTable.clear()
	currentSampleIter+=1
	dataTable.append(data.ranges)
	averageIncrement += data.angle_increment/samplingSize
	averageInitialAngle += data.angle_min/samplingSize

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('scan', LaserScan, callback)
    rospy.spin()

if __name__ == '__main__':
   listener()