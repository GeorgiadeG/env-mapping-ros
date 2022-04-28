#!/usr/bin/env python

import rospy
import pygame
import sys
import math
from sensor_msgs.msg import LaserScan
from pygame.locals import *

RED = (255, 0, 0)
GREEN = (0, 255, 0)
GRAY = (150, 150, 150)
WHITE = (255, 255, 255)
DARK_RED = (150, 0, 0)

#define PI
PI = 3.14159265358979

GRID_SIZE = 151
INIT_POS = (GRID_SIZE-1)/2
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
screen = pygame.display.set_mode((GRID_SIZE*squareSize, GRID_SIZE*squareSize))

# Given the x, y coordinates find the most close square from the grid and paint it red
def drawRectangle(x, y):
	x = x - x%squareSize
	y = y - y%squareSize

	# Draw a dark red square at new position
	rect = pygame.Rect(x, y, squareSize, squareSize)
	pygame.draw.rect(screen, DARK_RED, rect)

	# Draw a red square at new position
	rect = pygame.Rect(x+1, y+1, squareSize-2, squareSize-2)
	pygame.draw.rect(screen, RED, rect)


def plotData():
	# Data is a 2d table, we want to average the values of each column and then plot them
	averagedRanges = [0] * 760
	counterTable = [0] * 760
	screen.fill(WHITE)

	# Create a py-game that generates a grid of 2501 x 2501 squares

	# for i in range(0,GRID_SIZE):
	# 	for j in range(0,GRID_SIZE):
	# 		# Draw a rectangle with white color and gray border
	# 		rect = pygame.Rect(i*squareSize, j*squareSize, squareSize, squareSize)
	# 		pygame.draw.rect(screen, GRAY, rect)
	# 		inner_rect = pygame.Rect(i*squareSize+1, j*squareSize+1, squareSize-2, squareSize-2)
	# 		pygame.draw.rect(screen, WHITE, inner_rect)

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

			drawRectangle(x*100 + GRID_SIZE*squareSize / 2, y*100 + GRID_SIZE*squareSize / 2)
			
	# 		pygame.draw.rect(screen, RED, (x * 100 + SCREEN_SIZE/2, y * 100  + SCREEN_SIZE/2, squareSize, squareSize))

	# Draw the initial point of the lidar
	pygame.draw.rect(screen, GREEN, (INIT_POS*squareSize, INIT_POS*squareSize, squareSize, squareSize))
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