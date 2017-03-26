
"""Anna Buchele and Lydia Zuehsow"""

"""This program allows you to control your mouse by moving around a green object in front of the webcam.
	We think that with some more implementation, we would be able to do some really fururistic-looking stuff."""


from collections import deque
import cv2
import imutils
import os, sys
import argparse
import pygame
from pygame.locals import *
import time
import numpy as np
import random

class WebCam(object):
	def __init__(self, bufsize = 100, counter = 0):
		"""Run webcam, find green, return center coordinates?"""
		self.camera = cv2.VideoCapture(0)
		#construct argument parse, parse arguments
		self.ap = argparse.ArgumentParser()
		self.ap.add_argument("-v","--video",
			help="path to the(optional) video file")
		self.bufsize = bufsize
		self.ap.add_argument("-b", "--buffer", type=int, default = 100,
			help="max buffer size")
		self.pts = deque(maxlen=bufsize)
		self.rad = []
		self.counter = counter
		self.calpts = deque(maxlen=bufsize)
		self.calrad = []
		self.calcounter = counter


	def getcenter(self, greenLower, greenUpper):
		self.args = vars(self.ap.parse_args())

		#initialize tracked points, frame counter, coordinate deltas

		#grab current frame
		(self.grabbed, self.frame) = self.camera.read()
		
		#resize frame, blur frame, conert to HSV color space
		self.frame = imutils.resize(self.frame, width=600)
		blurred = cv2.GaussianBlur(self.frame,(11,11),0)
		hsv = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)

		#construct mask for "green", perform dialations and erosions
		#to remove erronous parts of mask
		mask = cv2.inRange(hsv, greenLower, greenUpper)
		mask = cv2.erode(mask,None,iterations=1)
		mask = cv2.dilate(mask,None,iterations=1)

		#find contours in the mask, initialize current (x,y) center
		self.cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,
			cv2.CHAIN_APPROX_SIMPLE)[-2]

		#only continue if at least one contour is found
		if len(self.cnts) > 0:

			#find largest contour in mask, use it to compute 
			#minimum enclosing circel and centroid for that contour

			c = max(self.cnts,key=cv2.contourArea)
			M = cv2.moments(c)
			(center,radius) = cv2.minEnclosingCircle(c)
			Mlist= [M["m10"], M["m00"],M["m01"],M["m00"]]
			if any(Mlist) == 0:
				return None
			else:
				center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
				return [center,radius]

class PygameView(object):
	"""Visualizes a fake desktop in a pygame window"""
	def __init__(self,model, screen):
		"""Initialise the view with a specific model"""
		self.model = model
		self.screen = screen
		redColor = pygame.Color(255,0,0)
		greenColor = pygame.Color(0,255,0)
		blueColor = pygame.Color(0,0,255)
		whiteColor = pygame.Color(255,255,255)


	def update(self):
		"""Draw the game state to the screen"""
		pygame.display.update()

class Mouse(object):
	"""Represents the mouse cursor"""
	def __init__(self, mousex=100, mousey=100):
		self.x = mousex
		self.y = mousey
	def initialsetup(self):
		pygame.mouse.set_pos(self.x,self.y)
	def MoveH(self,dY):
		if 0 < (self.x - dX/300) < screenwidth:
			self.x = self.x - (dX/100)
		elif (self.x - dX/300) >= screenwidth:
			self.x = screenwidth - 10
		elif (self.x - dX/300) <= 0:
			self.x = 10
		pygame.mouse.set_pos(self.x,self.y)
		ball.select()
	def MoveV(self,dY):
		if 0 < (self.y + dY/300) < screenheight:
			self.y = self.y + (dX/100)
		elif (self.y + dY/300)>= screenheight:
			self.y = screenheight - 10
		elif (self.y + dY/300) <= 0:
			self.y = 10
		pygame.mouse.set_pos(self.x,self.y)
		ball.select()



class Calibration(object):
	"""Performs calibration of the 'green thing' and represents the calibrated original "green object" """
	def __init__(self):
		self=self
	def startup(self,greenLower,greenUpper):

		calibrating = True
		count = 0
		calradi = 0
		caldx = 0
		caldy = 0
		calx = 0
		caly = 0
		caldXs=[]
		caldYs=[]
		calxs=[]
		calys=[]
		while calibrating:
			califind = webcam.getcenter(greenLower, greenUpper)
			A = "Please hold your object very still"
			B=	"in the center of the screen."
			C=  "The system is calibrating."
			D= "This will only take a moment."
			cv2.putText(webcam.frame,A,(10,30),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,255),3)
			cv2.putText(webcam.frame,B,(10,100),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,255),3)
			cv2.putText(webcam.frame,C,(10,170),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,255),3)
			cv2.putText(webcam.frame,D,(10,240),cv2.FONT_HERSHEY_SIMPLEX,0.9,(0,0,255),3)
			if califind == None:

				pass
			else:
				calicenter = califind[0]
				caliradius = califind[1]

				if caliradius > 20:
				#if radius is above a certain size we count it
					webcam.calpts.append(calicenter)
					webcam.calrad.append(caliradius)
					webcam.calcounter = webcam.calcounter + 1
					calcounter = webcam.calcounter
			buf = 10
			for i in range (1,len(webcam.calpts)):
			# ignoring tracked points that are None
				if webcam.calpts[i-1] is None or webcam.calpts[i] is None:
					pass
			#making sure we have enough points
				if webcam.calcounter >= buf and webcam.calpts[i-buf] is not None:
					#compute difference between x and y coordinates of the point and the point
					#minimum buffer length before it
					count = count + 1
					calx= webcam.calpts[i][0]
					caly= webcam.calpts[i][1]
					caldX = webcam.calpts[i-buf][0] - webcam.calpts[i][0]
					caldY = webcam.calpts[i-buf][1] - webcam.calpts[i][1]
					caldXs.append(caldX)
					caldYs.append(caldY)
					calxs.append(calx)
					calys.append(caly)
			cv2.imshow("Frame",webcam.frame)
			key = cv2.waitKey(1) & 0xFF

			#Eliminates accidental infinity loops by setting a frame limit on runtime.

			if count > 30:
				calradi = np.mean(webcam.calrad)
				caldx = np.mean(caldXs)
				caldy = np.mean(caldYs)
				calx = np.mean(calxs)
				caly = np.mean(calys)
				return [calradi, (caldx,caldy),(calx,caly)]
				running = False

class Ball(object):
	def __init__(self,color,x=50,y=50,selected=False):
		self.x = x
		self.y = y
		self.color = color
		self.selected = selected
	def select(self):
		if self.selected == True:
			self.x = cursor.x
			self.y = cursor.y


class DesktopModel(object):
	"""Stores the fake desktop state"""
	def __init__(self):
		self.desktop = screen.fill(whiteColor)
		pygame.display.update()

class Controller(object):
	def __init__(self,model):
		self.model = model
	def process_events(self):
 		"""Process all of the events in the queue"""
 		for event in pygame.event.get():
			if event.type == QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == GREENMOVEH:
				# if the event is for horizontal movement,
				# we pop out the first value of the list of 
				# dXs, then run the cursor function for horizontal movement
				dX = dXs.pop(0)
				cursor.MoveH(dX)
				pygame.event.post(select_event)
			elif event.type == GREENMOVEV:
				# if the event is for vertical movement,
				# we pop out the first value of the list of 
				# dYs, then run the cursor function for vertical movement
				dY = dYs.pop(0)
				cursor.MoveV(dY)
				pygame.event.post(select_event)
			elif event.type == SELECT:
				ball.color=redColor
				ball.selected = True


		pygame.event.clear()


if __name__ == '__main__':

	"""Initializing"""

	#Initialize pygame
	pygame.init()

	# Define some colors
	redColor = pygame.Color(255,0,0)
	greenColor = pygame.Color(0,255,0)
	blueColor = pygame.Color(0,0,255)
	whiteColor = pygame.Color(255,255,255)

	#Set pygame fake desktop size
	screenwidth= 1024
	screenheight= 768

	size = (screenwidth, screenheight)
	screen = pygame.display.set_mode(size)

	model = DesktopModel()
	view = PygameView(model, screen)
	master = Controller(model)




	"""WEBCAM STUFF"""

	#initialize stuff

	running = True
	ball = Ball(blueColor)
	ball.selected = False
	frame = 0
	eventcount = 0
	webcam = WebCam()

	greenLower= (29,86,6)
	greenUpper= (64,255,255)

	redLower= (29,86,6)
	redUpper= (255,255,255)

	calibrate = Calibration()
	[calradi,(caldx,caldy),(calx,caly)] = calibrate.startup(greenLower,greenUpper)

	cursor = Mouse(calx,caly)
	cursor.initialsetup()


	center = 0
	counter = 0
	calcounter = 0
	(dX,dY) = (0,0)
	dXs=[]
	dYs=[]
	(caldX,caldY)=(0,0)
	caldXs=[]
	caldYs=[]

	# Create new event for vertical and horizontal green movements
	GREENMOVEH = pygame.USEREVENT+1
	moveH_event= pygame.event.Event(GREENMOVEH)
	GREENMOVEV = pygame.USEREVENT+2
	moveV_event= pygame.event.Event(GREENMOVEV)
	SELECT = pygame.USEREVENT+3
	select_event= pygame.event.Event(SELECT)

	# makes sure only the events we want are on the event queue
	allowed_events = [GREENMOVEV,GREENMOVEH,QUIT,SELECT]
	pygame.event.set_allowed(allowed_events)

	buf = 10
	# "buf" is the buffer- the number of frames we go backwards 
	# to compare for movement- so if buf is 10, we compare 
	# the location of the "green" in the current frame 
	# to its location 10 frames earlier. 
	

	"""RUNTIME LOOP"""

	#This is the main loop of the program. 
	ballcolor1 = random.randint(0,255)
	ballcolor2 = random.randint(0,255)
	ballcolor3 = random.randint(0,255)
	frame = 0
	while running:
		ballcolor1 = ballcolor1 + random.randint(-10,10)
		ballcolor2= ballcolor2 + random.randint(-10,10)
		ballcolor3 = ballcolor3 + random.randint(-10,10)
		ballcolor = (int(ballcolor1),int(ballcolor2),int(ballcolor3))
		if 0> any(ballcolor) > 255:
			ballcolor= (100,100,100)

		pygame.draw.circle(screen,(ballcolor1,ballcolor2,ballcolor3),(int(ball.x),int(ball.y)),20,0)
		#Find the center of any green objects' contours

		gotcenter = webcam.getcenter(greenLower, greenUpper)
		if gotcenter == None:
			pass
		else:
			center = gotcenter[0]
			radius = gotcenter[1]
			cv2.circle(webcam.frame,center,5,(0,0,255), -1)

			if radius > 20:
				#if radius is above a certain size we count it
				webcam.pts.append(center)
				webcam.rad.append(radius)
				webcam.counter = webcam.counter + 1
				counter = webcam.counter

		for i in range (1,len(webcam.pts)):
			# ignoring tracked points that are None
			if webcam.pts[i-1] is None or webcam.pts[i] is None:
				pass
			#making sure we have enough points
			if webcam.counter >= buf and webcam.pts[i-buf] is not None:
				#compute difference between x and y coordinates of the point and the point
				#minimum buffer length before it
				dsX=0
				dsY=0
				dX = webcam.pts[i-buf][0] - webcam.pts[i][0]
				dY = webcam.pts[i-buf][1] - webcam.pts[i][1]
				if np.abs(dX) > 150:
					dXs.append(dX-caldx)
					pygame.event.post(moveH_event)
				if np.abs(dY) > 150:
					pygame.event.post(moveV_event)
					dYs.append(dY-caldy)
				raddif = webcam.rad[i] - calradi
				if np.abs(raddif)> (1/2)*calradi:
					pygame.event.post(select_event)
			#process the events in the queue
			master.process_events()

		# Update the frames of the webcam video
		cv2.imshow("Frame",webcam.frame)
		key = cv2.waitKey(1) & 0xFF

		frame = frame + 1
		# Update the fake pygame desktop
		view.update()


		time.sleep(.001)
		if key == ord("q"):
			break
		if key == ord("s"):
			cursor.x = calx
			cursor.y = caly
		if frame > 500:
			sys.quit
			break
if running == False:
		#release camera, close open windows
		webcam.camera.release()
		cv2.destroyAllWindows() 
