import sys
import time
import cv2
import numpy as np
#from cv2.dnn import readNetFromTensorflow
#from cv2.dnn import blobFromImage
from sklearn.cluster import KMeans, DBSCAN
import serial
from calibrate import *
from draw_line import get_yellow, get_range
from get_contours import get_gray_countour, contour_dump
from select_contours import contour_leftest, remove_contour

def _control_center( center_point ):

	ang = np.arctan2( center_point[0], center_point[1] )
	print("ang = ", ang)
	turn = 1-abs(ang)
	if turn < 0.1: turn = 0.1
	if ang < 0: turn = -turn
	speed = int(20*abs(turn)+10)
	return speed, turn

def get_contour_center( contour ):
	moment_contour = cv2.moments(contour)
	if moment_contour['m00']:
		moment_contour_x = int(moment_contour['m10']/moment_contour['m00'])
		moment_contour_y = int(moment_contour['m01']/moment_contour['m00'])
		return (moment_contour_x, moment_contour_y)

def get_contour_from_image( img ):
	line_yellows = get_yellow( img )
	line_whites  = get_range( img, np.array([0,160,0], np.uint8), np.array([255,255,255], np.uint8), cv2.COLOR_BGR2HLS)
	line_reds    = get_range( img, np.array([150,0,0], np.uint8), np.array([255,255,255], np.uint8), cv2.COLOR_BGR2HLS)
	img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY) # convert the color image to gray image
	contours = get_gray_countour(img_gray)
	contour_dump( "img/contours_full" + str(int(time.time()*10)) + ".jpg", contours, img)
	_, contours = remove_contour( line_yellows, contours, 1 )
	if len(contours) == 0: return None
	_, contours = remove_contour( line_whites, contours, -1 )
	if len(contours) == 0: return None
	contour_left = contour_leftest(contours)
	contour_dump( "img/contours_extracted" + str(int(time.time()*10)) + ".jpg", contour_left, img)
	return contour_left

def control( contour, s ):
	contour_center = get_contour_center(contour)
	if contour_center is None: return
	print("contour center = ", contour_center)
	control_vec = np.subtract( (np.shape(img)[1]/2, np.shape(img)[0]), contour_center )
	speed, turn = _control_center( control_vec )
	cmd = "/ServoTurn/run " + str(speed) + " " + "{0:.2f}".format(turn) + " \n"
	print("cmd = ", cmd)
	s.write(cmd.encode())
	print("speed, turn = ", speed, turn)

if __name__ == "__main__":
	s = serial.Serial("/dev/ttyACM0")
	vc = cv2.VideoCapture(1)
	for i in range(0,20): vc.read()
	f = open("timer.txt", "w")
	while 1:
		start_time = time.time()
		_, img = vc.read()
		img = calibrate(img)

		contour = get_contour_from_image(img)
		if not contour is None: control(contour,s)
		f.write( "Line 267: " + str(time.time()-start_time)+"\n")
#		time.sleep(.3)
