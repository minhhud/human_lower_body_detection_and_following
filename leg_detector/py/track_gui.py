#!/usr/bin/env python

import roslib
import roslib.packages
import roslib.message
roslib.load_manifest('leg_detector')
from roslib import scriptutil
import rospy
import math
import cv2
import cv
import wx
import numpy as np
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.pyplot import *
from geometry_msgs.msg import Point
from cir_msgs.msg import Device_Ultra_Msg

class MyFrame(wx.Frame):
	def __init__(self, parent, id, title):   	
		wx.Frame.__init__(self, parent, id, title, size=(500,500), style=wx.DEFAULT_FRAME_STYLE ^ wx.RESIZE_BORDER)

		rospy.init_node('track_gui', anonymous=True)	
		rospy.Subscriber('/DeviceNode/UltraSonic/data', Device_Ultra_Msg, self.ultra_data)

		self.panel = wx.Panel(self, 0, (10, 10), (300, 300),  style=wx.SUNKEN_BORDER)
		mainSizer = wx.BoxSizer(wx.VERTICAL)
		self.figure = Figure(figsize=(0.2,0.2))
		self.axes = self.figure.add_axes([0.05,0.05,0.9,0.9])
		self.axes.axis([-3, 3, -3, 3]);
		self.canvas = FigureCanvas(self.panel, -1, self.figure)
		mainSizer.Add(self.canvas, 1, wx.EXPAND)	
		self.panel.SetSizer(mainSizer)
		self.Centre()
		self.Show()

	def ultra_data(self, data):
		print data.param
		if len(data.param):
			wx.CallAfter(self.display, data)

	def display(self, data):
		self.axes.cla()
		self.axes.axis([-3, 3, -3, 3]);

		robot_x = 0.2*np.cos(np.linspace(0,2*math.pi,20));
		robot_y = 0.2*np.sin(np.linspace(0,2*math.pi,20)); 
		self.axes.fill(robot_x, robot_y, 'y');

		obj_x = 0.05*np.cos(np.linspace(0,2*math.pi,20));
		obj_y = 0.05*np.sin(np.linspace(0,2*math.pi,20)); 

		sensor = [0, 0, 0, 0, 0, 0, 0, 0]
		for n in range(8):
			sensor[n] = data.param[n]
			if data.param[n] > 80:
				sensor[n] = 80
			self.axes.fill(obj_x+((float(sensor[n])/100) * np.cos((90-45*n)*math.pi/180)), obj_y+((float(sensor[n])/100) * np.sin((90-45*n)*math.pi/180)), 'k');

		self.canvas.draw()

if __name__ == '__main__':
	global frame
	app = wx.App()
	frame = MyFrame(None, -1, 'Track_GUI')
	app.MainLoop()
