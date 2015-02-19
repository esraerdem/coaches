#!/usr/bin/env python

import Tkinter as tk
import tkMessageBox
import thread
from Tkinter import *
from ttk import *
import sys, time, os, glob, shutil, math, datetime
import rospy
from std_msgs.msg import String


Conditions = ['desire(unknown,swipe)']


class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.first_map_selected = True
        self.initUI()
        

    def initUI(self):

        self.parent.title("Condition Simulator")
        self.style = Style()
        self.style.theme_use("alt")
        self.parent.resizable(width=FALSE, height=FALSE)
        self.pack(fill=BOTH, expand=1)
        
        row = 0 
        col = 0
        
        for i in range(0,len(Conditions)):
	  self.add_condition(Conditions[i],row)
	  row = row+1
    
    def add_condition(self,cond,_row):
	print 'CondGUI: Adding condition: %s' % (cond)
	# Buttons
        b = Button(self, text=cond,command=lambda cond=cond: self.docondition(cond))
        b.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        

    def docondition(self,cond):
        global pub
	print "CondGUI: Send condition %s" %(cond)
	pub.publish(cond)
	


def main():
    global pub
    rospy.init_node('condition_simulator', anonymous=True)
    pub = rospy.Publisher('/diago/PNPConditionEvent', String, queue_size=1)
    root = tk.Tk()
    f = DIP(root)
    root.geometry("360x320+0+0")
    root.mainloop()
  


if __name__ == '__main__':
    main()

