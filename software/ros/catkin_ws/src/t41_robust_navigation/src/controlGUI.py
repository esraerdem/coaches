#!/usr/bin/env python

import Tkinter as tk
import tkMessageBox
import thread
from Tkinter import *
from ttk import *
import sys, time, os, glob, shutil, math, datetime
import rospy
from std_msgs.msg import String
from shared.msg import Event


Conditions = ['desire(unknown,swipe)','request(PBlue)','request(PRed)','request(PPink)', 'personPrinter', 'nohelp', 'helpbringdoc', 'helptechnician' ]


class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.first_map_selected = True
        self.initUI()
        

    def initUI(self):

        self.parent.title("Simulation Control GUI")
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
        global pubCond, pubEvent
        if ("request" in cond):
            e = Event()
            e.kind = 'request'
            if ('Blue' in cond):
              e.uid = '0'
            elif ('Red' in cond):
              e.uid = '1'
            elif ('Pink' in cond):
              e.uid = '2'
            print "CondGUI: Send event %s %s" %(e.kind, e.uid)
            pubEvent.publish(e)
        else:
            if (cond=='desire(unknown,swipe)'):
                scond = 'D_unknown_swipe'
                print "CondGUI: Sending condition %s ..." %(scond)
                for i in range(0,3):
                    pubCond.publish(cond)
                    rospy.sleep(2)
            else:
                print "CondGUI: Sending condition %s ..." %(cond)
                pubCond.publish(cond)              
	print "CondGUI: Done."
	


def main():
    global pubCond, pubEvent
    rospy.init_node('condition_simulator', anonymous=True)
    pubCond = rospy.Publisher('/diago/PNPConditionEvent', String, queue_size=1)
    pubEvent = rospy.Publisher('/diago/t22_event', Event, queue_size=1)
    root = tk.Tk()
    f = DIP(root)
    root.geometry("360x320+0+0")
    root.mainloop()


if __name__ == '__main__':
    main()

