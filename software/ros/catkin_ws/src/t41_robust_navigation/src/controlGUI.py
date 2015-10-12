#!/usr/bin/env python

import Tkinter as tk
#import tkMessageBox
import thread
from Tkinter import *
from ttk import *
import sys, time, os, glob, shutil, math, datetime, thread
import rospy
from std_msgs.msg import String
from shared.msg import Event, AllGoals


Conditions = ['Send_Goal','desire(unknown,swipe)','request(PBlue)','request(PRed)','request(PPink)', 'nohelp', 'helpbringdoc', 'helptechnician' ]
CBConditions = ['personHere', 'personPrinter' ]
ConditionVar = [ None ] * len(CBConditions) 
do_run = True

def printf(format, *args):
    sys.stdout.write(format % args)

class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent
        for i in range(0,len(CBConditions)):
            ConditionVar[i] = IntVar()
            ConditionVar[i].set(0)
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
        for i in range(0,len(CBConditions)):
            self.add_CBcondition(CBConditions[i],ConditionVar[i],row)
            row = row+1

    
    def add_CBcondition(self,cond,ivar,_row):
        print 'CondGUI: Adding CB condition: %s' % (cond)
        b = Checkbutton(self, text=cond, variable=ivar, command=lambda c=cond,v=ivar: self.doCBcondition(c,v))
        b.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        
    def add_condition(self,cond,_row):
        print 'CondGUI: Adding condition: %s' % (cond)
        # Buttons
        b = Button(self, text=cond,command=lambda cond=cond: self.docondition(cond))
        b.grid(sticky=W, row=_row, column=0, pady=4, padx=5)


    def doCBcondition(self,cond,var):
        print "CondGUI: Set and send condition %s = %d" %(cond,var.get())
        condsend = ""        
        if (var.get()==0):
           condsend = "!"
        condsend = condsend + cond
        print " ... published: %s" %(condsend)
        pubCond.publish(condsend)
        rospar = "PNPconditionsBuffer/" + cond; 
        rospy.set_param(rospar,var.get())
        rospy.sleep(0.05)
        

    def docondition(self,cond):
        global pubCond, pubEvent, pubKBGoal
        if (cond=="Send_Goal"):
            g = AllGoals()
            #g.mode='' 
            #g.goals[0] = Goal()           
            #  g.loc=''
            #  g.kind=''
            #  g.param=''
            #  g.value=0.0
            #  g.duration=0.0
            pubKBGoal.publish(g);
        elif ("request" in cond):
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
            rospy.sleep(0.05)
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
                rospar = "PNPconditionsBuffer/" + cond; 
                rospy.set_param(rospar,1)
                rospy.sleep(0.05)

    print "CondGUI: Done."

    def quit(self):
        do_run = False

# Thread for sending CB conditions
def run():
    rospy.sleep(5)
    for i in range(0,len(CBConditions)):
        rospar = "PNPconditionsBuffer/" + CBConditions[i]; 
        rospy.set_param(rospar,0)
    rospy.sleep(3)
    while (do_run):
        for i in range(0,len(CBConditions)):
            # printf("Sending condition %s = %s\n",CBConditions[i],ConditionVar[i].get())
            cond = ""
            if (ConditionVar[i].get()==0):
                cond = "!"
            cond = cond + CBConditions[i]
            pubCond.publish(cond)
            rospy.sleep(0.05)
        rospy.sleep(1.0)
    print "Condition sending thread terminated"

def main():
    global pubCond, pubEvent, pubKBGoal
    rospy.init_node('condition_simulator', anonymous=True)
    pubCond = rospy.Publisher('/diago/PNPConditionEvent', String, queue_size=1)
    pubEvent = rospy.Publisher('/diago/t22_event', Event, queue_size=1)
    pubKBGoal = rospy.Publisher('/diago/t12_goals_set', AllGoals, queue_size=1)
    root = tk.Tk()
    f = DIP(root)
    thread.start_new_thread(run, ())
    root.geometry("300x400+0+0")
    root.mainloop()


if __name__ == '__main__':
    main()

