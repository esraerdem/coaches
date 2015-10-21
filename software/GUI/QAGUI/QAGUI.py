#!/usr/bin/env python
#

try:
   # for Python3
   import tkinter as tk
   from tkinter import *
except ImportError:
   # for Python2
   import Tkinter as tk
   from Tkinter import *
import PIL
from PIL import Image, ImageTk

import cv2
import socket
import threading
import errno, time

import os
script_dir = os.path.dirname(__file__)

from parse_rules_file import eval_personalization_rules

profile = '<*,*,*,*>'

class Network:
   #This class starts the network and launches a thread to receive asynchronous messages
   def __init__(self, serverTcpIP, serverPort):
      self.serverTcpIP = serverTcpIP
      self.serverPort = serverPort
      self.recvmsg = ''
      self.text_to_display = ''
      self.netStatusOk = False
      self.thread_stop= threading.Event()
      self.initNetwork()
      self.netStatusThread = threading.Thread(target=self.verifyNetwork)
      self.netStatusThread.start()
      print 'Network started.'

   def setParent(self, parent):
      self.parent = parent # mantains link to the GUI so we can generate events on it

   def verifyNetwork(self):
      while (not self.thread_stop.is_set()):
         if (not self.netStatusOk):
            print 'Trying to reconnect'
            self.initNetwork()
         else:
            secsToSleep = 1
            time.sleep(secsToSleep)
      print 'Finished verifyNetwork thread'

   def initNetwork(self):
      while (not self.thread_stop.is_set()):
         try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1)
            self.sock.connect((self.serverTcpIP, self.serverPort))
            print "Connected to %s:%s." % (self.serverTcpIP,self.serverPort)
            self.recvThread = threading.Thread(target=self.receiveMessage)
            self.recvThread.start()
            self.netStatusOk = True
            break
         except socket.error as e:
            print '[error]', e
            if (e.errno == errno.ECONNREFUSED):
               print 'not able to connect', self.serverTcpIP, ':', self.serverPort
               secsToSleep = 5
               print "trying to reconnect in %d seconds" % (secsToSleep)
               time.sleep(secsToSleep)

   def receiveMessage(self):
      #generates events to update the GUI based on the received messages
      BUFFER_SIZE = 1024
      while (not self.thread_stop.is_set()):
         try:
            self.recvmsg = self.sock.recv(BUFFER_SIZE)
         except socket.timeout:
            continue

         if (self.recvmsg):
            print 'received: ', self.recvmsg
            # net_ROS object will receive:
            #  'display_{text|image|video}_welcome'
            #  'say_welcome'
            # net_speech object will receive:
            #  'ASR_text'
            self.recvmsg = self.recvmsg.replace('\x00',"")
            splitmsg = self.recvmsg.strip('\n\r').split("_") # Example: returns ['display', 'text', 'welcome']

            print splitmsg
            if (len(splitmsg) > 3 or len(splitmsg) < 2):
               print 'There is something wrong with the message format. Example: display_[mode]_[interactionname]'
               continue
            else:
               if (splitmsg[0] == 'display' and len(splitmsg) == 3):
                  mode = splitmsg[1]
                  interactionname = splitmsg[2]
                  rules_filename = "_".join([splitmsg[1], splitmsg[2]])
                  #to correctly load the file if the GUI is not executed from the current dir
                  rules_filename = os.path.join(script_dir, rules_filename)

                  # eval_personalization_rules(welcome) -> actual_interaction
                  actual_interaction= eval_personalization_rules(rules_filename, profile)
                  print "Display: ", actual_interaction

                  if (len(actual_interaction)>0):
                     # if (text) : show actual_interaction as a label in the GUI
                     if (mode == 'text'):
                        self.text_to_display = actual_interaction
                        self.parent.ltext.event_generate("<<NewTextMessage>>")

                     # if (image) : show image in actual_interaction as an image in the GUI
                     if (mode == 'image'):
                        self.image_to_display = actual_interaction
                        self.parent.limg.event_generate("<<NewImgMessage>>")
                     
                     # if (video) : ...TODO

               elif (splitmsg[0] == 'say' and  len(splitmsg) == 2):
                  # if (say_something) coming from tcp_interface: 
                  rules_filename = "_".join(["text", splitmsg[1]])
                  rules_filename = os.path.join(script_dir, rules_filename)

                  #  look for the string to say according to user profile
                  txt_say = eval_personalization_rules(rules_filename, profile)
                  print "Say: ", txt_say

                  if (len(txt_say)>0):
                     net_speech.sendMessage(txt_say)

               elif (splitmsg[0] == 'ASR'):
                  # if (ASR_something) coming from speech server
                  # net_ros.send("@ASR "+something)
                  print "ASR: " 
                  net_ROS.sendMessage("@ASR")
               else:
                  print 'Unrecognized instruction'
                  continue

         else: #if there is no data something happened in the server
            self.netStatusOk = False
            break

      print 'Finished receive thread'

   def getNewMessage(self):
      return self.recvmsg

   def getTextToDisplay(self):
      return self.text_to_display

   def getImgToDisplay(self):
      return self.image_to_display

   def sendMessage(self, message):
      if (self.netStatusOk):
         self.sock.send(message)
      
   def closeConnection(self):
      self.thread_stop.set()
      print 'Finishing threads...'
      time.sleep(5)
      
      if (self.recvThread.isAlive() or self.netStatusThread.isAlive()):
         print 'ups.. some thread still alive'
      
      self.sock.close()
      print "Connection to %s:%s closed." % (self.serverTcpIP,self.serverPort)
      
class profileSelectionGUI(object):

   def __init__(self, parent):
      self.toplevel = tk.Toplevel(parent)
      self.chosen_profile = ''
      if (len(sys.argv) > 1):
         profiles_filename = sys.argv[1]
      else:
         profiles_filename = "instance"
      try:
         profiles_filename = os.path.join(script_dir, profiles_filename)
         f = open(profiles_filename, 'r')
      except IOError:
         print 'cannot open', profiles_filename
      else:

         def callback(text):
            self.chosen_profile = text
            self.toplevel.destroy()

         #this is something temporal, we take just the 4 first profiles and show them in a grid
         maxProfiles = 4
         i=0
         sizegrid = 2
         for line in f:
            if (i == maxProfiles):
               break
            line = line.strip("\n")
            btn = tk.Button(self.toplevel, text=line, font=("Helvetica", 32), command=lambda line=line: callback(line)).grid(row=i/sizegrid, column=i%sizegrid, sticky='EWNS')
            i +=1

         #this shows all profiles as a list
         # for line in f:
         #    line = line.strip("\n")
         #    btn = tk.Button(self.toplevel, text=line, command=lambda line=line: callback(line))
         #    btn.pack()
            
         f.close()

   def show(self):
      self.toplevel.deiconify()
      self.toplevel.wait_window()
      return self.chosen_profile

class GUI(tk.Frame):

   def __init__(self, parent):
      tk.Frame.__init__(self, parent)
      self.parent = parent
      net_ROS.setParent(self)
      self.question = StringVar()
      self.question.set("Welcome to Rives de l'Orne ")
      self.initUI()

   def resize(self, w, h, w_box, h_box, pil_image):
      '''
      resize a pil_image object so it will fit into
      a box of size w_box times h_box, but retain aspect ratio
      '''
      f1 = 1.0*w_box/w  # 1.0 forces float division in Python2
      f2 = 1.0*h_box/h
      factor = min([f1, f2])
      # use best down-sizing filter
      width = int(w*factor)
      height = int(h*factor)
      return pil_image.resize((width, height), Image.ANTIALIAS)

   def setHeight(self, w, h, h_box, image):
      '''
      resize an image with height h maintaining aspect ratio
      '''
      factor = 1.0*h_box/h
      width = int(w*factor)
      height = int(h*factor)
      return image.resize((width, height), Image.ANTIALIAS)
      
   def initUI(self):
      self.parent.title("COACHES Q&A")
      self.parent.resizable(width=FALSE, height=FALSE)
      self.pack(expand=100)

      profileframe = Frame(self)
      profileframe.pack()
      topframe = Frame(self)
      topframe.pack()
      middleframe = Frame(self)
      middleframe.pack()
      bottomframe = Frame(self)
      bottomframe.pack(fill = tk.X)

      # Profile selection button
      self.profilebutton = Button(profileframe, text="Select profile", command=self.profileSelection)
      self.profilebutton.pack(side=LEFT)
      self.profile_label = Label(profileframe)
      self.profile_label.pack(side=LEFT, fill='x')

      # Video
      width, height = 500, 375
      rel_path = 'videos/rives_delorne.mp4'
      abs_file_path = os.path.join(script_dir, rel_path)
      cap = cv2.VideoCapture(abs_file_path)
      cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, width)
      cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, height)
      
      lvideo = tk.Label(topframe)
      lvideo.pack(side = LEFT)

      def show_frame():
         retval, frame = cap.read()
         if (retval): # true = still frames to read
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            h, w = cv2image.shape[:2]
            img = Image.fromarray(cv2image)
            im_resized = self.setHeight(w, h, height, img)
            imgtk = ImageTk.PhotoImage(image=im_resized)
            lvideo.imgtk = imgtk
            lvideo.configure(image=imgtk)
            lvideo.after(20, show_frame)

      show_frame()

      # Image
      rel_path = 'img/caen-bienvenue.jpg'
      abs_file_path = os.path.join(script_dir, rel_path)
      img = PIL.Image.open(abs_file_path)
      w, h = img.size
      im_resized = self.setHeight(w, h, height, img)
      imgtk = ImageTk.PhotoImage(image=im_resized)
      self.limg = Label(topframe, image=imgtk)
      self.limg.image = imgtk
      self.limg.bind("<<NewImgMessage>>", self.updateImg)
      self.limg.pack(side=RIGHT) 

      # Label
      self.ltext = Label(middleframe, textvariable=self.question, font=("Helvetica", 32), wraplength=1200)
      self.ltext.bind("<<NewTextMessage>>", self.updateLabel)
      self.ltext.pack()

      # Buttons
      rel_path = 'img/yes_150.jpg'
      abs_file_path = os.path.join(script_dir, rel_path)
      imY = PIL.Image.open(abs_file_path)
      self.phY = ImageTk.PhotoImage(imY)
      rel_path = 'img/no_150.jpg'
      abs_file_path = os.path.join(script_dir, rel_path)
      imN = PIL.Image.open(abs_file_path)
      self.phN = ImageTk.PhotoImage(imN)
            
      self.BtnY = Button(self, image=self.phY, command=self.ActionY)
      self.BtnY.pack(side=LEFT)      
      self.BtnN = Button(self, image=self.phN, command=self.ActionN)
      self.BtnN.pack(side=RIGHT)

   def updateLabel(self, event):
      print 'Event triggered'
      self.question.set(net_ROS.getTextToDisplay())

   def updateImg(self, event):
      print 'Event triggered'
      img_name = net_ROS.getImgToDisplay()
      abs_file_path = os.path.join(script_dir, img_name)
      img = PIL.Image.open(abs_file_path)
      w, h = img.size
      width, height = 500, 375
      im_resized = self.setHeight(w, h, height, img)
      imgtk = ImageTk.PhotoImage(image=im_resized)
      self.limg.configure(image = imgtk)
      self.limg.image = imgtk
      
   def ActionY(self):
      message = 'Yes\n\r'
      print(message)
      net_ROS.sendMessage(message)

   def ActionN(self):
      message = 'No\n\r'
      print(message)
      net_ROS.sendMessage(message)

   def profileSelection(self):
      global profile
      selection = profileSelectionGUI(self).show()
      if (len(selection) > 0):
         profile = selection
      self.profile_label.configure(text="Current profile: %s" % profile)

   def quit(self):
      net_ROS.closeConnection()
      #net_speech.closeConnection()
      pass

# Global variables:
# net_ROS and net_speech : objects of class Network
SPEECH_SERVER_TCP_IP = '127.0.0.1'
SPEECH_SERVER_TCP_PORT = 5000
ROS_SERVER_TCP_IP = '192.168.2.225' #'127.0.0.1'
ROS_SERVER_TCP_PORT = 9000

#net_speech = Network(SPEECH_SERVER_TCP_IP, SPEECH_SERVER_TCP_PORT)
net_ROS = Network(ROS_SERVER_TCP_IP, ROS_SERVER_TCP_PORT)

def main():
   root = tk.Tk()
   f = GUI(root)
#   root.geometry("1920x1080+0+0")
   root.geometry("1200x800+50+50")
   root.mainloop()
   f.quit()

if __name__ == '__main__':
   main()
