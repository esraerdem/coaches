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

from parse_rules_file import eval_personalization_rules

profile = '<*,*,*,*>'


class Network:
   #This class starts the network and launches a thread to receive asynchronous messages
   def __init__(self, parent, serverTcpIP, serverPort):
      self.parent = parent
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
               print 'not able to connect'
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
            # Example: received 'display_{text|image|video}_welcome'
            self.recvmsg = self.recvmsg.replace('\x00',"")
            splitmsg = self.recvmsg.strip('\n\r').split("_")
            print splitmsg
            if len(splitmsg) != 3:
               print 'There is something wrong with the message format'
               continue
            else:
               mode = splitmsg[1]
               interactionname = splitmsg[2]
               # eval_personalization_rules(welcome) -> actual_interaction
               actual_interaction= eval_personalization_rules(interactionname, profile)
               print actual_interaction
               # if (text) : show actual_interaction as a label in the GUI
               if (mode == 'text'):
                  self.text_to_display = actual_interaction

               # if (image) : show image in actual_interaction as an image in the GUI
                  
               # if (video) : ...
            
               self.parent.ltext.event_generate("<<NewMessage>>")

         else: #if there is no data something happened in the server
            self.netStatusOk = False
            break

      print 'Finished receive thread'

   def getNewMessage(self):
      return self.recvmsg

   def getTextToDisplay(self):
      return self.text_to_display

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
      print 'Connection closed.'
      
class profileSelectionGUI(object):

   def __init__(self, parent):
      self.toplevel = tk.Toplevel(parent)
      if (len(sys.argv) > 1):
         profiles_filename = sys.argv[1]
      else:
         profiles_filename = "instance"
      try:
         f = open(profiles_filename, 'r')
      except IOError:
         print 'cannot open', profiles_filename
      else:
         
         self.chosen_profile = ''
      
         def callback(text):
            self.chosen_profile = text
            self.toplevel.destroy()

         for line in f:
            line = line.strip("\n")
            btn = tk.Button(self.toplevel, text=line, command=lambda line=line: callback(line))
            btn.pack()
            
         f.close()

   def show(self):
      self.toplevel.deiconify()
      self.toplevel.wait_window()
      return self.chosen_profile

class GUI(tk.Frame):

   def __init__(self, parent, serverTcpIP, serverPort):
      tk.Frame.__init__(self, parent)
      self.parent = parent
      self.allCB = {}
      self.net = Network(self, serverTcpIP, serverPort)
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
      cap = cv2.VideoCapture('videos/rives_delorne.mp4')
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
      img = PIL.Image.open('img/caen-bienvenue.jpg')
      w, h = img.size
      im_resized = self.setHeight(w, h, height, img)
      imgtk = ImageTk.PhotoImage(image=im_resized)
      self.limg = Label(topframe, image=imgtk)
      self.limg.image = imgtk
      self.limg.pack(side=RIGHT) 

      # Label
      self.ltext = Label(middleframe, textvariable=self.question, font=("Helvetica", 32))
      self.ltext.bind("<<NewMessage>>", self.updateLabel)
      self.ltext.pack()

      # Buttons
      imY = PIL.Image.open('img/yes_150.jpg')
      self.phY = ImageTk.PhotoImage(imY)
      imN = PIL.Image.open('img/no_150.jpg')
      self.phN = ImageTk.PhotoImage(imN)
            
      self.BtnY = Button(self, image=self.phY, command=self.ActionY)
      self.BtnY.pack(side=LEFT)      
      self.BtnN = Button(self, image=self.phN,command=self.ActionN)
      self.BtnN.pack(side=RIGHT)

   def updateLabel(self, event):
      print 'Event triggered'
      self.question.set(self.net.getTextToDisplay())
      
   def ActionY(self):
      message = 'Yes\n\r'
      print(message)
      self.net.sendMessage(message)

   def ActionN(self):
      message = 'No\n\r'
      print(message)
      self.net.sendMessage(message)

   def profileSelection(self):
      global profile
      selection = profileSelectionGUI(self).show()
      if (len(selection) > 0):
         profile = selection
      self.profile_label.configure(text="Current profile: %s" % profile)

   def quit(self):
      self.net.closeConnection()
      pass

SERVER_TCP_IP = '127.0.0.1'
SERVER_TCP_PORT = 9000

def main():
   root = tk.Tk()
   f = GUI(root, SERVER_TCP_IP, SERVER_TCP_PORT)
#   root.geometry("1920x1080+0+0")
   root.geometry("1200x800+50+50")
   root.mainloop()
   f.quit()

if __name__ == '__main__':
   main()
