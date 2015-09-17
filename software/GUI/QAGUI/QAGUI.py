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
import numpy as np
import cv2
import socket
import threading

class Network:
   #This class starts the network and launches a thread to receive asynchronous messages
   def __init__(self, parent, serverTcpIP, serverPort):
      self.parent = parent
      self.serverTcpIP = serverTcpIP
      self.serverPort = serverPort
      self.initNetwork()
      self.recvmsg = ''
      self.thread_stop= threading.Event()
      self.recvThread = threading.Thread(target=self.receiveMessage)
      self.recvThread.start()
      print 'Network started.'

   def initNetwork(self):
      print "Connected to %s:%s." % (self.serverTcpIP,self.serverPort)
      self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.sock.connect((self.serverTcpIP, self.serverPort))
      self.sock.settimeout(1)

   def receiveMessage(self):
      #generates events to update the GUI
      BUFFER_SIZE = 1024
      while 1:
         try:
            self.recvmsg = self.sock.recv(BUFFER_SIZE)
            self.parent.ltext.event_generate("<<NewMessage>>");
            print 'received: ', self.recvmsg
         except socket.timeout:
            continue
         except:
            break
      print 'Finished receive thread'

   def getNewMessage(self):
      return self.recvmsg
      
   def sendMessage(self, message):
      self.sock.send(message)
      
   def closeConnection(self):
      self.thread_stop.set()
      self.sock.close()
      print 'Connection closed.'

class GUI(tk.Frame):

   def __init__(self, parent, serverTcpIP, serverPort):
      tk.Frame.__init__(self, parent)
      self.parent = parent
      self.allCB = {}
      self.net = Network(self, serverTcpIP, serverPort)
      self.question = StringVar()
      self.question.set('Welcome to Rives del''Orne ')
      self.initUI()

   def resize(self, w, h, w_box, h_box, pil_image):
      '''
      resize a pil_image object so it will fit into
      a box of size w_box times h_box, but retain aspect ratio
      '''
      f1 = 1.0*w_box/w  # 1.0 forces float division in Python2
      f2 = 1.0*h_box/h
      factor = min([f1, f2])
      #print(f1, f2, factor)  # test
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

      topframe = Frame(self)
      topframe.pack()
      middleframe = Frame(self)
      middleframe.pack()
      bottomframe = Frame(self)
      bottomframe.pack(fill = tk.X)

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
      phY = ImageTk.PhotoImage(imY)
      imN = PIL.Image.open('img/no_150.jpg')
      phN = ImageTk.PhotoImage(imN)
      
      self.BtnY = Button(bottomframe, image=phY, command=self.ActionY)
      self.BtnY.image = phY
      self.BtnY.pack(side=LEFT)
      
      self.BtnN = Button(bottomframe, image=phN,command=self.ActionN)
      self.BtnN.image = phN
      self.BtnN.pack(side=RIGHT)

   def updateLabel(self, event):
      print 'Event triggered'
      self.question.set(self.net.getNewMessage())
      
   def ActionY(self):
      message = 'Yes\n\r'
      print(message)
      self.net.sendMessage(message)

   def ActionN(self):
      message = 'No\n\r'
      print(message)
      self.net.sendMessage(message)

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
