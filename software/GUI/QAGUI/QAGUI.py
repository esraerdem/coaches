#!/usr/bin/env python
#
import tkinter as tk
import PIL
from PIL import Image, ImageTk
from tkinter import *


class GUI(tk.Frame):

   def __init__(self, parent):
      tk.Frame.__init__(self, parent)
      self.parent = parent
      self.allCB = {}
      self.initUI()
      
   def initUI(self):
      self.parent.title("COACHES Q&A")
      self.parent.resizable(width=FALSE, height=FALSE)
      self.pack(expand=100)

      # Label
      self.lbl = Label(self, text="Hello. Would you like to answer this question?", font=("Helvetica", 32))
      self.lbl.pack()
      
      imY = PIL.Image.open('img/yes_150.jpg')
      self.phY = ImageTk.PhotoImage(imY)
      imN = PIL.Image.open('img/no_150.jpg')
      self.phN = ImageTk.PhotoImage(imN)
            
      # Buttons
      self.BtnY = Button(self, image=self.phY, command=self.ActionY)
      #self.BtnY.image = phY
      self.BtnY.pack(side=LEFT)
      
      self.BtnN = Button(self, image=self.phN,command=self.ActionN)
      #self.BtnN.image = phN
      self.BtnN.pack(side=RIGHT)
 
   def ActionY(self):
      print('Yes')
      self.BtnN.image = self.phY
      self.BtnN.update()
      self.update()

   def ActionN(self):
      print('No')

   def quit(self):
      pass

def main():
   root = tk.Tk()
   f = GUI(root)
   root.geometry("1400x240+50+550")
   root.mainloop()
   f.quit()

if __name__ == '__main__':
   main()
