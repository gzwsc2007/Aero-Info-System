import serial
from Tkinter import *

def mousePressed(event):
    redrawAll()

def keyPressed(event):
    redrawAll()

def timerFired():
    redrawAll()
    delay = 50 # milliseconds
    canvas.after(delay, timerFired) # pause, then call timerFired again

def redrawAll():
    #canvas.delete(ALL)
    s = canvas.data.COM.readline()
    s = s.split(';')
    try:
        magX = eval(s[0])
        magY = eval(s[1])
        magZ = eval(s[2])   
    except:
        return

    canvas.create_oval(magX-1+640, magY-1+512, magX+1+640, magY+1+512, fill="red")

def init():
    pass

def run():
    # create the root and the canvas
    global canvas
    root = Tk()
    canvas = Canvas(root, width=1280, height=1024)
    canvas.pack()
    # Set up canvas data and call init
    class Struct: pass
    canvas.data = Struct()
    init()
    
    # initialize Serial com
    canvas.data.COM = serial.Serial(2,115200) # COM3
    canvas.data.COM.flushInput() # flush input buffer
    
    # set up events
    root.bind("<Button-1>", mousePressed)
    root.bind("<Key>", keyPressed)
    timerFired()
    # and launch the app
    root.mainloop()  # This call BLOCKS (so your program waits until you close the window!)

run()