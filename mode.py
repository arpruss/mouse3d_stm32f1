from sys import argv,exit
import ctypes
import ast

try:
    from pywinusb import hid
except ImportError:
    exit("You need pywinusb. Run python -m pip install pywinusb")
    
from time import sleep,time

TIMEOUT = 1
REPORT_ID = 1
REPORT_SIZE = 1

def sendMode(mode):
    data = [REPORT_ID] + [mode]
    myReport.set_raw_data(data)
    myReport.send()
    
myReport = None
currentMode = None

while myReport is None:    
    print("Searching for device...")
    for d in hid.HidDeviceFilter(product_id = 0xc629 +0*0xc62b).get_devices():
        if d.vendor_id == 0x1EAF or d.vendor_id == 0x046D:
            device = d
            device.open()

            for report in device.find_feature_reports():
                if report.report_id == REPORT_ID and report.report_type == "Feature":
                    myReport = report
                    break
                        
            if myReport is not None:
                currentMode = myReport.get()
                if len(currentMode) >= 2:
                    currentMode = currentMode[1]
                    break
            
            myReport = None
            device.close()

    if myReport is None:
        sleep(1)
        
def describe(mode):
    out = []
    if currentMode & 1:
        out.append("joystick")
    else:
        out.append("3dmouse")
        
    if currentMode &2 :
        out.append("dominant")
    else:
        out.append("allaxes")

    if currentMode &4 :
        out.append("cubic")
    else:
        out.append("standard")
        
    return ' '.join(out)

print("Device found in mode %02x (%s)" % (currentMode,describe(currentMode)))

if len(argv)<=1:
    print("python mode.py [joystick|3dmouse] [dominant|allaxes] [cubic|standard]")
    print("You can abbreviate modes to their first letters.")
    exit(0)

for a in argv[1:]:
    if a[0] == 'j':
        currentMode |= 1
    elif a[0] == '3':
        currentMode &= ~1
    elif a[0] == 'c':
        currentMode |= 4
    elif a[0] == 's':
        currentMode &= ~4
    elif a[0] == 'd':
        currentMode |= 2
    elif a[0] == 'a':
        currentMode &= ~2
    
print("Setting mode %02x (%s)" % (currentMode,describe(currentMode)))
sendMode(currentMode)
device.close()

if False:
    try:
        from tkinter import * 
    except ImportError:
        from Tkinter import *
    
    root = Tk()
    root.title("Mode")

    option = Listbox(root,selectmode=SINGLE)
    option.config(width=0)
    option.config(height=0)
    option.pack()
    
    current = query("M")

    selection = 0
    i = 0
    while True:
        n = query("M"+str(i))
        if n is None or n == "":
            break
        option.insert(END, n)
        if n == current:
            option.select_set(i)
            option.activate(i)
            selection = i
        i+=1

    def up(_):
        global selection
        if selection > 0:
            option.select_clear(selection)
            selection -= 1
            option.select_set(selection)
            option.activate(selection)
            
    def down(_):
        global selection
        if selection < option.size():
            option.select_clear(selection)
            selection += 1
            option.select_set(selection)
            option.activate(selection)
            
    def ok(*args):
        opt = option.get(ACTIVE)
        sendCommand("M:"+opt)
        if query("M") != opt:
            print("Error setting mode")
        else:
            print("Set to: "+opt)
        device.close()
        root.quit()
        
    def cancel(*args):
        device.close()
        root.quit()

    root.bind("<Down>", down)
    root.bind("<Up>", up)
    root.bind("<Return>", ok)
    root.bind("<Escape>", cancel)
        
    bottom = Frame(root)
    bottom.pack()

    button = Button(root, text="OK", command=ok)
    button.pack(in_=bottom,side=LEFT)
    button2 = Button(root, text="Cancel", command=cancel)
    button2.pack()
    button2.pack(in_=bottom,side=RIGHT)

    mainloop()
        