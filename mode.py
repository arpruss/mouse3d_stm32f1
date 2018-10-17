from sys import argv,exit
import ctypes
import ast

try:
    from pywinusb import hid
except ImportError:
    exit("You need pywinusb. Run python -m pip install pywinusb")
    
from time import sleep,time

TIMEOUT = 1
REPORT_ID = 5
REPORT_SIZE = 1

def sendMode(mode):
    data = [REPORT_ID] + [mode|0x40]
    myReport.set_raw_data(data)
    myReport.send()
    
myReport = None

while myReport is None:    
    for d in hid.HidDeviceFilter(product_id = 0xc62b).get_devices():
        if d.vendor_id == 0x1EAF or d.vendor_id == 0x046D:
            device = d
            device.open()

            for report in device.find_output_reports():
                if report.report_id == REPORT_ID and report.report_type == "Output":
                    myReport = report
                    break
                        
            if myReport is not None:
                break
            
            myReport = None
            device.close()
        
    sleep(0.25)

print("Device found")    
    
if len(argv)>1:
    m = ast.literal_eval(argv[1])
    sendMode(m)
    print("Setting mode %02x" % m)
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
        