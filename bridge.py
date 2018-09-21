import serial.tools.list_ports
import serial
import threading
from sys import exit,stderr
from time import sleep

stm = None
ball = None

def persistentOpen(description,baudrate=115200):
    while True:
        try:
            for p in serial.tools.list_ports.comports():
                if p.description.startswith(description):
                    print("Opening "+str(p))
                    conn = serial.Serial(port=p.device,baudrate=baudrate)
                    conn.description = description
                    conn.device = p.device ## check
                    return conn
        except serial.SerialException as e:
            print("Error "+str(e))
            sleep(0.5)

port1 = persistentOpen("USB Serial Device", baudrate=115200)
print(port1)
port2 = persistentOpen("USB-SERIAL CH340", baudrate=9600)
print("Ports opened")

def persistentRead(port):
    while True:
        try:
            data = port.read()
            return (port,data)
        except serial.SerialException as e:
            print("Reconnecting after "+str(e))
            try:
                port.close()
            except:
                pass
            d = port.description
            b = port.baudrate
            sleep(0.5)
            port = persistentOpen(port.description)
        except Exception as e:
            print(str(e))

def read1():
    global port1
    while True:
        (port1,data) = persistentRead(port1)
        try:
            port2.write(data)
        except Exception as e:
            print(str(e))
            
threading.Thread(target=read1).start()
        
while True:
    (port2,data) = persistentRead(port2)
    try:
        port1.write(data)
    except Exception as e:
        print(str(e))
