import serial.tools.list_ports
import serial
import select
from sys import exit,stderr

stm = None
ball = None

for p in serial.tools.list_ports.comports():
    print(p.description)
    if p.description.startswith("USB Serial Device"):
        stm = p.device
        print(p.description)
    elif p.description.startswith("USB-SERIAL CH340"):
        ball = p.device
        print(p.description)

if stm is None or ball is None:
    print("Cannot find device")
    exit(1)

port1 = serial.Serial(port=stm,baudrate=115200)
port2 = serial.Serial(port=ball,baudrate=9600)

#while port2.read() != b'7': pass

#port2.write(b"M\rS\rS\r")

while True:
#    select.select([port1], [], [])
    w = port1.in_waiting
    while w > 0:
        c = port1.read()
        print("stm:"+str(c))
#        stdout.write(c)
        port2.write(c)
        w -= 1
    w = port2.in_waiting
    while w > 0:
        c = port2.read()
#        print("ball:"+str(c))
        stderr.buffer.write(c)
        if c == b'\n' or c == b'\r': stderr.buffer.write(b'\r\n')
        stderr.flush()
        port1.write(c)
        w -= 1        
