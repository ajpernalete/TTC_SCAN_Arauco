# -*- coding: utf-8 -*-
#!/usr/bin/env python
import serial
import time

def write_serial( dato : str):
    try:
        # Encender Rojo = S21
        # Apagar Rojo = 20
        # Encender Verde = 31
        # Apagar Verde = 30

        ser = serial.Serial(
                        port='COM2', 
                        baudrate = 9600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout=None
                )
        sms = dato.encode()
        print(sms)
        ser.write(sms)
        ser.write(chr(13).encode())
    except Exception as e:
        print("Error Escribir Serial Encender: "+str(e))
        
def read_serial():
    try:
        # Encender Rojo = S21
        # Apagar Rojo = 20
        # Encender Verde = 31
        # Apagar Verde = 30

        ser = serial.Serial(
                        port='COM2', 
                        baudrate = 9600,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        bytesize=serial.EIGHTBITS,
                        timeout=1
                )
        bin = ser.readline()
        print(bin.decode())
    except Exception as e:
        print("Error Escribir Serial Encender: "+str(e))

encender_verde = 'S21'
apagar_verde = 'S20'

var = 0
while var < 5:
    read_serial()
    time.sleep(0.5)
    var+=1

write_serial(encender_verde)

var2 = 0
while var2 < 5:
    read_serial()
    time.sleep(0.5)
    var2+=1

write_serial(apagar_verde)

var3 = 0
while var3 < 5:
    read_serial()
    time.sleep(0.5)
    var3+=1

# while True:
#     read_serial()
#     time.sleep(1)
