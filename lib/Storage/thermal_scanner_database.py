# THERMAL SCANNER DATABASE PROGRAM
# Recieves input data from microcontroller and creates a thermal logs directory
# with the specified user name as a log file and a time stamp and temperature reading appended
# Written by: Sean Bullis
# 1/28/2022

#References:
# Timestamp : https://www.geeksforgeeks.org/get-current-timestamp-using-python/
# Serial Port: https://pyserial.readthedocs.io/en/latest/shortintro.html

import serial 
import sys
import time
import os
import datetime

#Define Thermal Logs Directory Path and DateTime variable
directory_path = ".\Thermal Logs"


#Make Thermal Log Directory if it doesn't already exist
if(os.path.isdir('.\Thermal Logs') == False):
    os.mkdir('.\Thermal Logs')
    print("Thermal Logs Directory Created.\n")
else:
    print("Thermal Logs Directory Already Exists.\n")

#Define the Serial Port Parameters
serialPort = serial.Serial(
        port="COM3",
        baudrate=115200,
        stopbits=serial.STOPBITS_ONE,
    )

#Open Serial Port if not already open
if(serialPort.isOpen() == False):
    serialPort.open()


#Read from serial port and format data / write to data file
while(1):
    user_data = serialPort.readline()
    user_data = str(user_data, 'utf-8')
    #print(user_data)
    data_split = user_data.splitlines()
    #print("Data Split: ", data_split)
    name = data_split[0]
    #print(name)
    temperature = data_split[2]
    print(temperature)
    print('\n')
    userlog = name + str(".txt")
    print(userlog)
    file_path = os.path.join(directory_path, userlog)
    print(file_path)
    f = open(file_path, 'a')
    ct = datetime.datetime.now()
    f.write(str(ct)+ str("       "))
    f.write(temperature)
    f.write('\n')
    f.close()



