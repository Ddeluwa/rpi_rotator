import math
import serial
import serial.rs485
import time
import sys,signal,os

panLeft = [0xFF, 0x02, 0x00, 0x04, 0x01, 0x00, 0x07]
panRight = [0xFF, 0x02, 0x00, 0x02, 0x40, 0x00, 0x44]
panStop = [0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02]

rotator = serial.Serial(
		port = '/dev/ttyUSB0',
		baudrate = 9600,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		bytesize = serial.EIGHTBITS,
		timeout = 1
	)

firstAngle = 0

while True :
    rotator.write(panStop)
    fileSlave = open('./GPS/SlaveGPS.txt','r')

    SlaveGPSData = fileSlave.readline()
    SGPSParsedData = SlaveGPSData.split(' ')
    SGPSlat = SGPSParsedData[0]
    SGPSlng = SGPSParsedData[1]

    

    if firstAngle == 0 :
        #initialize Antenna location -> 0 degree
        rotator.write(panRight)
        time.sleep(30)

        # load IMU Data
        fileIMU = open('./IMU/heading.txt','r')
        headingData = fileIMU.readline()
        floatHeadingData = float(headingData)
        angle = 360 - floatHeadingData
        angleSecond = 1.5 * angle / 10

        # rotate angle
        rotator.write(panLeft)
        time.sleep(int(angleSecond))
        print(headingData)
        rotator.write(panStop)
        firstAngle = angle
        fileIMU.close()

    else :
        # load IMU data
        fileIMU = open('./IMU/heading.txt','r')
        headingData = fileIMU.readline()
        floatHeadingData = float(headingData)
        angle = 360 - floatHeadingData

        # difference between current angle and rotator angle 
        tmp = angle - firstAngle

        print(tmp)
        if tmp < 0 :
            angleSecond = 1.5 * (-tmp) / 10
            rotator.write(panRight)
        elif tmp > 0 :
            angleSecond = 1.5 * tmp / 10
            rotator.write(panLeft)
        else :
            rotator.write(panStop)
        
        time.sleep(int(angleSecond))
        rotator.write(panStop)
        firstAngle = angle    
        fileIMU.close()
        time.sleep(0.5)
    