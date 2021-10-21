import math
import serial
import serial.rs485
import time
import sys
import signal
import os
from haversine import haversine
import tailer
import copy
import datetime

panLeft = [0xFF, 0x02, 0x00, 0x04, 0x01, 0x00, 0x07]
panRight = [0xFF, 0x02, 0x00, 0x02, 0x40, 0x00, 0x44]
panStop = [0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02]

def calculateAzimuth(SGPSlat, SGPSlng, MGPSlat, MGPSlng) :
	SGPSlat_rad = float(SGPSlat) * math.pi / 180
	MGPSlat_rad = float(MGPSlat) * math.pi / 180
	lng_diff_rad = (float(SGPSlng) - float(MGPSlng)) * math.pi / 180

	y = math.sin(lng_diff_rad) * math.cos(MGPSlat_rad)
	x = math.cos(SGPSlat_rad) * math.sin(MGPSlat_rad) - math.sin(SGPSlat_rad) * math.cos(MGPSlat_rad) * math.cos(lng_diff_rad)
	arctanXY = math.atan2(y,x)
	
	angle = (int(arctanXY * 180 / math.pi)+360) % 360
	return angle

# Rotator Serial Open
rotator = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# GPS + IMU 테스트 프로그램
angle = 0
heading = 0
IMU_heading = 0
on = 0
currentAntenna = 0
print(" 211021 Mode ")
while True :
    logfile = open("./console.log", 'a')
    # 고정된 Master GPS 정보
    mGPS = [37.48471, 127.11914]

    # Slave GPS 정보
    sGPSData =tailer.tail(open('./GPS/SlaveGPS.txt'), 1)    
    sGPS_parsedData = sGPSData[0].split(' ')
    sGPS_lat = float(sGPS_parsedData[0])
    sGPS_lng = float(sGPS_parsedData[1])
    sGPS = [sGPS_lat, sGPS_lng]

    # 위도 경도를 이용한 거리 계산, Kilometer 단위
    distance = haversine(mGPS, sGPS)*1000

    print("# =========================================================================== #")
    print(" Linked Master GPS [ " + str(mGPS[0]) + ", " + str(mGPS[1]) + " ]")
    print(" Linked Slave GPS [ " + str(sGPS[0]) + ", " + str(sGPS[1]) + " ]")

    logfile.write(" [ " +str(datetime.datetime.now()) + " ]\n")
    logfile.write("# =========================================================================== #\n")
    logfile.write(" Linked Master GPS [ " + str(mGPS[0]) + ", " + str(mGPS[1]) + " ]\n")
    logfile.write(" Linked Slave GPS [ " + str(sGPS[0]) + ", " + str(sGPS[1]) + " ]")

    # 위도 경도를 이용한 방위각 계산
    gpsCurrentAzimuth = calculateAzimuth(sGPS[0], sGPS[1], mGPS[0], mGPS[1])
        
    # 차량 방향 값
    IMU = tailer.tail(open('./IMU/heading.txt'), 1)
    IMU_headingData = float(IMU[0])

    if on == 0 :
        heading = 360 - IMU_headingData
        print(" GPS Angle : " + str(gpsCurrentAzimuth))
        print(" Heading : " + str(heading))
        logfile.write(" GPS Angle : " + str(gpsCurrentAzimuth) + "\n")    
        logfile.write(" Heading : " + str(heading) + "\n")
        if gpsCurrentAzimuth > heading :
            angle = gpsCurrentAzimuth - heading
            angular_velocity = 1.7 * angle / 10
            rotator.write(panLeft)
            time.sleep(angular_velocity)
            rotator.write(panStop)

        elif gpsCurrentAzimuth < heading :
            angle = heading - gpsCurrentAzimuth
            angular_velocity = 1.7 * angle / 10
            rotator.write(panRight)
            time.sleep(angular_velocity)
            rotator.write(panStop)

        currentAntenna = angle

        on = 1
        # 차량기준 안테나 위치
        print(" Current Antenna : " + str(currentAntenna))
        logfile.write(" Current Antenna : " + str(currentAntenna) + "\n")
    else :
        heading = 360 - IMU_headingData
        print(" GPS one Angle : " + str(gpsCurrentAzimuth))
        print(" Heading : " + str(heading))
        
        logfile.write(" GPS Angle : " + str(gpsCurrentAzimuth) + "\n")    
        logfile.write(" Heading : " + str(heading) + "\n")

        if gpsCurrentAzimuth > heading :
            angle = gpsCurrentAzimuth - heading
            if currentAntenna > angle :
                rotateAngle = currentAntenna - angle
                angular_velocity = 1.7 * rotateAngle / 10
                rotator.write(panLeft)
                time.sleep(angular_velocity)
                rotator.write(panStop)

                tmp = currentAntenna
                currentAntenna = tmp - rotateAngle
            elif currentAntenna < angle :
                rotateAngle = angle - currentAntenna
                angular_velocity = 1.7 * rotateAngle / 10
                rotator.write(panRight)
                time.sleep(angular_velocity)
                rotator.write(panStop)

                tmp = currentAntenna
                currentAntenna = tmp + rotateAngle
                        
        elif gpsCurrentAzimuth < heading :
            angle = heading - gpsCurrentAzimuth
            if currentAntenna > angle :
                rotateAngle = currentAntenna - angle
                angular_velocity = 1.7 * rotateAngle / 10
                rotator.write(panRight)
                time.sleep(angular_velocity)
                rotator.write(panStop)

                tmp = currentAntenna
                currentAntenna = tmp - rotateAngle

            elif currentAntenna < angle :
                rotateAngle = angle - currentAntenna
                anguler_velocity = 1.7 * rotateAngle / 10
                rotator.write(panLeft)
                time.sleep(angular_velocity)
                rotator.write(panStop)
                        
                tmp = currentAntenna
                currentAntenna = tmp + rotateAngle
        print(" Current Antenna : " + str(currentAntenna))
        logfile.write(" Current Antenna : " + str(currentAntenna) + "\n")
        logfile.close()
        time.sleep(0.5)