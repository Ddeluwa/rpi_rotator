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

panF_Left = [0xFF, 0x01, 0x00, 0x04, 0x01, 0x00, 0x06]
panF_Right = [0xFF, 0x01, 0x00, 0x02, 0x40, 0x00, 0x43]
panF_Stop = [0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01]

panS_Left = [0xFF, 0x02, 0x00, 0x04, 0x01, 0x00, 0x07]
panS_Right = [0xFF, 0x02, 0x00, 0x02, 0x40, 0x00, 0x44]
panS_Stop = [0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02]

def calculateAzimuth(SGPSlat, SGPSlng, MGPSlat, MGPSlng) :
	SGPSlat_rad = float(SGPSlat) * math.pi / 180
	MGPSlat_rad = float(MGPSlat) * math.pi / 180
	lng_diff_rad = (float(SGPSlng) - float(MGPSlng)) * math.pi / 180

	y = math.sin(lng_diff_rad) * math.cos(MGPSlat_rad)
	x = math.cos(SGPSlat_rad) * math.sin(MGPSlat_rad) - math.sin(SGPSlat_rad) * math.cos(MGPSlat_rad) * math.cos(lng_diff_rad)
	arctanXY = math.atan2(y,x)
	
	angle = (int(arctanXY * 180 / math.pi)+360) % 360
	return angle


# 현재 GPS 위치와 마스터의 GPS 위치를 계산하여 안테나 방향을 제어

# Rotator Serial Open
rotator = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

while True:
    
    # 안테나 위치 초기화
    #rotator.write(panF_Stop)
    #time.sleep(0.03)
    rotator.write(panS_Stop)
    time.sleep(0.03)
    print("Antenna rotation is initializing ...")
    #rotator.write(panF_Left)
    #time.sleep(20)
    #rotator.write(panF_Stop)
    #time.sleep(0.03)
    #rotator.write(panS_Left)
    #time.sleep(20)
    #rotator.write(panS_Stop)
    #time.sleep(0.03)
    # Menu
    print(" Mode Select ")
    print(" 1. Test_1     2. Test_2     3. Test_3")
    modeSelect = input(" Input : ")

    if modeSelect == "1":
        # GPS + IMU 테스트 프로그램
        angle = 0
        heading = 0
        IMU_heading = 0
        on = 0
        currentAntenna = 0
        print(" Test 1 Mode ")

        print(" Master GPS input ")
        #mGPS_lat = float(input(" Master lat : "))
        #mGPS_lng = float(input(" Master lng : "))
        
        while True :
            # 고정된 Master GPS 정보
            #mGPS = [37.48471, 127.11914]
            mGPS_one = [37.48471, 127.11914]
            mGPS_two = [37.48471, 127.11914]
            mGPS_three = [37.48471, 127.11914]
            mGPS = [mGPS_one, mGPS_two, mGPS_three]

            # Slave GPS 정보 - SSH로 받아와서 저장된 파일을 불러옴
                        #sGPSFile = open('./GPS/SlaveGPS.txt', 'r')
                        #sGPSData = sGPSFile.readline()
                        #sGPSFile.close()
            sGPSData =tailer.tail(open('./GPS/SlaveGPS.txt'), 1)
            
            sGPS_parsedData = sGPSData[0].split(' ')
            sGPS_lat = float(sGPS_parsedData[0])
            sGPS_lng = float(sGPS_parsedData[1])
            sGPS = [sGPS_lat, sGPS_lng]

            # 위도 경도를 이용한 거리 계산, Kilometer 단위
            distanceOne = haversine(mGPS_one, sGPS)*1000
            distanceTwo = haversine(mGPS_two, sGPS)*1000
            distanceThree = haversine(mGPS_three, sGPS)*1000

            # Master GPS 정보에 저장
            mGPS_one.append(distanceOne)
            mGPS_two.append(distanceTwo)
            mGPS_three.append(distanceThree)
            
            # Master의 길이 최대값 삭제
            #for master in mGPS :
            #    if max(mGPS[:][2]) in master :
            #        mGPS.remove(master)
            print("# =========================================================================== #")
            # 연결된 Master의 리스트
            connected_Master = copy.deepcopy(mGPS)
            print(" Linked Master One GPS [ " + str(connected_Master[0][0]) + ", " + str(connected_Master[0][1]) + " ]")
            #print(" Linked Master Two GPS [ " + str(connected_Master[1][0]) + ", " + str(connected_Master[1][1]) + " ]")

            # 위도 경도를 이용한 방위각 계산
            # 1번 마스터
            gpsOneCurrentAzimuth = calculateAzimuth(sGPS[0], sGPS[1], connected_Master[0][0], connected_Master[0][1])
            # 2번 마스터
            gpsTwoCurrentAzimuth = calculateAzimuth(sGPS[0], sGPS[1], connected_Master[1][0], connected_Master[1][1])

            # 차량 방향 값
            #IMU = open("./IMU/heading.txt", 'r')
            #IMU_headingData = float(IMU.readline())
            #IMU.close()
            IMU = tailer.tail(open('./IMU/heading.txt'), 1)
            IMU_headingData = float(IMU[0])

            if on == 0 :
                heading = 360 - IMU_headingData
                print(" GPS one Angle : " + str(gpsOneCurrentAzimuth))
                print(" GPS two Angle : " + str(gpsTwoCurrentAzimuth))
                print(" Heading : " + str(heading))


                # if gpsOneCurrentAzimuth > heading :
                #     angle = gpsOneCurrentAzimuth - heading
                #     angular_velocity = 1.7 * angle / 10
                #     rotator.write(panF_Left)
                #     time.sleep(angular_velocity)
                #     rotator.write(panF_Stop)

                # elif gpsOneCurrentAzimuth < heading :
                #     angle = heading - gpsOneCurrentAzimuth
                #     angular_velocity = 1.7 * angle / 10
                #     rotator.write(panF_Right)
                #     time.sleep(angular_velocity)
                #     rotator.write(panF_Stop)

                # currentAntennaOne = angle

                if gpsTwoCurrentAzimuth > heading :
                    angle = gpsTwoCurrentAzimuth - heading
                    angular_velocity = 1.7 * angle / 10
                    rotator.write(panS_Right)
                    time.sleep(angular_velocity)
                    rotator.write(panS_Stop)

                elif gpsTwoCurrentAzimuth < heading :
                    angle = heading - gpsTwoCurrentAzimuth
                    angular_velocity = 1.7 * angle / 10
                    rotator.write(panS_Left)
                    time.sleep(angular_velocity)
                    rotator.write(panS_Stop)

                currentAntennaTwo = angle

                on = 1
                # 차량기준 안테나 위치
                # print(" Current Antenna One : " + str(currentAntennaOne))
                print(" Current Antenna Two : " + str(currentAntennaTwo))

            else :
                heading = 360 - IMU_headingData
                print(" GPS one Angle : " + str(gpsOneCurrentAzimuth))
                print(" GPS two Angle : " + str(gpsTwoCurrentAzimuth))
                print(" Heading : " + str(heading))

                # if gpsOneCurrentAzimuth > heading :
                #     angle = gpsOneCurrentAzimuth - heading
                #     if currentAntennaOne > angle :
                #         rotateAngle = currentAntennaOne - angle
                #         angular_velocity = 1.7 * rotateAngle / 10
                #         rotator.write(panF_Left)
                #         time.sleep(angular_velocity)
                #         rotator.write(panF_Stop)

                #         tmp = currentAntennaOne
                #         currentAntennaOne = tmp - rotateAngle
                #     elif currentAntennaOne < angle :
                #         rotateAngle = angle - currentAntennaOne
                #         angular_velocity = 1.7 * rotateAngle / 10
                #         rotator.write(panF_Right)
                #         time.sleep(angular_velocity)
                #         rotator.write(panF_Stop)

                #         tmp = currentAntennaOne
                #         currentAntennaOne = tmp + rotateAngle
                        
                # elif gpsOneCurrentAzimuth < heading :
                #     angle = heading - gpsOneCurrentAzimuth
                #     if currentAntennaOne > angle :
                #         rotateAngle = currentAntennaOne - angle
                #         angular_velocity = 1.7 * rotateAngle / 10
                #         rotator.write(panF_Right)
                #         time.sleep(angular_velocity)
                #         rotator.write(panF_Stop)

                #         tmp = currentAntennaOne
                #         currentAntennaOne = tmp - rotateAngle

                #     elif currentAntennaOne < angle :
                #         rotateAngle = angle - currentAntennaOne
                #         anguler_velocity = 1.7 * rotateAngle / 10
                #         rotator.write(panF_Left)
                #         time.sleep(angular_velocity)
                #         rotator.write(panF_Stop)
                        
                #         tmp = currentAntennaOne
                #         currentAntennaOne = tmp + rotateAngle
                # print(" Current Antenna : " + str(currentAntennaOne))
                if gpsTwoCurrentAzimuth > heading :
                    angle = gpsTwoCurrentAzimuth - heading
                    if currentAntennaTwo > angle :
                        rotateAngle = currentAntennaTwo - angle
                        angular_velocity = 1.7 * rotateAngle / 10
                        rotator.write(panS_Right)
                        time.sleep(angular_velocity)
                        rotator.write(panS_Stop)

                        tmp = currentAntennaTwo
                        currentAntennaTwo = tmp - rotateAngle
                    elif currentAntennaTwo < angle :
                        rotateAngle = angle - currentAntennaTwo
                        angular_velocity = 1.7 * rotateAngle / 10
                        rotator.write(panS_Left)
                        time.sleep(angular_velocity)
                        rotator.write(panS_Stop)

                        tmp = currentAntennaTwo
                        currentAntennaTwo = tmp + rotateAngle
                        
                elif gpsTwoCurrentAzimuth < heading :
                    angle = heading - gpsTwoCurrentAzimuth
                    if currentAntennaTwo > angle :
                        rotateAngle = currentAntennaTwo - angle
                        angular_velocity = 1.7 * rotateAngle / 10
                        rotator.write(panS_Left)
                        time.sleep(angular_velocity)
                        rotator.write(panS_Stop)

                        tmp = currentAntennaTwo
                        currentAntennaTwo = tmp - rotateAngle

                    elif currentAntennaTwo < angle :
                        rotateAngle = angle - currentAntennaTwo
                        anguler_velocity = 1.7 * rotateAngle / 10
                        rotator.write(panS_Right)
                        time.sleep(angular_velocity)
                        rotator.write(panS_Stop)
                        
                        tmp = currentAntennaTwo
                        currentAntennaTwo = tmp + rotateAngle
                print(" Current Antenna : " + str(currentAntennaTwo))
                time.sleep(0.5)
    elif modeSelect == "2":
        # GPS 만으로 안테나 로테이팅 하는 테스트 프로그램
        angle = 0
        heading = 0
        print(" Test 2 Mode ")
        print(" Configuration : Fixed Master 1, Moving Slave 1")
        # 첫번째 Master 고정 값 불러옴
        mGPS = [37.48471, 127.11914]
        mGPS_lat = mGPS[0]
        mGPS_lng = mGPS[1]
        

        # print(" Do you Want to change Master GPS ?")
        # print(" 1. Y        2. N")
        # change = input(" input : ")
        # if change == "1" :
        #     mGPS_lat = float(input(" Master lat : "))
        #     mGPS_lng = float(input(" Master lng : "))
        #     print(" Changed !")
            
        while True :
            print("# ====================================================== #")
            print(" Current Master GPS [ " + str(mGPS_lat) + ", " + str(mGPS_lng) + " ]")

            sGPSData =tailer.tail(open('./GPS/SlaveGPS.txt'), 1)
            
            sGPS_parsedData = sGPSData[0].split(' ')
            sGPS_lat = float(sGPS_parsedData[0])
            sGPS_lng = float(sGPS_parsedData[1])
            sGPS = [sGPS_lat, sGPS_lng]

            print(" Current Slave GPS [ " + str(sGPS_lat) + ", " + str(sGPS_lng) + " ]")
            currentAzimuth = calculateAzimuth(sGPS_lat, sGPS_lng, mGPS_lat, mGPS_lng)

            # IMU heading read
            IMU = tailer.tail(open('./IMU/heading.txt'), 1)
            IMU_headingData = float(IMU[0])

            if heading == 0 :
                angle = currentAzimuth
                print(" Angle : " + str(currentAzimuth))
                angular_velocity = 1.7 * angle / 10

                # 로테이터 회전
                rotator.write(panS_Left)
                time.sleep(int(angular_velocity))
                rotator.write(panS_Stop)

                heading = currentAzimuth

            else :  
                angle = heading - currentAzimuth
                print(" Angle : " + str(currentAzimuth))
                
                if angle > 0 :
                    angular_velocity = 1.7 * angle / 10

                    rotator.write(panS_Right)
                    time.sleep(angular_velocity)
                    rotator.write(panS_Stop)
                elif angle < 0 :
                    angular_velocity = 1.7 * (-angle) / 10  
                    rotator.write(panS_Left)
                    time.sleep(angular_velocity)
                    rotator.write(panS_Stop)

                else : 
                    rotator.write(panS_Stop)
                heading = currentAzimuth
            time.sleep(0.5)

    elif modeSelect == "3":
        # IMU 만으로 로테이터 제어하는 프로그램 마스터 슬레이브 GPS 고정
        print(" Test 3 Mode ")
        mGPS_lat = 37.483832
        mGPS_lng = 127.118425
        sGPS_lat = 37.485250
        sGPS_lng = 127.122765

        angle = 0
        heading = 0
        while True :
            
            print(" Current Master GPS [ " + str(mGPS_lat) + ", " + str(mGPS_lng) + " ]") 
            print(" Current Slave GPS [ " + str(sGPS_lat) + ", " + str(sGPS_lng) + " ]")

            gpsAzimuth = calculateAzimuth(sGPS_lat, sGPS_lng, mGPS_lat, mGPS_lng)

            # IMU Data
            IMU = open("./IMU/heading.txt", "r")
            IMU_headingData = float(IMU.readline())
            IMU.close()

            if heading == 0 :
                angle = gpsAzimuth
                print(" Angle : " + str(gpsAzimuth))

                angular_velocity = 1.7 * angle / 10

                # 로테이터 회전
                rotator.write(panLeft)
                time.sleep(int(angular_velocity))
                rotator.write(panStop)
                currentIMUheading = IMU_headingData
                print(" Current IMU Heading : " + str(currentIMUheading))
                heading = gpsAzimuth
            else :
                IMU_angle = currentIMUheading - IMU_headingData
                
                sumAngle = gpsAzimuth + IMU_angle
                
                if sumAngle > 360 :
                    sumAngle -= 360 
                

                angle = IMU_angle
                print(" IMU Angle : " +str(IMU_angle))
                print(" Angle : " + str(sumAngle))
                if IMU_angle > 0 :
                    angular_velocity = 1.7 * IMU_angle / 10

                    rotator.write(panRight)
                    time.sleep(angular_velocity)
                    rotator.write(panStop)
                elif IMU_angle < 0 :
                    angular_velocity = 1.7 * (-IMU_angle) / 10  
                    rotator.write(panLeft)
                    time.sleep(angular_velocity)
                    rotator.write(panStop)

                else : 
                    rotator.write(panStop)
                currentIMUheading = IMU_headingData
                heading = sumAngle
            time.sleep(3)

    elif modeSelect == "4" :
        rotator.write(panRight)
        time.sleep(60)
        rotator.write(panStop)
    else:
        print(" Test case is not updated")
        continue

# # print(" Test 1 Mode ")
# #        print(" 1. Auto     2. Manual     3. Before menu")
#         masterSelect = input(" input : ")
#         if masterSelect == "1":
#             print(" Test 1 Mode : Auto")

#             # 첫번째 Master 고정 값 불러옴
#             mGPS_one_lat = 37.48476
#             mGPS_one_lng = 127.11905

#             print("Current Slave GPS [ " + sGPS_lat + ", " + sGPS_lng + " ]")
#             print("Current Master GPS [ " + str(mGPS_one_lat) + ", " + str(mGPS_one_lng) + " ]")

#                 # 슬레이브와 마스터 사이의 방위각 계산
#             azimuth_one = 360 - calculateAzimuth(float(sGPS_lat), float(sGPS_lng), mGPS_one_lat, mGPS_one_lng)
#             print("Azimuth : " + str(azimuth_one))
#             # 회전 각 속도 계산
#             angular_velocity = 1.8 * azimuth_one / 10
#             # 로테이터 회전
#             rotator.write(panLeft)
#             time.sleep(int(angular_velocity))
#             rotator.write(panStop)


#             time.sleep(10)

#             # 두번째 Master 고정 값 불러옴
#             mGPS_two_lat = 37.48500
#             mGPS_two_lng = 127.11941

#             print("Current Slave GPS [ " + sGPS_lat + ", " + sGPS_lng + " ]")
#             print("Current Master GPS [ " + str(mGPS_two_lat) + ", " + str(mGPS_two_lng) + " ]")

#             # 슬레이브와 마스터 사이의 방위각 계산
#             azimuth_two = 360 - calculateAzimuth(float(sGPS_lat), float(sGPS_lng), mGPS_two_lat, mGPS_two_lng)
#             print("Azimuth : " + str(azimuth_two))
#             tmp = azimuth_one - azimuth_two
#             print(" Difference : " + str(tmp))
#             if tmp > 0:
#                 # 회전 각 속도 계산
#                 angular_velocity = 1.8 * tmp / 10

#                 # 로테이터 회전
#                 rotator.write(panRight)
#                 time.sleep(int(angular_velocity))
#                 rotator.write(panStop)
#             elif tmp < 0:
#                 # 회전 각 속도 계산
#                 angular_velocity = 1.8 * (-tmp) / 10

#                 # 로테이터 회전
#                 rotator.write(panLeft)
#                 time.sleep(int(angular_velocity))
#                 rotator.write(panStop)

#         elif masterSelect == "2":
            

#             # 첫번째 Master 고정 값 불러옴
#             mGPS_one_lat = 37.48476
#             mGPS_one_lng = 127.11905

#             print("Current Slave GPS [ " +sGPS_lat + ", " + sGPS_lng + " ]")
            

#             fileIMU = open('./IMU/heading.txt', 'r')
#             IMUheading = fileIMU.readline()
#             print("Heading : " + IMUheading)

#             print("Current Master GPS [ " + str(mGPS_one_lat) + ", " + str(mGPS_one_lng) + " ]")

#             # 슬레이브와 마스터 사이의 방위각 계산
#             azimuth_one = 360 - calculateAzimuth(float(sGPS_lat), float(sGPS_lng), mGPS_one_lat, mGPS_one_lng)
#             print("Azimuth : " + str(azimuth_one))
#             # 회전 각 속도 계산
#             angular_velocity = 1.8 * azimuth_one / 10

#             # 로테이터 회전
#             rotator.write(panLeft)
#             time.sleep(int(angular_velocity))
#             rotator.write(panStop)

#             # 두번째 Master 고정 값 불러옴
#             mGPSFile_two = open("./GPS/masterGPS_two.txt", 'r')
#             mGPSData_two = mGPSFile_two.readline()
#             mGPS_parsedData = mGPSData_two.split(' ')
#             mGPS_two_lat = mGPS_parsedData[0]
#             mGPS_two_lng = mGPS_parsedData[1]
#             mGPSFile_two.close()

#             print("Current Slave GPS [ " + sGPS_lat + ", " + sGPS_lng + " ]")
#             print("Current Master GPS [ " + mGPS_two_lat + ", " + mGPS_two_lng + " ]")

#             # 슬레이브와 마스터 사이의 방위각 계산
#             azimuth_two = 360 - calculateAzimuth(sGPS_lat, sGPS_lng, mGPS_two_lat, mGPS_two_lng)
#             print("Azimuth : " + str(azimuth_two))
#             tmp = azimuth_one - azimuth_two
#             print(" Difference : " + str(tmp))
#             if tmp > 0:
#                 # 회전 각 속도 계산
#                 angular_velocity = 1.7 * tmp / 10

#                 # 로테이터 회전
#                 rotator.write(panRight)
#                 time.sleep(int(angular_velocity))
#                 rotator.write(panStop)
#             elif tmp < 0:
#                 # 회전 각 속도 계산
#                 angular_velocity = 1.7 * (-tmp) / 10

#                 # 로테이터 회전
#                 rotator.write(panLeft)
#                 time.sleep(int(angular_velocity))
#                 rotator.write(panStop)
#         elif masterSelect == "3":
#             continue
#         else:
#             print("No Mode in Test")
#             continue