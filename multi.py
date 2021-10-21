import math
import serial
import serial.rs485
import time
from multiprocessing import Process

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
def gpsRotator(sGPS_lat, sGPS_lng, mGPS_lat, mGPS_lng, rotator, heading, angle, type):
    print(" Current Master GPS [ " + str(mGPS_lat) + ", " + str(mGPS_lng) + " ]")
    print(" Current Slave GPS [ " + str(sGPS_lat) + ", " + str(sGPS_lng) + " ]")

    currentAzimuth = calculateAzimuth(sGPS_lat, sGPS_lng, mGPS_lat, mGPS_lng)
    if heading == 0 :
        angle = currentAzimuth
        print(" Angle : " + str(currentAzimuth))
        angular_velocity = 1.7 * angle / 10

        # 로테이터 회전
        if type == "F" :
            print("pan First is moving...")
            rotator.write(panF_Left)
            time.sleep(angular_velocity)
            rotator.write(panF_Stop)
        elif type == "S" :
            print("pan Second is moving...")
            rotator.write(panS_Left)
            time.sleep(angular_velocity)
            rotator.write(panS_Stop)

        heading = currentAzimuth


if __name__ == "__main__" :
    rotator = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate = 9600,
        parity = serial.PARITY_NONE,
        stopbits = serial.STOPBITS_ONE,
        bytesize= serial.EIGHTBITS,
        timeout = 1
    )

    angle = 0
    heading = 0

    while True :
        mGPS_lat = 37.48476
        mGPS_lng = 127.11905
        sGPS_lat = 37.485250
        sGPS_lng = 127.122765

        proc1 = Process(target=gpsRotator, args=(sGPS_lat, sGPS_lng, mGPS_lat, mGPS_lng, rotator, heading, angle, "F"))
        proc2 = Process(target=gpsRotator, args=(sGPS_lat, sGPS_lng, mGPS_lat, mGPS_lng, rotator, heading, angle, "S"))

        proc1.start()
        proc2.start() 

        proc1.join()
        proc2.join()
