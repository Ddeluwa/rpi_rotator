import math
import serial
import serial.rs485
import time
import sys,signal,os

pan_FLeft = [0xFF, 0x02, 0x00, 0x04, 0x01, 0x00, 0x07]
pan_FRight = [0xFF, 0x02, 0x00, 0x02, 0x40, 0x00, 0x44]
pan_FStop = [0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x02]

pan_SLeft = [0xFF, 0x01, 0x00, 0x04, 0x01, 0x00, 0x06]
pan_SRight = [0xFF, 0x01, 0x00, 0x02, 0x40, 0x00, 0x43]
pan_SStop = [0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01]



def calculateAzimuth(SGPSlat, SGPSlng, MGPSlat, MGPSlng) :
	SGPSlat_rad = float(SGPSlat) * math.pi / 180
	MGPSlat_rad = float(MGPSlat) * math.pi / 180
	lng_diff_rad = (float(SGPSlng) - float(MGPSlng)) * math.pi / 180

	y = math.sin(lng_diff_rad) * math.cos(MGPSlat_rad)
	x = math.cos(SGPSlat_rad) * math.sin(MGPSlat_rad) - math.sin(SGPSlat_rad) * math.cos(MGPSlat_rad) * math.cos(lng_diff_rad)
	arctanXY = math.atan2(y,x)
	
	angle = (int(arctanXY * 180 / math.pi)+360) % 360
	return angle




try :
# Rotator Setting
	rotator = serial.Serial(
		port = '/dev/ttyUSB0',
		baudrate = 9600,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		bytesize = serial.EIGHTBITS,
		timeout = 1
	)


	while True :
		Slavefile = open("./GPS/SlaveGPS.txt", 'r')
		
		if Slavefile.readline() == "" :
			Slavefile.close()
			print("Slave GPS Error, Manually")
			
			SlaveGPSlat = float(input("lat : "))
			SlaveGPSlng = float(input("lng : ")) 
		else :
			Slavefile.close()
			Slavefile = open("./GPS/SlaveGPS.txt", 'r')
			SlaveGPSData = Slavefile.readline()
			#print(SlaveGPSData)
			SlaveGPSParsedData = SlaveGPSData.split(' ')
			SlaveGPSlat = SlaveGPSParsedData[0]
			SlaveGPSlng = SlaveGPSParsedData[1]
			Slavefile.close()
		
		
		print(" Mode Select ") 
		print(" 1. auto 2. manual")
		modeSelect = input(" Input : ")
		if modeSelect == "1" :
			print(" Master GPS Auto ")
			print(" 1. Y	2. N	")
			masterSelect = input(" Input : ")
			if masterSelect == "1" :
				Masterfile = open("./GPS/MasterGPS.txt", 'r')
				MasterGPSData = Masterfile.readline()
				MasterGPSParsedData = MasterGPSData.split(' ')
				MasterGPSlat = MasterGPSParsedData[0]
				MasterGPSlng = MasterGPSParsedData[1]
				Masterfile.close()
				print("Master GPS [ " + MasterGPSlat + ", " + MasterGPSlng +" ]")
				azimuth = calculateAzimuth(SlaveGPSlat, SlaveGPSlng, MasterGPSlat, MasterGPSlng) 
				print(azimuth)
				
				#angle = 360 - azimuth
				angleSecond = 1.5 * azimuth / 10
				
				rotator.write(pan_FLeft)
				time.sleep(int(angleSecond))
				rotator.write(pan_SLeft)
				time.sleep(int(angleSecond))
				rotator.write(pan_FStop)
				rotator.write(pan_SStop)
					
			elif masterSelect == "2" :
				MasterGPSlat = float(input("MasterGPS lat : "))
				MasterGPSlng = float(input("MasterGPS lng : "))
				azimuth = calculateAzimuth(SlaveGPSlat, SlaveGPSlng, MasterGPSlat, MasterGPSlng) 
				#angle = 360 - azimuth
				angleSecond = 1.5 * azimuth / 10

				rotator.write(pan_FLeft)
				rotator.write(pan_SLeft)
				time.sleep(int(angleSecond))
				rotator.write(pan_FStop)
				rotator.write(pan_SStop)
					 
		elif modeSelect == "2" :
			print(" Manuel Mode ") 
			print(" 1. Key 2. Auto ")
			manualSelect = input(" Input : ")

			if manualSelect == "1" :
				left = 0
				right = 0
				while True:
					print(" =============== Key Manuel Mode  ================= ") 
					print("       L : left R : right I : Initialization	   ")
					print(" ================================================== ")
					actionCommand = input(" Command : ")
					if actionCommand == "L" :
						rotator.write(pan_FLeft)
						time.sleep(1.75)
						left = left + 1
						print("Count Left : " + str(left))
						rotator.write(pan_FStop)
					elif actionCommand == "R" :
						rotator.write(pan_FRight)
						time.sleep(1.75)
						right = right + 1
						print("Count right : " + str(right))
						rotator.write(pan_FStop)
					elif actionCommand == "I" :
						rotator.write(pan_FRight)
						time.sleep(55)
						left = 0
						right = 0
						rotator.write(pan_FStop)
					elif actionCommand == "F" :
						break
 	
			elif manualSelect == "2" :
				for turn in range(1,35) :
					rotator.write(pan_FStop)
					rotator.write(pan_SStop)
					rotator.write(pan_FLeft)
					rotator.write(pan_SLeft)
					time.sleep(1.6)
					
					rotator.write(pan_FStop)
					rotator.write(pan_SStop)
					time.sleep(3)
				for turn in range(1,35) :
					rotator.write(pan_FStop)
					rotator.write(pan_SStop)
					rotator.write(pan_FRight)
					rotator.write(pan_SRight)
					time.sleep(1.6)
					rotator.write(pan_FStop)
					rotator.write(pan_SStop)
					time.sleep(3)
			else :
				continue
		else :
			continue				
# 1. Rotator Stop
#rotator.write(rotatorControl.Stop())
# 2. Rotator Pan Right
#rotator.write(rotatorControl.PanRight())
#time.sleep(10)
#rotator.write(rotatorControl.Stop())
# 3. Rotator Pan Light
#rotator.write(rotatorControl.PanLeft())
#time.sleep(10)
#rotator.write(rotatorControl.Stop())
            

except KeyboardInterrupt:
	print("Ctrl + C")
	rotator.write(panStop)
	rotator.close()

except (OSError, serial.SerialException):
	print("Serial port check")
	rotator.write(panStop)
	rotator.close()
