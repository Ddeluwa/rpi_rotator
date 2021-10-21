from reference import *

try :
    IMUmodule = openSerial('/dev/ttyUSB1', 9600, serial.PARITY_NONE, serial.STOPBITS_ONE, serial.EIGHTBITS, 1)

    while True:
        IMU_rowDataFile = open("./IMU_row_data.txt", 'a')
        IMU_angleDataFile = open("./IMU_angle_data.txt", 'w')

        dataString = "55 53"
        i = 0
        if IMUmodule.read().hex() == "55" :
            if IMUmodule.read().hex() == "53" :
                while i < 9 :
                    IMU_rcv = IMUmodule.read().hex()
                    dataString = dataString + " " + IMU_rcv
                    i = i + 1
                IMU_rowDataFile.write(dataString + '\n')
                print(dataString)
                angleData = calculateIMURowData(dataString)
                print(angleData)
                IMU_angleDataFile.write(angleData)
                IMU_rowDataFile.close()
                IMU_angleDataFile.close()
                
            else :
                continue
        else :
            continue
        
        

except KeyboardInterrupt :
    print("Ctrl + C")
    closeSerial(IMUmodule)
    
