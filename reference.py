import serial
import serial.rs485
import time

def openSerial(sPort, sBaudrate, sParity, sStopbits, sBytesize, sTimeout) :
    ser = serial.Serial(
        port = sPort,
        baudrate = sBaudrate,
        parity = sParity,
        stopbits = sStopbits,
        bytesize = sBytesize,
        timeout = sTimeout
    )
    return ser
    
def closeSerial(ser) :
    ser.close()

def calculateIMURowData(dataString) :
    data = dataString.split(' ')
    # =============================================================== #
    #                            Angle Output                         #
    # =============================================================== #
    #     0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM     #
    # =============================================================== #
    # Calculated formular :                                           #
    #   Roll(X axis) Roll = ((RollH << 8)|RollL)/32768 * 180°         #
    #   Pitch(Y axis) Pitch = ((PitchH << 8)|PitchL)/32768 * 180°     #
    #   Yaw(Z axis) Yaw = ((YawH << 8)|YawL)/32768 * 180°             #
    # =============================================================== #
    #     Version calculated formular : Version = (VH << 8) | VL      #
    #     Checksum :                                                  #
    #   Sum = 0x55 + 0x53 + RollH + RollL + PitchH + PitchL + YawH    #
    #           +YawL + VH + VL                                       #
    # =============================================================== #
    rollL = int(data[2], 16)
    rollH = int(data[3], 16)
    pitchL = int(data[4], 16)
    pitchH = int(data[5], 16)
    yawL = int(data[6], 16)
    yawH = int(data[7], 16)
    tl = int(data[8], 16)
    th = int(data[9], 16)
    sum = int(data[10], 16)

    rollX = (( rollH << 8) | rollL ) / 32768 * 180
    pitchY = (( pitchH << 8 ) | pitchL ) / 32768 * 180
    yawZ = (( yawH << 8 ) | yawL ) / 32768 * 180
    
    angle = str(round(rollX, 2)) + ', ' + str(round(pitchY, 2)) + ', ' + str(round(yawZ, 2)) + '\n'

    return angle


class Rotator :
    dPanLeft = [0xFF, 0x01, 0x00, 0x04, 0x20, 0x00, 0x25]
    dPanRight = [0xFF, 0x01, 0x00, 0x02, 0x20, 0x00, 0x23]
    dTiltUp = [0xFF, 0x01, 0x00, 0x08, 0x00, 0x3F, 0x48]
    dTiltDown = [0xFF, 0x01, 0x00, 0x10, 0x3F, 0x00, 0x50]
    dStop = [0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01]
        

    def PanLeft(self) :
        return self.dPanLeft

    def PanRight(self) :
        return self.dPanRight

    def TiltUp(self) :
        return self.dTiltUp

    def TiltDown(self) :
        return self.dTiltDown

    def Stop(self) :
        return self.dStop



