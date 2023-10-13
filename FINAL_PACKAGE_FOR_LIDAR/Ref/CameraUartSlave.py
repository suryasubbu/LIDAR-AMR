import serial
import time
from random import seed
from random import random
import numpy as np
import subprocess

BT_FLAG = 0

if(BT_FLAG):
    import BtDataMethods

#seed(1)

port = serial.Serial("/dev/ttyAMA1", baudrate=115200, timeout=0.2)
#port = serial.Serial("/dev/serial0", baudrate=115200, timeout=0.2)
#port = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=0.5)

################### UART COMMANDS ############################

ShutdownCameraRasp = 100
RequestCameraRatioAngle = 101
IndicateBotDirection = 102
IndicateCameraLane = 103

##############################################################

RET_SUCCESS = 0
RET_COMMANDFAIL = 1
RET_RESPONSEFAIL = 2
RET_PACKETCORRUPT = 3
RET_TIMEOUT = 4
RET_INVALIDCOMMAND = 5
RET_CHECKSUM_FAIL = 6
RESPONSE_TIMEOUT = 0.2  # Check if response packet prepared within this time
RESPONSE_PACKET_WAIT_TIME = 0.01 # Wait this long for the main code to prepare the response packet

TIMEOUT_VAL = 2
COMMAND_CODE_LENGTH = 3
MAX_LEN_RECEIVE_PACKET = 100   # Can we modified if the length increases
RESPONSE_CODE_ERROR_OFFSET = 500

RxRetVal = RET_TIMEOUT
RxReceivedPacket = []

##############################################################
def SendUARTResponsePacket (response_data):

    CommandCode = response_data[0]
    Arg_count = response_data[1]
    Args = response_data[2]

    port.flushInput()   # Clear any residual data in receive channel

    ######################################
    # Pack and send command with arguments
    ######################################

    send_packet = 'A' + str(CommandCode) + 'S' + str(Arg_count)

    for i in range(Arg_count):
        send_packet = send_packet + 'S' + str(Args[i])

    check_sum = np.sum(response_data[2])+response_data[0]+response_data[1]

    #print("Sent data checksum = ",check_sum)

    send_packet = send_packet + 'S' + str(check_sum) + 'Z'

    #print("send_packet = ",send_packet)
 
    port.write(send_packet.encode())

def ReceiveUARTPacket():

    global RxRetVal, RxReceivedPacket

    retVal = RET_SUCCESS
    received_packet = []

    ###############################################
    # Look for response packet for the sent command
    ###############################################

    try:
        char_count = 0
        first_char = port.read()
        data_chunk = first_char.decode()

        start_time = time.time()
    
        while((data_chunk != 'A') and ((time.time()-start_time) < TIMEOUT_VAL)):
            first_char = port.read()
            data_chunk = first_char.decode()

        if(data_chunk != 'A'):
            retVal = RET_TIMEOUT
            RxReceivedPacket = received_packet
            RxRetVal = retVal
            return 

        next_char = port.read().decode()

        while((next_char != 'Z') and (char_count < MAX_LEN_RECEIVE_PACKET)):
            data_chunk += next_char
            next_char = port.read().decode()
            char_count = char_count + 1

        ResponseCode = int(data_chunk[1:COMMAND_CODE_LENGTH+1]);

        #print("Response Code = ",ResponseCode, "; data_chunk = ",data_chunk)

        # Start preparing the response pack

        received_packet.append(ResponseCode)

        if(char_count >=  MAX_LEN_RECEIVE_PACKET):
            retVal = RET_PACKETCORRUPT
            received_packet[0] += RESPONSE_CODE_ERROR_OFFSET
            RxReceivedPacket = received_packet
            RxRetVal = retVal
            return 

        ################################################
        # Seperate out response arg count and arguments
        ################################################
        ReceivedPacket = data_chunk[1+COMMAND_CODE_LENGTH+1:];

        SeperatorIndex = ReceivedPacket.index('S')

        ThisPacketDataCount = int(ReceivedPacket[0:SeperatorIndex])
        received_packet.append(ThisPacketDataCount)

        if(ThisPacketDataCount > 0):

            ThisPacketArguments = ReceivedPacket[SeperatorIndex+1:]

            #print("SeperatorIndex = ",SeperatorIndex,"; ThisPacketDataCount = ",ThisPacketDataCount, "; ThisPacketArguments = ",ThisPacketArguments)

            ThisArguments = np.zeros(ThisPacketDataCount)

            # Extract N-1 arguments

            for i in range(ThisPacketDataCount-1):
                SeperatorIndex = ThisPacketArguments.index('S')
                ThisArguments[i] = float(ThisPacketArguments[0:SeperatorIndex])
                ThisPacketArguments = ThisPacketArguments[SeperatorIndex+1:]

            # Extract last argument

            SeperatorIndex = ThisPacketArguments.index('S')
            ThisArguments[-1] = float(ThisPacketArguments[0:SeperatorIndex])
            #ThisArguments[ThisPacketDataCount-1] = float(ThisPacketArguments)

            received_packet.append(ThisArguments)

            received_checksum = float(ThisPacketArguments[SeperatorIndex+1:])
            calculated_check_sum = np.sum(received_packet[2])+received_packet[0]+received_packet[1]

            print("received_checksum = ",received_checksum,"; calculated_check_sum = ",calculated_check_sum)

            if(received_checksum == calculated_check_sum):
                print("Checksum matches")
            else:
                retVal = RET_CHECKSUM_FAIL
                print("Checksum failed to match")

        RxReceivedPacket = received_packet
        RxRetVal = retVal
        return

    except Exception as e:
        RxRetVal = RET_COMMANDFAIL
        print("I FOUND EXCEPTION")
        print(str(e))
        return

ratio = 0
curr_steering_angle = 0
response_packet = []
PacketReceivedFlag = 0
Bot_Direction = 0
Camera_Lane_Flag = 0

#my_ratio = 0
#my_curr_steering_angle = 0

# This is for receiving Obstruction detection data from Qualcomm board via Bluetooth
VERY_LARGE_DISTANCE = 1000
Camera_Obstruction_Dist = VERY_LARGE_DISTANCE

def CameraDataShare():

    global RxRetVal, RxReceivedPacket, response_packet, PacketReceivedFlag
    global ratio, curr_steering_angle, Bot_Direction, Camera_Lane_Flag
    global Camera_Obstruction_Dist
    #global my_curr_steering_angle, my_ratio
    if(PacketReceivedFlag):
        response_packet = []

        retVal = RxRetVal
        received_packet = RxReceivedPacket


        if(received_packet[0] == RequestCameraRatioAngle):

            if(BT_FLAG):
                try:
                    Value = int(BtDataMethods.Camera_Obs_distance)
                    Camera_Obstruction_Dist = Value
                except ValueError:
                    Camera_Obstruction_Dist = VERY_LARGE_DISTANCE
                    print("EXCEPTION: Camera data not available. So, setting to very large value")
                print("CAMERA: OBS DISTANCE = ",Camera_Obstruction_Dist)

                Args = [ratio, curr_steering_angle, Camera_Obstruction_Dist]
            else:
                Args = [ratio, curr_steering_angle]

            response_packet = [RequestCameraRatioAngle, len(Args), Args]

        elif(received_packet[0] == ShutdownCameraRasp):
            print("Received request to shutdown the Raspberry")
            response_packet = [ShutdownCameraRasp, 0, []]
        elif(received_packet[0] == IndicateBotDirection):
            Bot_Direction = received_packet[2][0]
            print("############################################")
            print("Received request about the bot direction. Bot_Direction is ", Bot_Direction,"; time = ",time.time())
            print("############################################")
            response_packet = [IndicateBotDirection, 0, []]
        elif(received_packet[0] == IndicateCameraLane):
            Camera_Lane_Flag = received_packet[2][0]
            print("############################################")
            print("Received request about the camera lane. Camera_Lane_Flag is ", Camera_Lane_Flag,"; time = ",time.time())
            print("############################################")
            response_packet = [IndicateCameraLane, 0, []]
        else:
            print("Invalid command code")
            response_packet = [received_packet[0] + RESPONSE_CODE_ERROR_OFFSET, 0,0]

        print("CD Rx",received_packet,"RV = ",retVal,"CD Tx",response_packet)

        PacketReceivedFlag = 0


def CameraUARTCommandHandler():

    global RxRetVal, RxReceivedPacket, response_packet, PacketReceivedFlag
    #print("INSIDE CameraUARTCommandHandler")

    ReceiveUARTPacket()

    if(RxRetVal == RET_SUCCESS):
        PacketReceivedFlag = 1;
        #print("CameraUARTCommandHandler RxReceivedPacket:",RxReceivedPacket,"; RxRetVal = ",RxRetVal)

        response_packet = []
        CameraDataShare()

    elif(RxReceivedPacket == []):
        PacketReceivedFlag = 0
        print("Packet not received")
        return
    else:
        PacketReceivedFlag = 0
        print("Error in packet received. Preparing default packet")
        response_packet = [RxReceivedPacket[0] + RESPONSE_CODE_ERROR_OFFSET, 1, [RxRetVal]]

    # Here, prepare the response packet according to the command received

    #print("CameraUARTCommandHandler: response_packet = ",response_packet)
    SendUARTResponsePacket(response_packet)

def CameraUartSlaveMethod():
    global RxReceivedPacket
    #print("GOT INSIDE UART SLAVE HANDLER")
    while (1):
        #print("Call CameraUARTCommandHandler")
        CameraUARTCommandHandler()

        if(RxReceivedPacket != []):
            if(RxReceivedPacket[0] == ShutdownCameraRasp):
                print("#######################################")
                print("Received Shutdown command. Sutting down camera module")
                print("#######################################")
                time.sleep(5)
                subprocess.Popen(['sudo','shutdown','-h','now'])


