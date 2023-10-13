import serial
import time
import json
from time import sleep
import CommonCommandHandler
import NetworkInterfaceHandler
from ctypes import *
import ctypes
import numpy as np
import subprocess

# JUST FOR DEBUG PURPOSE
from random import random
from random import seed

seed(1)

with open('/home/pi/StartUpJson.json') as json_file:
    StartUp_data_struct = json.load(json_file)

CameraThreadDelay = StartUp_data_struct['Camera'][0]['CameraThreadDelay']
ExeFlag = StartUp_data_struct['Camera'][0]['ExeFlag']
BT_Enable_Flag = StartUp_data_struct['BOT2'][0]['BT_Enable_Flag']

# Import libc  (not sure whether it is needed)
libc = CDLL('libc.so.6')

# Import BOT library
if(ExeFlag):
    mylib = CDLL('libBOT2.so')
else:
    mylib = CDLL('./libBOT2.so')

port = serial.Serial("/dev/serial0", baudrate=115200, timeout=0.2)  # Orig value timeout 1
#port = serial.Serial("/dev/ttyS0", baudrate=115200, timeout=0.2)  # Orig value timeout 1

mylib.GetCameraBasedPosRatio.argtypes = (ctypes.c_int, ctypes.c_int)
mylib.GetCameraBasedPosRatio.restype = ctypes.c_int

##############################################################

RET_SUCCESS = 0
RET_COMMANDFAIL = 1
RET_RESPONSEFAIL = 2
RET_RESPONSECORRUPT = 3
RET_TIMEOUT = 4

COMMAND_CODE_LENGTH = 3
TIMEOUT_VAL = 0.5   # Seconds wait time Orig Value 2
MAX_LEN_RECEIVE_PACKET = 100   # Can be modified if the length increases
RESPONSE_CODE_ERROR_OFFSET = 500

##############################################################

def SendUARTCommand (sent_data):

    CommandCode = sent_data[0]
    Arg_count = sent_data[1]

    Args = sent_data[2]

    port.flushInput()   # Clear any residual data in receive channel

    ######################################
    # Pack and send command with arguments
    ######################################

    send_packet = 'A' + str(CommandCode) + 'S' + str(Arg_count)

    for i in range(Arg_count):
        send_packet = send_packet + 'S' + str(Args[i])

    check_sum = np.sum(sent_data[2])+sent_data[0]+sent_data[1]

    #print("Sent data checksum = ",check_sum)

    send_packet = send_packet + 'S' + str(check_sum) + 'Z'

    #print("Packet sent = ",send_packet)

    port.write(send_packet.encode())

loop_count = 0

def ReceiveUARTResponse (CommandCode):

    global loop_count

    retVal = RET_SUCCESS
    response_packet = []

    loop_count = loop_count + 1

    ###############################################
    # Look for response packet for the sent command
    ###############################################

    #print("ENTERED ReceiveUARTResponse FUNCTION") 

    try:
        char_count = 0

        #print("POINT 0")

        first_char = port.read()
        #print("BEFORE WHILE LOOP. first_char = ", first_char)

        #print("POINT 1")

        data_chunk = first_char.decode()
        #data_chunk = first_char.decode('utf-8', errors='replace')
        #data_chunk = first_char.decode('utf-8')
        #print("BEFORE WHILE LOOP. First char = ", data_chunk)

        #print("POINT 2")

        start_time = time.time()

        #print("BEFORE TIMEOUT WHILE LOOP. First char = ", data_chunk)

        while((data_chunk != 'A') and ((time.time()-start_time) < TIMEOUT_VAL)):
            first_char = port.read()
            #print("INSIDE WHILE LOOP. first_char = ", first_char)
            data_chunk = first_char.decode()
            #data_chunk = first_char.decode('utf-8', errors='replace')
            #data_chunk = first_char.decode('utf-8')
            #print("INSIDE WHILE LOOP. First char = ", data_chunk)

        #print("AFTER TIMEOUT WHILE LOOP")

        if(data_chunk != 'A'):
            retVal = RET_TIMEOUT
            print("NO RESPONSE FROM SLAVE. TIMEOUT")
            return retVal, response_packet

        next_char = port.read().decode()
        #next_char = port.read().decode('utf-8', errors='replace')
        #next_char = port.read().decode('utf-8')

        while((next_char != 'Z') and (char_count < MAX_LEN_RECEIVE_PACKET)):
            data_chunk += next_char
            #next_char = port.read().decode()
            #next_char = port.read().decode('utf-8', errors='replace')
            next_char = port.read().decode('utf-8')
            char_count = char_count + 1

        if(char_count >= MAX_LEN_RECEIVE_PACKET):
            retVal = RET_RESPONSECORRUPT
            return retVal, response_packet

        #print("data_chunk = ",data_chunk,"; char_count = ",char_count)

        ResponseCode = int(data_chunk[1:COMMAND_CODE_LENGTH+1]);

        #print("received ResponseCode = ",ResponseCode)

        if(ResponseCode != CommandCode):
            if((ResponseCode-RESPONSE_CODE_ERROR_OFFSET) == CommandCode):
                print("Slave responded but has sent error resonse")
                retVal = RET_RESPONSEFAIL
            else:
                print("Received response but corrupted")
                retVal = RET_RESPONSECORRUPT
            return retVal, response_packet

        # Start preparing the response pack

        response_packet.append(ResponseCode)

        ################################################
        # Seperate out response arg count and arguments
        ################################################
        ReceivedPacket = data_chunk[1+COMMAND_CODE_LENGTH+1:];

        SeperatorIndex = ReceivedPacket.index('S')

        ThisPacketDataCount = int(ReceivedPacket[0:SeperatorIndex])
        response_packet.append(ThisPacketDataCount)

        if(ThisPacketDataCount > 0):

            ThisPacketArguments = ReceivedPacket[SeperatorIndex+1:]

            ThisArguments = np.zeros(ThisPacketDataCount)

            #print("response_packet = ",response_packet,"; ThisPacketDataCount = ",ThisPacketDataCount, "; ThisPacketArguments = ",ThisPacketArguments)

            # Extract N-1 arguments

            for i in range(ThisPacketDataCount-1):
                SeperatorIndex = ThisPacketArguments.index('S')
                This_Arg_Str = ThisPacketArguments[0:SeperatorIndex]
                ThisArguments[i] = float(ThisPacketArguments[0:SeperatorIndex])
                ThisPacketArguments = ThisPacketArguments[SeperatorIndex+1:]

            # Extract last argument
            SeperatorIndex = ThisPacketArguments.index('S')
            ThisArguments[-1] = float(ThisPacketArguments[0:SeperatorIndex])

            #ThisArguments[ThisPacketDataCount-1] = float(ThisPacketArguments)

            response_packet.append(ThisArguments)

            received_checksum = float(ThisPacketArguments[SeperatorIndex+1:])
            calculated_check_sum = np.sum(response_packet[2])+response_packet[0]+response_packet[1]

            #print("received_checksum = ",received_checksum,"; calculated_check_sum = ",calculated_check_sum)

            '''
            if(received_checksum == calculated_check_sum):
                print("Checksum matches")
            else:
            '''
            if(received_checksum != calculated_check_sum):
                retVal = RET_CHECKSUM_FAIL
                print("Checksum failed to match. Making response_packet null")
                response_packet = []

        return retVal, response_packet

    except Exception as e:
        retVal = RET_COMMANDFAIL
        print("I FOUND EXCEPTION")
        print(str(e), loop_count)
        return retVal, response_packet

def BotUARTCommandHandler(sent_data):

    SendUARTCommand(sent_data)

    #print("Command Sent:", sent_data)

    [retVal, response_packet] = ReceiveUARTResponse(sent_data[0])

    print("TD = ", sent_data, "; RD = ",response_packet, "; RV = ",retVal)

    return response_packet

################### UART COMMANDS ############################

ShutdownCameraRasp = 100
RequestCameraRatioAngle = 101
IndicateBotDirection = 102
IndicateCameraLane = 103

##############################################################

def GetCameraBasedPosRatioCLibraryFunction(CameraLanePosOffsetRatio, CameraSteeringAngle):
    if(CommonCommandHandler.EnableCameraModuleFlag):
        return mylib.GetCameraBasedPosRatio(CameraLanePosOffsetRatio, CameraSteeringAngle)
    else:
        return 0

CameraLanePosOffsetRatio = 255
CameraSteeringAngle = 255

def SendBotDirection(direction, shutdown_flag):
    send_packet = 'A'+str(direction)+ 'S' + str(shutdown_flag) + 'Z'
    port.write(send_packet.encode())

# These values are set in CommonCommandHandler in the beginning

prev_bot_direction = CommonCommandHandler.BotDirectionFlag
prev_camera_lane_flag = CommonCommandHandler.CameraLaneFlag

VERY_LARGE_NUMBER = 1000
BackCameraObsDistance = VERY_LARGE_NUMBER    # Default with very large value. Substituted with back camera data received via Bluetooth

First_time_flag = 1

def CameraDataHandler():

    #time.sleep(1)      # Start with a delay so that CommonCommandHandler sets its values for prev variables

    global CameraLanePosOffsetRatio,CameraSteeringAngle, prev_bot_direction, CameraThreadDelay, prev_camera_lane_flag, BackCameraObsDistance
    global First_time_flag
 
    print("I am inside CameraDataHandler")

    init_loop_time = time.time()

    while 1:
      start_time = time.time()

      #print("Camera Handler Loop")
 
      '''     
      if(random() > 0.5):
        bot_direction = 'FRONT' #abs(round(random()*1000,0))
      else:
        bot_direction = 'BACK' #round(random()*1000,0)
      '''

      #print("Main Bot. Direction = ",CommonCommandHandler.BotDirectionFlag)
      #SendBotDirection(bot_direction, NetworkInterfaceHandler.Shutdown_bot_flag)

      # Send BOT direction
      if((CommonCommandHandler.BotDirectionFlag != prev_bot_direction) or First_time_flag):
          data_to_send = [IndicateBotDirection, 1, [CommonCommandHandler.BotDirectionFlag]]
          prev_bot_direction = CommonCommandHandler.BotDirectionFlag
          response_packet = BotUARTCommandHandler(data_to_send)
          print("Done with bot direction command", data_to_send)
          if(First_time_flag):
              First_time_flag = 0

      # Send Camera Lane Flag (CameraLaneFlag)
      if(CommonCommandHandler.CameraLaneFlag != prev_camera_lane_flag):
          data_to_send = [IndicateCameraLane, 1, [CommonCommandHandler.CameraLaneFlag]]
          print("data_to_send = ",data_to_send,"prev_camera_lane_flag = ",prev_camera_lane_flag,"this_camera_flag = ",CommonCommandHandler.CameraLaneFlag)
          prev_camera_lane_flag = CommonCommandHandler.CameraLaneFlag
          response_packet = BotUARTCommandHandler(data_to_send)
          print("Done with camera lane flag command")

      # Send shutdown command

      if(NetworkInterfaceHandler.Shutdown_bot_flag):
          data_to_send = [ShutdownCameraRasp, 0, []]
          response_packet = BotUARTCommandHandler(data_to_send)
          print("Done with shutdown command")
          print("#################################################")
          print("I AM SHUTTING DOWN BOT RASPBERRY IN FEW SECONDS")
          print("#################################################")
          time.sleep(2)
          #subprocess.Popen(['sudo','shutdown','-h','now'])

      # Read Camera ratio and steering angle
      data_to_send = [RequestCameraRatioAngle, 0, []]
      response_packet = BotUARTCommandHandler(data_to_send)
      #print("Done with camera parameter read command")

      #print("response_packet = ",response_packet)
      if((response_packet == None) or (response_packet == [])):
          print("Response Not Received. Setting default values")
          CameraLanePosOffsetRatio = 255
          CameraSteeringAngle = 255
          if(BT_Enable_Flag):
              BackCameraObsDistance = VERY_LARGE_NUMBER
      elif(len(response_packet) < 3):
          print("Packet received but camera paramter not received. Setting default values")
          CameraLanePosOffsetRatio = 255
          CameraSteeringAngle = 255
          if(BT_Enable_Flag):
              BackCameraObsDistance = VERY_LARGE_NUMBER
      else:
          #print("Else case: response_packet = ",response_packet)
          CameraLanePosOffsetRatio = float(response_packet[2][0])
          CameraSteeringAngle = float(response_packet[2][1])
          if(BT_Enable_Flag):
              BackCameraObsDistance = float(response_packet[2][2])
          if((CameraLanePosOffsetRatio == 0) or (CameraSteeringAngle == 0)):
              print("Invalid data received from Camera. Forcing to 255")
              CameraLanePosOffsetRatio = 255
              CameraSteeringAngle = 255

      if(BT_Enable_Flag):
          print("Py CR = ",CameraLanePosOffsetRatio,"; CSA = ",CameraSteeringAngle,"; BackObsCam = ",BackCameraObsDistance )
      else:
          print("Py CR = ",CameraLanePosOffsetRatio,"; CSA = ",CameraSteeringAngle)

      CameraLanePosOffsetRatio = int(CameraLanePosOffsetRatio)
      CameraSteeringAngle = int(CameraSteeringAngle)

      GetCameraBasedPosRatioCLibraryFunction(CameraLanePosOffsetRatio, CameraSteeringAngle)

      Cam_Loop_Delay = time.time() - start_time

      #print("Absolute Camera Loop delay = ",Cam_Loop_Delay)

      #time.sleep(CameraThreadDelay) 
      if((CameraThreadDelay-Cam_Loop_Delay) > 0):
        time.sleep(CameraThreadDelay - Cam_Loop_Delay) 

      #print("Compensated Camera Loop delay = ",time.time()-init_loop_time)
      init_loop_time = time.time()

