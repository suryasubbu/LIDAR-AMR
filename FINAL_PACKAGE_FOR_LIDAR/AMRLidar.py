#!/usr/bin/env python3
##########This is AMR Lidar is a package in which it gets scan data from RPlidar Library and process to give all necessary data for AMR in HACHIDORI ROBOTICS #####


from rplidar import RPLidar
import time
import math
import numpy as np
import threading
from threading import Thread
from rplidar import RPLidarException


global BOTWIDTH,BOTLENGHT,BOTHEIGHT,BLINDSPOTVALUE,MINIMUMDISTANCEOFFSET,MAXIMUMDISTANCEOFFSET
'''#########Set your bot dimesions here ############'''
BOTWIDTH =700
BOTLENGHT = 800
BOTHEIGHT = 300
'''##########sensors cannot find obstacles less than 15 cm and greater than 12 m############'''
BLINDSPOTVALUE = 100000
'''##########Set the minimum and maximum distance offset here ############ The LIDAR will give exact distance from 150 to 7000mm and tested and tuned accordingly ##########'''
MINIMUMDISTANCEOFFSET = 150
MAXIMUMDISTANCEOFFSET = 12000
'''##########This is for the quadrant angle limits ############ depending The position of Lidar in AMR u have to change the limits##'''
'''##########This is for the quadrant - 1 angle limits ############ which should be between 0 to 90 only dont exten beyond 90#############'''
Q1_START_ANGLE =0
Q1_END_ANGLE =90
'''##########This is for the quadrant - 2 angle limits ############ which should be between 90 to 180 only dont exten beyond 90#############'''
Q2_START_ANGLE =90
Q2_END_ANGLE =180
'''##########This is for the quadrant - 3 angle limits ############ which should be between 180 to 270 only dont exten beyond 90#############'''
Q3_START_ANGLE =180
Q3_END_ANGLE =270
'''##########This is for the quadrant - 4 angle limits ############ which should be between 270 to 360 only dont exten beyond 90#############'''
Q4_START_ANGLE =270
Q4_END_ANGLE =360
########## Attention ! Assign the ports of Lidars here ############ This code is programmed in such a way that multiple lidars can be identified with the PORTS#############
PORT1 = "/dev/ttyUSB0"
PORT2 = "/dev/ttyUSB1"
'''######Commenting the com ports to use the code in RaspberryPi######For computer usage comment it viceversa#########'''
#PORT1 = "COM4"
#PORT2 = "COM11"
'''######LIDAR SENSORS WILL START THE SCAN AFTER 0.83 Seconds (before entering into the master Library while loop), So the value is initially setted as BLINDSPOTVALUE######
######This program is for running two sensors placed in diagnolly opposite side in AMR ###
### Front and Right side datas getting from one sensor, Back and left side datas getting from other sensor######'''
'''####SENSOR1 Values####'''
FRONT_OBSTACLE = BLINDSPOTVALUE
RIGHT_OBSTACLE = BLINDSPOTVALUE
FRONT_MAX = BLINDSPOTVALUE
RIGHT_MAX = BLINDSPOTVALUE
RIGHT_AVG =BLINDSPOTVALUE
RIGHT_WALL_THETA = BLINDSPOTVALUE
'''####SENSOR2 Values####'''
BACK_OBSTACLE = BLINDSPOTVALUE
LEFT_OBSTACLE = BLINDSPOTVALUE
BACK_MAX = BLINDSPOTVALUE
LEFT_MAX = BLINDSPOTVALUE
LEFT_AVG = BLINDSPOTVALUE
LEFT_WALL_THETA = BLINDSPOTVALUE
####give 1 to enable UART####
_HOST_COMMUNICATION = input("userinput for host communication ", )

####This will start the UART SLave Thread initially####
if _HOST_COMMUNICATION == "1" :
    import serial
    #import wiringpi
    import LidarUartSlave
    LidarUartSlaveThread = threading.Thread( target =LidarUartSlave.LidarUartSlaveMethod)
    LidarUartSlaveThread.start()
    ####Check Print statement####
    #print("UART SLAVE THREAD STARTED")

def PutLidarData(FRONT_OBSTACLE,BACK_OBSTACLE,FRONT_MAX,BACK_MAX,RIGHT_OBSTACLE,LEFT_OBSTACLE,RIGHT_MAX,LEFT_MAX,RIGHT_AVG,LEFT_AVG,RIGHT_WALL_THETA,LEFT_WALL_THETA) :
    if _HOST_COMMUNICATION == "1":
            LidarUartSlave.FRONT_OBSTACLE = FRONT_OBSTACLE
            LidarUartSlave.BACK_OBSTACLE = BACK_OBSTACLE
            LidarUartSlave.RIGHT_OBSTACLE = RIGHT_OBSTACLE
            LidarUartSlave.LEFT_OBSTACLE = LEFT_OBSTACLE
            LidarUartSlave.FRONT_MAX = FRONT_MAX
            LidarUartSlave.BACK_MAX = BACK_MAX
            LidarUartSlave.RIGHT_MAX = RIGHT_MAX
            LidarUartSlave.LEFT_MAX = LEFT_MAX
            LidarUartSlave.RIGHT_AVG = RIGHT_AVG
            LidarUartSlave.LEFT_AVG = LEFT_AVG
            LidarUartSlave.RIGHT_WALL = RIGHT_WALL_THETA
            LidarUartSlave.LEFT_WALL = LEFT_WALL_THETA
            
class AMRLIDAR(object):
        def __init__(self,PORT_NAME,Q1,Q2,Q3,Q4):
                self.PORT_NAME = PORT_NAME
                self.lock=threading.Lock()
                self.Q1 = Q1
                self.Q2 = Q2
                self.Q3 = Q3
                self.Q4 = Q4
                
        def process(self):
                self.lock.acquire()
                u = time.time()
                ####Calling the RPlidar Library####
                lidar = RPLidar(self.PORT_NAME)
                ####Check Print statement####
                print(self.PORT_NAME,'started')
                ####iter_scans is the fuction which gives the scan data for one rotation each time####
                for scan in lidar.iter_scans():
                    FINAL_ARRAY =[]
                    WALL_ANGLE_DATA_ARRAY = []
                    FRONT_OBSTACLE_DATA_ARRAY = []
                    RIGHT_OBSTACLE_DATA_ARRAY = []
                    LEFT_OBSTACLE_DATA_ARRAY =[]
                    SLOPE_POINTS_DATA_ARRAY = []
                    if len(scan) > 50:
                        ####Check Print statement####
                        ####these print statement will print the samples for one rotation and total no of sample####
                        #print (self.PORT_NAME,scan)
                        #print("no of values",len(scan))
                        for items in scan:
                            Distance = items[0]
                            Angle = items[1]
                            if Angle <= 90:
                                    x1 = Distance * math.cos(math.radians(90 - Angle))
                                    y1 = Distance * math.sin(math.radians(90 - Angle))
                                    '''# x1 and y1 denotes the coordinates in the quadrant 1'''
                            elif 90 < Angle <= 180:
                                    x2 = Distance * math.cos(math.radians(Angle - 90))
                                    y2 = Distance * math.sin(math.radians(Angle - 90))
                                    '''# x2 and y2 denotes the coordinates in the quadrant 2'''
                            elif Angle > 270 :
                                    x3 = Distance * math.cos(math.radians(Angle - 270))
                                    y3 = Distance * math.sin(math.radians(Angle - 270))
                                    '''# x3 and y3 denotes the coordinates in the quadrant 4'''
                            elif 180 < Angle <= 270:
                                    x4 = Distance * math.cos(math.radians(Angle - 180))
                                    y4 = Distance * math.sin(math.radians(Angle - 180))
                                    '''# x4 and y4 denotes the coordinates in the quadrant 3'''
                            

                            if Angle <= 90:
                                WALL_ANGLE_DATA_ARRAY.append((x1, y1, Angle))


                            if 90 < Angle <= 180:
                                WALL_ANGLE_DATA_ARRAY.append((x2, y2, Angle))

                            if self.Q1 == True :
                                if Q1_START_ANGLE < Angle <= Q1_END_ANGLE:
                                    ####RESTRICTING DATAS TO AMR LANE (incase of front and back side of sensors) AND ITS DIMENSION (incase of Rigth and Left side of sensors)######
                                    if MAXIMUMDISTANCEOFFSET > y1 > MINIMUMDISTANCEOFFSET:
                                        
                                        if x1 < BOTWIDTH:
                                            
                                            FRONT_OBSTACLE_DATA_ARRAY.append((y1))

                            if self.Q4 == True :

                                if Q4_END_ANGLE >= Angle > Q4_START_ANGLE:
                                    
                                    ####RESTRICTING DATAS TO AMR LANE (incase of front and back side of sensors) AND ITS DIMENSION (incase of Rigth and Left side of sensors)######
                                    if  MAXIMUMDISTANCEOFFSET > y3 > MINIMUMDISTANCEOFFSET:

                                        if x3 < BOTWIDTH:

                                            FRONT_OBSTACLE_DATA_ARRAY.append((y3))

                            if self.Q2 == True :

                                if Q2_START_ANGLE < Angle <= Q2_END_ANGLE:
                                    ####RESTRICTING DATAS TO AMR LANE (incase of front and back side of sensors) AND ITS DIMENSION (incase of Rigth and Left side of sensors)######
                                    if MAXIMUMDISTANCEOFFSET > x2 > MINIMUMDISTANCEOFFSET:

                                        if y2 < BOTLENGHT:

                                            RIGHT_OBSTACLE_DATA_ARRAY.append((x2))

                            if self.Q3 == True :

                                if Q3_START_ANGLE < Angle <= Q3_END_ANGLE:
                                    ####RESTRICTING DATAS TO AMR LANE (incase of front and back side of sensors) AND ITS DIMENSION (incase of Rigth and Left side of sensors)######
                                    if MAXIMUMDISTANCEOFFSET > x4 > MINIMUMDISTANCEOFFSET:

                                        if y4 < BOTLENGHT:

                                            LEFT_OBSTACLE_DATA_ARRAY.append((x4))


                        '''####FINDING THE MINIMUM,MAXIMUM and AVERAGE FOR ALL SIDES OF SENSOR WITH HELP OF DATA APPENDED ARRAY######'''
                        if len(FRONT_OBSTACLE_DATA_ARRAY) > 2:
                            frontmin = min(FRONT_OBSTACLE_DATA_ARRAY)
                            frontmax = max(FRONT_OBSTACLE_DATA_ARRAY)

                        else:
                            frontmin =BLINDSPOTVALUE
                            frontmax = BLINDSPOTVALUE
                            
                        if len(RIGHT_OBSTACLE_DATA_ARRAY) > 2:
                            Rsidemin = min(RIGHT_OBSTACLE_DATA_ARRAY)
                            Rsidemax = max(RIGHT_OBSTACLE_DATA_ARRAY)
                            Rsidaverage = np.average(RIGHT_OBSTACLE_DATA_ARRAY)

                        else:
                            Rsidemin = BLINDSPOTVALUE
                            Rsidemax = BLINDSPOTVALUE
                            Rsidaverage = BLINDSPOTVALUE
                            

                        if len(LEFT_OBSTACLE_DATA_ARRAY) > 2:
                            Lsidemin = min(LEFT_OBSTACLE_DATA_ARRAY)
                            Lsidemax = max(LEFT_OBSTACLE_DATA_ARRAY)
                            Lsidaverage = np.average(LEFT_OBSTACLE_DATA_ARRAY)

                        else:
                            Lsidemin = BLINDSPOTVALUE
                            Lsidemax = BLINDSPOTVALUE
                            Lsidaverage = BLINDSPOTVALUE
                            

                        if len(WALL_ANGLE_DATA_ARRAY) > 10:
                            ####THIS two Loops IS FOR WALL DETECTION BY FINDING SLOPE AND ANGLE respectively BETWEEN SVERAL POINTS TOOK IN MULTIPLE RANGE OF ANGLES######
                            for idem in WALL_ANGLE_DATA_ARRAY:
                                x = idem[0]
                                y = idem[1]
                                A = idem[2]

                                if 50 <  A < 70:
                                    s01 = x
                                    t01 = y
                                    A01 = A

                                if 80 <  A < 90:
                                    s1 = x
                                    t1 = y
                                    A1 = A

                                if 90 < A < 100:
                                    s2 = x
                                    t2 = y
                                    A2 = A

                                if 100 < A < 120:
                                    s3 = x
                                    t3 = y
                                    A3 = A

                                if 120 < A < 150:
                                    s4 = x
                                    t4 = y
                                    A4 = A

                            SLOPE_POINTS_DATA_ARRAY.append(((s1, t1), (s2, t2), (s3, t3),(s4,t4),(s01,t01), A1, A2, A3,A4,A01))
                            
                            for points in SLOPE_POINTS_DATA_ARRAY:
                                
                                a1 = points[0][0]
                                b1 = points[0][1]
                                a2 = points[1][0]
                                b2 = points[1][1]
                                a3 = points[2][0]
                                b3 = points[2][1]
                                a4 = points [3][0]
                                b4 = points [3][1]
                                a5 = points [4][0]
                                b5 = points [4][1]

                                if a2 != a3:
                                    slope1 = abs((b2 - b3) / (a2 - a3))
                                    theta1 = math.atan(slope1)
                                    thetaB = round(90 - math.degrees(theta1))
                                    thetaB1 = round(thetaB, 0)
                                if a1 != a2:
                                    slope = abs((b2 - b1) / (a2 - a1))
                                    theta = math.atan(slope)
                                    thetaA = round(90 - math.degrees(theta))
                                    thetaA1 = round(thetaA, 0)
                                if a3 != a1:
                                    slope2 = abs((b3 - b1) / (a3 - a1))
                                    theta2 = math.atan(slope2)
                                    thetaC = round(90 - math.degrees(theta2))

                                if a4 != a1:
                                    slope3 = abs((b4 - b1) / (a4 - a1))
                                    theta3 = math.atan(slope3)
                                    thetaD = round(90 - math.degrees(theta3))

                                if a5 != a1:
                                    slope4 = abs((b5 - b1) / (a5 - a1))
                                    theta4 = math.atan(slope4)
                                    thetaE = round(90 - math.degrees(theta4))
                                    
                                h = abs(thetaA1 - thetaB1)
                                k = abs(thetaB1 - thetaC)
                                g = abs(thetaD - thetaE)
                                w = abs(thetaE - thetaB)
                                j = abs(thetaE - thetaA)
                                v = time.time()
                                u = time.time()

                                if h <= 3 and k <= 3 and w <=3:
                                    if thetaB > 3:
                                        THETA = thetaB
                                    else:
                                        THETA = 0
                                elif g <= 3 and w <= 3 and j <= 3 :
                                    if thetaE > 3:
                                        THETA = thetaE
                                    else:
                                        THETA = 0

                                else :
                                        THETA = 720
                                
                        ####THIS APPEND CONSISTS OF ALL NECESSARY DATA FROM A SENSOR U CAN GET AND SIDES MENTIONED HERE ARE WITH RESPECTIVE TO SENSOR NOT TO BOT####
                        FINAL_ARRAY.append((frontmin, frontmax, Rsidemin, Rsidemax, Rsidaverage, Lsidemin, Lsidemax, Lsidaverage,self.PORT_NAME,THETA))
                        ####Check Print statement####
                        #print(FINAL_ARRAY)
                        for items in FINAL_ARRAY :
                            frontmin = round (items[0],2)
                            frontmax = round (items[1],2)
                            Rsidemin = round (items[2],2)
                            Rsidemax = round(items[3],2)
                            Rsideavg = round(items[4],2)
                            Lsidemin = round (items[5],2)
                            Lsidemax = round(items[6],2)
                            Lsideavg = round(items[7],2)
                            sensorport = items [8]
                            THETA = items
                            ####CLASSIFYING THE SENSORS WITH HELP OF PORTS AND U CAN NAME THE VALUES ACCORDINGLY TO THE POSITION OF TWO SENSORS ####
                            ####FOR EXAMPLE IF THE TWO SENSORS PLACED IN FRONT SIDE THEN U HAVE TO CODE FOR COMPARING DATA #### U CAN USE THESE GLOBAL VARIABLES FOR COMPARISION OPERATION####
                            global FRONT_OBSTACLE,BACK_OBSTACLE,FRONT_MAX,BACK_MAX,RIGHT_OBSTACLE,LEFT_OBSTACLE,RIGHT_MAX,LEFT_MAX,RIGHT_AVG,LEFT_AVG,RIGHT_WALL_THETA,LEFT_WALL_THETA
                            if sensorport == PORT1 :
                                    FRONT_OBSTACLE = frontmin
                                    RIGHT_OBSTACLE = Rsidemin
                                    FRONT_MAX = frontmax
                                    RIGHT_MAX = Rsidemax
                                    RIGHT_AVG = Rsideavg
                                    RIGHT_WALL_THETA = THETA
                            elif sensorport == PORT2 :
                                    BACK_OBSTACLE = frontmin
                                    LEFT_OBSTACLE = Rsidemin
                                    BACK_MAX = frontmax
                                    LEFT_MAX = Rsidemax
                                    LEFT_AVG = Rsideavg
                                    LEFT_WALL_THETA = THETA
                            else:
                                print("one Lidar Data alone printing",items)

                        timestamp = time.time() - u
                        #print("processsing data time is printed here",timestamp)
                        u = time.time()
                self.lock.release()

####THIS FUNCTION WILL START THE LIDAR THREADS AND PUT THE LIDAR DATA FOR UART COMMUNICATION####
def AMRLIDARCALC():
    STARTTIME = time.time()
    ####Assigning the objects for two LIDARs and starting two seperat threads####
    Lidar1 = AMRLIDAR(PORT1,True,True,True,True)
    threading.Thread(target = Lidar1.process).start()
    if PORT2 :
        Lidar2 = AMRLIDAR(PORT2,True,True,True,True) 
        threading.Thread(target = Lidar2.process).start()
        while True:
                time.sleep(.09)
                ####Puting the global variable from two lidar Threads####
                ####Check Print statement####
                #print(FRONT_OBSTACLE,BACK_OBSTACLE,FRONT_MAX,BACK_MAX,RIGHT_OBSTACLE,LEFT_OBSTACLE,RIGHT_MAX,LEFT_MAX,RIGHT_AVG,LEFT_AVG,RIGHT_WALL_THETA,LEFT_WALL_THETA)
                ####Calling PutLidarData Function with necessary arguments####
                PutLidarData(FRONT_OBSTACLE,BACK_OBSTACLE,FRONT_MAX,BACK_MAX,RIGHT_OBSTACLE,LEFT_OBSTACLE,RIGHT_MAX,LEFT_MAX,RIGHT_AVG,LEFT_AVG,RIGHT_WALL_THETA,LEFT_WALL_THETA)
                ####Check Print statement####
                #print("DATA PUTTING DONE",time.time()-STARTTIME)
                STARTTIME=time.time()

if __name__ == '__main__':
    ####Calling AMRLIDARCALC Function####
    AMRLIDARCALC()

