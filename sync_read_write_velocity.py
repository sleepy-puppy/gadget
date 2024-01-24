#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


#*******************************************************************************
#***********************     SyncRead and SyncWrite Example      ***********************
#  Required Environment to run this example :
#    - Protocol 2.0 supported DYNAMIXEL(X, P, PRO/PRO(A), MX 2.0 series)
#    - DYNAMIXEL Starter Set (U2D2, U2D2 PHB, 12V SMPS)
#  How to use the example :
#    - Select the DYNAMIXEL in use at the MY_DXL in the example code. 
#    - Build and Run from proper architecture subdirectory.
#    - For ARM based SBCs such as Raspberry Pi, use linux_sbc subdirectory to build and run.
#    - https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/
#  Author: Ryu Woon Jung (Leon)
#  Maintainer : Zerom, Will Son
# *******************************************************************************

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_VELOCITY          = 104
    LEN_GOAL_VELOCITY           = 4         # Data Byte Length
    ADDR_PRESENT_VELOCITY       = 128
    LEN_PRESENT_VELOCITY        = 4         # Data Byte Length
    DXL_MINIMUM_VELOCITY_VALUE  = 0         # Refer to the Minimum velocity Limit of product eManual
    DXL_MAXIMUM_VELOCITY_VALUE  = 100       # Refer to the Maximum velocity Limit of product eManual
    DXL_DEFAULT = 0
    BAUDRATE                    = 57600
# elif MY_DXL == 'PRO_SERIES':
#     ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
#     ADDR_GOAL_VELOCITY          = 596
#     LEN_GOAL_VELOCITY           = 4
#     ADDR_PRESENT_VELOCITY       = 611
#     LEN_PRESENT_VELOCITY        = 4
#     DXL_MINIMUM_VELOCITY_VALUE  = -150000   # Refer to the Minimum velocity Limit of product eManual
#     DXL_MAXIMUM_VELOCITY_VALUE  = 150000    # Refer to the Maximum velocity Limit of product eManual
#     BAUDRATE                    = 57600
# elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
#     ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
#     ADDR_GOAL_VELOCITY          = 564
#     LEN_GOAL_VELOCITY           = 4          # Data Byte Length
#     ADDR_PRESENT_VELOCITY       = 580
#     LEN_PRESENT_VELOCITY        = 4          # Data Byte Length
#     DXL_MINIMUM_VELOCITY_VALUE  = -150000    # Refer to the Minimum velocity Limit of product eManual
#     DXL_MAXIMUM_VELOCITY_VALUE  = 150000     # Refer to the Maximum velocity Limit of product eManual
#     BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyACM0'

OPERATING_MODE = 1
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
ADDR_PRO_ACC           = 108
DXL_PRO_ACC            = 225

index = 1
# dxl_goal_velocity = [DXL_MINIMUM_VELOCITY_VALUE, DXL_MAXIMUM_VELOCITY_VALUE]         # Goal velocity

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)

# Initialize GroupSyncRead instace for Present velocity
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# 모터의 작동 모드 설정
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, 11, OPERATING_MODE)  # 11번 레지스터는 작동 모드를 설정하는 레지스터입니다.
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, 11, OPERATING_MODE)  


packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_ACC, DXL_PRO_ACC) 
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_ACC, DXL_PRO_ACC)


# Add parameter storage for Dynamixel#1 present velocity value
dxl_addparam_result = groupSyncRead.addParam(DXL1_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL1_ID)
    quit()

# Add parameter storage for Dynamixel#2 present velocity value
dxl_addparam_result = groupSyncRead.addParam(DXL2_ID)
if dxl_addparam_result != True:
    print("[ID:%03d] groupSyncRead addparam failed" % DXL2_ID)
    quit()
    

def oper(id,index,rot):
    # Disable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Disable Dynamixel#2 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    packetHandler.write1ByteTxRx(portHandler, id, 10, rot)
    dxl_goal_velocity = [DXL_MINIMUM_VELOCITY_VALUE, DXL_MAXIMUM_VELOCITY_VALUE] 

    # Enable Dynamixel#1 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % id)


    # Allocate goal velocity value into byte array
    param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[index])), 
                           DXL_HIBYTE(DXL_LOWORD(dxl_goal_velocity[index])), 
                           DXL_LOBYTE(DXL_HIWORD(dxl_goal_velocity[index])), 
                           DXL_HIBYTE(DXL_HIWORD(dxl_goal_velocity[index]))]

    # Add Dynamixel#1 goal velocity value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(id, param_goal_velocity)
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncWrite addparam failed" % id)
        quit()

    # Syncwrite goal velocity
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Syncread present velocity
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = groupSyncRead.isAvailable(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)
        if dxl_getdata_result != True:
            print("[ID:%03d] groupSyncRead getdata failed" % id)
            quit()


        # Get Dynamixel#1 present velocity value
        dxl_present_velocity = groupSyncRead.getData(id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY)


        print("[ID:%03d] GoalVel:%03d  PresVel:%03d" % (id, dxl_goal_velocity[index], dxl_present_velocity))
        if not ((abs(dxl_goal_velocity[index] - dxl_present_velocity) > DXL_MOVING_STATUS_THRESHOLD)) :
            break




while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    oper(DXL1_ID, 1 ,1)
    oper(DXL2_ID, 1, 1)

    # Change goal velocity
    if index == 0:
        index = 1
    else:
        index = 0

    oper(DXL1_ID, 0, 1)
    oper(DXL2_ID, 0, 1)

    if index == 0:
        index = 1
    else:
        index = 0


    #ccw
    oper(DXL1_ID, 1, 0)
    oper(DXL2_ID, 1, 0)

    if index == 0:
        index = 1
    else:
        index = 0

    oper(DXL1_ID, 0, 0)
    oper(DXL2_ID, 0, 0)


# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
