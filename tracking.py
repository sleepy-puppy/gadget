import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
from ultralytics.utils.checks import check_yaml
from ultralytics.utils import yaml_load

# parameters
WITDH = 1280
HEIGHT = 720
model = YOLO('/home/songah/Downloads/person-knife/person_knife_best.pt')
CLASSES = yaml_load(check_yaml('/home/songah/Downloads/person-knife/person_knife_args.yaml'))['names'] #"/home/songah/Downloads/person-knife/args.yaml" this one doesn't work i think
colors = np.random.uniform(0, 255, size=(20, 3))



# Create a config or configure the pipeline to stream for Realsense-L515
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
if device_product_line == 'D400':
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)


# Start streaming
profile = pipeline.start(config)

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)





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
MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_VELOCITY          = 104
LEN_GOAL_VELOCITY           = 4         # Data Byte Length
ADDR_PRESENT_VELOCITY       = 128
LEN_PRESENT_VELOCITY        = 4         # Data Byte Length
DXL_MINIMUM_VELOCITY_VALUE  = 0         # Refer to the Minimum velocity Limit of product eManual
DXL_MAXIMUM_VELOCITY_VALUE  = 50        # Refer to the Maximum velocity Limit of product eManual
ADDR_PRO_ACC           = 108
DXL_PRO_ACC            = 200
DXL_DEFAULT = 0
BAUDRATE                    = 57600

PROTOCOL_VERSION            = 2.0
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
DEVICENAME                  = '/dev/ttyACM0'

ADDR_OPERATING_MODE = 1
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 1
dxl_goal_velocity = [DXL_MINIMUM_VELOCITY_VALUE, DXL_MAXIMUM_VELOCITY_VALUE]         # Goal velocity

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_VELOCITY, LEN_GOAL_VELOCITY)
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
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, 11, ADDR_OPERATING_MODE)  # 11번 레지스터는 작동 모드를 설정하는 레지스터입니다.
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, 11, ADDR_OPERATING_MODE)  # 11번 레지스터는 작동 모드를 설정하는 레지스터입니다.

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
    param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_velocity[index])), #index=0 -> select 'DXL_MINIMUM_VALUE', index=1 -> select 'DXL_MAXIMUN_VALUE'
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


# Streaming loop
try:
    while True:
        # Get frameset of color or depth
        frames = pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())


        depth_image = cv2.resize(depth_image, (WITDH, HEIGHT))
        color_image = cv2.resize(color_image, (WITDH, HEIGHT))

        # # Render images:
        # #   depth align to color on left
        # #   depth on right

        # 뎁스 이미지 스케일링
        #depth_image_scaled = cv2.convertScaleAbs(depth_image, alpha=0.025)
        # color image
        results= model(color_image, stream=True)


        class_ids = []
        confidences = []
        bboxes = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                confidence = box.conf
                if confidence > 0.5:
                    xyxy = box.xyxy.tolist()[0]
                    bboxes.append(xyxy)
                    confidences.append(float(confidence))
                    class_ids.append(box.cls.tolist())

        result_boxes = cv2.dnn.NMSBoxes(bboxes, confidences, 0.25, 0.45, 0.5)

        # print(result_boxes)
        font = cv2.FONT_HERSHEY_PLAIN
        depth_list = list()
        person_id_list = list()



        for i in range(len(bboxes)):

            label = str(CLASSES[int(class_ids[i][0])])
            if label == 'person':
                if i in result_boxes:
                    box = list(map(int, bboxes[i]))
                    x, y, x2, y2 = box
                    color = colors[i]
                    color = (int(color[0]), int(color[1]), int(color[2]))

                    cv2.rectangle(color_image, (x, y), (x2, y2), color, 2)
                    cv2.rectangle(color_image, (x, y), (x2, y2), color, 2) ##changed depth_image_scaled to color_image
                    cv2.putText(color_image, label, (x, y + 30), font, 3, color, 3)

                    # Calculate the average depth within the bounding box
                    depth_roi = color_image[y:y2, x:x2] ##changed depth_image_scaled to color_image
                    avg_depth = np.mean(depth_roi)
                    x1,y1 = (x+x2)//2, (y+y2)//2
                    distance = aligned_depth_frame.get_distance(x1, y1)
                    print("person", i+1, ":", x1,",",y1)


                    

                    if x1<620:
                        dx1=620-x1
                        dx2=0
                    elif 660<x1:
                        dx2=x1-660
                        dx1=0
                    if y1<360:
                        dy1=360-y1
                        dy2=0
                    elif 380<y1:
                        dy2=y1-380
                        dy1=0
                    
                    if (dx1<dy1 and dx2<dy1) or (dx1<dy2 and dx2<dy2):
                        if 370<y1:
                            oper(DXL1_ID, 1 ,1) #y시계방향
                            oper(DXL1_ID, 0, 1)
                        elif y1<350:
                            oper(DXL1_ID, 1 ,0) #y반시계방향
                            oper(DXL1_ID, 0, 0)
                            
                    elif (dy1<dx1 and dy2<dx1) or (dy1<dx2 and dy2<dx2):
                        if 650<x1:
                            oper(DXL2_ID, 1 ,1) #x시계방향
                            oper(DXL2_ID, 0, 1)
                        elif x1<630:
                            oper(DXL2_ID, 1 ,0) #x반시계방향
                            oper(DXL2_ID, 0, 0)

                    print(x1,y1)

                    # if x1<=630:
                    #     oper(DXL1_ID, 1 ,0) #x반시계방향
                    #     oper(DXL1_ID, 0, 0)

                    # elif 650<=x1:
                    #     oper(DXL1_ID, 1 ,1) #x시계방향
                    #     oper(DXL1_ID, 0, 1)

                    # if y1<=350:
                    #     oper(DXL2_ID, 1 ,1) #y시계방향
                    #     oper(DXL2_ID, 0, 1)

                    # elif 370<=y1:
                    #     oper(DXL2_ID, 1 ,0) #y반시계방향
                    #     oper(DXL2_ID, 0, 0)



                    # Display average depth value
                    cv2.putText(color_image, f"Depth: {distance:.2f} m", (x, y - 10), font, 1, color, 2) ##changed depth_image_scaled to color_image

            elif label == 'knife':
                if i in result_boxes:
                    box = list(map(int, bboxes[i]))
                    x, y, x2, y2 = box
                    color = colors[i]
                    color = (int(color[0]), int(color[1]), int(color[2]))

                    cv2.rectangle(color_image, (x, y), (x2, y2), color, 2)
                    cv2.rectangle(color_image, (x, y), (x2, y2), color, 2) ##changed depth_image_scaled to color_image
                    cv2.putText(color_image, label, (x, y + 30), font, 3, color, 3)

                    # Calculate the average depth within the bounding box
                    depth_roi = color_image[y:y2, x:x2] ##changed depth_image_scaled to color_image
                    avg_depth = np.mean(depth_roi)
                    x1,y1 = (x+x2)/2, (y+y2)/2
                    distance = aligned_depth_frame.get_distance(int(x1), int(y1))
                    print(x1,y1)

                    # Display average depth value
                    cv2.putText(color_image, f"Depth: {distance:.2f} m", (x, y - 10), font, 1, color, 2) ##changed depth_image_scaled to color_image

        cv2.imshow("Bgr frame", color_image)
        #cv2.imshow("Depth frame", depth_image_scaled)

        key = cv2.waitKey(1)
        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break
finally:
    pipeline.stop()



# Clear syncread parameter storage
groupSyncRead.clearParam()

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