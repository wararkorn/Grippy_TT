from robomaster import robot , camera
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time 
from chick_bbox import bounding_box
import keyboard
import threading
from delta import theta,cos_degree,sin_degree,tan_degree
import csv
from ultralytics import YOLO
from concurrent.futures import ThreadPoolExecutor

def ctoc():
    global angle_1,angle_2,angle_1_min,angle_1_max,angle_2_min,angle_2_max,height,chick_camera
    l = 11.5
    if angle_1 >= 0:
        if angle_2 >= 0:

            theta1, theta2 = theta(angle_1_min,angle_2_min)
            height = (cos_degree(theta1) * l) + 20.75

        else:

            theta1, theta2 = theta(angle_1_min,angle_2_max)
            height = (cos_degree(theta1) * l) + 20.75
    else:
        if angle_2 >= 0:

            theta1, theta2 = theta(angle_1_max,angle_2_min)
            height = (cos_degree(theta1) * l) + 20.75

        else:

            theta1, theta2 = theta(angle_1_max,angle_2_max)
            height = (cos_degree(theta1) * l) + 20.75

    chick_camera = ((1/(cos_degree(theta2)))*height) - ((1/(cos_degree(theta2)))*4.25)
    return float(chick_camera)

def sub_data_handler(sub_info):
    global distance
    distance = sub_info

def sub_position_handler(position_info):
    global x_position
    x_position, y, z = position_info



#Function คำนวณระยะห่างระหว่าง พิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box
def compute_distance():
    global width,height,cx_bbox,cy_bbox
    return f'{round(np.sqrt((((cx_bbox - (width//2)))**2 )+ (((cy_bbox - (height//2)))**2 )))} pixel'


#Function การควบคุมหุ่นในแกน X หรือ การควบคุมพิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error ในแนวแกน X อยู่ในช่วงที่ต้องการ
#กำหนดให้ error (x) อยู่ในช่วง +- 10 pixel 
def xaxis_control(speed_pid,xaxis_error):
    print(xaxis_error)
    if 20 > xaxis_error > 10:
        speed_pid = 20
        ep_chassis.drive_wheels(w1=-speed_pid, w2=speed_pid, w3=speed_pid, w4=-speed_pid)
        time.sleep(0.001)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    elif -20 < xaxis_error < -10:
        speed_pid = 20
        # speed_pid = -speed_pid
        ep_chassis.drive_wheels(w1=speed_pid, w2=-speed_pid, w3=-speed_pid, w4=speed_pid)
        time.sleep(0.001)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    elif xaxis_error > 20:
        speed_pid += 12.5
        ep_chassis.drive_wheels(w1=-speed_pid, w2=speed_pid, w3=speed_pid, w4=-speed_pid)
        time.sleep(0.001)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    
    elif xaxis_error < -20:
        speed_pid -= 12.5
        speed_pid = -speed_pid
        ep_chassis.drive_wheels(w1=speed_pid, w2=-speed_pid, w3=-speed_pid, w4=speed_pid)
        time.sleep(0.001)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)


    elif -10 <= xaxis_error <= 10:
        return str("Done")

    # print(speed)

#Function การควบคุมหุ่นในแกน Y หรือ การควบคุมให้พิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error ในแนวแกน Y อยู่ในช่วงที่ต้องการ
#กำหนดให้ error (y) อยู่ในช่วง +- 10 pixel 
def yaxis_control():
    global angle_1,angle_2,yaxis_error,angle_1_max,angle_1_min,angle_2_max,angle_2_min
    
    if yaxis_error > 10:
        print('yaxis_error > 10')
        angle_1 += 1
        angle_1_min = min(40,angle_1)

        if angle_2 > 20:
            # print('yaxis_error > 10 | angle_2 > 20')
            angle_2 -= 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)
        
        elif angle_1 < 40:
            # print('yaxis_error > 10 | angle_1 < 40')
            ep_servo.moveto(index=1, angle=angle_1_min).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 >= 40:
            # print('yaxis_error > 10 | angle_1 >= 40')
            angle_2 -= 1
            angle_2_max = max(-40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_max).wait_for_completed()
            time.sleep(0.001)
    

    elif yaxis_error < -10:
        print(yaxis_error)
        print('yaxis_error < -10')
        angle_1 -= 1
        angle_1_max = max(-40,angle_1)

        if angle_1 == 40:
            # print('yaxis_error < -10 | angle_1 == 40')
            angle_2 += 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 > -40:
            # print('yaxis_error < -10 | angle_1 > -40')
            ep_servo.moveto(index=1, angle=angle_1_max).wait_for_completed()
            time.sleep(0.001)   

        elif angle_1 <= -40:
            # print('yaxis_error < -10 | angle_1 <= -40')
            angle_2 += 1
            angle_2_min = min(40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_min).wait_for_completed()
            time.sleep(0.001)


    else:
        print('return Done')
        return str("Done")
    

    if abs(angle_1) > 40 and abs(angle_2) > 40:
        # print("she's such an angel")
        return str("she's such an angel")


#Function เข้าหาเป้าหมาย โดยที่ยังควบคุมพิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error อยู่ในช่วงที่ต้องการ
def Get_Closer():
    global yaxis_error,end,angle_1_max,angle_2_max,angle_2_min,angle_1_min,x_po,w,h,distance,front,m_w_k,m_w_k,chick_rh,chick_rw
    speed = 30
    stop = 0

    if yaxis_control() == str("Done"):
        ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        time.sleep(0.01)
        ep_chassis.drive_wheels(w1=stop, w2=stop, w3=stop, w4=stop)
        time.sleep(0.001)
        ctoc()
        
        # x = tan_degree(theta2)*height

        print(f'----------------------กล้องถึงไก่ : {chick_camera} cm -------------------------')
        # print(f'----------------------ความสูงของไก่ : {chick_rh} cm -------------------------')
        data = [chick_camera]
        with open('canigrab.csv' ,'a', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow(data) 
   

    

    elif yaxis_control() == str("she's such an angel") :
        # ep_chassis.drive_wheels(w1=-speed, w2=-speed, w3=-speed, w4=-speed)
        # time.sleep(0.01)
        print(' elif yaxis_control() == str("shes such an angel") ')
        ep_chassis.drive_wheels(w1=stop, w2=stop, w3=stop, w4=stop)
        time.sleep(0.001)
        print('Already Move on ถอยออกมาก่อนเนาะ')
        end = True
    
    else:
        yaxis_control()


#Function การทำงานทั้งหมดในส่วนของหุ่น 
def Robot_Processing():
    global x,y,w,h,cx_bbox,cy_bbox,width,height,p_errorx,iter,angle_1,angle_2,yaxis_error,xaxis_error,end ,angle_1_max,angle_2_max,angle_1_min,angle_2_min,x_position,x_po,distance,front
    global m_w_k,m_h_k,chick_rh,chick_rw
    iter = 1
    angle_1 = 37
    angle_2 = -12
    angle_1_max = 0
    angle_2_max = 0
    angle_1_min = 0
    angle_2_min = 0
    m_w_k = 0.0018315
    m_h_k = 0.00164385
    end = False
    # xaxis_error = 0
    # yaxis_error = 0
    # chick_rw = 0
    # chick_rh = 0
    l = 11.5
    ep_gripper.open(power=100)
    time.sleep(3)
    ep_gripper.pause()
    ep_servo.moveto(index=2, angle= -12).wait_for_completed()
    time.sleep(0.001)
    ep_servo.moveto(index=1, angle= 37).wait_for_completed()
    time.sleep(0.001)


    time.sleep(3.5)

    while True:
        time.sleep(0.00000000001)

        yaxis_error = yaxis_error
        xaxis_error = xaxis_error
        # print(f'distance : {compute_distance()}')

        print(f'cy_bbox : {cy_bbox} center : {height//2} y error : {yaxis_error}')

        kp = 0.3

        # if iter > 1:

        speed_pd = (kp*xaxis_error)
        # print(speed_pd)

#------------------------------------------------------------------------------------
        if xaxis_control(speed_pd , xaxis_error) == str("Done"):
            # print('xaxis_control(speed_x , e_x) == str("Done")')

            if end is True and yaxis_control() == str("she's such an angel"):
                print('พอเถอะพอ')
            #     print('จบ')
            #     while yaxis_control() != str('Done'):
            #         if yaxis_control() == str("she's such an angel"):
            #             ctoc()
                if chick_camera <= 25:
                    ep_gripper.close(power=100)
                    time.sleep(3)
                    # ep_gripper.pause()
                    ep_servo.moveto(index=2, angle= -12).wait_for_completed()
                    time.sleep(0.001)
                    ep_servo.moveto(index=1, angle= 37).wait_for_completed()
                    time.sleep(0.001)
                    print("Gorgeous")
                    ep_sensor.unsub_distance()
                    ep_robot.close()

                    break
            #             else:
            #                 pass
                else:
                    ep_chassis.drive_wheels(w1=30, w2=30, w3=30, w4=30)
                    time.sleep(0.01)
                    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                    time.sleep(0.001)
                    ep_gripper.close(power=100)
                    time.sleep(3)
                    ep_servo.moveto(index=2, angle= -12).wait_for_completed()
                    time.sleep(0.001)
                    ep_servo.moveto(index=1, angle= 37).wait_for_completed()
                    time.sleep(0.001)
                    # ep_gripper.pause()
                    print("Gorgeous")
                    ep_sensor.unsub_distance()
                    ep_robot.close()

                    break
                # if yaxis_control() == str("she's such an angel"):
                        

            #     else:
                    

            #         ctoc()
            #         print(f'----------------------กล้องถึงไก่ : {chick_camera} cm -------------------------')
                    
            #         if chick_camera <= 25:
            #             ep_gripper.close(power=50)
            #             time.sleep(1)
            #             ep_gripper.pause()
            #             print("Gorgeous")
            #             ep_sensor.unsub_distance()
            #             ep_robot.close()

            #             break

            else:
                # print('ยังขยับได้อยู่')
                # yaxis_control()
                Get_Closer()
        
        else:
            # print('xaxis_control(speed_x , e_x) != str("Done")')
            xaxis_control(speed_pd , xaxis_error)
#------------------------------------------------------------------------------------------

        # iter += 1

            # p_time = time.time()

            # p_errorx = xaxis_error
        # time.sleep(0.001)

        # else:
        #     ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=-0)
        #     time.sleep(0.001)


# def bounding_box_yolo():
#     global yaxis_error,xaxis_error,x,y,w,h,cx_bbox,cy_bbox,width,height,front,distance,m_h_k,m_w_k,chick_rw,chick_rh
#     # chick_rh = 0
#     # chick_rw = 0
#     model = YOLO("D:\\PSU\\241-251\\Chic_Chicccc\\best_m100.pt")
#     start = time.time()
#     while True :
        
#         if keyboard.is_pressed('q'):
#             break

#         img = ep_camera.read_cv2_image(strategy = "newest")

#         height = img.shape[0]
#         width = img.shape[1]
#         results = model.predict(img,verbose=False)
#         result = results[0]
#         box = result.boxes
#         c = box.cls

#         for i in range(len(c)):
#             if c[i] == 1:
#                 b = box.xyxy[i]
#                 x = int(b[0])
#                 y = int(b[1])
#                 w = int(b[2] - b[0])
#                 h = int(b[3] - b[1])


#                 cx_bbox = round(((x+w)+x)/2)
#                 cy_bbox = round(((y+h)+y)/2)
#                 xaxis_error = cx_bbox - (width//2)
#                 cv.circle(img, (cx_bbox,cy_bbox), 3, (0, 10, 0), -1)
#                 if round(height/2) < 360 :
#                     pass
#                 else:
#                     yaxis_error = height//2 - cy_bbox
#                     cv.rectangle(img , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)
#                 # start = time.time()

#                 # cv.rectangle(img , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)
                    
        

#         cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
#         cv.imshow("Where R U chickkk", img)
#         cv.waitKey(1)






    

if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_servo = ep_robot.servo
    ep_camera = ep_robot.camera
    ep_vision = ep_robot.vision
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_gripper = ep_robot.gripper
    ep_camera.start_video_stream(display = False)
    # ep_chassis.sub_position(freq=10,callback=sub_position_handler)
    # ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    boundingbox_process = threading.Thread(target= bounding_box_yolo)
    boundingbox_process.start()
    # robot_process = threading.Thread(target= Robot_Processing)
    # robot_process.start()


    # boundingbox_process = threading.Thread(target= bounding_box_yolo)
    # boundingbox_process.start()


