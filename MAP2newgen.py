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

#Function การคำนวณระยะห่างระหว่างกล้อง - ไก่ ในหน่วยเซนติเมตร
def ctoc():
    global angle_1,angle_2,angle_1_min,angle_1_max,angle_2_min,angle_2_max,height
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
        speed_pid = 25
        ep_chassis.drive_wheels(w1=-speed_pid, w2=speed_pid, w3=speed_pid, w4=-speed_pid)
        time.sleep(0.001)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

    elif -20 < xaxis_error < -10:
        speed_pid = 25
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
    
    if yaxis_error > 8:
        print('yaxis_error > 10')
        angle_1 += 2
        angle_1_min = min(40,angle_1)

        if angle_2 > 20:
            print('yaxis_error > 10 | angle_2 > 20')
            angle_2 -= 1
            angle_2_max = max(angle_2,-40)
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)
        
        if angle_1 <= -40:
            angle_2 -= 1
            angle_2_max = max(-40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_max).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 < -10:
            print('yaxis_error > 10 | angle_1 < 40')
            ep_servo.moveto(index=1, angle=angle_1_min).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 >= -10:
            print('yaxis_error > 10 | angle_1 >= 40')
            angle_2 -= 1
            angle_2_max = max(-40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_max).wait_for_completed()
            time.sleep(0.001)
    

    elif yaxis_error < -8:
        print(yaxis_error)
        print('yaxis_error < -10')
        angle_1 -= 1
        angle_1_max = max(-40,angle_1)

        if angle_1 == 40:
            print('yaxis_error < -10 | angle_1 == 40')
            angle_2 += 1
            angle_2_min = min(40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_min).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 > -40:
            print('yaxis_error < -10 | angle_1 > -40')
            ep_servo.moveto(index=1, angle=angle_1_max).wait_for_completed()
            time.sleep(0.001)   

        elif angle_1 <= -40:
            print('yaxis_error < -10 | angle_1 <= -40')
            angle_2 += 1
            angle_2_min = min(40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_min).wait_for_completed()
            time.sleep(0.001)


    else :
        print('return Done')
        return str("Done")
    

    if abs(angle_1) > 40 and abs(angle_2) > 40:
        print("she's such an angel")
        return str("she's such an angel")


#Function เข้าหาเป้าหมาย โดยที่ยังควบคุมพิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error อยู่ในช่วงที่ต้องการ
def Get_Closer():
    global yaxis_error,end,angle_1_max,angle_2_max,angle_2_min,angle_1_min,x_po,w,h,distance,front,m_w_k,m_w_k,chick_rh,chick_rw
    speed = 50
    stop = 0

    if yaxis_control() == str("Done"):
        ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        time.sleep(0.1)
        ep_chassis.drive_wheels(w1=stop, w2=stop, w3=stop, w4=stop)
        time.sleep(0.001)
        
        print(f'----------------------กล้องถึงไก่ : {ctoc()} cm -------------------------')
 

    elif yaxis_control() == str("she's such an angel") :
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
    cx_bbox = 640
    cy_bbox = 360
    iter = 1
    # angle_1 = 37
    # angle_2 = -12
    angle_1 = -10
    angle_2 = 0
    angle_1_max = 0
    angle_2_max = 0
    angle_1_min = 0
    angle_2_min = 0
    end = False

    l = 11.5
    ep_gripper.open(power=100)
    time.sleep(4)
    ep_gripper.pause()
    ep_servo.moveto(index=2, angle= 0).wait_for_completed()
    time.sleep(0.001)
    ep_servo.moveto(index=1, angle= 0).wait_for_completed()
    time.sleep(0.001)
    angle = ep_servo.get_angle(index = 1)

    time.sleep(3)

    while True:
        time.sleep(0.00001)

        print(f'distance : {compute_distance()}')

        xaxis_error = xaxis_error
        yaxis_error = yaxis_error

        print(f'cy_bbox : {cy_bbox} center : {height//2} y error : {yaxis_error}')

        kp = 0.25
        speed_pd = (kp*xaxis_error)

        if xaxis_control(speed_pd , xaxis_error) == str("Done"):
            # print('xaxis_control(speed_x , e_x) == str("Done")')

            if end is True and yaxis_control() == str("she's such an angel"):
                print('พอเถอะพอล')

            #เงื่อนไข เมื่อระยะห่างระหว่างกล้อง - ไก่ น้อยกว่า 25 cm (ระยะที่สามารถคีบไก่ได้) ให้ gripper หนีบไก่ขึ้นมา
                if ctoc() < 25:
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
            # เมื่อระยะห่างระหว่างกล้อง - ไก่ มากกว่าหรือเท่ากับ 25 cm ให้เดินหน้าจนกว่าระยะห่างระหว่างกล้อง - ไก่ จะน้อยกว่า 23 cm จากนั้นให้ gripper หนีบไก่ขึ้นมา
                else:
                    while ctoc() > 23:
                        if yaxis_control() == str("she's such an angel"):
                            ep_chassis.drive_wheels(w1=13, w2=13, w3=13, w4=13)
                            time.sleep(0.01)
                            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                            time.sleep(0.001)
                        else :
                            yaxis_control()
                    else: 
                        ep_gripper.close(power=100)
                        time.sleep(3)
                        ep_servo.moveto(index=2, angle= -12).wait_for_completed()
                        time.sleep(0.007)
                        ep_servo.moveto(index=1, angle= 37).wait_for_completed()
                        time.sleep(0.001)
                        # ep_gripper.pause()
                        print("Gorgeous")
                        ep_sensor.unsub_distance()
                        ep_robot.close()

                        break
             
            else:

                Get_Closer()
        
        else:
            xaxis_control(speed_pd , xaxis_error)
            

#Function สร้าง bounding box (ไก่ตาย) จากการเทรนผ่าน yolov8
def bounding_box_yolo():
    global yaxis_error,xaxis_error,x,y,w,h,cx_bbox,cy_bbox,width,height,front,distance,m_h_k,m_w_k,chick_rw,chick_rhs
    model = YOLO("D:\\PSU\\241-251\\Chic_Chicccc\\Yolov8Newgen\\runs\\detect\\train\\weights\\best.pt")
    while True :
        
        if keyboard.is_pressed('q'):
            break

        img = ep_camera.read_cv2_image(strategy = "newest")

        height = img.shape[0]
        width = img.shape[1]

        results = model.predict(img,verbose=False)
        result = results[0]
        box = result.boxes
        c = box.cls
        b = box.xyxy
        if len(c) != 0 :
            if c.sum() >= 2:
                if ctoc() <= 50:
                    if abs(round(((int(b[0][0])+int(b[0][2] - b[0][0]))+int(b[0][0]))/2) - 640) < abs(round(((int(b[1][0])+int(b[1][2] - b[1][0]))+int(b[1][0]))/2) - 640):
                        x = int(b[0][0])
                        y = int(b[0][1])
                        h = int(b[0][3] - b[0][1])
                        w = int(b[0][2] - b[0][0])
                        
                        cx_bbox = round(((x+w)+x)/2)
                        cy_bbox = round(((y+h)+y)/2)
                        cv.circle(img, (cx_bbox,cy_bbox), 3, (0, 10, 0), -1)
                        xaxis_error = cx_bbox - (width//2)
                        if height//2 != 360:
                            cv.circle(img, (640,360), 3, (0, 10, 0), -1)
                        else :
                            yaxis_error = height//2 - cy_bbox
                            cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
                        cv.rectangle(img , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)

                    else:
                        x = int(b[1][0])
                        y = int(b[1][1])
                        h = int(b[1][3] - b[1][1])
                        w = int(b[1][2] - b[1][0])
                        
                        cx_bbox = round(((x+w)+x)/2)
                        cy_bbox = round(((y+h)+y)/2)
                        cv.circle(img, (cx_bbox,cy_bbox), 3, (0, 10, 0), -1)
                        xaxis_error = cx_bbox - (width//2)
                        if height//2 != 360:
                            cv.circle(img, (640,360), 3, (0, 10, 0), -1)
                        else :
                            yaxis_error = height//2 - cy_bbox
                            cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
                        cv.rectangle(img , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)
                else:
                    w1 = int(b[0][2] - b[0][0])
                    w2 = int(b[1][2] - b[1][0])
                    if w1 < w2 :
                        x = int(b[0][0])
                        y = int(b[0][1])
                        h = int(b[0][3] - b[0][1])
                        w = int(b[0][2] - b[0][0])
                        
                        cx_bbox = round(((x+w)+x)/2)
                        cy_bbox = round(((y+h)+y)/2)
                        cv.circle(img, (cx_bbox,cy_bbox), 3, (0, 10, 0), -1)
                        xaxis_error = cx_bbox - (width//2)
                        if height//2 != 360:
                            cv.circle(img, (640,360), 3, (0, 10, 0), -1)
                        else :
                            yaxis_error = height//2 - cy_bbox
                            cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
                        cv.rectangle(img , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)

                    else:
                        x = int(b[1][0])
                        y = int(b[1][1])
                        h = int(b[1][3] - b[1][1])
                        w = int(b[1][2] - b[1][0])
                        
                        cx_bbox = round(((x+w)+x)/2)
                        cy_bbox = round(((y+h)+y)/2)
                        cv.circle(img, (cx_bbox,cy_bbox), 3, (0, 10, 0), -1)
                        xaxis_error = cx_bbox - (width//2)
                        if height//2 != 360:
                            cv.circle(img, (640,360), 3, (0, 10, 0), -1)
                        else :
                            yaxis_error = height//2 - cy_bbox
                            cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
                        cv.rectangle(img , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)
            
    
            else:
                for i in range(len(c)):
                    if c[i] == 1 :
                            x = int(b[i][0])
                            y = int(b[i][1])
                            w = int(b[i][2] - b[i][0])
                            h = int(b[i][3] - b[i][1])

                            cx_bbox = round(((x+w)+x)/2)
                            cy_bbox = round(((y+h)+y)/2)
                            cv.circle(img, (cx_bbox,cy_bbox), 3, (0, 10, 0), -1)
                            xaxis_error = cx_bbox - (width//2)
                            if height//2 != 360:
                                cv.circle(img, (640,360), 3, (0, 10, 0), -1)
                            else :
                                yaxis_error = height//2 - cy_bbox
                                cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
                            cv.rectangle(img , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)

            cv.imshow("Where R U chickkk", img)
            cv.waitKey(1)
        
        else :
            cx_bbox = 640
            cy_bbox = 360
            xaxis_error = 0
            yaxis_error = 0
      

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
    boundingbox_process = threading.Thread(target= bounding_box_yolo)
    boundingbox_process.start()
    robot_process = threading.Thread(target= Robot_Processing)
    robot_process.start()

