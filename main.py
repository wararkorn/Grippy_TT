from robomaster import robot , camera
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time 
from chick_bbox import bounding_box,show_bounding_box
import keyboard
import threading

#Function คำนวณระยะห่างระหว่าง พิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box
def compute_distance():
    global width,height,cx_bbox,cy_bbox
    return f'{np.sqrt((((cx_bbox - (width//2)))**2 )+ (((cy_bbox - (height//2)))**2 ))} pixel'


#Function การควบคุมหุ่นในแกน X หรือ การควบคุมพิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error ในแนวแกน X อยู่ในช่วงที่ต้องการ
#กำหนดให้ error (x) อยู่ในช่วง +- 20 pixel 
def xaxis_control(speed,xaxis_error):

    if xaxis_error > 20:
        speed += 11
        ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=speed, w4=-speed)
        time.sleep(0.001)

    elif xaxis_error < 20:
        speed -= 11
        speed = -speed
        ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=-speed, w4=speed)
        time.sleep(0.001)

    else:
        return str("Done")
    

#Function การควบคุมหุ่นในแกน Y หรือ การควบคุมให้พิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error ในแนวแกน Y อยู่ในช่วงที่ต้องการ
#กำหนดให้ error (y) อยู่ในช่วง +- 5 pixel 
def yaxis_control(yaxis_error):
    global angle_1,angle_2

    
    if yaxis_error > 5:
        angle_1 += 3
        angle_1_min = min(40,angle_1)

        if angle_2 > 20:
            angle_2 -= 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)
        
        elif angle_1 < 40:
            ep_servo.moveto(index=1, angle=angle_1_min).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 >= 40:
            angle_2 -= 1
            angle_2_max = max(-40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_max).wait_for_completed()
            time.sleep(0.001)
    

    elif yaxis_error < -5:
        angle_1 -= 3
        angle_1_max = max(-40,angle_1)

        if angle_1 == 40:
            angle_2 += 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 > -40:
            ep_servo.moveto(index=1, angle=angle_1_max).wait_for_completed()
            time.sleep(0.001)   

        elif angle_1 <= -40:
            angle_2 += 1
            angle_2_min = min(40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_min).wait_for_completed()
            time.sleep(0.001)


    else :
        return str("Done")
    

    if abs(angle_1) > 40 and abs(angle_2) > 40:
        return str("she's such an angel")


#Function เข้าหาเป้าหมาย โดยที่ยังควบคุมพิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error อยู่ในช่วงที่ต้องการ
def Get_Closer(yaxis_error):
    speed = 15

    if yaxis_control(yaxis_error) == str("Done"):
        ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        time.sleep(0.001)
    

    elif yaxis_control(yaxis_error) == str("she's such an angel") :
        ep_chassis.drive_wheels(w1=-speed, w2=-speed, w3=-speed, w4=-speed)
        time.sleep(0.001)
        while yaxis_control != str("Done"):
            yaxis_control(yaxis_error)
        else: 
            return (str("move on"))
    
    else:
        yaxis_control(yaxis_error)


#Function การทำงานทั้งหมดในส่วนของหุ่น 
def Robot_Processing():
    global x,y,w,h,cx_bbox,cy_bbox,width,height,feed_x,iter,angle_1,angle_2
    iter = 1
    angle_1 = 0
    angle_2 = 0

    ep_servo.moveto(index=2, angle=0).wait_for_completed()
    time.sleep(0.001)
    ep_servo.moveto(index=1, angle=0).wait_for_completed()
    time.sleep(0.001)

    p_time = time.time()
    time.sleep(1)


    while True:
        time.sleep(0.001)

        img = ep_camera.read_cv2_image(strategy="newest")
        x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)

        if w >= 10 and h >= 10 :
            print(f'distance : {round(compute_distance())}')

            c_time = time.time()

            e_x = cx_bbox - (width//2)
            e_y = cy_bbox - (height//2)

            kp = 0.04
            kd = 0.02

            if iter > 1:

                speed_x = (kp*e_x) + kd*((feed_x-e_x)/(p_time-c_time))

                if xaxis_control(speed_x , e_x) == str("Done"):
                    if Get_Closer(e_y) != (str("move on")):
                        Get_Closer(e_y)
                    else:
                        print(Get_Closer(e_y))
                        break
            iter += 1

            p_time = time.time()

            feedback_x = e_x
            time.sleep(0.001)

        else:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=-0)
            time.sleep(0.001)



if __name__ == "__main__":
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_servo = ep_robot.servo
    ep_camera = ep_robot.camera
    ep_vision = ep_robot.vision
    ep_chassis = ep_robot.chassis
    ep_camera.start_video_stream(display = False)

    robot_process = threading.Thread(target= Robot_Processing)
    robot_process.start()

    while True :

        if keyboard.is_pressed('q'):
            break

        img = ep_camera.read_cv2_image(strategy = "newest")
        x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)

        show_bounding_box()

    cv.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
