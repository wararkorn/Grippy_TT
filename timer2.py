from robomaster import robot , camera
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import time 
from chick_bbox import bounding_box
import keyboard
import threading
import csv

#Function คำนวณระยะห่างระหว่าง พิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box
def compute_distance():
    global width,height,cx_bbox,cy_bbox
    return f'{round(np.sqrt((((cx_bbox - (width//2)))**2 )+ (((cy_bbox - (height//2)))**2 )))} pixel'


#Function การควบคุมหุ่นในแกน X หรือ การควบคุมพิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error ในแนวแกน X อยู่ในช่วงที่ต้องการ
#กำหนดให้ error (x) อยู่ในช่วง +- 20 pixel 
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
#กำหนดให้ error (y) อยู่ในช่วง +- 5 pixel 
def yaxis_control():
    global angle_1,angle_2,yaxis_error

    
    if yaxis_error > 10:
        print('yaxis_error > 5')
        angle_1 += 1
        angle_1_min = min(40,angle_1)

        if angle_2 > 20:
            print('yaxis_error > 5 | angle_2 > 20')
            angle_2 -= 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)
        
        elif angle_1 < 40:
            print('yaxis_error > 5 | angle_1 < 40')
            ep_servo.moveto(index=1, angle=angle_1_min).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 >= 40:
            print('yaxis_error > 5 | angle_1 >= 40')
            angle_2 -= 1
            angle_2_max = max(-40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_max).wait_for_completed()
            time.sleep(0.001)
    

    elif yaxis_error < -10:
        print(yaxis_error)
        print('yaxis_error < -5')
        angle_1 -= 1
        angle_1_max = max(-40,angle_1)

        if angle_1 == 40:
            print('yaxis_error < -5 | angle_1 == 40')
            angle_2 += 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)

        elif angle_1 > -40:
            print('yaxis_error < -5 | angle_1 > -40')
            ep_servo.moveto(index=1, angle=angle_1_max).wait_for_completed()
            time.sleep(0.001)   

        elif angle_1 <= -40:
            print('yaxis_error < -5 | angle_1 <= -40')
            angle_2 += 1
            angle_2_min = min(40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2_min).wait_for_completed()
            time.sleep(0.001)


    else :
        # print('return Done')
        return str("Done")
    

    if abs(angle_1) > 40 and abs(angle_2) > 40:
        print("she's such an angel")
        return str("she's such an angel")


#Function เข้าหาเป้าหมาย โดยที่ยังควบคุมพิกัดจุดกึ่งกลางของภาพ กับ พิกัดจุดกึ่งกลางของ Bounding Box ให้ error อยู่ในช่วงที่ต้องการ
def Get_Closer():
    global yaxis_error,end
    speed = 30
    stop = 0

    if yaxis_control() == str("Done"):
        ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        time.sleep(0.01)
        ep_chassis.drive_wheels(w1=stop, w2=stop, w3=stop, w4=stop)
        time.sleep(0.001)
    

    elif yaxis_control() == str("she's such an angel") :
        ep_chassis.drive_wheels(w1=-speed, w2=-speed, w3=-speed, w4=-speed)
        time.sleep(0.01)
        ep_chassis.drive_wheels(w1=stop, w2=stop, w3=stop, w4=stop)
        time.sleep(0.001)
        print('Already Move on ถอยออกมาก่อนเนาะ')
        end = True
    
    else:
        yaxis_control()


#Function การทำงานทั้งหมดในส่วนของหุ่น 
def Robot_Processing():
    global x,y,w,h,cx_bbox,cy_bbox,width,height,p_errorx,iter,angle_1,angle_2,yaxis_error,xaxis_error,end 
    iter = 1
    angle_1 = 0
    angle_2 = 0
    end = False
    start = time.time()

    ep_servo.moveto(index=2, angle=0).wait_for_completed()
    time.sleep(0.001)
    ep_servo.moveto(index=1, angle=0).wait_for_completed()
    time.sleep(0.001)

    p_time = time.time()
    time.sleep(1)

    start = time.time()

    while True:
        time.sleep(0.001)

        # img = ep_camera.read_cv2_image(strategy="newest")
        # x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)

        if w >= 10 and h >= 10 :
            print(f'distance : {compute_distance()}')

            c_time = time.time()

            xaxis_error = xaxis_error
            yaxis_error = yaxis_error

            print(f'cy_bbox : {cy_bbox} center : {height//2} y error : {yaxis_error}')

            kp = 0.3
            # kp = 0.03
            # kd = 0.0001


            if iter > 1:

                speed_pd = (kp*xaxis_error)# + kd*((p_errorx-xaxis_error)/(p_time-c_time))
                print(speed_pd)
                # speed_slow = 20

                if xaxis_control(speed_pd , xaxis_error) == str("Done"):
                    print('xaxis_control(speed_x , e_x) == str("Done")')
                    if yaxis_control() == str("Done"):
                        end = time.time()
                        print(f'ใช้เวลา : {end - start}')
                        time.sleep(5)
                        start = time.time()
                    else:
                        yaxis_control()
                    # if end and yaxis_control() == str("she's such an angel"):
                    #     print('พอเถอะพอ')
                    #     while yaxis_control() != str('Done'):
                    #         yaxis_control()
                    #     else:
                    #         print("Gorgeous")
                    #         break

                    # else:
                    #     print('ยังขยับได้อยู่')
                    #     Get_Closer()
                
                else:
                    print('xaxis_control(speed_x , e_x) != str("Done")')
                    xaxis_control(speed_pd , xaxis_error)


            iter += 1

            p_time = time.time()

            p_errorx = xaxis_error
            time.sleep(0.001)

        else:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=-0)
            time.sleep(0.001)


def show_bounding_box():
    global yaxis_error,xaxis_error,x,y,w,h,cx_bbox,cy_bbox,width,height
    while True :

        if keyboard.is_pressed('q'):
            break

        img = ep_camera.read_cv2_image(strategy = "newest")
        # img = ep_camera.read_video_frame()
        x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)


        if w >= 10 and h >= 10 :
            yaxis_error = height//2 - cy_bbox
            xaxis_error = cx_bbox - (width//2)

            cv.rectangle(img , (x,y) , (x+w,y+h) , (0 , 0 , 0) , 1)
            cv.putText(img, f'{cx_bbox},{cy_bbox}', (cx_bbox,cy_bbox), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv.circle(img, (cx_bbox, cy_bbox), 3, (0, 10, 0), -1)
            cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
            cv.imshow("Robot", img)
            cv.waitKey(1)
        
        cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
        cv.imshow("Robot", img)
        cv.waitKey(1)

    cv.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()


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
    boundingbox_process = threading.Thread(target= show_bounding_box)
    boundingbox_process.start()

    # while True :

    #     if keyboard.is_pressed('q'):
    #         break

    #     img = ep_camera.read_cv2_image(strategy = "newest")
    #     x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)

        # show_bounding_box()
    #     if w >= 10 and h >= 10 :
    #         cv.rectangle(img , (x,y) , (x+w,y+h) , (0 , 0 , 0) , 1)
    #         cv.putText(img, f'{cx_bbox},{cy_bbox}', (cx_bbox,cy_bbox), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    #         cv.circle(img, (cx_bbox, cy_bbox), 3, (0, 10, 0), -1)
    #         cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
    #         cv.imshow("Robot", img)
    #         cv.waitKey(1)
    #     else:
    #         cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
    #         cv.imshow("Robot", img)
    #         cv.waitKey(1)

    # cv.destroyAllWindows()
    # ep_camera.stop_video_stream()
    # ep_robot.close()
