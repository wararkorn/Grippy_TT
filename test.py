from robomaster import robot , camera
import cv2 as cv
import numpy as np 
import matplotlib.pyplot as plt
import time
from chick_bbox import bounding_box
from perfect_move import move_x
import keyboard
    

    
def move_x(speed,errorx):
    if errorx > 10:
            speed += 11

        # speed = min(speed,70)   
        # if speed < 12 and speed > 5:
        #     speed += 7.5
        # print(speed)
            ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=speed, w4=-speed)
            time.sleep(0.001)
        # else:
        #     ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=speed, w4=-speed)
        #     time.sleep(0.001)
            print(speed)
    # elif errorx < 0:
    elif errorx < -10: 
            speed -= 11
        # speed = max(speed,-70)   
        # if speed > -12 and speed < -5:
        #     speed -= 7.5
            speed = -speed
            ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=-speed, w4=speed)
            time.sleep(0.001)
        # else :
        #     ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=speed, w4=-speed)
        #     time.sleep(0.001)
    else :
         return str("Perfect")

    print(speed)

def move_y(speed,errory):
    # speed = min(speed,80)
    if errory > 10:
            speed += 11
        # if speed < 12 and speed > 3:
        #     speed += 8.5
        #     ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        #     time.sleep(0.001)
        # else :
            ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
            time.sleep(0.001)

    elif errory < -10:
    # else:
            speed -= 11
        # if speed > -12 and speed < -3:
        #     speed -= 8.5
        #     ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        #     time.sleep(0.001)
        # else :
            ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
            time.sleep(0.001)
    # print(speed)
    else:
         return str("Perfect")

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_vision = ep_robot.vision
    ep_chassis = ep_robot.chassis

    ep_camera.start_video_stream(display=False)

    iter = 1
    p_time = time.time()
    time.sleep(1)
    start = time.time()

    while True:
        if keyboard.is_pressed('q'):
            break
        
        img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
        x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)

        c_time = time.time()

        e_x = cx_bbox - round(width/2)
        e_y = round(height/2) - cy_bbox

        kp_x = 0.08
        kd_x = 0.006

        kp_y = 0.15
        kd_y = 0

        if w > 20 and h > 20:

            if iter > 1:
                speed_x = ((kp_x*e_x) + kd_x*((feedback_x-e_x)/(p_time-c_time)))
                speed_y = ((kp_y*e_y) + kd_y*((feedback_y-e_y)/(p_time-c_time)))
                move_x(speed_x,e_x)
                if move_x(speed_x,e_x) == "Perfect":
                    if move_y(speed_y,e_y) is None:
                        move_y(speed_y,e_y)
                    else:
                        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
                        time.sleep(0.001)
                        end = time.time()
                        print(f'Used Time: {end - start}')
                        start = time.time()
                    
                # move_y(speed_y,e_y)
                # if abs(e_x) < 15 and abs(e_y) > 10:
                #     print(speed_y)
                #     move_y(speed_y,e_y)

                
                # check = speed_x < 2 and speed_y <2
                # check2 = e_x < 7 and e_y <7

                # if check and check2:
                #     end = time.time()
                #     print(f'เวลาที่ใช้ : {end - start}')
                #     # time.sleep(1)
                #     start = time.time()

            iter += 1

            p_time = time.time()
            
            feedback_x = e_x
            feedback_y = e_y
            time.sleep(0.001)

            cv.rectangle(img , (x,y) , (x+w,y+h) , (0 , 0 , 0) , 1)
            cv.putText(img, f'{cx_bbox},{cy_bbox}', (cx_bbox,cy_bbox), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
            cv.circle(img, (cx_bbox, cy_bbox), 3, (0, 10, 0), -1)

            
            
        cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
        cv.imshow("Robot", img)
        cv.waitKey(1)


    cv.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()
