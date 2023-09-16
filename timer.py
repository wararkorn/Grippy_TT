
from robomaster import robot , camera
import cv2 as cv
import numpy as np 
import matplotlib.pyplot as plt
import time
from chick_bbox import bounding_box
from perfect_move import move_x
import keyboard
import multiprocessing
import threading
import csv

def move_x(speed,errorx):
    if errorx > 20:
            speed += 11
            ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=speed, w4=-speed)
            time.sleep(0.001)
            
    elif errorx < -20: 
            speed -= 11
            speed = -speed
            ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=-speed, w4=speed)
            time.sleep(0.001)

    else :
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        return str("Perfect")



def controller():
    global p_time,c_time,img,e_x,e_y,feedback_x,feedback_y,iter,speed_x,angle_1,angle_2,start
    iter = 1
    angle_1 = 0
    angle_2 = 0
    ep_servo.moveto(index=2, angle=0).wait_for_completed()
    time.sleep(0.001)
    ep_servo.moveto(index=1, angle=0).wait_for_completed()
    start = time.time()
    time.sleep(0.001)
    p_time = time.time()
    while True:
        time.sleep(0.001)
        img = ep_camera.read_cv2_image(strategy="newest")
        x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)

        if w > 20 and h > 20 :
            c_time = time.time()    

            e_x = cx_bbox - (width//2)
            e_y = (height//2) - cy_bbox 

            kp_x = 0.04
            kd_x = 0
            
            # if w > 20 and h > 20:

            if iter > 1:
                speed_x = (kp_x*e_x) + kd_x*((feedback_x-e_x)/(p_time-c_time))
                if move_x(speed_x,e_x) == "Perfect":
                    Y_Controll()
                        # if closer_chainsmoker() == "sweet dream":
                        #     print("Can't Get Closer")
                        # else:
                        #     closer_chainsmoker()
                    # else:
                    #     Y_Controll()
                else:
                    move_x(speed_x,e_x)
            
                
            iter += 1

            p_time = time.time()
            
            feedback_x = e_x
            # feedback_y = e_y
            time.sleep(0.001)
        else:
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=-0)
            time.sleep(0.001)

def Y_Controll():
    global  x,y,w,h,cx_bbox,cy_bbox,e_x,speed_x,e_y,angle_1,angle_2,start
    # e_y = (height//2) - cy_bbox
    # print(e_y)
    # if move_x(speed_x,e_x) == "Perfect":
    print(e_y)

    if (angle_1 <= -35 and  angle_2 <= -35) & (e_y < 2 and e_y > -2) :
            return str("Eiei")
    
    if e_y > 2:
        angle_1 += 3
        angle_1 = min(40,angle_1)
        if  angle_2 > 20 :
            angle_2 -= 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)
    

        if angle_1 < 40:
            ep_servo.moveto(index=1, angle=angle_1).wait_for_completed()
            time.sleep(0.001)
            
        else:
            angle_2 -= 1
            angle_2 = max(-40,angle_2)
            print(f'eeeeeeeeeeeeeeeeeeee: {angle_2}')
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)

        # if  angle_2 > 20 :
        #     angle_2 -= 1
        #     ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
        #     time.sleep(0.001)
    
    elif e_y < -2:
        angle_1 -= 3
        angle_1 = max(-40,angle_1)
        print('eeeeeeeeeeeeeeeeeee' , angle_1)
        if angle_1 > -40:
            ep_servo.moveto(index=1, angle=angle_1).wait_for_completed()
            time.sleep(0.001)
    
        else:
            angle_2 += 1
            angle_2 = min(40,angle_2)
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.1)
            print('2222222222222' , angle_2)
        
        if angle_1 == 40:
            angle_2 += 1
            ep_servo.moveto(index=2, angle=angle_2).wait_for_completed()
            time.sleep(0.001)
    
        
    else:
        times = time.time() - start
        with open('timeprocess.csv', 'a', encoding='UTF8') as f:
            writer = csv.writer(f)
            writer.writerow([times])
        print(f'Elasped : {times}')
        time.sleep(3)
        start = time.time()
        # return str("Done")
    
def show_bbox(w,h):
    if w > 20 and h > 20 :
        cv.rectangle(img , (x,y) , (x+w,y+h) , (0 , 0 , 0) , 1)
        cv.putText(img, f'{cx_bbox},{cy_bbox}', (cx_bbox,cy_bbox), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        cv.circle(img, (cx_bbox, cy_bbox), 3, (0, 10, 0), -1)
        cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
        cv.imshow("Robot", img)
        cv.waitKey(1)
    else:
        cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
        cv.imshow("Robot", img)
        cv.waitKey(1)
         
def closer_chainsmoker():
    if Y_Controll() != str("Eiei"):
        speed = 13
        stop = 0
        ep_chassis.drive_wheels(w1=speed, w2=speed, w3=speed, w4=speed)
        time.sleep(0.1)
        # ep_chassis.drive_wheels(w1=stop, w2=stop, w3=stop,w4=stop)
        # time.sleep(0.1)
        Y_Controll()
    
    else:
        return str('sweet dream')
     

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_servo = ep_robot.servo
    ep_camera = ep_robot.camera
    ep_vision = ep_robot.vision
    ep_chassis = ep_robot.chassis
    # ep_servo.moveto(index=1, angle=0).wait_for_completed()
    # ep_servo.moveto(index=2, angle=0).wait_for_completed()
    ep_camera.start_video_stream(display=False)
    # iter = 1
    # speed_angle = 1
    wf1  = threading.Thread(target=controller)
    wf1.start()

    while True:
        if keyboard.is_pressed('q'):
            break
        
        img = ep_camera.read_cv2_image(strategy="newest")
        x,y,w,h,cx_bbox,cy_bbox,width,height = bounding_box(img)

        # # show_bbox(w,h)
        # if w > 20 and h > 20 :
        #     if XPD_controller() is not None:
        #         XPD_controller()  
        #     else:
        #         Y_Controll()
        
        show_bbox(w,h)
        
        #     cv.rectangle(img , (x,y) , (x+w,y+h) , (0 , 0 , 0) , 1)
        #     cv.putText(img, f'{cx_bbox},{cy_bbox}', (cx_bbox,cy_bbox), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        #     cv.circle(img, (cx_bbox, cy_bbox), 3, (0, 10, 0), -1)

            
            
        # cv.circle(img, (round(width/2),round(height/2)), 3, (0, 10, 0), -1)
        # cv.imshow("Robot", img)
        # cv.waitKey(1)


    cv.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()

