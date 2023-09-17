import time
import cv2
from robomaster import robot
from robomaster import camera
import numpy as np
import keyboard
import threading

def mask_img(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_y_and_o = np.array([5, 145, 130])
    upper_y_and_o = np.array([50, 255, 255])
    list_colors = [(lower_y_and_o, upper_y_and_o)]
    for idx_color, (low, high) in enumerate(list_colors):
        mask_img = cv2.inRange(hsv_img, low, high)
    return mask_img

def calculate_distance_from_center(image, center_col, center_row):
    # คำนวณระยะทางในแนวแกน X (นอน)
    distance_x = abs(image.shape[1] // 2 - center_col)  
    # คำนวณระยะทางในแนวแกน Y (ตั้ง)
    distance_y = abs(image.shape[0] // 2 - center_row)  
    # ระยะทางสุทธิ (ระยะห่างจากจุดกลางภาพไปยังตำแหน่งกึ่งกลางของ Bounding Box)
    distance = (distance_x * 2 + distance_y * 2) ** 0.5  # ปรับเปลี่ยนการคำนวณระยะทาง
    return distance


def match_center_xy(center_chick_x,center_chick_y): # เดิน
    global fl, fr, l, r, delta_x, delta_y  
    delta_x =   int(center_chick_x) -640
    delta_y =  int(center_chick_y) -360 
    # start_time = time.time()
    if -10 < delta_x < 10: 
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
        time.sleep(0.01)
        servooo(delta_y)
        #end_time = time.time()
    elif  delta_x < 0:
        ep_chassis.drive_wheels(w1=fl, w2=fr * -1, w3=l, w4=r * -1)
        time.sleep(0.2)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    elif delta_x > 0:
        ep_chassis.drive_wheels(w1=fl * -1, w2=fr, w3=l * -1, w4=r)
        time.sleep(0.2)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    # end_time = time.time() if end_time is None else end_time  # ถ้าไม่มีการกำหนดเวลาสิ้นสุดให้ใช้เวลาปัจจุบัน
    # duration = end_time - start_time
    # print(f"ระยะเวลาการเคลื่อนที่: {duration:.2f} วินาที")

#---------------------------------servo1-------------------------
def servooo(delta_y):
    global servo_1, servo_2
    delta_y =  int(center_chick_y) -180 
    if delta_y < 5: #ขยับขึ้น
        servo_1 += 3   
    if delta_y > -5: #ขยับลง
        servo_1 -= 3
    if servo_1 < -99 :
        servo_1 = -100
    else :
        servo_2 = 60
    print('delta_y',delta_y)
    # ep_servo.moveto(index=1, angle=-100).wait_for_completed() #-30 - -110 ค่า -100 คือลงสุด ถ้าเพิ่มสุด -23 
    # ep_servo.moveto(index=2, angle=80).wait_for_completed() #60 - 90 ค่า 80 คือ ค่าชิดสุด ถ้าเพิ่มสุด 70

# def move_forward():
#     if -5<delta_y<5 and 5<delta_x<5 :
#         ep_chassis.drive_wheels(w1 = fl , w2 = fr, w3 = l, w4 = r)
#     elif servo_1 == -100 :
#         ep_chassis.drive_wheels(w1 = 0, w2 = 0, w3 = 0, w4 = 0)
def show_haha():
    global frame,distance,center_chick_x,center_chick_y
    while True:
        if keyboard.is_pressed('q'):
            break
        
        frame = ep_camera.read_video_frame()  # อ่านเฟรมวิดีโอ
        image_x = int(frame.shape[1] / 2)
        image_y = int(frame.shape[0] / 2)
        if frame is not None:
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            yellow_mask = mask_img(frame)
            # ค้นหา Contours จากแมสก์สีเหลือง
            contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # หา Contour ที่มีพื้นที่สีเหลืองมากที่สุด
            max_contour = max(contours, key=cv2.contourArea, default=None)
            if max_contour is not None:
                x, y, w, h = cv2.boundingRect(max_contour)
                center_chick_y = y + h // 2
                center_chick_x = x + w // 2
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_chick_x, center_chick_y), 7, (0, 0, 255), -1) #-1
                cv2.circle(frame, (image_x, image_y), 5, (0, 0, 255), -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(frame, f'Center_row: {center_chick_x}, Center_col: {center_chick_y}', (x, y - 10), font, 0.5, (255, 255, 255), 2)
                print(frame.shape)
        #     # แสดงภาพที่มีกรอบสี่เหลี่ยม
        cv2.imshow('Yellow Object with Bounding Box', frame)
        cv2.waitKey(1)
        # distance = calculate_distance_from_center(frame, center_chick_x, center_chick_y)
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_servo = ep_robot.servo
    #ep_arm = ep_robot.robotic_arm
    ep_chassis = ep_robot.chassis
    ep_camera.start_video_stream(display=False)
    boundingbox = threading.Thread(target=show_haha)
    boundingbox.start()
    time.sleep(1)
    

    fl, fr, l, r = 13, 13, 13, 13
    servo_1 = -30
    servo_2 = 70

    while True:
        distance = calculate_distance_from_center(frame, center_chick_x, center_chick_y)
        print(f'ระยะทางสุทธิ: {distance} หน่วย pixel')
        match_center_xy(center_chick_x,center_chick_y)
        # move_forward()
        ep_servo.moveto(index=1, angle=servo_1).wait_for_completed()
        ep_servo.moveto(index=2, angle=servo_2).wait_for_completed()
        print(frame.shape[1])
        print(frame.shape[0])

        

        if keyboard.is_pressed('q'):
            break

    # ep_camera.stop_video_stream()
    # ep_robot.close()
    # cv2.destroyAllWindows()