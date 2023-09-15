from robomaster import robot
from robomaster import vision
from robomaster import blaster
import time

def move_x(speed,errorx):
    print(speed)
    if errorx > 10:
        # ep_chassis.drive_wheels(w1=-speed*(errorx/abs(errorx)), w2=speed*(errorx/abs(errorx)), w3=speed*(errorx/abs(errorx)), w4=-speed*(errorx/abs(errorx)))
        ep_chassis.drive_wheels(w1=-speed, w2=speed, w3=speed, w4=-speed)
        time.sleep(0.001)

    elif errorx < 10:
        speed = -speed
        ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=-speed, w4=speed)
        time.sleep(0.001)

def move_y(speed,errory):
    print(speed)
    if errory > 10:
        # ep_chassis.drive_wheels(w1=-speed*(errorx/abs(errorx)), w2=speed*(errorx/abs(errorx)), w3=speed*(errorx/abs(errorx)), w4=-speed*(errorx/abs(errorx)))
        ep_chassis.drive_wheels(w1=-speed, w2=-speed, w3=-speed, w4=-speed)
        time.sleep(0.001)

    elif errory < 10:
        speed = -speed
        ep_chassis.drive_wheels(w1=speed, w2=-speed, w3=-speed, w4=speed)
        time.sleep(0.001)

if __name__=="__main__":
    ep_robot = robot.Robot()
    ep_chassis = ep_robot.chassis