from robomaster import robot
import time

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_vision = ep_robot.vision
    ep_chassis = ep_robot.chassis
    ep_servo = ep_robot.servo

    # k = 0.1
    e_y = 20
    # print(e_y)

    # if move_x(speed_x,e_x) == "Perfect":
    
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    speed_y = 10
    # print(speed_y)
    # for _ in range(2):
    ep_servo.moveto(index=1, angle=-10).wait_for_completed()
    # time.sleep(1)
    
    