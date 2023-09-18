import numpy as np
import math

def theta(angle_1,angle_2):

    if angle_1 >= 0 :
        theta1 = 37 - angle_1
    else:
        theta1 = angle_1 - 37

    theta1 = abs(theta1)

    if angle_2 == -12:
        theta2 = 0

    else:
        if angle_2 <= 0:
            theta2 = 90 + (abs(angle_2) - 12) 

        else: 
            theta2 = 90 - (angle_2 + 12) 

    return theta1 , theta2

theta1 , theta2 = theta(-8,-20)
print(f'มุม 1 : {theta1} | มุม 2 : {theta2}')


def cos_degree(theta):
    return round(np.cos(theta * np.pi/180),3)

def tan_degree(theta):
    return round(np.tan(theta * np.pi/180),3)

def sin_degree(theta):
    return round(np.sin(theta * np.pi/180),3)




