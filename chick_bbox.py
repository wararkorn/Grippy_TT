import cv2 as cv 
import numpy as np 
import matplotlib.pyplot as plt

def bounding_box(img):
    height = img.shape[0]
    width = img.shape[1]
    # o_chick = cv.imread('.\pic\littlechic7.png')
    brightness_chick = cv.convertScaleAbs( img ,alpha= 0.5,beta=20)

    # blur_chick = cv.GaussianBlur(brightness_chick,(5,5),0)
    # blur_chick = cv.medianBlur(brightness_chick,3)
    blur_chick = cv.medianBlur(brightness_chick,11)

    clean_chick  = cv.convertScaleAbs(blur_chick, alpha= 1.2 , beta= 20)

    hsv_c = cv.cvtColor(clean_chick, cv.COLOR_RGB2HSV_FULL)

    # c_threshold = (100 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] <= 200) & (hsv_c[:,:,1] >= 150).astype(np.uint8) 
    # c_threshold = (100 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] >= 150) &(hsv_c[:,:,2] >= 105).astype(np.uint8) 
    c_threshold = (100 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] >= 150).astype(np.uint8) 
    # c_threshold = (105 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] >= 145) &(hsv_c[:,:,2] >= 110).astype(np.uint8)
    c_threshold = c_threshold*255

    contours , _ = cv.findContours(c_threshold , cv.RETR_TREE ,cv.CHAIN_APPROX_SIMPLE)
    
    if len(contours) != 0:
        x,y,w,h = cv.boundingRect(contours[0])
        
        cx_bbox = round(((x+w)+x)/2)
        cy_bbox = round(((y+h)+y)/2)
#---------------------------------------------------------------------
        if w < 13 or h < 13:
            blur_chick = cv.medianBlur(brightness_chick,19)
            # blur_chick = cv.medianBlur(o_chick,19)
            clean_chick  = cv.convertScaleAbs(blur_chick, alpha= 1.2 , beta= 20)
            hsv_c = cv.cvtColor(clean_chick, cv.COLOR_RGB2HSV_FULL)
            c_threshold = (100 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] >= 150).astype(np.uint8) 
            c_threshold = c_threshold*255

            contours , _ = cv.findContours(c_threshold , cv.RETR_TREE ,cv.CHAIN_APPROX_SIMPLE)

            if len(contours) != 0:
                x,y,w,h = cv.boundingRect(contours[0])
                
                cx_bbox = round(((x+w)+x)/2)
                cy_bbox = round(((y+h)+y)/2)

#---------------------------------------------------------------------

    
        return x,y,w,h,cx_bbox,cy_bbox,width,height
    else :
        x,y,w,h,cx_bbox,cy_bbox = 0,0,0,0,0,0
        return x,y,w,h,cx_bbox,cy_bbox,width,height


def show_bounding_box():
    global img,x,y,w,h,cx_bbox,cy_bbox,width,height
    if w >= 10 and h >= 10 :
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