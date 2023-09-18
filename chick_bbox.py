import cv2 as cv 
import numpy as np 
import matplotlib.pyplot as plt
import keyboard

def bounding_box(img):
    height = img.shape[0]
    width = img.shape[1]


    brightness_adjust = cv.convertScaleAbs( img ,alpha= 0.5,beta=20)
    blur_chick = cv.medianBlur(brightness_adjust,11)
    clean_chick  = cv.convertScaleAbs(blur_chick, alpha= 1.2 , beta= 20)
    hsv_c = cv.cvtColor(clean_chick, cv.COLOR_RGB2HSV_FULL)
    

    c_threshold = (100 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] >= 150).astype(np.uint8) 
    c_threshold = c_threshold*255

    contours , _ = cv.findContours(c_threshold , cv.RETR_TREE ,cv.CHAIN_APPROX_SIMPLE)
    
    if len(contours) != 0:
        x,y,w,h = cv.boundingRect(contours[0])
        
        cx_bbox = round(((x+w)+x)/2)
        cy_bbox = round(((y+h)+y)/2)
# ---------------------------------------------------------------------

        if w < 13 or h < 13:
            blur_chick = cv.medianBlur(brightness_adjust,19)
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
            else:
                x,y,w,h,cx_bbox,cy_bbox = 0,0,0,0,0,0
                return x,y,w,h,cx_bbox,cy_bbox,width,height
            
        else:
            return x,y,w,h,cx_bbox,cy_bbox,width,height
    else :
        x,y,w,h,cx_bbox,cy_bbox = 0,0,0,0,0,0
        return x,y,w,h,cx_bbox,cy_bbox,width,height


