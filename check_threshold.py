import cv2 as cv 
import numpy as np 
import matplotlib.pyplot as plt

o_chick = cv.imread('.\\pic\\littlechic2.png')
brightness_chick = cv.convertScaleAbs( o_chick ,alpha= 0.5,beta= 20)

# blur_chick = cv.GaussianBlur(brightness_chick,(21,21),0)
# blur_chick = cv.medianBlur(blur_chick,9)
blur_chick = cv.medianBlur(o_chick,11)
clean_chick  = cv.convertScaleAbs(blur_chick, alpha= 1.2 , beta= 50)

hsv_c = cv.cvtColor(clean_chick, cv.COLOR_RGB2HSV_FULL)

# c_threshold = (hsv_c[:,:,0] >= 100) &(hsv_c[:,:,0] <= 200) & (hsv_c[:,:,1] >= 85) & (hsv_c[:,:,1] <= 180) & (hsv_c[:,:,2] >= 120).astype(np.uint8) 
c_threshold = (100 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] >= 150).astype(np.uint8) 
c_threshold = c_threshold*255

#-----------------------------------------------------------------------------------------------------------------------

contours , Hierarchy = cv.findContours(c_threshold , cv.RETR_TREE ,cv.CHAIN_APPROX_SIMPLE)
all_pick = []
for cnt in contours:
    peri = cv.arcLength(cnt,True)
    approx = cv.approxPolyDP(cnt, 0.02*peri , True)
    x,y,w,h = cv.boundingRect(approx)
    # bob = c_threshold[y:y+h,x:x+w]
    # all_pick.append((x , bob))
    cv.rectangle(o_chick , (x,y) , (x+w , y+h) , (0 , 255 , 0) , 1)

cv.imshow("Test",o_chick)
cv.imshow("Test2",c_threshold)
cv.waitKey(0)
cv.destroyAllWindows()

#------------------------------------------------------------------------------------------------------------------------
# if len(contours) != 0:
#     x,y,w,h = cv.boundingRect(contours[0])

#     cx_bbox = round(((x+w)+x)/2)
#     cy_bbox = round(((y+h)+y)/2)
#     if w < 13 or h < 13:
#             blur_chick = cv.medianBlur(brightness_chick,19)
#             # blur_chick = cv.medianBlur(o_chick,19)
#             clean_chick  = cv.convertScaleAbs(blur_chick, alpha= 1.2 , beta= 20)
#             hsv_c = cv.cvtColor(clean_chick, cv.COLOR_RGB2HSV_FULL)
#             c_threshold = (100 <= hsv_c[:,:,0])  & (hsv_c[:,:,1] >= 150).astype(np.uint8) 
#             c_threshold = c_threshold*255

#             contours , _ = cv.findContours(c_threshold , cv.RETR_TREE ,cv.CHAIN_APPROX_SIMPLE)

#             if len(contours) != 0:
#                 x,y,w,h = cv.boundingRect(contours[0])
                
#                 cx_bbox = round(((x+w)+x)/2)
#                 cy_bbox = round(((y+h)+y)/2)



# else:
#     x,y,w,h = 0,0,0,0

#     cx_bbox = round(((x+w)+x)/2)
#     cy_bbox = round(((y+h)+y)/2)

# if w >= 10 and h >= 10 :
#     print(f'width : {w} height : {h}')
#     bbox = cv.rectangle(o_chick , (x,y) , (x+w,y+h) , (0 , 255 , 0) , 1)
#     cv.imshow("bbox",o_chick)
#     cv.imshow("binary",c_threshold)
#     cv.waitKey(0)
#     cv.destroyAllWindows()
# else:
#     print(f'width : {w} height : {h}')
#     cv.imshow("binary",c_threshold)
#     cv.imshow("image",o_chick)
#     cv.waitKey(0)
#     cv.destroyAllWindows()

#-------------------------------------------------------------------------------------------------------------------------