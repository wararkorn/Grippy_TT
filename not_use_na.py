import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

chic = cv.imread('.\pic\littlechic2.png')
chic = cv.convertScaleAbs(chic, alpha=0.5, beta=20)

# bblur = cv.GaussianBlur(chic,(5,5),0)
bblur = cv.medianBlur(chic,5)
chic = cv.convertScaleAbs(bblur, alpha=1.2, beta=15)

# edges = cv.Canny(bblur, 50, 150)

hsv = cv.cvtColor(chic,cv.COLOR_RGB2HSV_FULL)



reduce = (100 <= hsv[:,:,0]) & (hsv[:,:,0] >= 40)  & (hsv[:,:,1] <= 200) & (hsv[:,:,1] >= 150).astype(np.uint8) 
reduce = reduce*255
hsv = cv.bitwise_not(hsv)
print(reduce)
# plt.show()
cv.imshow('chicchic',reduce)
cv.imshow('eiei',chic)
cv.waitKey()
# plt.show()
'''& (hsv[:,:,0] <= 125) & (hsv[:,:,1] >= 220) & (170 <= hsv[:,:,2]) & ( hsv[:,:,2] <= 180)'''