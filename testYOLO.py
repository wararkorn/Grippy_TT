from ultralytics import YOLO
import cv2 as cv

img = "D:\\PSU\\241-251\\Chic_Chicccc\\Yolov8Newgen\\data\\images\\train\\chick232.jpg"
o_chick = cv.imread(f'{img}')
model = YOLO("D:\\PSU\\241-251\\Chic_Chicccc\\Yolov8Newgen\\runs\\detect\\train\\weights\\best.pt")
results = model.predict(f'{img}')
result = results[0]
box = result.boxes
c = box.cls

for i in range(len(c)):
    if c[i] == 1:
        b = box.xyxy[i]
        x = int(b[0])
        y = int(b[1])
        w = int(b[2] - b[0])
        h = int(b[3] - b[1])

        cx_bbox = round(((x+w)+x)/2)
        cy_bbox = round(((y+h)+y)/2)
        yaxis_error = o_chick.shape[0]//2 - cy_bbox
        xaxis_error = cx_bbox - (o_chick.shape[1]//2)

        cv.rectangle(o_chick , (x,y) , (x+w , y+h) , (204 , 244 , 0) , 1)



#     cv.rectangle(o_chick , (x,y) , (x+w , y+h) , (0 , 255 , 0) , 1)
#     # cv.circle(o_chick, (x,y), 3, (0, 10, 0), -1)
cv.imshow("Test",o_chick)
# # # cv.imshow("Test2",c_threshold)
cv.waitKey(0)
cv.destroyAllWindows()
