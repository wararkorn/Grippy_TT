from ultralytics import YOLO
import cv2 as cv

o_chick = cv.imread("D:\\PSU\\241-251\\Chic_Chicccc\\Yolov8Newgen\\data\\images\\train\\chick16.jpg")
model = YOLO("D:\\PSU\\241-251\\Chic_Chicccc\\Yolov8Newgen\\runs\\detect\\train\\weights\\best.pt")
results = model.predict("D:\\PSU\\241-251\\Chic_Chicccc\\Yolov8Newgen\\data\\images\\train\chick16.jpg")
result = results[0]
box = result.boxes
c = box.cls

# print(box)
# print(c)
print(c.sum())
# for i in range(len(c)):
#     if c[i] == 0:
#         b = box.xyxy[i]
#         x = int(b[0])
#         y = int(b[1])
#         w = int(b[2] - b[0])
#         h = int(b[3] - b[1])

        # cv.rectangle(o_chick , (x,y) , (x+w , y+h) , (0 , 255 , 0) , 1)



    #     cv.rectangle(o_chick , (x,y) , (x+w , y+h) , (0 , 255 , 0) , 1)
    #     # cv.circle(o_chick, (x,y), 3, (0, 10, 0), -1)
    # cv.imshow("Test",o_chick)
    # # # # # cv.imshow("Test2",c_threshold)
    # cv.waitKey(0)
    # cv.destroyAllWindows()
