from object_tracker import ObjectFinder
from object_tracker import MouseListener
import cv2

filenames = ["/home/jackie/ros_ws/src/baxter_demos/assets/flower"+str(i)+".jpg" for i in range(1, 4)] #lazy for now
cv2.namedWindow("Original")
cv2.namedWindow("Processed")

for filename in filenames:
    #Load file and display it
    img = cv2.imread(filename)
    cv2.imshow("Original", img)
    #Get a mouseclick
    ml = MouseListener()
    cv2.setMouseCallback("Original", ml.onMouse)
    while not ml.done:
        cv2.waitKey(10)
    detector = ObjectFinder("color", (ml.x_clicked, ml.y_clicked))
    detector.radius = 5
    thresholded = detector.colorDetect(img)
    cv2.imshow("Processed", thresholded)
    cv2.waitKey()
