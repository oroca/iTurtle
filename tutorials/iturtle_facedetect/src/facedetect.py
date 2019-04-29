#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
from std_msgs.msg import String
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os


def face_detect():
    rospy.init_node('facedetect', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    pub = rospy.Publisher('facedetect', String, queue_size=10)

    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))

    # allow the camera to warmup
    time.sleep(0.1)

    model_path = os.path.join(os.path.dirname(__file__), 'haarcascade_frontalface_alt.xml')
    face_cascade = cv2.CascadeClassifier(model_path)

    # capture frames from the camera
    t1 = rospy.get_time()
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        t2 = rospy.get_time()
        #print("captured: %d" % (t2-t1))
        if rospy.is_shutdown():
            break

        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

        scaleFactor = 1.3
        minNeighbors = 5
        flags = 0
        minSize = (150,150)
        maxSize = (0,0)
        faces = face_cascade.detectMultiScale(gray, scaleFactor, minNeighbors, flags, minSize)
        if faces is not None and len(faces) > 0:
            t3 = rospy.get_time()
            rospy.loginfo('face detected: %s, captured=%s, detected=%s, total=%s' % (str(faces), t2 - t1, t3 - t2, t3 - t1))
            pub.publish(str(faces))

        # for (x,y,w,h) in faces:
        #     cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
        # # show the frame
        # cv2.imshow("Frame", image)

        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
               
        rate.sleep()
        t1 = rospy.get_time()


if __name__ == '__main__':
    try:
        face_detect()
    except rospy.ROSInterruptException:
        pass

