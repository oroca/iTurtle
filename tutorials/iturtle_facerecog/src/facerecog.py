#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import rospy
from std_msgs.msg import String
# from picamera.array import PiRGBArray
# from picamera import PiCamera
import time
import cv2
import os
import numpy


fn_haar = 'haarcascade_frontalface_default.xml'
fn_dir = 'att_faces'
(im_width, im_height) = (112, 92)
names = {}

# OpenCV trains a model from the images
# NOTE FOR OpenCV2: remove '.face'
#model = cv2.face.createFisherFaceRecognizer()
#model = cv2.face.FisherFaceRecognizer_create()
model = cv2.face.LBPHFaceRecognizer_create()
#model = cv2.face.EigenFaceRecognizer_create()


def train():
    print('Training...')

    id = 0
    # Get the folders containing the training data
    for (subdirs, dirs, files) in os.walk(fn_dir):

        # Loop through each folder named after the subject in the photos
        for subdir in dirs:
            names[id] = subdir
            id = id + 1
    print("names=%d" % len(names))
    model.read('models/face_detection_model.xml')
    return 0


def facerecog():
    size = 2

    rospy.init_node('facerecog', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    pub = rospy.Publisher('facerecog', String, queue_size=10)

    # initialize the camera and grab a reference to the raw camera capture
    # camera = PiCamera()
    # camera.resolution = (640, 480)
    # camera.framerate = 32
    # rawCapture = PiRGBArray(camera, size=(640, 480))
    camera = cv2.VideoCapture(0)
    # Check if camera opened successfully
    if not camera.isOpened:
        print("Error opening video stream or file")
        return -1

    # allow the camera to warmup
    time.sleep(0.1)

    model_path = os.path.join(os.path.dirname(__file__), fn_haar)
    haar_cascade = cv2.CascadeClassifier(model_path)

    # capture frames from the camera
    t1 = rospy.get_time()
    # for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    (rval, frame) = camera.read()
    while rval:

        t2 = rospy.get_time()
        #print("captured: %d" % (t2-t1))
        if rospy.is_shutdown():
            print("ros shuting down...")
            break

        # Flip the image (optional)
        frame = cv2.flip(frame, 1, 0)

        # Convert to grayscalel
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Resize to speed up detection (optinal, change size above)
        mini = cv2.resize(
            gray, (int(gray.shape[1] / size), int(gray.shape[0] / size)))

        # Detect faces and loop through each one
        faces = haar_cascade.detectMultiScale(mini)
        for i in range(len(faces)):
            face_i = faces[i]

            # Coordinates of face after scaling back by `size`
            (x, y, w, h) = [v * size for v in face_i]
            face = gray[y:y + h, x:x + w]
            face_resize = cv2.resize(face, (im_width, im_height))

            # Try to recognize the face
            prediction = model.predict(face_resize)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)

            print(prediction)
            # [1]
            # Write the name of recognized face
            #cv2.putText(frame,'%s - %.0f' % (names[prediction[0]],prediction[1]),(x-10, y-10), cv2.FONT_HERSHEY_PLAIN,1,(0, 255, 0))
            if prediction[1] < 150:
                cv2.putText(frame, '%s - %.0f' % (names[prediction[0]], prediction[1]),
                            (x-10, y-10), cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 0))
                print('%s - %.0f' % (names[prediction[0]], prediction[1]))
                pub.publish(str(names[prediction[0]]))
            else:
                cv2.putText(frame, 'not recognized', (x-10, y-10),
                            cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))

        # Show the image and check for ESC being pressed
        cv2.imshow('OpenCV', frame)
        key = cv2.waitKey(10)
        if key == 27:
            print("esc")
            break

        # clear the stream in preparation for the next frame
        # rawCapture.truncate(0)

        rate.sleep()
        t1 = rospy.get_time()
        (rval, frame) = camera.read()
    return 0


if __name__ == '__main__':
    try:
        train()
        facerecog()
    except rospy.ROSInterruptException:
        pass
