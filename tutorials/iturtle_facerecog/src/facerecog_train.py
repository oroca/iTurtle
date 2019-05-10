#!/usr/bin/env python
# -*- encoding: UTF-8 -*-
import os
import rospy
from std_msgs.msg import String
import sys
import cv2
import numpy


size = 4
fn_haar = 'haarcascade_frontalface_default.xml'
fn_dir = 'att_faces'
(im_width, im_height) = (112, 92)
names = {}
model = cv2.face.LBPHFaceRecognizer_create()


def take_faces():

    rospy.init_node('facerecog_train', anonymous=False)
    rate = rospy.Rate(10)  # 10hz
    argv = rospy.myargv()
    print("myargv %s" % argv)
    # full_param_name = rospy.search_param('~label')
    # rospy.loginfo("full_param_name %s", full_param_name)
    label = rospy.get_param('~label')
    rospy.loginfo('Parameter %s has value %s',
                  rospy.resolve_name('~label'), label)

    if not label:
        label = argv[1]
    print("label=%s" % label)

    path = os.path.join(fn_dir, label)
    if not os.path.isdir(path):
        os.makedirs(path)

    model_path = os.path.join(os.path.dirname(__file__), fn_haar)
    haar_cascade = cv2.CascadeClassifier(model_path)
    webcam = cv2.VideoCapture(0)

    # Check if camera opened successfully
    if not webcam.isOpened:
        print("Error opening video stream or file")
        return -1

    # Generate name for image file
    pin = sorted([int(n[:n.find('.')])
                  for n in os.listdir(path) if n[0] != '.']+[0])[-1] + 1

    # Beginning message
    # print("\n\033[94mThe program will save 20 samples. \
    # Move your head around to increase while it runs.\033[0m\n")

    # The program loops until it has 20 images of the face.
    count = 0
    pause = 0
    count_max = 20
    while count < count_max:

        if rospy.is_shutdown():
            print("ros shuting down...")
            break

        # Put the image from the webcam into 'frame'
        (rval, frame) = webcam.read()
        if not rval:
            print("Failed to open webcam. Trying again...")
            break

        # Get image size
        height, width, channels = frame.shape

        # Flip frame
        frame = cv2.flip(frame, 1, 0)

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Scale down for speed
        mini = cv2.resize(
            gray, (int(gray.shape[1] / size), int(gray.shape[0] / size)))

        # Detect faces
        faces = haar_cascade.detectMultiScale(mini)

        # We only consider largest face
        faces = sorted(faces, key=lambda x: x[3])
        if faces:
            face_i = faces[0]
            (x, y, w, h) = [v * size for v in face_i]

            face = gray[y:y + h, x:x + w]
            face_resize = cv2.resize(face, (im_width, im_height))

            # Draw rectangle and write name
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
            cv2.putText(frame, label, (x - 10, y - 10), cv2.FONT_HERSHEY_PLAIN,
                        1, (0, 255, 0))

            # Remove false positives
            if(w * 6 < width or h * 6 < height):
                print("Face too small")
            else:

                # To create diversity, only save every fith detected image
                if(pause == 0):

                    print("Saving training sample " +
                          str(count+1)+"/"+str(count_max))

                    # Save image file
                    cv2.imwrite('%s/%s.png' % (path, pin), face_resize)

                    pin += 1
                    count += 1

                    pause = 1

        if(pause > 0):
            pause = (pause + 1) % 5
        cv2.imshow('OpenCV', frame)

        rate.sleep()
        if cv2.waitKey(1) >= 0:
            break

    webcam.release()

    # Closes all the frames
    cv2.destroyAllWindows()
    return 0


def train():
    global names
    # Part 1: Create fisherRecognizer
    print('Training...')

    if not os.path.isdir('models'):
        os.makedirs('models')

    # Create a list of images and a list of corresponding names
    (images, lables, id) = ([], [], 0)

    # Get the folders containing the training data
    for (subdirs, dirs, files) in os.walk(fn_dir):

        # Loop through each folder named after the subject in the photos
        for subdir in dirs:
            names[id] = subdir
            subjectpath = os.path.join(fn_dir, subdir)

            # Loop through each photo in the folder
            for filename in os.listdir(subjectpath):

                # Skip non-image formates
                f_name, f_extension = os.path.splitext(filename)
                if(f_extension.lower() not in ['.png', '.jpg', '.jpeg', '.gif', '.pgm']):
                    print("Skipping "+filename+", wrong file type")
                    continue
                path = subjectpath + '/' + filename
                lable = id

                # Add to training data
                images.append(cv2.imread(path, 0))
                lables.append(int(lable))
            id += 1

    # Create a Numpy array from the two lists above
    (images, lables) = [numpy.array(lis) for lis in [images, lables]]

    # # OpenCV trains a model from the images
    # # NOTE FOR OpenCV2: remove '.face'
    # #model = cv2.face.createFisherFaceRecognizer()
    # #model = cv2.face.FisherFaceRecognizer_create()
    # model = cv2.face.LBPHFaceRecognizer_create()
    # #model = cv2.face.EigenFaceRecognizer_create()
    model.train(images, lables)
    model.save('models/face_detection_model.xml')
    return 0


if __name__ == '__main__':
    try:
        print("sysv=%s" % sys.argv)
        if len(sys.argv) < 2:
            print("You must provide a name")
            sys.exit(0)
        take_faces()
        sys.exit(train())
    except rospy.ROSInterruptException:
        pass
