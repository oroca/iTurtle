# iturtle_facedetect

Simple face detection ROS project based on OpenCV.

## build

```
$ cd catkin_ws
$ mkdir -p src
$ git clone https://github.com/oroca/iTurtle.git
$ cp -r iTurtle/tutorials/iturtle_iturtle_facedetect  src/
$ catkin_make
$ cakkin_make install
```

## launch

```
$ source install/setup.bash
$ roscore
```

facedetect/facedetect listener node
```
$ roslaunch iturtle_facedetect facedetect.launch
```
