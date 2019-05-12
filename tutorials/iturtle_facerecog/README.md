## iTurtle Face recognition

iTurtle Face recognition ROS package based on OpenCV


### Install

clone this repository

copy facerecog package into your catkin workspace.
```
$ cp -r tutorials/iturtle_facerecog ~/catkinws/src/
```

build and install the package
```
$ cd ~/catkinws/
$ catkin_make && catkin_make install
```

### How to use

**run roscore firstly**

```
$ cd ~/catkinws/
$ source install/setup.bash
$ roscore
```

**launch trainning node**

you may need to provide a label for the face via `name` parameter.
```
$ cd ~/catkinws/
$ source install/setup.bash
$ roslaunch iturtle_facerecog facerecog_train.launch name:=yourname
```

**face recognition node**

when it detects the face it shows the label which you provided before.
```
$ cd ~/catkinws/
$ source install/setup.bash
$ roslaunch iturtle_facerecog facerecog.launch
```
