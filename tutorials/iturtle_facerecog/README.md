## iTurtle Face recognition

iTurtle Face recognition ROS package based on OpenCV


### Install

clone this repository

copy facerecog package into your catkin workspace.
`$ cp -r tutorials/iturtle_facerecog ~/catkinws/src/`

build and instlal
```
$ cd ~/catkinws/
$ catkin_make && catkin_make install
```

### How to use

run roscore firstly
```
$ cd ~/catkinws/
$ source install/setup.bash
$ roscore
```

launch trainning node
```
$ cd ~/catkinws/
$ source install/setup.bash
$ roslaunch iturtle_facerecog facerecog_train.launch name:=yourname
```

face recognition node
```
$ cd ~/catkinws/
$ source install/setup.bash
$ roslaunch iturtle_facerecog facerecog.launch
```


