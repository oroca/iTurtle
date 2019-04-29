# iturtle_omxplayer 
ROS Sound Player node based on omxplayer

## prerequisites

```
$ sudo apt-get install omxplayer
```

## workspace

```
$ cd catkin_ws
$ mkdir -p src
$ git clone https://github.com/oroca/iTurtle.git
$ cp -r iTurtle/tutorials/iturtle_iturtle_omxplayer  src/
$ catkin_make
$ catkin_make install
```

## run

```
$ source install/setup.bash
$ roscore
```

```
$ rosrun ros_omxplayer ros_omxplayer
or 
$ roslaunch ros_omxplayer omxplayer.launch
```

## test

copy `hello.m4a` to your home folder
```
$ rostopic pub -1 /play_sound_file std_msgs/String "$HOME/hello.m4a"
```

