## README

Turtlebot3 package has been built by installing and building the source.

![default](images/Fig.d1_Turtlebot3Package.JPG)



### gazebo world 실행

- Web Shell #1

```
$ source ~/turtlebot3_ws/install/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

- Mouse control in Gazebo 
    - 메뉴 : Camera 
        - Perspective = CHECK
        - Orbit View Control = CHECK

![default](images/Fig.d2_GazeboMouseControl.JPG)

### teleop_keyboard 실행

- Web Shell #2

```
$ source ~/turtlebot3_ws/install/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard
```

    - 키보드 속도 제어 
        w : linear velocity 증가 
        s : linear & angular velocity 정지 
        x : linear velocity 감소 
        a : angular velocity 증가
        d : angular velocity 감소 

### rviz2 실행

- Web Shell #3

```
$ source ~/turtlebot3_ws/install/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_bringup rviz2.launch.py
```


```python

```


```python

```


```python

```
