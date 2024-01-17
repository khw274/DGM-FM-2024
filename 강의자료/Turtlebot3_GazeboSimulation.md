# Turtlebot3_GazeboSimulation  

    - 워크스페이스 : turtlebot3_ws 


### 0. 파일 .bashrc

    - 명령창을 새로 열 때마다, 파일 .bashrc가 자동 실행된다. 
    - Turtlebot3 자율주행 실행을 위한 준비과정을 파일 .bashrc에 입력함
    - 히든 파일: .으로 시작하는 파일 

- Web Shell #1:

```
$ ls
$ ls -a
.bashrc

$ cat ~/.bashrc
......
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/local_setup.bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=burger

```

    - 관련 패키지 저장 폴더 :
        - ROS2/Gazebo/자율주행 관련 패키지 : /opt/ros/humble/
        - turtlebot3 관련 패키지 : (사용자계정 HOME)/turtlebot3_ws/




### 0. Turtlebot3 Gazebo Simulation

##### 1) Gazebo World 실행

- Web Shell #1:

```
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

##### 2) teleop_keyboard 실행

- Web Shell #2:

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

    - 키보드 속도 제어 
        w : linear velocity 증가 
        s : linear & angular velocity 정지 
        x : linear velocity 감소 
        a : angular velocity 증가
        d : angular velocity 감소 

##### 3) rviz2 실행

- Web Shell #3:

```
$ ros2 launch turtlebot3_bringup rviz2.launch.py

```


##### 4) Frame & Odometry 조사 

###### 4-1) Odometry 

- Web Shell #4: 

```
$ ros2 topic echo /odom

```

    - Odometry 
        - Wheel Encoder로부터 로봇의 위치와 방향을 계산함
        - 바퀴에 슬립이 일어나면 Odometry 데이터에 오류가 생김
        - Odometry 정확도를 높이기 위해, Wheel Encoder에 IMU와 GPS를 Sensor Fusion 하기도 함
        
    - 토픽 /odom 
        - TurtleBot3의 Odometry 데이터를 실시간으로 제공
        - Odometry는 로봇의 운동학적인 변화를 추적하고, 로봇의 현재 위치와 방향을 추정하는 데 사용
        - 토픽 /odom 토픽메시지 nav_msgs/msg/Odometry
            header:
                stamp: 
                    sec: 1700891810
                    nanosec: 191314675
                frame_id: odom
            child_frame_id: base_footprint
            pose:
                pose:
                    position:
                        x: -2.0
                        y: -0.5
                        z: 0.0
                    orientation:
                        x: 0.0
                        y: 0.0
                        z: 0.0
                        w: 1.0
                covariance:
                - 0.0
                ...... (총 36줄)
            twist:
                twist:
                    linear:
                        x: 0.0
                        y: 0.0
                        z: 0.0
                    angular:
                        x: 0.0
                        y: 0.0
                        z: 0.0
                covariance:
                - 0.0
                ......(총 36줄)


###### 4-2) Frame 

- Web Shell #5: 

```
$ ros2 run tf2_ros tf2_echo odom base_footprint

```
  
    - 프레임 odom
        - TurtleBot3의 Odometry 데이터를 나타내기 위한 고정 좌표계 프레임
        - 프레임 odom 상에서, 차일드_프레임 base_footprint 좌표계의 위치와 방향을 나타냄 

    - 프레임 트리 
        - odom (고정 좌표계)
            - base_footprint
                - base_link
                    - wheel_left_link
                    - wheel_right_link
                    - caster_back_link
                    - imu_link
                    - base_scan 



### ※ Gazebo Simulation vs Real World

#### <span style="color:red">Gazebo Simulation</span> 

| 구분       | 내용            | Gazebo Simulation                                                                                    |
|:---:|:---:|:---:|
| SLAM       | Gazebo World    | \$ ros2 launch <span style="color:red">turtlebot3_gazebo turtlebot3_world.launch.py</span>           |
| "          | SLAM            | \$ ros2 launch turtlebot3_cartographer cartographer.launch.py <span style="color:red">use_sim_time:=True</span>                     |
| "          | Teleop_keyboard | \$ ros2 run turtlebot3_teleop teleop_keyboard                                                        |
| "          | Save Map        | \$ ros2 run nav2_map_server map_saver_cli -f ~/map                                                   |
| Navigation | Gazebo World    | \$ ros2 launch <span style="color:red">turtlebot3_gazebo turtlebot3_world.launch.py</span>           |
| "          | Navigation      | \$ ros2 launch turtlebot3_navigation2 navigation2.launch.py <span style="color:red">use_sim_time:=True</span> map:=$HOME/map.yaml       |
| "          | Teleop_keyboard | \$ ros2 run turtlebot3_teleop teleop_keyboard                                                        |

#### <span style="color:lightblue">Real Robot</span> 

| 구분       | 내용            | Real Robot                                                                        |
|:---:|:---:|:---:|
| SLAM       | Real World      | \$ ros2 launch <span style="color:lightblue">turtlebot3_bringup robot.launch.py</span> | 
| "          | SLAM            | \$ ros2 launch turtlebot3_cartographer cartographer.launch.py                     | 
| "          | Teleop_keyboard | \$ ros2 run turtlebot3_teleop teleop_keyboard                                     |
| "          | Save Map        | \$ ros2 run nav2_map_server map_saver_cli -f ~/map                                |
| Navigation | Real World      | \$ ros2 launch <span style="color:lightblue">turtlebot3_bringup robot.launch.py</span> | 
| "          | Navigation      | \$ ros2 launch turtlebot3_navigation2 navigation2.launch.py <span style="color:black">map:=$HOME/map.yaml</span>  | 
| "          | Teleop_keyboard | \$ ros2 run turtlebot3_teleop teleop_keyboard                                     | 




### 1. SLAM

    - 목적 : 주변 지형에 대한 정보를 바탕으로 map 작성

    - SLAM (Simultaneous Localization and Mapping)
        - 주변 위치 정보로 부터 위치를 추정하고 
        - 주변 환경에 대한 지도를 동시에 작성하는 기술
        - 종류 : 
    
| SLAM 방법 종류       | 개발사                                 | 2D/3D | Static/Dynamic | 특징            | 라이센스           |
|:---:|:---:|:---:|:---:|:---:|:---:|
| <span style="color:red">Cartographer</span>         | Google                                 | 2D/3D | Static/Dynamic | 지속적 업데이트 | Apache License 2.0 |
| Hector               | 헥터 SLAM 팀                           | 2D    | Static/Dynamic | -               | BSD License        |
| Karto                | OSRF (Open Source Robotics Foundation) | 2D    | Static         | -               | New BSD License    |
| <span style="color:lightblue">Gmapping</span>             | OpenSLAM community                     | 2D    | Static         | 업데이트 종료   | CCL License        |
| Frontier_exploration | OSRF (Open Source Robotics Foundation) | 2D    | Static(?)      | -               | Apache License 2.0 |
| RTAB-Map             | Mathieu Labbé                          | 2D/3D | Static/Dynamic | -               | GPL v3             |





#### 1.1 Gazebo World 실행

- Web Shell #1: 

```
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

    - Gazebo 창 열림 
        - Gazebo Menu > Camera :
            - Perspective = CHECK
            - Orbit View Control = CHECK
        - Dragging LMB : Panning
        - Dragging MMB : Rotating
        - Dragging RMB : Zoom In/Out (Dragging Upward - Zoom In, Dragging Downward - Zoom Out)
        - Scrolling MMB : Zoom In/Out (Scrolling Forward - Zoom In, Scrolling Rearward - Zoom Out) 


#### 1.2 SLAM Node 실행 

- Web Shell #2: 

```
$ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

```

    - RViz 창 열림 
        - RViz > Display Control Panel : 
            - Add > By display type > RobotModel > OK
            - RobotModel > Description Topic = /robot_description
            - TF = UNCHECK 
        - Dragging LMB : Rotating
        - Dragging MMB : Panning
        - Dragging RMB : Zoom In/Out (Dragging Upward : Zoom In, Dragging Downward : Zoom Out)
        - Rotating MMB : Zoom In/Out (Rotating Outward : Zoom In, Rotating Inward : Zoom Out) 


#### 1.3 teleop_keyboard 실행 

- Web Shell #3: 

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

    - 정리 
        - 로봇을 천천히 움직이면서, 주변 환경을 남김없이 탐색하여 맵 완성 


#### 1.4 Map 저장 

- Web Shell #4: 

```
$ cd ~
$ ros2 run nav2_map_server map_saver_cli -f ~/map0

$ ls map0.*
map0.pgm  map0.yaml

```

    - 정리 
        - 주변 환경 탐색이 남김없이 완료된 후에, 맵을 저장함
        - map0.yaml 파일(지도 정보 파일)과 map0.pgm 파일(지도 이미지 파일) 생성
        - map0.yaml
            image: map0.pgm
            mode: trinary
            resolution: 0.05
            origin: [-1.24, -2.41, 0]
            negate: 0
            occupied_thresh: 0.65
            free_thresh: 0.25
        - map0.pgm 
            image file



### 2 Navigation

    - Navigation
        - 맵에서 출발 지점과 도착 지점을 지정하여 주면 (출발 지점은 현재 로봇의 위치에 정확하게 일치시켜야 함)
        - Global Map에서 출발 지점으로부터 도착 지점까지 정적인 장애물을 회피하도록 전역 경로 계획을 설정하고 
        - Local Map에서 이동 중인 로봇 주변의 정적 및 동적인 장애물을 회피하도록 지역 경로 계획을 설정함  

#### 2.1 Gazebo World 실행

- Web Shell #1: 

```
$ ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

    - Gazebo 창 열림 

#### 2.2 Navigation Node 실행 


##### 1)  Navigation Node 실행

- Web Shell #2: 

```
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map0.yaml

```

    - RViz 창 열림 


##### 2) Initial Pose 설정

- Graphical tools: 
    - RViz2 메뉴 > <span style="color:red">[2D Pose Estimate]</span> button 클릭
        - 드래깅 위치: 로봇 위치 설정
        - 드래깅 방향: 로봇 방향 설정
    - RViz > Display Control Panel :
        - RobotModel = CHECK
        - TF = UNCHECK 


##### 3) Localization 정확도 향상 (: 생략 가능)

- Web Shell #3: 

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

    ※ 주의 사항  
        - 키보드를 이용하여 로봇을 좌우로 조금씩 이동시켜 초록색 화살표들이 밀집되도록 로봇의 정확한 초기 위치를 조정
        - 이 과정을 생략해도 되지만, Localization 정확도가 다소 떨어질 수 있음


- Web Shell #3: teleop_keyboard 실행 중지 

```
$ [Ctrl] c

```
    
    ※ 주의 사항 
        - teleop_keyboard 실행을 종료시켜야 함 
        - 그렇지 않으면 토픽 cmd_vel이 이중으로 퍼블리시되어 혼란 초래


##### 4) Navigation Goal 설정

- Graphical tools: 
    - RViz2 메뉴 > <span style="color:red">[Navigation2 Goal]</span> button 클릭 
        - 드래깅 위치: 도착 위치 설정
        - 드래깅 방향: 도착 방향 설정
    - 정리 
        - 시작지점으로부터 도착지점까지, 
        - Map 기반으로 Cost Map으로부터 Global Path를 계산하고, 
        - Robot 주변의 LiDAR 정보로부터 Local Path를 계산함 



#### 2.3 RViz Displays for Navigation 

###### 1) Global Options

- Global Options
    - Fixed Frame = map

###### 2) Global Status

- Global Status: Ok
    - Fixed Frame = OK

###### 3) Grid

- Grid = CHECK 
    - Reference Frame = <Fixed Frame>
    - Plane Cell Count = 10
    - Cell Size = 1

###### 4) Robot Model

- RobotModel = CHECK
    - Status: Ok
    - Description Source = Topic
    - Description Topic = /robot_description
    - Links
        - base_footprint 
        - base_link 
        - base_scan 
        - caster_back_link
        - imu_link 
        - wheel_left_link
        - wheel_right_link

###### 5) TF

- TF = UNCHECK
    - Status: Ok
    - Frames
        - base_footprint 
        - base_link 
        - base_scan 
        - caster_back_link
        - imu_link 
        - map
        - odom 
        - wheel_left_link
        - wheel_right_link        
    - Tree 
        - map
            - odom
                - base_footprint
                    - base_link
                        - base_scan
                        - caster_back_link
                        - imu_link
                        - wheel_left_link
                        - wheel_right_link

###### 6) LaserScan

- LaserScan = CHECK
    - Status: Ok
    - Topic = /scan

###### 7) Bumper Hit

- Bumper Hit 
    - Status: Ok
    - Topic = /mobile_base/sensors/bumper_pointcloud

###### 8) Map

- Map = CHECK
    - Status: Ok
    - Topic = /map
    - Update Topic = /map_updates
    - Color Scheme = map (: map / costmap / raw)
    - Resolution = 0.05
    - Width = 125
    - Height = 116

###### 9) Amcl Particle Swarm

- Amcl Particle Swarm
    - Status: Ok
    - Topic = /particlecloud
    - Shape = Arrow (Flat) (: Arrow (Flat) / Arrow (3D) / Axes)

###### 10) Global Planner

- Global Planner = CHECK
    - Global Costmap 
        - Status: Ok
        - Topic = /global_costmap/costmap
        - Update Topic = /global_costmap/costmap_updates
        - Color Scheme = costmap (: map / costmap / raw)
        - Resolution = 0.05
        - Width = 125
        - Height = 116
    - Path 
        - Status: Ok
        - Topic = /plan
        - Line Style = Lines
    - BoxelGrid 
        - topic = /global_costmap/voxel_marked_cloud
        - Style = Boxes
        - Size (m) = 0.05
    - Polygon = UNCHECK
        - Topic = /global_costmap/published_footprint

###### 11) Controller

- Controller = CHECK
    - Local Costmap 
        - Status: Ok
        - Topic = /local_costmap/costmap
        - Update Topic = /local_costmap/costmap_updates
        - Color Scheme = costmap (: map / costmap / raw)
        - Resolution = 0.05
        - Width = 60
        - Height = 60
    - Local Plan
         - Status: Ok
        - Topic = /local_plan
        - Line Style = Lines
    - Trajectories = UNCHECK
        - Topic = /marker
    - Polygon 
        - Status: Ok
        - Topic = /local_costmap/published_footprint
    - VoxelGrid
        - Status: Ok
        - topic = /local_costmap/voxel_marked_cloud
        - Style = Flat Squares
        - Size (m) = 0.01

###### 12) RealSense

- RealSense = UNCHECK

###### 13) MarkerArray 

- MarkerArray
    - Status: Ok
    - Topic = /waypoints





#### 2.4 Parameter Tuning 

##### 1) Costmap Parameters

    - turtlebot3_navigation2/param/burger.yaml

| 파라미터                            | 내용                  | 테스트 #1 | 테스트 #2 | 비고       |
|:---:|:---:|:---:|:---:|:---:|
| inflation_layer.inflation_radius    | 인플레이션 반경       | 1.0       | 0.2       | line #154  |
| inflation_layer.cost_scaling_factor | 코스트 스케일링 팩터  | 3.0       | 30        | line #155  |



##### 2) DWB Parameters

    - turtlebot3_navigation2/param/burger.yaml

| 파라미터            | 내용                     | 테스트 #1 | 테스트 #2 | 비고       |
|:---:|:---:|:---:|:---:|:---:|
| min_vel_x           | x방향 최소 속도          | 0.0       | -         | line #95   | 
| min_vel_y           | y방향 최소 속도          | 0.0       | -         | line #96   | 
| max_vel_x           | x방향 최대 속도          | 0.22      | -         | line #97   | 
| max_vel_y           | y방향 최대 속도          | 0.0       | -         | line #98   | 
| max_vel_theta       | 최대 회전 속도           | 1.0       | -         | line #99   | 
| min_speed_xy        | 최소 이동 스피드         | 0.0       | -         | line #100  | 
| max_speed_xy        | 최대 이동 스피드         | 0.22      | -         | line #101  | 
| min_speed_theta     | 최소 회전 스피드         | 0.0       | -         | line #102  | 
| acc_lim_x           | x방향 가속 한계          | 2.5       | -         | line #105  | 
| acc_lim_y           | y방향 가속 한계          | 0.0       | -         | line #106  | 
| acc_lim_theta       | 회전 각가속 한계         | 3.2       | -         | line #107  | 
| decel_lim_x         | x방향 감속 한계          | -2.5      | -         | line #108  | 
| decel_lim_y         | x방향 감속 한계          | 0.0       | -         | line #109  | 
| decel_lim_theta     | 회전 각감속 한계         | -3.2      | -         | line #110  | 
| sim_time            | 예상 시뮬레이션 시간     | 1.5       | 5.0       | line #114  |
| transform_tolerance | tf 메시지 대기 허용 시간 | 1.0       | -         | line #117  |
| xy_goal_tolerance   | 목표 지점 허용 거리      | 0.05      | -         | line #118  |


    - 주의 
        - DWA : ros 버전의 local_planner 방법 중의 하나, DWA(Dynamic Window Approach)
        - DWB : ros 버전의 DWA를 ros2 버전의 DWB로 업데이트


# ===

# Launch 파일 정리 

##### 1) 런치파일 turtlebot3_world.launch.py (패키지 turtlebot3_gazebo)

- 런치파일 구성

| 항목        | 내용                             | 항목        | 내용               |
|:-----------:|:--------------------------------:|:-----------:|:------------------:|
| 아규먼트    | use_sim_time                     | -           | -                  |
| 아규먼트    | x_pose                           | -           | -                  |
| 아규먼트    | y_pose                           | -           | -                  |
| 런치파일    | gzserver.launch.py               | 패키지      | gazebo_ros         |
| 런치파일    | gzclient.launch.py               | 패키지      | gazebo_ros         |
| 런치파일    | robot_state_publisher.launch.py  | 패키지      | turtlebot3_gazebo  |
| 런치파일    | spawn_turtlebot3.launch.py       | 패키지      | turtlebot3_gazebo  |


- 런치파일 소스 

```
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld

```


##### 2) 런치파일 cartographer.launch.py (패키지 turtlebot3_cartographer)

- 런치파일 구성

| 항목        | 내용                      | 항목        | 내용                     |
|:-----------:|:-------------------------:|:-----------:|:------------------------:|
| 아규먼트    | cartographer_config_dir   | -           | -                        |
| 아규먼트    | configuration_basename    | -           | -                        |
| 아규먼트    | use_sim_time              | -           | -                        |
| 익스큐터블  | cartographer_node         | 패키지      | cartographer_ros         |
| 아규먼트    | resolution                | -           | -                        |
| 아규먼트    | publish_period_sec        | -           | -                        |
| 런치파일    | occupancy_grid.launch.py  | 패키지      | turtlebot3_cartographer  |
| 익스큐터블  | rviz2                     | 패키지      | rviz2                    |



- 런치파일 소스 

```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  turtlebot3_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='turtlebot3_lds_2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
                                   'rviz', 'tb3_cartographer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])

```

 

##### 3) 런치파일 navigation2.launch.py (패키지 turtlebot3_navigation2)

- 런치파일 구성

| 항목        | 내용               | 항목        | 내용          |
|:-----------:|:------------------:|:-----------:|:-------------:|
| 아규먼트    | map                | -           | -             |
| 아규먼트    | params_file        | -           | -             |
| 아규먼트    | use_sim_time       | -           | -             |
| 런치파일    | bringup_launch.py  | 패키지      | nav2_bringup  |
| 익스큐터블  | rviz2              | 패키지      | rviz2         |




- 런치파일 소스 

```
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'map',
            'map.yaml'))

    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('turtlebot3_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])

```





```python

```
