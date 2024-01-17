# Turtlebot3_RealWorld  

- Gazebo World Simulation vs Real World Operation 
 
    - Gazebo World Simulation
        - 데스크탑(또는 노트북) with KVM(Keyboard, Video, Mouse) - 가상 로봇 제어 및 휴먼 인터페이스 

    - Real World Operation
        - SBC(RaspberryPi 4B) without KVM(Keyboard, Video, Mouse) - 로봇 제어용 
        - 데스크탑(또는 노트북) with KVM(Keyboard, Video, Mouse) - 휴먼 인터페이스용
        
- SBC와 데스크탑(노트북) 간의 무선 연결 

    - ssh 
        - 원격 시스템으로의 암호화된 연결 설정
        - 원격 시스템으로의 접속 및 원격 명령 실행에 사용
        - 먼저, 데스크탑(노트북)과 SBC를 각각 무선공유기에 연결 
    
        ```
        [SBC] $ ifconfig
                {IP주소} 확인 
        
        [데스크탑(또는 노트북)] $ ssh {사용자계정ID}@{IP주소}
        
        ```



### 0. Turtlebot3 Real World

##### 1) Real World 실행

- [데스크탑(또는 노트북)] 명령창 #1  -(ssh)-> SBC:

```
$ ssh ***@***.***.***.*** 

$ ros2 launch turtlebot3_bringup robot.launch.py

```


##### 2) teleop_keyboard 실행

- [데스크탑(또는 노트북)] 명령창 #2:

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```


##### 3) rviz2 실행

- [데스크탑(또는 노트북)] 명령창 #3:

```
$ ros2 launch turtlebot3_bringup rviz2.launch.py

```


##### 4) Frame & Odometry 조사 

###### 4-1) Odometry 

- [데스크탑(또는 노트북)] 명령창 #4:

```
$ ros2 topic echo /odom

```

###### 4-2) Frame 

- [데스크탑(또는 노트북)] 명령창 #5:

```
$ ros2 run tf2_ros tf2_echo odom base_footprint

```




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

    - 알고리즘 : Cartographer

#### 1.1 Real World 실행 

- [데스크탑(또는 노트북)] 명령창 #1  -(ssh)-> SBC:

```
$ ros2 launch turtlebot3_bringup robot.launch.py

```


#### 1.2 SLAM Node 실행 

- [데스크탑(또는 노트북)] 명령창 #2: 

```
$ ros2 launch turtlebot3_cartographer cartographer.launch.py

```

    - RViz 창 열림 
        - RViz > Display Control Panel : 
            - Add > By display type > RobotModel > OK
            - RobotModel > Description Topic = /robot_description
            - TF = UNCHECK 


#### 1.3 teleop_keyboard 실행

- [데스크탑(또는 노트북)] 명령창 #3: 

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

    - 정리 
        - 로봇을 천천히 움직이면서, 주변 환경을 남김없이 탐색하여 맵 완성 


#### 1.4 Map 저장

- [데스크탑(또는 노트북)] 명령창 #4: 

```
$ cd ~
$ ros2 run nav2_map_server map_saver_cli -f ~/map1

$ ls map1.*
map1.pgm  map1.yaml

```



### 2 Navigation 

#### 2.1 Real World 실행 

- [데스크탑(또는 노트북)] 명령창 #1  -(ssh)-> SBC:

```
$ ros2 launch turtlebot3_bringup robot.launch.py

```


#### 2.2 Navigation Node 실행 

##### 1) Navigation Node 실행 

- [데스크탑(또는 노트북)] 명령창 #2: 

```
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map1.yaml

```


##### 2) Initial Pose 설정

- [데스크탑(또는 노트북)] RViz2 창: 
    - RViz2 메뉴 > <span style="color:red">[2D Pose Estimate]</span> button 클릭 
        - 드래깅 위치: 로봇 위치 설정
        - 드래깅 방향: 로봇 방향 설정 
        
    - RViz > Display Control Panel :
        - RobotModel = CHECK
        - TF = UNCHECK


##### 3) Localization 정확도 향상 (: 생략 가능)

- [데스크탑(또는 노트북)] 명령창 #3: 

```
$ ros2 run turtlebot3_teleop teleop_keyboard

```

    ※ 주의 사항  
        - 키보드를 이용하여 로봇을 좌우로 조금씩 이동시켜 초록색 화살표들이 밀집되도록 로봇의 정확한 초기 위치를 조정
        - 이 과정을 생략해도 되지만, Localization 정확도가 다소 떨어질 수 있음


- [데스크탑(또는 노트북)] 명령창 #3:  teleop_keyboard 실행 중지 

```
$ [Ctrl] c

```
    
    ※ 주의 사항 
        - teleop_keyboard 실행을 종료시켜야 함 
        - 그렇지 않으면 토픽 cmd_vel이 이중으로 퍼블리시되어 혼란 초래


##### 4) Navigation Goal 설정

- [데스크탑(또는 노트북)] RViz2 창: 
    - RViz2 메뉴 > <span style="color:red">[Navigation2 Goal]</span> button 클릭 
        - 드래깅 위치: 도착 위치 설정
        - 드래깅 방향: 도착 방향 설정 

    - 정리 
        - 시작지점으로부터 도착지점까지, 
        - Map 기반으로 Cost Map으로부터 Global Path를 계산하고, 
        - Robot 주변의 LiDAR 정보로부터 Local Path를 계산함 




# ===

### Launch 파일 정리 

##### 1) 런치파일 robot.launch.py (패키지 turtlebot3_bringup) 

- 런치파일 구성 

| 항목        | 내용                                 | 항목        | 내용               |
|:-----------:|:------------------------------------:|:-----------:|:------------------:|
| 아규먼트    | use_sim_time                         | -           | -                  |
| 아규먼트    | usb_port                             | -           | -                  |
| 아규먼트    | tb3_param_dir                        | -           | -                  |
| 런치파일    | turtlebot3_state_publisher.launch.py | 패키지      | turtlebot3_bringup |
| 런치파일    | ld08.launch.py                       | 패키지      | ld08_drive         |
| 익스큐터블  | turtlebot3_ros                       | 패키지      | turtlebot3_node    |


- 런치파일 소스

```
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    LDS_MODEL = os.environ['LDS_MODEL']
    LDS_LAUNCH_FILE = '/hlds_laser.launch.py'

    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    if LDS_MODEL == 'LDS-01':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))
    elif LDS_MODEL == 'LDS-02':
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('ld08_driver'), 'launch'))
        LDS_LAUNCH_FILE = '/ld08.launch.py'
    else:
        lidar_pkg_dir = LaunchConfiguration(
            'lidar_pkg_dir',
            default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'tb3_param_dir',
            default_value=tb3_param_dir,
            description='Full path to turtlebot3 parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LDS_LAUNCH_FILE]),
            launch_arguments={'port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[tb3_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])

```



```python

```
