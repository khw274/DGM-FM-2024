- 메시지(토픽, 서비스, 액션) 인터페이스 

| 메시지인터페이스 | 메시지정의패키지/메시지유형폴더/데이터구조 |
|:-----------------|:------------------------------------------:|
|       토픽메시지 |            메시지정의패키지/msg/데이터구조 |
|     서비스메시지 |            메시지정의패키지/srv/데이터구조 |
|       액션메시지 |         메시지정의패키지/action/데이터구조 |
    


# 7 turtlesim 

- 패키지         : turtlesim
- 실행파일 #1    : turtlesim_node 
- 실행파일 #2    : turtle_teleop_key 
- 실행파일 #3    : mimic 
- 실행파일 #4    : draw_square 


### 7.1 turtlesim 설치 확인

- turtlesim 

    - ROS 프로그램이 성공적으로 잘 설치되었는지 확인하기 위한 예제 프로그램
    - 일반적으로 ROS 설치 시 함께 설치되지만, 설치되지 않았다면 별도로 설치하여야 함


- turtlesim 설치 확인 

```
$ cd /opt/ros/humble/share
$ ls | grep turtlesim
turtlesim

```


&#8251; Rosject "JYSAH's Turtlebot3 Humble" : turtlesim 이미 설치되어 있음 
    
    
&#8251; turtlesim (ROS humble 버전) 설치 방법 <span style="color:red">(&#8251; 이미 설치되어 있으니, 설치하지 말 것!) </span>


```
$ sudo apt install ros-humble-turtlesim

```



### 7.2 turtlesim 실행
- ros2 run turtlesim turtlesim_node
- ros2 run turtlesim turtle_teleop_key

- Web Shell #1 : turtlesim_node 실행

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : turtle_teleop_key 실행

```
$ ros2 run turtlesim turtle_teleop_key 

```
    - 키보드 상/하/좌/우 화살표로 속도/방향 제어  
    

### 7.3 turtlesim 노드 정보 

#### 7.3.1 패키지 내 익스큐터블 정보 vs 실행 중인 노드 정보

##### 1) pkg executables 패키지명: 패키지 내 익스큐터블 정보 
- ros2 pkg executables 패키지명

- Web Shell #3 :

```
$ ros2 pkg executables turtlesim
turtlesim draw_square
turtlesim mimic
turtlesim turtle_teleop_key
turtlesim turtlesim_node

```

| executable        | node name      | description                  |
|:---|---:|:---:|
| turtlesim_node    | /turtlesim     | 거북이 제어 환경 생성        |
| turtle_teleop_key | /teleop_turtle | 거북이 키보드 제어           |
| draw_square       | /draw_square   | 거북이 사각형 모양 이동 액션 |
| mimic             | /turtle_mimic  | 복수의 거북이 이동 복사      |


##### 2) node list: 실행 중인 노드 정보 
- ros2 node list

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #3 :

```
$ ros2 node list
/turtlesim

```
    - executable: turtlesim_node <===> node: /turtlesim 

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 :

```
$ ros2 node list
/teleop_turtle
/turtlesim

```
    - executable: turtle_teleop_key <===> node: /teleop_turtle 
    

|  # |익스큐터블(executable) | 노드명(node name) |
|:---|:---|:---|
| 1 | turtlesim_node | turtlesim |
| 2 | turtle_teleop_key | teleop_turtle |


#### 7.3.2 노드 다중 실행 

##### 1) 단순 중복 실행 : 동일한 노드명 및 토픽명이 중복 생성되므로 비추천

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 :

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #4 :

```
$ ros2 node list
/teleop_turtle
/turtlesim
/turtlesim

$ ros2 topic list
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

```

&#8251; 주요 정리 
    - Graphical Tools 창에 두 개의 Turtlesim 운영환경이 열리고, 각각에 동일한 이름의 거북이 turtle1이 생성됨 
    
    - Web Shell #1과 Web Shell #3에서 각각 실행 중인 두 노드명이 동일하고 토픽명도 동일하므로 구분할 수 없음 
    
    - Web Shell #2에서 키보드 제어를 하면 두 거북이가 동일한 토픽 /turtle1/cmd_vel를 Subscribe하여 동일하게 움직임
    
    - 하지만 두 거북이가 동일한 토픽 /turtle1/pose를 각각 Publish하므로 두 위치 정보가 뒤섞여 혼란을 초래함
    
    - 만일 Web Shell #1 거북이를 먼저 생성하여 키보드 제어를 통해 이동시킨 후, Web Shell #3 거북이를 나중에 생성하면, 
      두 거북이가 동일한 토픽 /turtle1/pose으로 서로 다른 자신의 위치를 Publish 하므로 토픽에 의한 위치정보가 뒤섞임 
      
    - 동일한 두 개의 /turtlesim 노드를 동시에 실행하는 것은 추천하지 않음 



##### 2) 노드명 변경하여 중복 실행: 노드명은 구별되지만, 동일한 토픽명이 중복 생성되므로 비추천

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : 노드명을 new_trutlesim으로 변경하여 실행

```
$ ros2 run turtlesim turtlesim_node --ros-args --remap __node:=new_turtlesim

```

- Web Shell #4 :

```
$ ros2 node list
/new_turtlesim
/teleop_turtle
/turtlesim

$ ros2 topic list
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

```

&#8251; 주요 정리 
    - Web Shell #3에서 실행 중인 노드는 노드명을 new_turtle로 변경하였음 

    - 노드명만 변경되었고, 토픽 /turtle1/cmd_vel은 동일
    
    - 노드명은 구별되지만, 토픽 /turtle1/cmd_vel과 /turtle1/pose은 서로 동일함

    - Web Shell #2에서 키보드 제어를 하면 두 거북이가 동일한 토픽 /turtle1/cmd_vel를 Subscribe하여 동일하게 움직임
    
    - 하지만 두 거북이가 동일한 토픽 /turtle1/pose를 각각 Publish하므로 두 위치 정보가 뒤섞여 혼란을 초래함




##### 3) namespace 지정하여 중복 실행 : 추천


- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/ns1

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/ns2

```

- Web Shell #3 : 

```
$ ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/ns1

```

- Web Shell #4 : 

```
$ ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/ns2

```


- Web Shell #5 :

```
$ ros2 node list
/ns1/teleop_turtle
/ns1/turtlesim
/ns2/teleop_turtle
/ns2/turtlesim

$ ros2 topic list
/ns1/turtle1/cmd_vel
/ns1/turtle1/color_sensor
/ns1/turtle1/pose
/ns2/turtle1/cmd_vel
/ns2/turtle1/color_sensor
/ns2/turtle1/pose

```

&#8251; 주요 정리 
    - Web Shell #1 & #3 에서 namespace를 /ns1 로 지정  
    
    - Web Shell #2 & #4 에서 namespace를 /ns2 로 지정 

    - 노드명 & 토픽명의 원래 이름 앞에 namespace가 추가되므로, 노드명과 토픽명을 구별할 수 있음 
    
    - 토픽명 /ns1/turtle1/cmd_vel로 거북이 /ns1/turtle1을 개별 제어할 수 있음
      토픽명 /ns1/turtle1/pose로 거북이 현재 Pose를 알 수 있음

    - 토픽명 /ns2/turtle1/cmd_vel로 거북이 /ns2/turtle1을 개별 제어할 수 있음
      토픽명 /ns2/turtle1/pose로 거북이 현재 Pose를 알 수 있음



##### 4) 노드 mimic 이용한 중복 실행 


- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/ns1

```

- Web Shell #2 : node를 new_trutle로 변경하여 실행

```
$ ros2 run turtlesim turtlesim_node --ros-args --remap __ns:=/ns2

```

- Web Shell #3 : 

```
$ ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:=/ns1

```

- Web Shell #4 : 

```
$ ros2 run turtlesim mimic --ros-args --remap /input/pose:=/ns1/turtle1/pose --remap /output/cmd_vel:=/ns2/turtle1/cmd_vel

```


- Web Shell #5 :

```
$ ros2 node list
/ns1/teleop_turtle
/ns1/turtlesim
/ns2/turtlesim
/turtle_mimic

$ ros2 topic list
/ns1/turtle1/cmd_vel
/ns1/turtle1/color_sensor
/ns1/turtle1/pose
/ns2/turtle1/cmd_vel
/ns2/turtle1/color_sensor
/ns2/turtle1/pose

```

&#8251; 주요 정리 
    - Web Shell #1 & #3 에서 namespace를 /ns1 로 지정  
    
    - Web Shell #2 에서 namespace를 /ns2 로 지정 

    - Web Shell #3에서 키보드 제어를 하면 토픽 /ns1/turtle1/cmd_vel 를 Publish 하여, 거북이 /ns1/turtle1 이 이동
    
    - Web Shell #4의 노드 mimic에서 /input/pose를 /ns1/turtle1/pose로, /output/cmd_vel를 /ns2/turtle1/cmd_vel로 각각 remap 함
    
    1) 노드 /ns1/turtlesim: 
          Subscribe 토픽 /ns1/turtle1/cmd_vel -> 거북이 /ns1/turtle1을 이동
          Publish   토픽 /ns1/turtle1/pose    (데이터구조 Pose: x, y, theta, linear_velocity, angular_velocity)
          
    2) 노드 /turtle_mimic: 
          Subscribe 토픽 /ns1/turtle1/pose (remappimg /input/pose) -> /ns1/turtle1의 linear_velocity와 angular_velocity 데이터 확보 
          Publish   토픽 /ns2/turtle1/cmd_vel (remapping /output/cmd_vel) 
    
    3) 노드 /ns2/turtlesim: 
          Subscribe 토픽 /ns2/turtle1/cmd_vel -> 거북이 /ns2/turtle1을 이동

    - 거북이 /ns1/turtle1의 키보드 제어를 복사하여, 거북이 /ns2/turtle1 이동을 동일한 키보드 입력으로 함께 제어함




#### 7.3.3 노드 info 정보 

```
$ ros2 node info 노드명 

```

###### 1) node info /turtlesim: 노드 /turtlesim 정보 
- ros2 node info 노드명

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 :

```
$ ros2 node info /turtlesim
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:

```

&#8251; 주요 정리 
    - 토픽: /turtle1/cmd_vel 
        - 네임스페이스명: /turtle1
        - 토픽명: /cmd_vel
    - 데이터형: geometry_msgs/msg/Twist
    - 서브스크라이버노드: /turtlesim 


###### 2) node info /teleop_turtle: 노드명 /teleop_turtle 정보 
- ros2 node info 노드명

- Web Shell #4 :

```
$ ros2 node info /teleop_turtle
/teleop_turtle
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Service Servers:
    /teleop_turtle/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /teleop_turtle/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /teleop_turtle/get_parameters: rcl_interfaces/srv/GetParameters
    /teleop_turtle/list_parameters: rcl_interfaces/srv/ListParameters
    /teleop_turtle/set_parameters: rcl_interfaces/srv/SetParameters
    /teleop_turtle/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute

```

&#8251; 주요 정리 
    - 토픽: /turtle1/cmd_vel 
        - 네임스페이스명: /turtle1
        - 토픽명: /cmd_vel
    - 데이터형: geometry_msgs/msg/Twist
    - 퍼블리셔노드: /teleop_turtle 



### 7.4 Turtlesim 토픽

#### 7.4.1 토픽 전체 리스트 정보

##### 1) topic list -t: 실행 중인 노드들의 토픽 전체 리스트 출력 
- ros2 topic list -t

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 :

```
$ ros2 topic list -t
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
/turtle1/color_sensor [turtlesim/msg/Color]
/turtle1/pose [turtlesim/msg/Pose]

```

&#8251; 주요 정리 
    - 토픽: /turtle1/cmd_vel 
        - 네임스페이스명: /turtle1
        - 토픽명: /cmd_vel
    - 데이터형: geometry_msgs/msg/Twist


#### 7.4.2 토픽 /turtle1/cmd_vel info 정보

##### 1) topic info 토픽명: 토픽 info 확인
- ros2 topic info 토픽명


- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : info 토픽명

```
$ ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1

```


#### 7.4.3 토픽 /turtle1/cmd_vel echo 정보

##### 1) topic echo 토픽명: 토픽 내용 확인
- ros2 topic echo 토픽명

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : echo 토픽명 

    - 아래 명령을 실행한 후 
    - 노드명 /teleop_turtle을 실행 중인 Web Shell #2 명령창에서 
    - 위쪽 화살표를 눌러 거북이를 이동시키고 
    - 좌측 화살표를 눌러 거북이를 회전시킴

```
$ ros2 topic echo /turtle1/cmd_vel
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0
---

```

&#8251; 주요 정리 
    - linear: 이동 속도 (x축은 거북이 정면 방향)
    - angular: 회전속도 (z축은 거북이 평면의 외부 법선 방향)
    - (SI Units) 


#### 7.4.4 토픽 /turtle1/cmd_vel bw 정보

##### 1) topic bw 토픽명: 토픽 메시지 대역폭(초당 주고받는 토픽 메시지 크기) 확인
- ros2 topic bw 토픽명

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : bw 토픽명 

    - 아래 명령을 실행한 후 
    - 노드명 /teleop_turtle을 실행 중인 Web Shell #2 명령창에서 
    - 위쪽 화살표를 3회 연속 눌러 거북이를 이동시킴

```
$ ros2 topic bw /turtle1/cmd_vel
Subscribed to [/turtle1/cmd_vel]
284 B/s from 2 messages
        Message size mean: 52 B min: 52 B max: 52 B
114 B/s from 3 messages
        Message size mean: 52 B min: 52 B max: 52 B
66 B/s from 3 messages
        Message size mean: 52 B min: 52 B max: 52 B
46 B/s from 3 messages
        Message size mean: 52 B min: 52 B max: 52 B
36 B/s from 3 messages
        Message size mean: 52 B min: 52 B max: 52 B
29 B/s from 3 messages
        Message size mean: 52 B min: 52 B max: 52 B
25 B/s from 3 messages
        Message size mean: 52 B min: 52 B max: 52 B

```

&#8251; 주요 정리 
    - bw(band width): 초당 송수신 받는 토픽 메시지 크기 
    - 2개 토픽을 송수신하는데 약 284 B/s 대역폭을 가짐
    - 토픽을 보내지 않으면 값이 계속 감소됨


#### 7.4.5 토픽 /turtle1/cmd_vel hz 정보

##### 1) topic hz 토픽명: 토픽 메시지 주파수(초당 토픽 메시지 발행 횟수) 확인
- ros2 topic hz 토픽명

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : hz 토픽명 

    - 아래 명령을 실행한 후 
    - 노드명 /teleop_turtle을 실행 중인 Web Shell #2 명령창에서 
    - 위쪽 화살표를 10회 이상 연속 눌러 거북이를 이동시킴

```
$ ros2 topic hz /turtle1/cmd_vel
average rate: 5.134
        min: 0.188s max: 0.203s std dev: 0.00475s window: 7

```

&#8251; 주요 정리 
    - hz: 초당 송수신 받는 토픽 메시지 횟수 
    - 초당 5.134회 속도로 토픽 메시지 송수신 (메시지 하나 당 0.188-0.203초 소요)


#### 7.4.6 토픽 /turtle1/cmd_vel delay 정보 

##### 1) topic delay 토픽명: 토픽의 시간 지연(latency) 체크 
- ros2 topic delay 토픽명

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : delay 토픽명 

```
$ ros2 topic delay 토픽명 
average rate: 4.836

```

&#8251; 주요 정리 
    - delay: 토픽의 시간 지연(latency) 체크 
    - 시간 지연을 체크하려면. 토픽 데이터형에 header가 정의되어야만 가능함 
    - header 내의 stamp 값을 이용하여 토픽 발행시간을 알 수 있음
    - 토픽 /cmd_vel의 데이터형 Twist는 header가 없어서 delay를 이용할 수 없음


#### 7.4.7 토픽 /turtle1/cmd_vel 퍼블리시


##### 1) 옵션 --once: 토픽 1회 퍼블리시
- ros2 topic pub --once 토픽명 토픽데이터형 "{토픽데이터값}"

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : pub 토픽명 

    - 노드명까지 입력한 후, [tab][Tab]을 누르면 데이터형과 데이터값 형식이 자동 출력됨 
    - 원하는 값을 입력하여 실행함
    - theconstruct 시스템에서, [Tab][Tab]이 적용되지 않는 사례있음. 이 경우, 형식에 맞게 직접 입력해야 함

```
$ ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist [Tab][Tab]
$ ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

```


&#8251; 주요 정리 
    - geometry_msgs/msg/Twist 데이터 입력
        - "{linear: {x: 00., y:0.0, z: 0.0}, angular: {x: 00., y:0.0, z: 0.0}}"



##### 2) 옵션 --rate 주파수: 토픽 연속(Hz 수) 퍼블리시
- ros2 topic pub --rate 1 토픽명 토픽데이터형 "{토픽데이터값}"

- Web Shell #3 : pub 토픽명 

    - 노드명까지 입력한 후, [tab][Tab]을 누르면 데이터형과 데이터값 형식이 자동 출력됨 
    - 원하는 값을 입력하여 실행함
    - theconstruct 시스템에서, [Tab][Tab]이 적용되지 않는 사례있음. 이 경우, 형식에 맞게 직접 입력해야 함

```
$ ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist [Tab][Tab]
$ ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

```

- Web Shell #4 : echo 토픽명 

```
$ ros2 topic echo /turtle1/cmd_vel
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8
---
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8
---

```


&#8251; 주요 정리 
    - 옵션 
        - --once: 1회만 퍼블리시
        - --rate 1: 1Hz 속도로 계속 퍼블리시 




#### 7.4.8 bag 파일 

- 퍼블리시 topic 저장 

##### 1) bag record: bag 파일 저장
- ros2 bag record -a -o 파일.bag 
- ros2 bag record -o 파일.bag 토픽명1 토픽명2 ......


- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : 토픽 /turtle1/cmd_vel 퍼블리시

```
$ ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

```


- Web Shell #4 : bag 파일 저장

```
$ cd ~
$ mkdir bag
$ cd ~/bag
$ ros2 bag record -o test.bag /turtle1/cmd_vel
$ (1분 가량 기다린 후, [Ctrl]c를 눌러 강제 종료)

```

- Web Shell #3 : topic pub 강제 종료 

```
$ [Ctrl]c를 눌러 강제 종료)

```


##### 2) bag info: bag 파일 정보
- ros2 bag info 파일.bag 


- Web Shell #3 : bag 파일 정보 

```
$ cd ~/bag
$ ros2 bag info test.bag
Files:             test.bag_0.db3
Bag size:          17.1 KiB
Storage id:        sqlite3
Duration:          38.999s
Start:             Oct 15 2023 07:52:40.860 (1697356360.860)
End:               Oct 15 2023 07:53:19.860 (1697356399.860)
Messages:          40
Topic information: Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 40 | Serialization Format: cdr

```



##### 3) bag play: bag 파일 재생
- ros2 bag play 파일.bag 


- Web Shell #1 : turtlesim_node 실행

```
$ ros2 run turtlesim turtlesim_node

```


- Web Shell #2 : bag 파일 재생

```
$ cd ~/bag
$ ros2 bag play test.bag
[INFO] [1697356821.093377700] [rosbag2_storage]: Opened database 'test.bag/test.bag_0.db3' for READ_ONLY.

```



#### 7.4.9 메시지 인터페이스, msg

##### 1) interface show 토픽메시지 (메시지정의패키지/msg/토픽데이구조)
- ros2 interface show 메시지정의패키지/msg/토픽데이구조


- Web Shell #1 : 

```
$ ros2 interface show geometry_msgs/msg/Twist
Vector3  linear
Vector3  angular

```

```
$ ros2 interface show geometry_msgs/msg/Vector3
float64 x
float64 y
float64 z

```

##### 2) interface list: msg/service/action 메시지  확인
- ros2 interface list

- Web Shell #1 : 

```
$ ros2 interface list
Messages:
    ...
    geometry_msgs/msg/Twist
    ...
Services:
    ...
Actions:
    ...

```


##### 3) interface packages: 메시지정의패키지 확인
- ros2 interface packages


- Web Shell #1 : 

```
$ ros2 interface packages
...
geometry_msgs
...
my_msgs
...
nav_msgs
...
sensor_msgs
...
std_msgs
std_srvs
...
turtlesim
...

```



##### 4) interface package 메시지정의패키지: 
- ros2 interface package 메시지정의패키지

- Web Shell #1 : 

```
$ ros2 interface package turtlesim
turtlesim/srv/SetPen
turtlesim/srv/Spawn
turtlesim/msg/Color
turtlesim/msg/Pose
turtlesim/srv/TeleportRelative
turtlesim/action/RotateAbsolute
turtlesim/srv/TeleportAbsolute
turtlesim/srv/Kill

```

```
$ ros2 interface package my_msgs
my_msgs/msg/TwoInts
my_msgs/action/Fibonacci
my_msgs/srv/AddTwoInts

```

##### 5) proto 토픽/서비스/액션 메시지
- ros2 interface proto 메시지정의패키지/msg/데이터구조

- Web Shell #1 : 

```
$ ros2 interface proto geometry_msgs/msg/Twist
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
"

```



### 7.5 Turtlesim 서비스

#### 7.5.1 servise list -t: 서비스 및 서비스메시지 리스트 정보

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 :

```
$ ros2 service list -t
/clear [std_srvs/srv/Empty]
/kill [turtlesim/srv/Kill]
/reset [std_srvs/srv/Empty]
/spawn [turtlesim/srv/Spawn]
/teleop_turtle/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/teleop_turtle/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/teleop_turtle/get_parameters [rcl_interfaces/srv/GetParameters]
/teleop_turtle/list_parameters [rcl_interfaces/srv/ListParameters]
/teleop_turtle/set_parameters [rcl_interfaces/srv/SetParameters]
/teleop_turtle/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]
/turtle1/set_pen [turtlesim/srv/SetPen]
/turtle1/teleport_absolute [turtlesim/srv/TeleportAbsolute]
/turtle1/teleport_relative [turtlesim/srv/TeleportRelative]
/turtlesim/describe_parameters [rcl_interfaces/srv/DescribeParameters]
/turtlesim/get_parameter_types [rcl_interfaces/srv/GetParameterTypes]
/turtlesim/get_parameters [rcl_interfaces/srv/GetParameters]
/turtlesim/list_parameters [rcl_interfaces/srv/ListParameters]
/turtlesim/set_parameters [rcl_interfaces/srv/SetParameters]
/turtlesim/set_parameters_atomically [rcl_interfaces/srv/SetParametersAtomically]

```

&#8251; 주요 정리 
    - 서비스: /turtle1/set_pen 
        - 네임스페이스명: /turtle1
        - 서비스명: /set_pen
    - 데이터형: turtlesim/srv/SetPen


#### 7.5.2 service type: 서비스 type 서비스명 정보 

- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

- Web Shell #3 : type 서비스명

```
$ ros2 service type /clear
std_srvs/srv/Empty

$ ros2 service type /kill
turtlesim/srv/Kill

$ ros2 service type /reset
std_srvs/srv/Empty

$ ros2 service type /turtle1/set_pen
turtlesim/srv/SetPen

$ ros2 service type /spawn
turtlesim/srv/Spawn

```


#### 7.5.3 service call: 서비스 call 요청 


- Web Shell #1 : 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key 

```

    - 키보드 상/하/좌/우 화살표를 눌러 거북이 이동시킴


##### 1) 서비스 /clear std_srvs/srv/Empty

- Web Shell #3 : call 서비스명 서비스데이터형 데이터값

```
$ ros2 service call /clear std_srvs/srv/Empty [Tab][Tab]
$ ros2 service call /clear std_srvs/srv/Empty
waiting for service to become available...
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()

```
    - 이전 지나온 경로가 지워짐


##### 2) 서비스 /kill turtlesim/srv/Kill

- Web Shell #3 : call 서비스명 서비스데이터형 데이터값

```
$ ros2 service call /kill turtlesim/srv/Kill [Tab][Tab]
$ ros2 service call /kill turtlesim/srv/Kill "name: 'turtle1'"
waiting for service to become available...
requester: making request: turtlesim.srv.Kill_Request(name='turtle1')

response:
turtlesim.srv.Kill_Response()

```
    - 거북이가 사라짐


##### 3) 서비스 /reset std_srvs/srv/Empty

- Web Shell #3 : call 서비스명 서비스데이터형 데이터값

```
$ ros2 service call /reset std_srvs/srv/Empty
waiting for service to become available...
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()

```
    - 처음 상태로 reset 됨


##### 4) 서비스 /turtle1/set_pen turtlesim/srv/SetPen

- Web Shell #3 : call 서비스명 서비스데이터형 데이터값

```
$ ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 255, b: 255, width: 10}"

```
    - 거북이 경로 색깔 변경 (Web Shell #2: turtle_teleop_key에서 거북이 이동시킴)


##### 5) 서비스 /spawn turtlesim/srv/Spawn

- Web Shell #3 : call 서비스명 서비스데이터형 데이터값

```
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.5, y: 9, theta: 1.57, name: 'turtle2'}"
waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=5.5, y=9.0, theta=1.57, name='turtle2')

response:
turtlesim.srv.Spawn_Response(name='turtle2')

```
    - 지정 위치 및 자세에 해당하는 거북이 추가




#### 7.5.4 interface show: 서비스데이터형 확인 

- Web Shell #1 :

```
$ ros2 interface show turtlesim/srv/Spawn.srv
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name

```



### 7.6 Turtlesim 액션

#### 7.6.1 액션 서버 & 클라이언트 

- 액션명 /turtle1/rotate_absolute

    - 액션데이터형 turtlesim/action/RotateAbsolute


##### 1) 액션 서버 

- Web Shell #1 :

```
$ ros2 node info /turtlesim
/turtlesim
  ...
  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:

```


##### 2) 액션 클라이언트 

- Web Shell #1 :

```
$ ros2 node info /teleop_turtle
/teleop_turtle
  ...
  Action Servers:

  Action Clients:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute

```

    - 액션 /turtle1/rotate_absolute 
        - 액션 서버 /turtlesim
        - 액션 클라이언트 /teleop_turtle


#### 7.6.2 액션 정보 

##### 1) action info 
- ros2 action info 액션명

- Web Shell #1 :

```
$ ros2 action info /turtle1/rotate_absolute
Action: /turtle1/rotate_absolute
Action clients: 1
    /teleop_turtle
Action servers: 1
    /turtlesim

```


##### 2) action send_goal: 액션서버에 send_goal 보내기 
- ros2 action send_goal 액션명 액션데이터형 "{데이터값 딕셔너리}"

- Web Shell #1 : 

```
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute [Tab][Tab]
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5708}"
Waiting for an action server to become available...
Sending goal:
     theta: 1.5708

Goal accepted with ID: 760369ad7f7f4cc8af2618fa688b65f6

Result:
    delta: -0.16000032424926758

Goal finished with status: SUCCEEDED

```

##### 3) action send_goal --feedback: 액션서버에 feedback을 동반한 send_goal 보내기 
- ros2 action send_goal 액션명 액션데이터형 "{데이터값 딕셔너리}" --feedback


```
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5708}" --feedback
Waiting for an action server to become available...
Sending goal:
     theta: 1.5708

Feedback:
    remaining: 1.554800033569336

Goal accepted with ID: 21a4fdd053f84ea1935a9b3c582c9ac6

Feedback:
    remaining: 1.5388000011444092

Feedback:
    remaining: 1.5227999687194824

Feedback:
    remaining: 1.5067999362945557
...
Feedback:
    remaining: 0.05079972743988037

Feedback:
    remaining: 0.03479969501495361

Feedback:
    remaining: 0.018799662590026855

Result:
    delta: -1.536000370979309

Goal finished with status: SUCCEEDED

```


##### 4) interface show
- ros2 interface show 경로포함액션데이터형.action

- Web Shell #1 :

```
$ ros2 interface show turtlesim/action/RotateAbsolute.action
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining

```




### 7.7 Turtlesim 토픽/서비스/액션 사용 예제 


- Web Shell #1 : 노드 /turtlesim 실행 (패키지 turtlesim, 익스큐터블 turtlesim_node)

```
$ ros2 run turtlesim turtlesim_node

```


- Web Shell #2 : 노드 /turtlesim info 확인

```
$ ros2 node info /turtlesim
/turtlesim
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    /turtle1/set_pen: turtlesim/srv/SetPen
    /turtle1/teleport_absolute: turtlesim/srv/TeleportAbsolute
    /turtle1/teleport_relative: turtlesim/srv/TeleportRelative
    /turtlesim/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /turtlesim/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /turtlesim/get_parameters: rcl_interfaces/srv/GetParameters
    /turtlesim/list_parameters: rcl_interfaces/srv/ListParameters
    /turtlesim/set_parameters: rcl_interfaces/srv/SetParameters
    /turtlesim/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:

```


- Topic Subcriber / Service Server / Action Server


|  interface | topic, service, action name | data-type |
|:---|:---|:---|
| topic   | /parameter_events          | rcl_interfaces/msg/ParameterEvent |
| topic   | /turtle1/cmd_vel           | geometry_msgs/msg/Twist |
| service | /clear                     | std_srvs/srv/Empty |
| service | /kill                      | turtlesim/srv/Kill |
| service | /reset                     | std_srvs/srv/Empty |
| service | /spawn                     | turtlesim/srv/Spawn |
| service | /turtle1/set_pen           | turtlesim/srv/SetPen |
| service | /turtle1/teleport_absolute | turtlesim/srv/TeleportAbsolute |
| service | /turtle1/teleport_relative | turtlesim/srv/TeleportRelative |
| action  | /turtle1/rotate_absolute   | turtlesim/action/RotateAbsolute |



#### 7.7.1 Turtlesim 토픽 사용 예제

- Topic Subcriber 

| interface |               topic name |                         data-type |
|:----------|:-------------------------|:----------------------------------|
| topic     | /parameter_events        | rcl_interfaces/msg/ParameterEvent |
| topic     | /turtle1/cmd_vel         | geometry_msgs/msg/Twist           |


##### 1) 토픽명 /parameter_events (토픽데이터형 rcl_interfaces/msg/ParameterEvent) 

&#8251; 파라미터 이벤트 관리를 위한 토픽으로, 거북이 직접 제어와 관련이 약함. 
        후에 추가로 다루기로 함



##### 2) 토픽명 /turtle1/cmd_vel (토픽데이터형 geometry_msgs/msg/Twist) 

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```


- Web Shell #2 : interface show 토픽데이터형(geometry_msgs/msg/Twist)

```
$ ros2 interface show geometry_msgs/msg/Twist
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
Vector3  angular



$ ros2 interface show geometry_msgs/msg/Vector3
 This represents a vector in free space.

# This is semantically different than a point.
# A vector is always anchored at the origin.
# When a transform is applied to a vector, only the rotational component is applied.

float64 x
float64 y
float64 z
```
    - 정리
        - 토픽 데이터  : linear - 거북이 상대 이동 거리 
                              x - 전진방향 상대 이동 거리
                              y - 0
                              z - 0
                         angular - 거북이 상대 회전 방향
                              x - 0
                              y - 0
                              z - 거북이 반시계방향 상대 회전 각도 (in Rad)


- Web Shell #3 : topic pub --once 

```
$ ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5708}}"
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.5708))

```
    - 정리
        - Graphical Tools 창에서 진행거리 2, 회전방향 90도(1.5708(in Rad)) 동시에 한번만 실행하여 1/4 원을 그림


- Web Shell #4 : topic pub --rate 1 

```
$ ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.5708}}"
publisher: beginning loop
publishing #1: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.5708))

publishing #2: geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=2.0, y=0.0, z=0.0), angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=1.5708))

......

```
    - 정리
        - Graphical Tools 창에서 진행거리 2, 회전방향 90도(1.5708(in Rad)) 동시에 1Hz 주기로 계속 실행하여 무한히 원을 그림





&#8251; 토픽 리스트가 정상적으로 나타나지 않는 경우 

```
$ ros2 daemon stop
$ ros2 daemon start

$ ros2 topic list -t

```










#### 7.7.2 Turtlesim 서비스 사용 예제

- Service Server 

| interface |               service name |                         data-type |
|:----------|:---------------------------|:----------------------------------|
| service   | /clear                     | std_srvs/srv/Empty                |
| service   | /kill                      | turtlesim/srv/Kill                |
| service   | /reset                     | std_srvs/srv/Empty                |
| service   | /spawn                     | turtlesim/srv/Spawn               |
| service   | /turtle1/set_pen           | turtlesim/srv/SetPen              |
| service   | /turtle1/teleport_absolute | turtlesim/srv/TeleportAbsolute    |
| service   | /turtle1/teleport_relative | turtlesim/srv/TeleportRelative    |


##### 1) 서비스명 /clear (서비스데이터형 std_srvs/srv/Empty)

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```
    - 정리
        - 패키지     : turtlesim
        - 익스큐터블 : turtlesim_node 
        - 노드       : /turtlesim 


- Web Shell #2 : 노드 /teleop_turtle 실행

```
$ ros2 run turtlesim turtle_teleop_key

```
    - 정리
        - 패키지     : turtlesim
        - 익스큐터블 : turtle_teleop_key 
        - 노드       : /teleop_turtle 
        - 키보드 상/하/좌/우 화살표를 눌러 거북이를 이동 및 회전시킴 
        - 거북이 이동에 따은 경로가 표시되도록 함


- Web Shell #3 : interface show 서비스데이터형(std_srvs/srv/Empty)

```
$ ros2 interface show std_srvs/srv/Empty
---

```
    - 정리
        - 서비스 Request 데이터  : (없음)
        - 서비스 Response 데이터 : (없음)


- Web Shell #4 : service call 

```
$ ros2 service call /clear std_srvs/srv/Empty
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()

```
    - 정리
        - 노드 /teleop_turtle 실행 창에서 거북이 이동에 따른 경로가 지워짐


##### 2) 서비스명 /kill (서비스데이터형 turtlesim/srv/Kill)

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```


- Web Shell #2 : interface show 서비스데이터형(turtlesim/srv/Kill)

```
$ ros2 interface show turtlesim/srv/Kill
string name
---

```
    - 정리
        - 서비스 Request 데이터  : name - 실행 중인 거북이 중에 kill할 거북이 이름 (예, turtle1)
        - 서비스 Response 데이터 : (없음)


- Web Shell #3 : service call 서비스명 서비스데이터형 "{데이터값 딕셔너리}"

```
$ ros2 service call /kill turtlesim/srv/Kill "{name: turtle1}"
waiting for service to become available...
requester: making request: turtlesim.srv.Kill_Request(name='turtle1')

response:
turtlesim.srv.Kill_Response()

```
    - 정리
        - Graphical Tools 명령창에서 이름 "turtle1" 거북이가 삭제됨 



##### 3) 서비스명 /reset (서비스데이터형 std_srvs/srv/Empty)

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 노드 /teleop_turtle 실행 

```
$ ros2 run turtlesim turtle_teleop_key

```
    - 정리
        - 키보드 상/하/좌/우 화살표를 눌러 거북이를 이동 및 회전시킴 
        - 거북이 이동에 따은 경로가 표시되도록 함



- Web Shell #3 : interface show 서비스데이터형(std_srvs/srv/Empty)

```
$ ros2 interface show std_srvs/srv/Empty
---

```
    - 정리
        - 서비스 Request 데이터  : (없음)
        - 서비스 Response 데이터 : (없음)


- Web Shell #3 : service call 서비스명 서비스데이터형 "{데이터값 딕셔너리}"

```
$ ros2 service call /reset std_srvs/srv/Empty
waiting for service to become available...
requester: making request: std_srvs.srv.Empty_Request()

response:
std_srvs.srv.Empty_Response()

```
    - 정리
        - Graphical Tools 명령창이 초기화됨 (이동 경로가 삭제되고, 거북이가 초기위치로 돌아옴)



##### 4) 서비스명 /spawn (서비스데이터형 turtlesim/srv/Spawn)

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node
[INFO] [1697684001.268528966] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [1697684001.274205454] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

```
    - 정리
        - 노드 /turtlesim 실행됨 
        - 이름 turtle1 거북이가 x=5.544445, y=5.544445, theta=0.000000(in Rad) 포지션에 생성(Spawning)됨

- Web Shell #2 : 노드 /teleop_turtle 실행 

```
$ ros2 run turtlesim turtle_teleop_key

```
    - 정리
        - 키보드 상/하/좌/우 화살표를 눌러 거북이를 이동 및 회전시킴 
        - 거북이 이동에 따은 경로가 표시되도록 함



- Web Shell #3 : interface show 서비스데이터형(turtlesim/srv/Spawn)

```
$ ros2 interface show turtlesim/srv/Spawn
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name

```
    - 정리
        - 서비스 Request 데이터  : x - 거북이 생성 위치 x 좌표
                                   y - 거북이 생성 위치 y 좌표
                                   theta - 거북이 생성 방향 각도 (in Rad)
                                   name - 생성하고자 하는 거북이 이름 (이름을 지정하지 않으면, 자동으로 unique한 이름을 지정함)
        - 서비스 Response 데이터 : name - 생성된 거북이 이름


- Web Shell #4 : service call 서비스명 서비스데이터형 "{데이터값 딕셔너리}"

```
$ ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 0.7854, name: my_turtle}"
waiting for service to become available...
requester: making request: turtlesim.srv.Spawn_Request(x=1.0, y=1.0, theta=0.7854, name='my_turtle')

response:
turtlesim.srv.Spawn_Response(name='my_turtle')

```
    - 정리
        - 기존 Graphical Tools 명령창에 x=1.0, y=1.0 위치에, 방향 각도 theta=45도(0.7854(in Rad))로, 
        - 이름 my_turtle의 새로운 거북이가 생성됨 
        - 새로 생성된 거북이 my_turtle과 관련된 topic, service, action이 새로 추가됨 (아래 표 참조)


|  interface | topic, service, action name | data-type |
|:---|:---|:---|
| topic   | /my_turtle/cmd_vel           | geometry_msgs/msg/Twist |
| service | /my_turtle/set_pen           | turtlesim/srv/SetPen |
| service | /my_turtle/teleport_absolute | turtlesim/srv/TeleportAbsolute |
| service | /my_turtle/teleport_relative | turtlesim/srv/TeleportRelative |
| action  | /my_turtle/rotate_absolute   | turtlesim/action/RotateAbsolute |


- Web Shell #5 : remap (토픽 /turtle1/cmd_vel ---> 토픽 /my_turtle/cmd_vel)  

```
$ ros2 run turtlesim turtle_teleop_key --ros-args --remap /turtle1/cmd_vel:=/turtle2/cmd_vel

```
    - 정리
        - 명렁창 #5에서 토픽 /turtle1/cmd_vel을 토픽 /my_turtle/cmd_vel로 remap하여 
        - 키보드 제어를 하면 토픽 /my_turtle/cmd_vel가 퍼블리시되어 
        - 새로 Spawn한 거북이 my_turtle을 제어할 수 있다. 


&#8251; 참고 사항: 별도의 graphic moniter 창에 새로운 거북이 생성

```
$ ros2 run turtlesim turtlesim_node __node:=my_turtle

```


##### 5) 서비스명 /turtle1/set_pen (서비스데이터형 turtlesim/srv/SetPen)

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```


- Web Shell #2 : 노드 /teleop_turtle 실행 

```
$ ros2 run turtlesim turtle_teleop_key

```
    - 정리
        - 키보드 상/하/좌/우 화살표를 눌러 거북이를 이동 및 회전시킴 
        - 거북이 이동에 따은 경로가 표시되도록 함



- Web Shell #3 : interface show 서비스데이터형(turtlesim/srv/SetPen)

```
$ ros2 interface show turtlesim/srv/SetPen
uint8 r
uint8 g
uint8 b
uint8 width
uint8 off
---

```
    - 정리
        - 서비스 Request 데이터  : r - 거북이 경로선 색 r값(0-255, 디폴트 179 (0xb3))
                                   g - 거북이 경로선 색 g값(0-255, 디폴트 184 (0xb8))
                                   b - 거북이 경로선 색 b값(0-255, 디폴트 255 (0xff))
                                   width - 거북이 경로선 두께(디폴트 3) 
                                   off - 경로선 표시 OFF 여부(디폴트 0)
        - 서비스 Response 데이터 : (없음)


- Web Shell #4 : service call 서비스명 서비스데이터형 "{데이터값 딕셔너리}"

```
$ ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 100, g: 0, b: 0, width: 3, 'off': 0}"
waiting for service to become available...
requester: making request: turtlesim.srv.SetPen_Request(r=100, g=0, b=0, width=3, off=0)

response:
turtlesim.srv.SetPen_Response()

```
    - 정리
        - 명령창 #2에서 다시 거북이를 이동시키면, 펜색이 붉은 색(r=100, g=0, b=0)으로 표시됨 
        - ??? 딕셔너리 키워드 중 off를 'off'와 같이 string으로 표시해야 하는 이유 ??? 




##### 6) 서비스명 /turtle1/teleport_absolute (서비스데이터형 turtlesim/srv/TeleportAbsolute)

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```


- Web Shell #2 : interface show 서비스데이터형(turtlesim/srv/TeleportAbsolute)

```
$ ros2 interface show turtlesim/srv/TeleportAbsolute
float32 x
float32 y
float32 theta
---

```
    - 정리
        - 서비스 Request 데이터  : x - 거북이 위치 절대좌표 x값 
                                   y - 거북이 위치 절대좌표 x값 
                                   theta - 거북이 방향각도 theta (in Rad)
        - 서비스 Response 데이터 : (없음)


- Web Shell #3 : service call 서비스명 서비스데이터형 "{데이터값 딕셔너리}"

```
$ ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 8.0, y: 8.0, theta: 1.5708}"
waiting for service to become available...
requester: making request: turtlesim.srv.TeleportAbsolute_Request(x=8.0, y=8.0, theta=1.5708)

response:
turtlesim.srv.TeleportAbsolute_Response()

```
    - 정리
        - 지정된 절대 좌표로 이동한 후, 지정된 절대 각도로 방향을 회전함 
        - 초기 위치 및 방향 : x=5.544445, y=5.544445, theta=0.0



##### 7) 서비스명 /turtle1/teleport_relative (서비스데이터형 turtlesim/srv/TeleportRelative)

- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```


- Web Shell #2 : interface show 서비스데이터형(turtlesim/srv/TeleportRelative)

```
$ ros2 interface show turtlesim/srv/TeleportRelative
float32 linear
float32 angular
---

```
    - 정리
        - 서비스 Request 데이터  : linear - 거북이 위치 상대 진행 거리   
                                   angular - 거북이 방향 상대 회전 각도 (in Rad)
        - 서비스 Response 데이터 : (없음)


- Web Shell #3 : service call 서비스명 서비스데이터형 "{데이터값 딕셔너리}"

```
$ ros2 service call /turtle1/teleport_relative turtlesim/srv/TeleportRelative "{linear: 2.0, angular: 0.7854}"
requester: making request: turtlesim.srv.TeleportRelative_Request(linear=2.0, angular=0.7854)

response:
turtlesim.srv.TeleportRelative_Response()

```
    - 정리
        - 지정된 상대 각도만큼 회전한 후, 지정된 상대 거리만큼 이동함  





#### 7.7.3 Turtlesim 액션 사용 예제

- Action Server 

| interface |                action name |                         data-type |
|:----------|:---------------------------|:----------------------------------|
| action    |   /turtle1/rotate_absolute |   turtlesim/action/RotateAbsolute |


##### 1) 액션명 /turtle1/rotate_absolute (액션데이터형 turtlesim/action/RotateAbsolute)


- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```
    - 정리
        - 패키지     : turtlesim
        - 익스큐터블 : turtlesim_node 
        - 노드       : /turtlesim 



- Web Shell #2 : interface show 액션데이터형(turtlesim/action/RotateAbsolute)

```
$ ros2 interface show turtlesim/action/RotateAbsolute
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining

```
    - 정리
        - 액션 Goal 데이터     : theta     - 도착 방향의 절대 각도(in Rad)
        - 액션 Result 데이터   : delta     - 도착 방향에서 출발 방향까지 남은 상대 각도(in Rad) 
        - 액션 Feedback 데이터 : remaining - 현재 방향에서 도착 방향까지 남은 상대 각도(in Rad)


- Web Shell #3 : action send_goal without feedback

```
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.5708}"
Waiting for an action server to become available...
Sending goal:
     theta: 1.5708

Goal accepted with ID: 05d8063dec8e4b8d9f073c7b9cb7e5a4

Result:
    delta: -1.5520002841949463

Goal finished with status: SUCCEEDED

```
    - 정리
        - 거북이가 90도(1.5708Rad) 방향으로 회전


- Web Shell #4 : action send_goal with feedback

```
$ ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 3.1416}" --feedback
Waiting for an action server to become available...
Sending goal:
     theta: 3.1416

Goal accepted with ID: 3aa3a3acf52f4c02a614ad210001da8c

Feedback:
    remaining: 1.589599847793579

Feedback:
    remaining: 1.5735995769500732

......

Feedback:
    remaining: 0.005596455652266741

Result:
    delta: -1.584003210067749

Goal finished with status: SUCCEEDED

```
    - 정리
        - 거북이가 180도(3.1416Rad) 방향으로 회전하면서 중간 과정의 Feedback을 출력함







### 7.8 Turtlesim parameter 사용 예제

* 참조: https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html


- Web Shell #1 : 노드 /turtlesim 실행 

```
$ ros2 run turtlesim turtlesim_node

```

- Web Shell #2 : 노드 /teleop_turtle 실행 

```
$ ros2 run turtlesim turtle_teleop_key 

```

#### 7.8.1 파라미터 리스트 

##### 1) param list 

- Web Shell #3 : param list 

```
$ ros2 param list 
/teleop_turtle:
  scale_angular
  scale_linear
  use_sim_time
/turtlesim:
  background_b
  background_g
  background_r
  use_sim_time

```
    - 정리
        - 파라미터는 각 노드에 대하여 정의함
        - 예를 들면, use_sim_time은 개별 노드마다 각각 정의함 
        - background_r, background_g, background_b: 바탕화면의 rgb 칼러 값(0-255)
        - use_sim_time: rosbag에서 재생 시, 토픽을 읽어 들일 때 토픽 저장 시간 간격을 사용할지 여부 (디폴트: false)


#### 7.8.2 파라미터 get & set

##### 1) param get 노드명 파라미터명

- Web Shell #3 : param get /노드명 파라미터명

```
$ ros2 param get /turtlesim background_r
Integer value is: 69


$ ros2 param get /turtlesim background_g
Integer value is: 86


$ ros2 param get /turtlesim background_b
Integer value is: 255

```
    - 정리
        - Graphical Tools 창의 turtlesim 실행화면의 바탕색은 청색(r=69, g=86, b=255)


##### 2) param set 노드명 파라미터명 값

```
$ ros2 param set /turtlesim background_r 150
Set parameter successful

```
    - 정리
        - Graphical Tools 창의 turtlesim 실행화면의 바탕색이 청색(r=69, g=86, b=255)에서 보라색(r=150, g=86, b=255)으로 변함



#### 7.8.3 파라미터 dump & load 

##### 1) 노드의 파라미터들을 디폴트파일 노드명.yaml로 저장

- Web Shell #3 : param dump 노드명 

```
$ cd ~
$ mkdir param_yaml
$ cd ~/param_yaml

$ ros2 param dump /turtlesim 
Saving to:  ./turtlesim.yaml

$ ls
turtlesim.yaml ......

```
    - 정리
        - 홈 디랙토리 밑에 param_yaml 폴더를 만들고, 해당 폴더로 이동 
        - 현재 디렉토리 밑에 노드명.yaml 파일로 파라미터 값 저장


##### 2) 노드의 파라미터들을 지정 파일명.yaml로 저장

- Web Shell #3 : param dump 노드명 > 파일명.yaml

```
$ cd ~
$ mkdir param_yaml
$ cd ~/param_yaml

$ ros2 param dump /turtlesim > turtlesim.yaml

$ ls
turtlesim.yaml ......

```

- Web Shell #3 : yaml 파일 확인

```
$ cat turtlesim.yaml

$ cat turtlesim.yaml
/turtlesim:
  ros__parameters:
    background_b: 255
    background_g: 86
    background_r: 150
    use_sim_time: false

```

##### 3) 실행 중인 노드에 파라미터파일 로드 

- Web Shell #3 : param load 노드명 파일명.yaml
    - background_r: 150에서 69로 변경 

```
$ ros2 param load /turtlesim turtlesim.yaml
Set parameter background_b successful
Set parameter background_g successful
Set parameter background_r successful
Set parameter use_sim_time successful

```
    - 정리
        - 파일명.yaml에 기록된 파라미터 값들을 읽어서, 실행 중인 노드의 파라미터에 값을 지정함
        - Graphical Tools 창의 turtlesim 실행화면의 바탕색이 보라색(r=150, g=86, b=255)에서 청색(r=69, g=86, b=255)으로 변함


##### 4) 노드 실행 시, 파라미터파일 로드 

- Web Shell #3 : run 패키지 익스큐터블 --ros-args --params-file 파라미터파일.yaml

```
$ ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
[INFO] [1697720891.106305533] [turtlesim]: Starting turtlesim with node name /turtlesim
[INFO] [1697720891.109714707] [turtlesim]: Spawning turtle [turtle1] at x=[5.544445], y=[5.544445], theta=[0.000000]

```
    - 정리
        - turtlesim 노드를 실행하면서(Graphical Tools 창의 turtlesim 실행화면이 하나 더 열림) 
        - turtlesim 노드의 파라미터들을 turtlesim.yaml 파일의 파라미터 값으로 지정함.  




### 7.9 Lauch 활용 Turtlesim 실행 

- Launch는 복잡한 여러 개의 노드들을 한 번에 실행시킬 때 편리함

- "7.3.2 노드 다중 실행" > "4) 노드 mimic 이용한 중복 실행" 예제 중 Web Shell #1, #2, #4 를 Launch 파일 하나로 묶음 

1) Launch 디렉토리 생성 

```
$ cd ~/<작업영역>/src
$ ros2 pkg create <패키지명> --build-type ament_python --dependencies rclpy 
$ cd <패키지명> 
$ mkdir launch 
$ cd launch
$ touch <런치파일명>

```
    - 정리
        - <작업영역>   : ros2_ws
        - <패키지명>   : my_turtlesim
        - <런치파일명> : turtlesim_mimic.launch.py 
    

2) 런치파일 turtlesim_mimic.launch.py

```
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='ns1',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtlesim',
            namespace='ns2',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='turtle_mimic',
            remappings=[
                ('/input/pose', '/ns1/turtle1/pose'),
                ('/output/cmd_vel', '/ns2/turtle1/cmd_vel'),
            ]
        ), 
        ExecuteProcess(
            cmd=["ros2", "run", "rqt_graph", "rqt_graph"],
            output="screen",
        )
    ])

```

3) 설정파일 setup.py 수정

- 추가1: 3줄

```
import os
from glob import glob
from setuptools import setup
......

```

- 추가2: (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

```
......
    data_files=[
        ('share/ament_index/resource_index/packages', 
                         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
......

```

- 추가3: '<노드명> = <패키지명>.<소스파일명>:main',

```
......
    entry_points={
        'console_scripts': [
            '<노드명> = <패키지명>.<소스파일명>:main',
        ],
    },
......

```
    - 정리 
        - "추가3: '<노드명> = <패키지명>.<소스파일명>:main'"는 파이썬 소스로 실행되는 executable이 없으면 필요없다. 


4) 빌드

```
$ cd ~/<작업영역>
$ colcon build --symlink-install --packages-select <패키지명>

```



5) 실행 

- Web Shell #1 : 

```
$ source ~/<작업영역>/install/local_setup.bash
$ ros2 launch <패키지명> turtlesim_mimic.launch.py

```

- Web Shell #2 : 

```
$ ros2 run turtlesim turtle_teleop_key --ros-args --remap __ns:='/ns1'

```
    - 정리 
        - Web Shell #2에서 키보드 제어를 하면, 
          노드 /ns1/teleop_turtle이 토픽 /ns1/turtle1/cmd_vel을 퍼블리시하고 
        - Web Shell #1에서 노드 /ns1/turtlesim이 토픽 /ns1/turtle1/cmd_vel를 서브스크라이브하여 /ns1의 거북이가 움직이면,
          /ns1의 거북이의 움직임에 따라 토픽 /ns1/turtle1/pose를 퍼블리시하고
        - Web Shell #1에서 노드 /mimic이 토픽 /input/pose를 토픽 /ns1/turtle1/pose로 리맵하여 서브스크라이브하면,
          토픽 /input/pose의 움직임 제어 정보를 읽어 토픽 /output/cmd_vel을 토픽 /ns2/turtle1/cmd_vel로 리맵하여 퍼블리시하고, 
        - Web Shell #1에서 노드 /ns2/turtlesim이 토픽 /ns2/turtle1/cmd_vel를 서브스크라이브하여 /ns2의 거북이가 움직인다. 
          
    
    

=================================

# 정리 

### 1) 패키지 내의 executables 확인

- 패키지 : executables 

```
$ ros2 pkg executables 패키지명

```

### 2) executable의 node name 확인

- 노드 리스트 실행(1) -> executable 실행 -> 노드 리스트 실행(2) -> 새로 실행된 node name 확인 

```
$ ros2 node list

```

### 3) node의 interface 확인

- 노드명 

    - 토픽 Sub : 토픽명, 토픽메시지

    - 토픽 Pub : 토픽명, 토픽메시지

    - 서비스 Server : 서비스명, 서비스메시지

    - 서비스 Client : 서비스명, 서비스메시지

    - 액션 Server : 액션명, 액션메시지

    - 액션 Client : 액션명, 액션메시지


```
$ ros2 node info 노드명

```

### 4) 인터페이스 별 인터페이스메시지 확인 

```
$ ros2 topic info 토픽명 
$ ros2 topic type 토픽명

$ ros2 service type 서비스명

$ ros2 action type 액션명 

```

### 5) 데이터 구조 확인 

```
$ ros2 interface proto 인터페이스메시지 (메시지정의패키지/메시지유형폴더/데이토구조) 

$ ros2 interface show 인터페이스메시지 (메시지정의패키지/메시지유형폴더/데이토구조)

```

### 6) 메시지 발송 

- 토픽/서비스/액션 메시지 발송 

```
$ ros2 topic pub 토픽명 토픽메시지 [Tab][Tab]
$ ros2 topic pub 토픽명 토픽메시지 "{딕셔너리 값}"

$ ros2 service call 서비스명 서비스메시지 [Tab][Tab]
$ ros2 service call 서비스명 서비스메시지 "{딕셔너리 값}"

$ ros2 action send_goal 액션명 액션메시지 [Tab][Tab]
$ ros2 action send_goal 액션명 액션메시지 "{딕셔너리 값}"

```

=================================

# &#8251; Turtlesim 특징: 
    1) 토픽 /turtle1/cmd_vel : Subscribe 후 1초 경과하면 정지시킴 
        소스(148-153번째 줄) : https://github.com/ros/ros_tutorials/blob/noetic-devel/turtlesim/src/turtle.cpp
              if (ros::WallTime::now() - last_command_time_ > ros::WallDuration(1.0))
              {
                lin_vel_x_ = 0.0;
                lin_vel_y_ = 0.0;
                ang_vel_ = 0.0;
              }
              



```python

```
