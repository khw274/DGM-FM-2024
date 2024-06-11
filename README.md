# 2024년 DGM 미래 모빌리티 경진대회 
영남대학교 미래차 융합전공 [B-1]전기차융합부품트랙 2024 DGM 미래 모빌리티 경진대회

```🏆2024년 DGM 미래 모빌리티 경진대회 동상(RIS 트랙책임교수상)🏆```  

<img src="https://github.com/khw274/DGM-FM-2024/assets/125671828/9d3517b2-e4ea-4639-8fd8-f5d90ad25ff2" width="400" height="400"/>

## 공모 내용  
4차 산업혁명의 주요 분야 중 하나인 미래자동차 분양의 자율주행차량의 특성을 이해하고,   
ROS 프로그래밍을 학습하여 터틀봇을 활용한 자율주행 구현

## 대회 진행
### 기본 세팅
#### 설치
- 듀얼 부팅으로 Ubuntu 20.04를 설치
- Ubuntu 20.04에서 사용 가능한 ros2 foxy 버전 설치

#### SSH 통신
- PC에서 노드와 노드 사이의 연결과 통신을 위한 서버를 실행하기 위해 ROS Master 실행(실행 명령어 roscore),  
지정한 핫스팟의 ip로 ubuntu와 turtlebot3 간 ssh 통신(고정 ip, wifi 설정 이용)
```
$ cd /etc/netplan  → netplan을 통하혀 ip 설정
$ ls

50-cloud-init.yaml  → 해당 파일 수정

$ sudo nano 50-cloud-init.yaml
----
      wifies:
        wlanO:
        dhcp4: yes
        dhcp6: yes
        access-points:
              [네트워크 ID]:
                    password: [네트워크 PASSWORD]

[Ctri]x > Save=Y > [Enter]

재부팅 : $ shutdown -h
$ cd ~
$ ifconfig  → ifconfig(현재 설정된 네트워크 인터페이스 상태) 입력 후 wlan0 주소 확인,
              wlan0: 무선 네트워크 인터페이스
```
- 확인한 무선 네트워크 인터페이스 wlan0의 주소를 사용해 ```ssh ubuntu@[ip주소]``` 명령어로 turtlebot3에 접속
- 접속한 turtlebot3 명령 창에서 turtlebot3 bringup(그 외 모든 명령은 PC에서 수행)

### Turtlebot3 Real World 
실제 TurtleBot3 로봇을 물리적 환경에서 사용


#### SLAM (Simultaneous Localization and Mapping)  
- 주변 위치 정보로 부터 위치를 추정하고, 주변 환경에 대한 지도를 동시에 작성하는 기술
- SLAM 방법 중 Cartographer 방법 사용

##### 1) Real World 실행
- PC 명령창에서 SBC와 데스크탑(노트북) 간의 SSH 연결
SBC: Single Board Computer, 해당 대회에서는 Turtlebot3에 내장된 RaspberryPi 4B가 사용됨
```
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}

ros2 launch turtlebot3_bringup robot.launch.py
```
- 연결된 SBC 환경에서 ROS2를 사용하여 TurtleBot3 로봇을 시작, turtlebot3_bringup 패키지의 robot.launch.py 파일을 실행하여  
  TurtleBot3 로봇을 초기화하고 필요한 노드들을 실행

##### 2) SLAM Node 실행
- 원격 PC에서 새 터미널을 열고 SLAM 노드 실행
```
export TURTLEBOT3_MODEL=burger  → Turtlebot3 burget 모델에 맞는 설정이 자동 적용
                                  (모델 종류: burger, waffle, waffle_pi, 해당 대회에서 burger 모델 사용)
$ ros2 launch turtlebot3_bringup robot.launch.py
```

##### 3) 원격조작 노드 실행
SLAM 노드가 실행되면 Turtlebot3는 원격 조작을 해 지도의 알려지지 않은 영역을 탐색하여 지도를 그릴 수 있음  
- 원격 PC에서 새 터미널을 열고 teleop_keyboard 노드 실행
```
$ export TURTLEBOT3_MODEL=burger
$ ros2 run turtlebot3_teleop teleop_keyboard

Control Your TurtleBot3!
---------------------------
Moving around:
       w
  a    s    d
       x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
space key, s : force stop

CTRL-C to quit
```
<img src="https://github.com/khw274/DGM-FM-2024/assets/125671828/3285b3db-962a-4d13-a633-d51b856b0d15" width="500" height="700"/>  
실제 대회 맵 
<img src="https://github.com/khw274/DGM-FM-2024/assets/125671828/4bd132eb-0b75-4f57-80dc-4c3e0814b8ab" width="700" height="400"/>  
추출한 맵 파일

##### 4) Map 저장
- 원격 PC에서 명령어 실행
```
$ cd ~
$ ros2 run nav2_map_server map_saver_cli -f ~/map1

$ ls map1.*
map1.pgm  map1.yaml
```

#### Navigation
- 주어진 환경에서 로봇을 한 위치에서 지정된 목적지로 이동시키는 것
- Navigation은 Map, 로봇의 인코더, IMU 센서, 거리 센서를 이용하여 로봇이 지도 상의 현재 자세에서 지정된 목표 자세로 이동할 수 있도록 해준다

##### 1) Navigation Node 실행
- bringup으로 Real World을 실행했다는 전제하에 원격 PC에서 Navigation Node 실행
```
$ export TURTLEBOT3_MODEL=burger
$ ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml
```

##### 2) Initial Pose 설정(로봇의 초기 위치와 방향을 설정)
- 로봇이 목표 지점으로 이동하기 위해 경로를 계획할 때 Initial Pose가 필요하다
- TurtleBot3는 표시된 지도와 깔끔하게 겹치는 LDS 센서 데이터를 사용하여 지도에서 올바르게 위치해야 한다  
- LDS 센서: "Laser Distance Sensor" 또는 "Laser Detection and Ranging", 레이저를 이용하여 물체와의 거리를 측정하는 센서  
  레이저를 발사하고 반사된 레이저를 수신하여 물체와의 거리를 측정
```
1. 원격 PC의 RVIZ2의 메뉴에서 [2D Pose Estimate] button을 클릭
2. 실제 Turtlebot3 로봇이 위치한 지도를 클릭하고 로봇이 바라보는 방향으로 드래그
3. LDS 센서 데이터가 저장된 지도에 오버레이될 때까지 1단계와 2단계를 반복
```
![image](https://github.com/khw274/DGM-FM-2024/assets/125671828/154f1c59-d365-4e59-9904-e240618192db)
예시 이미지(출처: https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#navigation)

##### 3) Navigation Goal 설정(로봇의 목적지 설정)
```
1. 원격 PC의 RVIZ2의 메뉴에서 [Navigation2 Goal] button을 클릭
2. 도착 위치를 클릭하고 도착 위치로부터 진행할 방향으로 드래그
```

