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
