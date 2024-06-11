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
- Ubuntu 20.04에서 사용하는 ros noetic 버전 설치

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
$ ifconfig  → ifconfig(현재 설정된 네트워크 인터페이스 상태) 입력 후 wlan0 주소 확인, wlan0: 무선 네트워크 인터페이스
```
- 확인한 무선 네트워크 인터페이스 wlan0의 주소를 사용해 ```ssh ubuntu@[ip주소]``` 명령어로 turtlebot3에 접속
- 접속한 turtlebot3 명령 창에서 turtlebot3 bringup(그 외 모든 명령은 PC에서 수행)

[활용]
ROS2 Foxy, GAZEBO, RVIZ, Turtlebot3, UBUNTU 20.04

[SLAM]
● pc 명령창에서 slam 노드 실행
● slam 노드 성공적으로 실행 시 원격 조작 노드를 실행시켜 지도를 탐색하고 빈틈 없이 그림 => map saver 노드를 실행해 지도 저장

[NAVIGATION]
● 네비게이션 노드를 실행
● 로봇의 초기 위치가 중요함, RVIZ 메뉴에서 2D Pose Estimate를 클릭하고 실제 로봇이 바라보는 방향으로 드래그, 실제 주변 사물과 동기화가 잘 됐는지 확인
● RVIZ 메뉴에서 2D Nav Goal을 클릭해 목적지를 설정하고 녹색 화살표를 로봇이 향하는 방향으로 드래그 => 자동으로 turtlebot3가 목적지로 이동

