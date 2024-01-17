[기본 Tip] Python 코드 작성 스타일 

- 이름 규칙 
    - 타입, 클래스 : CamelCased (각 단어 첫글자만 대문자)
    - 파일, 패키지, 인터페이스, 변수, 함수, 메소드 : snake_case (모두 소문자, 단어 사이는 _ 연결)
    - 상수 : ALL_CAPITALS (모두 대문자, 단어 사이는 _ 연결)
- 들여쓰기 : space 4칸 사용


---



# 4 Python 예제 : HelloWorld 

- 패키지              : tutorial_topic 
- 노드 퍼블리셔       : helloworld_publisher
- 노드 서브스크라이버 : helloworld_subscriber 
- 토픽                : helloworld 


### 4.1 Workspace 생성 

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create tutorial_topic --build-type ament_python --dependencies rclpy std_msgs
$ cd tutorial_topic
$ ls
package.xml     resource     setup.cfg     setup.py     test     tutorial_topic

```


### 4.2 소스 파일  helloworld_publisher.py, helloworld_subscriber.py 


- 파일 helloworld_publisher.py

    ~/ros2_ws/src/tutorial_topic/tutorial_topic/helloworld_publisher.py

```
import rclpy
from rclpy.node import Node 

# 퍼블리셔의 QoS 설정을 위해 QoSProfile 클래스를 사용
from rclpy.qos import QoSProfile 
# 퍼블리싱하는 메시지 타입은 std_msgs.msg의 String이므로 import
from std_msgs.msg import String  

# rclpy의 Node 클래스를 상속하여 사용
class HelloworldPublisher(Node): 
    def __init__(self):
        # Node 클래스의 이름을 helloworld_publisher라 지정
        super().__init__('helloworld_publisher') 

        # 퍼블리시할 데이터 버퍼에 10개까지 저장
        qos_profile = QoSProfile(depth=10)       
        
        # 퍼블리셔 노드 생성 (msg type, topic name, QoS)
        self.helloworld_publisher = self.create_publisher(String, 'helloworld', qos_profile) 

        # 콜백 함수 실행 시 사용되는 타이머로 지정한 값마다 콜백함수를 실행 (timer_period_sec, 발행을 실행할 함수)
        self.timer = self.create_timer(1, self.publish_helloworld_msg) 
        self.count = 0

    # callback function, callback함수를 구현할 때는 멤버함수, lambda, 지역 함수 등으로 선언이 가능하다. 이 때는 멤버함수 사용
    def publish_helloworld_msg(self):
        # 메시지 타입 - String
        msg = String()        

        # 메시지의 data 입력          
        msg.data = 'Hello World: {0}'.format(self.count) 

        # 메시지 발행
        self.helloworld_publisher.publish(msg) 

        # print와 비슷한 함수로 기록용
        self.get_logger().info('Published message: {0}'.format(msg.data)) 
        self.count += 1

def main(args=None):              

    # 초기화       
    rclpy.init(args=args)               

    # node라는 이름으로 클래스 생성
    node = HelloworldPublisher()        
    try:
        # 노드를 spin, 즉 지정된 콜백함수를 실행
        rclpy.spin(node)                
    except KeyboardInterrupt:

        # ctrl+c와 같은 인터럽트 시그널을 받으면 반복을 끝냄
        node.get_logger().info('Keyboard Interrupt (SIGINT)')   
    finally:
        # 노드 소멸
        node.destroy_node() 

        # 노드 종료
        rclpy.shutdown()    

if __name__ == '__main__':
    main()

```


- 파일 helloworld_subscriber.py 

    ~/ros2_ws/src/tutorial_topic/tutorial_topic/helloworld_subscriber.py

```
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


class HelloworldSubscriber(Node):

    def __init__(self):
        super().__init__('Helloworld_subscriber')

        # 서브스크라이버 데이터를 버퍼에 10개까지 저장
        qos_profile = QoSProfile(depth=10)          
        self.helloworld_subscriber = self.create_subscription(
            String,
            'helloworld',
            self.subscribe_topic_message,
            qos_profile)                    # 메시지 타입, 토픽 이름, 콜백 함수, QoS

    def subscribe_topic_message(self, msg):
        # 데이터를 받으면 logging
        self.get_logger().info('Received message: {0}'.format(msg.data)) 


def main(args=None):

    # 초기화
    rclpy.init(args=args)           

    # 클래스 생성
    node = HelloworldSubscriber()   
    try:
        # 콜백함수 실행
        rclpy.spin(node)            
    except KeyboardInterrupt:
        # 시그널 시 정지
        node.get_logger().info('Keyboard Interrupt (SIGINT)')   
    finally:
        # 노드 소멸
        node.destroy_node()         

        # 노드 종료
        rclpy.shutdown()            


if __name__ == '__main__':
    main()

```



### 4.3 설정 파일  package.xml, setup.py, setup.cfg

- 파일 package.xml 수정

    수정사항 없음

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tutorial_topic</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>

```

- 파일 setup.py

    추가 : 'helloworld_publisher = tutorial_topic.helloworld_publisher:main',
    
    추가 : 'helloworld_subscriber = tutorial_topic.helloworld_subscriber:main',

```
from setuptools import setup

package_name = 'tutorial_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'helloworld_publisher = tutorial_topic.helloworld_publisher:main',
            'helloworld_subscriber = tutorial_topic.helloworld_subscriber:main',
        ],
    },
)

```

- 파일 setup.cfg

    수정사항 없음

```
[develop]
script-dir=$base/lib/tutorial_topic
[install]
install-scripts=$base/lib/tutorial_topic

```


### 4.4 빌드 

- 워크스페이스 내의 모든 패키지 빌드

```
$ cd ~/ros2_ws/ 
$ colcon build 

```


- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --packages-select tutorial_topic

``` 

- 특정 패키지 및 의존성 패키지를 함께 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --packages-up-to tutorial_topic 

```


### 4.5 실행 

- Web Shell #1

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run tutorial_topic helloworld_subscriber

```

- Web Shell #2

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run tutorial_topic helloworld_publisher

```

- Web Shell #3

```
$ rqt_graph    (종료: [Ctrl] c)

$ ros2 topic list 
/helloworld
/parameter_events
/rosout

$ ros2 topic echo /helloworld    (종료: [Ctrl] c)
data: 'Hello World: 431'
---
data: 'Hello World: 432'
---

$ ros2 topic info /helloworld
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1

$ ros2 topic type /helloworld
std_msgs/msg/String

```


### 4.6 Publisher 노드 요약 

```
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import 데이터형

class pub클래스명(Node):
  def __init__(self):
    super().__init__('pub노드명')
    qos_profile = QoSProfile(depth=10)

    self.pub객체 = self.create_publisher(데이터형, '토픽명', qos_profile)
    self.timer = self.create_timer(초, self.pub콜백함수)

  def pub콜백함수(self):
    msg = 데이터형()
    msg.data = ......
    self.pub객체.publish(msg)
    self.get_logger().info('문자열') 

def main(args=None):
  rclpy.init(args=args)
  node = pub클래스명() 
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt (SIGINT)')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 4.7 Subscriber 노드 요약 
```
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import 데이터형

class sub클래스명(Node):
  def __init__(self):
    super().__init__('sub노드명')
    qos_profile = QoSProfile(depth=10)

    self.sub객체 = self.create_subscription(데이터형, '토픽명', self.sub콜백함수, qos_profile)

  def sub콜백함수(self, msg):
    ...... (msg.data 출력/처리 부분 삽입)

def main(args=None):
  rclpy.init(args=args)
  node = sub클래스명() 
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt (SIGINT)')
  finally:
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```


### 4.8 다중 스레드(MultiThread) 사용 시, Publisher 노드 요약 

- 다중 스레드(MultiThread)    
   * 다중 스레드를 이용하여, 프로그램 실행 속도를 향상시킴 

- MultiThread 라이브러리     
   * 라이브러리:      
      import threading 

- MultiThread 사용법
   * 선언     
      self.thread = threading.Thread(target=self.스레드실행함수)      
      ...... (멥버 변수 등 추가)      
      self.thread.start()
         
   * 스레드실행함수     
      def 스레드실행함수(self):
          msg = String()      
          rate = self.create_rate(1)  # 1 Hz      
          while rclpy.ok():      
              msg.data = '문자열 : Hello, MultiThreading!'      
              self.publisher.publish(msg)      
              self.get_logger().info('Publishing: {0}'.format(msg.data))      
              rate.sleep()

   * main 함수      
      def main(args=None):      
          rclpy.init(args=args)      
          node = 클래스()      
          
          rclpy.spin(node)
          
          node.thread.join()
          
          rclpy.shutdown()


```
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import 데이터형

import threading

class pub클래스명(Node):
  def __init__(self):
    super().__init__('pub노드명')
    qos_profile = QoSProfile(depth=10)

    self.pub객체 = self.create_publisher(데이터형, '토픽명', qos_profile)
    
    self.thread = threading.Thread(target=self.스레드실행함수)
    ...... (멤버 변수 등 추가)
    self.thread.start()

  def 스레드실행함수(self):
    msg = 데이터형()
    
    rate = self.create_rate(1)  # 1 Hz
    
    while rclpy.ok():
      msg.data = ......
      self.pub객체.publish(msg)
      self.get_logger().info('문자열') 
      rate.sleep()

def main(args=None):
  rclpy.init(args=args)
  node = pub클래스명() 
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    node.get_logger().info('Keyboard Interrupt (SIGINT)')
  finally:
    node.destroy_node()
    
    self.pub객체.thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```



```python

```
