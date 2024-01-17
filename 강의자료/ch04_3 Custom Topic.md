# 4_3 Python 예제 :  Custom Topic 


- 4_3.1 Custom Topic 데이터형 선언 
- 4_3.2 Custom Topic 사용


## 4_3.1 Custom Topic 데이터형 선언 

- Custom Topic 패키지    : my_msgs
- Custom Topic 데이터형  : my_msgs/msg/TwoInts

   * int32 a
   * int32 b




### 4_3.1.1 Workspace 및 Package 생성

   &#8251; 주의사항 : Package 생성 시 : 
   
      * ament_cmake 사용, 
      * --dependencies 옵션 사용 안함

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create my_msgs --build-type ament_cmake 
$ cd my_msgs
$ ls
CMakeLists.txt    include     package.xml     src

```


### 4_3.1.2 Custom 메시지 파일 TwoInts.msg 

```
$ cd ~/ros2_ws/src/my_msgs
$ rm -r include src
$ mkdir msg
$ cd msg


```

- 파일 TwoInts.msg

    ~/ros2_ws/src/my_msgs/msg/TwoInts.msg

```
int32 a
int32 b

```

### 4_3.1.3 설정 파일  CMakeLists.txt & package.xml

- 파일 CMakeLists.txt

      내용 추가 : find_package(ament_cmake REQUIRED) 아래, 5줄 추가

```
......
find_package(ament_cmake REQUIRED)
......

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TwoInts.msg"
 )
 
  ......
 
```

- 파일 package.xml

      내용 추가 : <buildtool_depend>ament_cmake</buildtool_depend> 아래, 3줄 추가

```
  ......
  <buildtool_depend>ament_cmake</buildtool_depend>
  ......

  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>

  ......
 
```



### 4_3.1.4 빌드 

- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install --packages-select my_msgs

``` 



### 4_3.1.5 인터페이스 확인 - msg 확인

- Web Shell #1
   
```
$ source ~/ros2_ws/install/local_setup.bash

$ ros2 interface list | grep my_msgs
my_msgs/msg/TwoInts

$ ros2 interface show my_msgs/msg/TwoInts
int32 a
int32 b

```




## 4_3.2 Custom Topic 사용


- 패키지              : custom_topic 
- 노드 퍼블리셔       : custom_topic_pub_node 
- 노드 서브스크라이버 : custom_topic_sub_node 
- pub클래스           : pubCustomTopic
- sub클래스           : subCustomTopic
- pub노드명           : custom_topic_pub_node
- sub노드명           : custom_topic_sub_node
- 토픽                : two_ints
- 데이터형            : my_msgs/msg/TwoInts
   * int32 a
   * int32 b
- pub콜백함수         : pub_custom_topic_callback
- sub콜백함수         : sub_custom_topic_callback
- pub객체             : pub_custom_topic
- sub객체             : sub_custom_topic

- random number 라이브러리 및 생성

   * 라이브러리 : random
   * 함      수 : random.randint(최소값, 최대값)


### 4_3.2.1 Workspace 및 Package 생성

   &#8251; 주의사항 : Package 생성 시 : 
   
      * --dependencies 옵션 : custom topic 선언 패키지 my_msgs를 라이브러리로 사용 
      
```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create custom_topic --build-type ament_python --dependencies rclpy my_msgs
$ cd custom_topic
$ ls
custom_topic     package.xml     resource     setup.cfg     setup.py     test

```


### 4_3.2.2 소스 파일  custom_topic_pub.py, custom_topic_sub.py 


- 파일 custom_topic_pub.py

    ~/ros2_ws/src/custom_topic/custom_topic/custom_topic_pub.py

```
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from my_msgs.msg import TwoInts
import random

class pubCustomTopic(Node):
  def __init__(self):
    super().__init__('custom_topic_pub_node')
    qos_profile = QoSProfile(depth=10)

    self.pub_custom_topic = self.create_publisher(TwoInts, 'two_ints', qos_profile)
    self.timer = self.create_timer(1., self.pub_custom_topic_callback)

  def pub_custom_topic_callback(self):
    msg = TwoInts()
    msg.a = random.randint(1, 6)
    msg.b = random.randint(1, 6)
    self.pub_custom_topic.publish(msg)
    self.get_logger().info('Pub : a={0}, b={1}'.format(msg.a, msg.b)) 

def main(args=None):
  rclpy.init(args=args)
  node = pubCustomTopic() 
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


- 파일 custom_topic_sub.py 

    ~/ros2_ws/src/custom_topic/custom_topic/custom_topic_sub.py 

```
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from my_msgs.msg import TwoInts

class subCustomTopic(Node):
  def __init__(self):
    super().__init__('custom_topic_sub_node')
    qos_profile = QoSProfile(depth=10)

    self.sub_custom_topic = self.create_subscription(TwoInts, 'two_ints', self.sub_custom_topic_callback, qos_profile)

  def sub_custom_topic_callback(self, msg):
    sum = msg.a + msg.b
    self.get_logger().info('Sub : a={0}, b={1}, sum={2}'.format(msg.a, msg.b, sum))

def main(args=None):
  rclpy.init(args=args)
  node = subCustomTopic() 
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



### 4_3.2.3 설정 파일  setup.py

- 파일 setup.py

    추가 : 'custom_topic_pub_node = custom_topic.custom_topic_pub:main',
    
    추가 : 'custom_topic_sub_node = custom_topic.custom_topic_sub:main',

```
from setuptools import setup

package_name = 'custom_topic'

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
            'custom_topic_pub_node = custom_topic.custom_topic_pub:main',
            'custom_topic_sub_node = custom_topic.custom_topic_sub:main',
        ],
    },
)

```



### 4_3.2.4 빌드 

- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install --packages-select custom_topic

``` 


### 4_3.2.5 실행 

- Web Shell #1

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run custom_topic custom_topic_pub_node

```

- Web Shell #2

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run custom_topic custom_topic_sub_node 

```

- Web Shell #3

```
$ rqt_graph

```



```python

```
