# 4_2 Python 예제 :  dice 

- 패키지              : dice 
- 노드 퍼블리셔       : /dice_pub_node 
- 노드 서브스크라이버 : /dice_sub_node 
- 토픽                : /dice 
- 데이터형            : std_msgs Int32

- random number 라이브러리 및 생성

   * 라이브러리 : random
   * 함      수 : random.randint(최소값, 최대값)
   



### 4_2.1 Workspace 및 Package 생성

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create dice --build-type ament_python --dependencies rclpy std_msgs
$ cd dice
$ ls
dice        package.xml     resource     setup.cfg     setup.py     test

```


### 4_2.2 소스 파일  dice_pub.py, dice_sub.py 


- 파일 dice_pub.py

    ~/ros2_ws/src/dice/dice/dice_pub.py

```
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
import random

class pubDice(Node):
  def __init__(self):
    super().__init__('dice_pub_node')
    qos_profile = QoSProfile(depth=10)

    self.pub_dice = self.create_publisher(Int32, 'dice', qos_profile)
    self.timer = self.create_timer(1., self.pub_dice_callback)

  def pub_dice_callback(self):
    number = random.randint(1, 6)
    msg = Int32()
    msg.data = number
    self.pub_dice.publish(msg)
    self.get_logger().info('Dice Number : {0}'.format(number)) 

def main(args=None):
  rclpy.init(args=args)
  node = pubDice() 
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


- 파일 dice_sub.py 

    ~/ros2_ws/src/dice/dice/dice_sub.py

```
import rclpy
from rclpy.node import Node 
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
import numpy

class subDice(Node):
  def __init__(self):
    super().__init__('dice_sub_node')
    qos_profile = QoSProfile(depth=10)

    self.sub_dice = self.create_subscription(Int32, 'dice', self.sub_dice_callback, qos_profile)
    
    self.count = numpy.zeros(7, dtype=int)

  def sub_dice_callback(self, msg):
    number = msg.data
    self.count[0] += 1
    self.count[number] += 1
    self.get_logger().info('Sub Dice Number : {0} : {1} : {2:.2f}, {3:.2f}, {4:.2f}, {5:.2f}, {6:.2f}, {7:.2f}'\
                           .format(number\
                                   , self.count[0]\
                                   , self.count[1]/self.count[0]\
                                   , self.count[2]/self.count[0]\
                                   , self.count[3]/self.count[0]\
                                   , self.count[4]/self.count[0]\
                                   , self.count[5]/self.count[0]\
                                   , self.count[6]/self.count[0]))

def main(args=None):
  rclpy.init(args=args)
  node = subDice() 
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



### 4_2.3 설정 파일  setup.py

- 파일 setup.py

    추가 : 'dice_pub_node = dice.dice_pub:main',
    
    추가 : 'dice_sub_node = dice.dice_sub:main',

```
from setuptools import setup

package_name = 'dice'

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
            'dice_pub_node = dice.dice_pub:main',
            'dice_sub_node = dice.dice_sub:main',
        ],
    },
)

```



### 4_2.4 빌드 

- 워크스페이스 내의 모든 패키지 빌드

```
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install 

```


- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install --packages-select dice

``` 

- 특정 패키지 및 의존성 패키지를 함께 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install --packages-up-to dice 

```


### 4_2.5 실행 

- Web Shell #1

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run dice dice_pub_node

```

- Web Shell #2

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run dice dice_sub_node 

```

- Web Shell #3

```
$ rqt_graph

```



```python

```
