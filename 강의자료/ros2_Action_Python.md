# 6 액션 프로그래밍 (파이썬) 


- 6.1 Custom Action 데이터형 선언
- 6.2 Custom Action 사용


## 6.1 Custom Action 데이터형 선언 


- Custom Topic 패키지    : my_msgs
- Custom Topic 데이터형  : my_msgs/action/Fibonacci


```
   #Goal
   int32 order
   ---
   #Result
   int32[] sequence
   ---
   #Feedback
   int32[] partial_sequence

```


### 6.1.1 Workspace 및 Package 생성

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


### 6.1.2 Custom 액션 파일 Fibonacci.action

```
$ cd ~/ros2_ws/src/my_msgs
$ rm -r include src
$ mkdir action
$ cd action


```

- 파일 Fibonacci.action

    ~/ros2_ws/src/my_msgs/action/Fibonacci.action

```
#Goal
int32 order
---
#Result
int32[] sequence
---
#Feedback
int32[] partial_sequence

```

### 6.1.3 설정 파일  CMakeLists.txt & package.xml

- 파일 CMakeLists.txt

      내용 추가 : find_package(ament_cmake REQUIRED) 아래, 6줄 추가

```
......
find_package(ament_cmake REQUIRED)
......

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TwoInts.msg"
  "srv/AddTwoInts.srv"
  "action/Fibonacci.action"
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


### 6.1.4 빌드 

- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install --packages-select my_msgs

``` 

### 6.1.5 인터페이스 확인 - action 확인

- Web Shell #1
   
```
$ source ~/ros2_ws/install/local_setup.bash

$ ros2 interface list | grep my_msgs
my_msgs/msg/TwoInts
my_msgs/srv/AddTwoInts
my_msgs/action/Fibonacci

$ ros2 interface show my_msgs/action/Fibonacci
#Goal
int32 order
---
#Result
int32[] sequence
---
#Feedback
int32[] partial_sequence

```





## 6.2 Custom Action 사용 


- 패키지              : tutorial_action
- server노드명        : fibonacci_srv_node 
- client노드명        : fibonacci_cli_node 
- srv클래스           : FibonacciActionServer
- cli클래스           : FibonacciActionClient
- 액션명              : fibonacci
- 액션데이터형        : my_msgs/action/Fibonacci

```
#Goal
int32 order
---
#Result
int32[] sequence
---
#Feedback
int32[] partial_sequence
   
```


### 6.2.1 Workspace 및 Package 생성

   &#8251; 주의사항 : Package 생성 시 : 
   
      * --dependencies 옵션 : custom action 선언 패키지 my_msgs를 라이브러리로 사용 

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create tutorial_action --build-type ament_python --dependencies rclpy my_msgs
$ cd tutorial_action
$ ls
package.xml     resource     setup.cfg     setup.py     test       tutorial_action

```


### 6.2.2 소스 파일  fibonacci_action_srv.py,  fibonacci_action_cli.py 


- 파일  fibonacci_action_srv.py

    ~/ros2_ws/src/tutorial_action/tutorial_action/ fibonacci_action_srv.py

```
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from my_msgs.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_srv_node')
        self.srv = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.my_callback)

    def my_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Feedback
        feedback = Fibonacci.Feedback()
        feedback.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order + 1):
            feedback.partial_sequence.append(
                feedback.partial_sequence[i] + feedback.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback.partial_sequence))
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()
        
        # Result
        result = Fibonacci.Result()
        result.sequence = feedback.partial_sequence
        self.get_logger().info('Result: {0}'.format(result.sequence))
        return result


def main(args=None):
    rclpy.init(args=args)

    obj = FibonacciActionServer()

    rclpy.spin(obj)


if __name__ == '__main__':
    main()

```


- 파일  fibonacci_action_cli.py 

    ~/ros2_ws/src/tutorial_action/tutorial_action/ fibonacci_action_cli.py

```
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from my_msgs.action import Fibonacci

class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_cli_node')
        self.cli = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal = Fibonacci.Goal()
        goal.order = order

        self.cli.wait_for_server()

        return self.cli.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    obj = FibonacciActionClient()

    future = obj.send_goal(10)

    rclpy.spin_until_future_complete(obj, future)


if __name__ == '__main__':
    main()

```

### 6.2.3a 액션 서버 파일 fibonacci_action_srv.py 테스트 

- Web Shell #1 : 액션 서버 실행 

```
$ source ~/ros2_ws/install/local_setup.bash

$ cd ~/ros2_ws/src/tutorial_action/tutorial_action

$ python3 fibonacci_action_srv.py

```

- Web Shell #2 : Goal 보내기 

```
$ source ~/ros2_ws/install/local_setup.bash

$ ros2 action send_goal --feedback fibonacci my_msgs/action/Fibonacci "{order: 4}"

```


### 6.2.3b 액션 서버 파일 fibonacci_action_srv.py 테스트 

- Web Shell #1 :액션 서버 python3 실행 

```
$ source ~/ros2_ws/install/local_setup.bash

$ cd ~/ros2_ws/src/tutorial_action/tutorial_action

$ python3 fibonacci_action_srv.py

```

- Web Shell #2 :액션 클라이언트 python3 실행 

```
$ source ~/ros2_ws/install/local_setup.bash

$ cd ~/ros2_ws/src/tutorial_action/tutorial_action

$ python3 fibonacci_action_cli.py

```

액션 서버 python3 실행한 명령창에서 feedback과 result의 결과들이 출력됨



### 6.2.4 설정 파일  package.xml, setup.py, setup.cfg

- 파일 package.xml

    수정사항 없음


- 파일 setup.py

    추가 : 'fibonacci_srv_node = tutorial_action.fibonacci_action_srv:main',
    
    추가 : 'fibonacci_cli_node = tutorial_action.fibonacci_action_cli:main',
 
```
from setuptools import setup

package_name = 'tutorial_action'

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
            'fibonacci_srv_node = tutorial_action.fibonacci_action_srv:main',
            'fibonacci_cli_node = tutorial_action.fibonacci_action_cli:main',
        ],
    },
)

```


- 파일 setup.cfg

    수정사항 없음



### 6.2.5 빌드

- 빌드 

```
$ cd ~/ros2_ws/
$ colcon build --symlink-install --packages-select tutorial_action

```



### 6.2.6 실행

- Web Shell #1 : 액션 서버 fibonacci_srv_node 노드 실행

```
$ source ~/ros2_ws/install/local_setup.bash

$ ros2 run tutorial_action fibonacci_srv_node

```

- Web Shell #2 : 액션 클라이언트 fibonacci_cli_node 노드 실행

```
$ source ~/ros2_ws/install/local_setup.bash

$ ros2 run tutorial_action fibonacci_cli_node

```


### 6.2.7 Action Server 파일 요약  



```
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from 인터페이스라이브러리.action import 액션데이터형

class server클래스(Node):

    def __init__(self):
        super().__init__('svr노드명')
        self.srv = ActionServer(
            self,
            액션데이터형,
            '액션명',
            self.콜백함수)

    def 콜백함수(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # Feedback
        feedback = 액션데이터형.Feedback()
        feedback.데이터 = ......

        for i in range(1, goal_handle.request.데이터):
            (feedback.데이터 = ......)
            self.get_logger().info('Feedback: feedback.데이터 출력')
            goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()
        
        # Result
        result = 액션데이터형.Result()
        result.데이터 = ......
        self.get_logger().info('Result: result.데이터 출력')
        return result


def main(args=None):
    rclpy.init(args=args)

    obj = server클래스()

    rclpy.spin(obj)


if __name__ == '__main__':
    main()

```


### 6.2.8 Action Client 파일 요약

```
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from 인터페이스라이브러리.action import 액션데이터형


class client클래스(Node):

    def __init__(self):
        super().__init__('cli노드명')
        self.cli = ActionClient(self, 액션데이터형, '액션명')

    def send_goal(self, 데이터):
        goal = 액션데이터형.Goal()
        goal.데이터 = 데이터

        self.cli.wait_for_server()

        return self.cli.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)

    obj = client클래스()

    future = obj.send_goal(데이터 값)

    rclpy.spin_until_future_complete(obj, future)


if __name__ == '__main__':
    main()

```


