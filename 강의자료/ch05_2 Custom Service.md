# 5_2 Python 예제 :  Custom Service 


- 5_2.1 Custom Service 데이터형 선언 
- 5_2.2 Custom Service 사용


## 5_2.1 Custom Service 데이터형 선언 

- Custom Topic 패키지    : my_msgs
- Custom Topic 데이터형  : my_msgs/srv/AddTwoInts

```
   # Request
   int32 a
   int32 b
   ---
   # Response
   int32 sum
   
```



### 5_2.1.1 Workspace 및 Package 생성

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


### 5_2.1.2 Custom 메시지 파일 AddTwoInts.srv 

```
$ cd ~/ros2_ws/src/my_msgs
$ rm -r include src
$ mkdir srv
$ cd srv


```

- 파일 AddTwoInts.srv

    ~/ros2_ws/src/my_msgs/srv/AddTwoInts.srv

```
int32 a
int32 b
---
int32 sum

```

### 5_2.1.3 설정 파일  CMakeLists.txt & package.xml

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



### 5_2.1.4 빌드 

- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install --packages-select my_msgs

``` 



### 5_2.1.5 인터페이스 확인 - msg 확인

- Web Shell #1
   
```
$ source ~/ros2_ws/install/local_setup.bash

$ ros2 interface list | grep my_msgs
my_msgs/msg/TwoInts
my_msgs/srv/AddTwoInts

$ ros2 interface show my_msgs/srv/AddTwoInts
int32 a
int32 b
---
int32 sum

```




## 5_2.2 Custom Service 사용


- 패키지              : custom_service 
- server노드명        : custom_service_srv_node 
- client노드명        : custom_service_cli_node 
- srv클래스           : srvCustomService
- cli클래스           : cliCustomService
- 서비스명            : add_two_ints
- 데이터형            : my_msgs/srv/AddTwoInts
   * int32 a
   * int32 b
   * ---
   * int32 sum
- srv콜백함수         : srv_custom_service_callback
- cli콜백함수         : cli_custom_service_callback
- srv객체             : srv_custom_service
- cli객체             : cli_custom_service

- random number 라이브러리 및 생성

   * 라이브러리 : random
   * 함      수 : random.randint(최소값, 최대값)


### 5_2.2.1 Workspace 및 Package 생성

   &#8251; 주의사항 : Package 생성 시 : 
   
      * --dependencies 옵션 : custom service 선언 패키지 my_msgs를 라이브러리로 사용 
      
```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create custom_service --build-type ament_python --dependencies rclpy my_msgs
$ cd custom_service
$ ls
custom_service     package.xml     resource     setup.cfg     setup.py     test

```


### 5_2.2.2 소스 파일  custom_service_srv.py, custom_service_cli.py 


- 파일 custom_service_srv.py

    ~/ros2_ws/src/custom_service/custom_service/custom_service_srv.py

```
import rclpy
from rclpy.node import Node 
from my_msgs.srv import AddTwoInts

class srvCustomService(Node):
  def __init__(self):
    super().__init__('custom_service_srv_node')
    self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.my_callback)
  
  def my_callback(self, request, response):
    self.get_logger().info('Srv : a={0}, b={1}'.format(request.a, request.b)) 
    response.sum = request.a + request.b
    return response

def main(args=None):
    rclpy.init(args=args)               
    obj = srvCustomService()
    try:
        rclpy.spin(obj)
    except KeyboardInterrupt:
        obj.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        obj.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

```


- 파일 custom_service_cli.py

    ~/ros2_ws/src/custom_service/custom_service/custom_service_cli.py 

```
import rclpy
from rclpy.node import Node 
from my_msgs.srv import AddTwoInts
import random

class cliCustomService(Node):
  def __init__(self):
    super().__init__('custom_service_cli_node')
    self.cli = self.create_client(AddTwoInts, 'add_two_ints')

    while not self.cli.wait_for_service(timeout_sec=0.1):
        self.get_logger().info('waiting')
    
    self.req = AddTwoInts.Request()

  def send_request(self):
    self.req.a = random.randint(1, 6)
    self.req.b = random.randint(1, 6)
    self.future = self.cli.call_async(self.req)           # Asynchronous Service Client
    rclpy.spin_until_future_complete(self, self.future)
    return self.future.result()

def main(args=None):
  rclpy.init(args=args)
  obj = cliCustomService() 
  res = obj.send_request()
  obj.get_logger().info('Cli : a={0}, b={1}, sum={2}'.format(obj.req.a, obj.req.b, res.sum))

  obj.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()

```



### 5_2.2.3 설정 파일  setup.py

- 파일 setup.py

    추가 : 'custom_service_srv_node = custom_service.custom_service_srv:main',
    
    추가 : 'custom_service_cli_node = custom_service.custom_service_cli:main',

```
from setuptools import setup

package_name = 'custom_service'

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
            'custom_service_srv_node = custom_service.custom_service_srv:main',
            'custom_service_cli_node = custom_service.custom_service_cli:main',
        ],
    },
)

```



### 5_2.2.4 빌드 

- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --symlink-install --packages-select custom_service

``` 


### 5_2.2.5 실행 

- Web Shell #1

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run custom_service custom_service_srv_node

```

- Web Shell #2

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run custom_service custom_service_cli_node 

```

- Web Shell #3

```
$ rqt_graph

```



```python

```
