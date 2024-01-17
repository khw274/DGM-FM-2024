# 5 서비스 프로그래밍 (파이썬) 

- 패키지            : tutorial_service
- 서비스 서버       : service_server
- 서비스 클라이언트 : service_client 
- 서비스            : /empty_service 


### 5.1 Workspace 생성 

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create tutorial_service --build-type ament_python --dependencies rclpy std_msgs
$ cd tutorial_service
$ ls
package.xml     resource     setup.cfg     setup.py     test     tutorial_service

```



### 5.2 소스 파일  service_srv.py, service_cli.py 


- 파일 service_srv.py

    ~/ros2_ws/src/tutorial_service/tutorial_service/service_srv.py

```
import rclpy
from rclpy.node import Node 
from std_srvs.srv import Empty

class EmptyServer(Node):
    def __init__(self):
        super().__init__('service_srv_node') 
        self.srv = self.create_service(Empty, '/empty_service', self.my_callback)

    def my_callback(self, request, response):
        self.get_logger().info('Srv : service called')
        return response

def main(args=None):              
    rclpy.init(args=args)               
    node = EmptyServer()
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


- 파일 service_cli.py 

    ~/ros2_ws/src/tutorial_service/tutorial_service/service_cli.py

```
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class EmptyClient(Node):

    def __init__(self):
        super().__init__('service_cli_node')
        self.cli = self.create_client(Empty, '/empty_service')

        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('waiting') 
        
        self.req = Empty.Request()
    
    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
def main(args=None):              
    rclpy.init(args=args)               
    node = EmptyClient()
    res = node.send_request()
    node.get_logger().info('client: send request')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```



### 5.3 설정 파일  package.xml, setup.py, setup.cfg

- 파일 package.xml 수정

    수정사항없음

```
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tutorial_service</name>
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

    - 추가 : 'service_srv_node = tutorial_service.service_srv:main',
    
    - 추가 : 'service_cli_node = tutorial_service.service_cli:main',

```
from setuptools import setup

package_name = 'tutorial_service'

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
            'service_srv_node = tutorial_service.service_srv:main',
            'service_cli_node = tutorial_service.service_cli:main',
        ],
    },
)

```


- 파일 setup.cfg

    수정사항 없음

```
[develop]
script-dir=$base/lib/tutorial_service
[install]
install-scripts=$base/lib/tutorial_service

```



### 5.4 빌드

```
$ cd ~/ros2_ws/
$ colcon build --packages-select tutorial_service

```



### 5.5 실행

- Web Shell #1 : 서비스 서버 service_srv_node 노드 실행

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run tutorial_service service_srv_node

```

- Web Shell #2 : 서비스 클라이언트 service_cli_node 노드 실행

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run tutorial_service service_cli_node

```



### 5.6 Service Server 노드 요약 

```
import rclpy
from rclpy.node import Node 
from std_srvs.srv import 서비스데이터형

class server클래스(Node):
    def __init__(self):
        super().__init__('srv노드명') 
        self.srv = self.create_service(서비스데이터형, '서비스명', self.콜백함수)

    def 콜백함수(self, request, response):
        ...... (request.데이터 사용하여, response.데이터 계산)
        return response

def main(args=None):
    rclpy.init(args=args)               
    node = server클래스()
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




### 5.7 Service Client 노드 요약 


```
import rclpy
from rclpy.node import Node
from std_srvs.srv import 서비스데이터형

class client클래스(Node):

    def __init__(self):
        super().__init__('cli노드명')
        self.cli = self.create_client(서비스데이터형, '서비스명')

        while not self.cli.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('waiting') 
        
        self.req = 서비스데이터형.Request()
    
    def send_request(self):
        ...... (self.req.데이터: 내용 채움)
        self.future = self.cli.call_async(self.req)           # Asynchronous Service Client 
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
def main(args=None):              
    rclpy.init(args=args)               
    node = client클래스()
    res = node.send_request()
    node.get_logger().info('Cli : node.req.데이터 & res.데이터 출력'))
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

```




### 5.8 비동기식 Service Client vs 동기식 Service Client 

- 비동기식(asynchronous) Service Client : 추천 

- 동기식(synchronous) Service Client : 비추천 




```python

```
