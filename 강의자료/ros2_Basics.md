### Tips
- 명령어 관리 
    - 상하 화살표 : 이전 명령어 선택
    - [Tab] : 자동 완성
- 디렉토리 단축 표현 
    - ~ : 홈 디렉토리
    - . : 자신 디렉토리
    - .. : 상위 디렉토리
    = / : 루트 디렉토리


# 3. Test Program 

## 3.1 python 테스트 프로그램

* 작업영역 ros2_ws
* 패키지 pkg_python_test (~/ros2_ws/src/pkg_python_test)
* 파일 test_py.py (~/ros2_ws/src/pkg_python_test/pkg_python_test/test_py.py) 
* 파일 setup.py (~/ros2_ws/src/pkg_python_test/setup.py)


### 3.1.1 준비 작업

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create pkg_python_test --build-type ament_python --dependencies rclpy std_msgs
$ cd pkg_python_test
$ ls
package.xml     pkg_python_test     resource     setup.cfg     setup.py     test     
$ cd pkg_python_test
$ ls
__init__.py

```

### 3.1.2 소스 파일 편집 test_py.py

```
code editor: ~/ros2_ws/src/pkg_python_test/pkg_python_test/test_py.py
============
```

```
import rclpy
#from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('hello_world_node')
    #node = Node('hello_world_node')

    logger = node.get_logger()
    rate = node.create_rate(2) 
    while rclpy.ok():
        logger.info("Hello World!!!")
        #print("Hello World!!!")
        
        rclpy.spin_once(node)
        rate.sleep()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.1.3 설정 파일 편집 setup.py

```
추가 : 'test_py = pkg_python_test.test_py:main',

    entry_points={
        'console_scripts': [
            'test_py = pkg_python_test.test_py:main',
        ],
    },
```

### 3.1.4 빌드
- 워크스페이스 내의 모든 패키지 빌드

```
$ cd ~/ros2_ws/ 
$ colcon build  

```


- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --packages-select pkg_python_test

``` 

- 특정 패키지 및 의존성 패키지를 함께 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --packages-up-to pkg_python_test 

```

### 3.1.5 실행 ros2 run
- Web Shell

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run pkg_python_test test_py

```

### 3.1.6 실행 ros2 launch


#### 3.1.6.1 launch 파일 종류 :   \*.launch.py   \*.launch.xml(생략)   \*.launch.yaml(생략) 

##### [1/3] launch 파일 test_py.launch.py

```
$ cd ~/ros2_ws/src/pkg_python_test/
$ mkdir launch
$ cd launch
code editor: ~/ros2_ws/src/pkg_python_test/launch/test_py.launch.py
============
(test_py.launch.py 파일 작성 시, ExecuteProcess 또는 Node 방식 중 하나를 선택하여 작성)
```

###### A) ExecuteProcess 방식

```
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    # create launch description object
    ld = LaunchDescription()
    
    # execute process
    test_py_node = ExecuteProcess(
          cmd=["ros2", "run", "pkg_python_test", "test_py"], output="screen"
        )

    ld.add_action(test_py_node)

    # return launch description object
    return ld
```

###### B) Node 방식

```
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    # create launch description object
    ld = LaunchDescription()
    
    # node
    test_py_node = Node(
        package='pkg_python_test',
        executable='test_py',
        parameters=[],
        arguments=[],
        output="screen",
    )

    ld.add_action(test_py_node)

    # return launch description object
    return ld
```


##### [2/3] launch 파일 test_py.launch.xml

(생략)

##### [3/3]  launch 파일 test_py.launch.yaml

(생략)

#### 3.1.6.2 환경설정 파일 setup.py

```
$ cd ~/ros2_ws/src/pkg_python_test/
code editor: ~/ros2_ws/src/pkg_python_test/setup.py
============
```
```
추가: (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', 
                         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    ......

```

#### 3.1.6.3 Web Shell

```
$ cd ~/ros2_ws/ 
$ colcon build --packages-select pkg_python_test

$ source ~/ros2_ws/install/local_setup.bash
$ ros2 launch pkg_python_test test_py.launch.py

```





## 3.2 c++ 테스트 프로그램
* 작업영역 ros2_ws
* 패키지 pkg_cpp_test (~/ros2_ws/src/pkg_cpp_test)
* 파일 test_cpp.cpp (~/ros2_ws/src/pkg_cpp_test/src/test_cpp.cpp)
* 파일 CMakeLists.txt (~/ros2_ws/src/pkg_cpp_test/CMakeLists.txt)

### 3.2.1 준비 작업

```
$ cd ~
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src/

$ ros2 pkg create pkg_cpp_test --build-type ament_cmake --dependencies rclcpp std_msgs
$ cd pkg_cpp_test
$ ls
CMakeLists.txt     include     package.xml     src
$ cd src
$ ls
(없음)

```

### 3.2.2 소스 파일 편집 test_cpp.cpp

```
code editor: ~/ros2_ws/src/pkg_cpp_test/src/test_cpp.cpp
============
(test_cpp.cpp 파일 작성 시, 클래스를 이용안하거나 또는 이용하는 방식 중 하나를 선택하여 작성)
```

#### A) 클래스 이용 안함
```
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  //auto node = std::make_shared<rclcpp::Node>("hello_world_node");
  auto node = rclcpp::Node::make_shared("hello_world_node");

  rclcpp::WallRate loop_rate(2); // 루프 속도 설정 (2Hz)

  while(rclcpp::ok()) 
  {
    //RCLCPP_INFO("Hello World!!!");
    RCLCPP_INFO(node->get_logger(), "Hello World!!!");
    rclcpp::spin_some(node); 
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
```

#### B) 클래스 이용
```
#include "rclcpp/rclcpp.hpp"

class HelloWorldNode : public rclcpp::Node
{
public:
  HelloWorldNode() : Node("hello_world_node")
  {
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&HelloWorldNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    RCLCPP_INFO(get_logger(), "Hello World!!!");
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HelloWorldNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

 return EXIT_SUCCESS;
}
```


### 3.2.3 설정 파일 편집 CMakeLists.txt 

```
추가 : 

# find dependencies
......
find_package(rclcpp REQUIRED)
......

# Add executable
add_executable(test_cpp src/test_cpp.cpp)

# Ament target dependencies
ament_target_dependencies(test_cpp rclcpp)

# Link libraries
target_link_libraries(test_cpp)

install(TARGETS 
  test_cpp
  DESTINATION lib/${PROJECT_NAME}
)

......
```




### 3.2.4 빌드
- 워크스페이스 내의 모든 패키지 빌드

```
$ cd ~/ros2_ws/ 
$ colcon build  

```


- 특정 패키지만 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --packages-select pkg_cpp_test

``` 

- 특정 패키지 및 의존성 패키지를 함께 빌드 

``` 
$ cd ~/ros2_ws/ 
$ colcon build --packages-up-to pkg_cpp_test 

```

### 3.2.5 실행 ros2 run
- Web Shell

```
$ source ~/ros2_ws/install/local_setup.bash
$ ros2 run pkg_cpp_test test_cpp

```


### 3.2.6 실행 ros2 launch

#### 3.2.6.1 launch 파일 종류 :   \*.launch.py   \*.launch.xml   \*.launch.yaml

##### [1/3] launch 파일 test_cpp.launch.py

```
$ cd ~/ros2_ws/src/pkg_cpp_test/
$ mkdir launch
$ cd launch
code editor: ~/ros2_ws/src/pkg_cpp_test/launch/test_cpp.launch.py
============
```
```
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pkg_cpp_test',
            #namespace='',
            executable='test_cpp',
            name='test_cpp'
        )
    ])
```


##### [2/3] launch 파일 test_cpp.launch.xml

```
$ cd ~/ros2_ws/src/pkg_cpp_test/
$ mkdir launch
$ cd launch
code editor: ~/ros2_ws/src/pkg_cpp_test/launch/test_cpp.launch.xml
============
```
```
<launch>
  <node
    pkg="pkg_cpp_test"
    exec="test_cpp"
    name="test_cpp"
    output="screen">
  </node>
</launch>
```

##### [3/3] launch 파일 test_cpp.launch.yaml

```
$ cd ~/ros2_ws/src/pkg_cpp_test/
$ mkdir launch
$ cd launch
code editor: ~/ros2_ws/src/pkg_cpp_test/launch/test_cpp.launch.yaml
============
```
```
launch:

- node:
    pkg: "pkg_cpp_test"
    exec: "test_cpp"
    name: "test_cpp"
    #namespace: ""
```

#### 3.2.6.2 환경설정 파일 CMakeLists.txt

```
$ cd ~/ros2_ws/src/pkg_cpp_test/
code editor: ~/ros2_ws/src/pkg_cpp_test/CMakeLists.txt
============
```
```
추가:

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

```

#### 3.2.6.3 Web Shell

```
$ cd ~/ros2_ws/ 
$ colcon build --packages-select pkg_cpp_test

$ source ~/ros2_ws/install/local_setup.bash
$ ros2 launch pkg_cpp_test test_cpp.launch.py
$ ros2 launch pkg_cpp_test test_cpp.launch.xml
$ ros2 launch pkg_cpp_test test_cpp.launch.yaml

```



```python

```
