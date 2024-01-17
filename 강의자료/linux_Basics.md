Linux / Ubuntu 

---

Tips 

- 명령어 관련 자동완성 및 탐색  
    - [Tab] key : 자동완성     
        ......(작성 중)[Tab] key
    - &uarr; key : 이전 명령어 탐색     
        &uarr; key 
    - &darr; key : 이후 명령어 탐색     
        &darr; key 

- 디렉토리 관련 특수 문자     
    / : root directory     
    ~ : user home directory     
    .. : parent directory     
    . : current directory   
    
- 파일/디렉토리 이름 특수 문자     
    \* : 임의의 모든 문자열     
    ? : 임의의 한 문자
---



# 1 Linux 설치 

### 1.1a theconstructsim ROSject 직접 생성 시  

- ROS Distro = ROS2 Humble 선택 ---> ROS2 Humble 자동 설치됨 
    - ROS2 Humble에 매칭되는 Ubuntu 22.04 자동 설치됨 


### 1.1b ROSject "JYSAH's Turtlebot3 Humble" fork 시

- "JYSAH's Turtlebot3 Humble" fork ---> ROS2 Humble & Turtlebot3 설치되어 있음 
    - ROS2 Humble에 매칭되는 Ubuntu 22.04 자동 설치됨 


### 1.2 설치 프로그램 버전 확인 

- Ubuntu 버전 확인 

```
$ cat /etc/issue 
Ubuntu 22.04.2 LTS \n \l

```

- Jammy : Ubuntu 22.04 LTS 버전을 Jammy 이라 부름


### 1.4 Desktop Ubuntu 22.04와 theconstructsim Ubuntu 22.04 차이점 

- Desktop Ubuntu 22.04 
    - 명령창 생성 : [Ctrl]+[Alt]+t key 
    - 명령창 닫기 : [Ctrl]+[shift]+w key
    - 명령창 내 복사하기 : [Ctrl]+[shift]+c key
    - 명령창 내 붙여넣기 : [Ctrl]+[shift]+v key

- theconstructsim Ubuntu 22.04
    - 명령창 생성 : "Web shell" 아이콘으로 열고, "+" 탭을 눌러 명령창을 추가함 
    - 명령창 닫기 : 명령창 우측 상단 "x" 아이콘 
    - 명령창 내 복사하기 : [Ctrl]+c key
    - 명령창 내 붙여넣기 : [Ctrl]+v key


# 2 Linux 종류 

- Linux 
    - 커널 : Linux 커널 
    - 운영 체제 : GNU/Linux (Linux Kernel + GNU Libraries/Tools)
        - GNU[gnu:] : 자유소프트웨어

- Debian : 
    - Linux 커널 탑재
    - apt 도입, 의존성(dependency) 관리 용이, 소프트웨어 설치/관리/제거 용이 
    
- Ubuntu 
    - Debian 기반 (apt 계승) 
    - GUI 사용 편의성 
    - 플러그 앤 플레이 (하드웨어 자동 인식)
    - Heavy (적용 기종: Desktop, Notebook, (Jetson Nano))
    
- Raspberry Pi OS : 
    - RaspberryPi 보드 전용 운영 체제
    - Debian 기반 (apt 계승) 
    - 저성능 ARM CPU에 최적화 (적용기종: RaspberryPi)
    

# 3 Ubuntu 종류 

- Desktop : 
    - GUI 포함
    - 모니터/키보드 필요 
    - 데스크탑 PC에 주로 설치
    
- Server : 
    - GUI 불포함 
    - 모니터/키보드 불필요 (ssh 원격 접속)
    - 소형 Single Board Computer (Jetson Nano 등)에 주로 설치 


# 4 Shell 

- Shell
    - 명령어
    - 사용자와 Kernel 간의 커맨드라인 인터프리터 
    - 종류 
        - <b>sh</b> : Bourne shell (Original Shell of Linux)
        - <b>bash</b> : Bourne-again shell (Shell developed with GNU)
        - csh : C shell (Shell based C programming language)
        - ksh : csh 기능을 제공하면서 속도도 빠름


- Shell 확인 

```
$ echo $SHELL
    /bin/bash
```


# 5 Shell 명령어 

### 5.1 pwd (print working directory)

- 현 디렉토리 확인 

```
    $ pwd                  (현재 디렉토리 출력)
```

- 사례

```
    $ pwd
        /home/user
```

### 5.2 cd (change directory) 

- 디렉토리 이동

```
    $ cd 디렉토리          (디렉토리 경로 지정 필요)
    $ cd ~                 (사용자 홈 디렉토리로 이동) 
```

- 사례

```
    $ cd ~
    $ pwd
        /home/user
    $ cd /
    $ pwd
        /
    $ cd ~ 
    $ pwd
        /home/user
```


### 5.3 ls (list) 

- 파일/디렉토리 리스트 열거

```
    $ ls            (현재 폴더 아래 리스트 출력)
    $ ls -l         (현재 폴더 아래 long 포맷 리스트 출력)
    $ ls -a         (현재 폴더 아래 히든 파일 포함 리스트 출력)
    $ ls 디렉토리   (현재 폴더 아래 디렉토리 아래 리스트 출력)
```

- 사례

```
    $ ls
        a.sh  a_copy.sh  ai_ws  catkin_ws  notebook_ws  python  ros2_ws  simulation_ws  turtlebot3_ws  webpage_ws
    $ ls -l
        drwxr-xr-x 2 user user 4096 Dec  7  2020 ai_ws
        drwxr-xr-x 5 user user 4096 Feb  7 00:57 catkin_ws
        drwxr-xr-x 4 user user 4096 Feb  7 03:02 notebook_ws
        drwxr-xr-x 6 user user 4096 Feb  7 00:57 ros2_ws
        drwxr-xr-x 5 user user 4096 Dec  7  2020 simulation_ws
        drwxr-xr-x 2 user user 4096 Feb  7 00:57 webpage_ws
    $ ls -a
        .               .catkin_ws_python3  .python3_ws                ros2_ws
        ..              .clangd             .ros                       simulation_ws
        .__bashrc       .gazebo             .rviz                      webpage_ws
        .bash_aliases   .history            .sudo_as_admin_successful  
        .bashrc         .ignition           .theia                     
        .bashrc_bridge  .labelImg           .yarn                      
        .bashrc_ros1    .npmrc              ai_ws                      
        .bashrc_ros2    .py3venv            catkin_ws                  
        .cache          .pylintrc           notebook_ws
```


### 5.4 mkdir (make directory)

- 디랙토리 생성 

```
    $ mkdir 디렉토리      (현재 폴더 밑에 디렉토리 생성)
    $ mkdir -p 디렉토리1/디렉토리2   (현재 폴더 밑에 디렉토리1 생성 후, 디렉토리1 밑에 디렉토리2 생성)
```

- 사례

```
$ cd ~
$ ls
    a.sh  a_copy.sh  ai_ws  catkin_ws  notebook_ws  ros2_ws  simulation_ws  turtlebot3_ws  webpage_ws
$ mkdir python
$ ls 
    a.sh  a_copy.sh  ai_ws  catkin_ws  notebook_ws  python  robot_ws  ros2_ws  simulation_ws  turtlebot3_ws  webpage_ws
$ cd python
$ pwd
    /home/user/python
```


### 5.5 touch (빈 파일 만들기) 및 파일 작성

- 빈 파일 만들기 

```
    $ touch 파일명         (빈 파일 만들기) 
```


- 사례 

```
    $ cd ~/python
    $ touch test_file.py
    $ ls 
        test_file.py
```

- 파일 내용 채우기 
    - "Code editor" 아이콘 클릭 
    - tree : python > test_file.py 선택
    - 파일 내용 입력

```
        a = 10
        b = 20
        c = a + b
        print("{0} + {1} = {2}".format(a, b, c)) 
```

- 파이썬 파일 실행 

```
    $ cd ~/python
    $ python3 test_file.py
        10 + 20 = 30
```


### 5.6 cp (copy)

- 복사 

```
    $ cp 파일1 파일2            (파일1을 파일2로 복사)
    $ cp 파일 디렉토리          (파일을 디렉토리 밑으로 복사, 디렉토리 경로 지정 필요)  
```

- 사례 

```
    $ cp test_file.py test_file02.py
    $ ls 
        test_file.py  test_file02.py
```


### 5.7 mv (move)

- 파일/디렉토리 이동 

```
    $ mv 파일1 파일2             (파일1을 파일2로 rename)
    $ mv 파일 디렉토리           (파일을 디렉토리로 이동, 디렉토리 경로 지정 필요) 
    $ mv 디렉토리1 디렉토리2     (디렉토리2가 없었으면, 디렉토리1을 디렉토리2로 rename, 있었으면 디렉토리1을 디렉토리2 밑으로 이동) 
```

-사례 

```
    $ cd ~/python
    $ mkdir subtest
    $ ls
        subtest  test_file.py   test_file02.py
    $ mv subtest sub_test
    $ ls
        sub_test  test_file.py   test_file02.py
    $ mv test_file02.py sub_test
    $ ls 
        sub_test  test_file.py
```


### 5.8 cat (concatenate)

- 파일 이어서 붙이기 / 파일 화면에 출력하기 

```
    $ cat 파일1 파일2 > 파일3    (파일1과 파일2 내용을 이어서, 파일3에 새로 붙여넣기) 
    $ cat 파일1 파일2 >> 파일3   (파일1과 파일2 내용을 이어서, 파일3 내용 아래에 이어 붙이기) 
    $ cat 파일                   (파일 내용을 화면에 출력하기)
```

- 사례 

```
    $ cat test_file.py
        a = 10
        b = 20
        c = a + b
        print("{0} + {1} = {2}".format(a, b, c)) 
```






### 5.9 rm (remove) 

- 삭제

```

```



### 5.10 rmdir (remove directory)

```
```


### 5.9 clear

```
```


### 5.10 find

```
```


### 5.11 | grep 

```
```


# 6 추가 Tips 

- shell 명령어 실행 결과를 다른 shell 명령어의 인자로 보내는 방법 
    - 역따옴표 : \`shell 명령어\`
    - 달러표 : $(shell 명령어)

```
    $ ls `cat file_list.txt`
    또는 
    $ ls $(cat file_list.txt)
```


- 프롬프트 종류 

```
    $ : 일반 유저 프롬프트 
    # : root 유저 프롬프트 
```
    
- 다중 명령어 ; && || |

```
    $ 명령어1; 명령어2       (다중 명령어, 앞 명령어의 성공/실패 여부와 상관없이 뒷 명령어 수행)
    $ 명령어1 && 명령어2     (다중 명령어, 앞 명령어 성공 시에만 뒷 명령어 수행)
    $ 명령어1 || 명령어2     (다중 명령어, ;와 동일)
    $ 명령어1 | 명령어2      (파이프라인, 앞 명령어 결과를 뒷 명령어 입력으로 사용 )
```

- \> \>\> < << <<<

```
    $ 명령어 > filename      (명령어 출력 결과를 filename에 저장)
    $ 명령어 >> filename     (명령어 출력 결과를 filename 내용 밑에 이어 쓰기)
    $ 명령어 < filename      (filename 내용을 명령어 입력으로 사용)
    $ 명령어 << END          (명령어에 multiline으로 입력을 보냄)
        $ cat << END
        > hello
        > world
        > END
    $ 명령어 <<< "문자열"    (string 형식의 문자열을 명령어의 입력으로 사용)
```   
    
    
