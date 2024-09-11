------

### 의의 

- Rviz 상에서 [Goal pose icon] 으로 목적지를 선택하면  이에 기반해 [piecewise 경로를 생성]하고 이를 [pure pursuit 방법과 PD controller]를 사용하여 경로 추종하는 Simple path tracker.  



### 작동 커맨드

1. 터틀봇과 Rviz가 사전에 켜져있는 상태

2. `ros2 launch path_tracker path_tracker.launch.py `  
3. Rviz 상에서 fixed frame:odom 설정후  [goal pose icon] 클릭 후 경로를 생성하고 싶은 방향으로 click. (Rviz 상에서 경로(path type) 및 lookahead 위치(marker type) 확인 가능)

3. `ros2 service call /docking_activate std_srvs/srv/SetBool data:\ true\`  # true: 트랙킹 시작, false: 트랙킹 정지



### 노드 구성

1. path_generator_node: piecewise 경로 생성
2. pure_pursuit_node : pure pursuit 방법과 PD controller로  생성된 경로 추종



### 토픽,서비스,파라미터 구성

#### 	path_generator_node

​		**Subscribe topic:** /odom, /goal_pose

​		**Publish topic:** /path	

​		**Parameter:** qos_depth(int8_t), aligning_path_length(double)

####  	pure_pursuit_node

​		**Subscribe topic:** /odom, /path

​		**Publish topic:** /cmd_vel, /lookahead(For Rviz)

​		**Service:** /docking_activate

​		**Parameter:** qos_depth(int8_t), p_gain(double), d_gain(double), lookahead_distance(double), max_linear_speed(double), max_angular_speed(double)  



----

END.



