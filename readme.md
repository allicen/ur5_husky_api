# Пакеты для запуска на роботе UR5

Используется для проекта <a href="https://github.com/allicen/UR5_TrajOpt">UR5_TrajOpt</a>

Структура проекта:

```
├── scripts
└── src
    ├── camera_pub
    ├── gripper_move
    ├── pkgs_for_datasets
    └── ur5_info

```

Пояснения: 

- **scripts** - пользовательские bash-скрипты для запуска определенных сценариев на роботе.
- **src/camera_pub** - пакет для трансляции изображений с камер робота.
- **src/gripper_move** - пакет для управления гриппером.
- **src/pkgs_for_datasets** - пакеты для сбора датасета с робота.
- **src/ur5_info** - пакет для управления роботом (позволяет управлять роботом без запуска основного проекта UR5_TrajOpt).

## Важная информация по роботу

На роботе установлен ROS Melodic.

Путь до проекта на роботе: <code>/home/administrator/rubleva/ur5_husky_api</code>. Все пакеты из этой папки необходимо загрузить на робота и запустить их.

Пакеты для сбора датасета лежат здесь: <code>pkgs_for_datasets</code>

Перед запуском <code>ur5_info.launch</code> нужно поменять ip робота в файле или передавать ip аргументом <code>robot_ip:="127.0.0.1"</code>.

Проект ```ur5_husky_api``` можно использовать для:

1. запуска необхолдимых функций на роботе (трансляция изображений с камеры, управление гриппером);
2. автономного управления манипулятором без подключения основного проекта UR5_TrajOpt. 

###  1. Запуск необходимых пакетов на роботе

**1.0. Запустить все одним файлом**

```
cd /home/administrator/rubleva/ur5_husky_api
catkin build
source devel/setup.bash
roslaunch ur5_run ur5_run.launch
```

**1.1. Отдельное подключение камеры**

Чтобы работал просмотр с камер, необходимо:

1) На роботе запустить publisher:
```
cd /home/administrator/rubleva/ur5_husky_api
catkin build
source devel/setup.bash
roslaunch camera_pub camera.launch
```

2) Запустить на отдельном компьютере ноду для просмотра изображений с камер. 


**1.2. Отдельное подключение гриппера**

1) На роботе запустить publisher:

```
cd /home/administrator/rubleva/ur5_husky_api
catkin build
source devel/setup.bash
roslaunch gripper_move gripper.launch
```

2) С другого компьютера отправлять данные для управления гриппером.


**1.3. Отдельное подключение камеры глубины для детектирования объектов**

1) На роботе запустить publisher:

```
cd /home/administrator/rubleva/ur5_husky_api
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
source devel/setup.bash
roslaunch camera_pub detect_obj_in_gripper.launch
```


### 2. Автономное управление роботом UR5

1 терминал
<code>cd /home/administrator/rubleva/ur5_husky_api</code>
<code>catkin_make</code>
<code>source devel/setup.bash</code>
<code>roslaunch ur5_info ur5_info.launch</code> или <code>roslaunch ur5_info ur5_info.launch robot_ip:="127.0.0.1"</code>

2 терминал

Вызвать сервисы для управления гриппером или для управления манипулятором

а) Управление гриппером
<pre><code>cd ~/rubleva/ur5_husky_api
source devel/setup.bash
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.0" --once</code></pre>

Минимальное положение гриппера - 0, максимальное - 0.085

б) Управление манипулятором
<pre><code>cd ~/rubleva/ur5_husky_api
source devel/setup.bash
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[]], delay: [], gripperAngle: 0.0}" --once</code></pre>


ВАЖНО!!!
***position и delay - передавать массивы одинаковой длины***

### Примеры команд

// схватить куб:
```
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.04" --once
```

// отпустить куб:
```
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.085" --once
```

// робот берет куб слева с пола:
```
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[2.326473,-0.553581,1.686786,-2.687753,-1.561592,-0.000419]], delay: [0], gripperAngle: 0.0}" --once
```

// робот в максимально сложенном положении (смотрит вниз):
```
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.606798,-3.091649,2.827192,-1.962667,-1.436540,-0.000551]], delay: [0], gripperAngle: 0.0}" --once
```

// робот в сложенном состоянии смотрит вперед:
```
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0], gripperAngle: 0.0}" --once
```

// робот смотрит на куб:
```
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.6089142004596155, 1.1463332176208496, -1.400895897542135, -1.5800517241107386, 0.00013182648399379104]], delay: [0], gripperAngle: 0.0}" --once
```

// робот у пола хвватает куб:
```
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.0789810419082642, -0.5552123228656214, 1.9236936569213867, -3.0894487539874476, -1.5335939566241663, 0.0014620755100622773]], delay: [0], gripperAngle: 0.0}" --once
```

// робот в положении, из которого скидывает куб:
```
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.3088954130755823, 1.4463391304016113, -2.300877873097555, -1.5800159613238733, 0.00014381069922819734]], delay: [0], gripperAngle: 0.0}" --once
```

// передать 2 положения:
```
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4775620698928833, -1.3088954130755823, 1.4463391304016113, -2.300877873097555, -1.5800159613238733, 0.00014381069922819734], position:[1.0789810419082642, -0.5552123228656214, 1.9236936569213867, -3.0894487539874476, -1.5335939566241663, 0.0014620755100622773]], delay: [0,0], gripperAngle: 0.0}" --once
```

Положение Даниила 

1.59592, -2.8721300000000003, 2.67727, -2.96106, -1.4762199999999996, 0.00031159000000000004

rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.59592, -2.8721300000000003, 2.67727, -2.96106, -1.4762199999999996, 0.00031159000000000004]], delay: [0], gripperAngle: 0.085}" --once


================= 24
rosbag record  /tf /tf_static /realsense_gripper/color/camera_info /realsense_gripper/color/image_raw/compressed /realsense_gripper/aligned_depth_to_color/camera_info /realsense_gripper/aligned_depth_to_color/image_raw /zed_node/left/camera_info /zed_node/left/image_rect_color/compressed /zed_node/depth/camera_info /zed_node/depth/depth_registered /velodyne_points /occupancy_grid_map/grid_map /arm/1/joint_states /gripper_angle
/gripper/1/joint_states /zed_node/right_raw/image_raw_color/compressed /zed_node/left_raw/image_raw_color/compressed /zed_node/right_raw/camera_info /zed_node/left_raw/camera_info


================= 24
rosbag record  /tf /tf_static /realsense_gripper/color/camera_info /realsense_gripper/color/image_raw/compressed /realsense_gripper/aligned_depth_to_color/camera_info /realsense_gripper/aligned_depth_to_color/image_raw /zed_node/left/camera_info /zed_node/left/image_rect_color/compressed /zed_node/depth/camera_info /zed_node/depth/depth_registered /arm/1/joint_states /gripper_angle /gripper/1/joint_states /zed_node/right_raw/image_raw_color/compressed /zed_node/left_raw/image_raw_color/compressed /zed_node/right_raw/camera_info /zed_node/left_raw/camera_info

rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[]], delay: [0], gripperAngle: 0.085}" --once
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.5958999395370483, -2.8721306959735315, 2.677290439605713, -2.9610708395587366, -1.4761622587787073, 0.00028762139845639467]], delay: [0], gripperAngle: 0.085}" --once


////////////////// 11.10

rosbag record /aruco_localizator_v2/objects \
            /command_robotiq_action/feedback \
            /command_robotiq_action/result \
            /command_robotiq_action/status \
            /current_pose \
            /husky_velocity_controller/odom \
            /object_pose \
            /pick_up_object/cancel \
            /pick_up_object/goal \
            /put_object/cancel \
            /put_object/goal \
            /segmentation_labels \
            /tf \
            /tf_static \
            /state/arm/0/arm_state \
            /joint_states \
            /tracked_objects_3d

Subscriptions:                        
 * /aruco_localizator_v2/objects [unknown type]
 * /command_robotiq_action/feedback [robotiq_2f_gripper_msgs/CommandRobotiqGripperActionFeedback]
 * /command_robotiq_action/result [robotiq_2f_gripper_msgs/CommandRobotiqGripperActionResult]
 * /command_robotiq_action/status [actionlib_msgs/GoalStatusArray]
 * /current_pose [unknown type]    
 * /husky_velocity_controller/odom [nav_msgs/Odometry]     
 * /object_pose [husky_tidy_bot_cv/ObjectPose]              
 * /pick_up_object/cancel [actionlib_msgs/GoalID]
 * /pick_up_object/goal [communication_msgs/PickupObjectActionGoal]
 * /put_object/cancel [actionlib_msgs/GoalID]               
 * /put_object/goal [communication_msgs/PutObjectActionGoal]
 * /segmentation_labels [husky_tidy_bot_cv/Categories]
 * /tf [tf2_msgs/TFMessage] 
 * /tf_static [tf2_msgs/TFMessage]                          
 * /tracked_objects_3d [husky_tidy_bot_cv/Objects3d]  
