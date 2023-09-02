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
catkin_make
source devel/setup.bash
roslaunch ur5_run ur5_run.launch
```

**1.1. Отдельное подключение камеры**

Чтобы работал просмотр с камер, необходимо:

1) На роботе запустить publisher:
```
cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch camera_pub camera.launch
```

2) Запустить на отдельном компьютере ноду для просмотра изображений с камер. 


**1.2. Отдельное подключение гриппера**

1) На роботе запустить publisher:

```
cd /home/administrator/rubleva/ur5_husky_api
catkin_make
source devel/setup.bash
roslaunch gripper_move gripper.launch
```

2) С другого компьютера отправлять данные для управления гриппером.


### 2. Автономное управление роботом UR5

1 терминал
<code>cd /home/administrator/rubleva/ur5_husky_api</code>
<code>catkin_make</code>
<code>source devel/setup.bash</code>
<code>roslaunch ur5_info ur5_info.launch</code> или <code>roslaunch ur5_info ur5_info.launch robot_ip:="127.0.0.1"</code>

2 терминал

Вызвать сервисы для управления гриппером или для управления манипулятором

а) Управление гриппером
<pre><code>cd /rubleva/ur5_husky_api
source devel/setup.bash
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.0" --once</code></pre>

Минимальное положение гриппера - 0, максимальное - 0.085

б) Управление манипулятором
<pre><code>cd /rubleva/ur5_husky_api
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

