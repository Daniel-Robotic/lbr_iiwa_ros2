# Контроллер webots для LBRiiwa 

Проект находиться на стадии разработки...

### Пакеты

- `iiwa_controller` - пакет с описанием контроллера робота и основного мира
- `iiwa_interfaces` - пакет с описание `srv` сообщений
- `lbr_description` - папка с [проекта](https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/rolling) на GitHub
- `robot_control` - изначальная версия `iiwa_controller`, планировалась реализовывать управление на Gazebo симуляторе


### Запуск

Для запуска произведите сборку пакета командой `colcon build` и обновите
свою среду `ROS2`

Выолните команду:

```bash
ros2 launch iiwa_controller kuka.launch.py 
```

Должен запуститься 3D движок webots.

### Публикаторы

- `/LBRiiwa7R800/current_positions/virtual` - текущее положение осей виртуального робота (тип сообщения: `trajectory_msgs/msg/JointTrajectoryPoint`)
- `/LBRiiwa7R800/cmd_positions` - необходим для отправки позиций робота (тип сообщения: `trajectory_msgs/msg/JointTrajectory`)
- `/LBRiiwa7R800/publish_data` - пример отправки траекторных перемещений (тип сообщения: `std_srv/srv/Empty`)
- `/LBRiiwa7R800/stop_move` - сервис для остановки робота (тип сообщения: `std_srv/srv/Trigger`)
- `/LBRiiwa7R800/change_trajectory` - сервис для отхода робота на заданное колличество шагов. Шаг выбирается опционально. Планируется за этот промежуток времени расчитывать ОЗК и продолжать дальнейшее движение (тип сообщения: `iiwa_interfaces/srv/ChangeTrajectory`)