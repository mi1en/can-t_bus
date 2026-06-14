# vesc_can_driver (ROS 2 Jazzy)

Управление мотором **Flipsky 6374 190KV (VESC)** по CAN через адаптер
**Waveshare USB-CAN-A**. Пакет `ament_python`, без кастомных сообщений —
всё на `std_msgs/Float64`, работает с любым `ros2 topic`.

## Зависимости

```bash
sudo apt install ros-jazzy-rclpy ros-jazzy-std-msgs
pip install pyserial          # или: sudo apt install python3-serial
```

Доступ к COM-порту на Linux (иначе "permission denied"):
```bash
sudo usermod -aG dialout $USER   # затем перелогиниться
```

## Сборка

Положи пакет в воркспейс (он уже здесь: `vesc_ros2_ws/src/vesc_can_driver`) и собери:

```bash
cd ~/vesc_ros2_ws            # на Windows: cd C:\Users\plesh\vesc_ros2_ws
colcon build --packages-select vesc_can_driver
source install/setup.bash    # Windows: call install\setup.bat
```

## Запуск

```bash
ros2 launch vesc_can_driver vesc.launch.py
```

Свой порт/параметры — правь `config/vesc_params.yaml` (Linux `/dev/ttyUSB0`,
Windows `COM10`), либо передай свой файл:

```bash
ros2 launch vesc_can_driver vesc.launch.py params_file:=/path/to/my_params.yaml
```

Или ноду напрямую с инлайн-параметром:

```bash
ros2 run vesc_can_driver vesc_can_node --ros-args -p serial_port:=/dev/ttyUSB0
```

## Топики

Команды (публикуй любую, `std_msgs/Float64`):

| Топик                       | Смысл                              |
|-----------------------------|------------------------------------|
| `/vesc_can_node/cmd/duty`    | duty `[-1.0 .. 1.0]`               |
| `/vesc_can_node/cmd/current` | ток, А                             |
| `/vesc_can_node/cmd/erpm`    | eRPM (= механ. RPM × `pole_pairs`) |
| `/vesc_can_node/cmd/brake`   | тормозной ток, А                   |

Телеметрия (публикуется ~20 Гц, `std_msgs/Float64`):
`telemetry/rpm`, `erpm`, `motor_current`, `battery_current`, `duty`,
`temp_fet`, `temp_motor`, `voltage`.

> `voltage` появится, только если в VESC Tool включён **Status 5**
> (App Settings → General → CAN Messages Rate).

## Примеры

Плавно поехать вперёд на 10% и смотреть обороты:

```bash
ros2 topic pub /vesc_can_node/cmd/duty std_msgs/msg/Float64 "{data: 0.10}" -r 10
ros2 topic echo /vesc_can_node/telemetry/rpm
```

Стоп:

```bash
ros2 topic pub --once /vesc_can_node/cmd/duty std_msgs/msg/Float64 "{data: 0.0}"
```

## Teleop с клавиатуры (WASD)

Нода `vesc_teleop_node` рулит мотором с клавиатуры и публикует в `cmd/duty`.
**Запускать в своём терминале** (нужен интерактивный ввод):

```bash
# в одном терминале — драйвер:
ros2 launch vesc_can_driver vesc.launch.py
# в другом — teleop:
ros2 run vesc_can_driver vesc_teleop_node
```

Клавиши: **W** вперёд, **S** назад, **A** медленнее, **D** быстрее,
**Пробел** стоп, **Q/Esc** выход.

Teleop повторяет команду с частотой `publish_rate` (20 Гц) — watchdog доволен.

### Графики (опционально, по умолчанию ВЫКЛ)

Живой график телеметрии — отдельная вариативная функция. Включается параметром:

```bash
ros2 run vesc_can_driver vesc_teleop_node --ros-args -p enable_plot:=true
```

При `enable_plot:=false` (по умолчанию) `matplotlib` вообще не импортируется —
никакой нагрузки. Параметры графика: `plot_fps` (5 Гц), `plot_window` (30 с).

Можно поднять драйвер + teleop одной командой (teleop откроется в окне `xterm`,
поэтому нужен установленный `xterm`):

```bash
ros2 launch vesc_can_driver teleop.launch.py                 # без графиков
ros2 launch vesc_can_driver teleop.launch.py enable_plot:=true
```

## Безопасность

- **Watchdog**: если команд нет дольше `cmd_timeout` (по умолч. 0.5 с) — мотор
  плавно тормозит. Поэтому команды надо публиковать с частотой (флаг `-r`),
  как делает teleop. Поставь `cmd_timeout: 0.0`, чтобы отключить.
- При остановке ноды (Ctrl+C) duty плавно сводится к нулю, затем порт закрывается.
- Первый запуск — мотор на весу/без нагрузки, малый `duty`.
