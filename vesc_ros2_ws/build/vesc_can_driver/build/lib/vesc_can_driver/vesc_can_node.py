# -*- coding: utf-8 -*-
"""
ROS 2 нода управления мотором VESC через Waveshare USB-CAN-A.

Подписки (команды) — публикуй любую из них (std_msgs/Float64):
    ~/cmd/duty     duty   [-1.0 .. 1.0]
    ~/cmd/current  ток, А
    ~/cmd/erpm     электрические об/мин (eRPM = механ. RPM * pole_pairs)
    ~/cmd/brake    тормозной ток, А

Публикации (телеметрия, std_msgs/Float64, по умолчанию 20 Гц):
    ~/telemetry/rpm              механические об/мин
    ~/telemetry/erpm             электрические об/мин
    ~/telemetry/motor_current    ток мотора, А
    ~/telemetry/battery_current  ток батареи, А
    ~/telemetry/duty             фактический duty
    ~/telemetry/temp_fet         темп. ключей, °C
    ~/telemetry/temp_motor       темп. мотора, °C
    ~/telemetry/voltage          напряжение батареи, В (если включён Status 5)

Безопасность: watchdog — если новых команд нет дольше cmd_timeout секунд,
мотор плавно останавливается (0 — отключить watchdog).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from vesc_can_driver.waveshare_can import WaveshareUsbCanA, Vesc


class VescCanNode(Node):
    def __init__(self):
        super().__init__("vesc_can_node")

        # ── параметры ──
        p = self.declare_parameter
        self.serial_port    = p("serial_port", "/dev/ttyUSB0").value
        self.serial_baud    = p("serial_baud", 2000000).value
        self.can_bitrate    = p("can_bitrate", 500000).value
        self.vesc_id        = p("vesc_id", 1).value
        self.pole_pairs     = p("pole_pairs", 7).value
        self.send_period    = p("send_period", 0.05).value
        self.duty_ramp_rate = p("duty_ramp_rate", 0.30).value
        self.telemetry_rate = p("telemetry_rate", 20.0).value
        self.cmd_timeout    = p("cmd_timeout", 0.5).value   # сек; 0 = выкл
        self.cmd_vel_scale = p("cmd_vel_scale", 1.0).value

        # ── драйвер ──
        self.get_logger().info(
            f"Открываю {self.serial_port} @ {self.serial_baud}, CAN {self.can_bitrate}, "
            f"VESC ID {self.vesc_id}")
        try:
            self.adapter = WaveshareUsbCanA(
                self.serial_port, self.serial_baud, self.can_bitrate, extended=True)
        except Exception as e:
            self.get_logger().fatal(f"Не удалось открыть порт {self.serial_port}: {e}")
            raise
        self.vesc = Vesc(self.adapter, vesc_id=self.vesc_id,
                         send_period=self.send_period,
                         duty_ramp_rate=self.duty_ramp_rate,
                         pole_pairs=self.pole_pairs)

        # ── подписки на команды ──
        self.create_subscription(Float64, "~/cmd/duty",    self._on_duty,    10)
        self.create_subscription(Float64, "~/cmd/current", self._on_current, 10)
        self.create_subscription(Float64, "~/cmd/erpm",    self._on_erpm,    10)
        self.create_subscription(Float64, "~/cmd/brake",   self._on_brake,   10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        # ── публикации телеметрии ──
        self._pubs = {
            name: self.create_publisher(Float64, f"~/telemetry/{name}", 10)
            for name in ("rpm", "erpm", "motor_current", "battery_current",
                         "duty", "temp_fet", "temp_motor", "voltage")
        }
        self._tele_key = {
            "rpm": "rpm", "erpm": "erpm", "motor_current": "motor_current",
            "battery_current": "batt_current", "duty": "duty",
            "temp_fet": "temp_fet", "temp_motor": "temp_motor", "voltage": "voltage",
        }

        # ── таймеры ──
        self._last_cmd = self.get_clock().now()
        self.create_timer(1.0 / max(1.0, self.telemetry_rate), self._publish_telemetry)
        if self.cmd_timeout > 0:
            self.create_timer(0.1, self._watchdog)

        self.get_logger().info("VESC нода запущена.")

    # ── колбэки команд ──
    def _touch(self):
        self._last_cmd = self.get_clock().now()

    def _on_duty(self, msg):
        self.vesc.set_duty(msg.data); self._touch()

    def _on_current(self, msg):
        self.vesc.set_current(msg.data); self._touch()

    def _on_erpm(self, msg):
        self.vesc.set_rpm(msg.data); self._touch()

    def _on_brake(self, msg):
        self.vesc.set_current_brake(msg.data); self._touch()
    def _on_cmd_vel(self, msg: Twist):
        duty = max(-1.0, min(1.0, msg.linear.x * self.cmd_vel_scale))
        self.vesc.set_duty(duty); self._touch()
    # ── watchdog ──
    def _watchdog(self):
        dt = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self.vesc.stop()

    # ── телеметрия ──
    def _publish_telemetry(self):
        t = self.vesc.get_telemetry()
        for topic, key in self._tele_key.items():
            if key in t:
                self._pubs[topic].publish(Float64(data=float(t[key])))

    # ── завершение ──
    def destroy_node(self):
        try:
            self.vesc.shutdown()
            self.adapter.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VescCanNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
