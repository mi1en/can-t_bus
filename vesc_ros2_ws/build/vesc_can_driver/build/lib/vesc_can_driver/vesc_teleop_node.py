# -*- coding: utf-8 -*-
"""
Teleop-нода для VESC: управление мотором с клавиатуры (WASD).

Публикует duty в топик команды (по умолчанию /vesc_can_node/cmd/duty),
повторяя его с заданной частотой, чтобы не сработал watchdog основной ноды.

Клавиши:
    W — вперёд (duty = +max)
    S — назад  (duty = -max)
    A — медленнее (max_duty -= шаг)
    D — быстрее   (max_duty += шаг)
    Пробел — стоп (duty = 0)
    Q / Esc — выход

Графики телеметрии — опционально, ВЫКЛЮЧЕНЫ по умолчанию (параметр enable_plot).
При enable_plot=False matplotlib не импортируется вообще.
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# кроссплатформенное чтение клавиш
try:
    import termios
    import tty
    import select
    _POSIX = True
except ImportError:
    import msvcrt
    _POSIX = False


HELP = """
=== VESC teleop (WASD) ===
  W — вперёд     S — назад
  A — медленнее  D — быстрее
  Пробел — стоп
  Q / Esc — выход
"""

def _open_tty():
    """Открыть /dev/tty, чтобы читать клавиши даже если stdin перенаправлен."""
    try:
        return open("/dev/tty", "r")
    except Exception:
        return sys.stdin

class VescTeleop(Node):
    def __init__(self):
        super().__init__("vesc_teleop")

        p = self.declare_parameter
        self.cmd_topic     = p("cmd_topic", "/vesc_can_node/cmd/duty").value
        self.telemetry_ns  = p("telemetry_ns", "/vesc_can_node/telemetry").value
        self.publish_rate  = p("publish_rate", 20.0).value      # Гц, чтобы не сработал watchdog
        self.start_max     = p("start_max_duty", 0.10).value
        self.duty_step     = p("duty_step", 0.05).value         # шаг A/D
        # графики: вариативно и по умолчанию выключено
        self.enable_plot   = p("enable_plot", False).value
        self.plot_window   = p("plot_window", 30.0).value       # сек на экране
        self.plot_fps      = p("plot_fps", 5.0).value           # Гц обновления графика

        self._pub = self.create_publisher(Float64, self.cmd_topic, 10)

        self._target = 0.0
        self._max = float(self.start_max)
        self._alive = True
        self._fig = None

        # повтор команды для watchdog
        self.create_timer(1.0 / max(1.0, self.publish_rate), self._republish)

        # телеметрия нужна только для графиков
        self._tele = {}
        if self.enable_plot:
            self._subscribe_telemetry()

        # чтение клавиатуры в отдельном потоке
        self._key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self._key_thread.start()

        self.get_logger().info(f"Teleop -> {self.cmd_topic}")
        print(HELP)

    # ── публикация ──
    def _republish(self):
        self._pub.publish(Float64(data=float(self._target)))

    def publish_stop(self):
        self._target = 0.0
        try:
            self._pub.publish(Float64(data=0.0))
        except Exception:
            pass

    # ── клавиатура ──
    def _getch(self):
        if _POSIX:
            r, _, _ = select.select([self._tty], [], [], 0.1)
            return self._tty.read(1) if r else None
        else:
            if msvcrt.kbhit():
                return msvcrt.getch().decode(errors="ignore")
            time.sleep(0.05)
            return None

    def _key_loop(self):
        self._tty = _open_tty()
        old = None
        if _POSIX and self._tty.isatty():
            old = termios.tcgetattr(self._tty)
            tty.setcbreak(self._tty.fileno())
        try:
            while self._alive:
                ch = self._getch()
                if ch is None:
                    continue
                self._handle_key(ch)
        finally:
            if old is not None:
                termios.tcsetattr(self._tty, termios.TCSADRAIN, old)

    def _handle_key(self, ch):
        k = ch.lower()
        if   k == "w": self._target = self._max
        elif k == "s": self._target = -self._max
        elif k == "a": self._max = max(0.0, round(self._max - self.duty_step, 3))
        elif k == "d": self._max = min(1.0, round(self._max + self.duty_step, 3))
        elif ch == " ": self._target = 0.0
        elif k == "q" or ch == "\x1b":
            self._quit(); return
        else:
            return
        # если шла команда, обновим её под новый max
        if self._target > 0:   self._target = self._max
        elif self._target < 0: self._target = -self._max
        print(f"\rduty {self._target:+.2f}  (макс {self._max:.2f})    ", end="", flush=True)

    def _quit(self):
        print("\nВыход, стоп мотора.")
        self.publish_stop()
        self._alive = False
        if self._fig is not None:
            try:
                import matplotlib.pyplot as plt
                plt.close(self._fig)
            except Exception:
                pass

    # ── спин ──
    def spin_loop(self):
        while rclpy.ok() and self._alive:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ── опциональные графики ──
    def _subscribe_telemetry(self):
        for name in ("rpm", "motor_current", "battery_current", "duty"):
            self.create_subscription(
                Float64, f"{self.telemetry_ns}/{name}",
                lambda msg, n=name: self._tele.__setitem__(n, msg.data), 10)

    def run_live_plot(self):
        """Лёгкий живой график (вызывается только при enable_plot=True)."""
        from collections import deque
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        maxlen = int(self.plot_fps * self.plot_window)
        ts = deque(maxlen=maxlen)
        d_rpm, d_cur, d_bat, d_duty = (deque(maxlen=maxlen) for _ in range(4))
        t0 = time.time()

        fig, ax = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        self._fig = fig
        fig.canvas.manager.set_window_title("VESC teleop — телеметрия")
        (ln_rpm,)  = ax[0].plot([], [], color="tab:blue")
        (ln_cur,)  = ax[1].plot([], [], color="tab:red", label="мотор")
        (ln_bat,)  = ax[1].plot([], [], color="tab:orange", label="батарея")
        (ln_duty,) = ax[2].plot([], [], color="tab:purple")
        ax[0].set_ylabel("об/мин")
        ax[1].set_ylabel("Ток, А"); ax[1].legend(loc="upper left", fontsize=8)
        ax[2].set_ylabel("Duty"); ax[2].set_xlabel("Время, с")
        for a in ax:
            a.grid(True, alpha=0.3)

        def update(_):
            now = time.time() - t0
            ts.append(now)
            d_rpm.append(self._tele.get("rpm", 0.0))
            d_cur.append(self._tele.get("motor_current", 0.0))
            d_bat.append(self._tele.get("battery_current", 0.0))
            d_duty.append(self._tele.get("duty", 0.0))
            x = list(ts)
            ln_rpm.set_data(x, d_rpm)
            ln_cur.set_data(x, d_cur)
            ln_bat.set_data(x, d_bat)
            ln_duty.set_data(x, d_duty)
            x0 = max(0.0, now - self.plot_window)
            for a in ax:
                a.set_xlim(x0, max(self.plot_window, now))
                a.relim(); a.autoscale_view(scalex=False, scaley=True)
            return ln_rpm, ln_cur, ln_bat, ln_duty

        self._ani = FuncAnimation(fig, update, interval=int(1000 / self.plot_fps),
                                  blit=False, cache_frame_data=False)
        plt.show()
        self._alive = False


def main(args=None):
    rclpy.init(args=args)
    node = VescTeleop()
    try:
        if node.enable_plot:
            spin = threading.Thread(target=node.spin_loop, daemon=True)
            spin.start()
            node.run_live_plot()        # блокирует, пока открыто окно
        else:
            node.spin_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
