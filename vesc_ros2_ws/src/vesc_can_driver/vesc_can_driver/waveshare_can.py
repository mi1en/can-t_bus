# -*- coding: utf-8 -*-
"""
Драйвер мотора VESC через адаптер Waveshare USB-CAN-A.

Чистая логика без UI/графиков — переиспользуется ROS 2 нодой.
Протоколы:
  * Waveshare USB-CAN-A: формат переменной длины  AA [type] [ID LE] [data] 55
  * VESC CAN: extended ID = (command << 8) | vesc_id, данные = int32 big-endian * scale
"""

import struct
import time
import threading

import serial   # pip install pyserial


# Коды CAN-команд VESC (datatypes.h) и масштабы (comm_can.c)
CAN_PACKET_SET_DUTY          = 0    # duty,    масштаб 100000
CAN_PACKET_SET_CURRENT       = 1    # ток A,   масштаб 1000
CAN_PACKET_SET_CURRENT_BRAKE = 2    # тормоз A, масштаб 1000
CAN_PACKET_SET_RPM           = 3    # eRPM,    масштаб 1

# id статус-кадров, которые VESC сам шлёт в шину
CAN_PACKET_STATUS   = 9     # eRPM, ток мотора, duty
CAN_PACKET_STATUS_4 = 16    # темп. FET, темп. мотора, ток батареи
CAN_PACKET_STATUS_5 = 27    # тахометр, напряжение (нужен Status 5 в VESC Tool)


class WaveshareUsbCanA:
    _SPEED_CODES = {
        1000000: 0x01, 800000: 0x02, 500000: 0x03, 400000: 0x04,
        250000:  0x05, 200000: 0x06, 125000: 0x07, 100000: 0x08,
        50000:   0x09, 20000:  0x0A, 10000:  0x0B, 5000:   0x0C,
    }

    def __init__(self, port, serial_baud, can_bitrate, extended=True):
        self.ser = serial.Serial(port, serial_baud, timeout=0.05)
        self.extended = extended
        self._configure(can_bitrate, extended)

    def _configure(self, can_bitrate, extended):
        frame = bytearray(20)
        frame[0] = 0xAA
        frame[1] = 0x55
        frame[2] = 0x12
        frame[3] = self._SPEED_CODES[can_bitrate]
        frame[4] = 0x02 if extended else 0x01
        frame[13] = 0x00
        frame[14] = 0x01
        frame[19] = sum(frame[2:19]) & 0xFF
        self.ser.write(frame)
        time.sleep(0.05)

    def send(self, can_id, data):
        dlc = len(data)
        type_byte = 0xC0 | (0x20 if self.extended else 0x00) | (dlc & 0x0F)
        frame = bytearray()
        frame.append(0xAA)
        frame.append(type_byte)
        frame += struct.pack("<I", can_id) if self.extended else struct.pack("<H", can_id)
        frame += bytes(data)
        frame.append(0x55)
        self.ser.write(frame)

    def read(self, n=256):
        return self.ser.read(n)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()


class Vesc:
    """Высокоуровневое управление с плавной рампой duty и фоновой телеметрией."""

    def __init__(self, adapter: WaveshareUsbCanA, vesc_id=1,
                 send_period=0.05, duty_ramp_rate=0.30, pole_pairs=7):
        self.adapter = adapter
        self.vesc_id = vesc_id
        self.send_period = send_period
        self.duty_ramp_rate = duty_ramp_rate
        self.pole_pairs = pole_pairs

        self._mode = "duty"          # 'duty' | 'current' | 'rpm' | 'brake'
        self._target = 0.0
        self._duty_out = 0.0
        self._lock = threading.Lock()

        self.telemetry = {}
        self._tlock = threading.Lock()

        self._running = True
        self._ctrl_thread = threading.Thread(target=self._control_loop, daemon=True)
        self._rx_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._ctrl_thread.start()
        self._rx_thread.start()

    # ── отправка ──
    def _eid(self, command):
        return (command << 8) | self.vesc_id

    def _tx(self, command, raw_value):
        self.adapter.send(self._eid(command), struct.pack(">i", int(raw_value)))

    def _control_loop(self):
        while self._running:
            with self._lock:
                mode, target = self._mode, self._target
                if mode == "duty":
                    step = self.duty_ramp_rate * self.send_period
                    if self._duty_out < target:
                        self._duty_out = min(target, self._duty_out + step)
                    elif self._duty_out > target:
                        self._duty_out = max(target, self._duty_out - step)
                    out = self._duty_out

            try:
                if mode == "duty":
                    self._tx(CAN_PACKET_SET_DUTY, out * 100000.0)
                elif mode == "current":
                    self._tx(CAN_PACKET_SET_CURRENT, target * 1000.0)
                elif mode == "rpm":
                    self._tx(CAN_PACKET_SET_RPM, target)
                elif mode == "brake":
                    self._tx(CAN_PACKET_SET_CURRENT_BRAKE, target * 1000.0)
            except (serial.SerialException, OSError):
                pass
            time.sleep(self.send_period)

    # ── приём телеметрии ──
    def _reader_loop(self):
        buf = bytearray()
        while self._running:
            try:
                chunk = self.adapter.read(256)
            except (serial.SerialException, OSError):
                time.sleep(0.1)
                continue
            if chunk:
                buf += chunk
                self._parse(buf)

    def _parse(self, buf):
        while True:
            start = buf.find(0xAA)
            if start < 0:
                buf.clear(); return
            if start > 0:
                del buf[:start]
            if len(buf) < 2:
                return
            t = buf[1]
            if (t & 0xC0) != 0xC0:
                del buf[0]; continue
            ext = bool(t & 0x20)
            dlc = t & 0x0F
            idlen = 4 if ext else 2
            total = 2 + idlen + dlc + 1
            if len(buf) < total:
                return
            if buf[total - 1] != 0x55:
                del buf[0]; continue
            cid = int.from_bytes(buf[2:2 + idlen], "little")
            data = bytes(buf[2 + idlen:2 + idlen + dlc])
            del buf[:total]
            self._decode_status(cid, data)

    def _decode_status(self, cid, data):
        cmd = (cid >> 8) & 0xFF
        vid = cid & 0xFF
        if vid != self.vesc_id:
            return
        upd = {}
        try:
            if cmd == CAN_PACKET_STATUS and len(data) >= 8:
                erpm = struct.unpack(">i", data[0:4])[0]
                upd.update(erpm=erpm, rpm=erpm / self.pole_pairs,
                           motor_current=struct.unpack(">h", data[4:6])[0] / 10.0,
                           duty=struct.unpack(">h", data[6:8])[0] / 1000.0)
            elif cmd == CAN_PACKET_STATUS_4 and len(data) >= 8:
                upd["temp_fet"]     = struct.unpack(">h", data[0:2])[0] / 10.0
                upd["temp_motor"]   = struct.unpack(">h", data[2:4])[0] / 10.0
                upd["batt_current"] = struct.unpack(">h", data[4:6])[0] / 10.0
            elif cmd == CAN_PACKET_STATUS_5 and len(data) >= 6:
                upd["voltage"] = struct.unpack(">h", data[4:6])[0] / 10.0
        except struct.error:
            return
        if upd:
            with self._tlock:
                self.telemetry.update(upd)

    def get_telemetry(self):
        with self._tlock:
            return dict(self.telemetry)

    # ── публичные команды ──
    def set_duty(self, duty):
        duty = max(-1.0, min(1.0, duty))
        with self._lock:
            self._mode = "duty"; self._target = duty

    def set_current(self, amps):
        with self._lock:
            self._mode = "current"; self._target = amps

    def set_rpm(self, erpm):
        with self._lock:
            self._mode = "rpm"; self._target = erpm

    def set_current_brake(self, amps):
        with self._lock:
            self._mode = "brake"; self._target = amps

    def stop(self):
        self.set_duty(0.0)

    def shutdown(self):
        self.stop()
        for _ in range(40):
            with self._lock:
                if abs(self._duty_out) < 1e-3:
                    break
            time.sleep(0.05)
        self._running = False
        self._ctrl_thread.join(timeout=1.0)
        self._rx_thread.join(timeout=1.0)
