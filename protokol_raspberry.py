#!/usr/bin/env python3
"""
protokol_raspberry.py

Управляющая программа RPi 5 для станка ЭЭП (электро-эрозионная полировка
в сухом электролите).

━━ Быстрый старт ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  pip install smbus2
  sudo raspi-config → Interface Options → I2C → Enable
  i2cdetect -y 1          # должна быть точка на 0x08
  python3 protokol_raspberry.py

━━ Типичный цикл полировки ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  with EEPMachine() as m:
      m.check_connection()
      m.set_pulse_params(freq_hz=2000, duty_pct=40, level=100)
      m.set_gap_target(50.0)          # целевое U зазора 50В
      m.power_on()                    # включить силовое реле
      m.step_enable(True)             # включить DM556
      m.step_move(-500, speed=100)    # подвести инструмент к детали (500 шагов)
      m.step_wait()
      m.start_cycle()                 # запустить цикл (gap tracking + генератор)
      m.monitor(duration_s=60)        # мониторинг 60 секунд
      m.stop_cycle()
      m.step_goto(0, speed=300)       # отвести инструмент
      m.step_wait()
      m.power_off()

━━ Аварийная остановка ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  m.estop()               # всё выключить немедленно
  m.estop_reset()         # сбросить E-stop после устранения причины
"""

import time
import struct
import logging
from typing import Optional

try:
    from smbus2 import SMBus, i2c_msg
except ImportError:
    raise ImportError("pip install smbus2")

log = logging.getLogger(__name__)

#  Константы протокола 

PKT_START = 0xAA
PKT_MAX   = 32

# Команды
CMD_PING         = 0x01
CMD_GET_ALL      = 0x02
CMD_GET_SENSOR   = 0x03
CMD_SET_MODE     = 0x04
CMD_RESET        = 0x05
CMD_GET_STATUS   = 0x06
CMD_SET_DEVICE   = 0x10
CMD_GET_DEVICES  = 0x11
CMD_STEP_MOVE    = 0x20
CMD_STEP_GOTO    = 0x21
CMD_STEP_STOP    = 0x22
CMD_STEP_HOME    = 0x23
CMD_STEP_ENA     = 0x24
CMD_GET_STEPPER  = 0x25
CMD_SET_PULSE    = 0x30
CMD_GET_PULSE    = 0x31
CMD_SET_GAP_TGT  = 0x32
CMD_ESTOP        = 0x33
CMD_ESTOP_RESET  = 0x34
CMD_START_CYCLE  = 0x35
CMD_STOP_CYCLE   = 0x36

# Ответы
RESP_ACK          = 0xA1
RESP_SENSOR_DATA  = 0xA3
RESP_STATUS       = 0xA4
RESP_ERROR        = 0xA5
RESP_DEVICE_DATA  = 0xA6
RESP_STEP_STATUS  = 0xA7
RESP_PULSE_PARAMS = 0xA8

# Биты режима
MODE_SIM      = 0x01  # симуляция датчиков
MODE_AUTO_GAP = 0x02  # gap tracking активен
MODE_ESTOP    = 0x04  # E-stop активен
MODE_CYCLE    = 0x08  # цикл полировки активен

# Коды ошибок
ERROR_NAMES = {
    0x01: "unknown_cmd",
    0x02: "bad_crc",
    0x03: "sensor_na",
    0x04: "bad_len",
    0x05: "device_na",
    0x06: "e_stop_active",
    0x07: "not_ready",
}

STATUS_NAMES = {0x00: "ok", 0x01: "error", 0x02: "not_present"}

#  Типы датчиков ЭЭП 
SENSOR_TYPE_NAMES: dict[int, str] = {
    0x00: "None",
    0x01: "Gap Voltage",    # напряжение на зазоре
    0x02: "Discharge I",    # ток разряда
    0x03: "Temperature",    # температура
    0x04: "Vibration",      # амплитуда вибрации
    0x05: "Force",          # усилие прижима
    # 0x06: "NewSensor",
}
SENSOR_SCALES: dict[int, float] = {
    0x01: 10.0,    # raw/10 = В
    0x02: 1.0,     # мА
    0x03: 100.0,   # raw/100 = °C
    0x04: 1.0,     # мкм
    0x05: 1.0,     # г
}
SENSOR_UNITS: dict[int, str] = {
    0x01: "В",
    0x02: "мА",
    0x03: "°C",
    0x04: "мкм",
    0x05: "г",
}

#  Типы устройств 
DEVICE_TYPE_NAMES: dict[int, str] = {
    0x00: "None",
    0x01: "Relay",
    0x02: "PWM",
    0x03: "DigitalOut",
}
DEVICE_UNITS: dict[int, str] = {
    0x01: "",
    0x02: "/255",
    0x03: "",
}

#  Идентификаторы устройств (фиксированные, совпадают с Arduino) 
DEV_RELAY_MAIN = 0   # силовое реле разрядной цепи
DEV_RELAY_GEN  = 1   # реле генератора импульсов
DEV_VIBRO      = 2   # вибропривод (ШИМ 0..255)
DEV_FAN        = 3   # вентилятор охлаждения (ШИМ 0..255)


#  Утилиты пакета 

def _crc8(data: list[int]) -> int:
    crc = 0
    for b in data:
        crc ^= b
    return crc

def _build_packet(cmd: int, data: list[int] | None = None) -> list[int]:
    data = data or []
    payload = [cmd, len(data)] + data
    return [PKT_START] + payload + [_crc8(payload)]

def _parse_packet(raw: list[int]) -> Optional[dict]:
    if len(raw) < 4 or raw[0] != PKT_START:
        return None
    resp, dlen = raw[1], raw[2]
    if len(raw) < 3 + dlen + 1:
        return None
    data    = raw[3 : 3 + dlen]
    crc_got = raw[3 + dlen]
    if crc_got != _crc8([resp, dlen] + list(data)):
        log.warning("CRC ошибка: 0x%02X vs 0x%02X", crc_got, _crc8([resp, dlen] + list(data)))
        return None
    return {"type": resp, "data": list(data), "raw": raw}

def _parse_sensors(data: list[int]) -> list[dict]:
    result, i = [], 0
    while i + 4 < len(data):
        stype = data[i + 1]
        raw_v = struct.unpack(">h", bytes([data[i + 2], data[i + 3]]))[0]
        scale = SENSOR_SCALES.get(stype, 1.0)
        result.append({
            "id":         data[i],
            "type":       stype,
            "type_name":  SENSOR_TYPE_NAMES.get(stype, f"0x{stype:02X}"),
            "raw_value":  raw_v,
            "value":      raw_v / scale,
            "unit":       SENSOR_UNITS.get(stype, ""),
            "status":     data[i + 4],
            "status_str": STATUS_NAMES.get(data[i + 4], f"0x{data[i+4]:02X}"),
        })
        i += 5
    return result

def _parse_devices(data: list[int]) -> list[dict]:
    result, i = [], 0
    while i + 4 < len(data):
        dtype = data[i + 1]
        val   = struct.unpack(">h", bytes([data[i + 2], data[i + 3]]))[0]
        result.append({
            "id":         data[i],
            "type":       dtype,
            "type_name":  DEVICE_TYPE_NAMES.get(dtype, f"0x{dtype:02X}"),
            "value":      val,
            "unit":       DEVICE_UNITS.get(dtype, ""),
            "status":     data[i + 4],
            "status_str": STATUS_NAMES.get(data[i + 4], f"0x{data[i+4]:02X}"),
        })
        i += 5
    return result


#  Основной класс 

class EEPMachine:
    """
    Управляющий класс станка ЭЭП (электро-эрозионная полировка).
    I2C-мастер (RPi 5) ↔ Arduino Mega (слейв).
    """

    def __init__(self, bus: int = 1, addr: int = 0x08, read_delay_ms: float = 5.0):
        self._bus   = SMBus(bus)
        self._addr  = addr
        self._delay = read_delay_ms / 1000.0
        self._mode  = 0x00

    def close(self):
        self._bus.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    #  Низкоуровневый I2C 

    def _write(self, packet: list[int]) -> bool:
        try:
            self._bus.i2c_rdwr(i2c_msg.write(self._addr, packet))
            return True
        except OSError as e:
            log.error("I2C запись: %s", e)
            return False

    def _read(self, n: int = PKT_MAX) -> Optional[list[int]]:
        try:
            msg = i2c_msg.read(self._addr, n)
            self._bus.i2c_rdwr(msg)
            return list(msg)
        except OSError as e:
            log.error("I2C чтение: %s", e)
            return None

    def _cmd(self, cmd: int, data: list[int] | None = None) -> Optional[dict]:
        pkt = _build_packet(cmd, data)
        log.debug("TX → %s", " ".join(f"{b:02X}" for b in pkt))
        if not self._write(pkt):
            return None
        time.sleep(self._delay)
        raw = self._read()
        if raw is None:
            return None
        log.debug("RX ← %s", " ".join(f"{b:02X}" for b in raw))
        result = _parse_packet(raw)
        if result and result["type"] == RESP_ERROR and result["data"]:
            code = result["data"][0]
            log.warning("Arduino ошибка: %s (0x%02X)", ERROR_NAMES.get(code, "?"), code)
        return result

    def _ok(self, resp: Optional[dict]) -> bool:
        return resp is not None and resp["type"] == RESP_ACK

    #  Общие команды 

    def check_connection(self) -> bool:
        """Проверка связи с Arduino. Бросает RuntimeError если нет ответа."""
        resp = self._cmd(CMD_PING)
        ok = self._ok(resp)
        if not ok:
            raise RuntimeError("Arduino не отвечает. Проверь подключение (i2cdetect -y 1).")
        return True

    def ping(self) -> bool:
        return self._ok(self._cmd(CMD_PING))

    def get_status(self) -> Optional[dict]:
        """Возвращает dict(mode, mode_str, num_sensors, num_devices) или None."""
        resp = self._cmd(CMD_GET_STATUS)
        if resp is None or resp["type"] != RESP_STATUS:
            return None
        d = resp["data"]
        if len(d) < 3:
            return None
        mode = d[0]
        flags = []
        if mode & MODE_SIM:      flags.append("SIM")
        if mode & MODE_AUTO_GAP: flags.append("AUTO_GAP")
        if mode & MODE_ESTOP:    flags.append("E-STOP")
        if mode & MODE_CYCLE:    flags.append("CYCLE")
        self._mode = mode
        return {
            "mode":        mode,
            "mode_str":    "|".join(flags) if flags else "IDLE",
            "num_sensors": d[1],
            "num_devices": d[2],
            "estop":       bool(mode & MODE_ESTOP),
            "cycle_on":    bool(mode & MODE_CYCLE),
            "auto_gap":    bool(mode & MODE_AUTO_GAP),
        }

    def reset(self) -> bool:
        """Полный программный сброс (E-stop + reinit). НЕ перезагружает Arduino."""
        if self._ok(self._cmd(CMD_RESET)):
            self._mode = 0x00
            return True
        return False

    #  Датчики 

    def get_all_sensors(self) -> Optional[list[dict]]:
        """Запрос всех датчиков. В режиме SIM возвращает симулированные данные."""
        resp = self._cmd(CMD_GET_ALL)
        if resp is None or resp["type"] != RESP_SENSOR_DATA:
            return None
        return _parse_sensors(resp["data"])

    def get_sensor(self, sensor_id: int) -> Optional[dict]:
        resp = self._cmd(CMD_GET_SENSOR, [sensor_id & 0xFF])
        if resp is None or resp["type"] != RESP_SENSOR_DATA:
            return None
        s = _parse_sensors(resp["data"])
        return s[0] if s else None

    def get_gap_voltage(self) -> Optional[float]:
        """Напряжение на зазоре в Вольтах или None."""
        s = self.get_sensor(0)
        return s["value"] if s and s["status"] == 0 else None

    def get_discharge_current(self) -> Optional[float]:
        """Ток разряда в мА или None."""
        s = self.get_sensor(1)
        return s["value"] if s and s["status"] == 0 else None

    def get_temperature(self) -> Optional[float]:
        """Температура зоны обработки в °C или None."""
        s = self.get_sensor(2)
        return s["value"] if s and s["status"] == 0 else None

    # ── Режим симуляции ──────────────────────────────────────────────────

    def set_mode(self, mode: int) -> bool:
        resp = self._cmd(CMD_SET_MODE, [mode & 0xFF])
        if self._ok(resp):
            self._mode = mode & 0xFF
            return True
        return False

    def set_simulation(self, enable: bool) -> bool:
        """Включить/выключить симуляцию датчиков ЭЭП-процесса."""
        new = (self._mode | MODE_SIM) if enable else (self._mode & ~MODE_SIM & 0xFF)
        return self.set_mode(new)

    @property
    def simulation_active(self) -> bool:
        return bool(self._mode & MODE_SIM)

    # ── Устройства ───────────────────────────────────────────────────────

    def _set_device(self, dev_id: int, value: int) -> bool:
        packed = struct.pack(">h", max(-32768, min(32767, int(value))))
        return self._ok(self._cmd(CMD_SET_DEVICE, [dev_id & 0xFF, packed[0], packed[1]]))

    def get_devices(self) -> Optional[list[dict]]:
        resp = self._cmd(CMD_GET_DEVICES)
        if resp is None or resp["type"] != RESP_DEVICE_DATA:
            return None
        return _parse_devices(resp["data"])

    def power_on(self) -> bool:
        """Включить силовое реле разрядной цепи."""
        return self._set_device(DEV_RELAY_MAIN, 1)

    def power_off(self) -> bool:
        """Выключить силовое реле."""
        return self._set_device(DEV_RELAY_MAIN, 0)

    def set_vibration(self, level: int) -> bool:
        """Уставка вибропривода 0..255 (0 = стоп)."""
        return self._set_device(DEV_VIBRO, max(0, min(255, level)))

    def set_fan(self, level: int) -> bool:
        """Уставка вентилятора охлаждения 0..255."""
        return self._set_device(DEV_FAN, max(0, min(255, level)))

    # ── Шаговый двигатель Z ──────────────────────────────────────────────

    def step_enable(self, enable: bool) -> bool:
        """Включить/выключить драйвер DM556."""
        return self._ok(self._cmd(CMD_STEP_ENA, [1 if enable else 0]))

    def step_move(self, steps: int, speed: int = 0) -> bool:
        """Относительное перемещение: steps < 0 = подача, steps > 0 = отвод."""
        data  = list(struct.pack(">i", steps))
        data += list(struct.pack(">H", max(0, min(65535, speed))))
        return self._ok(self._cmd(CMD_STEP_MOVE, data))

    def step_goto(self, position: int, speed: int = 0) -> bool:
        """Абсолютная позиция в шагах от HOME."""
        data  = list(struct.pack(">i", position))
        data += list(struct.pack(">H", max(0, min(65535, speed))))
        return self._ok(self._cmd(CMD_STEP_GOTO, data))

    def step_stop(self) -> bool:
        """Немедленная остановка шагового."""
        return self._ok(self._cmd(CMD_STEP_STOP))

    def step_home(self) -> bool:
        """Обнулить счётчик позиции (без движения). Использовать после хоминга."""
        return self._ok(self._cmd(CMD_STEP_HOME))

    def get_stepper_status(self) -> Optional[dict]:
        """Состояние мотора: position, speed, running, enabled, direction."""
        resp = self._cmd(CMD_GET_STEPPER)
        if resp is None or resp["type"] != RESP_STEP_STATUS:
            return None
        d = resp["data"]
        if len(d) < 7:
            return None
        pos   = struct.unpack(">i", bytes(d[0:4]))[0]
        speed = (d[4] << 8) | d[5]
        flags = d[6]
        return {
            "position":  pos,
            "speed":     speed,
            "running":   bool(flags & 0x01),
            "enabled":   bool(flags & 0x02),
            "direction": "подача" if not (flags & 0x04) else "отвод",
        }

    def step_wait(self, poll_interval: float = 0.05, timeout: float = 60.0) -> Optional[int]:
        """
        Блокирует до завершения движения.
        Возвращает позицию или None (ошибка / таймаут).
        """
        t0 = time.monotonic()
        while True:
            st = self.get_stepper_status()
            if st is None:
                return None
            if not st["running"]:
                return st["position"]
            if time.monotonic() - t0 > timeout:
                log.warning("step_wait: таймаут %gs", timeout)
                return None
            time.sleep(poll_interval)

    # ── ЭЭП-процесс ─────────────────────────────────────────────────────

    def set_pulse_params(self, freq_hz: int, duty_pct: int, level: int) -> bool:
        """
        Параметры разрядных импульсов.

        Параметры:
            freq_hz  -- частота импульсов, Гц (100..65535)
            duty_pct -- скважность, % (1..99)
            level    -- мощность/напряжение (0..255, интерпретация зависит от
                        конкретного генератора импульсов)

        Применяются немедленно: Arduino вызывает applyPulseParams()
        (см. TODO в protokol_arduino.ino для привязки к реальному генератору).
        """
        data = list(struct.pack(">H", max(1, min(65535, freq_hz))))
        data += [max(1, min(99, duty_pct)), max(0, min(255, level))]
        return self._ok(self._cmd(CMD_SET_PULSE, data))

    def get_pulse_params(self) -> Optional[dict]:
        """
        Текущие параметры импульсов.
        Возвращает dict(freq_hz, duty_pct, level, gap_target_v) или None.
        """
        resp = self._cmd(CMD_GET_PULSE)
        if resp is None or resp["type"] != RESP_PULSE_PARAMS:
            return None
        d = resp["data"]
        if len(d) < 6:
            return None
        return {
            "freq_hz":     (d[0] << 8) | d[1],
            "duty_pct":    d[2],
            "level":       d[3],
            "gap_target_v": ((d[4] << 8) | d[5]) / 10.0,
        }

    def set_gap_target(self, voltage_v: float) -> bool:
        """
        Установить целевое напряжение зазора для gap tracking (Вольты).

        Пример: m.set_gap_target(50.0)  # 50В
        """
        raw = max(0, min(32767, int(voltage_v * 10)))
        return self._ok(self._cmd(CMD_SET_GAP_TGT,
                                   [(raw >> 8) & 0xFF, raw & 0xFF]))

    def estop(self) -> bool:
        """
        АВАРИЙНАЯ ОСТАНОВКА:
        - шаговый остановлен и обесточен
        - генератор выключен
        - силовое реле выключено
        - вибрация выключена
        - установлен флаг MODE_ESTOP (блокирует большинство команд)

        После устранения причины: вызови estop_reset().
        """
        ok = self._ok(self._cmd(CMD_ESTOP))
        if ok:
            self._mode |= MODE_ESTOP
            self._mode &= ~(MODE_AUTO_GAP | MODE_CYCLE)
        return ok

    def estop_reset(self) -> bool:
        """Сбросить E-stop. Перед этим убедись, что причина устранена."""
        ok = self._ok(self._cmd(CMD_ESTOP_RESET))
        if ok:
            self._mode &= ~MODE_ESTOP
        return ok

    def start_cycle(self) -> bool:
        """
        Запустить цикл полировки:
        - включить gap tracking (MODE_AUTO_GAP)
        - включить генератор импульсов
        - включить вибропривод на 50%

        Предусловия (проверяются на Arduino):
        - E-stop не активен
        - шаговый двигатель включён (step_enable(True))
        - силовое реле включено (power_on())

        После старта используй monitor() для наблюдения за процессом.
        """
        ok = self._ok(self._cmd(CMD_START_CYCLE))
        if ok:
            self._mode |= (MODE_AUTO_GAP | MODE_CYCLE)
        return ok

    def stop_cycle(self) -> bool:
        """Остановить цикл. Генератор и вибрация выключаются, gap tracking отключается."""
        ok = self._ok(self._cmd(CMD_STOP_CYCLE))
        if ok:
            self._mode &= ~(MODE_AUTO_GAP | MODE_CYCLE)
        return ok

    # Мониторинг процесса 

    def get_process_snapshot(self) -> Optional[dict]:
        """
        Снимок состояния процесса: датчики + шаговый + режим.
        Возвращает словарь или None при ошибке I2C.
        """
        sensors = self.get_all_sensors()
        stepper = self.get_stepper_status()
        status  = self.get_status()
        if sensors is None or stepper is None:
            return None

        # Индексируем датчики по типу для удобства
        by_type = {s["type"]: s for s in sensors}
        return {
            "timestamp":   time.time(),
            "sensors":     sensors,
            "gap_v":       by_type.get(0x01, {}).get("value"),
            "current_ma":  by_type.get(0x02, {}).get("value"),
            "temp_c":      by_type.get(0x03, {}).get("value"),
            "vibro_um":    by_type.get(0x04, {}).get("value"),
            "force_g":     by_type.get(0x05, {}).get("value"),
            "step_pos":    stepper.get("position"),
            "step_run":    stepper.get("running"),
            "mode_str":    status.get("mode_str") if status else "?",
            "estop":       bool(self._mode & MODE_ESTOP),
            "cycle_on":    bool(self._mode & MODE_CYCLE),
        }

    def monitor(
        self,
        duration_s: float = 0,
        interval_s: float = 0.5,
        callback=None,
        alert_temp_c: float = 80.0,
        alert_current_ma: float = 5000.0,
    ):
        """
        Мониторинг процесса полировки.

        Параметры:
            duration_s       -- длительность в секундах (0 = бесконечно до Ctrl+C)
            interval_s       -- интервал опроса
            callback         -- fn(snapshot: dict) → None; None = вывод в консоль
            alert_temp_c     -- порог температуры для предупреждения (°C)
            alert_current_ma -- порог тока для предупреждения (мА)

        Остановка: Ctrl+C или когда цикл завершился (MODE_CYCLE не активен).

        Пример с callback:
            def log_data(snap):
                with open("process.csv", "a") as f:
                    f.write(f"{snap['timestamp']:.1f},"
                            f"{snap['gap_v'] or 0:.1f},"
                            f"{snap['current_ma'] or 0:.0f},"
                            f"{snap['step_pos']}\\n")
            m.monitor(duration_s=300, callback=log_data)
        """
        print(f"[MONITOR] Старт (interval={interval_s}s). Ctrl+C — стоп.")
        t0 = time.monotonic()
        try:
            while True:
                snap = self.get_process_snapshot()
                if snap is None:
                    print(f"[{_ts()}] ⚠ Нет ответа от Arduino")
                    time.sleep(interval_s)
                    continue

                if callback:
                    callback(snap)
                else:
                    _print_snapshot(snap)

                # Предупреждения
                if snap.get("temp_c") and snap["temp_c"] > alert_temp_c:
                    print(f"[{_ts()}] ⚠ ПЕРЕГРЕВ: {snap['temp_c']:.1f}°C > {alert_temp_c}°C")
                if snap.get("current_ma") and snap["current_ma"] > alert_current_ma:
                    print(f"[{_ts()}] ⚠ ТОК: {snap['current_ma']:.0f}мА")

                if snap["estop"]:
                    print(f"[{_ts()}] E-STOP активен — мониторинг остановлен")
                    break

                if not snap["cycle_on"] and duration_s == 0:
                    # Цикл завершился (например, CMR_STOP_CYCLE)
                    break

                elapsed = time.monotonic() - t0
                if duration_s > 0 and elapsed >= duration_s:
                    print(f"[MONITOR] Время вышло ({duration_s}с)")
                    break

                time.sleep(interval_s)
        except KeyboardInterrupt:
            print(f"\n[MONITOR] Прерван пользователем")

    def stream(self, interval: float = 1.0, callback=None):
        """Синоним monitor() без таймаута — для обратной совместимости."""
        self.monitor(duration_s=0, interval_s=interval, callback=callback)

#  Форматированный вывод 

def _ts() -> str:
    return time.strftime("%H:%M:%S")

def _bar(value: float, max_val: float, width: int = 20) -> str:
    filled = int(min(value, max_val) / max_val * width)
    return "█" * filled + "░" * (width - filled)

def _print_snapshot(snap: dict) -> None:
    print(f"\n{'─' * 52}")
    print(f"  ЭЭП [{_ts()}]  режим: {snap['mode_str']}")
    print(f"{'─' * 52}")

    gv  = snap.get("gap_v")
    cur = snap.get("current_ma")
    tmp = snap.get("temp_c")
    vib = snap.get("vibro_um")
    frc = snap.get("force_g")
    pos = snap.get("step_pos")

    print(f"  U зазора:     {f'{gv:.1f} В':>8}  {_bar(gv or 0, 120)}" if gv is not None else "  U зазора:     н/д")
    print(f"  Ток разряда:  {f'{cur:.0f} мА':>8}  {_bar(cur or 0, 5000)}" if cur is not None else "  Ток разряда:  н/д")
    print(f"  Температура:  {f'{tmp:.1f} °C':>8}" if tmp is not None else "  Температура:  н/д")
    print(f"  Вибрация:     {f'{vib} мкм':>8}" if vib is not None else "  Вибрация:     н/д")
    print(f"  Усилие:       {f'{frc} г':>8}" if frc is not None else "  Усилие:       н/д")
    print(f"  Ось Z:        {f'{pos} шаг':>8}  {'⚙ движение' if snap.get('step_run') else '⏸ стоп'}" if pos is not None else "  Ось Z:        н/д")
    if snap.get("estop"):
        print(f"E-STOP АКТИВЕН")


# ━━ Точка входа (демо)

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-8s %(message)s",
        datefmt="%H:%M:%S",
    )

    with EEPMachine(bus=1, addr=0x08, read_delay_ms=5) as m:

        # 1. Связь
        print("Проверка связи с Arduino...")
        m.check_connection()
        print("Связь установлена.\n")

        # 2. Статус
        st = m.get_status()
        if st:
            print(f"Режим: {st['mode_str']}")
            print(f"Датчиков: {st['num_sensors']}  Устройств: {st['num_devices']}\n")

        # 3. Снимок состояния (ничего не запускаем — ждём команды из веб-интерфейса)
        snap = m.get_process_snapshot()
        if snap:
            _print_snapshot(snap)

        print("\nСтанок готов. Управление — через веб-интерфейс (app.py).")
        print("Для прямого мониторинга: m.monitor()")
        print("Для аварийной остановки: m.estop()\n")
