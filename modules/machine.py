"""
Модель станка электроэрозионной полировки.
При наличии Arduino (smbus2 + I2C) работает с реальным железом,
иначе — программная симуляция для разработки UI.
"""

import time
import threading
import random
import logging
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Optional, List

logger = logging.getLogger(__name__)

# Пытаемся подключить модуль протокола (требует smbus2 на RPi)
try:
    from protokol_raspberry import EEPMachine as _EEPMachine
    _HW_MODULE_OK = True
except ImportError:
    _HW_MODULE_OK = False
    logger.info("smbus2 не установлен — режим симуляции")

_DT = 0.2  # период цикла, сек


class MachineState(str, Enum):
    IDLE           = "idle"
    READY          = "ready"
    RUNNING        = "running"
    PAUSED         = "paused"
    FINISHING      = "finishing"
    ERROR          = "error"
    EMERGENCY_STOP = "e-stop"
    HOMING         = "homing"


@dataclass
class PulseParams:
    voltage:   float = 80.0
    current:   float = 10.0
    pulse_on:  float = 50.0
    pulse_off: float = 50.0
    frequency: float = 10000.0
    polarity:  str   = "normal"


@dataclass
class ProcessParams:
    mode:                     str   = "polish"
    target_ra:                float = 0.1
    max_time:                 int   = 0       # 0 = бесконечно
    electrolyte_type:         str   = "NaNO3"
    electrolyte_concentration: float = 15.0
    gap:                      float = 0.1
    flushing_pressure:        float = 2.0
    electrode_rotation:       float = 0.0


@dataclass
class SensorData:
    voltage_actual:            float = 0.0
    current_actual:            float = 0.0
    gap_actual:                float = 0.0
    electrolyte_temp:          float = 22.0
    electrolyte_flow:          float = 0.0
    electrolyte_conductivity:  float = 0.0
    vibration:                 float = 0.0
    surface_roughness:         float = 0.0
    electrode_wear:            float = 0.0
    tank_level:                float = 85.0
    ambient_temp:              float = 24.0


@dataclass
class ProcessStats:
    start_time:       Optional[float] = None
    elapsed_seconds:  int   = 0
    material_removed: float = 0.0
    energy_consumed:  float = 0.0
    pulse_count:      int   = 0
    short_circuits:   int   = 0
    arc_count:        int   = 0
    efficiency:       float = 0.0
    progress:         float = 0.0


class Machine:
    """Главный класс станка. Управляет железом или симуляцией."""

    def __init__(self):
        self.state   = MachineState.IDLE
        self.pulse   = PulseParams()
        self.process = ProcessParams()
        self.sensors = SensorData()
        self.stats   = ProcessStats()
        self.errors: List[dict] = []
        self.log:    List[dict] = []

        self.presets = {
            "rough":  PulseParams(voltage=120, current=30, pulse_on=100, pulse_off=50,  frequency=5000),
            "polish": PulseParams(voltage=80,  current=10, pulse_on=50,  pulse_off=50,  frequency=10000),
            "finish": PulseParams(voltage=40,  current=5,  pulse_on=20,  pulse_off=80,  frequency=20000),
            "mirror": PulseParams(voltage=20,  current=2,  pulse_on=10,  pulse_off=100, frequency=50000),
        }

        self._hw: Optional[_EEPMachine] = None  # type: ignore[type-arg]
        self._running = False
        self._loop_thread: Optional[threading.Thread] = None

        self._try_connect_hw()
        self._add_log("Станок инициализирован", "info")

    # ── Подключение к железу ────────────────────────────────────────────────

    def _try_connect_hw(self):
        if not _HW_MODULE_OK:
            return
        try:
            hw = _EEPMachine()
            hw.check_connection()
            self._hw = hw
            self._add_log("Arduino подключена по I2C ✓", "info")
        except Exception as e:
            logger.warning(f"Arduino не найдена ({e}) — симуляция")

    # ── Логирование ─────────────────────────────────────────────────────────

    def _add_log(self, message: str, level: str = "info"):
        entry = {
            'time':      time.strftime('%H:%M:%S'),
            'timestamp': time.time(),
            'message':   message,
            'level':     level,
        }
        self.log.append(entry)
        if len(self.log) > 200:
            self.log = self.log[-200:]
        logger.info(f"[{level.upper()}] {message}")

    def _add_error(self, code: str, message: str):
        self.errors.append({
            'time':    time.strftime('%H:%M:%S'),
            'code':    code,
            'message': message,
        })
        self._add_log(f"ОШИБКА {code}: {message}", "error")

    # ── Управление ──────────────────────────────────────────────────────────

    def start(self) -> dict:
        if self.state == MachineState.ERROR:
            return {'success': False, 'error': 'Сначала сбросьте ошибки'}
        if self.state == MachineState.RUNNING:
            return {'success': False, 'error': 'Процесс уже запущен'}

        self.state = MachineState.RUNNING
        self.stats = ProcessStats(start_time=time.time())
        self._running = True

        if self._hw:
            duty  = int(self.pulse.pulse_on / (self.pulse.pulse_on + self.pulse.pulse_off) * 100)
            level = int(min(255, self.pulse.voltage / 300 * 255))
            self._hw.set_pulse_params(freq_hz=int(self.pulse.frequency), duty_pct=duty, level=level)
            self._hw.set_gap_target(self.pulse.voltage * 0.5)
            self._hw.power_on()
            self._hw.step_enable(True)
            self._hw.start_cycle()
            self._add_log(f"Запуск на железе | V={self.pulse.voltage}В I={self.pulse.current}А f={self.pulse.frequency}Гц")
        else:
            self._add_log(f"Симуляция | Режим: {self.process.mode} | V={self.pulse.voltage}В I={self.pulse.current}А")

        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()
        return {'success': True}

    def pause(self) -> dict:
        if self.state != MachineState.RUNNING:
            return {'success': False, 'error': 'Процесс не запущен'}
        self._running = False
        self.state = MachineState.PAUSED
        if self._hw:
            self._hw.stop_cycle()
        self._add_log("Процесс приостановлен")
        return {'success': True}

    def resume(self) -> dict:
        if self.state != MachineState.PAUSED:
            return {'success': False, 'error': 'Процесс не на паузе'}
        self.state = MachineState.RUNNING
        self._running = True
        if self._hw:
            self._hw.start_cycle()
        self._loop_thread = threading.Thread(target=self._run_loop, daemon=True)
        self._loop_thread.start()
        self._add_log("Процесс возобновлён")
        return {'success': True}

    def stop(self) -> dict:
        self._running = False
        if self._hw:
            self._hw.stop_cycle()
            self._hw.power_off()
        self.state = MachineState.IDLE
        self.sensors.voltage_actual = 0
        self.sensors.current_actual = 0
        self._add_log("Процесс остановлен")
        return {'success': True}

    def emergency_stop(self) -> dict:
        self._running = False
        if self._hw:
            self._hw.estop()
        self.state = MachineState.EMERGENCY_STOP
        self.sensors.voltage_actual = 0
        self.sensors.current_actual = 0
        self._add_log("АВАРИЙНАЯ ОСТАНОВКА", "error")
        return {'success': True}

    def reset_errors(self) -> dict:
        self.errors.clear()
        if self._hw:
            self._hw.estop_reset()
        if self.state in (MachineState.ERROR, MachineState.EMERGENCY_STOP):
            self.state = MachineState.IDLE
        self._add_log("Ошибки сброшены")
        return {'success': True}

    def set_pulse_params(self, params: dict) -> dict:
        if self.state == MachineState.RUNNING:
            self._add_log("Параметры изменены во время работы", "warning")

        for key, value in params.items():
            if hasattr(self.pulse, key):
                setattr(self.pulse, key, float(value) if key != 'polarity' else value)

        if self._hw and self.state == MachineState.RUNNING:
            duty  = int(self.pulse.pulse_on / (self.pulse.pulse_on + self.pulse.pulse_off) * 100)
            level = int(min(255, self.pulse.voltage / 300 * 255))
            self._hw.set_pulse_params(freq_hz=int(self.pulse.frequency), duty_pct=duty, level=level)

        self._add_log(f"Параметры: V={self.pulse.voltage} I={self.pulse.current} Ton={self.pulse.pulse_on} Toff={self.pulse.pulse_off}")
        return {'success': True}

    def set_simulation(self, enable: bool) -> dict:
        if not self._hw:
            return {'success': False, 'error': 'Arduino не подключена'}

        if self._hw.set_simulation(enable):
            self._add_log(f"Симуляция Arduino {'включена' if enable else 'выключена'}")
            return {'success': True, 'simulation_active': enable}

        return {'success': False, 'error': 'Ошибка при отправке флага симуляции'}

    def set_process_params(self, params: dict) -> dict:
        for key, value in params.items():
            if hasattr(self.process, key):
                setattr(self.process, key, value)
        self._add_log(f"Режим: {self.process.mode}")
        return {'success': True}

    def load_preset(self, name: str) -> dict:
        if name in self.presets:
            self.pulse = PulseParams(**asdict(self.presets[name]))
            self._add_log(f"Загружен пресет: {name}")
            return {'success': True, 'params': asdict(self.pulse)}
        return {'success': False, 'error': f'Пресет "{name}" не найден'}

    # ── Данные ──────────────────────────────────────────────────────────────

    def get_full_state(self) -> dict:
        if self.stats.start_time and self.state == MachineState.RUNNING:
            self.stats.elapsed_seconds = int(time.time() - self.stats.start_time)
        return {
            'state':             self.state.value,
            'pulse':             asdict(self.pulse),
            'process':           asdict(self.process),
            'sensors':           asdict(self.sensors),
            'stats':             asdict(self.stats),
            'errors':            self.errors[-10:],
            'presets':           list(self.presets.keys()),
            'hw_connected':      self._hw is not None,
            'simulation_active': self._hw.simulation_active if self._hw else False,
        }

    def get_log(self, limit: int = 50) -> list:
        return self.log[-limit:]

    # ── Основной цикл ───────────────────────────────────────────────────────

    def _run_loop(self):
        """Цикл опроса: реальное железо или симуляция. Работает до stop()."""
        while self._running:
            try:
                if self._hw:
                    self._poll_hardware()
                else:
                    self._simulate_step()

                self.stats.elapsed_seconds = int(time.time() - self.stats.start_time)

                # Прогресс только если задано ограничение по времени (max_time > 0)
                if self.process.max_time > 0:
                    self.stats.progress = round(
                        min(100, self.stats.elapsed_seconds / self.process.max_time * 100), 1
                    )
                    if self.stats.progress >= 100:
                        self._finish()
                        break

                time.sleep(_DT)

            except Exception as e:
                logger.error(f"Ошибка цикла: {e}")
                self._add_error("LOOP_ERR", str(e))
                self.state = MachineState.ERROR
                break

    def _finish(self):
        self._running = False
        self.state = MachineState.FINISHING
        self._add_log("Процесс завершён по таймеру")
        time.sleep(1)
        self.state = MachineState.IDLE
        self.sensors.voltage_actual = 0
        self.sensors.current_actual = 0
        if self._hw:
            self._hw.stop_cycle()
            self._hw.power_off()

    # ── Опрос реального железа ──────────────────────────────────────────────

    def _poll_hardware(self):
        snap = self._hw.get_process_snapshot()
        if snap is None:
            self._add_log("Нет ответа от Arduino", "warning")
            return

        if snap.get('gap_v') is not None:
            self.sensors.voltage_actual = round(snap['gap_v'], 1)
        if snap.get('current_ma') is not None:
            self.sensors.current_actual = round(snap['current_ma'] / 1000, 3)
        if snap.get('temp_c') is not None:
            self.sensors.electrolyte_temp = round(snap['temp_c'], 1)
        if snap.get('vibro_um') is not None:
            self.sensors.vibration = round(snap['vibro_um'], 2)

        self.stats.pulse_count += int(self.pulse.frequency * _DT)
        self.stats.energy_consumed += round(
            self.sensors.voltage_actual * self.sensors.current_actual * _DT / 3600, 4
        )

        if snap.get('estop'):
            self._running = False
            self.state = MachineState.EMERGENCY_STOP
            self._add_log("E-STOP от Arduino", "error")

        if not snap.get('cycle_on') and self.state == MachineState.RUNNING:
            self._running = False
            self.state = MachineState.FINISHING
            self._add_log("Цикл завершён (Arduino)")
            time.sleep(1)
            self.state = MachineState.IDLE

    # ── Симуляция ───────────────────────────────────────────────────────────

    def _simulate_step(self):
        v_noise = random.gauss(0, self.pulse.voltage * 0.02)
        i_noise = random.gauss(0, self.pulse.current * 0.03)

        self.sensors.voltage_actual        = round(self.pulse.voltage + v_noise, 1)
        self.sensors.current_actual        = round(max(0, self.pulse.current + i_noise), 2)
        self.sensors.gap_actual            = round(self.process.gap + random.gauss(0, 0.005), 4)
        self.sensors.electrolyte_temp      = round(self.sensors.electrolyte_temp + random.gauss(0.01, 0.05), 1)
        self.sensors.electrolyte_flow      = round(3.5 + random.gauss(0, 0.2), 1)
        self.sensors.electrolyte_conductivity = round(45 + random.gauss(0, 1), 1)
        self.sensors.vibration             = round(abs(random.gauss(0.5, 0.3)), 2)
        self.sensors.electrode_wear        = round(min(100, self.sensors.electrode_wear + random.uniform(0, 0.01)), 2)
        self.sensors.tank_level            = round(max(0, self.sensors.tank_level - random.uniform(0, 0.005)), 1)

        self.stats.pulse_count      += int(self.pulse.frequency * _DT)
        self.stats.material_removed += round(self.sensors.current_actual * 0.001 * _DT, 4)
        self.stats.energy_consumed  += round(self.sensors.voltage_actual * self.sensors.current_actual * _DT / 3600, 4)
        self.stats.efficiency        = round(random.gauss(85, 3), 1)

        if random.random() < 0.002:
            self.stats.short_circuits += 1
            self._add_log("Короткое замыкание обнаружено", "warning")

        if random.random() < 0.001:
            self.stats.arc_count += 1
            self._add_log("Дуговой разряд", "warning")

        if self.sensors.electrolyte_temp > 45:
            self._add_log("Температура электролита высокая!", "warning")
