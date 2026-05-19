"""
Модель станка электроэрозионной полировки.
Подключение к реальному железу добавлю как дадут железки.
"""

import time
import threading
import random
import logging
from enum import Enum
from dataclasses import dataclass, field, asdict
from typing import Optional, List

logger = logging.getLogger(__name__)


class MachineState(str, Enum):
    IDLE = "idle"                  # Простой
    READY = "ready"                # Готов к работе
    RUNNING = "running"            # Полировка идёт
    PAUSED = "paused"              # Пауза
    FINISHING = "finishing"        # Завершение цикла
    ERROR = "error"                # Ошибка
    EMERGENCY_STOP = "e-stop"     # Аварийная остановка
    HOMING = "homing"             # Выход в ноль


@dataclass
class PulseParams:
    """Параметры импульсов"""
    voltage: float = 80.0           # Напряжение, В
    current: float = 10.0           # Ток, А
    pulse_on: float = 50.0          # Длительность импульса, мкс
    pulse_off: float = 50.0         # Пауза между импульсами, мкс
    frequency: float = 10000.0      # Частота, Гц
    polarity: str = "normal"        # normal / reverse


@dataclass
class ProcessParams:
    """Параметры процесса"""
    mode: str = "polish"            # polish / rough / finish / custom
    target_ra: float = 0.1          # Целевая шероховатость Ra, мкм
    max_time: int = 3600            # Макс. время, сек
    electrolyte_type: str = "NaNO3" # Тип электролита
    electrolyte_concentration: float = 15.0  # Концентрация, %
    gap: float = 0.1                # Межэлектродный зазор, мм
    flushing_pressure: float = 2.0  # Давление промывки, бар
    electrode_rotation: float = 0.0 # Вращение электрода, об/мин


@dataclass
class SensorData:
    """Данные с датчикоd"""
    voltage_actual: float = 0.0
    current_actual: float = 0.0
    gap_actual: float = 0.0
    electrolyte_temp: float = 22.0
    electrolyte_flow: float = 0.0
    electrolyte_conductivity: float = 0.0
    vibration: float = 0.0
    surface_roughness: float = 0.0
    electrode_wear: float = 0.0
    tank_level: float = 85.0
    ambient_temp: float = 24.0


@dataclass
class ProcessStats:
    """Статистика текущего процесса"""
    start_time: Optional[float] = None
    elapsed_seconds: int = 0
    material_removed: float = 0.0   # мг
    energy_consumed: float = 0.0    # Вт*ч
    pulse_count: int = 0
    short_circuits: int = 0
    arc_count: int = 0
    efficiency: float = 0.0        # %
    progress: float = 0.0          # % выполнения


class Machine:
    """Главный класс станка"""

    def __init__(self):
        self.state = MachineState.IDLE
        self.pulse = PulseParams()
        self.process = ProcessParams()
        self.sensors = SensorData()
        self.stats = ProcessStats()
        self.errors: List[dict] = []
        self.log: List[dict] = []
        
        # Пресеты режимов
        self.presets = {
            "rough": PulseParams(voltage=120, current=30, pulse_on=100, pulse_off=50, frequency=5000),
            "polish": PulseParams(voltage=80, current=10, pulse_on=50, pulse_off=50, frequency=10000),
            "finish": PulseParams(voltage=40, current=5, pulse_on=20, pulse_off=80, frequency=20000),
            "mirror": PulseParams(voltage=20, current=2, pulse_on=10, pulse_off=100, frequency=50000),
        }
        
        self._simulation_thread: Optional[threading.Thread] = None
        self._running = False
        
        self._add_log("Станок инициализирован", "info")

    def _add_log(self, message: str, level: str = "info"):
        entry = {
            'time': time.strftime('%H:%M:%S'),
            'timestamp': time.time(),
            'message': message,
            'level': level
        }
        self.log.append(entry)
        # логи последние 200 записей
        if len(self.log) > 200:
            self.log = self.log[-200:]
        logger.info(f"[{level.upper()}] {message}")

    def _add_error(self, code: str, message: str):
        error = {
            'time': time.strftime('%H:%M:%S'),
            'code': code,
            'message': message
        }
        self.errors.append(error)
        self._add_log(f"ОШИБКА {code}: {message}", "error")

    # Управление

    def start(self) -> dict:
        if self.state == MachineState.ERROR:
            return {'success': False, 'error': 'Сначала сбросьте ошибки'}
        if self.state == MachineState.RUNNING:
            return {'success': False, 'error': 'Процесс уже запущен'}
        
        self.state = MachineState.RUNNING
        self.stats = ProcessStats(start_time=time.time())
        self._running = True
        self._simulation_thread = threading.Thread(target=self._simulate, daemon=True)
        self._simulation_thread.start()
        self._add_log(f"Процесс запущен | Режим: {self.process.mode} | V={self.pulse.voltage}В I={self.pulse.current}А")
        return {'success': True}

    def pause(self) -> dict:
        if self.state != MachineState.RUNNING:
            return {'success': False, 'error': 'Процесс не запущен'}
        self.state = MachineState.PAUSED
        self._running = False
        self._add_log("Процесс приостановлен")
        return {'success': True}

    def resume(self) -> dict:
        if self.state != MachineState.PAUSED:
            return {'success': False, 'error': 'Процесс не на паузе'}
        self.state = MachineState.RUNNING
        self._running = True
        self._simulation_thread = threading.Thread(target=self._simulate, daemon=True)
        self._simulation_thread.start()
        self._add_log("Процесс возобновлён")
        return {'success': True}

    def stop(self) -> dict:
        self._running = False
        self.state = MachineState.IDLE
        self.sensors.voltage_actual = 0
        self.sensors.current_actual = 0
        self._add_log("Процесс остановлен")
        return {'success': True}

    def emergency_stop(self) -> dict:
        self._running = False
        self.state = MachineState.EMERGENCY_STOP
        self.sensors.voltage_actual = 0
        self.sensors.current_actual = 0
        self._add_log("АВАРИЙНАЯ ОСТАНОВКА", "error")
        return {'success': True}

    def reset_errors(self) -> dict:
        self.errors.clear()
        if self.state in (MachineState.ERROR, MachineState.EMERGENCY_STOP):
            self.state = MachineState.IDLE
        self._add_log("Ошибки сброшены")
        return {'success': True}

    def set_pulse_params(self, params: dict) -> dict:
        if self.state == MachineState.RUNNING:
            # Разрешаем менять на лету (с ограничениями)
            self._add_log("Параметры изменены во время работы", "warning")
        
        for key, value in params.items():
            if hasattr(self.pulse, key):
                setattr(self.pulse, key, float(value) if key != 'polarity' else value)
        
        self._add_log(f"Параметры: V={self.pulse.voltage} I={self.pulse.current} Ton={self.pulse.pulse_on} Toff={self.pulse.pulse_off}")
        return {'success': True}

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

    # Получение данных

    def get_full_state(self) -> dict:
        if self.stats.start_time and self.state == MachineState.RUNNING:
            self.stats.elapsed_seconds = int(time.time() - self.stats.start_time)
        
        return {
            'state': self.state.value,
            'pulse': asdict(self.pulse),
            'process': asdict(self.process),
            'sensors': asdict(self.sensors),
            'stats': asdict(self.stats),
            'errors': self.errors[-10:],
            'presets': list(self.presets.keys()),
        }

    def get_log(self, limit: int = 50) -> list:
        return self.log[-limit:]

    # Симуляция

    def _simulate(self):
        """Симуляция показаний датчиков для разработки UI"""
        while self._running:
            try:
                # Имитация показаний
                v_noise = random.gauss(0, self.pulse.voltage * 0.02)
                i_noise = random.gauss(0, self.pulse.current * 0.03)
                
                self.sensors.voltage_actual = round(self.pulse.voltage + v_noise, 1)
                self.sensors.current_actual = round(max(0, self.pulse.current + i_noise), 2)
                self.sensors.gap_actual = round(self.process.gap + random.gauss(0, 0.005), 4)
                self.sensors.electrolyte_temp = round(self.sensors.electrolyte_temp + random.gauss(0.01, 0.05), 1)
                self.sensors.electrolyte_flow = round(3.5 + random.gauss(0, 0.2), 1)
                self.sensors.electrolyte_conductivity = round(45 + random.gauss(0, 1), 1)
                self.sensors.vibration = round(abs(random.gauss(0.5, 0.3)), 2)
                self.sensors.electrode_wear = round(min(100, self.sensors.electrode_wear + random.uniform(0, 0.01)), 2)
                self.sensors.tank_level = round(max(0, self.sensors.tank_level - random.uniform(0, 0.005)), 1)
                
                # Статистика
                dt = 0.2  # период обновления
                self.stats.elapsed_seconds = int(time.time() - self.stats.start_time)
                self.stats.pulse_count += int(self.pulse.frequency * dt)
                self.stats.material_removed += round(self.sensors.current_actual * 0.001 * dt, 4)
                self.stats.energy_consumed += round(self.sensors.voltage_actual * self.sensors.current_actual * dt / 3600, 4)
                self.stats.efficiency = round(random.gauss(85, 3), 1)
                
                # Прогресс
                if self.process.max_time > 0:
                    self.stats.progress = round(min(100, self.stats.elapsed_seconds / self.process.max_time * 100), 1)
                
                # Случайные события
                if random.random() < 0.002:
                    self.stats.short_circuits += 1
                    self._add_log("Короткое замыкание обнаружено", "warning")
                
                if random.random() < 0.001:
                    self.stats.arc_count += 1
                    self._add_log("Дуговой разряд", "warning")
                
                # Проверка перегрева
                if self.sensors.electrolyte_temp > 45:
                    self._add_log("Температура электролита высокая!", "warning")
                
                # Завершение по времени
                if self.stats.progress >= 100:
                    self._running = False
                    self.state = MachineState.FINISHING
                    self._add_log("✅ Процесс завершён")
                    time.sleep(1)
                    self.state = MachineState.IDLE
                    self.sensors.voltage_actual = 0
                    self.sensors.current_actual = 0
                    break
                
                time.sleep(dt)
                
            except Exception as e:
                logger.error(f"Ошибка симуляции: {e}")
                self._add_error("SIM_ERR", str(e))
                self.state = MachineState.ERROR
                break
