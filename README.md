# EEP Control

Веб-интерфейс для управления станком электроэрозионной полировки (ЭЭП).

**Железо:** Raspberry Pi 5 + Arduino Mega 2560, связь по I2C.

---

## Что внутри

```
eep_control/
├── app.py                  — Flask-приложение, REST API, WebSocket
├── config.py               — конфигурация (хост, порт, параметры станка)
├── requirements.txt        — зависимости Python
├── protokol_raspberry.py   — управляющая программа RPi (I2C-протокол)
├── protokol_arduino.ino    — прошивка Arduino Mega 2560
├── modules/
│   ├── machine.py          — модель станка (состояния, параметры импульсов)
│   ├── process.py          — рецепты обработки (сталь, титан, медь)
│   └── system_info.py      — мониторинг RPi (CPU, RAM, температура)
├── templates/
│   └── index.html          — веб-интерфейс
└── static/                 — CSS и JS
```

### Что умеет

- Управление станком через браузер: старт / пауза / стоп / аварийная остановка
- Настройка параметров импульсов (напряжение, ток, частота, скважность)
- Готовые рецепты: полировка стали, финишная обработка титана, зеркальная полировка меди
- Телеметрия в реальном времени через WebSocket (5 раз в секунду)
- Мониторинг состояния Raspberry Pi (CPU, RAM, температура, uptime)

---

## Требования

### Железо

| Компонент | Описание |
|---|---|
| Raspberry Pi 5 | основной контроллер |
| Arduino Mega 2560 | контроллер силовой части |
| Level shifter (TXS0102 / BSS138 / PCA9306) | обязательно — RPi работает на 3.3В, Arduino на 5В |

Подключение I2C: RPi GPIO2 (SDA) и GPIO3 (SCL) → level shifter → Arduino pin 20 (SDA) и 21 (SCL).
Адрес Arduino на шине: `0x08`.

### Python-библиотеки

```
flask
flask-socketio
python-dotenv
psutil
smbus2
```

Установка:

```bash
pip install flask flask-socketio python-dotenv psutil smbus2
```

---

## Установка и запуск

### 1. Включить I2C на Raspberry Pi

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

Проверить, что Arduino видна на шине:

```bash
i2cdetect -y 1
# должна быть точка на адресе 0x08
```

### 2. Прошить Arduino

Открыть `protokol_arduino.ino` в Arduino IDE и загрузить на Arduino Mega 2560.

### 3. Клонировать проект

```bash
git clone https://github.com/Impi94/eep-control.git
cd eep-control
```

### 4. Установить зависимости

```bash
python3 -m venv venv
source venv/bin/activate
pip install flask flask-socketio python-dotenv psutil smbus2
```

### 5. Настройка (опционально)

Создать файл `.env` для переопределения параметров:

```env
HOST=0.0.0.0
PORT=5000
DEBUG=False
MACHINE_NAME=EEP-01
MAX_VOLTAGE=300
MAX_CURRENT=50
MAX_PULSE_FREQ=100000
```

### 6. Запустить

```bash
python3 app.py
```

Открыть в браузере: `http://<IP-адрес-RPi>:5000`

---

## Прямое управление через I2C (без веб-интерфейса)

```python
from protokol_raspberry import EEPMachine

with EEPMachine() as m:
    m.check_connection()
    m.set_pulse_params(freq_hz=2000, duty_pct=40, level=100)
    m.power_on()
    m.start_cycle()
    m.monitor(duration_s=60)
    m.stop_cycle()
    m.power_off()
```

Подробнее — в файле [УСТАНОВКА.txt](УСТАНОВКА.txt).

---

## API

| Метод | Путь | Описание |
|---|---|---|
| GET | `/api/state` | полное состояние станка |
| GET | `/api/system` | состояние Raspberry Pi |
| GET | `/api/log` | журнал событий |
| GET | `/api/recipes` | список рецептов |
| POST | `/api/control` | управление (`start`, `pause`, `resume`, `stop`, `emergency_stop`) |
| POST | `/api/params/pulse` | параметры импульсов |
| POST | `/api/params/process` | параметры процесса |
| POST | `/api/preset/<name>` | загрузить рецепт |

WebSocket-события: `state_update`, `telemetry`.
