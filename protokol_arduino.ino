/*
 * protokol_arduino.ino
 *
 * I2C Slave — станок электро-эрозионной полировки (ЭЭП) в сухом электролите
 * Arduino Mega (слейв) ↔ Raspberry Pi 5 (мастер)
 *
 * ── Подключение ───────────────────────────────────────────────────────────
 *   Arduino Mega pin 20 (SDA) ─► level shifter ─► RPi pin 3 (GPIO 2, SDA)
 *   Arduino Mega pin 21 (SCL) ─► level shifter ─► RPi pin 5 (GPIO 3, SCL)
 *   GND ────────────────────────────────────────── RPi pin 6 (GND)
 *   ВАЖНО: Arduino 5V логика, RPi 3.3V → обязателен level shifter!
 *   Подтяжки 4.7 кОм SDA/SCL к 3.3V со стороны RPi.
 *
 * ── Формат пакета (макс. 32 байта) ───────────────────────────────────────
 *   [0xAA][CMD][LEN][DATA_0..DATA_N][CRC8]
 *   CRC8 = XOR(CMD ^ LEN ^ DATA_0 ^ ... ^ DATA_N)
 *
 * ── Команды (RPi → Arduino) ──────────────────────────────────────────────
 *  Общие:
 *   0x01 CMD_PING         — проверка связи                       → ACK
 *   0x02 CMD_GET_ALL      — все показания датчиков               → SENSOR_DATA
 *   0x03 CMD_GET_SENSOR   — один датчик [data[0]=id]             → SENSOR_DATA
 *   0x04 CMD_SET_MODE     — установить режим [data[0]=mode]      → ACK
 *   0x05 CMD_RESET        — полный сброс состояния               → ACK
 *   0x06 CMD_GET_STATUS   — статус системы                       → STATUS
 *  Устройства:
 *   0x10 CMD_SET_DEVICE   — [id][val_hi][val_lo]                 → ACK
 *   0x11 CMD_GET_DEVICES  — состояние устройств                  → DEVICE_DATA
 *  Шаговый двигатель (ось Z):
 *   0x20 CMD_STEP_MOVE    — отн. перемещение [int32 шаги][uint16 скор.] → ACK
 *   0x21 CMD_STEP_GOTO    — абс. позиция    [int32 поз.][uint16 скор.] → ACK
 *   0x22 CMD_STEP_STOP    — стоп                                 → ACK
 *   0x23 CMD_STEP_HOME    — обнулить позицию (без движения)      → ACK
 *   0x24 CMD_STEP_ENA     — вкл/выкл драйвера [0/1]             → ACK
 *   0x25 CMD_GET_STEPPER  — статус мотора                        → STEP_STATUS
 *  ЭЭП-процесс:
 *   0x30 CMD_SET_PULSE    — параметры имп. [fH][fL][duty%][level] → ACK
 *   0x31 CMD_GET_PULSE    — параметры импульсов                  → PULSE_PARAMS
 *   0x32 CMD_SET_GAP_TGT  — целевое U зазора [V×10 hi][V×10 lo] → ACK
 *   0x33 CMD_ESTOP        — аварийная остановка (всё выкл)       → ACK
 *   0x34 CMD_ESTOP_RESET  — сброс E-stop                        → ACK
 *   0x35 CMD_START_CYCLE  — запустить цикл полировки             → ACK
 *   0x36 CMD_STOP_CYCLE   — остановить цикл                      → ACK
 *
 * ── Ответы (Arduino → RPi) ───────────────────────────────────────────────
 *   0xA1 RESP_ACK         — команда принята; data[0]=0x00
 *   0xA3 RESP_SENSOR_DATA — данные датчиков (5 байт × датчик)
 *   0xA4 RESP_STATUS      — data: [mode][num_sens][num_dev]
 *   0xA5 RESP_ERROR       — data[0]=код ошибки
 *   0xA6 RESP_DEVICE_DATA — состояния устройств (5 байт × устройство)
 *   0xA7 RESP_STEP_STATUS — [pos int32][speed uint16][flags]
 *   0xA8 RESP_PULSE_PARAMS— [freq uint16][duty%][level][gapTgt uint16]
 *
 * ── Запись датчика в пакете (5 байт): [id][type][val_hi][val_lo][status] ─
 *   SENSOR_GAP_VOLT (0x01): В × 10        (750 = 75.0В, U зазора)
 *   SENSOR_CURR     (0x02): мА            (1500 = 1500 мА, ток разряда)
 *   SENSOR_TEMP     (0x03): °C × 100      (2500 = 25.00°C)
 *   SENSOR_VIBRO    (0x04): мкм           (50 = 50 мкм, амплитуда вибрации)
 *   SENSOR_FORCE    (0x05): г             (200 = 200 г, усилие прижима)
 *
 * ── Биты режима g_mode ────────────────────────────────────────────────────
 *   0x01 MODE_SIM      — симуляция датчиков (треугольная волна)
 *   0x02 MODE_AUTO_GAP — автоконтроль зазора (gap tracking)
 *   0x04 MODE_ESTOP    — аварийная остановка активна
 *   0x08 MODE_CYCLE    — цикл полировки активен
 *
 * ── Алгоритм gap tracking (MODE_AUTO_GAP) ────────────────────────────────
 *   Каждые GAP_CTRL_MS мс читается U зазора (датчик 0).
 *   КЗ (U < GAP_SC_V): быстрый отвод на GAP_RETRACT_FAST шагов.
 *   Мало (U < target - hysteresis): отвод на 1 шаг.
 *   Много (U > target + hysteresis): подача на 1 шаг.
 *   В зоне допуска: стоп.
 *   Перед запуском: включить шаговый (CMD_STEP_ENA), задать скорость подачи.
 *
 * ── Шаговый двигатель DM556 ───────────────────────────────────────────────
 *   PUL- → GND,  PUL+ → Arduino (TODO: STEP_PIN_PUL)
 *   DIR- → GND,  DIR+ → Arduino (TODO: STEP_PIN_DIR)
 *   ENA- → GND,  ENA+ → Arduino (TODO: STEP_PIN_ENA)  ← активный LOW
 */

#include <Wire.h>

// ── I2C адрес ─────────────────────────────────────────────────────────────
#define I2C_SLAVE_ADDR  0x08

// ── Протокол ──────────────────────────────────────────────────────────────
#define PKT_START       0xAA
#define PKT_MAX         32
#define PKT_DATA_MAX    28

// ── Команды ───────────────────────────────────────────────────────────────
#define CMD_PING         0x01
#define CMD_GET_ALL      0x02
#define CMD_GET_SENSOR   0x03
#define CMD_SET_MODE     0x04
#define CMD_RESET        0x05
#define CMD_GET_STATUS   0x06
#define CMD_SET_DEVICE   0x10
#define CMD_GET_DEVICES  0x11
#define CMD_STEP_MOVE    0x20
#define CMD_STEP_GOTO    0x21
#define CMD_STEP_STOP    0x22
#define CMD_STEP_HOME    0x23
#define CMD_STEP_ENA     0x24
#define CMD_GET_STEPPER  0x25
#define CMD_SET_PULSE    0x30
#define CMD_GET_PULSE    0x31
#define CMD_SET_GAP_TGT  0x32
#define CMD_ESTOP        0x33
#define CMD_ESTOP_RESET  0x34
#define CMD_START_CYCLE  0x35
#define CMD_STOP_CYCLE   0x36

// ── Ответы ────────────────────────────────────────────────────────────────
#define RESP_ACK          0xA1
#define RESP_SENSOR_DATA  0xA3
#define RESP_STATUS       0xA4
#define RESP_ERROR        0xA5
#define RESP_DEVICE_DATA  0xA6
#define RESP_STEP_STATUS  0xA7
#define RESP_PULSE_PARAMS 0xA8

// ── Типы датчиков ЭЭП ────────────────────────────────────────────────────
#define SENSOR_NONE      0x00
#define SENSOR_GAP_VOLT  0x01  // U зазора, В×10     (750 = 75.0В)
#define SENSOR_CURR      0x02  // Ток разряда, мА
#define SENSOR_TEMP      0x03  // Температура, °C×100 (2500 = 25.00°C)
#define SENSOR_VIBRO     0x04  // Амплитуда вибрации, мкм
#define SENSOR_FORCE     0x05  // Усилие прижима, г
// Новый тип: #define SENSOR_XXX  0x06

// ── Типы устройств ────────────────────────────────────────────────────────
#define DEV_NONE         0x00
#define DEV_RELAY        0x01  // реле: 0=выкл, 1=вкл
#define DEV_PWM          0x02  // ШИМ: 0..255
#define DEV_DOUT         0x03  // цифровой выход: 0/1
// Новый тип: #define DEV_XXX  0x04

// ── Статусы ───────────────────────────────────────────────────────────────
#define STS_OK           0x00
#define STS_ERROR        0x01
#define STS_NOT_PRESENT  0x02

// ── Биты режима ───────────────────────────────────────────────────────────
#define MODE_SIM         0x01  // симуляция датчиков
#define MODE_AUTO_GAP    0x02  // автоконтроль зазора
#define MODE_ESTOP       0x04  // E-stop активен
#define MODE_CYCLE       0x08  // цикл полировки активен

// ── Коды ошибок ───────────────────────────────────────────────────────────
#define ERR_UNKNOWN_CMD  0x01
#define ERR_BAD_CRC      0x02
#define ERR_SENSOR_NA    0x03
#define ERR_BAD_LEN      0x04
#define ERR_DEVICE_NA    0x05
#define ERR_ESTOP        0x06  // операция заблокирована E-stop
#define ERR_NOT_READY    0x07  // не готов (шаговый не включён и т.п.)

// ── Количество датчиков и устройств ──────────────────────────────────────
#define NUM_SENSORS      5
#define NUM_DEVICES      4

// ── Структуры ─────────────────────────────────────────────────────────────
struct Sensor {
  uint8_t id;
  uint8_t type;
  int16_t value;
  uint8_t status;
};

struct Device {
  uint8_t  id;
  uint8_t  type;
  int16_t  value;
  uint8_t  status;
};

// ── Глобальные переменные ─────────────────────────────────────────────────
static Sensor  g_sensors[NUM_SENSORS];
static Device  g_devices[NUM_DEVICES];
static uint8_t g_txBuf[PKT_MAX];
static uint8_t g_txLen  = 0;
static uint8_t g_rxBuf[PKT_MAX];
static uint8_t g_rxLen  = 0;
static uint8_t g_mode   = 0x00;

// ── ЭЭП-процесс: параметры импульсов и зазора ────────────────────────────
static uint16_t g_pulseFreq   = 1000;  // Гц (частота разрядных импульсов)
static uint8_t  g_pulseDuty   = 50;    // % скважность
static uint8_t  g_pulseLevel  = 0;     // 0..255, мощность (подаётся на PWM)
static int16_t  g_gapTarget   = 500;   // В×10, целевое U зазора (50.0В)

// ── Gap tracking: параметры регулятора ───────────────────────────────────
#define GAP_SC_V            30    // В×10 — порог КЗ (3.0В)
#define GAP_HYSTERESIS      25    // В×10 — гистерезис регулятора (2.5В)
#define GAP_RETRACT_FAST    30    // шагов — быстрый отвод при КЗ
#define GAP_CTRL_MS         10    // мс — период вызова регулятора

// ── Шаговый двигатель: пины (TODO: заполни сам) ──────────────────────────
// #define STEP_PIN_PUL    /* номер пина */   // PUL+
// #define STEP_PIN_DIR    /* номер пина */   // DIR+
// #define STEP_PIN_ENA    /* номер пина */   // ENA+ (активный LOW для DM556)
#define STEP_DEFAULT_SPEED  200   // шагов/сек (скорость подачи по умолчанию)
#define STEP_PUL_US           5   // длительность PUL, мкс (мин. 2.5 мкс для DM556)
#define STEP_DIR_SETUP_US     5   // задержка DIR→PUL, мкс

// ── Шаговый двигатель: состояние ─────────────────────────────────────────
static int32_t  g_stepPos    = 0;
static int32_t  g_stepTarget = 0;
static uint16_t g_stepSpeed  = STEP_DEFAULT_SPEED;
static bool     g_stepRun    = false;
static bool     g_stepEna    = false;
static bool     g_stepDir    = true;
static uint32_t g_stepLastUs = 0;

// ── CRC8 ──────────────────────────────────────────────────────────────────
static uint8_t crc8(const uint8_t *buf, uint8_t len) {
  uint8_t c = 0;
  for (uint8_t i = 0; i < len; i++) c ^= buf[i];
  return c;
}

// ── Построение пакетов ────────────────────────────────────────────────────
static void buildPkt(uint8_t resp, const uint8_t *data, uint8_t dlen) {
  if (dlen > PKT_DATA_MAX) dlen = PKT_DATA_MAX;
  g_txBuf[0] = PKT_START;
  g_txBuf[1] = resp;
  g_txBuf[2] = dlen;
  if (dlen) memcpy(&g_txBuf[3], data, dlen);
  g_txBuf[3 + dlen] = crc8(&g_txBuf[1], 2 + dlen);
  g_txLen = 4 + dlen;
}
static void buildACK()             { uint8_t d = 0; buildPkt(RESP_ACK, &d, 1); }
static void buildErr(uint8_t code) { buildPkt(RESP_ERROR, &code, 1); }

static void buildSensorPkt(int8_t sid) {
  if (sid >= (int8_t)NUM_SENSORS) { buildErr(ERR_SENSOR_NA); return; }
  uint8_t buf[PKT_DATA_MAX], p = 0;
  int8_t from = (sid < 0) ? 0 : sid;
  int8_t to   = (sid < 0) ? NUM_SENSORS : sid + 1;
  for (int8_t i = from; i < to && (p + 5) <= PKT_DATA_MAX; i++) {
    buf[p++] = g_sensors[i].id;
    buf[p++] = g_sensors[i].type;
    buf[p++] = (uint8_t)(g_sensors[i].value >> 8);
    buf[p++] = (uint8_t)(g_sensors[i].value & 0xFF);
    buf[p++] = g_sensors[i].status;
  }
  buildPkt(RESP_SENSOR_DATA, buf, p);
}

static void buildDevicePkt() {
  uint8_t buf[PKT_DATA_MAX], p = 0;
  for (uint8_t i = 0; i < NUM_DEVICES && (p + 5) <= PKT_DATA_MAX; i++) {
    buf[p++] = g_devices[i].id;
    buf[p++] = g_devices[i].type;
    buf[p++] = (uint8_t)(g_devices[i].value >> 8);
    buf[p++] = (uint8_t)(g_devices[i].value & 0xFF);
    buf[p++] = g_devices[i].status;
  }
  buildPkt(RESP_DEVICE_DATA, buf, p);
}

static void buildStepperStatus() {
  uint8_t buf[7];
  buf[0] = (uint8_t)((uint32_t)g_stepPos >> 24);
  buf[1] = (uint8_t)((uint32_t)g_stepPos >> 16);
  buf[2] = (uint8_t)((uint32_t)g_stepPos >>  8);
  buf[3] = (uint8_t)((uint32_t)g_stepPos);
  buf[4] = (uint8_t)(g_stepSpeed >> 8);
  buf[5] = (uint8_t)(g_stepSpeed);
  buf[6] = (g_stepRun ? 0x01 : 0) | (g_stepEna ? 0x02 : 0) | (g_stepDir ? 0x04 : 0);
  buildPkt(RESP_STEP_STATUS, buf, 7);
}

static void buildPulseParams() {
  uint8_t buf[6];
  buf[0] = (uint8_t)(g_pulseFreq >> 8);
  buf[1] = (uint8_t)(g_pulseFreq);
  buf[2] = g_pulseDuty;
  buf[3] = g_pulseLevel;
  buf[4] = (uint8_t)(g_gapTarget >> 8);
  buf[5] = (uint8_t)(g_gapTarget);
  buildPkt(RESP_PULSE_PARAMS, buf, 6);
}

// ═══════════════════════════════════════════════════════════════════════════
//  ДАТЧИКИ ЭЭП
//  КАК ДОБАВИТЬ ДАТЧИК: увеличить NUM_SENSORS, добавить запись в initSensors(),
//  написать readSensorN() по образцу, вызвать в updateSensorsReal() и
//  updateSensorsSimulated(). Тип добавить выше (SENSOR_XXX).
// ═══════════════════════════════════════════════════════════════════════════

void initSensors() {
  // Датчик 0: Напряжение на зазоре
  // Измерение: делитель напряжения (например 200В→5В, R1=390кОм, R2=10кОм)
  // analogRead(A0) * 400L / 1023 → В×10 (0..4000 для 0..400В)
  g_sensors[0] = {0, SENSOR_GAP_VOLT, 0, STS_NOT_PRESENT};

  // Датчик 1: Ток разряда
  // Измерение: датчик тока (ACS712-20A, TMCS1108) или шунт + опусилитель
  // analogRead(A1) * <scale> → мА
  g_sensors[1] = {1, SENSOR_CURR,     0, STS_NOT_PRESENT};

  // Датчик 2: Температура зоны обработки
  // Примеры: NTC-термистор (аналог), K-тип термопара + MAX6675 (SPI),
  //          DS18B20 (OneWire), инфракрасный MLX90614 (I2C 0x5A)
  g_sensors[2] = {2, SENSOR_TEMP,     0, STS_NOT_PRESENT};

  // Датчик 3: Амплитуда вибрации вибропривода
  // Примеры: акселерометр ADXL345 (I2C 0x53), MPU-6050, аналоговый пьезо
  g_sensors[3] = {3, SENSOR_VIBRO,    0, STS_NOT_PRESENT};

  // Датчик 4: Усилие прижима инструмента
  // Примеры: тензодатчик + HX711 (питание 3.3/5В, DATA+SCK → цифровые пины),
  //          FSR резистивный сенсор давления (аналог)
  g_sensors[4] = {4, SENSOR_FORCE,    0, STS_NOT_PRESENT};
}

// ── Симуляция ЭЭП-процесса: паттерн разряд/пауза ─────────────────────────
static int16_t triWave(uint32_t t, uint32_t period, int16_t center, int16_t amp) {
  uint32_t p = t % period;
  int32_t  v = (p < period / 2)
    ? -amp + (int32_t)p * (2L * amp) / (int32_t)(period / 2)
    : amp  - (int32_t)(p - period / 2) * (2L * amp) / (int32_t)(period / 2);
  return (int16_t)(center + v);
}

static void updateSensorsSimulated() {
  uint32_t t = millis();
  // Чередование: ~30% — разряд, 70% — пауза (типичный режим ЭЭП)
  bool discharge = ((t / 30) % 10 < 3);

  // Напряжение зазора: в паузе ~75В (750), во время разряда падает до 30-50В
  g_sensors[0].value  = discharge ? triWave(t, 20, 400, 100) : 750 - (int16_t)(t % 80);
  g_sensors[0].status = STS_OK;

  // Ток разряда: 0 в паузе, 500..3000 мА во время разряда
  g_sensors[1].value  = discharge ? (1500 + triWave(t, 15, 0, 1000)) : 0;
  g_sensors[1].status = STS_OK;

  // Температура: медленно нарастает при работе, °C×100
  int16_t tBase = 2000 + (int16_t)min((uint32_t)800, t / 600);
  g_sensors[2].value  = tBase + triWave(t, 8000, 0, 50);
  g_sensors[2].status = STS_OK;

  // Амплитуда вибрации: зависит от уставки вибропривода (устройство 2)
  uint8_t vibroPWM    = (uint8_t)constrain(g_devices[2].value, 0, 255);
  g_sensors[3].value  = (vibroPWM > 0) ? (int16_t)(vibroPWM / 4 + 5) : 0;
  g_sensors[3].status = STS_OK;

  // Усилие прижима: зависит от позиции Z (чем глубже — тем больше)
  int16_t force       = (int16_t)constrain(150 - g_stepPos / 2, 20, 500);
  g_sensors[4].value  = force + triWave(t, 2000, 0, 20);
  g_sensors[4].status = STS_OK;
}

// ── Датчик 0: Напряжение на зазоре ───────────────────────────────────────
static void readSensor0() {
  // Делитель: R1+R2 образуют делитель с коэффициентом K.
  // Формула: U_зазора[В×10] = analogRead(A0) * (5000L * (R1+R2)) / (1023 * R2 * 10)
  // Пример: R1=390кОм, R2=10кОм → K=40: value = analogRead(A0) * 400L / 1023
  //
  // int raw = analogRead(A0);
  // g_sensors[0].value = (int16_t)(raw * 400L / 1023);
  // g_sensors[0].status = STS_OK;
  g_sensors[0].value = 0;
}

// ── Датчик 1: Ток разряда ─────────────────────────────────────────────────
static void readSensor1() {
  // Пример ACS712-20A (±20А, 100мВ/А, 0А = 2.5В на Vcc=5В):
  // int raw = analogRead(A1);
  // int16_t mA = (int16_t)((raw - 512) * 5000L / 1023 / 100 * 1000);
  // g_sensors[1].value  = abs(mA);
  // g_sensors[1].status = STS_OK;
  //
  // Пример HX711 (тензодатчик):
  // #include <HX711.h>
  // static HX711 hx; static bool _i = (hx.begin(DATA_PIN, SCK_PIN), true);
  // if (hx.is_ready()) g_sensors[1].value = (int16_t)(hx.get_units(1) * 100);
  g_sensors[1].value = 0;
}

// ── Датчик 2: Температура ─────────────────────────────────────────────────
static void readSensor2() {
  // Пример NTC 10кОм (аналоговый, A2):
  // int raw = analogRead(A2);
  // float R   = 10000.0f * raw / (1023 - raw);
  // float T   = 1.0f / (log(R / 10000.0f) / 3950.0f + 1.0f / 298.15f) - 273.15f;
  // g_sensors[2].value  = (int16_t)(T * 100);
  // g_sensors[2].status = STS_OK;
  //
  // Пример MAX6675 (K-тип термопара, SPI):
  // #include <max6675.h>
  // static MAX6675 tc(SCK_PIN, CS_PIN, MISO_PIN);
  // g_sensors[2].value  = (int16_t)(tc.readCelsius() * 100);
  // g_sensors[2].status = STS_OK;
  g_sensors[2].value = 0;
}

// ── Датчик 3: Амплитуда вибрации ──────────────────────────────────────────
static void readSensor3() {
  // Пример ADXL345 (I2C 0x53):
  // #include <Adafruit_ADXL345_U.h>
  // static Adafruit_ADXL345_Unified accel(12345);
  // static bool _i = (accel.begin(), g_sensors[3].status = STS_OK, true);
  // sensors_event_t evt; accel.getEvent(&evt);
  // float amp = sqrt(evt.acceleration.x*evt.acceleration.x +
  //                  evt.acceleration.z*evt.acceleration.z);
  // g_sensors[3].value = (int16_t)(amp * 10);  // мкм — нужна калибровка
  g_sensors[3].value = 0;
}

// ── Датчик 4: Усилие прижима ──────────────────────────────────────────────
static void readSensor4() {
  // Пример HX711 + тензодатчик 5кг:
  // #include <HX711.h>
  // static HX711 hx;
  // static bool _i = (hx.begin(A8, A9), hx.set_scale(2280.f), hx.tare(), true);
  // if (hx.is_ready()) {
  //   g_sensors[4].value  = (int16_t)constrain(hx.get_units(3), 0, 32767);
  //   g_sensors[4].status = STS_OK;
  // }
  //
  // Пример FSR (аналог A3):
  // int raw = analogRead(A3);
  // g_sensors[4].value  = (int16_t)(raw * 500L / 1023);  // г, грубо
  // g_sensors[4].status = STS_OK;
  g_sensors[4].value = 0;
}

// ── Обновление датчиков ───────────────────────────────────────────────────
static void updateSensorsReal() {
  readSensor0();
  readSensor1();
  readSensor2();
  readSensor3();
  readSensor4();
  // Новый датчик: readSensorN();
}

static void updateSensors() {
  if (g_mode & MODE_SIM) updateSensorsSimulated();
  else                   updateSensorsReal();
}

// ═══════════════════════════════════════════════════════════════════════════
//  УСТРОЙСТВА УПРАВЛЕНИЯ ЭЭП
//  КАК ДОБАВИТЬ: увеличить NUM_DEVICES, добавить в initDevices(),
//  написать applyDeviceN(), добавить case в applyDevice().
// ═══════════════════════════════════════════════════════════════════════════

void initDevices() {
  // Устройство 0: Силовое реле (главный коммутатор питания разрядной цепи)
  // Включать ТОЛЬКО после проверки всех условий безопасности.
  g_devices[0] = {0, DEV_RELAY, 0, STS_OK};
  // pinMode(PIN_RELAY_MAIN, OUTPUT); digitalWrite(PIN_RELAY_MAIN, LOW);

  // Устройство 1: Реле включения генератора разрядных импульсов
  // Включается после силового реле.
  g_devices[1] = {1, DEV_RELAY, 0, STS_OK};
  // pinMode(PIN_RELAY_GEN, OUTPUT); digitalWrite(PIN_RELAY_GEN, LOW);

  // Устройство 2: ШИМ вибропривода (амплитуда вибрации для распределения
  // сухого электролита). 0=стоп, 255=максимум.
  g_devices[2] = {2, DEV_PWM,   0, STS_OK};
  // pinMode(PIN_VIBRO_PWM, OUTPUT); analogWrite(PIN_VIBRO_PWM, 0);

  // Устройство 3: ШИМ вентилятора охлаждения
  g_devices[3] = {3, DEV_PWM,   0, STS_OK};
  // pinMode(PIN_FAN_PWM, OUTPUT); analogWrite(PIN_FAN_PWM, 0);
}

static void applyDevice0(int16_t v) {
  g_devices[0].value = v ? 1 : 0;
  // digitalWrite(PIN_RELAY_MAIN, v ? HIGH : LOW);
}
static void applyDevice1(int16_t v) {
  g_devices[1].value = v ? 1 : 0;
  // digitalWrite(PIN_RELAY_GEN, v ? HIGH : LOW);
}
static void applyDevice2(int16_t v) {
  g_devices[2].value = constrain(v, 0, 255);
  // analogWrite(PIN_VIBRO_PWM, (uint8_t)g_devices[2].value);
}
static void applyDevice3(int16_t v) {
  g_devices[3].value = constrain(v, 0, 255);
  // analogWrite(PIN_FAN_PWM, (uint8_t)g_devices[3].value);
}

static void applyDevice(uint8_t id, int16_t value) {
  if (id >= NUM_DEVICES) return;
  switch (id) {
    case 0: applyDevice0(value); break;
    case 1: applyDevice1(value); break;
    case 2: applyDevice2(value); break;
    case 3: applyDevice3(value); break;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  ШАГОВЫЙ ДВИГАТЕЛЬ Z — ПОДАЧА ИНСТРУМЕНТА (DM556)
// ═══════════════════════════════════════════════════════════════════════════

void initStepper() {
  g_stepPos = 0; g_stepTarget = 0; g_stepSpeed = STEP_DEFAULT_SPEED;
  g_stepRun = false; g_stepEna = false; g_stepDir = true; g_stepLastUs = 0;
  // TODO: раскомментируй после задания пинов:
  // pinMode(STEP_PIN_PUL, OUTPUT); digitalWrite(STEP_PIN_PUL, LOW);
  // pinMode(STEP_PIN_DIR, OUTPUT); digitalWrite(STEP_PIN_DIR, HIGH);
  // pinMode(STEP_PIN_ENA, OUTPUT); digitalWrite(STEP_PIN_ENA, HIGH); // HIGH=выкл
}

static void updateStepper() {
  if (!g_stepRun || !g_stepEna) return;
  if (g_stepPos == g_stepTarget) { g_stepRun = false; return; }

  uint32_t now      = micros();
  uint32_t interval = 1000000UL / (uint32_t)g_stepSpeed;
  if ((uint32_t)(now - g_stepLastUs) < interval) return;
  g_stepLastUs = now;

  bool dir = (g_stepTarget > g_stepPos);
  if (dir != g_stepDir) {
    g_stepDir = dir;
    // ── РЕАЛЬНОЕ ВРАЩЕНИЕ: раскомментируй после задания STEP_PIN_DIR ──────
    // digitalWrite(STEP_PIN_DIR, dir ? HIGH : LOW);
    // delayMicroseconds(STEP_DIR_SETUP_US);
  }
  // ── РЕАЛЬНОЕ ВРАЩЕНИЕ: раскомментируй после задания STEP_PIN_PUL ────────
  // digitalWrite(STEP_PIN_PUL, HIGH);
  // delayMicroseconds(STEP_PUL_US);
  // digitalWrite(STEP_PIN_PUL, LOW);
  //
  // СИМУЛЯЦИЯ: g_stepPos обновляется всегда — даже без реальных пинов.
  // RPi может читать позицию через CMD_GET_STEPPER и видеть движение.
  // Для реального вращения DM556: подключи ENA+/PUL+/DIR+ к пинам Arduino
  // (ENA-/PUL-/DIR- → GND уже подключены), задай STEP_PIN_* выше и
  // раскомментируй строки digitalWrite/delayMicroseconds выше.
  g_stepPos += dir ? 1 : -1;
  if (g_stepPos == g_stepTarget) {
    g_stepRun = false;
  }
}

// ═══════════════════════════════════════════════════════════════════════════
//  ЭЭП-ПРОЦЕСС: АВАРИЙНАЯ ОСТАНОВКА И КОНТРОЛЬ ЗАЗОРА
// ═══════════════════════════════════════════════════════════════════════════

// Немедленная аварийная остановка всего процесса
static void doEStop() {
  // 1. Остановить шаговый
  g_stepRun    = false;
  g_stepTarget = g_stepPos;
  g_stepEna    = false;
  // TODO: digitalWrite(STEP_PIN_ENA, HIGH);  // disable DM556

  // 2. Обесточить разрядную цепь
  applyDevice(1, 0);  // генератор выкл
  applyDevice(0, 0);  // силовое реле выкл
  applyDevice(2, 0);  // вибрация выкл

  // 3. Установить флаги
  g_mode = (g_mode | MODE_ESTOP) & ~(MODE_AUTO_GAP | MODE_CYCLE);

  Serial.println(F("[ESTOP!] Аварийная остановка"));
}

// Регулятор зазора: вызывается из loop() каждые GAP_CTRL_MS мс
static void updateGapControl() {
  if (!(g_mode & MODE_AUTO_GAP)) return;
  if (g_mode & MODE_ESTOP)       return;
  if (!g_stepEna)                return;
  if (g_sensors[0].status != STS_OK) return;

  int16_t U = g_sensors[0].value;  // текущее U зазора (В×10)

  if (U <= GAP_SC_V) {
    // ── КЗ: срочный отвод ────────────────────────────────────────────────
    applyDevice(1, 0);   // генератор выкл на время отвода
    uint16_t savedSpeed = g_stepSpeed;
    g_stepSpeed  = 2000;  // быстрый отвод
    g_stepTarget = g_stepPos + GAP_RETRACT_FAST;
    g_stepRun    = true;
    g_stepLastUs = micros();
    g_stepSpeed  = savedSpeed;  // вернём скорость для следующего хода
    Serial.print(F("[GAP] КЗ! U="));
    Serial.println(U);
  } else if (U < g_gapTarget - GAP_HYSTERESIS) {
    // ── Зазор мал → отвод на 1 шаг ──────────────────────────────────────
    if (!g_stepRun) {
      g_stepTarget = g_stepPos + 1;
      g_stepRun    = true;
      g_stepLastUs = micros();
    }
  } else if (U > g_gapTarget + GAP_HYSTERESIS) {
    // ── Зазор велик → подача на 1 шаг ───────────────────────────────────
    if (!g_stepRun) {
      g_stepTarget = g_stepPos - 1;
      g_stepRun    = true;
      g_stepLastUs = micros();
    }
  }
  // В зоне допуска — держим позицию, ничего не делаем
}

// ── Применение новых параметров импульсов ─────────────────────────────────
static void applyPulseParams() {
  // TODO: если Arduino сам генерирует импульсы через таймер:
  //   Timer1.setPeriod(1000000L / g_pulseFreq);
  //   Timer1.setPwmDuty(PIN_PULSE_OUT, (uint16_t)(g_pulseDuty * 1023 / 100));
  // TODO: если внешний генератор управляется ШИМ/SPI — добавь здесь
  //   analogWrite(PIN_GEN_LEVEL, g_pulseLevel);  // мощность
}

// ── Обработка команд ──────────────────────────────────────────────────────
static void processCommand() {
  if (g_rxLen < 4 || g_rxBuf[0] != PKT_START) { buildErr(ERR_BAD_LEN); return; }
  uint8_t cmd  = g_rxBuf[1];
  uint8_t dlen = g_rxBuf[2];
  uint8_t *d   = &g_rxBuf[3];

  if (g_rxLen < (uint8_t)(4 + dlen)) { buildErr(ERR_BAD_LEN); return; }
  if (crc8(&g_rxBuf[1], 2 + dlen) != g_rxBuf[3 + dlen]) { buildErr(ERR_BAD_CRC); return; }

  switch (cmd) {
    case CMD_PING:       buildACK(); break;
    case CMD_GET_ALL:    buildSensorPkt(-1); break;
    case CMD_GET_SENSOR:
      if (dlen < 1) { buildErr(ERR_BAD_LEN); return; }
      buildSensorPkt((int8_t)d[0]); break;
    case CMD_SET_MODE:
      if (dlen < 1) { buildErr(ERR_BAD_LEN); return; }
      // Нельзя снять E-stop через SET_MODE, только через ESTOP_RESET
      if (g_mode & MODE_ESTOP) d[0] |= MODE_ESTOP;
      g_mode = d[0]; buildACK(); break;
    case CMD_RESET:
      doEStop();
      initSensors(); initDevices(); initStepper();
      g_mode = 0x00;
      g_gapTarget  = 500; g_pulseFreq = 1000;
      g_pulseDuty  = 50;  g_pulseLevel = 0;
      buildACK(); break;
    case CMD_GET_STATUS: {
      uint8_t st[] = {g_mode, (uint8_t)NUM_SENSORS, (uint8_t)NUM_DEVICES};
      buildPkt(RESP_STATUS, st, 3); break;
    }

    // ── Устройства ────────────────────────────────────────────────────────
    case CMD_SET_DEVICE: {
      if (dlen < 3) { buildErr(ERR_BAD_LEN); return; }
      if (g_mode & MODE_ESTOP) { buildErr(ERR_ESTOP); return; }
      uint8_t did = d[0];
      if (did >= NUM_DEVICES) { buildErr(ERR_DEVICE_NA); return; }
      applyDevice(did, (int16_t)((uint16_t)d[1] << 8 | d[2]));
      buildACK(); break;
    }
    case CMD_GET_DEVICES: buildDevicePkt(); break;

    // ── Шаговый двигатель ─────────────────────────────────────────────────
    case CMD_STEP_MOVE: {
      if (dlen < 4) { buildErr(ERR_BAD_LEN); return; }
      if (g_mode & MODE_ESTOP) { buildErr(ERR_ESTOP); return; }
      int32_t  steps = (int32_t)((uint32_t)d[0]<<24|(uint32_t)d[1]<<16|(uint32_t)d[2]<<8|d[3]);
      uint16_t spd   = (dlen >= 6) ? ((uint16_t)d[4]<<8|d[5]) : 0;
      if (spd > 0) g_stepSpeed = spd;
      g_stepTarget = g_stepPos + steps;
      if (steps != 0 && g_stepEna) { g_stepRun = true; g_stepLastUs = micros(); }
      buildACK(); break;
    }
    case CMD_STEP_GOTO: {
      if (dlen < 4) { buildErr(ERR_BAD_LEN); return; }
      if (g_mode & MODE_ESTOP) { buildErr(ERR_ESTOP); return; }
      int32_t  pos = (int32_t)((uint32_t)d[0]<<24|(uint32_t)d[1]<<16|(uint32_t)d[2]<<8|d[3]);
      uint16_t spd = (dlen >= 6) ? ((uint16_t)d[4]<<8|d[5]) : 0;
      if (spd > 0) g_stepSpeed = spd;
      g_stepTarget = pos;
      if (g_stepPos != g_stepTarget && g_stepEna) { g_stepRun = true; g_stepLastUs = micros(); }
      buildACK(); break;
    }
    case CMD_STEP_STOP:
      g_stepRun = false; g_stepTarget = g_stepPos; buildACK(); break;
    case CMD_STEP_HOME:
      g_stepRun = false; g_stepPos = 0; g_stepTarget = 0; buildACK(); break;
    case CMD_STEP_ENA:
      if (dlen < 1) { buildErr(ERR_BAD_LEN); return; }
      if (g_mode & MODE_ESTOP) { buildErr(ERR_ESTOP); return; }
      g_stepEna = (d[0] != 0);
      if (!g_stepEna) g_stepRun = false;
      // TODO: digitalWrite(STEP_PIN_ENA, g_stepEna ? LOW : HIGH);
      buildACK(); break;
    case CMD_GET_STEPPER:
      buildStepperStatus(); break;

    // ── ЭЭП-процесс ──────────────────────────────────────────────────────
    case CMD_SET_PULSE:
      if (dlen < 4) { buildErr(ERR_BAD_LEN); return; }
      g_pulseFreq  = ((uint16_t)d[0] << 8) | d[1];
      g_pulseDuty  = constrain(d[2], 1, 99);
      g_pulseLevel = d[3];
      applyPulseParams();
      buildACK(); break;

    case CMD_GET_PULSE:
      buildPulseParams(); break;

    case CMD_SET_GAP_TGT:
      if (dlen < 2) { buildErr(ERR_BAD_LEN); return; }
      g_gapTarget = (int16_t)((uint16_t)d[0] << 8 | d[1]);
      buildACK(); break;

    case CMD_ESTOP:
      doEStop(); buildACK(); break;

    case CMD_ESTOP_RESET:
      if (g_mode & MODE_ESTOP) {
        g_mode &= ~MODE_ESTOP;
        Serial.println(F("[ESTOP] Сброшен"));
      }
      buildACK(); break;

    case CMD_START_CYCLE:
      if (g_mode & MODE_ESTOP)  { buildErr(ERR_ESTOP);     return; }
      if (!g_stepEna)            { buildErr(ERR_NOT_READY); return; }
      if (!g_devices[0].value)   { buildErr(ERR_NOT_READY); return; }  // силовое реле выкл
      g_mode |= (MODE_AUTO_GAP | MODE_CYCLE);
      applyDevice(1, 1);    // генератор включить
      applyDevice(2, 128);  // вибрация 50%
      applyPulseParams();
      Serial.println(F("[CYCLE] Старт"));
      buildACK(); break;

    case CMD_STOP_CYCLE:
      g_mode &= ~(MODE_AUTO_GAP | MODE_CYCLE);
      g_stepRun = false; g_stepTarget = g_stepPos;
      applyDevice(1, 0);   // генератор выкл
      applyDevice(2, 0);   // вибрация выкл
      Serial.println(F("[CYCLE] Стоп"));
      buildACK(); break;

    default:
      buildErr(ERR_UNKNOWN_CMD); break;
  }
}

// ── I2C колбэки ───────────────────────────────────────────────────────────
void onReceive(int n) {
  g_rxLen = 0;
  while (Wire.available() && g_rxLen < PKT_MAX)
    g_rxBuf[g_rxLen++] = Wire.read();
  processCommand();
}
void onRequest() {
  Wire.write(g_txBuf, g_txLen > 0 ? g_txLen : 1);
}

// ── Инициализация ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  initSensors();
  initDevices();
  initStepper();
  buildACK();

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.println(F("=== ЭЭП-станок готов ==="));
  Serial.print(F("I2C addr=0x")); Serial.println(I2C_SLAVE_ADDR, HEX);
  Serial.println(F("Пины шагового (STEP_PIN_*) не назначены — см. TODO"));
}

// ── Основной цикл ─────────────────────────────────────────────────────────
void loop() {
  static uint32_t lastSensorMs  = 0;
  static uint32_t lastGapCtrlMs = 0;
  uint32_t now = millis();

  // Датчики: каждые 50 мс
  if (now - lastSensorMs >= 50) {
    lastSensorMs = now;
    updateSensors();
  }

  // Регулятор зазора: каждые 10 мс
  if (now - lastGapCtrlMs >= GAP_CTRL_MS) {
    lastGapCtrlMs = now;
    updateGapControl();
  }

  // Генерация шагов: каждую итерацию
  updateStepper();
}
