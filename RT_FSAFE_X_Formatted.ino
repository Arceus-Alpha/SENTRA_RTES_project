#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPSPlus.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

// ESP32 + MPU6500 + OLED + SD + GPS fallback + FreeRTOS

#define SDA_PIN             21
#define SCL_PIN             22
#define SD_CS_PIN            5
#define GREEN_LED_PIN       25
#define RED_LED_PIN         26
#define BUZZER_PIN          27
#define GPS_RX_PIN          16
#define GPS_TX_PIN          17
#define GPS_BAUDRATE      9600

// Change to 1 if your LEDs are active-low.
#define LED_ACTIVE_LOW       0

#define MPU_ADDR          0x68
#define OLED_ADDRESS      0x3C
#define OLED_WIDTH         128
#define OLED_HEIGHT         64
#define OLED_RESET          -1

#define IMU_PERIOD_MS        10
#define OLED_PERIOD_MS      200
#define SUPERVISOR_PERIOD_MS 100
#define LED_PERIOD_MS      1000
#define LOGGER_PERIOD_MS   2000
#define GPS_REPORT_MS      1000
#define DEBUG_REPORT_MS    1000
#define QUEUE_LENGTH        250
#define WATCHDOG_TIMEOUT_S    5

#define ACCEL_SCALE (9.80665f / 4096.0f)
#define GYRO_SCALE  (PI / (180.0f * 65.5f))
#define FILTER_ALPHA 0.98f
#define PITCH_PIXELS_PER_DEG 0.65f

// Example fallback values for testing only.
// These are NOT a real GPS fix.
#define FALLBACK_LATITUDE   13.0827
#define FALLBACK_LONGITUDE  80.2707
#define FALLBACK_ALTITUDE    6.0

const char* FILE_NAME = "/imu_data.csv";

Adafruit_SSD1306 display(
  OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET
);

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

struct IMUData {
  uint32_t time_ms;
  float ax, ay, az;
  float gx, gy, gz;
  float temperature;
  float roll, pitch;
};

struct GPSData {
  double latitude;
  double longitude;
  double altitude;
  uint32_t satellites;
  bool validFix;
  bool usingFallback;
};

// Static batch storage prevents logger stack overflow.
static IMUData loggerBatch[QUEUE_LENGTH];

// RTOS resources
QueueHandle_t imuQueue = nullptr;
SemaphoreHandle_t i2cMutex = nullptr;
SemaphoreHandle_t sdMutex = nullptr;

// GPS data is shared between GPS and logger tasks.
portMUX_TYPE gpsMux = portMUX_INITIALIZER_UNLOCKED;

GPSData gpsData = {
  FALLBACK_LATITUDE,
  FALLBACK_LONGITUDE,
  FALLBACK_ALTITUDE,
  0,
  false,
  true
};

// Live attitude values
volatile float filteredRoll = 0.0f;
volatile float filteredPitch = 0.0f;

// System status
volatile bool systemError = false;
bool imuReady = false;
bool sdReady = false;
bool oledReady = false;

volatile uint32_t imuReadErrors = 0;
volatile uint32_t imuConsecutiveFailures = 0;
volatile bool sdFault = false;
volatile uint32_t i2cBusySkips = 0;
volatile bool lastReadBusBusy = false;
volatile uint32_t droppedSamples = 0;
volatile uint32_t loggedSamples = 0;
volatile uint32_t imuDeadlineMisses = 0;
volatile uint32_t imuMaxExecUs = 0;
volatile uint32_t imuLastHeartbeat = 0;
volatile uint32_t oledHeartbeat = 0;
volatile uint32_t gpsHeartbeat = 0;
volatile uint32_t loggerHeartbeat = 0;
volatile uint32_t queueHighWater = 0;
volatile uint32_t maxImuPeriodMs = 0;
volatile uint32_t loggerWriteErrors = 0;

TaskHandle_t imuTaskHandle = nullptr;
TaskHandle_t supervisorTaskHandle = nullptr;
TaskHandle_t oledTaskHandle = nullptr;
TaskHandle_t gpsTaskHandle = nullptr;
TaskHandle_t loggerTaskHandle = nullptr;
TaskHandle_t ledTaskHandle = nullptr;

void setLed(uint8_t pin, bool on) {
#if LED_ACTIVE_LOW
  digitalWrite(pin, on ? LOW : HIGH);
#else
  digitalWrite(pin, on ? HIGH : LOW);
#endif
}

bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool initMPU6500() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);

  if (Wire.endTransmission(false) != 0) {
    Serial.println("[IMU] WHO_AM_I communication failed");
    return false;
  }

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1) {
    Serial.println("[IMU] WHO_AM_I read failed");
    return false;
  }

  uint8_t who = Wire.read();

  if (who != 0x70 && who != 0x71 && who != 0x68) {
    Serial.printf("[IMU] Unexpected ID: 0x%02X\n", who);
    return false;
  }

  if (!writeMPURegister(0x6B, 0x01)) return false;
  delay(100);

  // Nominal 100 Hz sample rate.
  if (!writeMPURegister(0x19, 0x09)) return false;

  // Digital low-pass filter.
  if (!writeMPURegister(0x1A, 0x04)) return false;

  // Gyroscope: +/-500 degrees/second.
  if (!writeMPURegister(0x1B, 0x08)) return false;

  // Accelerometer: +/-8g.
  if (!writeMPURegister(0x1C, 0x10)) return false;

  if (!writeMPURegister(0x1D, 0x04)) return false;

  Serial.printf("[IMU] MPU6500 initialized, ID=0x%02X\n", who);
  return true;
}

bool readIMU(IMUData &d) {
  uint8_t bytes[14];

  // Never block the high-priority 10 ms IMU task behind an OLED frame.
  // If the bus is occupied, skip this sample and count it separately.
  if (xSemaphoreTake(i2cMutex, 0) != pdTRUE) {
    lastReadBusBusy = true;
    return false;
  }
  lastReadBusBusy = false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);

  bool txOK = Wire.endTransmission(false) == 0;
  size_t received = 0;

  if (txOK) {
    received = Wire.requestFrom(
      (uint8_t)MPU_ADDR, (uint8_t)14
    );
  }

  if (!txOK || received != 14) {
    while (Wire.available()) Wire.read();
    xSemaphoreGive(i2cMutex);
    return false;
  }

  for (int i = 0; i < 14; i++) {
    bytes[i] = Wire.read();
  }

  xSemaphoreGive(i2cMutex);

  auto makeInt16 = [](uint8_t hi, uint8_t lo) -> int16_t {
    return (int16_t)(((uint16_t)hi << 8) | lo);
  };

  int16_t rawAx = makeInt16(bytes[0], bytes[1]);
  int16_t rawAy = makeInt16(bytes[2], bytes[3]);
  int16_t rawAz = makeInt16(bytes[4], bytes[5]);
  int16_t rawTemp = makeInt16(bytes[6], bytes[7]);
  int16_t rawGx = makeInt16(bytes[8], bytes[9]);
  int16_t rawGy = makeInt16(bytes[10], bytes[11]);
  int16_t rawGz = makeInt16(bytes[12], bytes[13]);

  d.time_ms = millis();

  d.ax = rawAx * ACCEL_SCALE;
  d.ay = rawAy * ACCEL_SCALE;
  d.az = rawAz * ACCEL_SCALE;

  d.gx = rawGx * GYRO_SCALE;
  d.gy = rawGy * GYRO_SCALE;
  d.gz = rawGz * GYRO_SCALE;

  d.temperature = ((float)rawTemp / 333.87f) + 21.0f;

  d.roll = atan2f(d.ay, d.az) * 180.0f / PI;

  d.pitch = atan2f(
    -d.ax,
    sqrtf(d.ay * d.ay + d.az * d.az)
  ) * 180.0f / PI;

  return true;
}

bool initSDCard() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("[SD] Initialization failed");
    return false;
  }

  File file = SD.open(FILE_NAME, FILE_APPEND);

  if (!file) {
    Serial.println("[SD] Cannot open CSV");
    return false;
  }

  if (file.size() == 0) {
    file.println(
      "Time_ms,Accel_X_m_s2,Accel_Y_m_s2,Accel_Z_m_s2,"
      "Gyro_X_rad_s,Gyro_Y_rad_s,Gyro_Z_rad_s,"
      "Temp_C,Roll_deg,Pitch_deg,"
      "Latitude,Longitude,Altitude_m,Satellites,GPS_Fix,GPS_Fallback"
    );
    file.flush();
  }

  bool ok = file.getWriteError() == 0;
  file.close();

  if (ok) {
    Serial.println("[SD] CSV ready");
  } else {
    Serial.println("[SD] Header write failed");
  }

  return ok;
}

//
// RTES concepts:
// - Periodic scheduling
// - Fixed-period sampling
// - Task priority
// - Execution-time monitoring
// - Deadline-miss counter
// - Producer-consumer queue

void imuTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(IMU_PERIOD_MS);

  bool initialized = false;
  uint32_t previousSampleMs = 0;
  uint32_t previousStartMs = millis();

  float rollEstimate = 0.0f;
  float pitchEstimate = 0.0f;

  // Register this task with the ESP task watchdog.
  esp_task_wdt_add(nullptr);

  for (;;) {
    uint32_t startUs = micros();
    uint32_t startMs = millis();

    uint32_t actualPeriod = startMs - previousStartMs;
    previousStartMs = startMs;

    if (actualPeriod > maxImuPeriodMs) {
      maxImuPeriodMs = actualPeriod;
    }

    IMUData sample;

    if (readIMU(sample)) {
      imuConsecutiveFailures = 0;
      uint32_t now = sample.time_ms;

      if (!initialized) {
        rollEstimate = sample.roll;
        pitchEstimate = sample.pitch;
        initialized = true;
      } else {
        float dt = (now - previousSampleMs) / 1000.0f;

        if (dt > 0.0f && dt <= 0.1f) {
          float rollRate = sample.gx * 180.0f / PI;
          float pitchRate = sample.gy * 180.0f / PI;

          rollEstimate =
          FILTER_ALPHA * (rollEstimate + rollRate * dt) +
          (1.0f - FILTER_ALPHA) * sample.roll;

          pitchEstimate =
          FILTER_ALPHA * (pitchEstimate + pitchRate * dt) +
          (1.0f - FILTER_ALPHA) * sample.pitch;
        }
      }

      previousSampleMs = now;

      filteredRoll = rollEstimate;
      filteredPitch = pitchEstimate;

      sample.roll = rollEstimate;
      sample.pitch = pitchEstimate;

      // GPS validity never gates IMU logging.
      if (sdReady && imuQueue != nullptr) {
        if (xQueueSend(imuQueue, &sample, 0) != pdTRUE) {
          droppedSamples++;
        } else {
          uint32_t used =
          QUEUE_LENGTH - uxQueueSpacesAvailable(imuQueue);

          if (used > queueHighWater) {
            queueHighWater = used;
          }
        }
      }

    } else {
      if (lastReadBusBusy) {
        i2cBusySkips++;
      } else {
        imuReadErrors++;
        if (imuConsecutiveFailures < 1000000UL) {
          imuConsecutiveFailures++;
        }
        // Five consecutive real read failures indicate a likely IMU fault.
        if (imuConsecutiveFailures >= 5) {
          systemError = true;
        }
      }
    }

    uint32_t executionUs = micros() - startUs;

    if (executionUs > imuMaxExecUs) {
      imuMaxExecUs = executionUs;
    }

    // Count execution overruns and late periodic releases.
    if (executionUs >= IMU_PERIOD_MS * 1000UL ||
      actualPeriod > IMU_PERIOD_MS + 1) {
      imuDeadlineMisses++;
    }

    imuLastHeartbeat = millis();

    // Reset the watchdog after completing a cycle.
    esp_task_wdt_reset();

    vTaskDelayUntil(&lastWake, period);
  }
}

void drawHorizon(float roll, float pitch) {
  display.clearDisplay();

  const float rad = roll * PI / 180.0f;
  const float cs = cosf(rad);
  const float sn = sinf(rad);

  const int cx = 64;
  const int cy = 32;

  float horizonY = cy + pitch * PITCH_PIXELS_PER_DEG;

  int y1 = (int)roundf(horizonY - sn * (0 - cx));
  int y2 = (int)roundf(horizonY - sn * (127 - cx));

  // Sky pattern.
  for (int x = 5; x < 128; x += 12) {
    for (int offset = 7; offset <= 19; offset += 6) {
      int yy = (int)roundf(
        horizonY - sn * (x - cx) - offset * cs
      );

      if (yy >= 2 && yy < 62) {
        display.drawLine(x, yy, x + 3, yy, WHITE);
      }
    }
  }

  // Ground pattern.
  for (int x = 5; x < 128; x += 12) {
    for (int offset = 7; offset <= 19; offset += 6) {
      int yy = (int)roundf(
        horizonY - sn * (x - cx) + offset * cs
      );

      if (yy >= 2 && yy < 62) {
        display.drawPixel(x, yy, WHITE);
      }
    }
  }

  display.drawLine(0, y1, 127, y2, WHITE);

  // Pitch ladder.
  for (int deg = -20; deg <= 20; deg += 10) {
    if (deg == 0) continue;

    float offset = -deg * PITCH_PIXELS_PER_DEG;
    float tx = cx + sn * offset;
    float ty = horizonY + cs * offset;

    int halfWidth = (abs(deg) == 10) ? 8 : 12;

    display.drawLine(
      (int)roundf(tx - halfWidth * cs),
      (int)roundf(ty + halfWidth * sn),
      (int)roundf(tx + halfWidth * cs),
      (int)roundf(ty - halfWidth * sn),
      WHITE
    );
  }

  // Aircraft symbol.
  display.drawLine(42, 31, 57, 31, WHITE);
  display.drawLine(71, 31, 86, 31, WHITE);
  display.drawLine(57, 31, 62, 35, WHITE);
  display.drawLine(71, 31, 66, 35, WHITE);
  display.drawLine(62, 35, 66, 35, WHITE);
  display.drawLine(64, 27, 64, 31, WHITE);

  // Roll scale.
  for (int a = -60; a <= 60; a += 30) {
    float angle = a * PI / 180.0f;

    int x = 64 + (int)roundf(22 * sinf(angle));
    int y = 9 - (int)roundf(5 * cosf(angle));

    display.drawLine(x, y, x, y + 4, WHITE);
  }

  display.fillTriangle(64, 10, 60, 4, 68, 4, WHITE);

  display.fillRect(1, 54, 43, 9, BLACK);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(2, 55);
  display.printf("R%.0f", roll);

  display.fillRect(84, 54, 43, 9, BLACK);
  display.setCursor(85, 55);
  display.printf("P%.0f", pitch);

  display.drawRoundRect(0, 0, 128, 64, 4, WHITE);
}

//
// Low-priority periodic task. Frame counter changes on every
// refresh, so you can confirm that the display is updating.

void oledTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(OLED_PERIOD_MS);

  uint32_t frame = 0;
  uint32_t lastOledDiagnostic = 0;
  Serial.println("[OLED] Update task started");

  for (;;) {
    frame++;

    // OLED runs on core 0, separate from the high-priority IMU task.
    // A short wait is safe here because this task is low priority.
    if (oledReady &&
      xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE) {

      if (imuReady) {
        drawHorizon(
          (float)filteredRoll,
          (float)filteredPitch
        );

        display.fillRect(91, 2, 34, 8, BLACK);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(92, 2);
        display.printf("%lu", (unsigned long)(frame % 10000));
      } else {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0, 0);
        display.println("RT-FSAFE X");
        display.println("OLED REFRESH ACTIVE");
        display.println("MPU6500 NOT READY");
        display.println("Check SDA/SCL");
        display.printf("Frame:%lu", (unsigned long)frame);
      }

      display.display();
      xSemaphoreGive(i2cMutex);
    }

    oledHeartbeat = millis();

    if (millis() - lastOledDiagnostic >= 5000) {
      lastOledDiagnostic = millis();
      Serial.printf("[OLED] Refresh task alive, frame=%lu\n",
        (unsigned long)frame);
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

//
// GPS is optional. A missing GPS fix does not set systemError.

void gpsTask(void* parameter) {
  TickType_t lastReport = xTaskGetTickCount();
  const TickType_t reportPeriod = pdMS_TO_TICKS(GPS_REPORT_MS);

  for (;;) {
    while (GPSSerial.available() > 0) {
      gps.encode((char)GPSSerial.read());
    }

    bool realFix =
    gps.location.isValid() &&
    gps.location.age() < 5000;

    GPSData snapshot;

    if (realFix) {
      snapshot.latitude = gps.location.lat();
      snapshot.longitude = gps.location.lng();

      snapshot.altitude = gps.altitude.isValid()
      ? gps.altitude.meters()
      : FALLBACK_ALTITUDE;

      snapshot.satellites = gps.satellites.isValid()
      ? gps.satellites.value()
      : 0;

      snapshot.validFix = true;
      snapshot.usingFallback = false;
    } else {
      snapshot.latitude = FALLBACK_LATITUDE;
      snapshot.longitude = FALLBACK_LONGITUDE;
      snapshot.altitude = FALLBACK_ALTITUDE;

      snapshot.satellites = gps.satellites.isValid()
      ? gps.satellites.value()
      : 0;

      snapshot.validFix = false;
      snapshot.usingFallback = true;
    }

    portENTER_CRITICAL(&gpsMux);
    gpsData = snapshot;
    portEXIT_CRITICAL(&gpsMux);

    gpsHeartbeat = millis();

    if ((xTaskGetTickCount() - lastReport) >= reportPeriod) {
      lastReport = xTaskGetTickCount();

      Serial.printf(
        "[GPS] %s | Lat=%.6f Lon=%.6f Alt=%.1f m Sats=%lu\n",
        snapshot.validFix ? "REAL FIX" : "FALLBACK",
        snapshot.latitude,
        snapshot.longitude,
        snapshot.altitude,
        (unsigned long)snapshot.satellites
      );
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//
// Producer-consumer architecture:
// IMU task -> bounded queue -> logger task.
// The SD mutex protects access to the card.

void loggerTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(LOGGER_PERIOD_MS);

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    if (!sdReady || imuQueue == nullptr) continue;

    size_t count = 0;

    while (
      count < QUEUE_LENGTH &&
      xQueueReceive(imuQueue, &loggerBatch[count], 0) == pdTRUE
    ) {
      count++;
    }

    if (count == 0) {
      loggerHeartbeat = millis();
      continue;
    }

    if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
      loggerWriteErrors++;
      continue;
    }

    File file = SD.open(FILE_NAME, FILE_APPEND);

    if (!file) {
      Serial.println("[SD ERROR] CSV open failed");
      loggerWriteErrors++;
      sdFault = true;
      systemError = true;
      xSemaphoreGive(sdMutex);
      continue;
    }

    size_t written = 0;

    for (size_t i = 0; i < count; i++) {
      const IMUData &d = loggerBatch[i];

      GPSData g;

      portENTER_CRITICAL(&gpsMux);
      g = gpsData;
      portEXIT_CRITICAL(&gpsMux);

      size_t bytes = file.printf(
        "%lu,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,"
        "%.2f,%.2f,%.2f,%.7f,%.7f,%.2f,%lu,%d,%d\n",

        (unsigned long)d.time_ms,

        d.ax, d.ay, d.az,
        d.gx, d.gy, d.gz,
        d.temperature, d.roll, d.pitch,

        g.latitude, g.longitude, g.altitude,
        (unsigned long)g.satellites,
        g.validFix ? 1 : 0,
        g.usingFallback ? 1 : 0
      );

      if (bytes > 0) written++;
    }

    file.flush();
    bool writeError = file.getWriteError() != 0;
    file.close();

    xSemaphoreGive(sdMutex);

    loggedSamples += written;
    loggerHeartbeat = millis();

    Serial.printf(
      "[LOGGER] Batch=%u Written=%u Total=%lu Dropped=%lu QueueHigh=%lu\n",
      (unsigned int)count,
      (unsigned int)written,
      (unsigned long)loggedSamples,
      (unsigned long)droppedSamples,
      (unsigned long)queueHighWater
    );

    if (writeError || written != count) {
      loggerWriteErrors++;
      sdFault = true;
      systemError = true;
      Serial.println("[FAULT] SD write failed; check card and wiring");
    }
  }
}

// Green LED: task heartbeat.
// Red LED and buzzer: detected system error.

void ledTask(void* parameter) {
  bool phase = false;
  for (;;) {
    phase = !phase;
    if (systemError) {
      // Fault: red LED flashes 1 s ON / 1 s OFF; active buzzer beeps in sync.
      setLed(GREEN_LED_PIN, false);
      setLed(RED_LED_PIN, phase);
      digitalWrite(BUZZER_PIN, phase ? HIGH : LOW);
    } else {
      // Normal: green LED flashes 1 s ON / 1 s OFF.
      setLed(GREEN_LED_PIN, phase);
      setLed(RED_LED_PIN, false);
      digitalWrite(BUZZER_PIN, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

//
// Monitors task heartbeats and reports timing, queue, and
// dropped-sample statistics. GPS is deliberately optional.

void supervisorTask(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(SUPERVISOR_PERIOD_MS);

  uint32_t lastReport = 0;

  for (;;) {
    uint32_t now = millis();

    if (imuReady && imuLastHeartbeat != 0 &&
      now - imuLastHeartbeat > 1000) {
      Serial.println("[SUPERVISOR] IMU heartbeat stale");
      systemError = true;
    }

    if (oledReady && oledHeartbeat != 0 &&
      now - oledHeartbeat > 1500) {
      Serial.println("[SUPERVISOR] OLED heartbeat stale");
      systemError = true;
    }

    if (now - lastReport >= 1000) {
      lastReport = now;

      uint32_t queueUsed = imuQueue
      ? QUEUE_LENGTH - uxQueueSpacesAvailable(imuQueue)
      : 0;

      GPSData g;
      portENTER_CRITICAL(&gpsMux);
      g = gpsData;
      portEXIT_CRITICAL(&gpsMux);

      Serial.printf(
        "[RTES] MaxPeriod=%lu ms MaxExec=%lu us "
        "DeadlineMiss=%lu IMUErrors=%lu I2CBusySkips=%lu "
        "Queue=%lu/%u Dropped=%lu Logged=%lu "
        "GPS=%s IMUfailRun=%lu SDfault=%u Heap=%u\n",

        (unsigned long)maxImuPeriodMs,
        (unsigned long)imuMaxExecUs,
        (unsigned long)imuDeadlineMisses,
        (unsigned long)imuReadErrors,
        (unsigned long)i2cBusySkips,
        (unsigned long)queueUsed,
        (unsigned int)QUEUE_LENGTH,
        (unsigned long)droppedSamples,
        (unsigned long)loggedSamples,
        g.validFix ? "FIX" : "FALLBACK",
        (unsigned long)imuConsecutiveFailures,
        sdFault ? 1U : 0U,
        (unsigned int)ESP.getFreeHeap()
      );
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  setLed(GREEN_LED_PIN, false);
  setLed(RED_LED_PIN, false);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println();
  Serial.println("======================================");
  Serial.println("RT-FSAFE X - RTES IMPLEMENTATION");
  Serial.println("======================================");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  i2cMutex = xSemaphoreCreateMutex();
  sdMutex = xSemaphoreCreateMutex();
  imuQueue = xQueueCreate(QUEUE_LENGTH, sizeof(IMUData));

  if (!i2cMutex || !sdMutex || !imuQueue) {
    Serial.println("[FATAL] RTOS resource allocation failed");

    while (true) {
      setLed(RED_LED_PIN, true);
      delay(250);
      setLed(RED_LED_PIN, false);
      delay(250);
    }
  }

  // Initialize OLED before creating its task.
  oledReady = display.begin(
    SSD1306_SWITCHCAPVCC, OLED_ADDRESS
  );

  if (oledReady) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.println("RT-FSAFE X");
    display.println("RTES initializing");
    display.display();
    Serial.println("[OLED] Ready");
  } else {
    Serial.println("[OLED] Initialization failed");
    systemError = true;
  }

  // Initialize hardware before starting tasks.
  imuReady = initMPU6500();
  sdReady = initSDCard();

  // Missing required hardware at startup must activate the alarm.
  if (!imuReady) {
    Serial.println("[FAULT] MPU6500 unavailable at startup");
    systemError = true;
  }
  if (!sdReady) {
    Serial.println("[FAULT] SD card unavailable at startup");
    sdFault = true;
    systemError = true;
  }

  // GPS UART. No GPS fix is not a system error.
  GPSSerial.begin(
    GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN
  );

  Serial.println("[GPS] UART2 started; fallback enabled");

  // Configure watchdog for ESP-IDF 4.x or 5.x.
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WATCHDOG_TIMEOUT_S * 1000,
    .idle_core_mask = (1 << 0) | (1 << 1),
    .trigger_panic = true
  };

  esp_err_t wdtResult = esp_task_wdt_init(&wdtConfig);

  if (wdtResult == ESP_OK) {
    Serial.println("[WDT] Initialized");
  } else if (wdtResult == ESP_ERR_INVALID_STATE) {
    Serial.println("[WDT] Already initialized");
  } else {
    Serial.printf("[WDT] Init result=%d\n", (int)wdtResult);
  }
#else
  esp_err_t wdtResult =
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);

  if (wdtResult == ESP_OK) {
    Serial.println("[WDT] Initialized");
  } else {
    Serial.printf("[WDT] Init result=%d\n", (int)wdtResult);
  }
#endif

  BaseType_t result;

  // Priorities: IMU=4, supervisor=3,
  // LED/GPS=2, OLED/logger=1.

  if (imuReady) {
    result = xTaskCreatePinnedToCore(
      imuTask, "IMU_Acquisition", 4096,
      nullptr, 4, &imuTaskHandle, 1
    );

    if (result != pdPASS) {
      Serial.println("[FATAL] IMU task creation failed");
      systemError = true;
    }
  }

  result = xTaskCreatePinnedToCore(
    supervisorTask, "RTES_Supervisor", 4096,
    nullptr, 3, &supervisorTaskHandle, 0
  );

  if (result != pdPASS) {
    Serial.println("[FATAL] Supervisor task creation failed");
    systemError = true;
  }

  result = xTaskCreatePinnedToCore(
    ledTask, "LED_Heartbeat", 2048,
    nullptr, 2, &ledTaskHandle, 1
  );

  if (result != pdPASS) {
    Serial.println("[FATAL] LED task creation failed");
    systemError = true;
  }

  result = xTaskCreatePinnedToCore(
    gpsTask, "GPS_Parser", 4096,
    nullptr, 2, &gpsTaskHandle, 0
  );

  if (result != pdPASS) {
    Serial.println("[WARN] GPS task creation failed");
  }

  if (oledReady) {
    result = xTaskCreatePinnedToCore(
      oledTask, "OLED_Update", 4096,
      nullptr, 1, &oledTaskHandle, 0
    );

    if (result != pdPASS) {
      Serial.println("[ERROR] OLED task creation failed");
      systemError = true;
    }
  }

  if (imuReady && sdReady) {
    result = xTaskCreatePinnedToCore(
      loggerTask, "SD_Logger", 6144,
      nullptr, 1, &loggerTaskHandle, 0
    );

    if (result != pdPASS) {
      Serial.println("[ERROR] SD logger task creation failed");
      systemError = true;
    }
  }

  Serial.printf(
    "[SYSTEM] IMU=%s SD=%s OLED=%s FreeHeap=%u\n",
    imuReady ? "OK" : "FAIL",
    sdReady ? "OK" : "FAIL",
    oledReady ? "OK" : "FAIL",
    (unsigned int)ESP.getFreeHeap()
  );

  Serial.println("[SYSTEM] Setup complete");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
