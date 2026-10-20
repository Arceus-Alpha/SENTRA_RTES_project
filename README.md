# Sensor-Evaluation-Notification-Telemetry-Fault Detection-FreeRTOS

**SENTRA** is an ESP32-based real-time safety monitoring and telemetry logging system developed to demonstrate Real-Time Embedded Systems (RTES) concepts using FreeRTOS. It continuously acquires inertial sensor data, estimates roll and pitch, displays live attitude information, records telemetry to a microSD card, and monitors system health through periodic tasks and fault alerts.

The project integrates real-time scheduling, inter-task communication, synchronization, deadline monitoring, watchdog supervision, and fault handling into a practical embedded system.

## Features

- **Real-time IMU acquisition:** Reads accelerometer and gyroscope data from the MPU6500 at a nominal 100 Hz sampling rate.
- **Attitude estimation:** Calculates roll and pitch angles using accelerometer measurements and complementary filtering.
- **Live OLED visualization:** Displays an artificial horizon and attitude information on an SSD1306 128 × 64 OLED.
- **Telemetry logging:** Stores timestamped IMU measurements, attitude estimates, and GPS information in CSV format on a microSD card.
- **GPS integration:** Reads GPS data from a NEO-6M module using UART2 and provides clearly flagged fallback coordinates when a valid GPS fix is unavailable.
- **Fault indication:** Uses a green LED for normal operation and a red LED with an active buzzer to indicate detected faults.
- **System supervision:** Monitors task heartbeats, sensor failures, execution time, deadline misses, queue occupancy, and logging errors.
- **Watchdog monitoring:** Integrates the ESP32 task watchdog to help detect task stalls.
- **Performance diagnostics:** Reports real-time execution statistics through the Serial Monitor.

## Hardware Components

- ESP32 DevKit V1 (ESP32-WROOM-32)
- MPU6500 accelerometer and gyroscope
- SSD1306 128 × 64 OLED display
- MicroSD card module
- NEO-6M GPS module
- Green status LED
- Red fault LED
- Active buzzer
- Current-limited LED resistors and suitable power supply

## Software and Technologies

- Arduino framework
- ESP32 FreeRTOS
- C/C++
- I²C communication
- SPI communication
- UART2 serial communication
- MicroSD file handling and CSV logging
- Task watchdog and real-time performance monitoring

## Real-Time System Architecture

The firmware separates system functions into independent FreeRTOS tasks:

| Task | Period / operation | Responsibility |
|---|---|---|
| IMU acquisition | 10 ms | Samples the MPU6500 and publishes telemetry |
| OLED update | 200 ms | Refreshes the attitude display |
| GPS parser | 10 ms polling | Parses incoming GPS data |
| System supervisor | 100 ms | Checks task health and system status |
| LED and alarm | 1-second phase interval | Indicates normal and fault states |
| SD logger | 2-second interval | Collects queued samples and writes CSV records |

The IMU task uses a bounded FreeRTOS queue to transfer samples to the logger task. Mutexes protect shared I²C and SD-card resources, while diagnostic counters track dropped samples, queue high-water marks, execution time, and deadline misses.

## RTES Concepts Demonstrated

- Periodic and priority-based task scheduling
- Producer–consumer communication using FreeRTOS queues
- Mutual exclusion using mutexes
- Inter-task synchronization
- Execution-time measurement and deadline-miss detection
- Watchdog-based task supervision
- Fault detection and alarm generation
- Queue overflow monitoring and data-loss accounting
- Concurrent sensor acquisition, display updates, GPS parsing, and data logging

## Data Logging

The system records the following fields in `imu_data.csv`:

- Timestamp
- Three-axis acceleration
- Three-axis angular velocity
- IMU temperature
- Roll and pitch estimates
- Latitude, longitude, and altitude
- Satellite count
- GPS fix validity
- GPS fallback status

Fallback coordinates are used for testing and are explicitly marked as fallback data; they must not be interpreted as a genuine GPS position.

## Getting Started

1. Install the Arduino IDE and ESP32 board support package.
2. Install the required libraries: Adafruit GFX, Adafruit SSD1306, and TinyGPSPlus.
3. Connect the MPU6500, OLED, microSD module, GPS, LEDs, and buzzer according to the wiring documentation.
4. Configure the pin assignments and module power requirements.
5. Upload the firmware to the ESP32.
6. Open the Serial Monitor at **115200 baud** to observe initialization messages, GPS status, real-time performance metrics, and fault diagnostics.
7. Verify that the OLED updates, telemetry is written to the SD card, and fault alarms activate when a monitored component fails.

## Project Scope
SENTRA is an educational embedded-systems demonstrator for exploring real-time monitoring, telemetry acquisition, fault handling, and FreeRTOS scheduling. Although it uses flight-related terminology and attitude visualization, it is **not a certified flight controller or a safety-critical flight system** and should not be used to control an actual aircraft.

## Future Improvements

- Add a dedicated fault-event queue and fault-management task.
- Implement persistent fault-event logging.
- Add sensor calibration and improved attitude estimation.
- Measure and document worst-case execution times under different workloads.
- Evaluate power consumption and battery runtime under normal and fault conditions.
- Add automated tests for sensor disconnection, SD-card failure, queue saturation, and task delays.

