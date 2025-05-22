Lager/IPA Fermentation Monitor
==============================

A custom ESP32-based system to monitor and control beer fermentation in a modified refrigerator, designed for a Lager/IPA hybrid. Built with bare-metal C using ESP-IDF, it tracks beer temperature, ambient conditions, and fridge door state, while controlling cooling via a relay. Data is sent to Home Assistant via MQTT for centralized monitoring and automation.

Features
--------

-   **Sensors**:
    -   DS18B20: Beer temperature (waterproof, 1-Wire).
    -   BME280: Ambient temperature, humidity, pressure (I2C).
    -   Reed switch: Fridge door state (interrupt-driven).
-   **Control**: Relay for fridge compressor (hysteresis-based, PID option).
-   **Connectivity**: Wi-Fi and MQTT with robust reconnection handling.
-   **Target Temp Updates**: Set via MQTT (beer/fridge/set_temp) using a FreeRTOS queue.
-   **Error Handling**: Fallbacks for sensor failures, connection status checks.
-   **Status**: LED indicator for system activity.

Hardware
--------

-   **Microcontroller**: ESP32-WROOM-32.
-   **Pinout**:
    -   GPIO4: DS18B20 (1-Wire).
    -   GPIO21 (SDA), GPIO22 (SCL): BME280 (I2C).
    -   GPIO27: Reed switch (pull-up, interrupt).
    -   GPIO26: Relay (active high).
    -   GPIO2: LED (active high).
-   **Power**: 3.3V via regulator (e.g., AMS1117-3.3) from 5V USB or battery.
-   **Fridge**: Modified with relay overriding thermostat.

Software
--------

-   **Framework**: ESP-IDF v5.x.
-   **Language**: C, compiled for performance and control.
-   **Libraries**:
    -   onewire, ds18b20: DS18B20 temperature (custom or esp-idf-lib).
    -   bme280: Ambient sensor (from esp-idf-lib).
    -   ESP-IDF: Wi-Fi, MQTT, FreeRTOS, GPIO, I2C.
-   **Temperature Control**:
    -   **Hysteresis**: ±1°C band (default).
    -   **PID**: Optional (Kp=5.0, Ki=0.1, Kd=1.0, tuneable; commented out).

Setup Instructions
------------------

### Prerequisites

-   ESP-IDF v5.x installed (see [Espressif Docs](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)).
-   Hardware assembled on a custom PCB or breadboard.
-   Home Assistant with MQTT broker (e.g., Mosquitto) running.

### Build and Flash

1.  Clone or create project:

    ```bash
    idf.py create-project beer_monitor cd beer_monitor/main
    ```

2.  Copy main.c and config.h into main/.
3.  Update config.h with your Wi-Fi SSID, password, and MQTT broker address.
4.  Update CMakeLists.txt:

    ```makefile
    cmake_minimum_required(VERSION 3.5)
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(beer_monitor)
    set(SOURCES main.c)
    idf_build_set_property(COMPILE_OPTIONS "-I${CMAKE_CURRENT_SOURCE_DIR}" APPEND)
    ```

5.  Add dependencies (e.g., esp-idf-lib components) to components/ if needed.
6.  Build and flash:

    ```bash
    idf.py set-target esp32 
    idf.py build 
    idf.py -p /dev/ttyUSB0 flash monitor
    ```

### Home Assistant Integration

Add to configuration.yaml:

```yaml
mqtt:
  sensor:
    - name: "Beer Temp"
      state_topic: "beer/fridge/beer_temp"
      unit_of_measurement: "°C"
    - name: "Ambient Temp"
      state_topic: "beer/fridge/ambient_temp"
      unit_of_measurement: "°C"
    - name: "Humidity"
      state_topic: "beer/fridge/humidity"
      unit_of_measurement: "%"
  binary_sensor:
    - name: "Fridge Door"
      state_topic: "beer/fridge/door"
      payload_on: "open"
      payload_off: "closed"
```

Usage
-----

-   **Monitor**: View logs (idf.py monitor) or Home Assistant dashboard for real-time data.
-   **Set Target Temp**: Send via MQTT:

    ```bash
    mosquitto_pub -h homeassistant.local -t beer/fridge/set_temp -m "14.5"
    ```

    -   Range: 5-25°C.
-   **Temperature Control**:
    -   Hysteresis (default): Relay toggles at ±1°C from target.
    -   PID (optional): Uncomment in sensor_task, tune Kp/Ki/Kd via logs.

Testing and Tuning
------------------

-   **Target Temp**: Send values (e.g., 13.0, 14.5) and verify updates in logs.
-   **Hysteresis**: Check relay at target ± 1°C. Adjust hysteresis if needed.
-   **PID (if enabled)**:
    -   Log temp for 30 min (beer_temp every 5s).
    -   Tune: Reduce Kp if overshoot, increase Kd if oscillation, adjust Ki for steady-state error.
-   **Errors**: Disconnect sensors to test fallbacks (last valid temp used).

Future Enhancements
-------------------

-   Add Tilt Hydrometer (BLE) for specific gravity.
-   Refine PID control for tighter regulation.
-   Design and fabricate custom PCB.
-   Expand error handling (e.g., MQTT timeout, sensor retry logic).

Notes
-----

-   Initial target temp: 13°C (Lager/IPA hybrid).
-   Project status as of March 31, 2025: Functional core with room for expansion.