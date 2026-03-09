# ESP32-S3 Firmware

This folder is for the ESP-IDF project that will run on the ESP32-S3.

Suggested contents after project creation:

- `main/` for application code
- `CMakeLists.txt`
- `sdkconfig`
- `components/` only if you later need reusable embedded modules

Initial goal:

1. Create a fresh ESP-IDF example project here.
2. Set target to `esp32s3`.
3. Replace the example logic with a minimal PWM test for the motors.
