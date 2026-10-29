# ESP32-S3 start bridge

ESP-IDF 6.0. Only START commands are forwarded to STM32; AUTO/TASK status is returned to the phone.
No autonomous scheduler or manual debug interface runs on ESP32.

Build: idf.py build

Output: build/robot_start_bridge.bin. Flash using build/flash_args.

See ../../README.md for wiring, configuration and all three build instructions.
