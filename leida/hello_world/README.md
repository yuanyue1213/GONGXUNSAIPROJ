# N10 ESP32-S3 UART2 / Wi-Fi bridge

Main firmware: `main/n10_bridge_main.c`. The original `hello_world_main.c` template is not compiled.

See [parent README](../README.md) for wiring, flashing, Wi-Fi credentials and Python viewer commands.

Target: ESP32-S3; RX GPIO16, TX GPIO15; UART2 230400 8N1.

Build: `idf.py build`; flash: `idf.py -p COMx flash monitor`.

