# Open Vario Lite


## Configuration

Select ESP32-C3 board with command `Select Espressif Device Target`

Set OTA server URL, WiFi credentials with `idf.py menuconfig -> App Configuration`.

Build and flash app with `idf.py -p PORT -b 115200 flash monitor`

## OTA update

Run an HTTP(S) server in the `./build` directory before triggering OTA update.