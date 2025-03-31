# ESP-Vario


## Configuration

Select ESP32-C3 board with command `Select Espressif Device Target`

Set OTA server URL, WiFi credentials with `idf.py menuconfig -> App Configuration`

Initialize submodules with `git submodule init` and `git submdoule update`

Build and flash app with `idf.py -p PORT -b 115200 flash monitor`

## OTA update

Run an HTTP(S) server in the `./build` directory before triggering OTA update.
For instance: `python -m http.server 8070 --directory ./build`