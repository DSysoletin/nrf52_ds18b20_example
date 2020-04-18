# nrf52_ds18b20_example
Example of reading temperature from DS18B20 on NRF52840

This example should be used with Nordic SDK, it is made on top of UART example (examples/peripheral/uart/)
SDK config with this example should be placed at examples/peripheral/uart/pca10056/blank/config/sdk_config.h
To test it, place main.c and sdk config to UART example folder examples/peripheral/uart/, 
open Segger embedded studio, open UART example for pca10056 devboard (you can test this on separate chip 
also, not only on pca10056), compile code and upload it to your NRF52840.

From hardware point of view, you should solder 15K resistor between DS18B20 data pin (can be configured in main.c),
and +3.3V rail.

If your setup is correct, you'll see in UART this string (prints about every second):
read temp: 25.437500