# ESP_DCC_controller_w_railcom
ESP 8266 DCC controller with railcom support.  IN DEVELOPMENT.  Making this repository public as of 2024-08-27.  It uses most of the codebase from the ESP_DCC_controller project, but this project additionally supports a railcom cutout on the controller i.e. takes both X and Y to ground during a railcom cutout slot that appears after every loco packet sent to line.   I am currently working on the railcom detector hardware that will sit in the controller and leverage the UART running at 250kb/s.  I have designed a custom PCB for this project.

Now moved to ArduinoJson v7.0.4

https://arduinojson.org/v6/doc/upgrade/

https://arduinojson.org/news/2024/01/03/arduinojson-7/

Note: RX pin input is idle high.   The same is true of the comparator, i.e. active is a low signal
