# ESP_DCC_controller_w_railcom
ESP 8266 DCC controller with railcom support.  IN DEVELOPMENT.  Making this repository public as of 2024-08-27.  It uses most of the codebase from the ESP_DCC_controller project, but this project additionally supports a railcom cutout on the controller i.e. takes both X and Y to ground during a railcom cutout slot that appears after every loco packet sent to line.   I am currently working on the railcom detector hardware that will sit in the controller and leverage the UART running at 250kb/s.  I have designed a custom PCB for this project.

Now moved to ArduinoJson v7.0.4

https://arduinojson.org/v6/doc/upgrade/

https://arduinojson.org/news/2024/01/03/arduinojson-7/

Note: RX pin input is idle high.   The same is true of the comparator, i.e. active is a low signal

The railcom detector is integrated into the controller.  It uses an adaptation of the reference circuit in NRMA doc s-9.3.2_2012.   This is a floating earth detector based on the LM319.  The LM319 can operate from 
a split power rail, which means it does not use the negative rail to drive the opto LED current, instead driving the LED from the +ve rail and into the floating earth.  This means there's less chance of the negative
voltage reference being pulled by the operation of the comparator itself.   The LM317 is also faster than the LM393.  A high speed (10Mbd) 6N137 opto isolator is used to level shift between the floating earth and
the RX input on the ESP mini.  The 6N137 has an open collector output buffer and this is ideal as we can wire-OR this into the RX pin along with the on-board ESP mini USB-serial circuitry; this has an on-board 400R pull up.

The LM393 reference design is quite clever as firstly it uses a floating earth, which means the comparators are only measuring the true differential railcom voltage across the detetor resistor, and secondly it uses high value pull-ups into 
a NAND gate.  This is because the LM393 will pull low to its Vdd rail, which is the negative rail.  If you were to put a heavy current load on this such as an opto LED, it would load the negative rail too much and pull the -ve voltage ref
out of limits.  This why its outputs are not wire-ORd.   But, with this design you still need a level shifter from the detector floating earth back to whatever MCU you are using (unless it was powered by the railcom circuit).   So its possible to
improve on the circuit by firstly using an LM317, then wire-OR the outputs to drive a good 10mA active low LED in an opto isolator, which itself is inverting and drives an open collector hard low for the RX signal.
