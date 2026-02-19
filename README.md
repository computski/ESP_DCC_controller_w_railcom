# ESP_DCC_controller_w_railcom
PRODUCTION. ESP 8266 DCC controller with railcom support. Uses most of the codebase from the ESP_DCC_controller project, but this project additionally supports a railcom cutout on the controller i.e. takes both X and Y to ground (or both 12v) during a railcom cutout slot that appears after every loco packet sent to line.   A railcom decoder will assert its data during the railcom cutout, and this is detected by dedicated hardware based around an LM319.  This data is feed to the ESP hardware UART and is gated-in during the cutout period via a railcom sync signal to a 6n317 opto isolator.  The UART operates at 250kbs.  I have designed a custom PCB for this project.

Now moved to ArduinoJson v7.0.4

https://arduinojson.org/v6/doc/upgrade/

https://arduinojson.org/news/2024/01/03/arduinojson-7/

Note: RX pin input is idle high.   The same is true of the comparator, i.e. active is a low signal

The railcom detector is integrated into the controller.  It uses an adaptation of the reference circuit in NRMA doc s-9.3.2_2012.   This is a floating earth detector based on the LM319.  The LM319 can operate from 
a split power rail, which means it does not use the negative rail to drive the opto LED current, instead driving the LED from the +ve rail and into the floating earth.  This means there's less chance of the negative
voltage reference being pulled by the operation of the comparator itself.   The LM319 is also faster than the LM393.  A high speed (10Mbd) 6N137 opto isolator is used to level shift between the floating earth and
the RX input on the ESP mini.  The 6N137 has an open collector output buffer and this is ideal as we can wire-OR this into the RX pin along with the on-board ESP mini USB-serial circuitry; this has an on-board 400R pull up.

The 6N137 also has a gating input on the output stage, and we drive this from railcom sync which is active high during the cutout.  this greatly cuts down on junk data into the UART and reduces its workload.

2025-05-13 fixed a bug in global.h which incorrectly defined the drive pins for the 2-pin drive arrangement on the LMD18200.  Railcom detector now works properly and with loco facing in either direction.

2025-11-05 adding support for JMRI and LocoNet over TCP.  Work in progress.

2026-02-19 LocoNet working, added support for OPC_IMM dcc immediate messages which allows use of multi aspect signals
