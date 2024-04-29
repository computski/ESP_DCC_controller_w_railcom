# ESP_DCC_controller_w_railcom
ESP 8266 DCC controller with railcom support

Now moved to ArduinoJson v7.0.4

BUGS: reworked railcom cutout to have both legs of LMD pull to ground. comparator goes high during the cutout but also pulses high immediately afterwards depsite
there being a load on the track and the dcc waveform being bipolar at this point

Also loco headlamp twinkles slightly, this might be the cutout, but these are only 500uS wide.  Maybe a post packet power enablement jitter.  Also we should supress railcom during Service Mode to help make
CV reading more reliable.  SM relies on the loco drawing current, this is not available during cutouts.  Maybe add a flag to the packet structure?

https://arduinojson.org/v6/doc/upgrade/

https://arduinojson.org/news/2024/01/03/arduinojson-7/
