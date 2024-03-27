# AnalysIR-ESP8266-Serial-PlatformIO
 This repo has the source code for the ESP8266 firmware which is provided with the [AnalysIR software](https://www.analysir.com/blog/get-analysir/), minimally modified to work with PlatformIO and the latest ESP8266 Arduino core

To be specific, on line #29, `#include <Arduino.h>` was added, as PlatformIO requires that.

In addition, on lines #39-40, the code was modified to include `ICACHE_RAM_ATTR` because without that, the program crashes repeatedly upon first starting up. [More info here](https://forum.arduino.cc/t/using-interrupts-with-a-node-mcu/591995/5)

Finally, the `setup()` and `loop()` functions were moved to the end of the program, to compile properly using PlatformIO. Everything else remains the same.

If you'd like to change the pin definitions, see lines #43-44. 

You may still receive an error message `Error on Serial Port: Frame` in AnalysIR (I do) - this can be ignored.

This repo is mainly because I keep modifying this program, forgetting how I did it, and re-doing all of these steps. But I thought I'd make it public in case it helps somebody else. 