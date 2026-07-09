### DHT22 measurement data displayed on TM1637

In this example we periodically read sensor data (temperature and rel. humidity)
from DHT22 external device.
The measurement data is displayed on TM1637 4x7 segment display.

#### Hardware components

* `S0`: *Button0*
* `S_UP`: *Button_Up*
* `S_DOWN`: *Button_Down*
* `D0`: *LED*
* `R0`: *470Ω resistor*
* `DHT22`: *temp/rhum sensor*
* `TM1637`: *4x7 segment display*

Also, there are some components provided by NodeMCU-ESP32S:

* *D1* LED with resistor at `GPIO2`,
* *S1* Button (with "BOOT" label) at `GPIO0`.

#### Connections

```
ESP32.GPIO18 -- S0.leg0
ESP32.GND    -- S0.leg1
ESP32.GPIO32 -- S_UP.leg0
ESP32.GND    -- S_UP.leg1
ESP32.GPIO33 -- S_DOWN.leg0
ESP32.GND    -- S_DOWN.leg1
ESP32.GPIO17 -- R0.leg0
R0.leg1      -- D0.+
ESP32.GND    -- D0.-
ESP32.GPIO25 -- TM1637.CLK
ESP32.GPIO26 -- TM1637.DIO
ESP32.GND    -- TM1637.GND
ESP32.VCC    -- TM1637.VCC
ESP32.GPIO27 -- DHT22.OUT
ESP32.GND    -- DHT22.-
ESP32.VCC    -- DHT22.+
```

#### Control

The application can be controlled with the 4 buttons (`Button0`, `Button1`, `Button_Up` and `Button_Down`).
Currently, the following functions are assigned to the buttons:

* `Button0`
  * **short press** Display temp/rhum min/max measured values
  * **long press** Enter configuration (not implemented yet)
* `Button1`
  * **short press** Cycle through display brightness values
  * **long press** print measurement statistics (uart console)
* `Button_Up` Increase "alive" blinking period (up to 10 sec)
* `Button_Down` Decrease "alive" blinking period (down to 0.5 sec)
