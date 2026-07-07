### DHT22 measurement displayed on TM1637

In this example we periodically read sensor data (temperature and rel. humidity)
from DHT22 external device.
The measurement data is displayed on TM1637 4x7 segment display.

#### Hardware components

* S1: DHT22 temp/hum sensor
* DISP: TM1637
* B0: Button0
* L0: LED + resistor

#### Connections

```
ESP32.GPIO17 -- L0.+
ESP32.GND    -- L0.-
ESP32.GPIO18 -- B0.+
ESP32.GND    -- B0.-
ESP32.GPIO25 -- DISP.CLK
ESP32.GPIO26 -- DISP.DIO
ESP32.GPIO27 -- S1.OUT
ESP32.GND    -- S1.-
ESP32.VCC    -- S1.+
ESP32.GND    -- DISP.GND
ESP32.VCC    -- DISP.VCC
```
