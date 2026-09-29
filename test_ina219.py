"""Standalone INA219 wiring check — run on the Pi, no GUI or motor needed.

    source fred-venv/bin/activate
    python test_ina219.py

Prints bus voltage / current / power twice per second until Ctrl+C. Use it
right after wiring the sensor, BEFORE trusting the data in main.py.

2026-09-02: the Pi's hardware I2C1 (GPIO2/GPIO3) is electrically damaged
(every address ACKs, even nothing connected). Routed around it with a
software I2C bus instead - see current_sensor.py's docstring for the full
story. This now talks to bus 3 (GPIO17=SDA physical pin 11, GPIO27=SCL
physical pin 13) via ina219_raw.py (smbus2 register-level driver - the
Blinka/adafruit_ina219 stack needed a pypi.org install that's blocked on
this network).

If it fails, check in this order:
  1. Bus 3 exists?          ls /dev/i2c-3  (needs the i2c-gpio overlay +
                             a reboot - see /boot/firmware/config.txt)
  2. Sensor answering?      the smbus2 quick-write scan in
                             current_sensor.py's docstring, must show
                             exactly ONE address, not zero and not all 120.
  3. SDA on GPIO17 (physical pin 11), SCL on GPIO27 (physical pin 13),
     VCC on 3.3 V, GND common with the Pi.

Sanity checks on the readings:
  - Bus voltage should sit near the spooler driver's supply (≈12 V if there
    is a 5->12 V boost, ≈5 V if the driver runs straight off the Pi rail) -
    except this board's shunt is wired low-side, so expect it to sit near
    0 V regardless; only `current` is meaningful here.
  - Current ~0 mA with the motor stopped; it should jump when the spooler
    spins (tens to a few hundred mA expected).
  - If current reads NEGATIVE while the motor runs, Vin+ and Vin- are
    swapped: fine for a quick test, but swap them for correct signs.
"""
import time

from ina219_raw import INA219Raw

I2C_BUS_NUMBER = 4   # must match current_sensor.CurrentSensor.I2C_BUS_NUMBER
I2C_ADDRESS = 0x45   # must match current_sensor.CurrentSensor.I2C_ADDRESS


def main() -> None:
    ina = INA219Raw(I2C_BUS_NUMBER, address=I2C_ADDRESS)

    print(f"INA219 set at 0x{I2C_ADDRESS:02X} on bus {I2C_BUS_NUMBER}. "
          "Ctrl+C to stop.\n")
    print(f"{'t (s)':>8}  {'bus (V)':>8}  {'current (mA)':>13}  {'power (mW)':>11}")
    t0 = time.time()
    while True:
        print(f"{time.time() - t0:8.1f}  {ina.bus_voltage:8.3f}  "
              f"{ina.current:13.2f}  {ina.power * 1000:11.1f}")
        time.sleep(0.5)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
