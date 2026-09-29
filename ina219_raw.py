"""Minimal register-level INA219 driver over smbus2 - no internet-dependent
CircuitPython/Blinka packages needed (the campus network blocks outbound
HTTPS to pypi.org, so `pip install adafruit-circuitpython-extended-bus`
hangs). Talks straight to the INA219 registers over a given /dev/i2c-N bus.

Uses the same 32V/2A calibration (0.1 mA/bit current LSB) the
adafruit_ina219 driver used to configure, so current_sensor.py's downstream
math and observed magnitudes don't change - only the transport does.
Property names/units match the adafruit_ina219 API this replaces:
bus_voltage (V), shunt_voltage (V), current (mA), power (W).
"""
import smbus2

_REG_CONFIG = 0x00
_REG_SHUNT = 0x01
_REG_BUS = 0x02
_REG_POWER = 0x03
_REG_CURRENT = 0x04
_REG_CALIBRATION = 0x05

# BRNG=1 (32V), PGA=11 (/8, 320mV shunt range), BADC=0011 (12-bit),
# SADC=0011 (12-bit), MODE=111 (continuous shunt+bus) - the classic
# "32V, 2A" calibration from Adafruit's original INA219 examples.
_CONFIG_32V_2A = 0x399F
_CAL_32V_2A = 4096
_CURRENT_LSB_MA = 0.1     # mA per bit
_POWER_LSB_MW = 2.0       # mW per bit (20 x current LSB, per datasheet)
_SHUNT_LSB_V = 0.00001    # 10 uV per bit
_BUS_LSB_V = 0.004        # 4 mV per bit


class INA219Raw:
    def __init__(self, bus_number: int, address: int = 0x44) -> None:
        self._bus = smbus2.SMBus(bus_number)
        self._address = address
        self._write16(_REG_CALIBRATION, _CAL_32V_2A)
        self._write16(_REG_CONFIG, _CONFIG_32V_2A)

    def _write16(self, reg: int, value: int) -> None:
        # Raw i2c_rdwr, not the smbus write_i2c_block_data() wrapper - the
        # latter silently failed to land writes on this Pi's i2c-gpio
        # bit-banged buses (confirmed by write-then-readback testing).
        msg = smbus2.i2c_msg.write(
            self._address, [reg, (value >> 8) & 0xFF, value & 0xFF])
        self._bus.i2c_rdwr(msg)

    def _read16(self, reg: int) -> int:
        wmsg = smbus2.i2c_msg.write(self._address, [reg])
        rmsg = smbus2.i2c_msg.read(self._address, 2)
        self._bus.i2c_rdwr(wmsg, rmsg)
        data = list(rmsg)
        return (data[0] << 8) | data[1]

    def _read16_signed(self, reg: int) -> int:
        value = self._read16(reg)
        return value - 0x10000 if value > 0x7FFF else value

    @property
    def bus_voltage(self) -> float:
        raw = self._read16(_REG_BUS)
        return ((raw >> 3)) * _BUS_LSB_V

    @property
    def shunt_voltage(self) -> float:
        return self._read16_signed(_REG_SHUNT) * _SHUNT_LSB_V

    @property
    def current(self) -> float:
        return self._read16_signed(_REG_CURRENT) * _CURRENT_LSB_MA

    @property
    def power(self) -> float:
        return self._read16(_REG_POWER) * _POWER_LSB_MW / 1000.0
