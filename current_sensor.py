"""INA219 current sensor on the spooler motor supply line (I2C).

The sensor has its OWN sampling thread so the current is captured FAST
(``RAW_SAMPLE_RATE_HZ``, default 200 Hz) — much faster than the 10 Hz control
loops — because the signal is noisy and the downstream analysis (on the
laptop) wants to filter it and detect transients. The raw, unfiltered samples
are what go to the Database/CSV; only the GUI trace is lightly smoothed and
decimated for readability. Do NOT filter before recording: filtering belongs
on the laptop, where it can be redone with different parameters.

Timestamps: main.py hands this module the hardware loop's clock origin
(``init_time``), and every sample is stamped ``time.time() - init_time`` —
the exact same clock and origin as the temperature/RPM buffers, so the
signals align with no post-processing.

INA219 configuration: 32V/2A calibration, 12-bit ADC resolution on both
shunt and bus (0.1 mA/bit current resolution) - see ina219_raw.py.
NOTE: this board reads ~0 V bus voltage because the shunt is wired LOW-SIDE
(in the ground return); the current reading is unaffected. Move the shunt to
the positive supply wire if the real bus voltage is ever needed.

Wiring
------
2026-09-02: the Pi's onboard hardware I2C1 (GPIO2/GPIO3, physical pins 3/5)
was found to be electrically damaged - EVERY address ACKs on that bus even
with nothing connected (SDA stuck low), confirmed both with a Blinka
busio.I2C.scan() and a raw smbus2 quick-write scan. Routed around it with a
kernel software (bit-banged) I2C bus instead:

    dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=17,i2c_gpio_scl=27

added to /boot/firmware/config.txt (reboot required to load), giving
/dev/i2c-3 on GPIO17 (physical pin 11, SDA) / GPIO27 (physical pin 13, SCL).
The INA219's SDA/SCL wires now go there instead of GPIO2/3 - VCC and GND are
unaffected (still 3.3 V physical pin 1 / GND physical pin 9).

I2C address: this (replacement) INA219 module answers at 0x40 (factory
default, no address jumpers bridged) - confirmed with a clean single-address
scan on bus 3. The previous board was hardcoded to 0x45 (both jumpers
bridged); that value is gone along with the GPIO2/3 bus, don't reuse it.

Driver: talks to the INA219 registers directly via smbus2
(``ina219_raw.INA219Raw``) instead of the Blinka/adafruit_ina219 stack -
`adafruit-circuitpython-extended-bus` (needed to point Blinka's I2C at a
non-default bus number) couldn't be installed because outbound HTTPS to
pypi.org is blocked on this network. ina219_raw exposes the same
bus_voltage (V) / shunt_voltage (V) / current (mA) / power (W) properties
adafruit_ina219 did, so nothing below this docstring needed to change beyond
the wiring/import swap.

The sensor is OPTIONAL: if it is absent, miswired or the library is missing,
the program runs normally and the current buffers simply stay empty.
"""
import threading
import time
from collections import deque

from database import Database


class CurrentSensor:
    """Sample the spooler supply current (mA) on a dedicated 200 Hz thread."""

    RAW_SAMPLE_RATE_HZ = 200.0   # raw rate recorded to the Database/CSV
    # Push one smoothed point (mean of the last N raw samples) to the GUI
    # every N raw samples -> a clean 20 Hz trace; the CSV keeps the raw data.
    PLOT_EVERY_N = 10
    I2C_BUS_NUMBER = 4   # i2c-gpio bit-banged bus on GPIO24(SDA)/GPIO23(SCL)
    I2C_ADDRESS = 0x45   # this INA219's factory-default address
    MAX_READ_ERRORS = 50   # consecutive failures before giving up on the sensor
    # Raw samples are also streamed to the laptop in batches of this many
    # (20 @ 200 Hz = one telemetry message per 100 ms), each sample keeping
    # its own Pi-clock timestamp so nothing is lost to batching.
    TELEMETRY_BATCH = 20

    def __init__(self, gui=None) -> None:
        self.gui = gui   # for the Spooler Current plot (optional)
        self.available = False
        self.latest_current = 0.0      # mA (most recent raw reading)
        self.latest_bus_voltage = 0.0  # V (~0 with the low-side wiring)
        self._fail_count = 0
        self._ina = None
        self._init_time = None
        self._thread = None
        self._stop = threading.Event()
        try:
            from ina219_raw import INA219Raw

            self._ina = INA219Raw(CurrentSensor.I2C_BUS_NUMBER,
                                   address=CurrentSensor.I2C_ADDRESS)
            self.available = True
            print(f"[CurrentSensor] INA219 ready at 0x{self.I2C_ADDRESS:02X} "
                  f"on bus {self.I2C_BUS_NUMBER}: {self._ina.current:.1f} mA "
                  f"(raw sampling at {self.RAW_SAMPLE_RATE_HZ:.0f} Hz)")
        except Exception as exc:
            print(f"[CurrentSensor] INA219 not available ({exc}). "
                  "Current logging disabled; everything else runs normally.")

    # ------------------------------------------------------------------ #
    # Lifecycle (called from main.py)
    # ------------------------------------------------------------------ #
    def start(self, init_time: float) -> None:
        """Start the sampling thread.

        ``init_time`` is the hardware loop's ``time.time()`` origin, so every
        sample timestamp lands on the same clock as the other sensors.
        """
        if not self.available or self._thread is not None:
            return
        self._init_time = init_time
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()

    # ------------------------------------------------------------------ #
    # Sampling thread
    # ------------------------------------------------------------------ #
    def _run(self) -> None:
        period = 1.0 / CurrentSensor.RAW_SAMPLE_RATE_HZ
        window = deque(maxlen=CurrentSensor.PLOT_EVERY_N)
        since_plot = 0
        batch_t, batch_i = [], []
        next_tick = time.time()
        # TEMP DEBUG (remove once the current-reading question is fully put
        # to bed): prints shunt voltage alongside current once a second.
        debug_tick = 0

        while not self._stop.is_set() and self.available:
            t = time.time() - self._init_time
            try:
                current_ma = float(self._ina.current)
                bus_voltage = float(self._ina.bus_voltage)
                shunt_mv = float(self._ina.shunt_voltage) * 1000.0
                debug_tick += 1
                if debug_tick >= CurrentSensor.RAW_SAMPLE_RATE_HZ:
                    debug_tick = 0
                    print(f"[CurrentSensor DEBUG] shunt={shunt_mv:+.3f} mV  "
                          f"bus={bus_voltage:.3f} V  current={current_ma:+.2f} mA")
            except Exception as exc:
                self._fail_count += 1
                if self._fail_count == 1:
                    print(f"[CurrentSensor] read error: {exc}")
                if self._fail_count >= CurrentSensor.MAX_READ_ERRORS:
                    self.available = False
                    print("[CurrentSensor] too many consecutive read errors; "
                          "sensor disabled for this session.")
                    return
                time.sleep(period)
                continue
            self._fail_count = 0

            self.latest_current = current_ma
            self.latest_bus_voltage = bus_voltage
            Database.ina_timestamps.append(t)
            Database.spooler_current.append(current_ma)
            Database.spooler_bus_voltage.append(bus_voltage)

            # Telemetry to the laptop: raw samples, batched. Best-effort - if
            # no laptop is connected the batch is simply dropped (the Pi's own
            # Database/CSV above keeps everything regardless).
            batch_t.append(round(t, 4))
            batch_i.append(round(current_ma, 2))
            if len(batch_t) >= CurrentSensor.TELEMETRY_BATCH:
                self._send_telemetry(batch_t, batch_i)
                batch_t, batch_i = [], []

            # Smoothed + decimated GUI trace (append-only; the GUI thread
            # redraws on its own QTimer).
            window.append(current_ma)
            since_plot += 1
            if self.gui is not None and since_plot >= CurrentSensor.PLOT_EVERY_N:
                since_plot = 0
                try:
                    self.gui.current_plot.update_plot(
                        t, sum(window) / len(window))
                except Exception as exc:
                    print(f"[CurrentSensor] plot error: {exc}")

            # Precise pacing: schedule against the ideal tick, not the loop
            # body's duration, so the rate stays at RAW_SAMPLE_RATE_HZ.
            next_tick += period
            delay = next_tick - time.time()
            if delay > 0:
                time.sleep(delay)
            elif delay < -1.0:
                next_tick = time.time()   # fell far behind; resync

    def _send_telemetry(self, timestamps, currents) -> None:
        """Stream one batch of raw samples to the laptop (best-effort)."""
        source = getattr(self.gui, "diameter_source", None) if self.gui else None
        if source is None or not source.connected:
            return
        source.send_message({
            "type": "telemetry",
            "sensor": "spooler_current",
            "u": "mA",
            "t": timestamps,   # Pi-clock seconds, one per sample
            "i": currents,
        })
