"""High-rate measurement sampler (paso 0 of the PC-brain migration).

Samples the MEASUREMENTS at a standard fast rate (``STANDARD_RATE_HZ``,
100 Hz) while the control loops keep their tuned 10 Hz cadence untouched —
sampling rate and control rate are deliberately decoupled. The spooler
current has its own 200 Hz thread (see ``current_sensor.py``); the camera
diameter runs at the camera's own fps on the laptop. Everything shares the
hardware loop's clock, so the signals align by timestamp.

Signals per sample:
    temperature   raw thermistor conversion (Steinhart-Hart), NO moving
                  average and NO write to the control's averaging buffer -
                  filtering belongs to the analysis side.
    spooler RPM   encoder count delta over a ~0.1 s sliding window: the
                  same quantization noise as the old 10 Hz measurement,
                  refreshed at 100 Hz.
    stepper RPM   the commanded setpoint (the stepper is open loop, so the
                  command IS the best available measurement).
    fan duty      the effectively applied duty (0 when the fan is stopped).

Runs INSIDE the hardware-control thread (called from main.py's loop, which
polls at ~500 Hz), so the MCP3008/encoder SPI reads never race the control
loops' own reads. Reads are non-destructive: the encoder count is an
absolute register and the control code keeps its own previous-position
state, so the 10 Hz loops see exactly what they always saw.
"""
import math
import threading
import time
from collections import deque

from database import Database
from extruder import Thermistor
from spooler import Spooler


class FastSampler:
    """Read temperature / RPM / setpoints at 100 Hz into the Database."""

    STANDARD_RATE_HZ = 100.0
    PERIOD = 1.0 / STANDARD_RATE_HZ
    RPM_WINDOW_S = 0.1   # sliding encoder window: 10 Hz-equivalent noise
    # One batch to the laptop per 50 ms: the laptop-side PID loops (10 Hz)
    # control on this data, so staleness must stay well under their period.
    TELEMETRY_PERIOD_S = 0.05
    # In REMOTE mode the Pi's control loops are off, so nothing feeds the
    # temperature/RPM plots - this sampler does it instead, decimated.
    PLOT_EVERY_N = 10          # one plot point per 10 samples (10 Hz)

    def __init__(self, gui, extruder, spooler) -> None:
        self.gui = gui
        self.extruder = extruder
        self.spooler = spooler
        self._next_tick = None   # ideal time of the next sample
        self._since_plot = 0
        window_samples = int(self.RPM_WINDOW_S * self.STANDARD_RATE_HZ) + 1
        self._enc_window = deque(maxlen=window_samples)   # (t, count) pairs

    @staticmethod
    def _voltage_to_celsius(voltage: float) -> float:
        """Steinhart-Hart with the same constants the control loop uses,
        but WITHOUT touching Thermistor's moving-average buffer."""
        if voltage < 0.0001 or voltage >= Thermistor.VOLTAGE_SUPPLY:
            return 0.0
        resistance = ((Thermistor.VOLTAGE_SUPPLY - voltage)
                      * Thermistor.RESISTOR) / voltage
        ln = math.log(resistance / Thermistor.RESISTANCE_AT_REFERENCE)
        return (1.0 / ((ln / Thermistor.BETA_COEFFICIENT)
                       + (1.0 / Thermistor.REFERENCE_TEMPERATURE))) - 273.15

    def sample(self, current_time: float) -> None:
        """Take one sample if the 100 Hz period elapsed. Called every
        hardware-loop iteration, in every mode (manual/monitor/experiment)."""
        # Schedule against IDEAL tick times, not against the previous sample:
        # gating on the previous sample adds the hardware loop's latency
        # (~2-4 ms) to every period and erodes the real rate to ~70 Hz. With
        # ideal ticks, a late sample shortens the next wait, so the average
        # rate stays at STANDARD_RATE_HZ.
        if self._next_tick is None:
            self._next_tick = current_time
        if current_time < self._next_tick:
            return
        self._next_tick += FastSampler.PERIOD
        if current_time - self._next_tick > 0.5:
            # Fell far behind (stall): resync instead of bursting a backlog.
            self._next_tick = current_time + FastSampler.PERIOD
        try:
            temperature = self._voltage_to_celsius(
                self.extruder.channel_0.voltage)

            count = self.spooler.read_encoder()
            self._enc_window.append((current_time, count))
            rpm = 0.0
            if len(self._enc_window) >= 2:
                t_old, count_old = self._enc_window[0]
                dt = current_time - t_old
                if dt > 0:
                    # NEGATED, matching spooler.py's control loop: the encoder
                    # counts DOWN when the motor spins forward. Same sanity
                    # filter as the original (kills 32-bit wrap glitches).
                    rpm = -((count - count_old)
                            / Spooler.PULSES_PER_REVOLUTION) * (60.0 / dt)
                    if abs(rpm) > 65:
                        rpm = 0.0

            stepper_rpm = float(self.gui.get_extrusion_speed())
            fan_duty = (float(self.gui.get_fan_duty())
                        if getattr(self.gui, "fan_enabled", True) else 0.0)

            Database.fast_timestamps.append(current_time)
            Database.fast_temperature.append(temperature)
            Database.fast_spooler_rpm.append(rpm)
            Database.fast_stepper_rpm.append(stepper_rpm)
            Database.fast_fan_duty.append(fan_duty)

            # REMOTE mode: the control loops (the plots' usual feeders) are
            # off, so drive the temperature/RPM plots from here at 10 Hz.
            # Setpoint 0, like monitor mode: raw commands have no reference.
            self._since_plot += 1
            if self._since_plot >= FastSampler.PLOT_EVERY_N:
                self._since_plot = 0
                remote = getattr(self.gui, "remote", None)
                if remote is not None and remote.is_active():
                    self.gui.temperature_plot.update_plot(
                        current_time, temperature, 0)
                    self.gui.motor_plot.update_plot(current_time, rpm, 0)
        except Exception as exc:
            print(f"[FastSampler] error: {exc}")

    # ------------------------------------------------------------------ #
    # Telemetry to the laptop (step 3a)
    # ------------------------------------------------------------------ #
    def start_telemetry(self) -> None:
        """Start streaming the fast buffers to the laptop.

        Runs on its OWN daemon thread so a slow/stalled network can never
        hold up the hardware-control thread. The thread just watches the
        append-only Database buffers (index-based, no locks needed) and
        ships whatever is new every TELEMETRY_PERIOD_S.
        """
        thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        thread.start()

    def _telemetry_loop(self) -> None:
        sent = 0
        while True:
            time.sleep(FastSampler.TELEMETRY_PERIOD_S)
            source = getattr(self.gui, "diameter_source", None)
            if source is None or not source.connected:
                # No laptop: skip ahead so we never dump a backlog on connect
                # (the Pi CSV keeps the full history regardless).
                sent = len(Database.fast_timestamps)
                continue
            n = min(len(Database.fast_timestamps),
                    len(Database.fast_temperature),
                    len(Database.fast_spooler_rpm),
                    len(Database.fast_stepper_rpm),
                    len(Database.fast_fan_duty))
            if n <= sent:
                continue
            try:
                temp_sp = float(self.gui.get_target_temperature())
                rpm_sp = float(self.gui.get_motor_setpoint())
            except Exception:
                temp_sp, rpm_sp = 0.0, 0.0
            source.send_message({
                "type": "telemetry",
                "sensor": "fast",
                "t": [round(v, 4) for v in Database.fast_timestamps[sent:n]],
                "temp": [round(v, 3) for v in Database.fast_temperature[sent:n]],
                "rpm": [round(v, 3) for v in Database.fast_spooler_rpm[sent:n]],
                "step": [round(v, 3) for v in Database.fast_stepper_rpm[sent:n]],
                "fan": [round(v, 1) for v in Database.fast_fan_duty[sent:n]],
                # Active setpoints (per batch - they change on human timescales)
                "temp_sp": round(temp_sp, 2),
                "rpm_sp": round(rpm_sp, 2),
            })
            sent = n
