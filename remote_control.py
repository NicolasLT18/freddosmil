"""Remote command mode (step 3b): the laptop drives the actuators RAW.

While active, the Pi applies raw actuator commands received over the TCP
link and its own control loops stay out of the way (same hardware-ownership
rule as Experiment). The STOP buttons on the Pi GUI keep working and abort
remote mode for safety. The measurement side (fast_sampler, current_sensor,
telemetry) is mode-independent and keeps running, so the laptop still sees
everything while it drives.

SAFETY WATCHDOG: while remote mode is active the laptop must keep talking -
any {"type":"cmd"} or {"type":"hb_cmd"} message counts. If nothing arrives
for WATCHDOG_TIMEOUT_S, the Pi zeroes heater/motors/fan and leaves remote
mode on its own. This is the safety belt of the PC-brain architecture: a
dead link can never leave the heater powered with no brain attached.

Wire protocol (laptop -> Pi), newline-delimited JSON like everything else:

    {"type": "cmd_mode", "enabled": true}          enter remote mode (outputs
                                                   start zeroed) / false = exit
    {"type": "cmd", "heater_pwm": 30}              any subset of: heater_pwm,
                                                   spooler_duty, stepper_rpm,
                                                   fan_duty (values clamped)
    {"type": "hb_cmd"}                             heartbeat (>= 2 per second)

Pi -> laptop status: {"type": "remote_status", "active": bool, "message": str}

Thread model: the network thread only writes plain attributes; the hardware
loop calls update() every iteration and applies them. Actuators are touched
ONLY when a value changes - this also avoids the software-PWM restart chop
the old stepper control loop suffers from.
"""
import time


class RemoteControl:
    """Own the actuators on behalf of the laptop, with a link watchdog."""

    WATCHDOG_TIMEOUT_S = 2.0
    # Clamp limits per command key (match the Pi GUI's own widget ranges).
    LIMITS = {
        "heater_pwm":   (0.0, 100.0),   # %
        "spooler_duty": (0.0, 100.0),   # %
        "stepper_rpm":  (0.0, 20.0),    # RPM
        "fan_duty":     (0.0, 100.0),   # %
    }

    def __init__(self, gui) -> None:
        self.gui = gui
        self.active = False
        self.last_rx = 0.0     # wall-clock time of the last cmd/heartbeat
        self.status_reason = "off"
        # Desired outputs: written by the network thread, applied by the
        # hardware loop. Plain floats -> safe enough to poke cross-thread.
        self.targets = {key: 0.0 for key in RemoteControl.LIMITS}
        self._applied = None   # last values actually written to the hardware

    # ------------------------------------------------------------------ #
    # Called from the NETWORK thread (external_diameter._handle_command)
    # ------------------------------------------------------------------ #
    def set_mode(self, enabled: bool) -> None:
        if enabled:
            if self.gui.experiment.is_active():
                self._notify("rejected: an experiment is running")
                return
            self.targets = {key: 0.0 for key in self.targets}
            self._applied = None
            self.last_rx = time.time()
            # Make the Pi's own control loops let go of the hardware.
            self.gui.device_started = False
            self.gui.heater_open_loop_enabled = False
            self.gui.dc_motor_open_loop_enabled = False
            self.gui.dc_motor_close_loop_enabled = False
            self.status_reason = "laptop in control"
            self.active = True
            self._notify("remote mode ON - all outputs start at zero")
        else:
            self.abort("remote mode OFF (laptop request)")

    def command(self, message: dict) -> None:
        """Take any subset of LIMITS keys from one {"type":"cmd"} message."""
        self.last_rx = time.time()
        if not self.active:
            return
        for key, (low, high) in RemoteControl.LIMITS.items():
            if key in message:
                try:
                    value = float(message[key])
                except (TypeError, ValueError):
                    continue
                self.targets[key] = min(high, max(low, value))

    def heartbeat(self) -> None:
        self.last_rx = time.time()

    def is_active(self) -> bool:
        return self.active

    def abort(self, reason: str = "aborted") -> None:
        """Leave remote mode and get every output zeroed.

        May run on the network thread, so it does not touch the PWMs
        itself: it raises the same one-shot stop flags the GUI's STOP
        buttons use, and the hardware loop services them on its next pass.
        The fan is held off too (fan_enabled) because manual mode would
        otherwise re-drive it from the GUI slider immediately."""
        self.active = False
        self.status_reason = reason
        self.targets = {key: 0.0 for key in self.targets}
        self._applied = None
        self.gui.heater_stop_requested = True
        self.gui.stepper_stop_requested = True
        self.gui.dc_motor_stop_requested = True
        self.gui.fan_enabled = False
        self._notify(reason)

    # ------------------------------------------------------------------ #
    # Called from the HARDWARE loop (main.py, every iteration while active)
    # ------------------------------------------------------------------ #
    def update(self, extruder, spooler, fan) -> None:
        if not self.active:
            return

        # Watchdog: the laptop went quiet -> kill every output ourselves.
        if time.time() - self.last_rx > RemoteControl.WATCHDOG_TIMEOUT_S:
            self._safe_stop(extruder, spooler, fan)
            self.active = False
            self.status_reason = "WATCHDOG tripped: link quiet, outputs zeroed"
            print(f"[RemoteControl] {self.status_reason}")
            self._notify(self.status_reason)
            return

        desired = dict(self.targets)
        if desired == self._applied:
            return   # nothing changed - do not touch the PWMs
        previous = self._applied or {}
        try:
            if desired["heater_pwm"] != previous.get("heater_pwm"):
                extruder.heater_pwm.ChangeDutyCycle(desired["heater_pwm"])
            if desired["spooler_duty"] != previous.get("spooler_duty"):
                spooler.update_duty_cycle(desired["spooler_duty"])
            if desired["stepper_rpm"] != previous.get("stepper_rpm"):
                if desired["stepper_rpm"] > 0.0:
                    extruder.set_motor_speed(desired["stepper_rpm"])
                else:
                    extruder.stop_stepper()
            if desired["fan_duty"] != previous.get("fan_duty"):
                fan.update_duty_cycle(desired["fan_duty"])
            self._applied = desired
        except Exception as exc:
            print(f"[RemoteControl] apply error: {exc}")

    def _safe_stop(self, extruder, spooler, fan) -> None:
        try:
            extruder.stop_heater()
            extruder.stop_stepper()
            spooler.stop_motor()
            fan.update_duty_cycle(0)
        except Exception as exc:
            print(f"[RemoteControl] safe-stop error: {exc}")
        # Hold the fan off: without this, manual mode re-drives it from the
        # GUI slider on the very next loop pass and the stop looks "ignored".
        self.gui.fan_enabled = False
        self.targets = {key: 0.0 for key in self.targets}
        self._applied = None

    # ------------------------------------------------------------------ #
    # Status
    # ------------------------------------------------------------------ #
    def status_line(self) -> str:
        if self.active:
            return "REMOTE CONTROL: laptop driving (watchdog armed)"
        return f"Remote: {self.status_reason}"

    def _notify(self, message: str) -> None:
        """Tell the laptop what happened (best-effort)."""
        try:
            self.gui.diameter_source.send_message({
                "type": "remote_status",
                "active": self.active,
                "message": message,
            })
        except Exception:
            pass
