"""FrED main program - lighter hardware loop, INA219 logged and selectable CSV export.

Drop-in replacement for ``main.py`` (run it with ``python3 main_and_ina.py``).
Three differences, all contained in THIS file - no other module is modified:

1. INA219 in the database and in the export
   The spooler current sensor (``current_sensor.py``) already fills
   ``Database.ina_timestamps / spooler_current / spooler_bus_voltage`` from its
   own 200 Hz thread. Here those buffers become first-class export channels:
   they are listed in the export panel and written by the same exporter as
   everything else, on the same clock (the hardware loop's ``init_time``), so
   the current lines up with temperature/RPM with no post-processing.

2. Lighter hardware loop
   The loop polls at the same ~500 Hz (the 100 Hz FastSampler needs the
   headroom) but each poll is now nearly free:
     * The fan duty and the stepper speed are written to the hardware ONLY when
       the commanded value changes, instead of on every single poll. Before,
       ``fan.control_loop()`` re-wrote the PWM ~500 times a second AND appended
       one entry to ``Database.fan_duty_cycle`` each time (~1.8 M values per
       hour); ``extruder.stepper_control_loop()`` did the same to
       ``extruder_rpm`` and glitched the step PWM (duty 0, then reapply) on
       every pass.
     * ``Database.time_readings`` becomes a counter object instead of an
       unbounded list of floats (it was only ever used for the loop-rate
       read-out in the interface, which still works - see LoopClock).
     * The mode branches are a single dispatch instead of a repeated
       if/continue chain.

3. Choose what gets exported
   A "Select data to export" panel with one checkbox per signal is inserted
   into the interface, and the "Download CSV File" button writes only the
   ticked channels. Everything is still RECORDED - the checkboxes filter the
   CSV, not the logging - so the same run can be re-exported with a different
   selection without losing data.
"""
import csv
import threading
import time
import traceback

import RPi.GPIO as GPIO
from PyQt5.QtWidgets import (QCheckBox, QGroupBox, QHBoxLayout, QLabel,
                             QMessageBox, QPushButton, QScrollArea,
                             QVBoxLayout, QWidget)

from database import Database
from user_interface import UserInterface
from fan import Fan
from spooler import Spooler
from extruder import Extruder
from current_sensor import CurrentSensor
from fast_sampler import FastSampler

# Hardware-control loop poll period. The loop only reads sensors / runs the
# PID / appends data - the plots are redrawn on the GUI thread by a QTimer, so
# drawing never blocks sampling. ~500 Hz gives the 100 Hz FastSampler and the
# user-selectable control rate (up to 100 Hz) plenty of margin.
LOOP_SLEEP = 0.002


# ======================================================================== #
# Exportable channels
# ======================================================================== #
class Channel:
    """One tickable signal: a label, and the CSV column(s) it writes.

    ``columns`` is a list of ``(header, Database buffer name)`` pairs, so a
    single checkbox can cover a natural set such as Kp/Ki/Kd.
    """

    def __init__(self, key: str, label: str, columns, default: bool = True):
        self.key = key
        self.label = label
        self.columns = columns
        self.default = default


class Group:
    """Channels that share one timestamp buffer, written as one CSV table.

    Signals recorded on different clocks/rates (10 Hz control loops, 100 Hz
    fast sampler, 200 Hz INA219) cannot share rows, so each group is its own
    table with its own "Timestamp (s)" column. Every group uses the same time
    origin, so the tables align by timestamp.
    """

    def __init__(self, title: str, time_buffer: str, channels):
        self.title = title
        self.time_buffer = time_buffer
        self.channels = channels


EXPORT_GROUPS = [
    Group("TEMPERATURE DATA", "temperature_timestamps", [
        Channel("temperature", "Temperature (C)",
                [("Temperature (C)", "temperature_readings")]),
        Channel("temperature_setpoint", "Temperature setpoint (C)",
                [("Temperature setpoint (C)", "temperature_setpoint")]),
        Channel("temperature_error", "Temperature error (C)",
                [("Temperature error (C)", "temperature_error")], default=False),
        Channel("temperature_pid", "Temperature PID output",
                [("Temperature PID output", "temperature_pid_output")],
                default=False),
        Channel("temperature_gains", "Temperature gains (Kp/Ki/Kd)",
                [("Temperature Kp", "temperature_kp"),
                 ("Temperature Ki", "temperature_ki"),
                 ("Temperature Kd", "temperature_kd")], default=False),
    ]),
    Group("DIAMETER DATA", "camera_timestamps", [
        Channel("diameter", "Diameter (mm)",
                [("Diameter (mm)", "diameter_readings")]),
        Channel("diameter_setpoint", "Diameter setpoint (mm)",
                [("Diameter setpoint (mm)", "diameter_setpoint")]),
    ]),
    Group("MOTOR DATA", "spooler_timestamps", [
        Channel("spooler_rpm", "Spooler RPM",
                [("Spooler RPM", "spooler_rpm")]),
        Channel("spooler_setpoint", "Spooler setpoint (RPM)",
                [("Spooler setpoint (RPM)", "spooler_setpoint")]),
        Channel("spooler_gains", "Spooler gains (Kp/Ki/Kd)",
                [("Spooler Kp", "spooler_kp"),
                 ("Spooler Ki", "spooler_ki"),
                 ("Spooler Kd", "spooler_kd")], default=False),
        # Recorded by the stepper loop, which now runs only when the commanded
        # speed changes, so this column is a change log rather than a time
        # series. For a paced signal use "Stepper setpoint" in the fast table.
        Channel("extruder_rpm", "Extruder stepper command (RPM)",
                [("Extruder RPM", "extruder_rpm")], default=False),
    ]),
    Group("SPOOLER CURRENT DATA (INA219)", "ina_timestamps", [
        Channel("ina_current", "Spooler current (mA) [INA219]",
                [("Current (mA)", "spooler_current")]),
        Channel("ina_bus_voltage", "Spooler bus voltage (V) [INA219]",
                [("Bus voltage (V)", "spooler_bus_voltage")]),
    ]),
    Group("FAST SAMPLING DATA (100 Hz)", "fast_timestamps", [
        Channel("fast_temperature", "Temperature (C) @100 Hz",
                [("Temperature (C)", "fast_temperature")]),
        Channel("fast_spooler_rpm", "Spooler RPM @100 Hz",
                [("Spooler RPM", "fast_spooler_rpm")]),
        Channel("fast_stepper_rpm", "Stepper setpoint (RPM) @100 Hz",
                [("Stepper setpoint (RPM)", "fast_stepper_rpm")], default=False),
        Channel("fast_fan_duty", "Fan duty (%) @100 Hz",
                [("Fan duty (%)", "fast_fan_duty")], default=False),
    ]),
]


class _ListWriter:
    """Minimal file-like sink so csv.writer can append straight to a list."""

    def __init__(self, sink):
        self.write = sink.append


def build_csv(selected_keys, starts=None, t0: float = 0.0) -> str:
    """Render the selected channels as a multi-table CSV string.

    ``starts`` optionally maps a buffer name to the index its slice starts at
    (to export only a recording window); ``t0`` is subtracted from every
    timestamp so the exported clock starts at 0 s.
    """
    starts = starts or {}
    out = []
    writer = csv.writer(_ListWriter(out))

    for group in EXPORT_GROUPS:
        channels = [c for c in group.channels if c.key in selected_keys]
        if not channels:
            continue
        timestamps = getattr(Database, group.time_buffer)[
            starts.get(group.time_buffer, 0):]
        if not timestamps:
            continue

        columns = []   # (header, sliced buffer)
        for channel in channels:
            for header, buffer_name in channel.columns:
                columns.append((header, getattr(Database, buffer_name)[
                    starts.get(buffer_name, 0):]))

        if out:
            writer.writerow([])
            writer.writerow([])
        writer.writerow([group.title])
        writer.writerow(["Timestamp (s)"] + [header for header, _ in columns])
        for i in range(len(timestamps)):
            row = [f"{timestamps[i] - t0:.3f}"]
            for _, values in columns:
                row.append(values[i] if i < len(values) else "")
            writer.writerow(row)

    return "".join(out)


def export_csv(filename: str, selected_keys) -> str:
    """Write the selected channels to ``<filename>.csv`` and return its name."""
    filename = filename.strip() or "fred_data"
    if not filename.lower().endswith(".csv"):
        filename += ".csv"
    with open(filename, "w", newline="", encoding="utf-8") as file:
        file.write(build_csv(selected_keys))
    print(f"CSV file {filename} generated.")
    return filename


# ======================================================================== #
# Export-selection panel (inserted into the existing interface)
# ======================================================================== #
class ExportSelector(QGroupBox):
    """Checkbox panel choosing which signals the CSV export contains."""

    def __init__(self, gui: UserInterface) -> None:
        super().__init__("Select data to export")
        self.gui = gui
        self.boxes = {}

        layout = QVBoxLayout()
        note = QLabel("Everything is always recorded; these boxes only choose "
                      "what goes into the CSV. Each block is written as its own "
                      "table (they have different sampling rates), all sharing "
                      "the same clock.")
        note.setWordWrap(True)
        note.setStyleSheet("color: #888888;")
        layout.addWidget(note)

        buttons = QHBoxLayout()
        for text, state in (("Select all", True), ("Select none", False)):
            button = QPushButton(text)
            button.setStyleSheet(gui.BUTTON_STYLE)
            button.clicked.connect(lambda _checked, s=state: self.set_all(s))
            buttons.addWidget(button)
        layout.addLayout(buttons)

        for group in EXPORT_GROUPS:
            title = QLabel(group.title)
            title.setStyleSheet("font-weight: bold; margin-top: 4px;")
            layout.addWidget(title)
            for channel in group.channels:
                box = QCheckBox(channel.label)
                box.setChecked(channel.default)
                self.boxes[channel.key] = box
                layout.addWidget(box)

        self.setLayout(layout)

    def set_all(self, state: bool) -> None:
        for box in self.boxes.values():
            box.setChecked(state)

    def selected_keys(self):
        return {key for key, box in self.boxes.items() if box.isChecked()}


def install_export_panel(gui: UserInterface) -> ExportSelector:
    """Add the checkbox panel and point "Download CSV File" at the new export.

    Must run on the GUI thread, before ``start_gui()``.
    """
    selector = ExportSelector(gui)

    # The controls column lives inside the window's QScrollArea; drop the panel
    # in just above the trailing stretch.
    scroll = gui.window.findChild(QScrollArea)
    host = scroll.widget() if scroll else None
    layout = host.layout() if isinstance(host, QWidget) else None
    if layout is not None:
        layout.insertWidget(max(layout.count() - 1, 0), selector)
    else:   # layout not found (interface changed): show it as its own window
        selector.show()

    def download() -> None:
        keys = selector.selected_keys()
        if not keys:
            QMessageBox.warning(gui.app.activeWindow(), "Download CSV",
                                "No data selected. Tick at least one signal in "
                                "'Select data to export'.")
            return
        try:
            name = export_csv(gui.csv_filename.text(), keys)
        except Exception as exc:
            QMessageBox.critical(gui.app.activeWindow(), "Download CSV",
                                 f"Could not write the CSV file: {exc}")
            return
        QMessageBox.information(gui.app.activeWindow(), "Download CSV",
                                f"Saved {name} with {len(keys)} selected "
                                "signal(s).")

    for button in gui.window.findChildren(QPushButton):
        if button.text() == "Download CSV File":
            try:
                button.clicked.disconnect()
            except TypeError:
                pass   # nothing connected yet
            button.clicked.connect(download)
    return selector


# ======================================================================== #
# Hardware control
# ======================================================================== #
class LoopClock:
    """Counts hardware-loop iterations without keeping every timestamp.

    Replaces ``Database.time_readings`` (a list that grew by ~500 floats per
    second and is only read for the interface's loop-rate read-out, which uses
    ``len()``). Supports the few list operations the rest of the code performs
    on that buffer.
    """

    def __init__(self) -> None:
        self.count = 0
        self.last = 0.0

    def append(self, value: float) -> None:
        self.count += 1
        self.last = value

    def __len__(self) -> int:
        return self.count

    def __bool__(self) -> bool:
        return self.count > 0

    def __getitem__(self, index):
        if index in (-1, self.count - 1):
            return self.last
        raise IndexError("LoopClock only keeps the latest timestamp")


class Actuators:
    """Write the fan duty and stepper speed only when the command changes.

    Both are open-loop commands that the user (or an experiment) changes on
    human timescales, so re-sending them ~500 times a second only burned CPU,
    glitched the stepper PWM and filled the database with duplicates.
    """

    EPSILON = 0.01   # ignore changes smaller than this (%, RPM)

    def __init__(self, gui: UserInterface, fan: Fan, extruder: Extruder) -> None:
        self.gui = gui
        self.fan = fan
        self.extruder = extruder
        self.fan_duty = None
        self.stepper_rpm = None

    def invalidate(self) -> None:
        """Forget the cached commands.

        Called when an experiment or the laptop's remote mode drives the
        actuators directly, so the next manual pass re-sends its own values.
        """
        self.fan_duty = None
        self.stepper_rpm = None

    def update_fan(self) -> None:
        try:
            duty = (float(self.gui.get_fan_duty())
                    if getattr(self.gui, "fan_enabled", True) else 0.0)
        except Exception as exc:
            print(f"Error reading the fan duty cycle: {exc}")
            return
        if self.fan_duty is not None and abs(duty - self.fan_duty) < self.EPSILON:
            return
        self.fan_duty = duty
        self.fan.update_duty_cycle(duty)

    def update_stepper(self) -> None:
        try:
            rpm = float(self.gui.get_extrusion_speed())
        except Exception as exc:
            print(f"Error reading the extrusion speed: {exc}")
            return
        if (self.stepper_rpm is not None
                and abs(rpm - self.stepper_rpm) < self.EPSILON):
            return
        self.stepper_rpm = rpm
        self.extruder.stepper_control_loop()


def _emergency_stop(extruder: Extruder, spooler: Spooler, fan: Fan) -> None:
    """Drive every actuator to zero (used when a STOP aborts a remote run)."""
    extruder.stop_heater()
    extruder.stop_stepper()
    spooler.stop_motor()
    fan.update_duty_cycle(0)


def hardware_control(gui: UserInterface) -> None:
    """Thread that reads the sensors, runs the control loops and logs data."""
    time.sleep(1)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    try:
        fan = Fan(gui)
        spooler = Spooler(gui)
        extruder = Extruder(gui)
        fan.start(1000, 45)
        spooler.start(1000, 0)
        # 100 Hz measurement sampler (the control loops keep their tuned
        # cadence). Runs in THIS thread so SPI reads never race the control.
        fast_sampler = FastSampler(gui, extruder, spooler)
        fast_sampler.start_telemetry()   # stream the samples to the laptop
        actuators = Actuators(gui, fan, extruder)
    except Exception as exc:
        # No GUI calls from this thread: a modal QMessageBox opened outside the
        # GUI thread never renders and blocks this thread forever.
        print(f"Error in hardware control: {exc}", flush=True)
        traceback.print_exc()
        return

    # Optional INA219 spooler-current sensor. It samples on its OWN 200 Hz
    # thread, stamped with the same init_time clock as every other buffer, and
    # writes straight into Database.ina_* (exported by build_csv above). If it
    # is missing or miswired, the loop below runs exactly as it would without it.
    current_sensor = CurrentSensor(gui)

    clock = LoopClock()
    Database.time_readings = clock
    init_time = time.time()
    current_sensor.start(init_time)

    while True:
        try:
            current_time = time.time() - init_time
            clock.append(current_time)

            # High-rate measurements (100 Hz) first, so sampling runs in every
            # mode including the ones that skip the control section below.
            fast_sampler.sample(current_time)

            # --- Manual STOP requests (one-shot) --------------------------- #
            # Each Stop button just sets a flag; we service it here so the
            # actuator output is actively driven to zero, not left at its last
            # value. The control flags are already cleared by the UI handler.
            stop_pressed = (gui.stepper_stop_requested
                            or gui.heater_stop_requested
                            or gui.dc_motor_stop_requested)
            if gui.stepper_stop_requested:
                extruder.stop_stepper()
                actuators.stepper_rpm = None   # re-send on the next change
                gui.stepper_stop_requested = False
            if gui.heater_stop_requested:
                extruder.stop_heater()
                gui.heater_stop_requested = False
            if gui.dc_motor_stop_requested:
                spooler.stop_motor()
                gui.dc_motor_stop_requested = False

            if gui.start_motor_calibration:
                spooler.calibrate()
                gui.start_motor_calibration = False

            # --- Remote experiment: an automated run sent from the laptop.
            #     While active it owns the hardware (manual controls ignored),
            #     EXCEPT the STOP buttons, which abort it for safety. -------- #
            if gui.experiment.is_active():
                actuators.invalidate()
                if stop_pressed:
                    gui.experiment.abort()
                    _emergency_stop(extruder, spooler, fan)
                else:
                    gui.experiment.update(current_time, extruder, spooler, fan)
                time.sleep(LOOP_SLEEP)
                continue

            # --- Remote command mode: the laptop drives the actuators raw.
            #     The Pi's STOP buttons abort it; a link watchdog inside
            #     update() kills the outputs on silence. -------------------- #
            if gui.remote.is_active():
                actuators.invalidate()
                if stop_pressed:
                    gui.remote.abort("STOP pressed on the Pi")
                    _emergency_stop(extruder, spooler, fan)
                else:
                    gui.remote.update(extruder, spooler, fan)
                    if gui.diameter_loop_enabled:
                        gui.diameter_source.update(current_time)
                time.sleep(LOOP_SLEEP)
                continue

            # --- Monitor mode: graph the temperature and spooler RPM with NO
            #     control output applied. Takes priority over the control loops
            #     so nothing drives the system while observing. -------------- #
            if gui.monitor_mode_enabled:
                extruder.monitor_temperature(current_time)
                spooler.monitor_rpm(current_time)
                if gui.diameter_loop_enabled:
                    gui.diameter_source.update(current_time)
                actuators.update_fan()
                time.sleep(LOOP_SLEEP)
                continue

            # --- Manual control -------------------------------------------- #
            if gui.dc_motor_open_loop_enabled and not gui.dc_motor_close_loop_enabled:
                spooler.dc_motor_open_loop_control(current_time)
            elif gui.dc_motor_close_loop_enabled and not gui.dc_motor_open_loop_enabled:
                spooler.dc_motor_close_loop_control(current_time)

            if gui.heater_open_loop_enabled and not gui.device_started:
                extruder.temperature_open_loop_control(current_time)
                actuators.update_stepper()

            # Diameter feedback streamed from the external CV computer
            if gui.diameter_loop_enabled:
                gui.diameter_source.update(current_time)

            if gui.device_started:
                extruder.temperature_control_loop(current_time)
                actuators.update_stepper()

            actuators.update_fan()
            time.sleep(LOOP_SLEEP)
        except Exception as exc:
            # Zero the outputs and keep the loop alive. NEVER pop a dialog from
            # this thread: a modal QMessageBox outside the GUI thread never
            # renders, and this loop would hang forever.
            print(f"Error in hardware control loop: {exc}", flush=True)
            traceback.print_exc()
            fan.stop()
            spooler.stop()
            extruder.stop()


if __name__ == "__main__":
    print("Starting FrED Device (lighter loop + INA219 export)...")
    ui = UserInterface()
    install_export_panel(ui)
    time.sleep(2)

    # Daemon: the loop runs forever, so it must not keep the process alive
    # after the window is closed (main.py's join() hung on exit).
    hardware_thread = threading.Thread(target=hardware_control, args=(ui,),
                                       daemon=True)
    hardware_thread.start()

    # Start GUI (blocking)
    try:
        ui.start_gui()
    except KeyboardInterrupt:
        print("GUI stopped.")

    print("FrED Device Closed.")
