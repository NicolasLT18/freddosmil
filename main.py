"""Main file to run the FrED device"""
import threading
import time
import traceback
import RPi.GPIO as GPIO
from database import Database
from user_interface import UserInterface
from fan import Fan
from spooler import Spooler
from extruder import Extruder
from current_sensor import CurrentSensor
from fast_sampler import FastSampler

# Hardware-control loop poll period. The loop now only reads sensors / runs the
# PID / appends data - the plots are redrawn on the GUI thread by a QTimer
# (UserInterface.Plot.redraw), so drawing no longer blocks sampling. With that
# bottleneck gone, the loop polls at ~500 Hz so it can comfortably service the
# user-selectable sampling rate (UserInterface.get_sample_period) up to 100 Hz.
# Each non-sampling poll is cheap (just a timestamp check), and the camera/CV is
# gone, so this is light on the Pi 4.
LOOP_SLEEP = 0.002

def hardware_control(gui: UserInterface) -> None:
    """Thread to handle hardware control"""
    time.sleep(1)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    try:
        fan = Fan(gui)
        spooler = Spooler(gui)
        extruder = Extruder(gui)
        fan.start(1000, 45)
        spooler.start(1000, 0)
        # 100 Hz measurement sampler (control loops stay at their tuned
        # 10 Hz). Runs in THIS thread so SPI reads never race the control.
        fast_sampler = FastSampler(gui, extruder, spooler)
        fast_sampler.start_telemetry()   # stream the samples to the laptop
    except Exception as e:
        # No GUI calls from this thread: a modal QMessageBox opened outside
        # the GUI thread never renders and blocks this thread forever.
        print(f"Error in hardware control: {e}", flush=True)
        traceback.print_exc()
        return

    # Optional INA219 spooler-current sensor. Handles its own failures: if it
    # is not wired/installed the loop below runs exactly as before. It samples
    # on its OWN thread at 200 Hz, stamped with this same init_time clock so
    # its timestamps match every other sensor's.
    current_sensor = CurrentSensor(gui)

    init_time = time.time()
    current_sensor.start(init_time)
    while True:
        try:
            current_time = time.time() - init_time
            Database.time_readings.append(current_time)

            # High-rate measurements (100 Hz), BEFORE the experiment/monitor
            # branches below 'continue', so sampling runs in every mode.
            fast_sampler.sample(current_time)

            # --- Manual STOP requests from the interface (one-shot) --------- #
            # Each Stop button just sets a flag; we service it here so the
            # actuator output is actively driven to zero, not left at its last
            # value. The control flags are already cleared by the UI handler.
            stop_pressed = (gui.stepper_stop_requested or
                            gui.heater_stop_requested or
                            gui.dc_motor_stop_requested)
            if gui.stepper_stop_requested:
                extruder.stop_stepper()
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
            #     While it is active it owns the hardware (manual controls are
            #     ignored), EXCEPT the STOP buttons, which abort it for safety. ---
            if gui.experiment.is_active():
                if stop_pressed:
                    gui.experiment.abort()
                    extruder.stop_heater()
                    extruder.stop_stepper()
                    spooler.stop_motor()
                    fan.update_duty_cycle(0)
                else:
                    gui.experiment.update(current_time, extruder, spooler, fan)
                time.sleep(LOOP_SLEEP)
                continue

            # --- Remote command mode (step 3b): the laptop drives the
            #     actuators raw. The Pi's STOP buttons abort it; a link
            #     watchdog inside update() kills the outputs on silence. ---
            if gui.remote.is_active():
                if stop_pressed:
                    gui.remote.abort("STOP pressed on the Pi")
                    extruder.stop_heater()
                    extruder.stop_stepper()
                    spooler.stop_motor()
                    fan.update_duty_cycle(0)
                else:
                    gui.remote.update(extruder, spooler, fan)
                    if gui.diameter_loop_enabled:
                        gui.diameter_source.update(current_time)
                time.sleep(LOOP_SLEEP)
                continue

            # --- Monitor mode: read & graph temperature and spooler RPM with
            #     NO control output applied. Takes priority over the control
            #     loops below so nothing drives the system while observing. ---
            if gui.monitor_mode_enabled:
                extruder.monitor_temperature(current_time)
                spooler.monitor_rpm(current_time)
                if gui.diameter_loop_enabled:
                    gui.diameter_source.update(current_time)
                fan.control_loop()
                time.sleep(LOOP_SLEEP)
                continue

            # DC Motor Control Logic
            if gui.dc_motor_open_loop_enabled and not gui.dc_motor_close_loop_enabled:
                spooler.dc_motor_open_loop_control(current_time)

            elif gui.dc_motor_close_loop_enabled and not gui.dc_motor_open_loop_enabled:
                spooler.dc_motor_close_loop_control(current_time)

            # Heater Control Logic
            if gui.heater_open_loop_enabled and not gui.device_started:
                extruder.temperature_open_loop_control(current_time)
                extruder.stepper_control_loop()

            # Diameter feedback streamed from the external CV computer
            if gui.diameter_loop_enabled:
                gui.diameter_source.update(current_time)

            if gui.device_started:
                extruder.temperature_control_loop(current_time)
                extruder.stepper_control_loop()

            fan.control_loop()
            time.sleep(LOOP_SLEEP)
        except Exception as e:
            # Zero the outputs and keep the loop alive. NEVER pop a dialog
            # from this thread: a modal QMessageBox outside the GUI thread
            # never renders, and this loop hangs forever (remote commands
            # were silently ignored after that).
            print(f"Error in hardware control loop: {e}", flush=True)
            traceback.print_exc()
            fan.stop()
            spooler.stop()
            extruder.stop()

if __name__ == "__main__":
    print("Starting FrED Device...")
    ui = UserInterface()
    time.sleep(2)

    hardware_thread = threading.Thread(target=hardware_control, args=(ui,))
    hardware_thread.start()
    threading.Lock()

    # Start GUI (blocking)
    try:
        ui.start_gui()
    except KeyboardInterrupt:
        print("GUI stopped.")

    # Cleanup
    hardware_thread.join()
    print("FrED Device Closed.")

