"""
Main control loop.

KEYBOARD COMMANDS:
------------------
Cam Angle Mode:
  c<angle>  - Set cam angle to specified degrees (e.g., "c45" sets cam angle to 45°)
  
Force Mode:
  f<force>  - Set peak perturbation force in Newtons (e.g., "f100" sets force to 100 N)
  
Other Commands:
  Type commands during runtime when prompted by the controller.
  Press 'q' or Ctrl+C to quit and safely shut down the actuator.

USAGE:
------
1. Ensure hardware (moteus controller and NI-DAQ) is connected
2. Run: python main_loop.py
3. Wait for initial calibration to complete
4. Enter commands via keyboard when prompted
"""

import ACTUATOR
import time
import traceback
import Controllers
import threading
import asyncio

async def main():
    datafile_name = "test0"
    daq_channel = "Dev1/port0/line0"
    counter_channel = "Dev1/port0/line3"  # Counter channel for pulse counting
    actuator = await ACTUATOR.connect_to_actuator(dataFile_name=datafile_name,
                                                  daq_channel=daq_channel,
                                                  counter_channel=counter_channel)

    await actuator.initial_calibration()
    print('Start!')

    actuator_controller = Controllers.TTLController(actuator, with_keyboard=True)

    # Defining controller variables
    target_period = 1 / actuator.config.control_loop_freq
    t0 = time.perf_counter()
    last_actuation_time = t0
    last_print_time = t0

    # Keyboard interrupt setup
    if actuator_controller.keyboard_thread is not None:
        quit_event = actuator_controller.keyboard_thread.quit_event
    else:
        quit_event = threading.Event()

    try:
        while not quit_event.is_set():
            while time.perf_counter() - last_actuation_time < target_period:
                pass
            
            time_now = time.perf_counter()
            last_actuation_time = time_now
            loop_time = time_now - t0 

            await actuator.read_data(loop_time=loop_time)
            await actuator.read_pulse_count()  # Read pulse counter

            await actuator_controller.command()
            
            actuator.write_data()
            
            # data print setup at ~1Hz
            if time_now - last_print_time >= 1:
                _print_status(actuator, actuator_controller)
                last_print_time = time_now  # Update the last controller update timestamp

    except Exception as err:
        print(traceback.print_exc())
        print("Unexpected error: ", err)
    
    finally:
        await actuator_controller.homing_procedure()
        quit_event.clear()
        await actuator.close()


def _print_status(actuator, actuator_controller):
    # print("Velocity: ", actuator.data.actuator_velocity, "Cam Angle: ", actuator.data.cam_angle, "Gains", actuator.config.camControllerGainKp, actuator.config.camControllerGainKd)
    # print("Torque: ", actuator.data.actuator_torque, "Commanded: ", actuator.data.commanded_actuator_torque)
    print("Cam Angle: ", actuator.data.cam_angle, "Commanded: ", actuator.data.commanded_cam_angle)
    # print("Force: ", actuator.data.measured_force, "Commanded: ", actuator.data.commanded_cable_force)
    # print("Actuator Angle: ", actuator.data.actuator_angle-actuator.actuator_offset)
    print("Cable Length: ", actuator.data.disturbance_displacement)
    # print("actuator velocity: ", actuator.data.actuator_velocity, "Commanded: ", actuator.data.commanded_actuator_velocity)
    print("Actuator Mode: ", actuator_controller.setpoint_type)
    print("Commanded Torque: ", actuator.data.commanded_actuator_torque)
    print(f"Pulse Count: {actuator.data.ttl_pulse_count} → Force: {actuator_controller.force_setpoint_value}N")
    if actuator_controller.excessive_negative_displacement:
        print("Warning: Excessive negative cable displacement detected!")
    if actuator_controller.excessive_positive_displacement:
        print("Warning: Excessive positive cable displacement detected!")
    if actuator_controller.cable_tension_lost:
        print("Warning: Cable tension lost detected! Check for cable routing or breakage! Retype command to restore controller.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unexpected error: {e}")
