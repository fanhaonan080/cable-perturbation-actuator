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

2. Run with experimental protocol:
   python main_loop.py --protocol training --bodyweight_kg 70
   python main_loop.py --protocol real_trial --bodyweight_kg 65 --seed 42

3. Run in test mode (sample force map):
   python main_loop.py

4. Wait for initial calibration to complete
5. Enter commands via keyboard when prompted

COMMAND-LINE ARGUMENTS:
-----------------------
--protocol {training,real_trial}  Protocol type for force perturbations
--bodyweight_kg WEIGHT               Participant bodyweight in kg
--bodyweight_lbs WEIGHT           Participant bodyweight in lbs (alternative to --bodyweight_kg)
--seed SEED                       Random seed for real_trial protocol (optional)
--datafile NAME                   Data file name (default: test0)
"""

import ACTUATOR
import time
import traceback
import Controllers
import threading
import asyncio
import argparse

async def main(protocol_type=None, bodyweight_kg=None, protocol_seed=None, datafile_name="test0"):
    daq_channel = "Dev1/port0/line0"
    counter_channel = "Dev1/port0/line3"  # Counter channel for pulse counting
    actuator = await ACTUATOR.connect_to_actuator(dataFile_name=datafile_name,
                                                  daq_channel=daq_channel,
                                                  counter_channel=counter_channel)

    await actuator.initial_calibration()
    print('Start!')

    actuator_controller = Controllers.TTLController(
        actuator, 
        with_keyboard=True,
        protocol_type=protocol_type,
        bodyweight_kg=bodyweight_kg,
        protocol_seed=protocol_seed
    )

    actuator_controller.angle_setpoint_value = 45  # Default setpoint value (can be changed via keyboard commands)
    actuator_controller.setpoint_type = Controllers.SetpointType.CAM_ANGLE  # Default control mode (can be changed via keyboard commands)


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
    current_force = actuator_controller.pulse_force_map.get(actuator.data.ttl_pulse_count, 0)
    next_force = actuator_controller.pulse_force_map.get(actuator.data.ttl_pulse_count+1, 0)
    current_pct = (current_force / actuator_controller.bodyweight_N * 100) if actuator_controller.bodyweight_N else 0
    next_pct = (next_force / actuator_controller.bodyweight_N * 100) if actuator_controller.bodyweight_N else 0
    print(f"Current Trail: {actuator.data.ttl_pulse_count} → Force: {current_force}N ({current_pct:.1f}% BW)")
    print(f"Next Trail: {actuator.data.ttl_pulse_count+1} → Force: {next_force}N ({next_pct:.1f}% BW)")
    if actuator_controller.excessive_negative_displacement:
        print("Warning: Excessive negative cable displacement detected!")
    if actuator_controller.excessive_positive_displacement:
        print("Warning: Excessive positive cable displacement detected!")
    if actuator_controller.cable_tension_lost:
        print("Warning: Cable tension lost detected! Check for cable routing or breakage! Retype command to restore controller.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Cable Perturbation Actuator Control Loop',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Training protocol for 70kg participant:
    python main_loop.py --protocol training --bodyweight_kg 70
    
  Training protocol for 154lb participant:
    python main_loop.py --protocol training --bodyweight_lbs 154
    
  Real trial protocol for 65kg participant with seed:
    python main_loop.py --protocol real_trial --bodyweight_kg 65 --seed 42
    
  Test mode (no protocol):
    python main_loop.py
        """
    )
    
    parser.add_argument('--protocol', type=str, choices=['training', 'real_trial'],
                       help='Protocol type: training (40 trials) or real_trial (50 trials)')
    parser.add_argument('--bodyweight_kg', type=float,
                       help='Participant bodyweight in kilograms')
    parser.add_argument('--bodyweight_lbs', type=float, dest='bodyweight_lbs',
                       help='Participant bodyweight in pounds (alternative to --bodyweight_kg)')
    parser.add_argument('--seed', type=int, default=None,
                       help='Random seed for real_trial protocol (optional)')
    parser.add_argument('--datafile', type=str, default='test0',
                       help='Data file name (default: test0)')
    
    args = parser.parse_args()
    
    # Validate bodyweight input
    if args.bodyweight_kg is not None and args.bodyweight_lbs is not None:
        parser.error("Cannot specify both --bodyweight_kg and --bodyweight_lbs. Choose one.")
    
    # Convert lbs to kg if needed
    bodyweight_kg = None
    if args.bodyweight_kg is not None:
        bodyweight_kg = args.bodyweight_kg
    elif args.bodyweight_lbs is not None:
        bodyweight_kg = args.bodyweight_lbs * 0.453592  # Convert lbs to kg
        print(f"Bodyweight: {args.bodyweight_lbs} lbs = {bodyweight_kg:.2f} kg")
    
    # Validate that bodyweight is provided if protocol is specified
    if args.protocol is not None and bodyweight_kg is None:
        parser.error("--bodyweight_kg or --bodyweight_lbs is required when --protocol is specified")
    
    try:
        asyncio.run(main(
            protocol_type=args.protocol,
            bodyweight_kg=bodyweight_kg,
            protocol_seed=args.seed,
            datafile_name=args.datafile
        ))
    except KeyboardInterrupt:
        print("Program interrupted.")
    except Exception as e:
        print(f"Unexpected error: {e}")
