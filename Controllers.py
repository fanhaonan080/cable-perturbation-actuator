from ACTUATOR import SpringActuator_moteus
import filters
from enum import Enum
import numpy as np  
from scipy.interpolate import PchipInterpolator
from scipy.interpolate import UnivariateSpline
import threading
from typing import Type
import time
import asyncio
import json
from datetime import datetime
import os

class SetpointType(Enum):
    NONE = 0
    CAM_ANGLE = 1
    ACTUATOR_VELOCITY = 2
    ACTUATOR_ANGLE = 4
    ACTUATOR_TORQUE = 5
    HOME_POSITION = 6
    CABLE_FORCE = 7
    GANTRY = 8

class Controller:
    """Parent controller object. Child classes inherit methods."""
    def __init__(self, actuator: SpringActuator_moteus, with_keyboard: bool = True):
        self.actuator = actuator
        self.setpoint_type = SetpointType.NONE
        self.setpoint_value = 0
        self.force_setpoint_value: float = 0.1
        self.angle_setpoint_value: float = actuator.config.homeAngle
        self.keyboard_thread = None
        if with_keyboard:
            self.keyboard_thread = ParameterParser(controller=self)
            self.keyboard_thread.start()         # Start keyboard thread if needed
        self.command_map = {
            'a': (SetpointType.ACTUATOR_ANGLE, 'angle'),
            'v': (SetpointType.ACTUATOR_VELOCITY, 'velocity'),
            't': (SetpointType.ACTUATOR_TORQUE, 'torque'),
            'c': (SetpointType.CAM_ANGLE, 'cam_angle'),
            'f': (SetpointType.CABLE_FORCE, 'force'),
            'h': (SetpointType.HOME_POSITION, 'home'),
            'g': (SetpointType.GANTRY, 'gantry'),
        }

    def command(self, reset: bool):
        """Override this method in child classes to define control logic."""
        raise ValueError("command() not defined in child class of Controller")

    def update_controller_variables(self):
        """Override this method in child classes to update controller variables."""
        pass 


class TTLController(Controller):
    def __init__(self, actuator: SpringActuator_moteus, with_keyboard: bool = True):
        super().__init__(actuator, with_keyboard=with_keyboard)
        self.cam_control_filter = filters.Butterworth(N=2, Wn=98, fs=actuator.config.control_loop_freq)
        self.torque_filter = filters.PaddedMovingAverage(4)

        self.home_returned = False

        self.excessive_negative_displacement = False
        self.excessive_positive_displacement = False
        self.cable_tension_lost = False
        
        self.angle_to_force_mode_switched = False
        self.transition_angle = None # cam angle when switching to force mode
        self.reference_angle = None # actuator angle when switching to force mode

        switching_ang_pts = [69,70,71.8,72.25,73.5,74,75,75.75,76.5,78]
        force_pts = [6,15,25,35,50,65,80,100,120,200]
        # self.switch_angle = PchipInterpolator(force_pts,switching_ang_pts)
        self.switch_angle = UnivariateSpline(force_pts, switching_ang_pts, s=0.5)

        # Force control mode tracking for time-based torque ramping
        self._force_control_mode = None  # "cam_angle" or "torque"
        self._torque_mode_start_time = None  # Time when torque mode was entered
       
        self._torque_ramp_duration = 3  # Default ramp duration in seconds
        self.negative_displacement_threshold = -500  # Threshold for excessive negative displacement
        self.positive_displacement_threshold = 1800  # Threshold for excessive positive displacement
        self.force_control_mode_threshold = 5  # Force threshold in Newtons to switch between cam angle and torque control
        # Track the setpoint mode before TTL trigger for proper restoration
        self._pre_ttl_setpoint_type = SetpointType.CAM_ANGLE  # Default to cam angle mode
        
        # Pulse count to force mapping - randomized for experiments
        # self.pulse_force_map, self.force_map_file = self._generate_randomized_force_map(
        #     min_force=20,
        #     max_force=60,
        #     num_trials=45,
        #     save_dir="force_maps",
        #     seed=None
        # )
        # test force map
        self.pulse_force_map = {
            1: 0,
            2: 20,
            3: 30,
            4: 0,
            5: 30,
            6: 0,
            7: 0,
            8: 30,
            9: 40,
            10: 30,
            11: 0,
            12: 30,
        }

    def _generate_randomized_force_map(self, min_force: float, max_force: float, 
                                       num_trials: int, save_dir: str, seed: int = None) -> tuple[dict, str]:
        """Generate a randomized force map and save it to a JSON file for tracking."""
        # Generate random forces as multiples of 10, including 0
        np.random.seed(seed)
        max_mult = int(max_force / 10)
        random_forces = np.random.randint(0, max_mult + 1, num_trials) * 10
        
        # Set forces below min_force to 0
        random_forces = np.where(random_forces < min_force, 0, random_forces)
        
        # Create pulse count to force mapping
        force_map = {i+1: float(force) for i, force in enumerate(random_forces)}
        
        # Save to JSON with metadata
        os.makedirs(save_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filepath = os.path.join(save_dir, f"force_map_{timestamp}.json")
        
        save_data = {
            "metadata": {
                "timestamp": timestamp,
                "min_force": min_force,
                "max_force": max_force,
                "num_trials": num_trials,
                "seed": seed,
                "datetime": datetime.now().isoformat(),
                "mean_force": float(np.mean(random_forces)),
                "std_force": float(np.std(random_forces))
            },
            "force_map": force_map
        }
        
        with open(filepath, 'w') as f:
            json.dump(save_data, f, indent=2)
        
        print(f"Generated randomized force map: {num_trials} trials, {min_force}-{max_force}N")
        print(f"Saved to: {filepath}")
        
        return force_map, filepath

    async def command(self, reset: bool = False) -> bool:
        """Issue a command to the actuator based on the current setpoint type."""
        try:
            # Update force setpoint based on pulse count
            pulse_count = self.actuator.data.ttl_pulse_count
            self.force_setpoint_value = self.pulse_force_map.get(pulse_count, 0) # Default to 0N if pulse count exceeds mapping
            
            # Handle TTL-triggered mode switching
            if self.actuator.data.ttl_triggered:
                # Save current mode before switching (only if not already in CABLE_FORCE)
                if self.setpoint_type != SetpointType.CABLE_FORCE:
                    self._pre_ttl_setpoint_type = self.setpoint_type
                self.setpoint_type = SetpointType.CABLE_FORCE
                # print(f"TTL Triggered: Pulse Count = {pulse_count}, Force = {self.force_setpoint_value}N")
            else:
                # Restore previous mode when TTL is not triggered
                if self.setpoint_type == SetpointType.CABLE_FORCE:
                    self.setpoint_type = self._pre_ttl_setpoint_type
                    # print(f"TTL Not Triggered: Restoring to {self._pre_ttl_setpoint_type.name} mode")
            
            # Update safety flags
            self.excessive_negative_displacement = self.actuator.data.disturbance_displacement < self.negative_displacement_threshold
            self.excessive_positive_displacement = self.actuator.data.disturbance_displacement > self.positive_displacement_threshold
            
            # Toggle behavior: only set to True when condition is met, stays True until manually reset
            if self.actuator.data.cam_angle < 0.5:
                self.cable_tension_lost = True
            
            # Check for safety violations
            if self.excessive_negative_displacement or self.excessive_positive_displacement or self.cable_tension_lost:
                if self.cable_tension_lost:
                    self.setpoint_type = SetpointType.NONE  # Override to NONE to stop the actuator if cable tension is lost
                await self.actuator.command_actuator_velocity(0, max_torque=0.3)  # Stop the actuator to prevent damage
                # await self.actuator.command_controller_off()

            else:

                if self.setpoint_type == SetpointType.CAM_ANGLE:
                    await self.actuator.command_cam_angle(self.angle_setpoint_value, error_filter=self.cam_control_filter)

                elif self.setpoint_type == SetpointType.CABLE_FORCE:
                    force_curve_is_finished = await self.force_control()
                    if force_curve_is_finished:
                        self.setpoint_type = self._pre_ttl_setpoint_type
                        self.actuator.data.ttl_triggered = False

                elif self.setpoint_type == SetpointType.NONE:
                    await self.actuator.command_relative_actuator_angle(0, self.actuator.data.actuator_angle)
            
                elif self.setpoint_type == SetpointType.HOME_POSITION:
                    self.actuator.update_camController_gains(kp_gain=10 ,kd_gain=0.01)
                    await self.actuator.command_cam_angle(self.actuator.config.homeAngle, error_filter=self.cam_control_filter)
                    if abs(self.actuator.data.cam_angle - self.actuator.config.homeAngle) < 0.2:
                        self.home_returned = True
                else:
                    raise ValueError(f"Unsupported setpoint type: {self.setpoint_type}")

        except Exception as e:
            print(f"Error during command execution: {e}")
        
                    
    def update_controller_variables(self, 
                                    setpoint_type: SetpointType, 
                                    setpoint_value: float):
        """Update the controller variables with safety checks."""
        if self.check_input_safety(setpoint_type=setpoint_type, setpoint_val=setpoint_value):
            
            # if setpoint_type != self.setpoint_type:
            #     self.cam_control_filter.restart()
            # Reset cable_tension_lost flag when manual command is entered
            self.cable_tension_lost = False
            
            print("current setpoint:", self.setpoint_type, self.setpoint_value)
            print(f"Updating: {setpoint_type}, value: {setpoint_value}")

            if setpoint_type==SetpointType.CABLE_FORCE:
                self.angle_to_force_mode_switched = False
                self.transition_angle = self.actuator.data.cam_angle
                self.force_setpoint_value = setpoint_value
                if self.setpoint_type != SetpointType.CABLE_FORCE:
                    self.torque_filter.restart()

            elif setpoint_type==SetpointType.CAM_ANGLE:
                self.angle_setpoint_value = setpoint_value
                if self.setpoint_type != SetpointType.CAM_ANGLE:
                    self.cam_control_filter.restart()

                print(f"Updated cam angle setpoint to {setpoint_value} degrees")
            else:
                pass
            self.setpoint_type = setpoint_type
            self.setpoint_value = setpoint_value
            self.reference_angle = self.actuator.data.actuator_angle
            self.home_returned = False
        else:
            print("Given setpoint may cause instability, so not updating")
    
    def check_input_safety(self, setpoint_type: SetpointType, setpoint_val: float) -> bool:
        """Check if the given setpoint is safe to use."""
        return True
    
    async def homing_procedure(self, timeout: float = 1.0):
        print("Moving to Home Position...")
        self.setpoint_type = SetpointType.HOME_POSITION

        home_start_time = time.perf_counter()
        target_period = 1 / self.actuator.config.control_loop_freq

        while not self.home_returned:
            if time.perf_counter() - home_start_time > timeout: 
                print(f"Failed to reach Home Position within {timeout}s timeout")
                return

            await self.actuator.read_data()
            await self.command()
            await asyncio.sleep(target_period)

        print("I'm Home, Now Exiting Gracefully")
        return
    
    
    async def force_control(self, target_force: float = None, ramp_duration: float = None):
        '''
        Hybrid force control: cam angle control below switch point, time-based torque ramping above.
        
        Parameters:
        -----------
        target_force : float
            Target cable force in Newtons
        error_filter : filters.Filter, optional
            Filter for cam angle error derivative (defaults to cam_angle_control_filter)
        ramp_duration : float, optional
            Time in seconds to ramp from initial to final torque (default: 0.1s)
        
        Returns:
        --------
        force_curve_is_finished : bool
            True if the force curve has completed (only relevant in torque mode)
        '''
        force_curve_is_finished = False

        if ramp_duration is not None:
            self._torque_ramp_duration = ramp_duration
        target_force = self.force_setpoint_value if target_force is None else target_force
        if target_force < self.force_control_mode_threshold:
            await self.actuator.command_cam_angle(self.angle_setpoint_value, error_filter=self.cam_control_filter)
            force_curve_is_finished = True
            return force_curve_is_finished
        else:
            # Define switching points: force -> cam angle where mode switches
            switch_point = min(float(self.switch_angle(target_force))-5, 70)
            # switch_point = 71

            if self.actuator.data.cam_angle < switch_point:
                # Mode 1: Cam angle control (below switch point)
                # Reset torque mode tracking if transitioning from torque mode
                if self._force_control_mode == "torque":
                    self._force_control_mode = "cam_angle"
                    # self._torque_mode_start_time = None
                
                # Convert target force to desired cam angle using existing spline
                target_cam_angle = min(self.actuator._force_to_CAM_angle(target_force), switch_point - 0.1)
                print(f"Target Force: {target_force:.2f} N, "
                    f"Switch Point: {switch_point:.2f} deg, "
                    f"Target Cam Angle: {target_cam_angle:.2f} deg, "
                    f"Current Cam Angle: {self.actuator.data.cam_angle:.2f} deg")
                
                await self.actuator.command_cam_angle(target_cam_angle, error_filter=self.cam_control_filter)
                # await self.actuator.command_actuator_velocity(0, max_torque=0.1)  # Placeholder command
                
                self._force_control_mode = "cam_angle"
                if self.actuator.data.cam_angle >= switch_point - 0.5:
                    switch_point = 5
                
            else:
                # Mode 2: Torque control (at or above switch point)
                if self._force_control_mode != "torque":
                    self._force_control_mode = "torque"
                    self._torque_mode_start_time = time.perf_counter()
                
                # Calculate time elapsed since entering torque mode
                time_in_torque_mode = time.perf_counter() - self._torque_mode_start_time
                
                # Define time-based torque ramp using pchip
                
                
                # Torque values: ramp from initial to final torque
                initial_torque = target_force * self.actuator.design_constants.ACTUATOR_RADIUS * 0.5
                peak_torque = target_force * self.actuator.design_constants.ACTUATOR_RADIUS * 1.0
                
                torque_values = [
                    initial_torque,                                          
                    peak_torque,                                                
                    peak_torque,                                              
                    initial_torque,                                                
                    initial_torque
                    ]

                time_points = [
                    0.0, 
                    self._torque_ramp_duration * 0.01,
                    self._torque_ramp_duration * 0.675,
                    self._torque_ramp_duration * 0.99,
                    self._torque_ramp_duration
                ]

                pchip_torque = PchipInterpolator(time_points, torque_values)
                
                # Interpolate torque based on time elapsed
                if time_in_torque_mode <= self._torque_ramp_duration:
                    desired_torque = float(pchip_torque(time_in_torque_mode))
                else:
                    # Complete, hold at the end torque
                    desired_torque = torque_values[-1]
                
                await self.actuator.command_actuator_torque(desired_torque)

                print(f"Target Force: {target_force:.2f} N, "
                    f"Time in Force Mode: {time_in_torque_mode:.2f} s, "
                    f"Commanded Force: {desired_torque/self.actuator.design_constants.ACTUATOR_RADIUS:.2f} N")

                force_curve_is_finished = time_in_torque_mode > self._torque_ramp_duration            
                return force_curve_is_finished


class ParameterParser(threading.Thread):
    """
    Thread to handle keyboard input for updating controller setpoints and gains.
    """
    def __init__(self,
                 controller: Controller = None,
                 lock: Type[threading.Lock] = threading.Lock(), 
                 quit_event: Type[threading.Event] = threading.Event(),
                 name='keyboard-input-thread'):
        super().__init__(name=name, group=None)
        self.controller = controller
        self.actuator = controller.actuator if controller else None
        self.quit_event = quit_event
        self.lock = lock
        self.daemon = True  # Ensures thread exits when the main program ends

    def run(self):
        self._print_help()

        while not self.quit_event.is_set():
            try:
                msg = input("Enter command: ").strip()
                if not msg:
                    continue
                
                command_type, value = self._parse_command(msg)

                if command_type == 'q':
                    self._handle_quit()
                    break
                elif command_type == 'r':
                    self._handle_reset_counter()
                    continue
                elif command_type in ('p', 'd'):
                    self._handle_gain_update(command_type, value)
                elif command_type in self.controller.command_map:
                    setpoint_type, name = self.controller.command_map[command_type]
                    self._update_setpoint(setpoint_type, value, name)
                else:
                    print(f"Invalid command '{command_type}'. Try: {', '.join(self.controller.command_map.keys())}")
                    continue

            except (EOFError, KeyboardInterrupt):
                print("\nEOFError caught. quit_event set from input thread.")
                self.quit_event.set()
                break

        print("Input thread terminated.")
    
    def _print_help(self):
        print(
            "Input options: \n"
            "a: actuator_angle, v: actuator_velocity, t: actuator_torque, \n"
            "c: cam_angle, f: cable_force, h: home_position, \n"
            "r: reset_pulse_counter, q: quit\n"
        )

    def _parse_command(self, input_str: str) -> tuple[str, float]:
        """Parse input into (command_type, value). Raises ValueError if invalid."""
        cmd_type = input_str[0].lower()
        cmd_value = input_str[1:].strip()
        
        try:
            value = float(cmd_value) if cmd_value else 0.0
        except ValueError:
            print("Invalid value. Please enter a valid number after the command letter.")
            print(f"Invalid value for '{cmd_type}': {cmd_value}")
            return None, None
        return cmd_type, value
    
    def _handle_quit(self):
        print("Quit command received.")
        with self.lock:
            self.quit_event.set()
    
    def _handle_reset_counter(self):
        print("Resetting pulse counter...")
        with self.lock:
            self.actuator.reset_pulse_counter()

    def _handle_gain_update(self, gain_type: str, value: float):
        with self.lock:
            if gain_type == 'p':
                self.actuator.update_camController_gains(kp_gain=value)
            elif gain_type == 'd':
                self.actuator.update_camController_gains(kd_gain=value)
        print(f"Gain '{gain_type}' updated to {value:.3f}")

    def _update_setpoint(self, setpoint_type: SetpointType, value: float, name: str):
        with self.lock:
            self.controller.update_controller_variables(setpoint_type=setpoint_type,
                                                        setpoint_value=value)
        print(f"Setpoint '{name}' set to {value}")
    
 