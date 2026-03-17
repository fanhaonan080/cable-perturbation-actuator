"""
API with 
instantaneous data writing,
TTL DAQ integration,
force control,
new initialization

Main methods: 
connect_to_actuator() function to initialize and return
SpringActuator_moteus() class with methods: 
- initial_calibration()
- read_data()
- write_data()
- command_actuator_velocity()
- command_actuator_torque()
- command_cam_angle()
- command_controller_off()  stops the motor
- close()

Additional methods:
- update_camController_gains()
- command_relative_actuator_angle()
- force_control()


"""
import filters
import asyncio
import os
from scipy.interpolate import pchip_interpolate
import numpy as np
import math
import traceback
from dataclasses import dataclass
import time
import csv
import moteus
from moteus import multiplex as moteusMp
from enum import Enum
import nidaqmx
from scipy.interpolate import PchipInterpolator

class ControllerConfig:
    '''
    Tunable Constants used for control loops
    '''
    control_loop_freq = 200             # hertz
    scaling_factor = 0.3543984634257938  # df(20) -1.60054871969536
    camControllerGainKp = 250*scaling_factor           # correspondes to 88.59961585644845       
    camControllerGainKd = 0.24*scaling_factor          # correspondes to 0.08505563122219051
    
    feedforward_force = 0 
    disturbance_rejector_gain = 1
    kp_scale=1
    kd_scale=2
    ilimit=0

    calibrationVelocity = 60            # deg / sec (winch speed during calibration)
    homeAngle = 5                       # deg
    calibrationTime = 5                 # sec

    actuatorVelocitySaturation = 1850   # deg / sec (winch velocity limit)
    actuatorTorqueSaturation = 5        # Nm 

    cable_left_cali = 0.0               # m

class Constants:
    '''
    Constants dependent on actuator and general constants for conversion
    '''
    MAX_ALLOWABLE_CURRENT = 5                                   # Ampere
    MOTOR_TO_ACTUATOR_TR = 8
    MOTOR_POS_EST_TO_ACTUATOR_DEG = 360 / MOTOR_TO_ACTUATOR_TR  # Deg / Revolution / Transmission ratio
    CAM_ENC_TO_DEG = 360                                        # Deg / Revolution
    CAM_ANG_TO_CABLE_LEN_POLYNOMIAL = [-1.58431437e-12, 4.34233905e-10, -4.74033359e-08, 2.67675145e-06, -8.37020178e-05, 1.46224327e-03, -1.66912552e-02, -1.41612564e+00, 1.53916188e+02]
    CAM_ANG_TO_CABLE_LEN_POLYNOMIAL_DOT = [-2.50135485e-15, -8.66575879e-12,  2.06894486e-09, -1.83708046e-07, 7.95250499e-06, -1.75882870e-04,  1.95926578e-03, -1.64926789e-02, -1.45312981e+00]
    SPLINE_A_PTS_FORCE_ANGLE_CONVERSION = [0,10,20,40,70,80,85]
    SPLINE_F_PTS_FORCE_ANGLE_CONVERSION = [2.15,2.2,2.275,2.4,6.5,9.5,11]

class DesignConstants:
    'Actuator Design Constants, with measurements in mm and degrees with reference at CAM center'
    ACTUATOR_RADIUS = 0.0325  
    ROLLER_RADIUS = 0.0045
    CAM_LEVER_ARM = 0.055
    ROLLER_A_CORD = (-36.14,61.24)
    ROLLER_B_CORD = (-23.66,18)
    ROLLER_C_INIT_ANG = 59
    CAM_RANGE = 70   # was 75
    INIT_CABLE_LEN_BW_ANKLE_ACT = 0.0945
    CAM_DISENGAGE_FORCE_VAL = 6

#DATA LOGGING AND STORING METHODS
@dataclass
class ActuatorDataContainer:
    '''Reserving space for instantaneous data.'''
    sys_time: float = None
    loop_time: float = None
    mc_fault: int = 0
    mc_mode: int = 0 
    mc_power: float = 0
    mc_current: int = 0
    mc_command_velocity: float = 0
    actuator_angle: float = 0
    actuator_velocity: float = 0
    actuator_torque: float = 0
    cam_encoder_raw: int = 0
    cam_angle: float = 0
    cam_velocity: float = 0
    commanded_actuator_angle: float = None
    commanded_actuator_velocity: float = None
    commanded_actuator_torque: float = None
    commanded_cable_force: float = None
    commanded_cam_angle: float = None
    commanded_cable_length: float = None
    disturbance_displacement: float = 0
    disturbance_velocity: float = 0
    disturbance_acceleration: float = 0
    cam_angle_error: float = 0

    # Extra logged values for debugging
    measured_disturbance_velocity: float = None
    measured_disturbance_displacement: float = None
    measured_force: float = None

    mc_command_current: float = None
    mc_command_feedforward_torque: float = None
    mc_control_torque: float = None
    mc_torque_error: float = None

    # TTL and mode tracking
    ttl_signal: bool = False
    actuator_mode: str = "transparent" # "torque" when ttl_signal is True
    ttl_triggered: bool = False

class SpringActuator_moteus:
# ACTUATOR INITIALIZATION AND DATA FUNCTIONS
    def __init__(self, dataFile_name: str, daq_channel: str = None):
        self.motor_ctrl = moteus.Controller()
        # Data containers and constants
        self.data = ActuatorDataContainer()
        self.constants = Constants()
        self.design_constants = DesignConstants()
        self.config = ControllerConfig()

        # Calibration and offset variables
        self.actuator_offset = 0                 # post-calibration zero reference actuator angle
        self.cam_offset = 0                      # post-calibration zero reference cam angle
        self.cam_calibrate_offset = 0
        self.has_calibrated = False

        # Cable output estimation helpers and filters
        self.func_camAng_to_cableLen = np.poly1d(self.constants.CAM_ANG_TO_CABLE_LEN_POLYNOMIAL)
        self.func_camAng_to_cableLen_dot = np.poly1d(self.constants.CAM_ANG_TO_CABLE_LEN_POLYNOMIAL_DOT)
        self.dist_velocity_filter = filters.Butterworth(N=2, Wn=30, fs=self.config.control_loop_freq)
        self.dist_acceleration_filter = filters.Butterworth(N=2, Wn=50, fs=self.config.control_loop_freq)

        # Cam angle filters
        self.cam_angle_filter = filters.Butterworth(N=2, Wn=98, fs=self.config.control_loop_freq)
        self.cam_angle_control_filter = filters.Butterworth(N=2, Wn=98, fs=self.config.control_loop_freq)
        
        
        self.dataFile_name = dataFile_name
        self.setup_data_writer(dataFile_name)
        self.motor_sign = 1                         # motor_serial to be used to update motor_sign if required   

        # DAQ Channel for TTL signal
        self.daq_channel = daq_channel
        self.daq_task = None
        if self.daq_channel is not None:
            self._setup_daq()
        
        # Force control mode tracking for time-based torque ramping
        self._force_control_mode = None  # "cam_angle" or "torque"
        self._torque_mode_start_time = None  # Time when torque mode was entered
        self._torque_ramp_duration = 1.0  # Default ramp duration in seconds
    
    def _setup_daq(self):
        '''Setup DAQ task for reading TTL signal'''
        try:
            self.daq_task = nidaqmx.Task()
            self.daq_task.di_channels.add_di_chan(self.daq_channel)
            print(f"DAQ initialized on channel {self.daq_channel}")
        except Exception as e:
            print(f"Warning: Failed to initialize DAQ: {e}")
            self.daq_task = None

    async def read_ttl_signal(self):
        '''Read TTL signal from DAQ and update actuator mode'''
        if self.daq_task is not None:
            try:
                prev_ttl = self.data.ttl_signal
                # Offload blocking DAQ read to thread to avoid blocking event loop
                self.data.ttl_signal = await asyncio.to_thread(self.daq_task.read)

                if self.data.ttl_signal and not prev_ttl:
                    self.data.ttl_triggered = True
                    print("TTL rising edge detected.")

                # Update mode based on TTL signal
                if self.data.ttl_signal:
                    self.data.actuator_mode = "torque"
                else:
                    self.data.actuator_mode = "transparent"
                    
            except Exception as e:
                print(f"Error reading TTL signal: {e}")
        # return self.ttl_signal

    async def read_data(self, loop_time=None):
        '''Read data from Moteus, store in Data Container.'''
        to_query = {
            # Core states
            moteus.Register.MODE: moteusMp.INT8,
            moteus.Register.POSITION: moteusMp.F32,
            moteus.Register.VELOCITY: moteusMp.F32,
            moteus.Register.TORQUE: moteusMp.F32,
            moteus.Register.Q_CURRENT: moteusMp.F32,
            moteus.Register.FAULT: moteusMp.INT8,
            moteus.Register.POWER: moteusMp.INT32,            

            # Encoder 2 (Cam) states
            moteus.Register.ENCODER_2_POSITION: moteusMp.F32,
            moteus.Register.ENCODER_2_VELOCITY: moteusMp.F32,
            
            # Debug signals
            moteus.Register.COMMAND_VELOCITY: moteus.F32,
            moteus.Register.COMMAND_FEEDFORWARD_TORQUE: moteusMp.F32,
            # moteus.Register.COMMAND_Q_CURRENT: moteusMp.F32,
            moteus.Register.CONTROL_TORQUE: moteusMp.F32,
            moteus.Register.TORQUE_ERROR: moteusMp.F32
            }

        last_dist_vel = self.data.disturbance_velocity
        last_cam_encoder_raw = self.data.cam_encoder_raw if self.data.cam_encoder_raw is not None else None
        data_feed = await self.motor_ctrl.custom_query(to_query)
        mc_data = data_feed.values
        # print("Data read from moteus:", mc_data)
        self.data.loop_time = loop_time if loop_time!=None else self.data.loop_time
        self.data.sys_time = time.perf_counter()
        # print("read_data called")
        # self.data.mc_command_current = mc_data[moteus.Register.COMMAND_Q_CURRENT]
        self.data.mc_command_feedforward_torque = mc_data[moteus.Register.COMMAND_FEEDFORWARD_TORQUE]
        self.data.mc_control_torque = mc_data[moteus.Register.CONTROL_TORQUE]
        self.data.mc_torque_error = mc_data[moteus.Register.TORQUE_ERROR]

        
        self.data.mc_current = mc_data[moteus.Register.Q_CURRENT]
        self.data.mc_power = mc_data[moteus.Register.POWER]
        self.data.mc_fault = mc_data[moteus.Register.FAULT]
        self.data.mc_mode = mc_data[moteus.Register.MODE]
        self.data.actuator_angle = mc_data[moteus.Register.POSITION] * self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        self.data.actuator_velocity = mc_data[moteus.Register.VELOCITY] * self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        self.data.actuator_torque = mc_data[moteus.Register.TORQUE] * self.constants.MOTOR_TO_ACTUATOR_TR
        self.data.mc_command_velocity = mc_data[moteus.Register.COMMAND_VELOCITY]
        
        # CAM Angle
        self.data.cam_encoder_raw = mc_data[moteus.Register.ENCODER_2_POSITION] * self.constants.CAM_ENC_TO_DEG
        self.data.cam_velocity = -1*mc_data[moteus.Register.ENCODER_2_VELOCITY] * self.constants.CAM_ENC_TO_DEG
        
        cam_angle_jump = (self.data.cam_encoder_raw - last_cam_encoder_raw) if last_cam_encoder_raw is not None else 0
        if cam_angle_jump > 180:  # jump of pi in degrees (from 0 to 2pi as angle increases), though shouldn't be an issue as per current configuration 
            cam_angle_wrapped = self.data.cam_encoder_raw - 360.0
        else:
            cam_angle_wrapped = self.data.cam_encoder_raw
        self.data.cam_angle = (-1*(cam_angle_wrapped - self.cam_offset)) if self.cam_offset is not None else None
        # self.data.cam_angle = (
        #     self.cam_angle_filter.filter(-1*(cam_angle_wrapped - self.cam_offset))
        #     if self.cam_offset is not None else None
        # )
        
        self.data.disturbance_displacement = self._disturbance_observer_displacement() if self.has_calibrated else None
        self.data.disturbance_velocity = self._disturbance_observer() if self.has_calibrated else 0
        self.data.disturbance_acceleration = self.dist_acceleration_filter.filter(self.data.disturbance_velocity - last_dist_vel)
        
        self.data.commanded_actuator_torque = None
        self.data.commanded_actuator_angle = None
        self.data.commanded_actuator_velocity = None
        self.data.commanded_cam_angle = None      
        # print("Reading TTL signal...")
        # Read TTL signal and update mode
        await self.read_ttl_signal()
        # self.data.ttl_signal = self.ttl_signal
        # self.data.actuator_mode = self.actuator_mode

        
    def write_data(self):
        '''Writes data file, only if new data packet is available'''
        if self.dataFile_name is not None:
            self.writer.writerow(self.data.__dict__)

    def setup_data_writer(self, file_ID: str):
        if file_ID is not None:
            '''file_ID is used as a custom file identifier after date.'''
            subfolder_name = 'exo_data/'
            os.makedirs(subfolder_name, exist_ok=True)
            self.filename = subfolder_name + time.strftime("%Y%m%d_%H%M_") + file_ID + '.csv'
            with open('curr_datafile_name.txt', 'w') as f:
                f.write(self.filename)
            self.my_file = open(self.filename, 'w', newline='')
            self.writer = csv.DictWriter(
                self.my_file, fieldnames=self.data.__dict__.keys())
            self.writer.writeheader()


# COMMANDING CONTROLLER FUNCTIONS
    def update_camController_gains(self, 
                                   kp_gain = None, 
                                   kd_gain = None, 
                                   kp_scale = None, 
                                   kd_scale = None, 
                                   dist_gain = None):
        '''
        updates gains for cam angle controller and general motor controller
        '''   
        if kp_gain != None: self.config.camControllerGainKp = kp_gain
        if kd_gain != None: self.config.camControllerGainKd = kd_gain
        if kp_scale != None: self.config.kp_scale = kp_scale
        if kd_scale != None: self.config.kd_scale = kd_scale
        if dist_gain != None: self.config.disturbance_rejector_gain = dist_gain


    async def command_relative_actuator_angle(self, 
                                              des_rel_angle: float, 
                                              reference_angle: float):
        '''
        des_rel_angle: Takes in angle (degrees) to rotate actuator from current position
        '''        
        desired_motor_pos = (des_rel_angle + reference_angle) / self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        await self.motor_ctrl.set_position(position=desired_motor_pos, query = False)

    
    async def command_actuator_velocity(self, 
                                        des_velocity: float,
                                        f_torque: float = None,
                                        max_torque: float = None):
        '''
        des_velocity: Takes in velocity command in degrees/seconds
        '''
        # Velocity Saturation
        des_velocity = max(min(des_velocity, self.config.actuatorVelocitySaturation), -self.config.actuatorVelocitySaturation)
        # des_velocity = min(des_velocity, self.config.actuatorVelocitySaturation) if des_velocity>0 else max(des_velocity, -self.config.actuatorVelocitySaturation)
        self.data.commanded_actuator_velocity = des_velocity    
        desired_motor_vel = des_velocity / self.constants.MOTOR_POS_EST_TO_ACTUATOR_DEG
        # max_torque = self.config.actuatorTorqueSaturation if max_torque is None else max_torque
        if max_torque is not None:
            max_torque = max(min(max_torque, self.config.actuatorTorqueSaturation), -self.config.actuatorTorqueSaturation)
            max_motor_torque = max_torque / self.constants.MOTOR_TO_ACTUATOR_TR
        else:
            max_motor_torque = None

        if f_torque!=None:
            await self.motor_ctrl.set_position(position = math.nan,
                                               velocity = desired_motor_vel,
                                               feedforward_torque = f_torque,
                                               maximum_torque = max_motor_torque,
                                               kp_scale = self.config.kp_scale,
                                               kd_scale = self.config.kd_scale,
                                               ilimit_scale = self.config.ilimit)
        else:
            await self.motor_ctrl.set_position(position = math.nan,
                                               velocity = desired_motor_vel,
                                               maximum_torque = max_motor_torque,
                                               kp_scale = self.config.kp_scale,
                                               kd_scale = self.config.kd_scale,
                                               ilimit_scale = self.config.ilimit)
    
    async def command_actuator_torque(self, des_torque: float):
        '''
        des_torque: Takes in torque command in Nm for actuator
        '''
        # Torque Saturation
        des_torque = max(min(des_torque, self.config.actuatorTorqueSaturation), -self.config.actuatorTorqueSaturation)
        self.data.commanded_actuator_torque = des_torque
        desired_motor_torque = des_torque / self.constants.MOTOR_TO_ACTUATOR_TR
        await self.motor_ctrl.set_position(position = math.nan, 
                                           kp_scale = 0.0, 
                                           kd_scale = 0.0, 
                                           feedforward_torque = desired_motor_torque, 
                                           ilimit_scale = 0)

    async def command_cam_angle(self, des_angle: float, error_filter: filters.Filter):  
        '''
        des_angle: takes in cam angle in degrees
        '''
        if error_filter is None:
            error_filter = self.cam_angle_control_filter
        des_angle = max(min(des_angle, self.design_constants.CAM_RANGE), 5)  # CAM_RANGE is the maximum angle of the cam, 5 is the minimum angle to avoid singularity
        self.data.commanded_cam_angle = des_angle
        prev_cam_ang_err = self.data.cam_angle_error
        curr_cam_ang_err = des_angle - self.data.cam_angle
        curr_cam_ang_err_diff = error_filter.filter((curr_cam_ang_err - prev_cam_ang_err) * self.config.control_loop_freq)
        
        k = (-self.func_camAng_to_cableLen_dot(self.data.cam_angle)/(self.design_constants.ACTUATOR_RADIUS*1000 * math.pi / 180))
        des_act_vel = k* (self.config.camControllerGainKp * curr_cam_ang_err + self.config.camControllerGainKd * curr_cam_ang_err_diff)
        
        disturbance_vel = self.data.disturbance_velocity #+ self.data.disturbance_acceleration
        dist_compensation = -1*self.config.disturbance_rejector_gain * (((disturbance_vel) / (1000*self.design_constants.ACTUATOR_RADIUS)) * (180 / math.pi))
        des_act_vel += dist_compensation
        feedforward_torque = (self.config.feedforward_force * self.design_constants.ACTUATOR_RADIUS) / self.constants.MOTOR_TO_ACTUATOR_TR if self.config.feedforward_force!=0 else None
        self.data.cam_angle_error = curr_cam_ang_err
        await self.command_actuator_velocity(des_velocity=des_act_vel, f_torque=feedforward_torque)
    
    async def force_control(self, target_force, ramp_duration: float = None):
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
        str : "cam_angle" or "torque" indicating which control mode is active
        '''
        
        if ramp_duration is not None:
            self._torque_ramp_duration = ramp_duration
        
        # Define switching points: force -> cam angle where mode switches
        switching_ang_pts = [69, 70, 71.8, 72.25, 73.5, 74, 75, 75.75, 76.5, 78]
        force_pts = [6, 15, 25, 35, 50, 65, 80, 100, 120, 200]
        pchip_switch = PchipInterpolator(force_pts, switching_ang_pts)
        switch_point = float(pchip_switch(target_force))-2
        
        if self.data.cam_angle < switch_point:
            # Mode 1: Cam angle control (below switch point)
            # Reset torque mode tracking if transitioning from torque mode
            if self._force_control_mode == "torque":
                self._force_control_mode = "cam_angle"
                self._torque_mode_start_time = None
            
            # Convert target force to desired cam angle using existing spline
            target_cam_angle = min(self._force_to_CAM_angle(target_force),switch_point - 0.1)
            # print(f"Target Force: {target_force:.2f} N, Switch Point: {switch_point:.2f} deg, Target Cam Angle: {target_cam_angle:.2f} deg")
            await self.command_cam_angle(target_cam_angle, error_filter=self.cam_angle_control_filter)
            self._force_control_mode = "cam_angle"
            # return "cam_angle"
            
        else:
            # Mode 2: Torque control (at or above switch point)
            # Track entry into torque mode
            if self._force_control_mode != "torque":
                self._force_control_mode = "torque"
                self._torque_mode_start_time = time.perf_counter()
            
            # Calculate time elapsed since entering torque mode
            time_in_torque_mode = time.perf_counter() - self._torque_mode_start_time
            
            # Define time-based torque ramp using pchip
            # Time points from 0 to ramp_duration
            time_points = [0.0, self._torque_ramp_duration * 0.25, 
                          self._torque_ramp_duration * 0.5, 
                          self._torque_ramp_duration * 0.75, 
                          self._torque_ramp_duration]
            
            # Torque values: ramp from initial to final torque
            initial_torque = target_force * self.design_constants.ACTUATOR_RADIUS * 0.5
            final_torque = target_force * self.design_constants.ACTUATOR_RADIUS * 1.0
            
            torque_values = [
                initial_torque,                                              # t=0.0
                initial_torque + (final_torque - initial_torque) * 0.15,    # t=25%
                initial_torque + (final_torque - initial_torque) * 0.45,    # t=50%
                initial_torque + (final_torque - initial_torque) * 0.75,    # t=75%
                final_torque                                                 # t=100%
            ]
            
            pchip_torque = PchipInterpolator(time_points, torque_values)
            
            # Interpolate torque based on time elapsed
            if time_in_torque_mode <= self._torque_ramp_duration:
                # Still ramping up
                desired_torque = float(pchip_torque(time_in_torque_mode))
            else:
                # Ramp complete, hold at final torque
                desired_torque = final_torque
            
            await self.command_actuator_torque(desired_torque)
            # return "torque"


    async def command_controller_off(self):
        '''Commands motor to stop and turn off controller'''
        await self.motor_ctrl.set_stop() 

# MECHANISM INITIAL CALIBRATION
    async def initial_calibration(self):
        '''
        Calibration routine for initializing the actuator, from either slacked or taut condition drives the cam angle to ~5 degrees
        Working: Slack out cable until cam angle stops changing and then taut in the cable till it reaches 5 degrees
        '''

        try:
            print('Calibrating...')
            stability_window_size=20
            stability_window = []
            for i in range(int(self.config.calibrationTime * self.config.control_loop_freq)):
                print('Calibrating cam zero position: Spooling out cable...')
                await self.command_actuator_velocity(-1 * self.motor_sign * self.config.calibrationVelocity)
                print("commanding velocity")
                await self.read_data()
                print(f"Current cam encoder raw value: {self.data.cam_encoder_raw}")
                stability_window.append(self.data.cam_encoder_raw)
                if len(stability_window) > stability_window_size:
                    stability_window.pop(0)
                if i>stability_window_size and abs((max(stability_window) - min(stability_window))) < 0.5:
                    self.cam_offset = self.data.cam_encoder_raw
                    break
                time.sleep(1/self.config.control_loop_freq)

            await self.command_actuator_velocity(0)

            for _ in range(int(self.config.calibrationTime * self.config.control_loop_freq)):
                print('Calibrating cam zero position: Spooling in cable...')
                await self.read_data()
                print(f"Current cam angle: {self.data.cam_angle}")
                await self.command_actuator_velocity(self.motor_sign * self.config.calibrationVelocity)
                if self.data.cam_angle > self.config.homeAngle:
                    break
                time.sleep(1/self.config.control_loop_freq)
            else:
                # Loop completed without break - calibration failed
                await self.command_actuator_velocity(0)
                raise RuntimeError(f'Calibration failed: Could not reach home angle {self.config.homeAngle} within {self.config.calibrationTime}s timeout. Current cam angle: {self.data.cam_angle}')
            
            await self.command_actuator_velocity(0)
            print("Holding position to finalize calibration...")
            for _ in range(int(0.5 * self.config.control_loop_freq)):
                await self.read_data()
                await self.command_cam_angle(self.config.homeAngle, error_filter=self.cam_angle_control_filter)
                time.sleep(1/self.config.control_loop_freq)
            
            await self.command_actuator_velocity(0)
            await self.read_data()
            if self.data.cam_angle_error < 0.5:
                self.has_calibrated = True
                self.actuator_offset = self.data.actuator_angle
                self.cam_calibrate_offset = self.data.cam_angle

            if not self.has_calibrated:
                raise RuntimeError('Calibration Timed Out!')
            
            print(f"CAM Angle: {self.data.cam_angle}, CAM Offset: {self.cam_offset}, Actuator Offset: {self.actuator_offset}")
            
            for _ in range(int(self.config.calibrationTime * self.config.control_loop_freq)):
                await self.read_data()
                self.config.cable_left_cali = self.data.disturbance_displacement
                print('Calibrated cable left = ',self.config.cable_left_cali)
                if abs(self.config.cable_left_cali) > 0:
                    break
                time.sleep(1/self.config.control_loop_freq)

            print('Finished Calibrating')

        except Exception as err:
            raise RuntimeError(err)

        finally:
            await self.command_controller_off()

    async def close(self):
        await self.command_controller_off()
        
        if self.dataFile_name is not None:
            self.my_file.close()
        # Close DAQ task if it exists
        if self.daq_task is not None:
            try:
                self.daq_task.close()
                print("DAQ task closed")
            except Exception as e:
                print(f"Error closing DAQ task: {e}")
        

# Helper functions for conversion across different measurements
    def _force_to_CAM_angle(self,cable_force):
        return pchip_interpolate(self.constants.SPLINE_F_PTS_FORCE_ANGLE_CONVERSION, 
                                 self.constants.SPLINE_A_PTS_FORCE_ANGLE_CONVERSION, 
                                 cable_force)

    def _disturbance_observer(self):
        disturbance_vel = -(self.func_camAng_to_cableLen_dot(self.data.cam_angle)*self.data.cam_velocity) - ((self.data.actuator_velocity*math.pi/180)*self.design_constants.ACTUATOR_RADIUS*1000)
        disturbance_vel = self.dist_velocity_filter.filter(disturbance_vel)
        return disturbance_vel
    
    def _disturbance_observer_displacement(self):
        ang = self.data.actuator_angle-self.actuator_offset
        offset_length = 0
        winch_cable_out = self.design_constants.ACTUATOR_RADIUS * 1000 * ang * math.pi / 180
        disturbance_pos = (self.func_camAng_to_cableLen(self.cam_calibrate_offset) 
                                         - self.func_camAng_to_cableLen(self.data.cam_angle)
                                         - winch_cable_out
                                         + offset_length)
        return disturbance_pos


async def connect_to_actuator(dataFile_name: str,
                              daq_channel: str = None):
    '''
    Connect to Actuator, instantiate Actuator object
    
    Parameters:
    -----------
    dataFile_name : str
        Name for the data file
    daq_channel : str, optional
        DAQ channel for TTL signal (e.g., "Dev1/port0/line0")
        If None, TTL functionality is disabled
    '''
    # Connection settings
    try:
        # moteus_controller = moteus.Controller()
        # await moteus_controller.set_stop()
        actuator = SpringActuator_moteus(dataFile_name, daq_channel)
        await actuator.command_controller_off()
    except Exception as err:
        traceback.print_exc()
        raise RuntimeError('Unable to connect to motor controller, Check Connection')

    print("Connected to Moteus")
    
    # actuator = SpringActuator_moteus(moteus_controller, dataFile_name)
    return actuator