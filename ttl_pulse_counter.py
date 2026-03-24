"""
TTL Pulse Counter

A simple standalone script to count TTL pulses from a NI-DAQ device.
Based on the TTL reading implementation in ACTUATOR.py.

USAGE:
------
1. Ensure NI-DAQ hardware is connected
2. Update the DAQ_CHANNEL variable below if needed
3. Run: python ttl_pulse_counter.py
4. Press Ctrl+C to quit and display final statistics

The script will:
- Detect and count rising edges of TTL pulses
- Display pulse count and timestamps in real-time
- Show statistics on exit
"""

import nidaqmx
import time
import sys

# Configuration
DAQ_CHANNEL = "Dev1/port0/line1"  # Update this to match your DAQ channel
SAMPLE_RATE_HZ = 1000  # Sampling frequency in Hz

class TTLPulseCounter:
    def __init__(self, daq_channel: str, sample_rate: float = 1000):
        """
        Initialize TTL pulse counter.
        
        Args:
            daq_channel: DAQ channel name (e.g., "Dev1/port0/line1")
            sample_rate: Sampling rate in Hz (default: 1000)
        """
        self.daq_channel = daq_channel
        self.sample_rate = sample_rate
        self.daq_task = None
        
        # Pulse tracking
        self.pulse_count = 0
        self.last_ttl_state = False
        self.pulse_timestamps = []
        
        # Timing
        self.start_time = None
        self.last_sample_time = None
        
    def setup_daq(self):
        """Setup DAQ task for reading TTL signal."""
        try:
            self.daq_task = nidaqmx.Task()
            self.daq_task.di_channels.add_di_chan(self.daq_channel)
            print(f"✓ DAQ task setup successfully on channel: {self.daq_channel}")
            return True
        except Exception as e:
            print(f"✗ Error setting up DAQ: {e}")
            return False
    
    def read_ttl_signal(self):
        """
        Read TTL signal from DAQ and detect rising edges.
        
        Returns:
            bool: Current TTL state
        """
        if self.daq_task is None:
            return False
        
        try:
            current_ttl_state = self.daq_task.read()
            current_time = time.perf_counter()
            
            # Detect rising edge (transition from False to True)
            if current_ttl_state and not self.last_ttl_state:
                self.pulse_count += 1
                elapsed_time = current_time - self.start_time
                self.pulse_timestamps.append(elapsed_time)
                
                # Print pulse detection
                print(f"✓ PULSE #{self.pulse_count} detected at {elapsed_time:.3f}s")
                
            self.last_ttl_state = current_ttl_state
            self.last_sample_time = current_time
            
            return current_ttl_state
            
        except Exception as e:
            print(f"✗ Error reading TTL signal: {e}")
            return False
    
    def run(self):
        """
        Main loop to continuously read and count TTL pulses.
        Press Ctrl+C to stop.
        """
        if not self.setup_daq():
            print("Failed to setup DAQ. Exiting...")
            return
        
        print("\n" + "="*60)
        print("TTL PULSE COUNTER - RUNNING")
        print("="*60)
        print(f"DAQ Channel: {self.daq_channel}")
        print(f"Sample Rate: {self.sample_rate} Hz")
        print(f"Sample Period: {1000/self.sample_rate:.2f} ms")
        print("\nPress Ctrl+C to stop and view statistics")
        print("="*60 + "\n")
        
        self.start_time = time.perf_counter()
        last_status_time = self.start_time
        
        try:
            while True:
                # Read TTL signal
                self.read_ttl_signal()
                
                # Print periodic status (every 5 seconds)
                current_time = time.perf_counter()
                if current_time - last_status_time >= 5.0:
                    elapsed = current_time - self.start_time
                    print(f"[Status] Running for {elapsed:.1f}s | Total pulses: {self.pulse_count}")
                    last_status_time = current_time
                
                # Control sampling rate
                time.sleep(1.0 / self.sample_rate)
                
        except KeyboardInterrupt:
            print("\n\n" + "="*60)
            print("STOPPING TTL PULSE COUNTER")
            print("="*60)
            self._print_statistics()
        
        finally:
            self.cleanup()
    
    def _print_statistics(self):
        """Print statistics about detected pulses."""
        total_time = time.perf_counter() - self.start_time
        
        print(f"\nTotal Runtime: {total_time:.2f} seconds")
        print(f"Total Pulses Detected: {self.pulse_count}")
        
        if self.pulse_count > 0:
            avg_rate = self.pulse_count / total_time
            print(f"Average Pulse Rate: {avg_rate:.2f} pulses/second")
            
            if self.pulse_count > 1:
                # Calculate inter-pulse intervals
                intervals = [self.pulse_timestamps[i] - self.pulse_timestamps[i-1] 
                           for i in range(1, len(self.pulse_timestamps))]
                avg_interval = sum(intervals) / len(intervals)
                min_interval = min(intervals)
                max_interval = max(intervals)
                
                print(f"\nInter-Pulse Interval Statistics:")
                print(f"  Average: {avg_interval:.3f}s ({1/avg_interval:.2f} Hz)")
                print(f"  Minimum: {min_interval:.3f}s ({1/min_interval:.2f} Hz)")
                print(f"  Maximum: {max_interval:.3f}s ({1/max_interval:.2f} Hz)")
            
            print(f"\nPulse Timestamps (seconds from start):")
            for i, timestamp in enumerate(self.pulse_timestamps, 1):
                print(f"  Pulse #{i}: {timestamp:.3f}s")
        else:
            print("No pulses detected during runtime.")
        
        print("="*60 + "\n")
    
    def cleanup(self):
        """Close DAQ task and cleanup resources."""
        if self.daq_task is not None:
            try:
                self.daq_task.close()
                print("✓ DAQ task closed successfully")
            except Exception as e:
                print(f"✗ Error closing DAQ task: {e}")


def main():
    """Main entry point."""
    # Create and run the pulse counter
    counter = TTLPulseCounter(
        daq_channel=DAQ_CHANNEL,
        sample_rate=SAMPLE_RATE_HZ
    )
    counter.run()


if __name__ == "__main__":
    main()
