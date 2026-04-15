"""
Minimal TTL Test Code
Tests digital input reading from NI DAQ for TTL signals
"""

import nidaqmx
import time

# Configuration
DAQ_CHANNEL = "Dev1/port0/line0"  # Adjust to your DAQ device and channel
SAMPLE_RATE = 100  # Hz
TEST_DURATION = 10  # seconds

def test_ttl_basic():
    """Basic TTL reading test - prints state continuously"""
    print(f"Testing TTL on channel: {DAQ_CHANNEL}")
    print(f"Reading for {TEST_DURATION} seconds...")
    print("Connect TTL signal to test. Press Ctrl+C to stop early.\n")
    
    try:
        with nidaqmx.Task() as task:
            # Add digital input channel
            task.di_channels.add_di_chan(DAQ_CHANNEL)
            
            start_time = time.time()
            prev_state = False
            edge_count = 0
            
            while (time.time() - start_time) < TEST_DURATION:
                # Read current state
                current_state = task.read()
                
                # Detect rising edge
                if current_state and not prev_state:
                    edge_count += 1
                    print(f"[{time.time() - start_time:.3f}s] Rising edge detected! (Count: {edge_count})")
                
                # Detect falling edge
                elif not current_state and prev_state:
                    print(f"[{time.time() - start_time:.3f}s] Falling edge detected.")
                
                prev_state = current_state
                time.sleep(1.0 / SAMPLE_RATE)
            
            print(f"\nTest complete. Total rising edges detected: {edge_count}")
    
    except nidaqmx.errors.DaqError as e:
        print(f"DAQ Error: {e}")
        print("\nTroubleshooting:")
        print("1. Check if DAQ device is connected")
        print("2. Verify channel name (use NI MAX to find correct channel)")
        print("3. Ensure no other program is using the DAQ")
    except KeyboardInterrupt:
        print("\n\nTest stopped by user.")


def test_ttl_state_monitor():
    """Continuous state monitoring - shows HIGH/LOW"""
    print(f"Monitoring TTL state on channel: {DAQ_CHANNEL}")
    print("Press Ctrl+C to stop.\n")
    
    try:
        with nidaqmx.Task() as task:
            task.di_channels.add_di_chan(DAQ_CHANNEL)
            
            while True:
                state = task.read()
                state_str = "HIGH" if state else "LOW"
                print(f"\rTTL State: {state_str}  ", end='', flush=True)
                time.sleep(0.05)
    
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped.")


if __name__ == "__main__":
    print("="*50)
    print("TTL Signal Test")
    print("="*50)
    print("\nSelect test mode:")
    print("1. Basic edge detection test (10 seconds)")
    print("2. Continuous state monitor")
    
    choice = input("\nEnter choice (1 or 2): ").strip()
    
    if choice == "1":
        test_ttl_basic()
    elif choice == "2":
        test_ttl_state_monitor()
    else:
        print("Invalid choice. Running basic test...")
        test_ttl_basic()
