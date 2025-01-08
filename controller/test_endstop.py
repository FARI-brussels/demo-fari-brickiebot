import serial
import time
from collections import deque
from threading import Thread, Event

class ArduinoCommunicator:
    def __init__(self, port='/dev/ttyACM0', baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.arduino = None
        self.buffer = deque(maxlen=3)  # Circular buffer for last 3 readings
        self.running = Event()
        self.last_status = 0
        
    def connect(self):
        try:
            self.arduino = serial.Serial(
                self.port,
                self.baud_rate,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(0.1)  # Wait for connection to establish
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
            return True
        except serial.SerialException as e:
            print(f"Connection error: {e}")
            return False
            
    def read_status(self):
        if not self.arduino or not self.arduino.is_open:
            return None
            
        try:
            # Look for start marker
            while self.arduino.in_waiting > 0:
                byte = self.arduino.read()
                if byte == b'\xff':  # Start marker
                    # Read status byte
                    status = self.arduino.read()
                    # Read end marker
                    end = self.arduino.read()
                    
                    if end == b'\xfe':  # Valid frame
                        return ord(status)
            return None
        except serial.SerialException:
            return None
            
    def status_monitoring_loop(self):
        while self.running.is_set():
            status = self.read_status()
            if status is not None:
                self.buffer.append(status)
                # Only update and print if status changed
                current_status = self.get_current_status()
                if current_status != self.last_status:
                    self.last_status = current_status
                    self._print_status(current_status)
            time.sleep(0.001)  # Small delay to prevent CPU hogging
            
    def get_current_status(self):
        """Get most common status from recent readings to filter noise"""
        if not self.buffer:
            return 0
        return max(set(self.buffer), key=self.buffer.count)
        
    def _print_status(self, status):
        if status & (1 << 0): print("X-axis limit switch triggered!")
        if status & (1 << 1): print("Y-axis limit switch triggered!")
        if status & (1 << 2): print("Z-axis limit switch triggered!")
        print(f"Raw status: {status}")
        
    def start(self):
        if self.connect():
            self.running.set()
            self.monitor_thread = Thread(target=self.status_monitoring_loop)
            self.monitor_thread.daemon = True
            self.monitor_thread.start()
            return True
        return False
        
    def stop(self):
        self.running.clear()
        if self.arduino:
            self.arduino.close()

def main():
    communicator = ArduinoCommunicator()
    
    try:
        if communicator.start():
            print("Monitoring started. Press Ctrl+C to exit.")
            while True:
                time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        communicator.stop()

if __name__ == "__main__":
    main()