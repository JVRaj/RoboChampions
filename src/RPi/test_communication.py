#!/usr/bin/env python3
"""
Test script for RPI-ESP32 communication
Tests command sending and sensor data reception
"""

import serial
import time
import threading

class CommunicationTester:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.running = True
            print(f"Connected to {port} at {baudrate} baud")
            time.sleep(2)  # Let ESP32 initialize
        except Exception as e:
            print(f"Failed to connect: {e}")
            exit(1)
    
    def wait_for_ok(self):
        """Wait for ESP32 handshake"""
        print("Waiting for ESP32 handshake...")
        while True:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"ESP32: {line}")
                if line.upper() == "OK":
                    print("Sending back OK...")
                    self.ser.write(b"OK\n")
                    print("Handshake complete!")
                    break
    
    def send_command(self, cmd_type, *args):
        """Send structured command to ESP32"""
        try:
            if cmd_type == "MOVE":
                speed, direction, steer = args
                cmd = f"CMD:MOVE,{speed},{direction},{steer}\n"
            elif cmd_type == "STOP":
                cmd = "CMD:STOP\n"
            elif cmd_type == "AUTONOMOUS":
                cmd = "CMD:AUTONOMOUS\n"
            else:
                print(f"Unknown command: {cmd_type}")
                return
            
            self.ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")
        except Exception as e:
            print(f"Send error: {e}")
    
    def read_sensor_data(self):
        """Read and parse sensor data"""
        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return None
            
            # Try to parse CSV format: angle,left,right,front
            parts = line.split(",")
            if len(parts) == 4:
                try:
                    angle = int(parts[0])
                    left = int(parts[1])
                    right = int(parts[2])
                    front = int(parts[3])
                    return {
                        'angle': angle,
                        'left': left,
                        'right': right,
                        'front': front,
                        'raw': line
                    }
                except ValueError:
                    pass
            
            # If not CSV, print as status/debug message
            print(f"ESP32: {line}")
            return None
            
        except Exception as e:
            print(f"Read error: {e}")
            return None
    
    def sensor_monitor(self):
        """Monitor sensor data in background thread"""
        print("Starting sensor monitor...")
        sensor_count = 0
        
        while self.running:
            data = self.read_sensor_data()
            if data:
                sensor_count += 1
                if sensor_count % 10 == 0:  # Print every 10th reading
                    print(f"Sensors: Angle={data['angle']}, L={data['left']}, R={data['right']}, F={data['front']}")
            time.sleep(0.01)
    
    def test_commands(self):
        """Test various commands"""
        print("\n=== Testing Commands ===")
        
        # Test basic movement
        print("Testing forward movement...")
        self.send_command("MOVE", 100, 0, 0)
        time.sleep(2)
        
        print("Testing left turn...")
        self.send_command("MOVE", 80, 0, -30)
        time.sleep(1)
        
        print("Testing right turn...")
        self.send_command("MOVE", 80, 0, 30)
        time.sleep(1)
        
        print("Testing reverse...")
        self.send_command("MOVE", 80, 1, 0)
        time.sleep(1)
        
        print("Testing stop...")
        self.send_command("STOP")
        time.sleep(1)
        
        print("Testing autonomous mode...")
        self.send_command("AUTONOMOUS")
        time.sleep(3)
        
        print("Returning to remote mode...")
        self.send_command("MOVE", 0, 0, 0)
        time.sleep(1)
        
        print("Final stop...")
        self.send_command("STOP")
        
        print("Command test complete!")
    
    def interactive_mode(self):
        """Interactive command mode"""
        print("\n=== Interactive Mode ===")
        print("Commands:")
        print("  f [speed] - Move forward")
        print("  b [speed] - Move backward") 
        print("  l [steer] - Turn left")
        print("  r [steer] - Turn right")
        print("  s - Stop")
        print("  a - Autonomous mode")
        print("  q - Quit")
        print()
        
        while self.running:
            try:
                cmd = input("Command: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 's':
                    self.send_command("STOP")
                elif cmd == 'a':
                    self.send_command("AUTONOMOUS")
                elif cmd.startswith('f'):
                    parts = cmd.split()
                    speed = int(parts[1]) if len(parts) > 1 else 100
                    self.send_command("MOVE", speed, 0, 0)
                elif cmd.startswith('b'):
                    parts = cmd.split()
                    speed = int(parts[1]) if len(parts) > 1 else 100
                    self.send_command("MOVE", speed, 1, 0)
                elif cmd.startswith('l'):
                    parts = cmd.split()
                    steer = int(parts[1]) if len(parts) > 1 else -30
                    self.send_command("MOVE", 80, 0, steer)
                elif cmd.startswith('r'):
                    parts = cmd.split()
                    steer = int(parts[1]) if len(parts) > 1 else 30
                    self.send_command("MOVE", 80, 0, steer)
                else:
                    print("Unknown command")
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Input error: {e}")
    
    def run_test(self):
        """Run full test sequence"""
        try:
            # Wait for handshake
            self.wait_for_ok()
            
            # Start sensor monitoring in background
            monitor_thread = threading.Thread(target=self.sensor_monitor, daemon=True)
            monitor_thread.start()
            
            print("\nWaiting 2 seconds for sensor data...")
            time.sleep(2)
            
            # Test commands
            self.test_commands()
            
            # Interactive mode
            self.interactive_mode()
            
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\nCleaning up...")
        self.running = False
        self.send_command("STOP")
        time.sleep(0.5)
        if hasattr(self, 'ser'):
            self.ser.close()
        print("Test complete!")

if __name__ == "__main__":
    import sys
    
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    
    print("ESP32-RPI Communication Tester")
    print("================================")
    print(f"Connecting to: {port}")
    print()
    
    tester = CommunicationTester(port)
    tester.run_test()
