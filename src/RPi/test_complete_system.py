#!/usr/bin/env python3
"""
Complete System Test for WRO Future Engineers Robot
Tests integration between RPI Controller and ESP32 Remote
"""

import time
import threading
import cv2
import json
from rpi_controller import RPiController

def test_camera_connection():
    """Test camera connectivity and basic frame capture"""
    print("Testing camera connection...")
    
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("❌ Camera not found or cannot be opened")
        return False
    
    ret, frame = cap.read()
    if not ret:
        print("❌ Cannot read frame from camera")
        cap.release()
        return False
    
    print(f"✅ Camera connected - Frame size: {frame.shape}")
    cap.release()
    return True

def test_calibration_file():
    """Test calibration file loading"""
    print("\nTesting calibration file...")
    
    try:
        with open("camera_calibration.json", 'r') as f:
            calibration = json.load(f)
        print("✅ Calibration file loaded successfully")
        
        # Check structure
        if "color_ranges" in calibration and "image_processing" in calibration:
            print("✅ Calibration structure is valid")
            return True
        else:
            print("⚠️ Calibration structure incomplete")
            return False
            
    except FileNotFoundError:
        print("⚠️ No calibration file found - will use defaults")
        return True
    except Exception as e:
        print(f"❌ Error loading calibration: {e}")
        return False

def test_serial_communication():
    """Test serial communication with ESP32"""
    print("\nTesting serial communication...")
    
    try:
        # Try to create controller instance
        controller = RPiController()
        
        # Test communication
        controller.send_command("STOP")
        time.sleep(0.5)
        
        # Check if we receive data
        sensor_data = controller.get_sensor_data()
        if sensor_data:
            print(f"✅ Serial communication working - Sensor data: {sensor_data}")
            return True
        else:
            print("⚠️ No sensor data received from ESP32")
            return False
            
    except Exception as e:
        print(f"❌ Serial communication failed: {e}")
        return False

def test_vision_processing():
    """Test vision processing pipeline"""
    print("\nTesting vision processing...")
    
    try:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ Cannot open camera for vision test")
            return False
        
        # Capture a frame
        ret, frame = cap.read()
        if not ret:
            print("❌ Cannot capture frame")
            cap.release()
            return False
        
        # Create controller for vision processing
        controller = RPiController()
        
        # Test block detection
        red_blocks, green_blocks = controller.detect_blocks(frame)
        
        print(f"✅ Vision processing working - Red blocks: {len(red_blocks)}, Green blocks: {len(green_blocks)}")
        cap.release()
        return True
        
    except Exception as e:
        print(f"❌ Vision processing failed: {e}")
        return False

def test_decision_logic():
    """Test decision-making logic"""
    print("\nTesting decision logic...")
    
    try:
        controller = RPiController()
        
        # Test with mock sensor data
        mock_data = {
            'angle': 45.0,
            'left': 25,
            'right': 30,
            'front': 50
        }
        
        # Test PID calculation
        steering = controller.calculate_pid_steering(mock_data['angle'])
        print(f"✅ PID steering calculation: {steering}")
        
        # Test reverse trigger
        controller.reverse_trigger.push(True)
        controller.reverse_trigger.push(True)
        controller.reverse_trigger.push(True)
        
        should_reverse = controller.reverse_trigger.should_reverse()
        print(f"✅ Reverse trigger logic: {should_reverse}")
        
        return True
        
    except Exception as e:
        print(f"❌ Decision logic failed: {e}")
        return False

def interactive_test():
    """Interactive test mode"""
    print("\n" + "="*50)
    print("INTERACTIVE TEST MODE")
    print("="*50)
    
    try:
        controller = RPiController()
        print("Controller created successfully")
        
        print("\nAvailable commands:")
        print("  'w' - Move forward")
        print("  's' - Stop")
        print("  'a' - Turn left")
        print("  'd' - Turn right")
        print("  'r' - Move backward")
        print("  'v' - Start vision mode")
        print("  'q' - Quit")
        
        while True:
            command = input("\nEnter command: ").lower().strip()
            
            if command == 'q':
                break
            elif command == 'w':
                controller.send_command("MOVE", 100, 0, 0)
                print("Moving forward")
            elif command == 's':
                controller.send_command("STOP")
                print("Stopped")
            elif command == 'a':
                controller.send_command("MOVE", 80, 0, -30)
                print("Turning left")
            elif command == 'd':
                controller.send_command("MOVE", 80, 0, 30)
                print("Turning right")
            elif command == 'r':
                controller.send_command("MOVE", 80, 1, 0)
                print("Moving backward")
            elif command == 'v':
                print("Starting vision mode for 10 seconds...")
                start_time = time.time()
                
                cap = cv2.VideoCapture(0)
                if cap.isOpened():
                    while time.time() - start_time < 10:
                        ret, frame = cap.read()
                        if ret:
                            red_blocks, green_blocks = controller.detect_blocks(frame)
                            sensor_data = controller.get_sensor_data()
                            
                            print(f"Blocks - Red: {len(red_blocks)}, Green: {len(green_blocks)}")
                            if sensor_data:
                                print(f"Sensors - Angle: {sensor_data.get('angle', 'N/A')}, "
                                      f"Distances: L={sensor_data.get('left', 'N/A')}, "
                                      f"R={sensor_data.get('right', 'N/A')}, "
                                      f"F={sensor_data.get('front', 'N/A')}")
                        
                        time.sleep(0.5)
                    cap.release()
                else:
                    print("Could not open camera")
            else:
                print("Unknown command")
        
        controller.send_command("STOP")
        print("Test completed")
        
    except Exception as e:
        print(f"Interactive test failed: {e}")

def main():
    """Main test function"""
    print("WRO Future Engineers Robot - Complete System Test")
    print("=" * 60)
    
    tests = [
        ("Camera Connection", test_camera_connection),
        ("Calibration File", test_calibration_file),
        ("Serial Communication", test_serial_communication),
        ("Vision Processing", test_vision_processing),
        ("Decision Logic", test_decision_logic)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n{'='*20} {test_name} {'='*20}")
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"❌ {test_name} failed with exception: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "="*60)
    print("TEST RESULTS SUMMARY")
    print("="*60)
    
    passed = 0
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{test_name:<25} {status}")
        if result:
            passed += 1
    
    print(f"\nOverall: {passed}/{len(results)} tests passed")
    
    if passed == len(results):
        print("🎉 All tests passed! System is ready for operation.")
        
        # Offer interactive test
        response = input("\nWould you like to run interactive test? (y/n): ")
        if response.lower() == 'y':
            interactive_test()
    else:
        print("⚠️ Some tests failed. Please check the issues above.")

if __name__ == "__main__":
    main()
