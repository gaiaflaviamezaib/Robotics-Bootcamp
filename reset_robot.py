#!/usr/bin/env python3
import time
import sys
import os

# Add parent directory to path to import hardware modules if run from subfolder
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from hardware import servo
from hardware import dc_motor
from hardware import leds

def reset_all():
    print("Resetting robot hardware...")

    # 1. Stop DC Motors
    print("- Stopping DC motors")
    try:
        dc_motor.setup()
        dc_motor.motorStop()
    except Exception as e:
        print(f"  Error stopping motors: {e}")

    # 2. Reset Servos (Wheels and Head)
    print("- Centering all servos (Wheels, Head Pan, Head Tilt)")
    try:
        sc = servo.ServoCtrl()
        sc.moveInit() # This automatically sets all servos (0, 1, 2) to their initial center positions
        time.sleep(1) # Give them time to physically move back
    except Exception as e:
        print(f"  Error resetting servos: {e}")

    # 3. Turn off LEDs
    print("- Turning off all LEDs")
    try:
        leds.switchSetup()
        leds.set_all_switch_off()
        import RPi.GPIO as GPIO
        GPIO.output(5, GPIO.LOW) # Turn off the top LED manually
    except Exception as e:
        print(f"  Error resetting LEDs: {e}")

    print("\nRobot is fully reset and ready!")

if __name__ == "__main__":
    reset_all()
