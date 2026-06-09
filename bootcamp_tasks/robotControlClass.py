from mpu6050 import mpu6050
import time
import numpy as np
import RPi.GPIO as GPIO
import math
import threading
import random as rd

import sys
import os
# On ajoute dynamiquement le dossier racine du projet au path
# (Comme ça ton ami n'a PAS besoin de hardcoder "/home/robot00/mu_code", ça s'adapte tout seul !)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from hardware import dc_motor as DC
from hardware import servo
from hardware import line_sensor as ts
from hardware import oled_display as OLED

#robot object is defined at the bottom of this file, so do from robotControlClass import robot in any other file to use robotControl class.

class robotControl:

    #region fixed vars
    #CUSTOM MAX ANGLES (based on measurement of real angles)
    MAX_ANGLE_WHEELS = 120
    minWheelsAngle = -120
    MAX_ANGLE_HEAD_HORIZ = 80
    minHeadHorizAngle = -80
    MAX_ANGLE_HEAD_VERT = 95
    minHeadVertAngle = -25
    
    #GPIO PINS
    TRIG_PIN = 11
    ECHO_PIN = 8 
    LEFT_INFRA_PIN = 19  
    MIDDLE_INFRA_PIN = 16
    RIGHT_INFRA_PIN = 20
    LED_LEFT_PIN = 6
    LED_RIGHT_PIN = 13
    LED_TOP_PIN = 5
    #endregion
    def __init__(self):
        """
            Function that runs initial logic at creation of robot object.
        """

        self.measureInitialized = False
        self.DCInitialized = False
        self.infraInitialized = False
        self.OLEDInitialized = False
        self.servosInitialized = False
        self.lightInitialized = False
        
        self.sensor = None
        
        self.measureLock = threading.Lock()
        self.measurement_active = False
        self.measureStartTime = 0
        self.measureEndTime = 0

###############################        
#############SETUP#############
    def _switchSetup(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.LED_TOP_PIN, GPIO.OUT)
        GPIO.setup(self.LED_LEFT_PIN, GPIO.OUT)
        GPIO.setup(self.LED_RIGHT_PIN, GPIO.OUT)
        
        self.lightInitialized = True
    
    def _DCSetup(self):
    
        DC.setup()
        self.DCInitialized = True

    def _servoSetup(self):
        
        self.sc = servo.ServoCtrl()
        self.servosInitialized = True

    def _infraSetup(self):
        
        ts.setup()
        self.infraInitialized = True

    def _measureSetup(self):
        """
            Runs automatically first time distanceMeasure() is called. Sets up initial measurement logic.
        """

        # Callback function triggered by Echo pin edges
        def echo_callback(channel):
            
            if not self.measurement_active:
                return
            
            # global start_time, end_time, distance_ready
            if GPIO.input(self.ECHO_PIN): # Rising edge: signal sent
                self.measureStartTime = time.perf_counter()
            else: # Falling edge: signal received back
                self.measureEndTime = time.perf_counter()
                self.measureDistanceReady = True

        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.TRIG_PIN, GPIO.OUT)
        GPIO.setup(self.ECHO_PIN, GPIO.IN)
        # Ensure the trigger pin starts LOW
        GPIO.output(self.TRIG_PIN, GPIO.LOW)
        time.sleep(0.1)
        # Attach interrupt on both rising and falling edges
        try:
            GPIO.remove_event_detect(self.ECHO_PIN)
        except Exception:
            pass
        GPIO.add_event_detect(self.ECHO_PIN, GPIO.BOTH, callback=echo_callback)

        self.measureDistanceReady = False
        
        self.measureInitialized = True

    def __getattr__(self, name): #setup OLED
        
        if name == "OLED":
            
            if not self.OLEDInitialized:
            # if not self.OLEDInitialized:
                oled = OLED.OLED_Display()
                
                object.__setattr__(self, name, oled)
                
                self.OLEDInitialized = True
                return oled

        else:
            raise AttributeError(f"No attribute {name} exists for {type(self).__name__}!")
#############SETUP#############
###############################        

    def leftLED(self, state):
        """
            turns left light 1: on, 0: off
        """
        
        if not self.lightInitialized:
            self._switchSetup()
            
        if state not in (0, 1):
            raise ValueError(f"State param should be 0 or 1! Gave: {state}")
        
        if state == 1:
            GPIO.output(self.LED_LEFT_PIN, GPIO.HIGH)
        else:
            GPIO.output(self.LED_LEFT_PIN, GPIO.LOW)
            
    def topLED(self, state):
        """
            turns top light 1: on, 0: off
        """
        
        if not self.lightInitialized:
            self._switchSetup()

        if state not in (0, 1):
            raise ValueError(f"State param should be 0 or 1! Gave: {state}")
        
        if state == 1:
            GPIO.output(self.LED_TOP_PIN, GPIO.HIGH)
        else:
            GPIO.output(self.LED_TOP_PIN, GPIO.LOW)
            
    def rightLED(self, state):
        """
            turns right light 1: on, 0: off
        """

        if not self.lightInitialized:
            self._switchSetup()

        if state not in (0, 1):
            raise ValueError(f"State param should be 0 or 1! Gave: {state}")
        
        if state == 1:
            GPIO.output(self.LED_RIGHT_PIN, GPIO.HIGH)
        else:
            GPIO.output(self.LED_RIGHT_PIN, GPIO.LOW)
            
    def measureDistance(self):
        """
            Returns the distance to the nearest object detected in front of the ultrasonic sensor.
            Returns none if no echo detected.
        """
        with self.measureLock:
            if not self.measureInitialized:
                self._measureSetup()
                #changes val of measureInitialized at end of initMeasureSetup
            
            self.measureDistanceReady = False
            self.measurement_active = True
            # Send a 15 us trigger pulse
            GPIO.output(self.TRIG_PIN, GPIO.HIGH)
            
            s = time.perf_counter()
            
            while time.perf_counter()-s < 0.000015: #more accurate time.sleep(0.000015)
                pass
            # time.sleep(0.000015)
            GPIO.output(self.TRIG_PIN, GPIO.LOW)
        
            # Wait for measurement result or timeout
            timeout = time.time() + 0.05 # 50 ms
            while not self.measureDistanceReady and time.time() < timeout:
                time.sleep(0.001)
            
            self.measurement_active = False
            
            if self.measureDistanceReady:
                self.measureDistanceReady = False
                duration = self.measureEndTime - self.measureStartTime
                
                if duration < 0 or duration > 0.05:
                    return None

            # Calculate distance (speed of sound = 343 m/s)
                distance_cm = (duration * 34300) / 2
                return round(distance_cm, 2)
            else:
                return None # Timeout or no echo detected

    def getInfraredState(self, printRes=True):
        """
            printRes: optional, prints infrared sensor status in console by default (True)
            returns the right, middle and left infrared sensor status (1 = on line, 0 = not on line)
        """
        
        if not self.infraInitialized:
            self._infraSetup()
            
        # Read the current state of each line sensor
        status_right = GPIO.input(self.RIGHT_INFRA_PIN)   # Read the right sensor's status
        status_middle = GPIO.input(self.MIDDLE_INFRA_PIN) # Read the middle sensor's status
        status_left = GPIO.input(self.LEFT_INFRA_PIN)     # Read the left sensor's status

        # Print the status of each sensor in a formatted string
        if printRes:
            print('LS: %d   MS: %d   RS: %d\n' % (status_left, status_middle, status_right))

        return [int(status_left), int(status_middle), int(status_right)] # 1 is on line (black), 0 is off line (white)

    def getAccelerometerData(self, printRes=True):
        """
            Returns pitch, roll, and acceleration (acc_x, acc_y, acc_z) from accelerometer.
        """

        if self.sensor is None:
            self.sensor = mpu6050(0x68)
            
        accel_x = 0
        accel_y = 0
        accel_z = 0
        for i in range(0,10):
            accelerometer_data = self.sensor.get_accel_data()
            accel_x +=  accelerometer_data['x'] # - 1.432
            accel_y +=  accelerometer_data['y'] + 0.200
            accel_z +=  accelerometer_data['z'] - 1.940

        if printRes:

            print('\nX=%.2f, Y=%.2f, Z=%.2f'%(accel_x/10, accel_y/10, accel_z/10))

        # Calculate roll and pitch angles
        roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180 / math.pi
        pitch = math.atan2(-accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180 / math.pi
        # yaw = math.atan2(math.sqrt(accel_x**2 + accel_y **2), -accel_z) * 180 /math.pi

        if printRes:
            print(f"\nRoll: {roll:.2f}, Pitch: {pitch:.2f}")#, Yaw: {yaw:.2f}")



        return roll, pitch, [accel_x, accel_y, accel_z]

    def resetAll(self):
        """
            Resets servos, sets motors to 0, and cleans up GPIO.
            Only to be used at the very end of any program.
        """
            
        self.resetServos()
        self.stopMotors()
        
        self.rightLED(0)
        self.leftLED(0)
        self.topLED(0)

        GPIO.cleanup()
        
        print("Servos, motors, and LEDs reset. GPIO cleaned up.")

    def resetServos(self):
        """
            Resets head and wheel servos to 0.
        """
        if not self.servosInitialized:
            self._servoSetup()
            
        self.sc.moveInit()

    def stopMotors(self):
        """
            Stops back wheel motors.
        """
        if not self.DCInitialized:
            self._DCSetup()
            
        DC.motorStop()

    def runMotors(self, dir, s):
        """
            runs both motors in direction dir (0 forwards, 1 backwards) at speed s (0-100).
        """

        self.runLeftMotor(dir, s)
        self.runRightMotor(dir, s)

        if s == 0:
            self.stopMotors()

    def runLeftMotor(self, dir, s):
        """
            runs left motor in direction dir at speed s.
        """
        if not self.DCInitialized:
            self._DCSetup()
            
        DC.motor_left(1, dir, s)

    def runRightMotor(self, dir, s):
        """
            runs right motor in direction dir at speed s.
        """
        if not self.DCInitialized:
            self._DCSetup()
            
        DC.motor_right(1, dir, s)

    def turnWheels(self, angle, printPowers=False):
        """
            turns front wheel servomotor to angle "angle".
        """
        if not self.servosInitialized:
            self._servoSetup()
            
        if not(self.minWheelsAngle <= angle <= self.MAX_ANGLE_WHEELS):
            raise ValueError(f"Angle for front wheel movement is capped between {self.minWheelsAngle} and {self.MAX_ANGLE_WHEELS}!")

        self.sc.moveAngle(0, angle, printPowers)

    def turnHeadHoriz(self, angle, printPowers=False):
        """
            turns horizontal head servomotor to angle "angle".
        """
        if not self.servosInitialized:
            self._servoSetup()
            
        if not(self.minHeadHorizAngle <= angle <= self.MAX_ANGLE_HEAD_HORIZ):
            raise ValueError(f"Angle for horizontal head movement is capped between {self.minHeadHorizAngle} and {self.MAX_ANGLE_HEAD_HORIZ}!")
        self.sc.moveAngle(1, angle, printPowers)

    def turnHeadVert(self, angle, printPowers=False):
        """
            turns vertical head servomotor to angle "angle".
        """
        if not self.servosInitialized:
            self._servoSetup()
            
        if not(self.minHeadVertAngle <= angle <= self.MAX_ANGLE_HEAD_VERT):
            raise ValueError(f"Angle for vertical head movement is capped between {self.minHeadVertAngle} and {self.MAX_ANGLE_HEAD_VERT}!")

        self.sc.moveAngle(2, angle, printPowers)

    def gradualAcceleration(self, acc, t, initSpeed=0):
        """
            accelerate both motors at acc for time t (or until max speed is reached) from current speed.
            initSpeed: optional, initial speed of robot to start acceleration at (will start at 0 by default)
            returns final speed after acceleration period.
        """

        speed = initSpeed

        startTime = time.time()
        while time.time() - startTime < t:
            self.runMotors(0, speed)
            if speed + acc > 100:
                speed = 100
                self.runMotors(0, speed)
                break

            speed += acc

        return speed

    def gradualSlowdown(self, t, initSpeed, stepSize=10):
        """
            slows down robot to a stop linearly in t seconds.
            initSpeed: initial speed to start slowdown at.
            stepSize: optional, number of speed decreases per second.
        """

        speed = initSpeed

        removeSpeed = speed / (t*stepSize) - 1e-9 #1e-9 so speed !< 0

        for _ in range(t*stepSize):
            speed -= removeSpeed

            try:
                self.runMotors(0, speed)
            except:

                break

            time.sleep(1/stepSize)

        self.stopMotors()

    def smoothTurn(self, id, start, end, loop, turnTime):

        """
            Turns "id" servomotor from angle start to angle end.
            id can be a single id or a list of id's (ie [0, 2])
            loop: if True, will go start-end-start.
            turnTime: turn time from start to end
        """

        if not self.servosInitialized:
            self.sc = servo.ServoCtrl()
            self.servosInitialized = True

        if not isinstance(id, list):
            idList = [id]
        else:
            idList = id

        runthrough = np.linspace(start, end, round(abs(start-end)/2))
        incrementDelay = turnTime/len(runthrough)

        for i in runthrough:

            for id in idList:
                self.sc.moveAngle(id, i)

            time.sleep(incrementDelay)

        if loop:
            for i in np.linspace(end, start, round(abs(start-end)/2)):

                for id in idList:
                    self.sc.moveAngle(id, i)

                time.sleep(incrementDelay)

    def driftReverseDirForward(self):
        """
            makes robot go forward, drift 180, go forward.
        """

        speed = 100
        self.turnWheels(0)
        self.runLeftMotor(0, speed)
        self.runRightMotor(0, speed)

        time.sleep(1)


        self.turnWheels(25)

        self.runLeftMotor(1, speed)
        self.runRightMotor(0, speed)

        time.sleep(1.5)
        self.turnWheels(0)
        self.runLeftMotor(0, speed)
        self.runRightMotor(0, speed)

        time.sleep(1)

        self.runMotors(0, 0)

    def driftReverseDirBackward(self):
        """
            makes robot go backwards, turn 180, go backwards.
        """

        speed = 100

        self.runMotors(1, speed)

        time.sleep(2)

        self.turnWheels(-25)

        self.runLeftMotor(1, speed)
        self.runRightMotor(0, speed)

        time.sleep(2)

        self.turnWheels(0)
        self.runMotors(1, speed)

        time.sleep(1)

        self.runMotors(0, 0)

    def celebrate(self, t):
        """
            "celebrates" for t seconds.
        """
        
        s = time.time()
        headHorizAngle = 0
        headVertAngle = 0
        wheelsAngle = 0
        
        headHorizIncrement = 1
        headVertIncrement = 1
        wheelsIncrement = 1
        
        switch_topLED = True
        switch_leftLED = True
        switch_rightLED = True
        
        lastSwitch_topLED = time.time()
        lastSwitch_leftLED = time.time()
        lastSwitch_rightLED = time.time()
        
        maxSwitchTime = .5
        minSwitchTime = .1
        randtime_topLED = rd.random() * (maxSwitchTime-minSwitchTime) + minSwitchTime
        randtime_leftLED = rd.random() * (maxSwitchTime-minSwitchTime) + minSwitchTime
        randtime_rightLED = rd.random() * (maxSwitchTime-minSwitchTime) + minSwitchTime
        
        while time.time() - s < t:
            
            if time.time() - lastSwitch_topLED > randtime_topLED:
                switch_topLED = not(switch_topLED)
                robot.topLED(switch_topLED)

                randtime_topLED = rd.random() * (maxSwitchTime-minSwitchTime) + minSwitchTime
                lastSwitch_topLED = time.time()
                
            if time.time() - lastSwitch_leftLED > randtime_leftLED:
                switch_leftLED = not(switch_leftLED)
                robot.leftLED(switch_leftLED)

                randtime_leftLED = rd.random() * (maxSwitchTime-minSwitchTime) + minSwitchTime
                lastSwitch_leftLED = time.time()
                
            if time.time() - lastSwitch_rightLED > randtime_rightLED:
                switch_rightLED = not(switch_rightLED)
                robot.rightLED(switch_rightLED)
                randtime_rightLED = rd.random() * (maxSwitchTime-minSwitchTime) + minSwitchTime
                lastSwitch_rightLED = time.time()
                

            self.turnHeadHoriz(headHorizAngle)
            self.turnHeadVert(headVertAngle)
            self.turnWheels(wheelsAngle)
            
            if not (self.minHeadHorizAngle <= headHorizAngle + headHorizIncrement <= self.MAX_ANGLE_HEAD_HORIZ):
                headHorizIncrement *= -1
            if not (self.minHeadVertAngle <= headVertAngle + headVertIncrement <= self.MAX_ANGLE_HEAD_VERT):
                headVertIncrement *= -1
            if not (self.minWheelsAngle <= wheelsAngle + wheelsIncrement <= self.MAX_ANGLE_WHEELS):
                wheelsIncrement *= -1
            
            headHorizAngle += headHorizIncrement
            headVertAngle += headVertIncrement
            wheelsAngle += wheelsIncrement
        
        self.resetServos()


robot = robotControl()
