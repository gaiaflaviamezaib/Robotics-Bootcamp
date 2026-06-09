# Ultrasonic distance sensor (HC-SR04)
import time
import RPi.GPIO as GPIO

TRIG_PIN = 11
ECHO_PIN = 8


class UltrasonicSensor:
    """HC-SR04 ultrasonic distance sensor."""

    def __init__(self, trig=TRIG_PIN, echo=ECHO_PIN):
        self.trig = trig
        self.echo = echo
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig, GPIO.OUT)
        GPIO.setup(self.echo, GPIO.IN)
        GPIO.output(self.trig, False)
        time.sleep(0.05)

    def measure(self, timeout=0.04):
        """Return distance in cm, or None on timeout."""
        GPIO.output(self.trig, True)
        time.sleep(0.00001)
        GPIO.output(self.trig, False)

        start = time.time()
        deadline = start + timeout

        while GPIO.input(self.echo) == 0:
            start = time.time()
            if start > deadline:
                return None

        while GPIO.input(self.echo) == 1:
            stop = time.time()
            if stop > deadline:
                return None

        elapsed = stop - start
        distance = (elapsed * 34300) / 2
        return round(distance, 1)

    def cleanup(self):
        GPIO.output(self.trig, False)


class HeadScanner:
    """Servo-based head scanner for directional ultrasonic measurements."""

    def __init__(self, servo_ctrl=None, servo_id=1, min_angle=-120, max_angle=120, step=10):
        self.servo_ctrl = servo_ctrl
        self.servo_id = servo_id
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.step = step

    def cleanup(self):
        if self.servo_ctrl:
            self.servo_ctrl.moveAngle(self.servo_id, 0)
