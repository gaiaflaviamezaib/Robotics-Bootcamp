import RPi.GPIO as GPIO
import time

import robotControlClass as roboClass

# Define a callback function to handle the interrupt
def line_detected(channel): 
    r, m, l = robot.getInfraredState(printRes=False)
                
    if r and not(m) and not(l):
        #turnright60
        robot.turnWheels(-60)
        
    elif r and m and not(l):
        #turnright30
        robot.turnWheels(-30)

    elif m and not(r) and not(l):
        #straight
        robot.turnWheels(0)

    elif m and l and not(r):
        #turnleft30
        robot.turnWheels(30)

    elif l and not(m) and not(r):
        #turnleft60
        robot.turnWheels(60)


GPIO.setmode(GPIO.BCM) # Use BCM pin numbering
LEFT_PIN = 19
MIDDLE_PIN = 16
RIGHT_PIN = 20

PINS = [LEFT_PIN, MIDDLE_PIN, RIGHT_PIN]

for pin in PINS:
    # Set up infrared pins as inputs
    GPIO.setup(pin, GPIO.IN)
    # Set up the interrupt with a callback on a RISING edge (motion detected)
    GPIO.add_event_detect(pin, GPIO.RISING, callback=line_detected)


if __name__ == "__main__":
    try:
        
        robot = roboClass.robotControl()

        goForward=False
        if goForward:
            speed = 25
            robot.runMotors(0, speed)

        print("Waiting for motion...")
        
        runTime = 15
        
        startTime = time.time()
        while time.time()-startTime < runTime:    
            time.sleep(1) # Sleep to reduce CPU usage
            
    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
