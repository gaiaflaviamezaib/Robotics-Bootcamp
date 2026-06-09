import time
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from bootcamp_tasks.robotControlClass import robotControl

# =========================================================================
# CONFIGURATION : TRAVERSAL STEPS
# =========================================================================
# Modify this list to define what the robot does at each intersection.
PATH_INSTRUCTIONS = ["left", "stop"]

robot = robotControl()

def turn_right(robot):
    print("\n>>> Executing right turn...")
    backTurnCoeff = 25
    frontTurnCoeff = 15
    frontSpeed = 25
    backSpeed = 40
    
    robot.turnWheels(backTurnCoeff)
    robot.runMotors(1, backSpeed)
    time.sleep(0.55)
    
    robot.turnWheels(-frontTurnCoeff)
    robot.runMotors(0, frontSpeed)
    time.sleep(1)
    
    state = robot.getInfraredState(False)
    while not (state[0] or state[1] or state[2]):
        time.sleep(0.01)
        state = robot.getInfraredState(False)
        
    robot.stopMotors()
    robot.turnWheels(0)
    time.sleep(0.2)
    print(">>> Right turn completed.\n")

def turn_left(robot):
    print("\n>>> Executing left turn...")
    backTurnCoeff = 25
    frontTurnCoeff = 15
    frontSpeed = 25
    backSpeed = 40
    
    robot.turnWheels(-backTurnCoeff)
    robot.runMotors(1, backSpeed)
    time.sleep(0.55)
    
    robot.turnWheels(frontTurnCoeff)
    robot.runMotors(0, frontSpeed)
    time.sleep(1)
    
    state = robot.getInfraredState(False)
    while not (state[0] or state[1] or state[2]):
        time.sleep(0.01)
        state = robot.getInfraredState(False)
        
    robot.stopMotors()
    robot.turnWheels(0)
    time.sleep(0.2)
    print(">>> Left turn completed.\n")

# =========================================================================
# MAIN LOOP (PID EXACTLY LIKE infinity_circuit.py)
# =========================================================================
print("="*40)
print(f"Starting simple traversal.")
print(f"Planned instructions: {PATH_INSTRUCTIONS}")
print("="*40)

# PID Configuration copied from infinity_circuit.py
Kp = 5.0
Ki = 1.0
Kd = 2.5
turn_multiplier = 1.0
pastStates = []
lastError = 0

def getError(infraState, lastError):
    if infraState == [1, 0, 1]: return 0.0
    elif infraState == [1, 1, 0]: return 1.0   
    elif infraState == [1, 0, 0]: return 2.0   
    elif infraState == [0, 1, 0]: return 0.0
    elif infraState == [0, 1, 1]: return -1.0  
    elif infraState == [0, 0, 1]: return -2.0  
    elif infraState == [1, 1, 1]: return lastError
    elif infraState == [0, 0, 0]: return lastError
    else: return lastError

robot._measureSetup()
print("Centering wheels for startup...")
robot.turnWheels(0)
time.sleep(0.5)

programStartTime = time.time()
lastIntersectionTime = time.time() - 10
instructions = list(PATH_INSTRUCTIONS)
last_printed_state = None

try:
    while True:
        now = time.perf_counter()
        
        currentInfraState = robot.getInfraredState(False)
        
        if currentInfraState != last_printed_state:
            print(f"[SENSORS] {currentInfraState}")
            last_printed_state = currentInfraState

        # Intersection detection (Ignored during the first 1.5 seconds)
        is_startup_immunity = time.time() - programStartTime < 1.5
        
        if currentInfraState == [1, 1, 1] and not is_startup_immunity and time.time() - lastIntersectionTime > 2.0:
            print("\n" + "="*40)
            print(f"[INTERSECTION DETECTED] Sensors: {currentInfraState}")
            
            # Exact re-centering on the intersection (inertia)
            robot.stopMotors()
            robot.turnWheels(0)
            time.sleep(0.25)
            if robot.getInfraredState(False) != [1, 1, 1]:
                print("Slight overshoot, reversing to re-center on the line...")
                robot.runMotors(1, 22)
                while robot.getInfraredState(False) != [1, 1, 1]:
                    time.sleep(0.02)
                robot.stopMotors()
                time.sleep(0.25)
                
            print("Centering complete. Stopping for 2 seconds to confirm...")
            time.sleep(2.0)
            
            if not instructions:
                print("No more instructions available. Stopping.")
                break
                
            nextAction = instructions.pop(0)
            print(f"-> Action: {nextAction}")
            
            if nextAction == "stop":
                print("Destination reached!")
                break
                
            elif nextAction == "straight":
                print("Going straight...")
                robot.runMotors(0, 30)
                robot.turnWheels(0)
                time.sleep(0.8)
                
            elif nextAction == "left":
                robot.stopMotors()
                turn_left(robot)
                
            elif nextAction == "right":
                robot.stopMotors()
                turn_right(robot)
                
            lastIntersectionTime = time.time()
            continue

        # PID line following
        error = getError(currentInfraState, lastError)
            
        proportional = error
        
        pastStates.append(error)
        if len(pastStates) > 20:
            pastStates.pop(0)
            
        integral = sum(pastStates)
        integral = max(-40, min(40, integral))
        
        derivative = (error - lastError)
        lastError = error

        output = (Kp * proportional) + (Ki * integral) + (Kd * derivative)
        output = max(-25, min(25, output))

        robot.turnWheels(output * turn_multiplier)
        robot.runMotors(0, 23)

        dt = time.perf_counter() - now
        if dt < 0.01:
            time.sleep(0.01 - dt)

    print("\n=== TRAVERSAL SUCCESSFULLY COMPLETED ===")

except KeyboardInterrupt:
    print("\nStopped by user (Ctrl+C).")
finally:
    robot.resetAll()
    print("Robot stopped safely.")
