import time

import robotControlClass as roboClass

if __name__ == "__main__":
    try:
        
        robot = roboClass.robotControl()
        
        speed = 20
        robot.runMotors(0, speed)
        
        print("Waiting for motion...")
        while True:
            roll, pitch, acc = robot.getAccelerometerData()    # Calculate and print angles
            
            if not(-40 < roll < 40) or not(-15 < pitch < 15):
                print('INCLINE TOO STEEP! STOP THE ROBOT!')
                robot.stopMotors()
            
            else:
                robot.runMotors(0, speed)
                
            time.sleep(0.2)             # Delay between readings
            
    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
