import time
import robotControlClass as roboClass

if __name__ == "__main__":
    try:    
        robot = roboClass.robotControl()
        
        goForward=False
        if goForward:
          speed = 25
          robot.runMotors(0, speed)
        
        runTime = 60
        
        startTime = time.time()
        while time.time()-startTime < runTime:
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

    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
