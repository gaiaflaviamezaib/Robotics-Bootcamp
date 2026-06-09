import time

import robotControlClass as roboClass

if __name__ == "__main__":
  
    try:
      
      robot = roboClass.robotControl()
      
      while True:
        
        dist = robot.measureDistance()
        
        roll, pitch, acc = robot.getAccelerometerData(printRes=False)
        
        if dist > 10:
          robot.OLEDDisplay.set_all_lines([
            f"Obstacle dist: {round(dist, 2)}",
            f"Pitch: {round(pitch, 2)}",
            f"Roll: {round(roll, 2)}",
            "","",""])
        
        else:
          robot.OLEDDisplay.set_all_lines(["","","","","",""]) #clear display
          robot.OLEDDisplay.update_line(3, "WARNING!")
          robot.OLEDDisplay.update_line(4, " OBSTACLE TOO CLOSE!")
        
        time.sleep(.2)
      
    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
