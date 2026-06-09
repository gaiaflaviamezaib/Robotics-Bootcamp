import time
import threading
import numpy as np

import robotControlClass as roboClass

def smoothTurn(start, end, loop):
    
    global turningEnded

    for i in np.linspace(start, end, round(abs(start-end)/2)):
   
        robot.turnHeadHoriz(i)
        print(f"angle: {i} deg\n")
        loopControlEvent.wait()
        loopControlEvent.clear()
        time.sleep(.1)
    
    if loop:
          for i in np.linspace(end, start, round(abs(start-end)/2)):
        
              # sc.moveAngle(id, i)
              robot.turnHeadHoriz(i)
              loopControlEvent.wait()
              loopControlEvent.clear()
              time.sleep(.1)
    turningEnded = True

def measureAnalysis():
    print("Ultrasonic distance measurement started (Ctrl+C to stop)")
    try:
      while True:
        
        dist = robot.measureDistance()
        
        if dist is not None:
          print(f"Distance: {dist} cm")
          if dist < 5:
            print("CAREFUL! STOP THE ROBOT! (AN OBSTACLE IS TOO CLOSE!)")
          
        else:
          print("No echo detected")
        
        loopControlEvent.set()
        
        if turningEnded:
          endMeasurementEvent.set()
          break
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
        
turningEnded=False
if __name__ == "__main__":
  
    try:
      
      robot = roboClass.robotControl()
            
      robot.resetServos()
      
      loopControlEvent = threading.Event()
      endMeasurementEvent = threading.Event()
      
      thread1 = threading.Thread(target=smoothTurn, args = (45, -45, False,))
      thread2 = threading.Thread(target=measureAnalysis)
      thread1.start()
      thread2.start()
      
      endMeasurementEvent.wait()
      
    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
        
