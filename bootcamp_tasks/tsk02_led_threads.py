import RPi.GPIO as GPIO
import threading
import time

import robotControlClass as roboClass

def switchSetup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(5, GPIO.OUT)
    GPIO.setup(6, GPIO.OUT)
    GPIO.setup(13, GPIO.OUT)

def LEDB_statuschange(triggerDelay):
  global LEDB_status
  while not stop_threadsB:
    LEDB_status = not(LEDB_status)
    time.sleep(triggerDelay)
  LEDB_status = False

def LED6_statuschange(triggerDelay):
  global LED6_status
  while not stop_threads6:
    LED6_status = not(LED6_status)
    time.sleep(triggerDelay)
  LED6_status = False

def LED13_statuschange(triggerDelay):
  global LED13_status
  while not stop_threads13:
    LED13_status = not(LED13_status)
    time.sleep(triggerDelay)
  LED13_status = False
    
def LEDB(blinkDelay):
  global LEDB_status
  while not stop_threadsB:
    if LEDB_status==1:
      GPIO.output(5, GPIO.HIGH)
      time.sleep(blinkDelay)
    GPIO.output(5,GPIO.LOW)
    time.sleep(blinkDelay)
  GPIO.output(5,GPIO.LOW)

def LED6(blinkDelay):
  global LED6_status
  while not stop_threads6:
    if LED6_status==1:
      GPIO.output(6, GPIO.HIGH)
      time.sleep(blinkDelay)
    GPIO.output(6,GPIO.LOW)
    time.sleep(blinkDelay)
  GPIO.output(6,GPIO.LOW)

def LED13(blinkDelay):
  global LED13_status
  while not stop_threads13:
    if LED13_status==1:
      GPIO.output(13, GPIO.HIGH)
      time.sleep(blinkDelay)
    GPIO.output(13,GPIO.LOW)
    time.sleep(blinkDelay)
  GPIO.output(13,GPIO.LOW)


LEDB_status=1
LED6_status=1
LED13_status=1
stop_threadsB=False
stop_threads6=False
stop_threads13=False

LEDB_statusChangeDelay = 2
LED6_statusChangeDelay = 2
LED13_statusChangeDelay = 1

LEDB_blinkDelay = .05
LED6_blinkDelay = .1
LED13_blinkDelay = .2

if __name__ == "__main__":

    try:
      
      robot = roboClass.robotControl()
      
      switchSetup()
      thread1=threading.Thread(target=LEDB, args = (LEDB_blinkDelay,)) #arg: blink delay when led is on
      thread2=threading.Thread(target=LEDB_statuschange, args = (LEDB_statusChangeDelay,)) #arg: time between turning on and off led
      thread3=threading.Thread(target=LED6, args = (LED6_blinkDelay,))
      thread4=threading.Thread(target=LED6_statuschange, args = (LED6_statusChangeDelay,))
      thread5=threading.Thread(target=LED13, args = (LED13_blinkDelay,))
      thread6=threading.Thread(target=LED13_statuschange, args = (LED13_statusChangeDelay,))
      thread1.start()
      print("LED B threads started!")
      thread2.start()
      
      time.sleep(2)
      
      thread3.start()
      print("LED 6 threads started!")
      thread4.start()
      
      time.sleep(5)
      
      stop_threadsB=True
      print("LED B threads terminated!")
      
      time.sleep(3)
      
      thread5.start()
      print("LED 13 threads started!")
      thread6.start()
      
      time.sleep(5)
      
      stop_threads6=True
      print("LED B threads terminated!")
      stop_threads13=True
      print("LED 13 threads terminated!")
      
      print('All threads have been terminated.')
      
    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
