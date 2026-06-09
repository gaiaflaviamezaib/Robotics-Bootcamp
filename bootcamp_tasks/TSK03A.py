import robotControlClass as roboClass

if __name__ == '__main__':
    try:
    
        robot = roboClass.robotControl()
        
        robot.smoothTurn(0, -45, 45, True, 2)  
        
    except KeyboardInterrupt:
        print("User terminated the program.")
    except Exception as e:
        print(f"Program terminated with exception:\n{e}")
    else:
        print("Program terminated normally.")
    finally:
        robot.resetAll()
