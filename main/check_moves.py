from config import Config
from robot import Robot
from message import Message
import time

class Arlo(Robot):
    def __init__(self, config):
        super().__init__(config)
        self.name = "arlo"
        return
    
if __name__ == '__main__':
    config = Config()
    robot = Arlo(config)
    robot.start()
    
    robot.move(900, speed=0.4)
    time.sleep(1)

    print("+")
    robot.send_move_error(+0.3)
    time.sleep(10)

    #print("-")
    #robot.send_move_error(-0.1)
    #time.sleep(5)

    print(" 0")
    robot.send_move_error(0)
    time.sleep(3)

    robot.stop()

    
    robot.close()
    
    

