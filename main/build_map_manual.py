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
    
    while True:
        robot.update_slam()
        print("slam updated")
        time.sleep(1)
    
    robot.close()
    
    

