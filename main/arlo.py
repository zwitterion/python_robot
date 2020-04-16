from config import Config
from robot import Robot
from message import Message
import time
from messagebus_manager import ProcessNames

class Arlo(Robot):
    def __init__(self, config):
        super().__init__(config)
        self.name = "arlo"
        return
    
    def rotate(self, steps, speed=0.8):
        self.config.log.info("Robot is starting to move: {} unit(s)".format(steps))
        heading = +90
        msg = Message(Message.move, params=[speed, heading, steps])
        msg.reply_to = self.name
        self.message_bus.send(ProcessNames.controller_interface, msg)

    def move_x(self, steps, speed=0.8):
        self.config.log.info("Robot is starting to move: {} unit(s)".format(steps))
        heading = 0
        msg = Message(Message.move, params=[speed, heading, steps])
        msg.reply_to = self.name
        self.message_bus.send(ProcessNames.controller_interface, msg)



    def run(self):

        while True:
            if len(self.path) > 0:
                self.rotate(self.robot_position, self.path[0])
                self.move(self.robot_position, self.path[0])
            else:
                time.sleep(1)

        return

if __name__ == '__main__':
    config = Config()
    robot = Arlo(config)
    robot.start()

    #robot.run()

    #result = robot.rotate(steps=100)
    result = robot.move_x(steps=200)
    time.sleep(2)
    result = robot.stop()


    robot.close()
    

