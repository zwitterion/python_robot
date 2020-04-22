import pytest
from planner import Map
from config import Config
import logging

# content of test_class.py
class TestClass:

    def test_cost(self, caplog):
        
        self.config = Config(logger=caplog)

        map = Map((10,10))
        
        map.target_position = [5,5]
        map.robot_position = [1,1]
        path = map.get_path()
        
        print(path)
        print(map.get_path_cost(path))


        return

t = TestClass()
logging.basicConfig(filename='test_logs.log', level=logging.INFO)
t.test_cost(logging)