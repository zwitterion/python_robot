# pytest tests for moduel robot.py
import pytest
from robot import Robot
from config import Config
import logging

# content of test_class.py
class TestClass:

    def test_max_distance(self, caplog):
        
        self.config = Config(logger=caplog)
        robot = Robot(self.config)
        
        path = []
        assert robot.get_max_distance(path) == 0, "path length should be zero"

        path = [(1,1)]
        assert robot.get_max_distance(path) == 1, "path length should be one"

        path = [(1,1),(2,2),(3,3), (10,1)]
        assert robot.get_max_distance(path) == 2, "path length should be 2"

        path = [(1,1),(2,1),(3,1), (10,1)]
        assert robot.get_max_distance(path) == 3, "path length should be 3"

        path = [(1,10),(1,9),(1,8), (2,8)]
        assert robot.get_max_distance(path) == 2, "path length should be 2"


        return

    def test_get_orientation_to_target(self, caplog):

        self.config = Config(logger=caplog)
        robot = Robot(self.config)

        a = [0,0]
        b = [1,1]
        assert robot.get_orientation_to_target(a, b) == 45

        a = [0,0]
        b = [1,0]
        assert robot.get_orientation_to_target(a, b) == 0

        a = [0,0]
        b = [1,-1]
        assert robot.get_orientation_to_target(a, b) == -45

        a = [0,0]
        b = [0,1]
        assert robot.get_orientation_to_target(a, b) == 90

        a = [0,0]
        b = [0,-1]
        assert robot.get_orientation_to_target(a, b) == -90

        a = [0,0]
        b = [-1,1]
        assert robot.get_orientation_to_target(a, b) == 135

        a = [0,0]
        b = [-1,-1]
        assert robot.get_orientation_to_target(a, b) == -135

        a = [0,0]
        b = [-1,0]
        assert robot.get_orientation_to_target(a, b) == 180

        return

#t = TestClass()
#logging.basicConfig(filename='test_logs.log', level=logging.INFO)
#t.test_max_distance(logging)