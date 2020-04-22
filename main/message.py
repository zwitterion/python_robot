import uuid
import json

class Message(object):
  
  status = 1
  futaba = 2
  odometer = 3
  start = 4
  stop = 5
  get_lidar_data = 6
  lidar_data = 7
  get_map=8
  map_update=9
  reset_map=10
  set_target_position = 11

  move = 100
  move_error = 102
  command_completed = 200
  echo =210
  debug=220  
  shutdown = 300

  def __init__(self, command=None, params=None, reply_to=None):
    super().__init__() 
    self.id = str(uuid.uuid4())
    self.cmd = command
    self.params = params
    self.reply_to = reply_to
  
  def to_json(self):
    return json.dumps({k: self.__dict__.get(k, None) for k in ('id', 'cmd', 'params', "reply_to")})
  
  def from_json(js):
    d = json.loads(js)
    msg = Message()
    msg.id = d["id"]
    msg.cmd = d["cmd"]
    msg.params = d["params"]
    msg.reply_to = d.get("reply_to", None)
    return msg  

  def __repr__(self):
    return self.to_json()

if __name__ == "__main__":
    msg = Message(1, (10,20,30), "to.me")
    print(msg.to_json())
    msg = Message(1, (10,20,30))
    print(msg.to_json())
    msg = Message(1)
    print(msg.to_json())
    
"""
class Command():
  
  status = 1
  futaba = 2
  odometer = 3
  start = 4
  stop = 5
  echo =10
  execute_plan = 6
  move = 100
  shutdown = 300



  pass
"""


