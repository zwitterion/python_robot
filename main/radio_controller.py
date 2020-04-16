# encapsulates logic related to Futaba, Taranis and similar
# multi-channel digital radio controllers.
import numpy as np

# how big of a difference between to channel values to be cosnidered a change (noise)
channel_threshold = 10

channel_min_value = 172
channel_max_value = 1811


class FutabaController():
    
    channel_direction = 2
    channel_speed = 3

    def __init__(self, params):
        self.is_on = params[0] == 0
        self.error_code = params[0]
        # array of 16 int channel values 
        self.channel = params[1]

        # convert speed channel to -1,+1
        speed = 2.0*((self.channel[FutabaController.channel_speed] - channel_min_value) / (channel_max_value-channel_min_value) - 0.5)
        self.channel[FutabaController.channel_speed] = np.clip(speed, -1.0, +1.0)
        
        # convert direction to angle -90, +90
        direction = 180.0*((self.channel[FutabaController.channel_direction] - channel_min_value) / (channel_max_value-channel_min_value) - 0.5)
        self.channel[FutabaController.channel_direction] = np.clip(direction, -90.0, +90.0)



        return
    
    def compare(self, other):
        """ returns an array of 17 True/False values indicating changes bettwen this instance and passed one"""
        """ [0] represent changes in on/off """
        """ [1-16] represent changes in channel values"""
        changes = 17*[-1]
        changes[0] = (other.is_on != self.is_on)
        
        for i in range(len(self.channel)):
            changes[i+1] = abs(self.channel[i] - other.channel[i]) > channel_threshold
        return changes

    def get_direction(self):
        return self.channel[FutabaController.channel_direction] 

    def get_speed(self):
        return self.channel[FutabaController.channel_speed] 

if __name__ == '__main__':
    # test
    f1 = FutabaController([12, [100, 200, 300]])
    f2 = FutabaController([0, [100, 250, 302]])
    assert(f1.compare(f2)[0:4] == [True, False, True, False])
    print ("it works")
