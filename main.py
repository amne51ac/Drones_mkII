# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 14:56:33 2015

@author: amne51ac, andrewhessloch
"""


import dronekit
import atexit


class Drones():

    def __init__(self):
        # actions to perform on start of program, you know what init does.

        atexit.register(self.cleanup)

    def connect(self):
        self.vehicle = dronekit.connect('127.0.0.1:14550', wait_ready=True)

    def waypoint_handling(self):

        def prepare_waypoint(self):
            # self.vehicle.
            return

        def send_waypoint(self):
            return

        def street_waypoints(self):
            return

        return

    def interface(self):
        return

    def mainloop(self):
        return

    def cleanup(self):
        # actions to take at the end of the program, cleanup if you will lol
        if self.vehicle:
            self.vehicle.close()


if __name__ == "__main__":
    # This says that if it is the main program then execute this bit, otherwise
    # it is probably being imported as a library, you know...

    Drone = Drones()
    print "This is the beginning of the drone adventure!"
