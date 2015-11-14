# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 14:56:33 2015

@author: amne51ac, andrewhessloch
"""


import dronekit
import atexit


class Drones():
    waypoints = None
    address = None

    def __init__(self, address='127.0.0.1:14550'):
        # actions to perform on start of program, you know what init does.
        self.address = address

        atexit.register(self.cleanup)

    def connect(self):
        self.output("Connecting to %s" % self.address)
        try:
            self.vehicle = dronekit.connect(self.address, wait_ready=True)
        except dronekit.lib.APIException:
            self.output("No connection from drone.")
            exit

        self.output("Connected to %s" % self.address)

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

    def output(self, message):
        print message

    def cleanup(self):
        # actions to take at the end of the program, cleanup if you will lol
        self.output("Cleaning up...")
        if self.vehicle:
            self.vehicle.close()
        self.output("Done.")


if __name__ == "__main__":
    # This says that if it is the main program then execute this bit, otherwise
    # it is probably being imported as a library, you know...

    Drone = Drones()
    print "This is the beginning of the drone adventure!"
