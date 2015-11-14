# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 14:56:33 2015

@author: amne51ac, andrewhessloch
"""


import dronekit
import atexit


class Delivery_Drones():
    # delivery drones is a program for autonomous delivery quadcopter platforms
    waypoints = None
    address = None
    cmds = None

    def __init__(self, address='127.0.0.1:14550'):
        # actions to perform on start of program, you know what init does.
        self.address = address
        self.connect()
        self.cmds = self.vehicle.commands

        atexit.register(self.cleanup)

    def connect(self):
        # builds the connection between this machine and the copter itself
        self.output("Connecting to %s" % self.address)
        try:
            self.vehicle = dronekit.connect(self.address, wait_ready=True)
        except dronekit.lib.APIException:
            self.output("No connection from drone.")
            exit

        self.output("Connected to %s" % self.address)

    def waypoint_handling(self):

        def prepare_waypoint(self):
            # prepare the waypoint object for relaying to the drone
            # self.vehicle.
            return

        def send_waypoint(self):
            # send the waypoint object to the drone
            return

        def street_waypoints(self):
            # a function for utilizing google maps queries to facilitate drone
            # navigation
            return

        return

    def interface(self):
        # here we can construct the graphic interface for the users
        return

    def mainloop(self):
        # this loop will help track heartbeat messages and such

        return

    def output(self, message):
        # this allows us to use a standard formatting for messages and
        # implement logging as well
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

    Drone = Delivery_Drones()

    Drone.mainloop()

    print "This is the beginning of the drone adventure!"
