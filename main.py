# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 14:56:33 2015

@author: amne51ac, andrewhessloch
"""

import dronekit
import atexit
from pymavlink import mavutil
import time

"""
This is the rewrite of the drones project from the Illinois Instute of
Technology, led by professor Jeremy Hajek, written by Mark Milhouse and Andrew
Hessler.  Drone engineering by Nabil Boutaleb, Mark Milhouse, Sean Macintosh,
Natalia Gruszka, and Mark Latocha.  This code is a private prototype and
intended for demonstration purposes only, not for distrobution or use with
live production systems.

Delivery_Drones():
    # delivery drones is a program for autonomous delivery quadcopter platforms
    waypoints
    address
    cmds

    __init__(self, address='127.0.0.1:14550'):
        # actions to perform on start of program, you know what init does.

    connect(self):
        # builds the connection between this machine and the copter itself

    waypoint_handling(self):

        prepare_waypoint(self):
            # prepare the waypoint object for relaying to the drone
            # self.vehicle.

        send_waypoint(self):
            # send the waypoint object to the drone

        street_waypoints(self):
            # a function for utilizing google maps queries to facilitate drone
            # navigation

    interface(self):
        # here we can construct the graphic interface for the users

    mainloop(self):
        # this loop will help track heartbeat messages and such

    output(self, message):
        # this allows us to use a standard formatting for messages and
        # implement logging as well

    cleanup(self):
        # actions to take at the end of the program, cleanup if you will lol

"""


class Delivery_Drones():
    # delivery drones is a program for autonomous delivery quadcopter platforms
    waypoints = None
    address = None
    cmds = None

    def __init__(self, address='/dev/cu.usbserial-DN008PBS'):
        # actions to perform on start of program, you know what init does.
        self.address = address
        self.connect()
        self.cmds = self.vehicle.commands

        atexit.register(self.cleanup)

    def connect(self):
        # builds the connection between this machine and the copter itself
        self.output("Connecting to %s" % self.address)
        try:
            self.vehicle = dronekit.connect(self.address, baud=57600)  # , wait_ready=True)
        except dronekit.lib.APIException:
            self.output("No connection from drone.")
            self.cleanup()
        self.output("Connected to %s" % self.address)

    def waypoint_handling(self):

        def prepare_waypoint(self):
            # prepare the waypoint object for relaying to the drone
            for i in self.waypoints:
                self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, i.lat, i.lon, 11))
            return

        def send_waypoint(self):
            # send the waypoint object to the drone
            self.cmds.upload()
            return

        def street_waypoints(self):
            # a function for utilizing google maps queries to facilitate drone
            # navigation
            return

        return

    def arm_copter(self):
        # a function for safely arming the vehicle

        while not self.vehicle.is_armable:
            print "waiting for vehicle to be armable"
            time.sleep(1)

        if self.vehicle.armed:
            self.output("Vehicle already armed")
        else:
            self.vehicle.armed = True

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
        try:
            if self.vehicle:
                self.vehicle.close()
        finally:
            self.output("Done.")
            exit()


if __name__ == "__main__":
    # This says that if it is the main program then execute this bit, otherwise
    # it is probably being imported as a library, you know...

    Drone = Delivery_Drones()

    Drone.mainloop()

    print "This is the beginning of the drone adventure!"
