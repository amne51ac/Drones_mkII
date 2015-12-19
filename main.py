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

        atexit.register(self.cleanup)
        self.address = address
        self.connect()
        self.cmds = self.vehicle.commands

    def connect(self):
        # builds the connection between this machine and the copter itself
        self.output("Connecting to %s" % self.address)
        try:
            self.vehicle = dronekit.connect(self.address, baud=57600,
                                            wait_ready=True)
        except dronekit.lib.APIException:
            self.output("No connection from drone.")
            exit()
        self.output("Connected to %s" % self.address)

    def clear_waypoint(self):
        self.cmds.clear()
        self.cmds.upload()

    def prepare_waypoint(self, lat=12, lon=12):
        # prepare the waypoint object for relaying to the drone
        self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, 11))
        # one for destination
        # one for land
        # one for release
        # one for takeoff
        # undo release
        # rtl

    def send_waypoint(self):
        # send the waypoint object to the drone
        self.cmds.upload()
        self.cmds.wait_ready()

    def street_waypoints(self):
        # a function for utilizing google maps queries to facilitate drone
        # navigation
        return

    def arm_copter(self, check=True):
        # a function for safely arming the vehicle

        if check:
            while not self.vehicle.is_armable:
                self.output("Waiting for vehicle to be armable...")
                time.sleep(1)

        if self.vehicle.armed:
            self.output("Vehicle already armed")
        else:
            self.vehicle.armed = True

        while not self.vehicle.armed:
            self.output("Arming...")
            time.sleep(1)

        self.output("!!!Armed!!!")

    def takeoff(self, target=20):
        self.output("Ready to launch...")
        self.vehicle.simple_takeoff(target)  # and target altitude
        self.output("Taking off...")

        while True:
            self.output(("Altitude: ",
                         self.vehicle.location.global_relative_frame.alt))
            if self.vehicle.location.global_relative_frame.alt >= target*0.95:
                self.output("Reached target altitude")
                break
            time.sleep(1)

        self.cmds.next = 0
        self.vehicle.mode = dronekit.VehicleMode("AUTO")

    def begin():
        #self.vehicle.
        pass

    def interface(self):
        # here we can construct the graphic interface for the users
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
                if self.vehicle.armed:
                    self.vehicle.armed = False
                self.vehicle.close()
        finally:
            self.output("Done.")
            exit()

    def mainloop(self):
        # clear old waypoints
        self.clear_waypoint()
        # take the coordinates
        self.prepare_waypoint()
        # write coordinates to the copter
        self.send_waypoint()
        # arm the copter
        self.arm_copter()
        # send the takeoff command
        self.takeoff()
        # begin mission
        # self.begin()

        self.output("Ready to end.")
        raw_input()
        exit()
        


if __name__ == "__main__":
    # This says that if it is the main program then execute this bit, otherwise
    # it is probably being imported as a library, you know...

    Drone = Delivery_Drones()

    print "This is the beginning of the drone adventure!"
