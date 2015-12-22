# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 14:56:33 2015

@author: amne51ac, andrewhessloch
"""

import dronekit
import atexit
from pymavlink import mavutil
import time
import sys
import glob
import usb
import serial

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

    def serial_ports():
        """ Lists serial port names
            :raises EnvironmentError:
                On unsupported or unknown platforms
            :returns:
                A list of the serial ports available on the system
        """
        if sys.platform.startswith('win'):
            # ports = ['COM%s' % (i + 1) for i in range(256)]
            busses = usb.busses()
            for bus in busses:
                devices = bus.devices
                for dev in devices:
                    print "Device:", dev.filename
                    print "  idVendor: %d (0x%04x)" % (dev.idVendor,
                                                       dev.idVendor)
                    print "  idProduct: %d (0x%04x)" % (dev.idProduct,
                                                        dev.idProduct)

        elif (sys.platform.startswith('linux') or
              sys.platform.startswith('cygwin')):
            # this excludes your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')
        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')
        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result

    def connect(self):
        # builds the connection between this machine and the copter itself
        self.output("Connecting to %s" % self.address)
        try:
            self.vehicle = dronekit.connect(self.address, baud=57600,
                                            wait_ready=True)
        except:  # dronekit.lib.APIException:
            self.output("No connection from drone.")
            exit()
        self.output("Connected to %s" % self.address)

    def clear_waypoint(self):
        self.cmds.clear()
        self.cmds.upload()

    def prepare_waypoint(self, lat=12, lon=12):
        # prepare the waypoint object for relaying to the drone
        # one for destination
        self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, 11))
        # one for land
        self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0, 11))
        # one for release
        self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0, 0, 0, 0, 0, 0, 0, 11))
        # one for takeoff
        self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 11))
        # undo release
        self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 0, 0, 0, 0, 0, 0, 0, 11))
        # rtl
        self.cmds.add(dronekit.Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, lon, 11))

    def send_waypoint(self):
        # send the waypoint object to the drone
        self.cmds.upload()
        self.cmds.wait_ready()

    def street_waypoints(self):
        # a function for utilizing google maps queries to facilitate drone
        # navigation
        return

    def arm_copter(self, check=False):
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

    def mainloop(self, lat=41.840157, lon=-87.624705):
        # clear old waypoints
        self.clear_waypoint()
        # take the coordinates
        self.prepare_waypoint(lat, lon)
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


class Phone_App():
    lat = 0.0
    lon = 0.0

    def __init__(self):
        atexit.register(self.cleanup)
        return self.lat, self.lon
        self.cleanup()
        pass

    def wait_connection(self):
        pass

    def cleanup(self):
        pass


if __name__ == "__main__":
    # This says that if it is the main program then execute this bit, otherwise
    # it is probably being imported as a library, you know...

    Drone = Delivery_Drones()

    lat, lon = Phone_App()

    Drone.mainloop(lat, lon)

    print "This is the beginning of the drone adventure!"
