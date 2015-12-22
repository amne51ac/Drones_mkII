"""
@author: PBoutaleb
"""

import sys
import glob
import serial
import usb


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


if __name__ == '__main__':
    print(serial_ports())
