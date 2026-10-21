"""Open ESP32 serial interfaces without triggering native USB reset."""

import os
import termios

import serial
from serial.tools import list_ports


def open_serial(port, baud, write_timeout):
    """Keep native USB DTR/RTS asserted and prevent hangup on Linux close."""
    resolved = os.path.realpath(port)
    native_usb = any(
        os.path.realpath(info.device) == resolved
        and (info.vid, info.pid) == (0x303A, 0x1001)
        for info in list_ports.comports()
    )
    device = serial.Serial()
    device.port = port
    device.baudrate = baud
    device.timeout = 0
    device.write_timeout = write_timeout
    # Linux CDC ACM asserts both lines on open. Clearing DTR first while
    # RTS remains asserted resets the ESP32-S3 USB Serial/JTAG controller.
    device.dtr = native_usb
    device.rts = native_usb
    device.exclusive = True
    try:
        device.open()
        if native_usb:
            attributes = termios.tcgetattr(device.fileno())
            attributes[2] &= ~termios.HUPCL
            termios.tcsetattr(device.fileno(), termios.TCSANOW, attributes)
    except Exception:
        device.close()
        raise
    return device
