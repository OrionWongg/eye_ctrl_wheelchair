"""Check reset avoidance on native USB and unchanged adapter behavior."""

import termios
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

from whill_controller.serial_connection import open_serial


def test_native_usb_preserves_lines_and_disables_hangup():
    """Resolve aliases and avoid the DTR-low/RTS-high reset transition."""
    device = MagicMock()
    info = SimpleNamespace(device='/dev/ttyACM0', vid=0x303A, pid=0x1001)
    attributes = [0, 0, termios.HUPCL | termios.CLOCAL, 0, 0, 0, []]
    module = 'whill_controller.serial_connection.'
    with patch(module + 'list_ports.comports', return_value=[info]), \
            patch(module + 'os.path.realpath', return_value='/dev/ttyACM0'), \
            patch(module + 'serial.Serial', return_value=device), \
            patch(module + 'termios.tcgetattr', return_value=attributes), \
            patch(module + 'termios.tcsetattr') as configure:
        def check_lines():
            assert device.dtr is True
            assert device.rts is True
        device.open.side_effect = check_lines
        assert open_serial('/dev/ble2wchair', 115200, 1) is device
        flags = configure.call_args.args[2][2]
        assert not flags & termios.HUPCL
        assert flags & termios.CLOCAL
        assert device.exclusive is True


def test_other_interfaces_keep_existing_settings():
    """Do not change control lines on USB adapters or pseudo terminals."""
    device = MagicMock()
    module = 'whill_controller.serial_connection.'
    with patch(module + 'list_ports.comports', return_value=[]), \
            patch(module + 'serial.Serial', return_value=device), \
            patch(module + 'termios.tcgetattr') as attributes:
        open_serial('/dev/pts/1', 115200, 2)
        assert device.dtr is False
        assert device.rts is False
        attributes.assert_not_called()
