"""Read X11 key state independently of Tk's synthetic auto-repeat releases."""
import ctypes
from ctypes.util import find_library


class PhysicalKeyboard:
    def __init__(self):
        self.display = None
        self.x11 = ctypes.CDLL(find_library('X11') or 'libX11.so.6')
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.x11.XStringToKeysym.argtypes = [ctypes.c_char_p]
        self.x11.XStringToKeysym.restype = ctypes.c_ulong
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XKeysymToKeycode.restype = ctypes.c_ubyte
        self.x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
        self.x11.XQueryKeymap.restype = ctypes.c_int
        self.display = self.x11.XOpenDisplay(None)
        if not self.display:
            raise RuntimeError('Hold-to-run requires an accessible X11 keyboard state')
        self.keycodes = {key: self.x11.XKeysymToKeycode(
            self.display, self.x11.XStringToKeysym(key.encode())) for key in ('w', 'a', 'd', 'u')}

    def is_down(self, key):
        if not self.display or not self.keycodes.get(key):
            return False
        state = (ctypes.c_ubyte * 32)()
        if not self.x11.XQueryKeymap(self.display, state):
            return False
        code = self.keycodes[key]
        return bool(state[code // 8] & (1 << (code % 8)))

    def close(self):
        if self.display:
            self.x11.XCloseDisplay(self.display)
            self.display = None
