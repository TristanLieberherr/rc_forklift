# Mode
_DRIVE  = 0
_STEER  = 2
_LIFT   = 4
# Value
STOP    = 0x00
FWD     = 0x01
REV     = 0x02
# Aliases
CENTER  = STOP
LEFT    = FWD
RIGHT   = REV
UP      = FWD
DOWN    = REV



class ControlRegister:
    def __init__(self):
        self._reg = 0x00

    def set_reg(self, reg):
        self._reg = reg

    def get_reg(self):
        return self._reg

    def _set_value(self, mode, value):
        self._reg &= ~(0x03<<mode)
        self._reg |= value<<mode

    def set_drive(self, value):
        self._set_value(_DRIVE, value)

    def set_steer(self, value):
        self._set_value(_STEER, value)

    def set_lift(self, value):
        self._set_value(_LIFT, value)

    def _get_value(self, mode):
        return (self._reg>>mode) & 0x03

    def get_drive(self):
        return self._get_value(_DRIVE)

    def get_steer(self):
        return self._get_value(_STEER)

    def get_lift(self):
        return self._get_value(_LIFT)
