class ServoMotor:
    def __init__(self, bus, channel):
        self._bus = bus
        self._channel = channel
        self.__CENTER = 315
        self.__STEP = 55
        self.__I2C_ADDRESS         = 0x7F
        self.__MODE1               = 0x00
        self.__PRESCALE            = 0xFE
        self.__CHANNEL_ON_L        = 0x06
        self.__CHANNEL_ON_H        = 0x07
        self.__CHANNEL_OFF_L       = 0x08
        self.__CHANNEL_OFF_H       = 0x09
        self.__SLEEP               = 0x10
        self.__OSC_CLOCK           = 25000000.0 #25 MHz
        self.set_frequency(50)

    def set_pwm(self, start, end):
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_ON_L + 4 * self._channel, start & 0xFF)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_ON_H + 4 * self._channel, start >> 8)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_OFF_L + 4 * self._channel, end & 0xFF)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__CHANNEL_OFF_H + 4 * self._channel, end >> 8)

    def set_frequency(self, frequency):
        prescale_value = int(round(self.__OSC_CLOCK / (4096 * frequency)) - 1)
        mode1 = self._bus.read_byte_data(self.__I2C_ADDRESS, self.__MODE1)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__MODE1, mode1 | self.__SLEEP)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__PRESCALE, prescale_value)
        self._bus.write_byte_data(self.__I2C_ADDRESS, self.__MODE1, mode1 & ~self.__SLEEP)

    def center(self):
        self.set_pwm(0, self.__CENTER)

    def right(self):
        self.set_pwm(0, self.__CENTER - self.__STEP)

    def left(self):
        self.set_pwm(0, self.__CENTER + self.__STEP)
        