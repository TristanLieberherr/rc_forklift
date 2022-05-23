import RPi.GPIO as GPIO



class DCMotor:
    def __init__(self, pin1, pin2):
        self._pin1 = pin1
        self._pin2 = pin2
        GPIO.setup(self._pin1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self._pin2, GPIO.OUT, initial=GPIO.LOW)
    
    def stop(self):
        GPIO.output(self._pin1, GPIO.LOW)
        GPIO.output(self._pin2, GPIO.LOW)

    def forward(self):
        GPIO.output(self._pin1, GPIO.LOW)
        GPIO.output(self._pin2, GPIO.HIGH)

    def reverse(self):
        GPIO.output(self._pin1, GPIO.HIGH)
        GPIO.output(self._pin2, GPIO.LOW)
