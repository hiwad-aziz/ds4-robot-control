import RPi.GPIO as GPIO


class Motor:
    """
    Provides an interface to a DC motor connected to an L298N motor driver.

    :param int forward:
        The GPIO pin that the forward input of the motor driver chip is
        connected to.

    :param int backward:
        The GPIO pin that the backward input of the motor driver chip is
        connected to.

    :param int pwm:
        The GPIO pin that the PWM pin of the motor driver chip is
        connected to.
    """

    def __init__(self, forward_pin, backward_pin, pwm):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(forward_pin, GPIO.OUT)
        GPIO.setup(backward_pin, GPIO.OUT)
        GPIO.setup(pwm, GPIO.OUT)
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        self.pwm_pin = pwm
        self.pwm = GPIO.PWM(pwm, 100)

    def __del__(self):
        if not self.pwm == None:
            self.pwm.stop()
            GPIO.cleanup(self.pwm_pin)
        if not self.forward_pin == None:
            GPIO.cleanup(self.forward_pin)
        if not self.backward_pin == None:
            GPIO.cleanup(self.backward_pin)

    def forward(self, speed):
        GPIO.output(self.forward_pin, GPIO.HIGH)
        GPIO.output(self.backward_pin, GPIO.LOW)
        self.pwm.start(speed * 100)

    def backward(self, speed):
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.HIGH)
        self.pwm.start(speed * 100)

    def stop(self):
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.LOW)
        self.pwm.stop()
