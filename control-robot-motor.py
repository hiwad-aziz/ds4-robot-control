from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO


DS4_VALUE_OFFSET = 32767


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


class RobotController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.left_motors = Motor(forward_pin=23, backward_pin=24, pwm=12)
        self.right_motors = Motor(forward_pin=22, backward_pin=27, pwm=13)
        self.speed_left = 1
        self.speed_right = 1
        self.turn_factor_left = 1
        self.turn_factor_right = 1

    def on_R2_press(self, value):
        self.speed_left = abs(value) / DS4_VALUE_OFFSET * self.turn_factor_left
        self.speed_right = abs(value) / DS4_VALUE_OFFSET * \
            self.turn_factor_right
        self.left_motors.forward(self.speed_left)
        self.right_motors.forward(self.speed_right)

    def on_R2_release(self):
        self.left_motors.stop()
        self.right_motors.stop()

    def on_L2_press(self, value):
        self.speed_left = abs(value) / DS4_VALUE_OFFSET * self.turn_factor_left
        self.speed_right = abs(value) / DS4_VALUE_OFFSET * \
            self.turn_factor_right
        self.left_motors.backward(self.speed_left)
        self.right_motors.backward(self.speed_right)

    def on_L2_release(self):
        self.left_motors.stop()
        self.right_motors.stop()

    def on_L3_right(self, value):
        self.turn_factor_left = 1
        self.turn_factor_right = (
            DS4_VALUE_OFFSET - abs(value)) / DS4_VALUE_OFFSET

    def on_L3_left(self, value):
        self.turn_factor_left = (
            DS4_VALUE_OFFSET - abs(value)) / DS4_VALUE_OFFSET
        self.turn_factor_right = 1

    def on_L3_release(self):
        self.turn_factor_left = 1
        self.turn_factor_right = 1


controller = RobotController(
    interface="/dev/input/js0", connecting_using_ds4drv=False)
# you can start listening before controller is paired, as long as you pair it within the timeout window
controller.listen(timeout=60)