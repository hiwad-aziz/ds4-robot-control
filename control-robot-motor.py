from pyPS4Controller.controller import Controller
import RPi.GPIO as GPIO

# DS4 stick values are in the range [-32767, 32767].
# This variable is necessary to calculate the turn factors.
DS4_VALUE_OFFSET = 32767
# GPIO Pins (BCM)
LEFT_MOTORS_FORWARD_PIN = 23
LEFT_MOTORS_BACKWARD_PIN = 24
LEFT_MOTORS_PWM_PIN = 12
RIGHT_MOTORS_FORWARD_PIN = 22
RIGHT_MOTORS_BACKWARD_PIN = 27
RIGHT_MOTORS_PWM_PIN = 13


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
    """
    Provides an interface to control a 4-wheeled skid steer robot with a Dualshock 4 controller.

    Controls:
        - L2: drive forward
        - R2: drive backward
        - L1: turn left in place
        - R1: turn right in place
        - L3: adjust turn factor
    """

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.left_motors = Motor(
            forward_pin=LEFT_MOTORS_FORWARD_PIN, backward_pin=LEFT_MOTORS_BACKWARD_PIN, pwm=LEFT_MOTORS_PWM_PIN)
        self.right_motors = Motor(forward_pin=RIGHT_MOTORS_FORWARD_PIN,
                                  backward_pin=RIGHT_MOTORS_BACKWARD_PIN, pwm=RIGHT_MOTORS_PWM_PIN)
        self.speed_left = 1
        self.speed_right = 1
        self.turn_factor_left = 1
        self.turn_factor_right = 1

    def __stop(self):
        self.left_motors.stop()
        self.right_motors.stop()

    def __reset_speed(self):
        self.speed_left = 1
        self.speed_right = 1

    def __reset_turn_factors(self):
        self.turn_factor_left = 1
        self.turn_factor_right = 1

    def on_R2_press(self, value):
        self.speed_left = abs(value) / DS4_VALUE_OFFSET * self.turn_factor_left
        self.speed_right = abs(value) / DS4_VALUE_OFFSET * \
            self.turn_factor_right
        self.left_motors.forward(self.speed_left)
        self.right_motors.forward(self.speed_right)

    def on_R2_release(self):
        self.__stop()

    def on_L2_press(self, value):
        self.speed_left = abs(value) / DS4_VALUE_OFFSET * self.turn_factor_left
        self.speed_right = abs(value) / DS4_VALUE_OFFSET * \
            self.turn_factor_right
        self.left_motors.backward(self.speed_left)
        self.right_motors.backward(self.speed_right)

    def on_L2_release(self):
        self.__stop()

    def on_L3_right(self, value):
        self.turn_factor_left = 1
        self.turn_factor_right = (
            DS4_VALUE_OFFSET - abs(value)) / DS4_VALUE_OFFSET

    def on_L3_left(self, value):
        self.turn_factor_left = (
            DS4_VALUE_OFFSET - abs(value)) / DS4_VALUE_OFFSET
        self.turn_factor_right = 1

    def on_L3_release(self):
        self.__reset_turn_factors()

    def on_R1_press(self):
        self.__reset_speed()
        self.left_motors.forward(self.speed_left)
        self.right_motors.backward(self.speed_right)

    def on_R1_release(self):
        self.__stop()

    def on_L1_press(self):
        self.__reset_speed
        self.left_motors.backward(self.speed_left)
        self.right_motors.forward(self.speed_right)

    def on_L1_release(self):
        self.__stop()


controller = RobotController(
    interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen(timeout=60)
