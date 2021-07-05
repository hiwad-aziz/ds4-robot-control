# Dualshock 4 Robot Control
A python script to control a 4-wheel skid steer robot with a PlayStation 4 Dualshock 4 controller and a Raspberry Pi 4 which I use to test mapping algorithms on a mobile robot.

## Dependencies
[RPi.GPIO](https://pypi.org/project/RPi.GPIO/)
[pyPS4Controller](https://pypi.org/project/pyPS4Controller/)

## Hardware Parts
 - Raspberry Pi 4
 - L298N motor driver

## Setup
Set the appropriate values at the top of the script. In my setup, two motors from the same side are connected to one output on the L298N.
Connect your PS4 controller to the Raspberry Pi and run the script.
