#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

def climb_up_stairs():
    num_steps = 0
    # Keep running until it is on top of the stairs
    while not color_sensor.color() == Color.BLUE:
        front_motor.run(150)
        back_motor.run(140)

        # Keep running motors until robot is at an angle
        while gyro.angle() < 10:
            wait(10)
        
        # Run lift motor
        lift_motor.run(140)
        front_motor.run(70)
        back_motor.run(50)

        # Keep moving rear structure up until robot is at an angle close to 0
        while not gyro.angle() < -3:
            if (touch_sensor.pressed()):
                break
            wait(10)
        lift_motor.hold()

        # Move the robot forward for some time
        front_motor.run(130)
        back_motor.run(150)
        wait(1000)

        # Keep moving forward slowly and adjust lift
        front_motor.run(100)
        back_motor.run(100)
        lift_motor.run_target(160, 0)
        num_steps += 1

    front_motor.hold()
    back_motor.hold()
    return num_steps


def climb_down_stairs(num_steps):
    # Keep running until bottom of stairs has been reached
    while num_steps > 0:
        front_motor.run(-130)
        back_motor.run(-120)

        # Keep running motors until robot is at an angle (tilt angle has to be lower to avoid falling)
        while gyro.angle() < 5:
            wait(10)

        # Run lift motor
        lift_motor.run(-140)
        front_motor.run(-70)
        back_motor.run(-50)

        # Keep moving the rear structure up until robot is at an angle close to 0
        while not gyro.angle() < -2:
            if (touch_sensor.pressed()):
                break
            wait(10)
        lift_motor.hold()

        # Move the robot forward for some time
        front_motor.run(-100)
        back_motor.run(-130)
        wait(1000)

        # Keep moving forward slowly and adjust lift
        front_motor.run(-80)
        back_motor.run(-80)
        lift_motor.run_target(-160, 0)
        num_steps -= 1
    
    front_motor.hold()
    back_motor.hold()
    return

# Instantiate motors that will control the wheels and lift of the robot
front_motor = Motor(Port.B, Direction.COUNTERCLOCKWISE)
back_motor = Motor(Port.A, Direction.CLOCKWISE)
lift_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE, [8, 24, 40])

# Instantiate all sensors used for stair climbing up and down
gyro = GyroSensor(Port.S2)
touch_sensor = TouchSensor(Port.S3)
color_sensor = ColorSensor(Port.S4)

# Initialize stair climbing behaviors
steps = climb_up_stairs()
climb_down_stairs(steps)
