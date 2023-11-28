#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import random

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

#in case robot intends to move forward continously
counter_forward = 0


#in case robot is stuck in an endless loop
time_counter = 0

def wander():
    global time_counter
    global counter_forward
    ev3.speaker.say("Wander")
    #robot should wander around until it detects an obstacle/fire
    while(True):
        #robot should turn right every 1000 ms
        #if(time_counter > 1000):
         #   turn_right(gyro)
          #  time_counter = 0
        #time_counter += 10
        #robot should move forward until it detects an obstacle/fire
        if(TouchSensor1.pressed() or TouchSensor2.pressed()):
            #if robot detects obstacle, call wall_following()
            move_reverse(1200)
            mylist = ["left", "right"]
            random.shuffle(mylist)
            i = random.randint(0,1)
            if(mylist[i] == "left"):
                turn_left(1200)
                if(UltrasonicSensor.distance() <= 60):
                    wall_following()
            else:
                turn_right(1200)
                if(UltrasonicSensor.distance() <= 60):
                    wall_following()
            move_reverse(800)
       
        else:
            if((ColorSensor.color()==Color.BLUE and ColorSensor.reflection() > 30) or (ColorSensor.color()==Color.WHITE and ColorSensor.reflection() > 30)):
                print("color detected")
                #ev3.speaker.say("detects yellow color")
                find_fire()
            
            if(UltrasonicSensor.distance() < 60):
                wall_following()
            else:
                mylist = ["forward", "left", "right"]
                random.shuffle(mylist)
                
                if(mylist[0] == "forward"):
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 60):
                        wall_following()
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 60):
                        wall_following()

                    
                elif(mylist[0] == "left"):
                    turn_left(700)
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                else:
                    turn_right(700)
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                if(mylist[1] == "forward"):
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                elif(mylist[1] == "left"):
                    turn_left(700)
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                else:
                    turn_right(700)
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                
                if(mylist[2] == "forward"):
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                elif(mylist[2] == "left"):
                    turn_left(700)
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                else:
                    turn_right(700)
                    move_forward(1000)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                  
        if((ColorSensor.color()==Color.BLUE and ColorSensor.reflection() > 30) or (ColorSensor.color()==Color.WHITE and ColorSensor.reflection() > 30)):
                print("color detected")
                #ev3.speaker.say("detects yellow color")
                find_fire()
    return



def wall_following():
    global time_counter
    global counter_forward
    ev3.speaker.say("Wall Follower")
    #robot should follow the wall until it detects another wall or freespace
    #if it detects another wall, it should turn left and call wander()
    #if it detects freespace, it should call wander()
    # 
    #initialize the target distance from wall, should be 10
    Target_distance = 100
    #while the robot is 10 cm from the wall, move forward and maintain that distance
    while(True):
        time_counter = time_counter + 10
        # Note: Might need adjustments based on the platform in the lab
        # if the robot is more than 11 cm from the wall, adjust the robot to move closer to the wall
        if(UltrasonicSensor.distance() >= 150):
            wander()
        elif(UltrasonicSensor.distance() == Target_distance):
            move_forward_wall(100)
            #if the robot detects hits a wall, turn left and continue wall following
            if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                move_reverse(1200)
                mylist = ["left", "right"]
                random.shuffle(mylist)
                i = random.randint(0,1)
                if(mylist[i] == "left"):
                    turn_left(1200)
                    move_check(100)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                        move_reverse(1200)
                        turn_left(2400)
                else:
                    turn_right(1200)
                    move_check(100)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                        move_reverse(1200)
                        turn_left(2400)
        elif(UltrasonicSensor.distance() < Target_distance):
            if(UltrasonicSensor.distance() < 40):
                turn_left_wall(0)
                move_forward_wall(100)
                turn_right(200)
            move_forward_wall(100)
            #if the robot detects hits a wall, turn left and continue wall following
            if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                move_reverse(1200)
                mylist = ["left", "right"]
                random.shuffle(mylist)
                i = random.randint(0,1)
                if(mylist[i] == "left"):
                    turn_left(1200)
                    move_check(100)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                        move_reverse(1200)
                        turn_left(2400)
                else:
                    turn_right(1200)
                    move_check(100)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                        move_reverse(1200)
                        turn_left(2400)


        
        else: #UltrasonicSensor.distance() > Target_distance
            turn_right_wall(100)
            move_forward_wall(100)
            move_forward_wall(100)
            turn_left_wall(100)
            #if the robot detects hits a wall, turn left and continue wall following
            if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                move_reverse(1200)
                mylist = ["left", "right"]
                random.shuffle(mylist)
                i = random.randint(0,1)
                if(mylist[i] == "left"):
                    turn_left(1200)
                    move_check(100)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                        move_reverse(1200)
                        turn_left(2400)
                else:
                    turn_right(1200)
                    move_check(100)
                    if(UltrasonicSensor.distance() <= 100):
                        wall_following()
                    if(TouchSensor1.pressed() or TouchSensor2.pressed()):
                        move_reverse(1200)
                        turn_left(2400)

        #after a certain amount of time, turn right and call wander()
        if(time_counter > 1000):
            time_counter = 0
            turn_right(1200)
            move_forward(1000)
            wander()
    return


# Note: These two functions are redundant. You can combine them as one function because the robot will stop as soon as the
#           color sensor detects the color we choose
def find_fire():
    #robot should find the fire and extinguish it
    #use color sensor to detect fire
    while(ColorSensor.color() == Color.BLUE and ColorSensor.reflection() > 50) or(ColorSensor.color() == Color.WHITE and ColorSensor.reflection() > 50):
        extinguish_fire()
        if(ColorSensor.color() != Color.BLUE and ColorSensor.color() != Color.WHITE):
            ev3.speaker.say("Mission Success")
            quit()
            
    return

def extinguish_fire():
    #robot should extinguish fire
    #not sure what to do here (maybe use a motor for a fan?)
    #play a sound when fire is extinguished
    ev3.speaker.beep()
    ev3.speaker.say("Fire Detected")
    #if robot detects the candle, it should run it over and say "Mission Success"
    move_fan()

#motor functions
def move_forward(time):
    global counter_forward
    temp_time = 100
    if(counter_forward > 4):
        counter_forward = 0
        mylist = ["left", "right"]
        random.shuffle(mylist)
        i = random.randint(0,1)
        if(mylist[i] == "left"):
            turn_left(1200)
            if(UltrasonicSensor.distance() <= 60):
                wall_following()
            return
        else:
            turn_right(1200)
            if(UltrasonicSensor.distance() <= 60):
                wall_following()
            return

    time = 1000
    left_motor.run_time(350, time, Stop.HOLD, False)
    right_motor.run_time(350, time, Stop.HOLD, True)
    counter_forward = counter_forward + 1
    if((ColorSensor.color()==Color.BLUE and ColorSensor.reflection() > 30) or (ColorSensor.color()==Color.WHITE and ColorSensor.reflection() > 30)):
            print("color detected")
            #ev3.speaker.say("detects yellow color")
            find_fire()    

    if(TouchSensor1.pressed() or TouchSensor2.pressed()):
            #if robot detects obstacle, call wall_following()

        move_reverse(1200)
        mylist = ["left", "right"]
        random.shuffle(mylist)
        i = random.randint(0,1)
        if(mylist[i] == "left"):
            turn_left(1200)
            if(UltrasonicSensor.distance() <= 60):
                wall_following()
        else:
            turn_right(1200)
            if(UltrasonicSensor.distance() <= 60):
                wall_following()
        move_reverse(800)
            

def move_forward_wall(time):
    if((ColorSensor.color()==Color.BLUE and ColorSensor.reflection() > 30) or (ColorSensor.color()==Color.WHITE and ColorSensor.reflection() > 30)):
            print("color detected")
            #ev3.speaker.say("detects yellow color")
            find_fire() 
    else:
        time = 500
        left_motor.run_time(150, time, Stop.HOLD, False)
        right_motor.run_time(150, time, Stop.HOLD, True)

    
def move_reverse(time):
    if((ColorSensor.color()==Color.BLUE and ColorSensor.reflection() > 30) or (ColorSensor.color()==Color.WHITE and ColorSensor.reflection() > 30)):
            print("color detected")
            #ev3.speaker.say("detects yellow color")
            find_fire() 
    else:        
        #time = 1000
        left_motor.run_time(-200, time, Stop.HOLD, False)
        right_motor.run_time(-200, time, Stop.HOLD, True)

def move_check(time):
    time = 1000
    left_motor.run_time(150, time, Stop.HOLD, False)
    right_motor.run_time(150, time, Stop.HOLD, True)
    

# Turns the robot ~30 degrees couterclockwise (left)
def turn_left(time):
    if((ColorSensor.color()==Color.BLUE and ColorSensor.reflection() > 30) or (ColorSensor.color()==Color.WHITE and ColorSensor.reflection() > 30)):
            print("color detected")
            #ev3.speaker.say("detects yellow color")
            find_fire() 
    else:
        #time = 700
        left_motor.run_time(-150, time, Stop.HOLD, False)
        right_motor.run_time(150, time, Stop.HOLD, True)

# Turns the robot ~30 degrees clockwise (right)
def turn_right(time):
    if((ColorSensor.color()==Color.BLUE and ColorSensor.reflection() > 30) or (ColorSensor.color()==Color.WHITE and ColorSensor.reflection() > 30)):
            print("color detected")
            #ev3.speaker.say("detects yellow color")
            find_fire() 
    else:
        #time = 700
        left_motor.run_time(150, time, Stop.HOLD, False)
        right_motor.run_time(-150, time, Stop.HOLD, True)

# Turns the robot ~15 degrees couterclockwise (left)
def turn_right_wall(time):
    time = 300
    left_motor.run_time(150, time, Stop.HOLD, False)
    right_motor.run_time(-150, time, Stop.HOLD, True)

# Turns the robot ~15 degrees couterclockwise (left)
def turn_left_wall(time):
    time = 300
    left_motor.run_time(-150, time, Stop.HOLD, False)
    right_motor.run_time(150, time, Stop.HOLD, True)


# Turns on fan motor to "blow out" fire
def move_fan():
    fan_motor.run_time(10000, 5000, Stop.HOLD, False)


#initialize the EV3 Brick
ev3 = EV3Brick()
#initialize the motors (change port letters to match your robot)
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
fan_motor = Motor(Port.C)

#initialize the sensors (change port letters to match your robot)
UltrasonicSensor = UltrasonicSensor(Port.S1)
ColorSensor = ColorSensor(Port.S2)
TouchSensor1 = TouchSensor(Port.S3)
TouchSensor2 = TouchSensor(Port.S4)

wander()
