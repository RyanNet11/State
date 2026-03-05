from vex import *
import math
import json as js
# Brain should be defined by default
brain=Brain()
brain.sdcard
joystick = Controller()
brain.screen.print("Hello V5")

#######################################
#          initate sensors            #
#######################################
class DistanceSensors:
    def __init__(self,name, device, offset, angle_deg):
        self.device = device
        self.offset = offset    # offset in inches
        self.angle = angle_deg  # Convert to radians for math
        self.name = name        # Name of the sensor
        
    def get_computed_hypotunuse_Back(self):
        # here i'm trying to compute the leg of the triangle made with the 
        # distance sensor, the center of rotation, and the walls detected
        theta = 0 
        h = IMU.heading() 
        
        # compute theta based off current angle 
        
        if 45 < h < 135:
            theta = abs(90-h)
        elif 135 < h < 225:
            theta = abs(180-h)
        elif 225 < h < 315:
            theta = abs(270-h)
        elif 315 < h < 45:
            theta = abs(0-h)
        
        #calculate the leg using the formula ((distance + offset) multiplied by cos(theta))
        hyp = (math.cos(math.radians(theta)) * (self.device.object_distance(INCHES) + self.offset))
        return  hyp
    
    def backSensorLegTest(self):
        # a test to see if my math above was wrong. I should only need cos to do this
        leg = abs(math.cos(math.radians(360-IMU.heading())) * (self.offset + self.device.object_distance(INCHES))) + abs(math.sin(math.radians(360-IMU.heading())) * (self.offset + self.device.object_distance(INCHES)))
        return leg


joystick = Controller(ControllerType.PRIMARY)
ocholer = Controller(ControllerType.PARTNER)

intakeFront = Motor(Ports.PORT1)
intakeBack = Motor(Ports.PORT10, False)
intakeTop = Motor(Ports.PORT8)



motors = [intakeBack, intakeFront, intakeTop]  




RF = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
RB = Motor(Ports.PORT14, GearSetting.RATIO_6_1, False)
RS = Motor(Ports.PORT15,GearSetting.RATIO_18_1, False) #small motor port 10


LF = Motor(Ports.PORT11, GearSetting.RATIO_6_1, True)
LB = Motor(Ports.PORT12, GearSetting.RATIO_6_1, True)
LS = Motor(Ports.PORT13,GearSetting.RATIO_18_1, True) # small motor port 3

intakeFront = Motor(Ports.PORT1)
intakeBack = Motor(Ports.PORT10, False)

LeftMotors = MotorGroup(LF, LB, LS)
RightMotors = MotorGroup(RF, RB, RS)

IMU = Inertial(Ports.PORT17)
IMU.calibrate()

piston = Pneumatics(brain.three_wire_port.a)

odomx = Rotation(Ports.PORT16)
odomy = Rotation(Ports.PORT8)
def driveForwardTest(inches):
    degrees = inches /3 * 4 / 2.75 /3.14159 * 360
    
    LeftMotors.spin_to_position(degrees, DEGREES, 300, RPM, False)
    RightMotors.spin_to_position(degrees, DEGREES, 300, RPM, True)
def stopMotors():
    LeftMotors.stop()
    RightMotors.stop()
    
def distanceTracker(): #returns distance in inches
    global current, previous
    # Wheel diamiter is 2'' 
    # One rotation will be 2 pi inches traveled
    # pervious = current
    # current = odomx.angle
    # if current < pervious:
    #     distance = current 
    # distance = 3.1415 * 
    # 360 degrese is 2 pi inches traveled. Every egree is going to be (2pi)/360 inches travled
    
    ####### Motor Drive Sensor finder##########
    # 1 rotation of the 36 tooth is 360 degrees
    while(True):
        distance = odomL.position() * 3.1415 / 360 * 2
        brain.screen.set_cursor(1,1)
        brain.screen.print(odomL.position())
        brain.screen.new_line()
        brain.screen.print(distance)
        wait(20)
    

    while(True):
        brain.screen.set_cursor(1,1)
        brain.screen.print(LB.position(), RB.position())
        
        distanceL = (LB.position() + LS.position()) *3/4 *2.75 *3.14159 / 360 / 2
    
        distanceR = (RB.position() + RS.position()) *3/4 *2.75 *3.14159 / 360 / 2
        
        brain.screen.set_cursor(2,1)
        brain.screen.print(distanceL, distanceR)
        brain.screen.set_cursor(3,1)
        average = (distanceL + distanceR) / 2
        brain.screen.print(average)
        joystick.screen.clear_line(3)
        joystick.screen.print(average)
        

        wait(20)
        
def segmentedMotorDistanceTracker():
    while(True):
        distanceLB = (LB.position()) *3/4 *2.75 *3.14159 / 360 
        brain.screen.set_cursor (6, 1)
        
        brain.screen.print(distanceLB)
        distanceLS = (LS.position()) *3/4 *2.75 *3.14159 / 360 * 3
        brain.screen.set_cursor (3, 1)
        brain.screen.print(distanceLS)
        distanceLF = ( LF.position()) *3/4 *2.75 *3.14159 / 360 
        brain.screen.set_cursor (1, 1)
        brain.screen.print(distanceLF)


        distanceRB = (RB.position()) *3/4 *2.75 *3.14159 / 360 
        brain.screen.set_cursor (6, 8)
        brain.screen.print(distanceRB)
        distanceRS = (RS.position()) *3/4 *2.75 *3.14159 / 360 * 3
        brain.screen.set_cursor (3, 8)
        brain.screen.print(distanceRS)
        distanceRF = (RF.position()) *3/4 *2.75 *3.14159 / 360 
        brain.screen.set_cursor (1, 8)
        brain.screen.print(distanceRF)
        
        #average it all
        distanceL = ((distanceLF + distanceLB + distanceLS) / 3)
        distanceR = ((distanceRF + distanceRB + distanceRS) / 3)
        brain.screen.set_cursor(1,16)
        brain.screen.print(distanceL)
        brain.screen.set_cursor(1,22)
        brain.screen.print(distanceR)

        

        wait(20)

 
def incrementalDrive(x):
    degrees = x / 3 * 4 / 2.75 /3.14159 * 360
    stepSize = 5
    maxRPM = 200
    accel_ratio = 0.2 #20%
    
    accel_deg = degrees * accel_ratio
    decel_start = degrees - accel_deg
    
    current = 0 
    LeftMotors.set_position(0)
    RightMotors.set_position(0)
    
    while current < degrees: 
        if current < accel_deg: 
            speed = maxRPM * (current/ accel_deg)
        elif current > decel_start: 
            speed = maxRPM * ((degrees - current) / accel_deg)
        else: 
            speed = maxRPM
        
        speed = max(speed, 30)
        
        LeftMotors.spin(FORWARD, speed, RPM)
        RightMotors.spin(FORWARD, speed, RPM)
        time.sleep(0.2)
        current = abs(LeftMotors.position(DEGREES))
    LeftMotors.stop(BRAKE)
    RightMotors.stop(BRAKE)
def driveUntilStopped(inches):
    degrees = inches / 3 * 4 / 2.75 /3.14159 * 360
    
    LeftMotors.spin_to_position(degrees, DEGREES, 300, RPM, False)
    RightMotors.spin_to_position(degrees, DEGREES, 300, RPM, False)
    while(True):
        if LeftMotors.is_spinning() == True:
            brain.screen.set_cursor(5,30)
            brain.screen.print(LeftMotors.torque())
            if LeftMotors.torque() > 0.1:
                stopMotors()
        else:
            break
j = brain.sdcard.loadfile("MotorTempsOverTime.txt")
brain.sdcard.loadfile('mostyn.bmp')
if brain.sdcard.exists("mostyn.bmp"):
    brain.screen.print("we have a file")
brain.screen.clear_screen()
brain.screen.draw_image_from_file("mostyn.bmp", 0, 0)

def intake():
    while(True):
            if joystick.buttonL1.pressing() == True:
                intakeFront.spin(FORWARD, 80, PERCENT)
                intakeBack.spin(REVERSE, 80, PERCENT)
                
            elif joystick.buttonR1.pressing() == True:
                intakeFront.spin(REVERSE, 80, PERCENT)
                intakeBack.spin(FORWARD, 80, PERCENT)
                
            elif joystick.buttonL2.pressing() == True:
                intakeFront.spin(FORWARD, 80, PERCENT)
                intakeBack.spin(FORWARD, 0, PERCENT)
                
            elif joystick.buttonR2.pressing() == True:
                intakeFront.spin(REVERSE, 80, PERCENT)
                intakeBack.spin(FORWARD, 0, PERCENT)

            else:
                intakeFront.spin(FORWARD, 0, PERCENT)
                intakeBack.spin(FORWARD, 0, PERCENT)
                
                
            
def DrivePID(target_inches):
    # Reset sensors
    odomx.reset_position()
    LeftMotors.set_position(0, DEGREES)
    RightMotors.set_position(0, DEGREES)

    # PID constants
    kP = 0.40
    kI = 0.0004
    kD = 0.001

    error = 0
    prevError = 0
    integral = 0
    derivative = 0

    # Conversion factors
    # ODOMETRY: 2.0" wheel → 1 rotation = 6.283 in
    odo_wheel_diam = 2.0
    odo_inch_per_deg = (math.pi * odo_wheel_diam) / -360

    # DRIVE MOTORS: 36:48 gear ratio, 2.75" wheel
    # 360 36 tooth to 48 tooth to 2 inches 
    motor_inch_per_deg = 0.1180

    brain.screen.clear_screen()
    brain.screen.print("Driving PID...")

    while True:
        # Distance from odometry
        odo_distance = odomx.position() * odo_inch_per_deg

        # Distance from motor encoders
        left_deg = LeftMotors.position(DEGREES)
        right_deg = RightMotors.position(DEGREES)
        avg_motor_distance = ((left_deg + right_deg) / 2) * motor_inch_per_deg

        # Weighted blend: trust odometry more (70/30)
        current_distance = (odo_distance * 0.7) + (avg_motor_distance * 0.3)

        # PID math
        error = target_inches - odo_distance
        integral += error
        derivative = error - prevError
        prevError = error

        power = (kP * error) + (kI * integral) + (kD * derivative)

        # Clamp motor power
        power = max(min(power, 100), -100)

        # Apply to motors
        LeftMotors.spin(FORWARD, power, PERCENT)
        RightMotors.spin(FORWARD, power, PERCENT)

        # Display debug info
        brain.screen.set_cursor(1, 1)
        brain.screen.print("Dist: {:.2f} in".format(current_distance))
        brain.screen.new_line()
        brain.screen.print("odom: {:.2f}".format(odo_distance))
        brain.screen.new_line()
        brain.screen.print("Moten: {:.2f}".format(avg_motor_distance))

        # Stop when close enough (0.5 inch tolerance)
        if abs(error) < 0.5:
            LeftMotors.stop(BRAKE)
            RightMotors.stop(BRAKE)
            break

        wait(20, MSEC)
 
def DrivePIDTest(distance):
    odomx.reset_position()
    
    # 1.3333
    current = 0
    prev = 0
    while(True):
        prev = current
        current = odomx.position() * 3.1415 / 360 * 2 # omni odom conversion
        if current < distance:
            difference = distance-current
        elif current > difference:
            difference = distance + current
        difference = distance - current
        brain.screen.set_cursor(1,1)
        brain.screen.print(current)
        brain.screen.new_line()
        brain.screen.print(difference)
        joystick.screen.clear_line(3)
        joystick.screen.print(difference)

        if difference > 15:
            LeftMotors.spin(FORWARD, 50, PERCENT)
            RightMotors.spin(FORWARD, 50, PERCENT)
        elif difference > 2:
            LeftMotors.spin(FORWARD, 30, PERCENT)
            RightMotors.spin(FORWARD, 30, PERCENT)
        elif difference > 1:
            LeftMotors.spin(FORWARD, 0 , PERCENT)
            RightMotors.spin(FORWARD, 0 , PERCENT)
        elif difference < 1:
            LeftMotors.spin(REVERSE, 0 , PERCENT)
            RightMotors.spin(REVERSE, 0 , PERCENT)
        elif difference < 2:
            LeftMotors.spin(REVERSE, 30 , PERCENT)
            RightMotors.spin(REVERSE, 30 , PERCENT)
        wait(1)
def turn_pid(target):
    # error = 0
    # prev_error = 0
    brain.screen.print("starting turn")
    while(True):
        difference = IMU.heading() - target
        error = difference * kP
        if error > 100:
            error = 100
        if difference > threashold:
            LeftMotors.set_velocity(0, PERCENT)
            RightMotors.set_velocity(0, PERCENT)

            break

        LeftMotors.set_velocity(-error, PERCENT)
        RightMotors.set_velocity(error, PERCENT)
        wait(10)
def distance_traveled():
    odomx.changed
    
    

def AutonMid():
    turnheadingRight(-55)
    
    intakeFront.spin(REVERSE, 90, PERCENT)
    intakeBack.spin(FORWARD, 80, PERCENT)
    
    travelOnHeading(19, -55)
    travelOnHeading(7,-55)
    turnheading(45)
    intakeFront.spin(REVERSE, 0, PERCENT)
    intakeBack.spin(FORWARD, 0, PERCENT)
    travelOnHeading(13,45)
    intakeFront.spin(REVERSE, 90, PERCENT)
    intakeBack.spin(FORWARD, 80, PERCENT)
    intakeTop.spin(REVERSE, 80, PERCENT)
    wait(2, SECONDS)
    travelOnHeading(-6,45)
    intakeFront.spin(REVERSE, 0, PERCENT)
    intakeBack.spin(FORWARD, 0, PERCENT)
    intakeTop.spin(REVERSE, 0, PERCENT)
    turnheading(90)
    travelOnHeading(36,90)
    
    
    # travelOnHeading(10, IMU.heading())
    # EatBlock(30)   
    # intakeFront.spin(REVERSE, 0, PERCENT)
    # intakeBack.spin(FORWARD, 0, PERCENT)
    # turnheading(45)
    # TravelWithDistanceSensorOnHeading(distanceL, 12.5, 45)
    # intakeFront.spin(REVERSE, 90, PERCENT)
    # intakeBack.spin(FORWARD, 80, PERCENT)
    # intakeTop.spin(REVERSE, 80, PERCENT)
    # wait(2,SECONDS)
    # travelOnHeading(-2, 45)
    # turnheading(90)
    # travelOnHeading(36, 90)
    # EatBlock(16)
    
def Skills():
    travelOnHeading(38,0)
    turnheading(-90)
    toungepneumatics()
    travelOnHeading(18,-90)
    intakeFront.spin(REVERSE, 90, PERCENT)
    intakeBack.spin(FORWARD, 80, PERCENT)


def systemTime():
    while(True):
        brain.screen.set_cursor(2,1)
        brain.screen.print(Timer.system()) # prints program time
def go():
    LeftMotors.spin(FORWARD, 100, PERCENT)
    RightMotors.spin(FORWARD, 100, PERCENT)

    # ### Prints the motor temps in a square ###
    
    #     brain.screen.set_cursor(5,10)
    #     brain.screen.print(LF.temperature(TemperatureUnits.FAHRENHEIT))
    #     brain.screen.set_cursor(5,20)
    #     brain.screen.print(RF.temperature(TemperatureUnits.FAHRENHEIT))
    #     brain.screen.set_cursor(10,10)
    #     brain.screen.print(LB.temperature(TemperatureUnits.FAHRENHEIT))
    #     brain.screen.set_cursor(10,20)
    #     brain.screen.print(RB.temperature(TemperatureUnits.FAHRENHEIT))
        
    # ## Prints the motor efficiency in a square ###
    
    #     brain.screen.set_cursor(6,10)
    #     brain.screen.print(LF.velocity())
    #     brain.screen.set_cursor(6,20)
    #     brain.screen.print(RF.velocity())
    #     brain.screen.set_cursor(11,10)
        
    #     brain.screen.print(LB.velocity())
    #     brain.screen.set_cursor(11,20)
    #     brain.screen.print(RB.velocity())
        
    # ### print motor temps on the contorller ###
    #     joystick.screen.clear_screen()
    #     joystick.screen.set_cursor(1,0)
    #     joystick.screen.print(round(LF.temperature()))
    #     joystick.screen.set_cursor(1,7)
    #     joystick.screen.print(round(RF.temperature()))
        
    #     joystick.screen.set_cursor(1,14)
    #     joystick.screen.print(round(LF.velocity()))
    #     joystick.screen.set_cursor(1,19)
    #     joystick.screen.print(round(RF.velocity()))
        
    #     joystick.screen.set_cursor(2,0)
    #     joystick.screen.print(round(LB.temperature()))
    #     joystick.screen.set_cursor(2,7)
    #     joystick.screen.print(round(RB.temperature()))
        
    #     joystick.screen.set_cursor(2,14)
    #     joystick.screen.print(round(LB.velocity()))
    #     joystick.screen.set_cursor(2,19)
    #     joystick.screen.print(round(RB.velocity()))
    #     wait(120) 

        if IMU.installed() != True:
            brain.screen.print("IMU ")
            joystick.screen.print("IMU ")
             
        if intakeFront.installed() != True:
            brain.screen.print("INTF ")
            joystick.screen.print("INTF ")
            
        if intakeBack.installed() != True:
            brain.screen.print("INTB ")
            joystick.screen.print("INTB ")
             
        if LB.installed() != True:
            brain.screen.print("LB ")
            joystick.screen.print("LB  ")
             
            
        if LF.installed() != True:
            brain.screen.print("LF ")
            joystick.screen.print("LF  ")
            
        if LS.installed() != True:
            brain.screen.print("LS ")
            joystick.screen.print("LS  ")
             
            
        if RB.installed() != True:
            brain.screen.print("RB ")
            joystick.screen.print("RB  ")
             
            
        if RF.installed() != True:
            brain.screen.print("RF ")
            joystick.screen.print("RF  ")
         
        if RS.installed() != True:
            brain.screen.print("RS ")
            joystick.screen.print("RS  ")
        
        ### Prints the motor temps in a square ###
