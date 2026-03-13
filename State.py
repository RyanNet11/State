# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       ryanV                                                        #
# 	Created:      12/6/2025, 5:27:04 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
import json 
from vex import *
import math

# Initiate Vex Stuff
brain=Brain()
brain.sdcard
joystick = Controller()
timer = Timer()


#######################################
#          initate PORTS              #
#######################################

# Drivetrain Motors
RF = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False)
RB = Motor(Ports.PORT17, GearSetting.RATIO_6_1, False)
RS = Motor(Ports.PORT15,GearSetting.RATIO_18_1, False) #small motor port 10


LF = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
LB = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
LS = Motor(Ports.PORT6,GearSetting.RATIO_18_1, True) # small motor port 3

# Intake Motors 
intakeFrontR = Motor(Ports.PORT18, True)
intakeFrontL = Motor(Ports.PORT11)
intakeMiddle = Motor(Ports.PORT19)
intakeBackTop= Motor(Ports.PORT4)
intakeFrontTop= Motor(Ports.PORT20, True)

# Group Motors
LeftMotors = MotorGroup(LF, LB, LS)
LeftMotors.set_stopping(BRAKE)
RightMotors = MotorGroup(RF, RB, RS)
RightMotors.set_stopping(BRAKE)
entireIntake = MotorGroup(intakeFrontR, intakeFrontL, intakeMiddle, intakeBackTop,intakeFrontTop)
MostOfIntake = MotorGroup(intakeFrontR, intakeFrontL, intakeMiddle, intakeBackTop)

# Pneumatics 
pincher = Pneumatics(brain.three_wire_port.a)
thirdStage = Pneumatics(brain.three_wire_port.b)
finger = Pneumatics(brain.three_wire_port.c)

# Odom Seneors
odomL = Rotation(Ports.PORT3)
odomR = Rotation(Ports.PORT14, True)
odomL.reset_position()
odomR.reset_position()

# Distance Sensors
distanceLeft = Distance(Ports.PORT13) 
distanceRright = Distance(Ports.PORT13)
distanceBack = Distance(Ports.PORT13)
distanceFrontLeft = Distance(Ports.PORT13)
distanceFrontRight = Distance(Ports.PORT13)



#######################################
#      IMU initiate and Calibrate     #
#######################################

# Calibrate the imu so we dont suck at auton!
# Having a calibrated sensors allows us to elminate
# most errors
IMU = Inertial(Ports.PORT12)
IMU.calibrate()   

# Wait untill the sensors is done calibrating to start

while(True):
    if IMU.is_calibrating() == True:
        wait(5)
    else:
        break

# Set the IMU heading to reflect the 
# Starting state of the robot
IMU.set_heading(0, DEGREES)

#######################################
#          Global Variables           #
#######################################

xpos = 5        # X position in feild
ypos = 87.75    # Y position in feild
theta = 0       # Angle in feild 

xposT = 0       # total x pos
yposT = 0       # total Y pos
L = 16          # look ahead distnce
O = 13.5        # drivetrain offset (wheel base)
currentPoint = 0

#######################################
#         Global Paths to follow      #
#######################################

points =  [{"i": 1, "y": 89.8, "x": 5.8}, {"i": 2, "y": 89.8, "x": 9.2}, {"i": 3, "y": 90.2, "x": 12.4}, {"i": 4, "y": 90.2, "x": 15.2}, {"i": 5, "y": 91.0, "x": 18.8}, {"i": 6, "y": 92.40001, "x": 22.8}, {"i": 7, "y": 95.0, "x": 25.4}, {"i": 8, "y": 97.2, "x": 27.4}, {"i": 9, "y": 100.0, "x": 27.6}, {"i": 10, "y": 105.4, "x": 27.6}, {"i": 11, "y": 108.8, "x": 27.2}, {"i": 12, "y": 111.8, "x": 25.8}, {"i": 13, "y": 115.4, "x": 22.8}, {"i": 14, "y": 117.2, "x": 18.4}, {"i": 15, "y": 118.0, "x": 13.2}, {"i": 16, "y": 118.8, "x": 9.2}, {"i": 17, "y": 118.8, "x": 5.8}]
toGoal1 = [{"i": 1, "x": 38.4, "y": 132.0}, {"i": 2, "x": 41.8, "y": 132.0}, {"i": 3, "x": 45.8, "y": 132.0}, {"i": 4, "x": 48.4, "y": 132.0}, {"i": 5, "x": 52.0, "y": 132.2}, {"i": 6, "x": 56.6, "y": 132.4}, {"i": 7, "x": 61.0, "y": 132.2}, {"i": 8, "x": 66.4, "y": 132.0}, {"i": 9, "x": 70.8, "y": 132.0}, {"i": 10, "x": 75.6, "y": 132.0}, {"i": 11, "x": 80.0, "y": 132.0}, {"i": 12, "x": 83.6, "y": 131.4}, {"i": 13, "x": 87.2, "y": 131.0}, {"i": 14, "x": 90.6, "y": 130.6}, {"i": 15, "x": 95.2, "y": 132.2}, {"i": 16, "x": 99.6, "y": 132.0}, {"i": 17, "x": 102.2, "y": 131.2}, {"i": 18, "x": 105.0, "y": 130.0}, {"i": 19, "x": 107.4, "y": 129.0}, {"i": 20, "x": 109.2, "y": 126.0}, {"i": 21, "x": 110.8, "y": 123.0}, {"i": 22, "x": 110.8, "y": 119.8}, {"i": 23, "x": 110.8, "y": 115.4}, {"i": 24, "x": 109.2, "y": 112.8}, {"i": 25, "x": 105.2, "y": 112.2}, {"i": 26, "x": 101.4, "y": 112.2}, {"i": 27, "x": 98.8, "y": 112.2}]



#######################################
#               Odomatry              #
#######################################

def odomTracker():
    prevL = 0
    prevR = 0
    prevTheta = 0
    global xpos, ypos, xposT, yposT, theta
    while(True):
        # get current positions of the rotation sensors and 
        # relate it to how far robot travels using 
        # the ratio for rotations to distance travled 
        
        curL = odomL.position() * 3.1415 / 360 * 2   
        curR = odomR.position() * 3.1415 / 360 * 2   
        
        # get the angle of the robot 
        theta = math.radians(IMU.heading())        
        # set distance deltas by taking the current 
        # distance and subtracting the previous position
        dL = curL - prevL        
        dR = curR - prevR        
        
        # average the two distances (left and right) to get 
        # how far we traveled total 
        dT = (dL + dR) / 2        
        
        # relate the distances and The heading by 
        # taking the cosine and sin of theta times distance
        # add these to global x and y variables
        avg_theta = prevTheta + (theta - prevTheta) / 2
        
        xpos += dT * math.cos(avg_theta)
        ypos += -dT * math.sin(avg_theta)
        
        # xposT += abs(dT * math.cos(theta))   # get total distancec by taking abs of 
        # yposT += abs(dT * math.sin(theta))   # both values
        
        prevL = curL            # set the current points to previous points
        prevR = curR  
        prevTheta = theta          

        wait(8, MSEC)

def odomReadout():
    # For testing purposes, print the 
    # Odomatry values on the brain
    
    global xpos, ypos, xposT, yposT
    
    while(True):
        brain.screen.set_cursor(1,1)     # prints position in feild
        brain.screen.print(xpos)
        brain.screen.set_cursor(1,20)    # as x and Y positions
        brain.screen.print(ypos)
        
        brain.screen.set_cursor(3,1)     # prints the total distance traveled
        brain.screen.print(xposT)
        brain.screen.set_cursor(3,20)    # in x and Y 
        brain.screen.print(yposT)
        
        wait(10)
        
#######################################
#       Pure Pursuit Functions        #
#######################################


def SaveOdomToSD():
    # start by saving a phrase to the 
    # SD card to mark a new program start
    
    pos = "NEW PROGRAM INITIATION"
    json_str = json.dumps(pos) + "\n"
    json_bytes = bytearray(json_str, 'utf-8')
    brain.sdcard.appendfile("odomOverTime.txt", json_bytes)

    while(True):
        global xpos,ypos,theta
        
        # Get the current position of the robot
        pos = {"x":xpos, "y":ypos}
        
        # change it into a bytearray for the SD card
        json_str = json.dumps(pos) + "\n"
        json_bytes = bytearray(json_str, 'utf-8')
         
        # Save it to the SD card 
        brain.sdcard.appendfile("odomOverTime.txt", json_bytes)
        
        # Repeat every 100 ms
        wait(100)


def find_closest_index(robot_x, robot_y, path, lastPoint):
    # start by setting the best index to the 
    # worst index to make sure our answer
    # is better than the worst answer 
    
    best_i = lastPoint
    best_dist = float("inf")

    # start a loop for each point in the list
    i = -1
    for p in path:
        i += 1

        # make sure the point is after the last point
        # to make sure we dont work backwards
        if i < lastPoint:
            continue

        px = p["x"]
        py = p["y"]

        # Find the Distance
        d = (px - robot_x)**2 + (py - robot_y)**2

        # set best variables to the current itterative point
        # if they are the best value
        if d < best_dist:
            best_dist = d
            best_i = i
    # return the best calculated value and the 
    # point directly before it 
    return best_i, (best_i -1)

 
def sign(x):
    # return the sign of a number
  return 1 if x >= 0 else -1


def find_lookahead(x, y, point_1, point_2, start_index, L):

    # Move the situation to the origin 
    x1 = point_1["x"] - x
    y1 = point_1["y"] - y
    x2 = point_2["x"] - x
    y2 = point_2["y"] - y

    # setup to take the discriminant 
    
    dx = x2 - x1
    dy = y2 - y1

    dr = math.sqrt(dx*dx + dy*dy)
    D = x1*y2 - x2*y1

    # Take the discriminant and see if there
    # is a solution or not
    
    discriminant = L*L*dr*dr - D*D

    if discriminant < 0:
        return None

    # If the discriminant is positive, there exists a solution
    # Next we find the solution 
    
    sqrt_disc = math.sqrt(discriminant)

    sign_dy = sign(dy)

    sol1x = (D*dy + sign_dy*dx*sqrt_disc)/(dr*dr)
    sol1y = (-D*dx + abs(dy)*sqrt_disc)/(dr*dr)

    sol2x = (D*dy - sign_dy*dx*sqrt_disc)/(dr*dr)
    sol2y = (-D*dx - abs(dy)*sqrt_disc)/(dr*dr)

    # add the values of X and Y back to return 
    # the situation away from the origin 
    
    sol1x += x
    sol1y += y
    sol2x += x
    sol2y += y

    # Take minimum and maximum values for X and Y
    
    minX = min(point_1["x"], point_2["x"])
    maxX = max(point_1["x"], point_2["x"])
    minY = min(point_1["y"], point_2["y"])
    maxY = max(point_1["y"], point_2["y"])

    # See if the solutinos are within the range
    # of the points we used
    
    if minX <= sol1x <= maxX and minY <= sol1y <= maxY:
        return start_index, sol1x, sol1y

    if minX <= sol2x <= maxX and minY <= sol2y <= maxY:
        return start_index, sol2x, sol2y

    return None
    
    
    
# def find_lookahead(robot_x, robot_y, path, start_index, L):

#     for i in range(start_index, len(path)):

#         px = path[i]["x"]
#         py = path[i]["y"]

#         dist = ((px - robot_x)**2 + (py - robot_y)**2)**0.5

#         if dist >= L:
#             return i, px, py

#     # fallback
#     last_point = path[-1]
#     return len(path)-1, last_point["x"], last_point["y"]
# def to_robot_frame(robot_x, robot_y, robot_theta, look_x, look_y):
#     dx = look_x - robot_x
#     dy = look_y - robot_y

#     sin_t = math.sin(robot_theta)
#     cos_t = math.cos(robot_theta)

#     x_r =  dx * cos_t + dy * sin_t
#     y_r = -dx * sin_t + dy * cos_t

#     return x_r, y_r

def to_robot_frame(robot_x, robot_y, robot_theta, look_x, look_y):
    
    # Change in X and Y
    dx = look_x - robot_x
    dy = look_y - robot_y

    # Take the calculation of theta 
    cos_t = math.cos(robot_theta)
    sin_t = math.sin(robot_theta)
    
    # Calculate new with rotation equasions
    x_r =  cos_t * dx - sin_t * dy
    y_r = sin_t * dx + cos_t * dy

    return x_r, y_r

def compute_curvature(x_r, y_r):
    # set L2 as the distance between the points
    # by squaring both X and Y and adding them
    L2 = x_r*x_r + y_r*y_r

    #if the distnace is very small, just return 0 to avoid weird values
    # The function will return 0 for values close to 0 to avoid returning
    # infinity (cant divide by 0)
    if abs(L2) < 1e-6:
        return 0
    
    # Return the values of 2y / L2
    
    return 2 * y_r / L2

def wheel_speeds(base_velocity, curvature, track_width):

    # calculate angular velocity 
    
    angular_velocity = base_velocity * curvature

    # Use angular velocity, the track with, and the curvature
    # value to calculate the desired speed of the robot
    
    left  = base_velocity - angular_velocity * track_width / 2
    right = base_velocity + angular_velocity * track_width / 2

    # Return those values 
    return left, right

def normalize(left, right, max_speed):
    # make sure the speed values aren't too
    # fast for the robot 
    
    m = max(abs(left), abs(right))

    if m > max_speed:
        scale = max_speed / m
        left *= scale
        right *= scale

    return left, right
    
    
def DrivePurePursuit(path):
    global xpos,ypos,theta, currentPoint
    wait(20)
    
    last = path[-1]
    lastpoint = 0
    normedSpeeds = [0,0]
    
    json_str = json.dumps(path) + "\n"
    json_bytes = bytearray(json_str, 'utf-8')
    brain.sdcard.appendfile("odomOverTime.txt", json_bytes)
    
    while(True):
        
      # Set the robot to drive the speeds calculated
      # during the last loop itteration
      
      LeftMotors.spin(FORWARD, normedSpeeds[0], PERCENT)
      RightMotors.spin(FORWARD, normedSpeeds[1], PERCENT)
      
      # Find the current point
      currentPoint, _ = find_closest_index(xpos, ypos, path, lastpoint)

      lookahead = None
      
      # Calculate the lookahead distance 
      
      for i in range(currentPoint, len(path)-1):

          result = find_lookahead(
              xpos,
              ypos,
              path[i],
              path[i+1],
              i,
              L
          )

          if result is not None:
              lookahead = result
              break

      if lookahead is None:
          lastp = path[-1]
          look_x = lastp["x"]
          look_y = lastp["y"]
          lookahead = lastp, look_x, look_y
      else:
          _, look_x, look_y = lookahead
      
      # Move the frame of the lookahead point and robot
      # to the origin

      centered = to_robot_frame(xpos, ypos, theta, look_x, look_y)
      
      # Compute the needed curvature to get to
      # The lookahead point then norm the values
      # to make sure that there isn't a 
      # large error
      computed = compute_curvature(centered[0],centered[1])
      computed = max(-1.5, min(1.5, computed))
      
      # set the speeds of the wheels
      speeds = wheel_speeds((50 * (1 - min(abs(computed), 1))),computed, O)
      
      # Normalise the speeds to make sure we dont go too fast
      normedSpeeds = normalize(speeds[0],speeds[1],100)
      
      # Debug step to see the speeds
      brain.screen.new_line()
      brain.screen.print(normedSpeeds)
      
      # calculate the distance to the last point
      
      dist = ((xpos - last["x"])**2 + (ypos - last["y"])**2)**0.5
      
      # If we are within 4 inches of that position, stop the
      # pursuit program 
      if dist < 4:
          RightMotors.stop()
          LeftMotors.stop()
          brain.screen.print("done")
          break 
      lastpoint = currentPoint        
      
      # SD card stuff for looking at what happned
      # Save data as data and then save to SD card
      data = {
          "xpos":xpos,
          "ypos":ypos,
          "theta":theta,
          "currentPoint":currentPoint,
          "lookahead":lookahead,
          "centeredPoint":centered,
          "Curvature":computed,
          "speeds":speeds,
          "normedSpeeds":normedSpeeds
      }
      json_str = json.dumps(data) + "\n"
      json_bytes = bytearray(json_str, 'utf-8')
      brain.sdcard.appendfile("odomOverTime.txt", json_bytes)

      wait(20)
    # Set the motors to 0 when we exit the loop
    LeftMotors.set_velocity(0, PERCENT)
    RightMotors.set_velocity(0,PERCENT)

#######################################
#     Auton Helper Functions          #
#######################################

def drive(speed):
    RightMotors.spin(FORWARD,speed,PERCENT)
    LeftMotors.spin(FORWARD,speed,PERCENT)

def stop():
    RightMotors.spin(FORWARD,0,PERCENT)
    LeftMotors.spin(FORWARD,0,PERCENT)

def wiggle():
    LeftMotors.spin(FORWARD,20,PERCENT)
    RightMotors.spin(REVERSE,20,PERCENT)
    wait(200)
    LeftMotors.spin(FORWARD,-20,PERCENT)
    RightMotors.spin(REVERSE,-50,PERCENT)
    wait(200)
    LeftMotors.spin(FORWARD,20,PERCENT)
    RightMotors.spin(REVERSE,20,PERCENT)
    wait(200)
    LeftMotors.spin(FORWARD,-20,PERCENT)
    RightMotors.spin(REVERSE,-20,PERCENT)
    wait(200)
    LeftMotors.spin(FORWARD,20,PERCENT)
    RightMotors.spin(REVERSE,20,PERCENT)
    wait(200)
    LeftMotors.spin(FORWARD,-20,PERCENT)
    RightMotors.spin(REVERSE,-20,PERCENT)
    wait(200)

def spin(number):
    RightMotors.spin(REVERSE,number,PERCENT)
    LeftMotors.spin(FORWARD,number,PERCENT)

    
def intakeByPoints():
    global currentPoint
    rangemin = 10
    rangemax = 15
    while(True):
        if (rangemin < currentPoint) and (currentPoint < rangemax):
                MostOfIntake.spin(FORWARD, 100, PERCENT)
        wait(5, MSEC)
           
#######################################
#     All Controller Functions       #
#######################################

def pincherPneumatics():
            if pincher.value():
                pincher.close()
            else:
                pincher.open()

def intakepneumatics():
            if thirdStage.value():
                thirdStage.close()
            else:
                thirdStage.open()

def fingerpneumatics():
            if finger.value():
                finger.close()
            else:
                finger.open()

def CubicThrottleCurve(inputPercent):
    x = inputPercent / 100
    sign = 1 if x >= 0 else -1         # this guy is in use
    x = abs(x)

    curved = math.pow(x, 2.75)
    return curved * 100 * sign

def logThrottleCurve(inputPercent, a = 4.0):
    x = inputPercent / 100
    sign = 1 if x >= 0 else -1
    x = abs(x)
    curved = math.log(a * x + 1) / math.log(a + 1)  # not used, but might be helpful?
    brain.screen.set_cursor(3,10)
    brain.screen.print(curved * sign * 100)
    return curved * sign * 100    

def controllerDrive():
    while True:  # drive with the cubic throttle curve
        LeftMotors.spin(FORWARD, CubicThrottleCurve(joystick.axis3.value()), PERCENT)
        RightMotors.spin(FORWARD, CubicThrottleCurve(joystick.axis2.value()), PERCENT)

def Intake():
    while(True):
        if joystick.buttonR1.pressing() == True:
            MostOfIntake.spin(FORWARD, 100, PERCENT)
        elif joystick.buttonR2.pressing() == True:
            entireIntake.spin(FORWARD, 100,PERCENT)
        elif joystick.buttonL1.pressing() == True:
            MostOfIntake.spin(REVERSE, 100,PERCENT)
        elif joystick.buttonL2.pressing() == True:
            entireIntake.spin(REVERSE, 100,PERCENT)
        else:
            entireIntake.spin(FORWARD, 0, PERCENT)    

def pnumaticss():
    joystick.buttonA.pressed(pincherPneumatics)   # all our pnumatics
    joystick.buttonX.pressed(intakepneumatics)
    joystick.buttonUp.pressed(fingerpneumatics)      
          # a is front
          # x is top

#######################################
#    Compitition functions& controls  #
#######################################        
def autonomous():
    odom = Thread(odomTracker)
    DrivePurePursuit(points)
    pincherPneumatics()
    MostOfIntake.spin(FORWARD,100,PERCENT)
    wait(3000)
    drive(-30)
    wait(700)
    stop()
    spin(50)
    wait(500)
    stop()
    pincherPneumatics()
    MostOfIntake.spin(FORWARD,0,PERCENT)
    DrivePurePursuit(toGoal1)
    intakepneumatics()
    MostOfIntake.spin(FORWARD,100,PERCENT)
    wait(5000)
    MostOfIntake.spin(FORWARD,0,PERCENT)

    
    # drive(-50)
    # wait(500)
    # stop()
    
def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    pneum = Thread(pnumaticss)
    vroom = Thread(controllerDrive)
    intake = Thread(Intake)
    odom = Thread(odomTracker)
    drive = Thread(SaveOdomToSD)
    
autonomous()
# intakeSensing = Thread(intakeByPoints)
#comp = Competition(user_control, autonomous)

# actions to do when the program starts
#brain.screen.clear_screen()