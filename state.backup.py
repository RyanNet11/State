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


brain=Brain()
joystick = Controller()
timer = Timer()


#######################################
#          initate PORTS              #
#######################################


RF = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False)
RB = Motor(Ports.PORT17, GearSetting.RATIO_6_1, False)
RS = Motor(Ports.PORT15,GearSetting.RATIO_18_1, False) #small motor port 10


LF = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
LB = Motor(Ports.PORT7, GearSetting.RATIO_6_1, True)
LS = Motor(Ports.PORT6,GearSetting.RATIO_18_1, True) # small motor port 3
intakeFrontR = Motor(Ports.PORT18, True)
intakeFrontL = Motor(Ports.PORT11)
intakeMiddle = Motor(Ports.PORT19)
intakeBackTop= Motor(Ports.PORT4)
intakeFrontTop= Motor(Ports.PORT20, True)

LeftMotors = MotorGroup(LF, LB, LS)
RightMotors = MotorGroup(RF, RB, RS)
entireIntake = MotorGroup(intakeFrontR, intakeFrontL, intakeMiddle, intakeBackTop,intakeFrontTop)
MostOfIntake = MotorGroup(intakeFrontR, intakeFrontL, intakeMiddle, intakeBackTop)

pincher = Pneumatics(brain.three_wire_port.a)
thirdStage = Pneumatics(brain.three_wire_port.b)
finger = Pneumatics(brain.three_wire_port.c)

odomL = Rotation(Ports.PORT3)
odomR = Rotation(Ports.PORT14, True)
odomL.reset_position()
odomR.reset_position()

distanceLeft = Distance(Ports.PORT13) 
distanceRright = Distance(Ports.PORT13)
distanceBack = Distance(Ports.PORT13)
distanceFrontLeft = Distance(Ports.PORT13)
distanceFrontRight = Distance(Ports.PORT13)



#######################################
#      IMU initiate and Calibrate     #
#######################################

IMU = Inertial(Ports.PORT12)
IMU.calibrate()   #calibrate the imu so we dont suck at auton!
while(True):
    if IMU.is_calibrating() == True:
        wait(5)
    else:
        break

#######################################
#          Global Variables           #
#######################################
xpos = 100
ypos = 5
theta = 0       
xposT = 0   # total x pos
yposT = 0   # total Y pos
L = 8      # look ahead distnce
O = 13.5    # drivetrain offset (wheel base)
points =  [
  {
    "i": 1,
    "x": 9.4,
    "y": 101.4
  },
  {
    "i": 2,
    "x": 11.4,
    "y": 101.6
  },
  {
    "i": 3,
    "x": 14.0,
    "y": 102.0
  },
  {
    "i": 4,
    "x": 16.4,
    "y": 102.0
  },
  {
    "i": 5,
    "x": 18.4,
    "y": 102.0
  },
  {
    "i": 6,
    "x": 21.2,
    "y": 102.0
  },
  {
    "i": 7,
    "x": 24.8,
    "y": 102.0
  },
  {
    "i": 8,
    "x": 26.6,
    "y": 101.0
  },
  {
    "i": 9,
    "x": 28.2,
    "y": 99.6
  },
  {
    "i": 10,
    "x": 29.4,
    "y": 98.0
  },
  {
    "i": 11,
    "x": 30.8,
    "y": 95.6
  },
  {
    "i": 12,
    "x": 30.8,
    "y": 93.4
  },
  {
    "i": 13,
    "x": 30.6,
    "y": 91.0
  },
  {
    "i": 14,
    "x": 30.0,
    "y": 88.2
  }
]




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
        
        xposT += abs(dT * math.cos(theta))   # get total distancec by taking abs of 
        yposT += abs(dT * math.sin(theta))   # both values
        
        prevL = curL            # set the current points to previous points
        prevR = curR  
        prevTheta = theta          

        wait(5, MSEC)

def odomReadout():
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
    pos = "NEW PROGRAM INITIATION"
    json_str = json.dumps(pos) + "\n"
    json_bytes = bytearray(json_str, 'utf-8')
    brain.sdcard.appendfile("odomOverTime.txt", json_bytes)

    while(True):
        global xpos,ypos,theta
        pos = {"x":xpos, "y":ypos}
        
        json_str = json.dumps(pos) + "\n"
        json_bytes = bytearray(json_str, 'utf-8')

        brain.sdcard.appendfile("odomOverTime.txt", json_bytes)
        wait(100)

def find_closest_index(robot_x, robot_y,path):
    best_i = 0
    best_dist = 1e9
    
    for p in (path):
        pi, px, py = p["i"], p["x"], p["y"]
        d = (px - robot_x)**2 + (py - robot_y)**2  # squared distance multiplied by the point number
        if d < best_dist:
            best_dist = d
            best_i = pi

    return best_i

def find_lookahead(robot_x, robot_y, path, start_index, L):
    # if start_index - len(path) < 1:  

    for i in range(start_index, len(path)):

        px = path[i]["x"]
        py = path[i]["y"]

        dist = ((px - robot_x)**2 + (py - robot_y)**2)**0.5

        if dist >= L:
            return i, px, py

    # fallback
    last_point = path[-1]
    return len(path)-1, last_point["x"], last_point["y"]
  
def to_robot_frame(robot_x, robot_y, robot_theta, look_x, look_y):
    # subtract robot position and point
    dx = look_x - robot_x
    dy = look_y - robot_y

    # calculate the sin and cos of robot
    sin_t = math.sin(robot_theta)
    cos_t = math.cos(robot_theta)

    # use the transformation to 0 radians 
    x_r =  dx * cos_t + dy * sin_t
    y_r = -dx * sin_t + dy * cos_t

    return x_r, y_r

def compute_curvature(x_r, y_r):
    L2 = x_r**2 + y_r**2

    if L2 == 0:
        return 0

    return 2 * y_r / L2

def wheel_speeds(base_velocity, curvature, track_width):
    left  = base_velocity * (1 - curvature * track_width / 2)
    right = base_velocity * (1 + curvature * track_width / 2)
    return left, right

def normalize(left, right, max_speed):
    m = max(abs(left), abs(right))
    if m > max_speed:
        scale = max_speed / m
        left *= scale
        right *= scale
    return left, right

def DrivePurePursuit():
    global xpos,ypos,theta, points
    joystick.screen.set_cursor(1,1)
    joystick.screen.print("driving pure pursuit")
    LeftMotors.spin(FORWARD)
    RightMotors.spin(FORWARD)
    wait(20)
    pos = points
    last = points[-1]
    json_str = json.dumps(pos) + "\n"
    json_bytes = bytearray(json_str, 'utf-8')
    brain.sdcard.appendfile("odomOverTime.txt", json_bytes)

    while(True):
        currentPoint = find_closest_index(xpos,ypos,points)
        nextPoint = find_lookahead(xpos,ypos,points,currentPoint, L)
        centered = to_robot_frame(xpos,ypos,theta,nextPoint[1],nextPoint[2])
        computed = compute_curvature(centered[0],centered[1])
        speeds = wheel_speeds(50,computed, 13.5)
        normedSpeeds = normalize(speeds[0],speeds[1],100)
        brain.screen.set_cursor(1,1)
        brain.screen.print(normedSpeeds)
        LeftMotors.set_velocity(normedSpeeds[0], PERCENT)
        RightMotors.set_velocity(normedSpeeds[1],PERCENT)
        
        dist = ((xpos - last["x"])**2 + (ypos - last["y"])**2)**0.5
        if dist < 1.5:
            RightMotors.stop()
            LeftMotors.stop()
            brain.screen.print("done")
            break 
        
        data = {
            "xpos":xpos,
            "ypos":ypos,
            "theta":theta,
            "currentPoint":currentPoint,
            "nextPoint":nextPoint,
            "centeredPoint":centered,
            "computedPoints":computed,
            "speeds":speeds,
            "normedSpeeds":normedSpeeds
        }
        # brain.screen.clear_screen()
        # brain.screen.set_cursor(1,1)
        # brain.screen.print("xpos ", xpos, "ypos " , ypos)
        # brain.screen.new_line()
        # brain.screen.print("theta ",theta)
        # brain.screen.new_line()
        # brain.screen.print("currentPoint ",currentPoint)
        # brain.screen.new_line()
        # brain.screen.print("nextPoint ",nextPoint)
        # brain.screen.new_line()
        # brain.screen.print("centeredPoint ",centered)
        # brain.screen.new_line()
        # brain.screen.print("computedPoints ",computed)
        # brain.screen.new_line()
        # brain.screen.print("speeds ",speeds)
        # brain.screen.new_line()
        # brain.screen.print("normedSpeeds ",normedSpeeds)
                                  
        json_str = json.dumps(data) + "\n"
        json_bytes = bytearray(json_str, 'utf-8')
        brain.sdcard.appendfile("odomOverTime.txt", json_bytes)

        wait(10)
    LeftMotors.set_velocity(0, PERCENT)
    RightMotors.set_velocity(0,PERCENT)

      
        
#######################################
#     All Controller Functions       #
#######################################
def port_sensing():
    connected = [IMU, LB, LF, LS, RB, RS, RF, odomL, odomR, intakeFrontL, intakeFrontR, intakeMiddle, intakeBackTop, intakeFrontTop]
    
    while(True):
        joystick.screen.clear_line(3)

        for x in connected:
            if x.installed() != True:
                joystick.screen.print(x )
        wait(240)         

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
    #brain.screen.print(curved * sign * 100)
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
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    pneum = Thread(pnumaticss)
    vroom = Thread(controllerDrive)
    intake = Thread(Intake)
    odom = Thread(odomTracker)
    drive = Thread(SaveOdomToSD)

odom = Thread(odomTracker)
drive = Thread(DrivePurePursuit)
#comp = Competition(user_control, autonomous)

# actions to do when the program starts
#brain.screen.clear_screen()