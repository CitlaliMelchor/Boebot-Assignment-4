# Initial Webot controller for Boebot training in Automation for Bioproduction
# Assignement 4
# Nick van de Westeringh

# load libraries
from controller import Robot, LED, Motor, PositionSensor, DistanceSensor
import math

# define constants
TIME_STEP = 20 #[ms], must be a multiple of "Worlds basicTimeStemp"
PRINT_INTERVAL = 250 #[ms], time between console messages
BLINK_INTERVAL = 250 #[ms], time LED stays on and off
MAX_SPEED = 20 #[rad/s], maximum rotaional wheel speed
DRIVING_VELOCITY = 0.15 #[m/s], desired driving velocity of Boebot
WHEEL_RADIUS = 0.034 #[m], radius of Boebot wheel
TURN_RADIUS = 0.25 #[m], radius of the turn on the headland
TRACK_WIDTH = 0.095 #[m], space between wheels
LENGTH_BOARD = 1.0 #[m], length of the board
ROW_WIDTH = 0.5 #[m], distance between middel of boards/rows
DEG_SYM = '°'
THETA_SYM ='ϴ'

#### Vehicle control #############################################################################
# TODO: Experiment with these values and determine appropriate ones
# Controller parameters
K_P = -0.9 #-0.0005
ZETA = 2 # critically damped

# TODO: Experiment with these values
# Desired speed (s) and initial robot pose: x, y, theta (position and heading)

# With a faster speed the robot has not enought time to correct its position
# When the speed is lower, the controlling is done more accurately 

# TODO: Implement this function
def control(dy,theta):
    # when turning c != 0
    # compute K_D at each iteration
    W = TRACK_WIDTH
    s = DRIVING_VELOCITY # desired driving velocity
    
    K_D = -ZETA*math.sqrt(-4*K_P*s) # from eq (26)
    
    d = K_P*dy + K_D*theta # eq (22) theta dot
    
    left_speed = (2*s - d*W)/(2*WHEEL_RADIUS) #w1
    right_speed = ((d*W/2)+s)/WHEEL_RADIUS #w2
    
    return left_speed, right_speed
###################################################################################################

def Toggle(status):
    # change from 1 to 0 and back
    if status == 1:
        value = 0
    else:
        value = 1
    return value

def CalcTheta(front, back):
    # calculate theta, angle between wall and driving direction (X-axis)

    # position of sensors:
    x_offset = 0.132 #[m]distance between sensors along x axis (driving direction)
    y_offset_front = 0.010 #[m] Offset y axis front sensor
    y_offset_back = 0.010 #[m] Offset y axis back sensor
    
    # calculate correction angle for placement of sensors on boebot
    offset_theta = math.asin((y_offset_front - y_offset_back) / x_offset) #[rad]
    # calculate distance between sensors 
    distance_sensors = math.sqrt(x_offset**2 + (y_offset_front - y_offset_back)**2) #[m]

    # calculate input for arcsinus
    asin_in = (front - back) / distance_sensors
    if -1 < asin_in < 1:  # or abs(front – back) > distance_sensors
        theta = math.asin(asin_in) - offset_theta
    elif asin_in <= -1:
        theta = math.asin(-1) - offset_theta
    else:
        theta = math.asin(1) - offset_theta

    return theta

# create the Robot instance.
boebot = Robot()


# assign sensors
    # Position sensor meausures rotation of wheel [rad]
left_position_sensor = boebot.getPositionSensor("left wheel sensor")
right_position_sensor = boebot.getPositionSensor("right wheel sensor")
    # Distance sensor measures distance between boebot and object
left_front_distance_sensor = boebot.getDistanceSensor("left front sensor")
right_front_distance_sensor = boebot.getDistanceSensor("right front sensor")
left_back_distance_sensor = boebot.getDistanceSensor("left back sensor")
right_back_distance_sensor = boebot.getDistanceSensor("right back sensor")

#initialize sensors
left_position_sensor.enable(TIME_STEP)
right_position_sensor.enable(TIME_STEP)
left_front_distance_sensor.enable(TIME_STEP)
right_front_distance_sensor.enable(TIME_STEP)
left_back_distance_sensor.enable(TIME_STEP)
right_back_distance_sensor.enable(TIME_STEP)

# assign actuators
# LEDs
left_led = boebot.getLED("left_led")
right_led = boebot.getLED("right_led")
# motors
left_motor = boebot.getMotor("left wheel motor")
right_motor = boebot.getMotor("right wheel motor")
# Initilaize motors - set positiont to infinity to control speed and set velocity to 0.
left_motor.setPosition(float('inf')) 
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# initialize variables
steps_counted = 0
time_running = 0.0 #[s]
left_position = 0.0 #[rad]
right_position = 0.0 #[rad]
state = "DRIVING"
speed = DRIVING_VELOCITY / WHEEL_RADIUS #[rad/s]
left_speed = speed #[rad/s]
right_speed = speed #[rad/s]
turning_distance = math.pi * TURN_RADIUS #[m] length of turn
left_led_value = 0
right_led_value = 0
distance_limit = LENGTH_BOARD #[m] set first limit to take action in state machine




# main loop
while boebot.step(TIME_STEP) != -1:
    # read sensors
    left_led_status = left_led.get()
    right_led_status = right_led.get()
    left_position = left_position_sensor.getValue()
    right_position = right_position_sensor.getValue()
    left_front_distance = left_front_distance_sensor.getValue()
    right_front_distance = right_front_distance_sensor.getValue()
    left_back_distance = left_back_distance_sensor.getValue()
    right_back_distance = right_back_distance_sensor.getValue()

    # create information from inputs (user functions)
    steps_counted += 1
    time_running = (steps_counted * TIME_STEP /1000) #[s]
    average_traveled_distance = (left_position * WHEEL_RADIUS + right_position * WHEEL_RADIUS) / 2 #[m]
    left_theta = CalcTheta( left_front_distance, left_back_distance)
    left_theta_deg = math.degrees(left_theta)
    right_theta = CalcTheta( right_front_distance, right_back_distance)
    right_theta_deg = math.degrees(right_theta)
    
    # TODO: calcuate controller inputs
    theta = CalcTheta(right_front_distance, right_back_distance)
    delta_y = ((right_front_distance + right_back_distance)/2)-0.24

    # take decisions (state machine)
    # TODO: Where to call the controller?
    if state == "DRIVING":
        	
        if right_front_distance >= ROW_WIDTH or left_front_distance >= ROW_WIDTH: # passed board
            state = "TURNING"
            left_led_value = 0
            # set speed for left and right wheel during turn [rad/s]
            left_speed = speed * ((TURN_RADIUS + TRACK_WIDTH /2) / TURN_RADIUS)
            right_speed = speed * ((TURN_RADIUS - TRACK_WIDTH /2) / TURN_RADIUS)
            # set new limit, add length of turn to current limit
            distance_limit = average_traveled_distance + turning_distance
        else:
            left_speed, right_speed = control(delta_y,theta)
            left_led_value = Toggle(left_led_status)
            
    elif state == "TURNING":
        right_led_value = 1
        if average_traveled_distance >= distance_limit:
            state = "DRIVING"
            right_led_value = 0
            # set speed for both wheels to drive straight
            left_speed, right_speed = control(delta_y,theta)
            # left_speed = speed
            # right_speed = speed
            # set new limit, add length of board to current limit
            distance_limit = average_traveled_distance + LENGTH_BOARD
        else:
            left_led_value = Toggle(left_led_status)
    else:
        break
    
    # compute behavior (user functions)
    
    
    # actuate console, LED and motors
    if steps_counted % (PRINT_INTERVAL / TIME_STEP) < 1:
        print("# %4d, T:%.2fs, %s, Vl:%.2f r:%.2frad/s, Tr:%.3fm < %.3fm, DFl:%.2f r:%.2f, DBl:%.2f r:%.2f, ϴl%5.2f (%3.0f°) r%5.2f (%3.0f°), y:%.2f." 
            % (steps_counted, time_running, state, left_speed, right_speed, average_traveled_distance, 
            distance_limit, left_front_distance, right_front_distance, left_back_distance, right_back_distance, 
            left_theta, left_theta_deg, right_theta, right_theta_deg, delta_y))
     
    if steps_counted % (BLINK_INTERVAL / TIME_STEP) < 1:
        left_led.set(left_led_value)
        right_led.set(right_led_value)
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)