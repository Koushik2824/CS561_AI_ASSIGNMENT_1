from controller import Robot, DistanceSensor, LightSensor
import math

# Create an instance of the robot.
robot = Robot()

# Get the time step of the current world
timestep = int(robot.getBasicTimeStep())

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# Set motors to velocity control mode with initial velocity 0.
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Initialize proximity sensors
proximity_sensors = []
proximity_sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for name in proximity_sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    proximity_sensors.append(sensor)

# Initialize light sensors
light_sensors = []
light_sensor_names = ['ls0', 'ls1', 'ls2', 'ls3', 'ls4', 'ls5', 'ls6', 'ls7']
for name in light_sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(timestep)
    light_sensors.append(sensor)

def normalizeVector(a: tuple) -> tuple:
    ''' Takes a two dimensional vector and returns a unit vector in the same direction.'''
    magnitude = ((a[0]**2 + a[1]**2)**0.5)
    return (a[0]/magnitude,a[1]/magnitude)

# These are the position of the sensors available in the E-Puck robot.
# Note that the reference
# https://www.cyberbotics.com/doc/guide/epuck?version=cyberbotics:R2019a
# states that the same set of sensors are used for both Proximity and Light sensing.
# Also note that the values listed here are in reference to frame of reference of the bot.
# The blue axis, pointing upward is the +ve y axis.
# The green axis, pointing along the axle's length is the +ve x axis.
# The red axis, pointing in the forward direction of motion of the bot is the +ve z axis.
sensor_positions = [
    normalizeVector((-0.010, +0.030)),
    normalizeVector((-0.025, +0.022)),
    normalizeVector((-0.031,  0.000)),
    normalizeVector((-0.015,  -0.030)),
    normalizeVector((0.015, -0.030)),
    normalizeVector((0.031, 0.000)),
    normalizeVector((0.025, 0.022)),
    normalizeVector((0.010, 0.030))
]

# The threshold value above which we say a obstacle is near and we would like to avoid it.
PROXIMITY_THRESHOLD = 80.0  
# The threshold value above which we say light is near and we would like to avoid it.
LIGHT_THRESHOLD = 2500.0

# Starting value for the EWMA. (Exponentially weighted moving average)
# We use EWMA for smoothing out the variations due to noise in the angle change calculations in the loop below.
initial_ac = 0.0
# Weight for the EWMA.
alpha = 0.25

# Some physical properties of the robot model used, with the units written in comment immediately following the values.
# These are taken from the reference above.
WHEEL_RADIUS = 20.5/1000 # m
AXLE_LENGTH =  52/1000 # m
BASE_LINEAR_SPEED = 0.14 # m/s
MAX_VELOCITY = 6.28 # rad/s

# Gain factor, for magnifying the response due to angle change.
Kp = 25 # Todo: Can Change and test with the Gain factor....

# The below data is required for the calculation of the direction in which the robot should move to avoid obstacles.
# The reason behind doing so is, that if we sense a obstacle nearby from a particular sensor,
# we should try going in the opposite way to reduce the chances of collision.
# For every sensor saying so, we take a resultant vector.
directions_of_motion_due_obstacle = [
        (-(sensor_positions[i][0]),
          -(sensor_positions[i][1])) 
          for i in range(8)]

# With the above comment's abstraction, we can treat light in pretty much the same way,
# BUT the difference with Dark Knight comes in the fact that it should follow the light.
# So if we sense light nearby we should try going in the direction of the sensor.
directions_of_motion_due_light = [
        (+(sensor_positions[i][0]), 
         +(sensor_positions[i][1])) 
    for i in range(8)]

# The simulation loop
while robot.step(timestep) != -1:
    # Read proximity and light sensor values.
    proximity_values = [sensor.getValue() for sensor in proximity_sensors]
    
    light_values = [(sensor.getValue()) for sensor in light_sensors]
    
    # If no obstacle or light detected, just move forward.
    net_direction_due_obstacle_x = 0
    net_direction_due_obstacle_y = 1
    net_direction_due_light_x = 0
    net_direction_due_light_y = 0
    
    # Calculate the component of the resultant vector of the direction which reduces the chance of collision.
    for i in range(8):
       if proximity_values[i] > PROXIMITY_THRESHOLD:
           net_direction_due_obstacle_x += directions_of_motion_due_obstacle[i][0]
           net_direction_due_obstacle_y += directions_of_motion_due_obstacle[i][1] 
       if light_values[i] < LIGHT_THRESHOLD:
           net_direction_due_light_x += directions_of_motion_due_light[i][0]
           net_direction_due_light_y += directions_of_motion_due_light[i][1]
       
    # Collecting the computations above.
    net_direction_due_obstacle = (net_direction_due_obstacle_x,net_direction_due_obstacle_y)
    net_direction_due_light = (net_direction_due_light_x,net_direction_due_light_y)
    
    net_movement = (
            net_direction_due_obstacle[0] + net_direction_due_light[0], 
            net_direction_due_obstacle[1] + net_direction_due_light[1]
             )

    net_movement = normalizeVector(net_movement)

    # Calculate the required angle change.
    desired_angle = math.atan2(net_movement[0], net_movement[1])
    
    # Magnify the change.
    angular_correction = Kp * desired_angle
    
    # EWMA 
    angular_correction = alpha*initial_ac + (1.0-alpha)*angular_correction
    initial_ac = angular_correction
    
    # Calculating the speed of the wheels. The equations are taken from online references on basic physics.
    right_wheel_speed = (BASE_LINEAR_SPEED / WHEEL_RADIUS) + ((AXLE_LENGTH / (2 * WHEEL_RADIUS)) * angular_correction)
    left_wheel_speed  = (BASE_LINEAR_SPEED / WHEEL_RADIUS) - ((AXLE_LENGTH / (2 * WHEEL_RADIUS)) * angular_correction)
    
    # Set the wheel speeds.
    right_motor.setVelocity(min(max(right_wheel_speed,-MAX_VELOCITY),MAX_VELOCITY))
    left_motor.setVelocity(min(max(left_wheel_speed,-MAX_VELOCITY),MAX_VELOCITY))