"""dark_knight_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

psNames = [('ps' + str(i)) for i in range(8)]
lsNames = [('ls' + str(i)) for i in range(8)]

ps = []
ls = []
for i in range(8):
    prxmt_sensor = robot.getDevice(psNames[i])
    prxmt_sensor.enable(timestep)
    ps.append(prxmt_sensor)
    light_sensor = robot.getDevice(lsNames[i])
    light_sensor.enable(timestep)
    ls.append(light_sensor)
    
# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    prxmt_vals = [prxmt_sensor.getValue() for prxmt_sensor in ps]
    light_vals = [light_sensor.getValue() for light_sensor in ls]
    print(prxmt_vals)
    print(light_vals) 
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
