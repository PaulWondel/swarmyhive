"""partyBoiPog controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor
from controller import DistanceSensor
from controller import Motor

TIME_STEP = 128

# create the Robot instance.
supervisor = Supervisor()
supervisorNode = supervisor.getSelf()


# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

def robotWheels():
    wheels = []
    wheelsNames = ["wheel1", "wheel2", "wheel3", "wheel4"]
    for i in range(4):
        wheels.append(supervisor.getMotor(wheelsNames[i]))
        wheels[i].setPosition(float('inf'))
        wheels[i].setVelocity(10.0)
    
    


def enableSensors(TIME_STEP):
    sLeft = supervisor.getDistanceSensor("ds_left")
    sRight = supervisor.getDistanceSensor("ds_right")
    sUp = supervisor.getDistanceSensor("ds_front")
    sDown = supervisor.getDistanceSensor("ds_back")
    
    sLeft.enable(TIME_STEP)
    sRight.enable(TIME_STEP)
    sUp.enable(TIME_STEP)
    sDown.enable(TIME_STEP)
    
    return sLeft, sRight, sUp, sDown

def getSensorValues():
    sLeftValue = sLeft.getValue()
    sRightValue = sRight.getValue()
    sUpValue = sUp.getValue()
    sDownValue = sDown.getValue()
    
    return sLeftValue, sRightValue, sUpValue, sDownValue
    
def getBotLocation():
    positionFromSupervisor = supervisorNode.getPosition()
    scaledX = (positionFromSupervisor[0] - 0.06665) * 7.5
    scaledZ = (positionFromSupervisor[2] - 0.06665) * 7.5
    roundedScaledX = round(scaledX)
    roundedScaledZ = round(scaledZ)
    #print("x: " ,scaledX)
    #print("z: " ,scaledX)
    #print("sx: " ,roundedScaledX)
    #print("sz: " ,roundedScaledZ)
    print("x before mod: ", positionFromSupervisor[0])
    x = (positionFromSupervisor[0]+ 0.06665) % 0.13333 
    print ("module result: ",x)
    if(x < 0.01):
        print("in center")



"""
robot_node = supervisor.getFromDef("botJr")
trans_field = robot_node.getField("translation")
rotation_field = robot_node.getField("rotation")
compass = supervisor.getCompass("compass")
compass.enable(TIME_STEP)
"""
sLeft, sRight, sUp, sDown = enableSensors(TIME_STEP)


while supervisor.step(TIME_STEP) != -1:
    sLeftValue, sRightValue, sUpValue, sDownValue = getSensorValues()
    robotWheels()
    getBotLocation()
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
