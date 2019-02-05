from robolink import *    # API to communicate with RoboDK
from robodk import *      # basic matrix operations

# Any interaction with RoboDK must be done through
# Robolink()
RDK = Robolink()

# get the robot item:
# robot = RL.Item('ABB IRB 1600ID-4/1.5')
robot = RDK.Item('', ITEM_TYPE_ROBOT)

# get the home target and the welding targets:
home = RDK.Item('Home')
target = RDK.Item('Target 1')
# get the pose of the target (4x4 matrix):
poseref = target.Pose()

# move the robot to home, then to the center:
robot.MoveJ(home)
robot.MoveJ(target)

# Set the speed to 50 mm/s
robot.setSpeed(50)

# call a robot program to start the weld gun
RDK.RunProgram('WeldStart')

# make an hexagon around the center:
for i in range(7):
    ang = i*2*pi/6 #ang = 0, 60, 120, ..., 360
    posei = poseref * rotz(ang)*transl(200, 0, 0) * rotz(-ang)
    robot.MoveL(posei)

# call a robot program to stop the weld gun
RDK.RunProgram('WeldStop')

# move back to the center, then home:
robot.MoveL(target)
robot.MoveJ(home)


    
    
    



