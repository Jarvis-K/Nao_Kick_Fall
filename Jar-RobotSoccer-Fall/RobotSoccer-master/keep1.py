"""
This example shows how to use ALRedBallTracker.
It is launched for a little while, then stopped.
"""

import sys
import time
from naoqi import *

# if (len(sys.argv) < 2):
#     print "Usage: 'python %s IP [PORT]'" % (__file__)
#     sys.exit(1)

# IP = sys.argv[1]
PORT = 9559
if (len(sys.argv) < 2):
    IP = '169.254.212.54'

if (len(sys.argv) > 2):
    PORT = sys.argv[2]


print "Connecting to", IP, "with port", PORT
motionProxy = ALProxy("ALMotion", IP, PORT)
redBallTracker = ALProxy("ALRedBallTracker", IP, PORT)
beha=ALProxy('ALBehaviorManager',IP,PORT)
# beha.runBehavior("tand")
headpitchangle = 1.433
redBallTracker
#Setiranje Head Stiffness na ON.
motionProxy.setStiffnesses("Head", 1.0)
# Setiranje na agolot na glavata na 0; podocneznata proverka dali topceto e levo ili desno ke ja pravime vrz osnova na agolot na glava.
motionProxy.setAngles(['HeadYaw', 'HeadPitch'], [0.0, headpitchangle], 0.07)
motionProxy.setAngles(['HeadYaw', 'HeadPitch'], [0.0, headpitchangle], 0.07)
#Startuvanje na red ball tracker.
redBallTracker.startTracker()

useSensor = True
a=0
b=0
#niza od joints na leva raka, desna raka, leva noga i desna noga, podocna se dodeluvaat vrednosti na ovie nizi
JointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
JointNamesL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
JointNAmesLL = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "RAnkleRoll"]
JointNamesRR = ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "LAnkleRoll"]
motionProxy.setAngles(['HeadYaw', 'HeadPitch'], [0.0, headpitchangle], 0.07)
#brzina na krevanje leva ili desna raka za 50 stepeni; se toa go ima vo if-ovite podolu
pFractionMaxSpeed = 1.0

#se dodeka e true; beskonecen ciklus
while(1):
    #print motionProxy.getAngles("HeadYaw", useSensor)[0]
    if motionProxy.getAngles("HeadYaw", useSensor)[0] < 0:
		#proveruvame dali glavata e zavrtena na desno
        print "right arm on-left arm off"
        Arm1 = [-50,  0, 0, -40]
        Arm1 = [ x * motion.TO_RAD for x in Arm1]
        motionProxy.angleInterpolationWithSpeed(JointNamesR, Arm1, pFractionMaxSpeed)
        Arm2 = [50,  0, 0, 40]
        Arm2 = [ x * motion.TO_RAD for x in Arm2]
        motionProxy.angleInterpolationWithSpeed(JointNamesL, Arm2, pFractionMaxSpeed)
        # LeftLeg  = [0.,20.,-55.,125.7,-75.7, 20]
        # LeftLeg = [x * motion.TO_RAD for x in LeftLeg]
        # motionProxy.angleInterpolationWithSpeed(JointNAmesLL, LeftLeg, 0.5)
        # RightLeg = [+0, +0, +0, +100, -29.6, +0]
        # RightLeg = [x * motion.TO_RAD for x in RightLeg]
        # motionProxy.angleInterpolationWithSpeed(JointNamesRR, RightLeg, 0.5)
    elif motionProxy.getAngles("HeadYaw", useSensor)[0] > 0:
        #proveruvame dali glavata e zavrtena na levo
        print "left arm on-right arm off"
		#levata raka ja podiga za 50stepni, desnata ja spusta za 50
        Arm1 = [50,  0, 0, -40]
        Arm1 = [ x * motion.TO_RAD for x in Arm1]
        motionProxy.angleInterpolationWithSpeed(JointNamesR, Arm1, pFractionMaxSpeed)
        Arm2 = [-50,  0, 0, 40]
        Arm2 = [ x * motion.TO_RAD for x in Arm2]
        motionProxy.angleInterpolationWithSpeed(JointNamesL, Arm2, pFractionMaxSpeed)
		#kod kade sto se odreduva pagjanjeto na naoto, na stranata koja sto e topceto; vo slucajov leva
        LeftLeg  = [0.,20.,-55.,125.7,-75.7, 20]
        LeftLeg = [x * motion.TO_RAD for x in LeftLeg]
        motionProxy.angleInterpolationWithSpeed(JointNAmesLL, LeftLeg, 0.5)
        RightLeg = [+0, +0, +0, +100, -29.6, +0]
        RightLeg = [x * motion.TO_RAD for x in RightLeg]
        motionProxy.angleInterpolationWithSpeed(JointNamesRR, RightLeg, 0.5)
    time.sleep(2.0)



redBallTracker.stopTracker()
motionProxy.setStiffnesses("Head", 0.0)
print "ALRedBallTracker stopped."
