"""
This example shows how to use ALRedBallTracker.
It is launched for a little while, then stopped.
"""

import sys
import time
from naoqi import *
import NAO_motion
import NAO_sensor
import time
import math
import motion

class KeepTheGoal():

    def __init__(self, logger, config):


        logger.info("Controller-Class initialized")

        self.logger = logger
        self.config = config
        self.motion = NAO_motion.Motion(logger, config)
        self.sensor = NAO_sensor.Sensor(logger, config)
        self.speech = config.getProxy("ALTextToSpeech")

        # How close Nao will try to get to the ball in meters(>0.25)
        self.distanceToTarget = 0.27
        # How many times the ball can be out of sight before Nao stops
        self.ballLostMax = 10
        # Time the head movement stops for ball recognition
        self.retardSecond = 2
        # Nao walking speed 0.0 to 1.0
        self.walkingSpeed = 0.3
        # An obstacle ahead, lower than this value will cause Nao to stop
        self.maxSonar = 0.3
        # Time the thread sleeps for the next walking iteration in seconds
        self.walkIterationTime = 0.05
        self.rotations = 0
        self.isStop = False
        self.firstIteration = True


    def start(self):
        ###
        # Summary: start method with general setup calls
        # Parameters: self
        # Return: 1 if its stoped
        ###

        if (self.isStop):
            self.end()
            return 1

        self.logger.info("Controller: Starting...")

        self.motion.getBehaviors()
        # The maximum volume of text-to-speech outputs
        self.speech.setVolume(0.5)

        self.speech.say("Standing up")
        self.motion.standUp()

        # We need to close the hands here, because Nao sometimes opens them after standing up
        self.motion.closeBothHands()

        # stop head tracking
        self.sensor.stopHeadTracker()

        self.lookForBallCloseRange()


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
