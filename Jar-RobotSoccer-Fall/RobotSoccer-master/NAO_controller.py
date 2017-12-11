# coding:utf-8
"""
# Summary: controller class for the general line of execution
# Parameters: none
# Return: --
"""
from naoqi import ALProxy

import NAO_motion
import NAO_sensor
import time
import math
import motion


class Controller():
    
    def __init__(self, logger, config):
       
    ###
    # Summary: main method for setting up proxys and variables
    # Parameters: self, logger in which we will write all the important messages and config to
    #              activate all the configurations
    # Return: --
    ###

        # We initialize Controller Class
        logger.info("Controller-Class initialized")
        
        self.logger = logger
        self.config = config
        self.motion = NAO_motion.Motion(logger, config)
        self.sensor = NAO_sensor.Sensor(logger, config)
        self.speech = config.getProxy("ALTextToSpeech")
    #How close Nao will try to get to the ball in meters(>0.25)
        self.distanceToTarget = 0.26
        #How many times the ball can be out of sight before Nao stops
        self.ballLostMax = 10 
        #Time the head movement stops for ball recognition
        self.retardSecond = 2
        #Nao walking speed 0.0 to 1.0
        self.walkingSpeed = 0.1
        #An obstacle ahead, lower than this value will cause Nao to stop 
        self.maxSonar = 0.3 
        #Time the thread sleeps for the next walking iteration in seconds
        self.walkIterationTime = 0.05
        self.rotations = 0
        self.isStop = False
        self.firstIteration = True
        self.waitTime= 2

        
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
        #The maximum volume of text-to-speech outputs
        self.speech.setVolume(0.7)
        
        # self.speech.say("Standing up")
        self.motion.standUp()

        # self.motion.kickBall()
        # self.motion.moveTo(0.05,0,0)
        # self.motion.moveTo(0.4,0,0)

        #
        # We need to close the hands here, because Nao sometimes opens them after standing up
        # self.motion.closeBothHands()
        
        # stop head tracking
        # self.sensor.stopHeadTracker()

        # self.lookForBallCloseRange()
        self.keepGoal()

        
    def lookForBallCloseRange(self):
    ###
    # Summary: used to look for the red ball in close range (0.2-1.2m)
    # Parameters: self
    # Return: 1 if its stopped
    ###    
        
        if (self.isStop):
            self.end()
            return 1
        
        
        # self.speech.say("Looking short")
        # activate camera, 0top 1bottom
        self.sensor.setCamera(1)
        self.sensor.subscribeToRedball()

        #Look for the ball straight ahead
        
        # if(self.firstIteration):
        #     time.sleep(2)
        #     self.firstIteration = False

        if (self.sensor.isNewBall()):
            # New Ball found
            self.ballFound()
            return 1
        # #Look for the ball on the left
        # self.motion.turnHead(30, 0.5) 
        # time.sleep(self.retardSecond)
        # if(self.sensor.isNewBall()):
        #     #New Ball found          
        #     self.motion.turnAround(30)
        #     self.ballFound()
        #     return 1
        
        # #Look for the ball on the right       
        # self.motion.turnHead(-30, 0.5)     
        # time.sleep(self.retardSecond)
        # if(self.sensor.isNewBall()):
        #     #New Ball found   
        #     self.motion.turnAround(-30)
        #     self.ballFound()
        #     return 1


        #"Could not find my ball" message    
        # self.speech.say("Could not find my ball")
        self.sensor.unsubscribeToRedBall()
        
        if(self.rotations != 5):
            #Recursion
            # moving head
            self.motion.turnHead(0, 0.1)
            # moving body
            self.motion.turnAround(60)
            # rotating robot
            self.rotations = self.rotations + 1
            self.lookForBallCloseRange()
        else:
            self.end()
        
            
            
    def ballFound(self):    
    ###
    # Summary: Called when the ball has been found
    # Parameters: self
    # Return: 1 if it s stopped
    ###    
        
        if (self.isStop):
            self.end()
            return 1
        
        #"I found my ball" message 
        # self.speech.say("I found my ball")
        
        self.rotations = 0
        # move head
        self.motion.turnHead(0, 0.5) 
        # Nao walks to ball
        self.walkToBall()
        


    def walkToBall(self):
    ###
    # Summary: Used to walk to the red ball target
    # Parameters: self
    # Return: 1 if it s stopped
    ###        
        
        if (self.isStop):
            self.end()
            return 1

        #"Looking at my ball" message 
        # self.speech.say("Looking at my ball")
        
        ballLost = 0
        atBall = False
        
        #Starting the sensors 
        self.sensor.startHeadTracker()
        self.sensor.startSonar()
        flag=True
        while(atBall == False):
            if (self.isStop):
                self.sensor.stopHeadTracker()
                self.sensor.stopSonar()
                self.end()
                return 1
                
            time.sleep(self.walkIterationTime)
        
            headAngle = self.motion.getSensorValue("HeadYaw")[0]
            
            #Check whether or not we are looking at our own shoulders
            if(headAngle < - 0.75 or headAngle > 0.75):
                self.logger.info("HeadYaw: " + str(headAngle))
                self.sensor.stopHeadTracker()
                self.motion.turnHead(0.0, 0.5)
                self.sensor.startHeadTracker()
            
            # get the ball position
            x = self.sensor.getBallPosition()[0]
            y = self.sensor.getBallPosition()[1]

            self.distance = math.sqrt(math.pow(x,2)+math.pow(y,2))
            angle = math.atan2(y, x)
            angleRounded = int(angle/(5.0*motion.TO_RAD))*(5.0*motion.TO_RAD)
            
            #The walking velocity angle must be between -1 and 1
            if(angleRounded>1):
                angleRounded = 1
            if(angleRounded<-1):
                angleRounded = -1
            
            self.logger.info("Ball at: " + str(x) + "," + str(y) + " with " + str(angleRounded) + " in " + str(self.distance))
            
            #Reducing the speed the closer we get to the ball
            speed = self.walkingSpeed
            # while (self.distance>=self.distanceToTarget):
            #     # self.motion.moveTo(0.1, y, 0)
            #     x = self.sensor.getBallPosition()[0]
            #     y = self.sensor.getBallPosition()[1]
            #     self.distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
            #
            # while (self.distance>=self.distanceToTarget):
            #     print ">=distance"
            #     x = self.sensor.getBallPosition()[0]
            #     y = self.sensor.getBallPosition()[1]
            #     self.motion.moveTo(x-0.05, y, 0)
            #     x = self.sensor.getBallPosition()[0]
            #     y = self.sensor.getBallPosition()[1]
            #     self.distance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))


            if(self.distance <= self.walkingSpeed):
                speed = self.distance
            if(self.sensor.isNewBall() == False):
                ballLost = ballLost + 1
                self.logger.info("Ball lost?")
            else:
                ballLost = 0
                
            #Collision detection and reaction
            self.colLeft = False
            self.colRight = False
            
            if(self.sensor.getSonarLeft() <= self.maxSonar):
                self.colLeft = True
                
            if(self.sensor.getSonarRight() <= self.maxSonar):
                self.colRight = True
            #
            # self.tooClose(self.colLeft, self.colRight)
            #
            # # If we lost sight of the ball a certain amount
            # if(ballLost >= self.ballLostMax):
            #     self.speech.say("I lost track of my ball")
            #     atBall = True
            #     self.motion.stopEverything()
            #     self.sensor.stopHeadTracker()
            #     self.sensor.stopSonar()
            #     self.motion.standUp()
            #     self.lookForBallCloseRange()
            #     return 1
            
            #If we reached our target distance
            if(True):
                self.logger.info("At my Target")
                self.motion.stopEverything()
                self.sensor.stopHeadTracker()
                atBall = True
                
                self.motion.standUp()
        
                # self.speech.say("Final correction")

                # self.motion.turnAround(math.degrees(angle))

                x = self.sensor.getBallPosition()[0]
                y = self.sensor.getBallPosition()[1]
                ballDistance = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
                # write into log file the position of the ball
                self.logger.info("Near Ball distance: " + str(ballDistance))
                # Nao rotates around the ball
                # self.motion.rotateAroundBall(ballDistance, 30)

                self.sensor.stopSonar()
                self.findGoal()
                return 1        
                
    
        
    # def findGoal(self):
    # ###
    # # Summary: Method for finding the goal
    # # Parameters: self
    # # Return: 
    # ###   
    
    #     if (self.isStop):
    #         # end and return 1 if Nao is stopped
    #         self.end()
    #         return 1
                
    #     # self.speech.say("Looking for the goal")
    #     # self.motion.standUp()

    #     x = self.sensor.getBallPosition()[0]
    #     y = self.sensor.getBallPosition()[1]

    #     self.motion.moveTo(x-0.015 , y, 0)

    #     # self.motion.moveTo(x-0.015 , 0, 0)

    #     # self.motion.standUp()
    #     # time.sleep(0.05)
    #     self.motion.kickBall()
    #     # set on the bottom camera. 0top 1bottom 
        
    def findGoal(self):
        ###
        # Summary: Method for finding the goal
        # Parameters: self
        # Return: 
        ###   

        if (self.isStop):
            # end and return 1 if Nao is stopped
            self.end()
            return 1
                
        # self.speech.say("Looking for the goal")
        self.motion.standUp()

        x = self.sensor.getBallPosition()[0]
        y = self.sensor.getBallPosition()[1]
        # self.motion.moveTo(x-0.20,y-0.05, 0)

        self.motion.moveTo(x - 0.25,y, 0)
        self.motion.standUp()

        x = self.sensor.getBallPosition()[0]
        y = self.sensor.getBallPosition()[1]

        self.motion.moveTo(x - 0.07, y-0.11, 0)
        # time.sleep(0.5)
        self.motion.kickBall()
    def tooClose(self, colLeft, colRight):
    ###
    # Summary: this method control robot in case of collisions
    # Parameters: self, colLeft and colRight controls collisions from left and right sides
    # Return: --
    ###   
    
        if (self.isStop):
            self.end()
            return 1
        
        # if(colLeft and colRight):
        #     # if sensors detect collisions on right and left sides that means there is an obstacle
        #     self.speech.say("Warning, obstacle ahead!")
        #     # stop moving
        #     self.motion.stopEverything()
        #     # stop head tracking
        #     self.sensor.stopHeadTracker()
        #     self.end()
        # else:
        #     if(colLeft):
        #         # if left sensor detects collisions that means there is an obstacle on the left side
        #         self.logger.info("Colliding left")
        #         self.motion.stopEverything()
        #         self.sensor.stopHeadTracker()
        #         self.motion.moveTo(0, -0.2, 0)
        #         self.lookForBallCloseRange()
        #     if(colRight):
        #         # if right sensor detects collisions that means there is an obstacle on the right side
        #         self.logger.info("Colliding right")
        #         self.motion.stopEverything()
        #         self.sensor.stopHeadTracker()
        #         self.motion.moveTo(0, 0.2, 0)
        #         self.lookForBallCloseRange()
               

    def end(self):
    ###
    # Summary: method invokated in case of finishing 
    # Parameters: self
    # Return: --
    ###   
        # Nao stops every move
        self.motion.stopEverything()
        # Nao says Bye Bye
        self.speech.say("Bye Bye")
        # remove datas of the ball
        self.sensor.removeBallData()
        # Nao will be in resting position
        self.motion.rest()
    def keepGoal(self):
        self.motion.motion.setFallManagerEnabled(False)
        # self.motion.moveTo(0.2, 0,0)
        # Nao rotates around the ball
        self.sensor.setCamera(1)
        print "setcamera"
        self.sensor.subscribeToRedball()
        self.sensor.startHeadTracker()
        print "subscribeToRedball"

        # x2=0
        # y2=0
        # print "x2=0,y2=0!!!"
        headpitchangle = -20 * motion.TO_RAD
        self.motion.motion.setAngles(['HeadYaw', 'HeadPitch'], [0, headpitchangle],0.07 )



        while(True):
            # time.sleep(1)
        # while (1):
        #     print "In the while,x2=0,y2=0!!!"
            self.motion.motion.setAngles([ 'HeadPitch'], [ headpitchangle], 0.07)
            x2 = float(self.sensor.getBallPosition()[0])
            y2 = float(self.sensor.getBallPosition()[1])
            z2 = float(self.sensor.getBallPosition()[2])

            if(x2>1.6 or x2 < 0.5 ):
                # x2=0
                # y2=0
                print "into the if","x2=",x2,"y2=",y2
                continue
            else :
                x3= float(self.sensor.getBallPosition()[0])
                y3= float(self.sensor.getBallPosition()[1])
                z3=float(self.sensor.getBallPosition()[2])
                print x3,y3,z3
                if x3 != x2:
                    self.fall(y3)
                    # time.sleep(10)
                    # self.motion.standUp()
                    break
                else:
                    continue

            # print ("x",x1,x2)
            # print ("y", y1, y2)
            # while(y1==y2):
            #     x2 = float(self.sensor.getBallPosition()[0])
            #     y2 = float(self.sensor.getBallPosition()[1])
            #     print ("same")
            # y3=x1-y1*(x1-x2)/(y1-y2)
            # self.motion.moveTo(0, y3 - 0.05, 0)
            # y3=1200*x2/(1200-y2)
            # print x2,y2,z2
            self.sensor.unsubscribeToRedBall()
        # redBallTracker.stopTracker()
        # motionProxy.setStiffnesses("Head", 0.0)
        # print "ALRedBallTracker stopped."


    def fall(self, direction):
        #´óÓÚ0 Ïò×óµ¹
        IP = self.config.IP
        PORT = self.config.PORT
        print "Connecting to", IP, "with port", PORT
        motionProxy = ALProxy("ALMotion", IP, PORT)
        # redBallTracker = ALProxy("ALRedBallTracker", IP, PORT)
        # headpitchangle = -20* motion.TO_RAD
        # Setiranje Head Stiffness na ON.

        motionProxy.setStiffnesses("Head", 1.0)
        # Setiranje na agolot na glavata na 0; podocneznata proverka dali topceto e levo ili desno ke ja pravime vrz osnova na agolot na glava.
        # motionProxy.setAngles(['HeadYaw', 'HeadPitch'], [0.0, headpitchangle], 0.07)
        # Startuvanje na red ball tracker.
        # redBallTracker.startTracker()

        useSensor = True
        a = 0
        b = 0
        # niza od joints na leva raka, desna raka, leva noga i desna noga, podocna se dodeluvaat vrednosti na ovie nizi
        JointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll"]
        JointNamesL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
        JointNAmesLL = ["LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "RAnkleRoll"]
        JointNamesRR = ["RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "LAnkleRoll"]
        # motionProxy.setAngles(['HeadYaw', 'HeadPitch'], [0.0, headpitchangle], 0.07)
        # brzina na krevanje leva ili desna raka za 50 stepeni; se toa go ima vo if-ovite podolu
        pFractionMaxSpeed = 1.0

            # xÎªÇ°ºó·½Ïò£¨ÕýÎªÇ°£¬¸ºÎªºó£© yÎª×óÓÒ·½Ïò£¨ÕýÎª×ó£¬¸ºÎªÓÒ£©
            # JLÎª×ó JRÎªÓÒ
            # ¹Ø¼üÖµÎª¸ºÎªÉÏ
            # 1¼ç°ò  2¼çºáÏò 3ÖâºáÏò£¿   4ÖâÏòÉÏ
            # print motionProxy.getAngles("HeadYaw", useSensor)[0]
        if direction > 0:
            # proveruvame dali glavata e zavrtena na desno
            print "left arm on-left arm off"
            print "left fall"
            # Arm1 = [50, 0, 0, 40]
            # Arm1 = [x * motion.TO_RAD for x in Arm1]
            # motionProxy.angleInterpolationWithSpeed(JointNamesR, Arm1, pFractionMaxSpeed)
            Arm2 = [-90, 0, 0, 0]
            Arm2 = [x * motion.TO_RAD for x in Arm2]

            LeftLeg = [0., 20., -55., 125.7, -75.7, 20]
            LeftLeg = [x * motion.TO_RAD for x in LeftLeg]
            motionProxy.angleInterpolationWithSpeed(JointNamesL + JointNAmesLL, Arm2 + LeftLeg, pFractionMaxSpeed)

            # motionProxy.angleInterpolationWithSpeed(JointNAmesLL, LeftLeg, 0.5)

            time.sleep(0.5)
            LeftLeg = [0] *6
            LeftLeg = [x * motion.TO_RAD for x in LeftLeg]
            motionProxy.angleInterpolationWithSpeed(JointNAmesLL, LeftLeg, 0.5)

        elif direction < 0:  #
            print "right fall"
            # proveruvame dali glavata e zavrtena na levo
            print "Right arm on-right arm off"
            # levata raka ja podiga za 50stepni, desnata ja spusta za 50
            Arm1 = [-90, 0, 0, 0]
            Arm1 = [x * motion.TO_RAD for x in Arm1]
            motionProxy.angleInterpolationWithSpeed(JointNamesR, Arm1, pFractionMaxSpeed)


            # kod kade sto se odreduva pagjanjeto na naoto, na stranata koja sto e topceto; vo slucajov leva
            RightLeg = [0., 20., -55., 125.7, -75.7, 20]
            RightLeg = [x * motion.TO_RAD for x in RightLeg]
            motionProxy.angleInterpolationWithSpeed(JointNamesRR, RightLeg, 0.5)
            time.sleep(0.5)

            #ÉìÖ±
            RightLeg = [0., 0, 0, 0, 0, 0]
            RightLeg = [x * motion.TO_RAD for x in RightLeg]
            motionProxy.angleInterpolationWithSpeed(JointNamesRR, RightLeg, 0.5)




