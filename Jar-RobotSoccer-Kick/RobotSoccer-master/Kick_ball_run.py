from naoqi import ALProxy
IP = '169.254.212.54'
PORT = 9559

manage = ALProxy("ALBehaviorManager", IP, PORT)
manage.runBehavior("kick_ball-098ccf")
