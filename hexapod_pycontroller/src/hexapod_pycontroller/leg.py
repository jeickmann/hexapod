import rospy
import std_msgs

from math import sqrt, atan2, acos, pi

class Leg:
    coxaHeight = 0.03
    femurLength = 0.053
    tibiaLength = 0.088

    def __init__(self):
        self.coxa = 0
        self.femur = 0.7
        self.tibia = -2

    def setPosition(self, pX, pY, pZ):
        z = pZ + Leg.coxaHeight
        self.coxa = -atan2(pX, pY)
        
        G = sqrt(pX*pX + pY*pY)
        L = sqrt(z*z + G*G)
        
        a1 = acos(z/L)
        a2 = acos((Leg.tibiaLength*Leg.tibiaLength-Leg.femurLength*Leg.femurLength-L*L)/(-2*Leg.femurLength*L))
        self.femur = a1+a2-pi/2
        self.tibia = -pi+acos((L*L-Leg.tibiaLength*Leg.tibiaLength-Leg.femurLength*Leg.femurLength)/(-2*Leg.tibiaLength*Leg.femurLength))