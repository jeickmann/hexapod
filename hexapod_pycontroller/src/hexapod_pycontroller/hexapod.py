#!/usr/bin/env python

import time
from utils import Point
from leg import Leg

class Hexapod:
    def __init__(self):
        self.vDir = Point()
        self.vRot = Point()

        self.legs = [
            Leg(Point(0.0575, 0.0315, 0.03)),
            Leg(Point(0, 0.0315, 0.03)),
            Leg(Point(-0.0575, 0.0315, 0.03)),
            Leg(Point(0.0575, -0.0315, 0.03)),
            Leg(Point(0, -0.0315, 0.03)),
            Leg(Point(-0.0575, -0.0315, 0.03))
        ]

        #self.legs[0].position.x -= 0.04
        #self.legs[0].setPosition()

        self.legs[0].post = self.legs[1]
        self.legs[1].ante = self.legs[0]

        self.legs[1].post = self.legs[2]
        self.legs[2].ante = self.legs[1]
        
        self.legs[3].post = self.legs[4]
        self.legs[4].ante = self.legs[3]

        self.legs[4].post = self.legs[5]
        self.legs[5].ante = self.legs[4]

        #also connect front legs and rear legs to prevent them lifting at the same time
        self.legs[0].ante = self.legs[3]
        self.legs[3].ante = self.legs[0]
        self.legs[2].post = self.legs[5]
        self.legs[5].post = self.legs[2]
        pass

    def setCommand(self, vDir, vRot = None):
        self.vDir = vDir
        if(vRot != None):
            self.vRot = vRot

    def update(self, step):
        ##TODO update all legs after testing
        #self.legs[1].update(step, self.vDir, self.vRot)
        for leg in self.legs:
           leg.update(step, self.vDir, self.vRot)