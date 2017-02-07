#!/usr/bin/env python

import rospy
import std_msgs
import geometry_msgs.msg
from leg import Leg
from hexapod import Hexapod
from utils import Point

class LegNode:
    def __init__(self, leg, legindex):
        self.coxapub = rospy.Publisher(legindex+'_coxa_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.femurpub = rospy.Publisher(legindex+'_femur_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.tibiapub = rospy.Publisher(legindex+'_tibia_controller/command', std_msgs.msg.Float64, queue_size=10)

        rospy.Subscriber(legindex+'/position', geometry_msgs.msg.Point, self.callback)

        self.leg = leg

    def publish(self):
        #print "Publishing " +  str(self.leg.femur)
        self.coxapub.publish(self.leg.coxa)
        self.femurpub.publish(self.leg.femur)
        self.tibiapub.publish(self.leg.tibia)
    
    def callback(self, position):
        self.leg.setPosition(position.x, position.y, position.z)

hexapod = Hexapod()
flposPub = rospy.Publisher('flpos', geometry_msgs.msg.Point, queue_size=10)
publisher = [LegNode(hexapod.legs[0], 'fl'),
                LegNode(hexapod.legs[1], 'ml'),
                LegNode(hexapod.legs[2], 'rl'),
                LegNode(hexapod.legs[3], 'fr'),
                LegNode(hexapod.legs[4], 'mr'),
                LegNode(hexapod.legs[5], 'rr')]

def updateHexapod(event):
    if(event.last_real != None):
        hexapod.update((event.current_real - event.last_real).to_sec())
        for leg in publisher:
            leg.publish()
        #publisher[0].publish()

def pycontroller():
    rospy.init_node('hexapod_pycontroller', anonymous=True)
    print "Init done"
    hexapod.setCommand(Point(0.02, 0, 0))
    timer = rospy.Timer(rospy.Duration(0.02), updateHexapod)
    rospy.spin()

