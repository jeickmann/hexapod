#!/usr/bin/env python

import rospy
import std_msgs
from leg import Leg

class Legpublisher:
    def __init__(self, leg, legindex):
        self.coxapub = rospy.Publisher('/hexapod/'+legindex+'_coxa_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.femurpub = rospy.Publisher('/hexapod/'+legindex+'_femur_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.tibiapub = rospy.Publisher('/hexapod/'+legindex+'_tibia_controller/command', std_msgs.msg.Float64, queue_size=10)
        self.leg = leg

    def publish(self):
        self.coxapub.publish(self.leg.coxa)
        self.femurpub.publish(self.leg.femur)
        self.tibiapub.publish(self.leg.tibia)
    

def pycontroller():
    publisher = [Legpublisher(Leg(), 'fl'),
                Legpublisher(Leg(), 'fr'),
                Legpublisher(Leg(), 'ml'),
                Legpublisher(Leg(), 'mr'),
                Legpublisher(Leg(), 'rl'),
                Legpublisher(Leg(), 'rr')]

    publisher[0].leg.setPosition(0, 0.07, 0.01)

    publisher[2].leg.setPosition(0, 0.08, 0.01)
    rospy.init_node('hexapod_pycontroller', anonymous=True)
    print "Init done"
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown(): 
        for p in publisher:
            p.publish()

        rate.sleep()
