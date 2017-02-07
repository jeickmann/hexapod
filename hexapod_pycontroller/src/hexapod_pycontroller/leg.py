import rospy

from utils import Point
from math import sqrt, atan2, acos, pi, exp, log, atan2, copysign

class VectorField:
    constantSpeed = 1
    PEPLift = 1
    
    @staticmethod
    def evaluate(p):
        return (VectorField.constantField(p)
                + VectorField.PEPField(p)
                + VectorField.centerField(p)
                + VectorField.zTopField(p)
                + VectorField.AEPField(p))

    @staticmethod
    def constantField(p):
        return Point(VectorField.constantSpeed, 0, 0)
    
    @staticmethod
    def PEPField(p):
        strengthX = min(1, max(0, 1-p.x/0.4))
        strengthZ = min(1, max(0, 1-p.z/0.4))
        absStrength = min(strengthX, strengthZ)
        return Point(-absStrength*VectorField.constantSpeed, 0, absStrength*VectorField.PEPLift)
    
    @staticmethod
    def centerField(p):
        transformedP = p - Point(0.5, 0, 0);
        transformedP.x *= 2
        factor = 1 - max(1, transformedP.abs())
        return transformedP * factor

    @staticmethod
    def zTopField(p):
        force = max(0, p.z-1)
        return Point(0, 0, -force)

    @staticmethod
    def AEPField(p):
        force = 0
        if(p.x > 0.8):
            force = min(VectorField.constantSpeed, p.x * VectorField.constantSpeed)
        
        return Point(-force, 0, -force)

class LegMode:
    STANCE = 0
    SWING = 1

def evaluateConstraint(minimum, maximum, transitionWidth, value):
    median = (maximum+minimum)/2
    factor = log(0.1)/transitionWidth
    if(value > median):
        return exp(factor*(maximum-value))
    else:
        return exp(factor*(value- minimum))


MIN_TIBIA_ANGLE = -2.6
MIN_FEMUR_ANGLE = -0.25
MAX_FEMUR_ANGLE = 1.5
MIN_COXA_ANGLE = -1
MAX_COXA_ANGLE = 1

MIN_VALUE = -1000000
MAX_VALUE = 1000000


class Leg:
    coxaHeight = 0.03
    femurLength = 0.053
    tibiaLength = 0.088

    legCenterX = 0
    legCenterY = 0.07

    initialStepLength = 0.04
    initialStepHeight = 0.02

    walkingHeight = 0.05

    kPredict = 2.97
    RequalsOne = 1.075

    def __init__(self, bodyOffset):
        self.bodyOffset = bodyOffset
        self.localHome = bodyOffset + Point(0, copysign(self.legCenterY, bodyOffset.y), 0)
        self.coxa = 0
        self.femur = 0
        self.tibia = 0
        self.mode = LegMode.STANCE
        self.ante = None
        self.post = None
        self.lastRestricted = 0
        self.deltaR = 0
        self.lastRCoxa = 0
        self.position = Point(0, copysign(0.07, bodyOffset.y), -0.05) + bodyOffset
        self.position.z = -self.walkingHeight
        self.setPosition()

    # Position in Body Space
    def setPosition(self, pos = None):
        if(pos != None):
            self.position.set(pos)
        
        localPos = self.position - self.bodyOffset
        if(self.bodyOffset.y < 0):
            localPos.y *= -1

        modeStr = "STANCE"
        if(self.mode == LegMode.SWING):
            modeStr = "SWING"
        d = 0
        if(self.post != None):
            d = self.distanceToLeg(self.post)

        #print "Mode: " + modeStr + " - SettingPos " + str(localPos) + " " +  str(d)
        
        z = -localPos.z
        self.coxa = -atan2(localPos.x, localPos.y)
        
        G = sqrt(localPos.x*localPos.x + localPos.y*localPos.y)
        L = sqrt(z*z + G*G)
        
        a1 = acos(z/L)
        a2 = acos((Leg.tibiaLength*Leg.tibiaLength-Leg.femurLength*Leg.femurLength-L*L)/(-2*Leg.femurLength*L))
        self.femur = a1+a2-pi/2
        self.tibia = -pi+acos((L*L-Leg.tibiaLength*Leg.tibiaLength-Leg.femurLength*Leg.femurLength)/(-2*Leg.tibiaLength*Leg.femurLength))


    def update(self, step, vDir, vRot):
        #TODO add delayed swing action activation (section 4.4.3) and swing priority (4.4.4)
        currentR = self.getRestrictedness()
        if(currentR - self.lastRestricted != 0):
            self.deltaR = currentR  - self.lastRestricted
        self.lastRestricted = currentR

        #is it even possible to switch to Swing?
        if(self.mode == LegMode.STANCE 
            and (self.ante == None or self.ante.mode == LegMode.STANCE) 
            and (self.post == None or self.post.mode == LegMode.STANCE)):
            if(currentR > 0.95 and self.deltaR > 0):
                self.switchToSwing()
        elif(self.mode == LegMode.SWING and self.isOnGround()):
            self.switchToStance()

        if(self.mode == LegMode.STANCE):
            if(currentR < 1):
                self.advanceStance(step, vDir, vRot)
        else:
            self.advanceSwing(step, vDir, vRot)

    def isOnGround(self):
        return self.position.z <= -Leg.walkingHeight

    def switchToSwing(self):
        print "Switching to swing"
        self.mode = LegMode.SWING
        self.lastPEP = self.position.clone()
        self.stepLength = Leg.initialStepLength
        self.stepHeight = Leg.initialStepHeight
        self.lastRCoxa = 1000000000

    def switchToStance(self):
        print "Switching to Stance"
        self.mode = LegMode.STANCE

    def advanceStance(self, step, vDir, vRot):
        self.position += vDir*-step
        
        self.setPosition() #update with that selfsame position

    def normalizePos(self, pos, lastPEP):
        return Point((pos.x-lastPEP.x)/self.stepLength, 0, (pos.z-lastPEP.z)/(self.stepHeight-lastPEP.z))

    def toTaskCoordinate(self, pos, vDir, vRot):
        angle = atan2(vDir.y, vDir.x) 
        taskPos = pos - self.localHome
        return taskPos.rotateZ(-angle)

    def fromTaskCoordinate(self, pos, vDir, vRot):
        angle = atan2(vDir.y, vDir.x) 
        #TODO apply transformation
        alignedPos = pos.rotateZ(angle)
        return alignedPos + self.localHome

    #TODO not needed and currently incorrect
    def denormalizeVelocities(self, v):
        return v.clone()
        #return Point(v.x*self.stepLength, 0, v.z*(self.stepHeight-self.lastPEP.z))

    def predictR(self, xStar):
        return exp(Leg.kPredict*(xStar-Leg.RequalsOne))

    def advanceSwing(self, step, vDir, vRot):
        taskPos = self.toTaskCoordinate(self.position, vDir, vRot)
        taskCoordPEP = self.toTaskCoordinate(self.lastPEP, vDir, vRot)
        normalizedPos = self.normalizePos(taskPos, taskCoordPEP)
        #do we need to shorten the step?
        rSwing = self.getSwingRestrictedness()
        rPredict = self.predictR(normalizedPos.x)
        if(rSwing > rPredict):
            normalizedPos.x = Leg.RequalsOne + log(rSwing)/Leg.kPredict
            self.stepLength = (taskPos.x-taskCoordPEP.x)/normalizedPos.x
            print str(normalizedPos.x) + "Shortening step to " + str(self.stepLength)

        #print str(normalizedPos.x) + "," + str(rSwing) + ", " + str(rPredict)

        vector = VectorField.evaluate(normalizedPos)
        taskVelocities = self.denormalizeVelocities(vector)
        taskPosNew = taskPos + taskVelocities * step * vDir.abs()
        #TODO make y converge slowly...
        #taskPosNew.y = 0
        newPos = self.fromTaskCoordinate(taskPosNew, vDir, vRot)
        self.setPosition(newPos)
        pass

    def distanceTo(self, x, y):
        dX = self.position.x - x
        dY = self.position.y - y
        return sqrt(dX*dX+dY*dY)

    def distanceToLeg(self, leg):
        return self.distanceTo(leg.position.x, leg.position.y)

    def getRestrictedness(self):
        rTibia = evaluateConstraint(MIN_TIBIA_ANGLE, 0, 0.5, self.tibia)
        rFemur =  evaluateConstraint(MIN_FEMUR_ANGLE, MAX_FEMUR_ANGLE, 0.5, self.femur)
        rCoxa =  evaluateConstraint(MIN_COXA_ANGLE, MAX_COXA_ANGLE, 0.5, self.coxa)
        rSingularity = evaluateConstraint(0.01, MAX_VALUE, 0.02, self.distanceTo(self.bodyOffset.x, self.bodyOffset.y))
        rWorkspace = evaluateConstraint(MIN_VALUE, 0.04, 0.02, self.distanceTo(self.localHome.x, self.localHome.y))
        restrictedness = rTibia + rFemur + rCoxa + rSingularity + rWorkspace
        if(self.mode == LegMode.SWING):
            restrictedness += evaluateConstraint(0.005, MAX_VALUE, 0.04, self.distanceToLeg(self.ante))
        else:
            restrictedness += evaluateConstraint(0.005, MAX_VALUE, 0.04, self.distanceToLeg(self.post))

        return restrictedness

    def getSwingRestrictedness(self):
        rCoxa = evaluateConstraint(MIN_COXA_ANGLE, MAX_COXA_ANGLE, 0.25, self.coxa)
        rAnte = evaluateConstraint(0.005, MAX_VALUE, 0.02, self.distanceToLeg(self.ante))
        rWorkspace = evaluateConstraint(MIN_VALUE, 0.04, 0.01, self.distanceTo(self.localHome.x, self.localHome.y))

        rSwing = rAnte
        if(rCoxa - self.lastRCoxa > 0):
            #print "Increasing" +  str(rCoxa)
            rSwing += rCoxa + rWorkspace
        
        self.lastRCoxa = rCoxa
        return rSwing