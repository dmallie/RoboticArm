from __future__ import division
import time
import mysql.connector
from mysql.connector import Error
#from Modules import distanceSensor
import numpy as np
import os

import Ultrasonic
import Servo
import Mapping
import Adafruit_PCA9685

#import RPi.GPIO as GPIO

conn = None
cursor = None
config = {
        'user':'eagles',
        'password': 'UniversityOfTurku',
        'host': '192.168.43.11',
        'database':'roboticarm',
}
#**********************************************************
class Transitions(object):
    def __init__(self, toState):
        self.toState = toState
    def Execute(self):
        print("Transitioning.....")
class State(object):
    def __init__(self, FSM):
        self.FSM = FSM
    def Enter(self):
        pass

class FSM(object):
    def __init__(self,character):
        self.states = {}
        self.transitions = {}
        self.trans = None
        self.curState = None
        self.prevState = None
        self.toState = None
        self.trackLanding = None
    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition
    def AddState(self, stateName, state):
        self.states[stateName] = state
    def SetState(self, stateName):
        self.prevState = self.curState
        self.curState = self.states[stateName]
    def ToTransition(self, stateName):
        self.trans = self.transitions[stateName]
        self.toState = stateName
        if self.toState == "Idle" or self.toState == "Grabbing":
            self.trackLanding = self.toState
    def FindState(self):
        return self.toState
    def Execute(self):
        if(self.trans):
            self.curState.Exit()
            self.trans.Execute()
            self.SetState(self.trans.toState)
            self.curState.Enter()
            self.trans = None
        self.curState.Execute()
    def AfterLanding(self):
        return self.trackLanding

#******************************************************************************
#               Startup
#******************************************************************************
class Startup(State):
    def __init__(self, FSM):
        super(Startup, self).__init__(FSM)
        self.conn = None
        self.cursor = None
    def Enter(self):
        super(Startup, self).Enter()
    def Execute(self):
        self.clearOrders()
        self.resetStatusReport()
        self.checkMotors()
        self.Exit()
    def Exit(self):
        print("******* Go to Idle state ***********")
        self.FSM.ToTransition("Idle")

    def clearOrders(self):
        deactivate = 0
        ids = []
        self.connect()
        query = '''SELECT id FROM task WHERE setOrder != %s '''
        updateQuery = '''UPDATE task SET setOrder = %s  WHERE id = %s '''
        self.cursor.execute(query,(deactivate, ))
        ret = self.cursor.fetchall()
        for rows in ret:
            ids.append(rows[0])
        for value in ids:
            self.cursor.execute(updateQuery, (deactivate, value))
            self.conn.commit()
        self.close()
    def resetStatusReport(self):
        self.connect()
        updateQuery = ''' UPDATE statusreport SET cameraStatus = %s WHERE id = %s'''
        rowNo = 1
        updateCameraStatus = 1111
        self.cursor.execute(updateQuery, (updateCameraStatus, rowNo,))
        self.conn.commit()
        self.close()
    def checkMotors(self):
        self.openGripper()
        self.moveBaseServo()
        self.adjustHeight()
        self.closeGripper()
    def openGripper(self):
        Servo.baseServo(s.gripperPin,s.openGripper)
        print("Gripper Opened!!")
        time.sleep(2)
    def closeGripper(self):
        Servo.baseServo(s.gripperPin,s.closeGripper)
        print("Gripper closed.")
        time.sleep(2)
    def moveBaseServo(self):
        rotation = "clockwise"
        index = 80
        position = s.lbAngle
        while(True):
            if rotation == "clockwise":
                print("Arm position: ", position)
                Servo.baseServo(s.baseservoPin,position)
                time.sleep(2)
                position += index
                if position > s.ubAngle:
                    rotation = "anti-clockwise"
                    position = s.ubAngle
            else:
                print("Arm position: ", position)
                Servo.baseServo(s.baseservoPin, position)
                time.sleep(2)
                position -= index
                if position < s.lbAngle:
                    break
    def adjustHeight(self):
        Servo.baseServo(s.thirdServoPin, s.heightAngle)
        time.sleep(2)
    def connect(self):
        try:
            self.conn = mysql.connector.connect(**config)
            if self.conn.is_connected():
                self.cursor = self.conn.cursor()
                #print('Connected to MySQL database')
        except Error as e:
            print(e)
    def close(self):
        self.conn.close()
        self.cursor.close()
#******************************************************************************
#               Idle
#******************************************************************************
class Idle(State):
    def __init__(self, FSM):
        super(Idle, self).__init__(FSM)
        self.conn = None
        self.cursor = None
        self.image = None
        self.id = 0
        self.name = None
        self.nextState = "Idle"
    def Enter(self):
        super(Idle, self).Enter()
    def Execute(self):
        self.checkDb()
        self.updateStatusReport()
        time.sleep(5)
        self.openGripper()
        self.Exit()
    def checkDb(self):
        self.connect()
        query = ''' SELECT id FROM task WHERE setOrder != %s '''
        queryReturn = 0
        orderValue = 0
        self.cursor.execute(query,(orderValue, ))
        queryReturn = self.cursor.fetchone()

        if queryReturn != None:
            value = int(queryReturn[0])
            self.fetchPicture()
            self.resetSetOrder()
            self.nextState = "BeforeOnair"
        else:
            self.nextState = "Idle"
            print("There is no new order!!")
        self.close()
    def fetchPicture(self):
        self.connect()
        query = ''' SELECT id, name, picture FROM task WHERE setOrder != %s '''
        orderValue = 0
        self.cursor.execute(query, (orderValue,))
        queryReturn = self.cursor.fetchone()
        self.id = int(queryReturn[0])
        self.name = queryReturn[1]
        self.image = queryReturn[2]
        self.savePicture()
        self.close()

    def savePicture(self):
        BASE_DIR = os.path.dirname(__file__)
        PIC_DIR = os.path.join(BASE_DIR, 'Picture')
        save_path = os.path.join(PIC_DIR,self.name)
        open(save_path,'wb').write(self.image)
        s.image = save_path
        s.id = self.id
    def resetSetOrder(self):
        self.connect()
        query = ''' UPDATE task SET setOrder = %s WHERE id = %s '''
        newValue = 0
        self.cursor.execute(query,(newValue, self.id))
        self.conn.commit()
        self.close()
    def updateStatusReport(self):
        self.connect()
        query = ''' UPDATE statusreport SET cameraStatus = %s, taskId = %s WHERE id = %s'''
        rowNo = 1
        updatedCameraStatus = 2111
        print("self.id: ", self.id)
        self.cursor.execute(query,(updatedCameraStatus, self.id, rowNo,))
        self.conn.commit()
        self.close()
    def writePicture(self):
        BASE_DIR = os.path.dirname(__file__)
        PIC_DIR = os.path.join(BASE_DIR, 'Picture')
        save_path = os.path.join(PIC_DIR, self.name)
        open(save_path, 'wb').write(self.image)
    def connect(self):
        try:
            self.conn = mysql.connector.connect(**config)
            if self.conn.is_connected():
                self.cursor = self.conn.cursor()
                #print('Connected to MySQL database')
        except Error as e:
            print(e)
    def close(self):
        self.conn.close()
        self.cursor.close()
    def openGripper(self):
        Servo.baseServo(s.gripperPin, s.openGripper)
        time.sleep(1)
    def Exit(self):
        if self.nextState == "Idle":
            print("******* Stay in Idle state ***********")
        else:
            print("******* Go to BeforeOnair state ***********")
        self.FSM.ToTransition(self.nextState)
#******************************************************************************
#                BeforeOnair
#******************************************************************************
class BeforeOnair(State):
    def __init__(self, FSM):
        super(BeforeOnair, self).__init__(FSM)
        self.height = 0
        self.meanHeight = 0
        self.windowLength = 10
        self.heightArray = [self.windowLength]
        self.nextState = "BeforeOnair"

        for i in range(self.windowLength-1):
            self.heightArray.append(0)

    def Enter(self):
        super(BeforeOnair, self).Enter()
    def Execute(self):
        page = 0
        self.ResetHeightArray()
        while(True):
            time.sleep(0.5)
            self.height = Ultrasonic.getReading2()
            if self.ValidateReading():
                page += 1
                for i in range(self.windowLength-1):
                    self.heightArray[i] = self.heightArray[i+1]
                self.heightArray[self.windowLength-1] = self.height
                print("Height: ", self.heightArray)
            if(page >= 10):
                page = 0
                self.meanHeight = np.mean(self.heightArray)
                self.meanHeight = round(self.meanHeight, 0)
                print("self.meanHeight: ", self.meanHeight)
                if self.meanHeight >= 25:
                    print("self.meanHeight: ", self.meanHeight)
                    break
        self.Exit()
    def ResetHeightArray(self):
        for i in range(self.windowLength-1):
            self.heightArray[i] = 0.0
        self.meanHeight = 0
    def ValidateReading(self):
        if self.height < 13 and self.height > 200:
            return False
        elif abs(self.heightArray[self.windowLength-1] - self.height) < 50:
            return True
        else:
            return False
    def Exit(self):
        if self.meanHeight >= 25:
            self.nextState = "OnAir"
            print("******* Next state is OnAir ***********")
        else:
            print("******* Stay in BeforeOnAir ***********")
            self.nextState = "BeforeOnair"
        self.FSM.ToTransition(self.nextState)
#******************************************************************************
#                OnAir
#******************************************************************************
class OnAir(State):
    def __init__(self, FSM):
        super(OnAir, self).__init__(FSM)
        self.count = 0
        self.windowLength = 10
        self.reading = None
        self.heightArray = [self.windowLength]
        self.nextState = "OnAir"
        self.mean = 0.0
        self.loop = True

        for i in range(self.windowLength-1):
            self.heightArray.append(0)

    def Enter(self):
        super(OnAir, self).Enter()
    def Execute(self):
        self.count = 0
        self.ResetHeightArray()
        while(self.loop):
            time.sleep(0.5)
            self.reading = Ultrasonic.getReading2()
            if self.ValidateReading():
                self.count += 1
                for i in range(self.windowLength - 1):
                    self.heightArray[i] = self.heightArray[i+1]
                self.heightArray[self.windowLength-1] = self.reading
            if self.count >= 10:
                self.count = 0
                self.mean = np.mean(self.heightArray)
                print("Mean height: ", self.mean)
                if self.mean >= 14 and self.mean <= 25:
                    self.nextState = "Landing"
                    self.loop = False
        self.Exit()
    def ResetHeightArray(self):
        for i in range(self.windowLength-1):
            self.heightArray[i] = 0.0
        self.mean = 0.0
        self.loop = True
    def ValidateReading(self):
        if self.reading >= 13 and self.reading <= 100:
            print("Height: ", self.heightArray)
            return True
        else:
            return False

    def Exit(self):
        print("******* Go to Landing state ***********")
        self.FSM.ToTransition(self.nextState)

#******************************************************************************
#                Landing
#******************************************************************************
class Landing(State):
    """docstring for Landing."""
    def __init__(self, FSM):
        super(Landing, self).__init__(FSM)
        self.reading = None
        self.windowLength = 10
        self.heightArray = [self.windowLength]
        self.duration = 3       #duration landing state stays in second
        self.count = 0
        self.mean = 0.0
        self.nextState = None

        for i in range(self.windowLength-1):
            self.heightArray.append(0)
    def Enter(self):
        super(Landing, self).Enter()
    def Execute(self):
        self.nextState = None
        self.ResetHeightArray()
        if self.VerifyStatus():
            if self.FSM.AfterLanding() == "Idle":
                self.nextState = "Navigation"
            else:
                self.nextState = "Dropping"
        else:
            self.nextState = "OnAir"
        self.Exit()
    def ResetHeightArray(self):
        for i in range(self.windowLength-1):
            self.heightArray[i] = 0
        self.mean = 0.0
        self.nextState = None
    def VerifyStatus(self):
        while(True):
            time.sleep(self.duration/self.windowLength)
            self.reading = Ultrasonic.getReading2()
            if self.ValidateReading():
                self.count += 1
                for i in range(self.windowLength -1):
                    self.heightArray[i] = self.heightArray[i+1]
                self.heightArray[self.windowLength-1] = self.reading
            if self.count >= self.windowLength:
                self.count = 0
                self.mean = np.mean(self.heightArray)
                print("mean height: ", self.mean)
                if self.mean >= 14 and self.mean <= 18:
                    return True
                else:
                    print("Go back to state Air")
                    return False

    def ValidateReading(self):
        if self.reading >= 13 and self.reading <= 21:
            print("heightArray: ", self.heightArray)
            return True
        else:
            return False

    def Exit(self):
        print("******* ",self.nextState, " *******")
        self.FSM.ToTransition(self.nextState)
#******************************************************************************
#                Navigation
#******************************************************************************
class Navigation(State):
    def __init__(self, FSM):
        super(Navigation, self).__init__(FSM)
        self.conn = None
        self.cursor = None
        self.loop = True
        self.X1 = None
        self.X2 = None
        self.X3 = None
        self.X4 = None
        self.rotation = 'clockwise'
        self.nextState = None
        self.stuckPeriod = None
        self.index = 20
        self.position = None

    def Enter(self):
        super(Navigation, self).Enter()
    def Execute(self):
        self.position = s.lbAngle
        self.updateX1()
        self.nextState = None
        time_1 = time.time()
        time_2 = 0.0
        while self.loop:
            self.fetchStatusReport()
            if self.X3 == 2:
                time_1 = time.time()
                self.loop = False
                self.nextState = "Grabbing"
                print("Next state is: ", self.nextState)
            elif self.X4 == 2:
                time_1 = time.time()
                self.loop = False
                print("statusreport: ",self.X4,self.X3,self.X2,self.X1)
                self.nextState = "FailedOrder"
                print("Next state is: ", self.nextState)
            elif self.X2 == 2:
                time_1 = time.time()
                self.rotateCamera()
                self.UpdateX2()
            else:
                time_2 = time.time()
                print("Inactive duration: ", int(time_2- time_1))
                if int(time_2 - time_1) > 60:
                    self.nextState = "FailedOrder"
                    break
                time.sleep(0.5)
        self.Exit()
    def updateX1(self):
        self.connect()
        updateQuery = ''' UPDATE statusreport SET cameraStatus = %s WHERE id = %s'''
        rowNo = 1
        updateCameraStatus = 1112
        print("camerStatus: ", updateCameraStatus)
        self.cursor.execute(updateQuery, (updateCameraStatus, rowNo,))
        self.conn.commit()
        self.close()
    def fetchStatusReport(self):
        self.connect()
        query = ''' SELECT cameraStatus FROM statusreport WHERE id = %s'''
        rowNo = 1
        self.cursor.execute(query,(rowNo,))
        queryReturn = self.cursor.fetchone()
        self.X1 = int(queryReturn[0]) % 10
        self.X2 = int((queryReturn[0] % 100)/10)
        self.X3 = int((queryReturn[0] % 1000)/ 100)
        self.X4 = int(queryReturn[0] / 1000)
        self.close()
    def rotateCamera(self):
        if self.position == s.lbAngle and self.rotation == "counterClockwise":
            self.updateX4()
        else:
            self.moveServo()
    def moveServo(self):
        if self.rotation == "clockwise":
            self.position += self.index
            if self.position <= s.ubAngle:
                print("current position: ", self.position)
            else:
                self.rotation = "counterClockwise"
                self.position = s.ubAngle
        else:
            if self.position >= s.lbAngle:
                self.position -= self.index
                print("current position: ", self.position)
        Servo.baseServo(s.baseservoPin, self.position)
    def UpdateX2(self):
        self.connect()
        updateQuery = ''' UPDATE statusreport SET cameraStatus = %s WHERE id = %s'''
        rowNo = 1
        self.X2 = 1
        updateValue = self.X4*1000 + self.X3*100 + self.X2*10 + self.X1
        self.cursor.execute(updateQuery, (updateValue, rowNo,))
        self.conn.commit()
        self.close()
    def updateX4(self):
        self.connect()
        query = ''' UPDATE statusreport SET  cameraStatus = %s WHERE id = %s '''
        self.X4 = 2
        self.X1 = 1
        rowNo = 1
        updateValue = self.X4*1000 + self.X3*100 + self.X2*10 + self.X1
        self.cursor.execute(query, (updateValue, rowNo,))
        self.conn.commit()
        self.close()
    def connect(self):
        try:
            self.conn = mysql.connector.connect(**config)
            if self.conn.is_connected():
                self.cursor = self.conn.cursor()
                #print('Connected to MySQL database')
        except Error as e:
            print(e)
    def close(self):
        self.conn.close()
        self.cursor.close()
    def Exit(self):
        print("******* ", self.nextState, " ***********")
        self.rotation = "clockwise"
        self.position = 0
        self.loop = True
        self.FSM.ToTransition(self.nextState)

#******************************************************************************
#                Grabbing
#******************************************************************************
class Grabbing(State):
    """docstring for Grabbing.State"""
    def __init__(self, FSM):
        super(Grabbing,self).__init__(FSM)
        self.readDistance = 0
        self.averageDistance = 0
        self.window = 10
        self.openGripper = -110
        self.closeGrip = -150
        self.isGripClosed = False
        self.distanceArray = [self.window]

        for i in range(self.window):
            self.distanceArray.append(0)

        self.isCloseEnough = False
    def Enter(self):
        super(Grabbing, self).Enter()
    def Execute(self):
        time.sleep(10)
        self.ResetDistanceArray()
        self.HowClose()
        if self.isCloseEnough:
            self.closeGripper()
        if self.isCloseEnough:
            self.FSM.ToTransition("BeforeOnair")
        else:
            self.FSM.ToTransition("FailedOrder")
    def ResetDistanceArray(self):
        for i in range(self.window):
            self.distanceArray[i] = 0
        self.averageDistance = 0
    def HowClose(self):
        page = 0
        while(True):
            time.sleep(1)
            self.readDistance = Ultrasonic.getReading1()
            if self.ValidateReading():
                page += 1
                for iteration in range(self.window):
                    self.distanceArray[iteration] = self.distanceArray[iteration+1]
                self.distanceArray[self.window] = self.readDistance
                print("distanceArray: ", self.distanceArray)
            if (page >= 10):
                page = 0
                self.averageDistance = np.mean(self.distanceArray)
                if self.averageDistance <= 35 and self.averageDistance >= 1:
                    self.isCloseEnough = True
                else:
                    self.isCloseEnough = False
                print("self.averageDistance: ", self.averageDistance)
                print("self.isCloseEnough: ", self.isCloseEnough)
                break
    def ValidateReading(self):
        if self.readDistance < 2 or self.readDistance > 200:
            return False
        else:
            return True
    def closeGripper(self):
        servoAngle = Mapping.getGripAngle(s.closeGripper)
        Servo.gripperServo(s.gripperPin,int(servoAngle))
        time.sleep(1)
        s.gripAngle = s.closeGripper
    def Exit(self):
        self.ResetGrabbingValues()
    def ResetGrabbingValues(self):
        self.readDistance = 0
        self.averageDistance = 0
        self.gripRange = 110
        self.gripClosed = False

        for i in range(self.window):
            self.distanceArray[i] = 0

        self.isCloseEnough = False

#******************************************************************************
#                FailedOrder
#******************************************************************************
class FailedOrder(State):
    """docstring for FailedOrder."""
    def __init__(self, FSM):
        super(FailedOrder, self).__init__(FSM)
        self.conn = None
        self.cursor = None

    def Enter(self):
        super(FailedOrder, self).Enter()

    def Execute(self):
        self.notifyStatus()
        self.resetStatusReport()
        self.Exit()

    def connect(self):
        try:
            self.conn = mysql.connector.connect(**config)
            if self.conn.is_connected():
                self.cursor = self.conn.cursor()
                print('Connected to MySQL database')
        except Error as e:
            print(e)
    def notifyStatus(self):
        self.connect()
        query = ''' UPDATE task SET status = %s WHERE id = %s'''
        status = "Mission Failed"
        self.cursor.execute(query,(status, s.id))
        self.conn.commit()
        self.close()
    def resetStatusReport(self):
        self.connect()
        query = ''' UPDATE statusreport SET cameraStatus = %s WHERE id = %s'''
        status = 1111
        self.cursor.execute(query,(status, 1,))
        self.conn.commit()
        self.close()
    def close(self):
        self.cursor.close()
        self.conn.close()

    def Exit(self):
        self.FSM.ToTransition("Idle")
#******************************************************************************
#                Dropping
#******************************************************************************
class Dropping(State):
    def __init__(self, FSM):
        super(Dropping, self).__init__(FSM)
        self.conn = None
        self.cursor = None

    def Enter(self):
        super(Dropping, self).Enter()
    def Execute(self):
        self.openGripper()
        self.updateDB()
        self.resetDroppingValues()
        self.Exit()
    def connect(self):
        try:
            self.conn = mysql.connector.connect(**config)
            if self.conn.is_connected():
                self.cursor = self.conn.cursor()
                print('Connected to MySQL database')
        except Error as e:
            print(e)
    def openGripper(self):
        Servo.baseServo(s.gripperPin,s.openGripper)
        time.sleep(1)
    def updateDB(self):
        self.connect()
        query = ''' UPDATE task SET status = %s WHERE id = %s'''
        status = "Mission Success"
        self.cursor.execute(query,(status, s.id))
        self.conn.commit()
        self.close()
    def resetDroppingValues(self):
        self.isGripOpen = False
        s.gripAngle = 110
    def close(self):
        self.conn.close()
        self.cursor.close()

    def Exit(self):
        print("******* Go to Idle state ***********")
        self.FSM.ToTransition("Idle")
Char = type("Char",(object,),{})

class StateMachine(Char):
    """docstring for StateMachine.Char  def __init__(self, arg):
        super(StateMachine,Char._
        _init__()
        self.arg = arg"""
    def __init__(self):
        self.id = None
        self.openGripper = 140
        self.closeGripper = 200
        self.gripAngle = 0
        self.servoAngle = 0
        self.gripperPin = 4
        self.baseservoPin = 0
        self.thirdServoPin = 12
        self.heightAngle = 200
        self.ubAngle = 420
        self.lbAngle = 180
        self.image  = None
        self.FSM = FSM(self)
        ## state
        self.FSM.AddState("Startup", Startup(self.FSM))
        self.FSM.AddState("Idle", Idle(self.FSM))
        self.FSM.AddState("BeforeOnair", BeforeOnair(self.FSM))
        self.FSM.AddState("OnAir", OnAir(self.FSM))
        self.FSM.AddState("Landing", Landing(self.FSM))
        self.FSM.AddState("Navigation", Navigation(self.FSM))
        self.FSM.AddState("Grabbing", Grabbing(self.FSM))
        self.FSM.AddState("FailedOrder", FailedOrder(self.FSM))
        self.FSM.AddState("Dropping", Dropping(self.FSM))
        # Transitions
        self.FSM.AddTransition("Startup", Transitions("Startup"))
        self.FSM.AddTransition("Idle", Transitions("Idle"))
        self.FSM.AddTransition("BeforeOnair", Transitions("BeforeOnair"))
        self.FSM.AddTransition("OnAir", Transitions("OnAir"))
        self.FSM.AddTransition("Landing", Transitions("Landing"))
        self.FSM.AddTransition("Navigation", Transitions("Navigation"))
        self.FSM.AddTransition("Grabbing", Transitions("Grabbing"))
        self.FSM.AddTransition("FailedOrder", Transitions("FailedOrder"))
        self.FSM.AddTransition("Dropping", Transitions("Dropping"))

        self.FSM.SetState("Startup")

    def Execute(self):
        self.FSM.Execute()

if __name__ == '__main__':
    s = StateMachine()
    while(True):
        s.Execute()
