import mysql.connector
from mysql.connector import connection, errorcode, Error

#from Modules import distanceSensor
from Camera import VideoFeedback

#from picamera.array import PiRGBArray
#from picamera import PiCamera
#import cv2
import numpy as np
import os
import sys, gc
from time import sleep

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
#*****************************************************************************
#               Idle
#*****************************************************************************
class Idle(State):
    def __init__(self, FSM):
        super(Idle, self).__init__(FSM)
        self.conn = None
        self.cursor = None
        self.X1 = 1
        self.X2 = 1
        self.X3 = 1
        self.X4 = 1
        self.loop = True
    def Enter(self):
        super(Idle, self).Enter()
    def Execute(self):
        self.loop = True
        while self.loop:
            self.checkDb()
            print("Camera Idle State:")
            sleep(1)
        self.Exit()

    def checkDb(self):
        self.connect()
        query = ''' SELECT cameraStatus FROM statusreport WHERE id = %s '''
        tableRow = 1
        self.cursor.execute(query, (tableRow,))
        queryReturn = self.cursor.fetchone()
        self.X1 = int(queryReturn[0]) % 10
        self.X2 = int((queryReturn[0] % 100)/10)
        self.X3 = int((queryReturn[0] % 1000)/ 100)
        self.X4 = int(queryReturn[0] / 1000)

        if self.X1 == 2:
            self.loop = False
        self.close()
    def connect(self):
        try:
            self.conn = mysql.connector.connect(**config)
            if self.conn.is_connected():
                self.cursor = self.conn.cursor()
        except Error as e:
            print(e)
    def close(self):
        self.conn.close()
        self.cursor.close()
    def __del__(self):
        pass
    def Exit(self):
        print("******* Go to Navigation state ***********")
        self.FSM.ToTransition("Navigation")

#******************************************************************************
#                Navigation
#******************************************************************************
class Navigation(State):
    def __init__(self, FSM):
        super(Navigation, self).__init__(FSM)
        self.threshold = 170
        self.conn = None
        self.cursor = None
        #self.vision = VideoFeedback(self.threshold)
        self.id = None
        self.Name = None
        self.image = None
        self.savePath = None

    def Enter(self):
        super(Navigation, self).Enter()
    def Execute(self):
        collected = gc.collect()
        self.fetchTaskId()
        self.DownloadPicture()
        self.vision = VideoFeedback(self.threshold)
        self.initialiseCamera()
        collected = gc.collect()
        self.Exit()
    def fetchTaskId(self):
        self.connect()
        query = '''SELECT taskId FROM statusreport WHERE id = %s '''
        rowNo = 1
        self.cursor.execute(query,(rowNo, ))
        queryReturn = self.cursor.fetchone()
        self.id = queryReturn[0]
        self.close()
    def DownloadPicture(self):
        self.connect()
        query = ''' SELECT name, picture FROM task WHERE id = %s '''
        self.cursor.execute(query, (self.id, ))
        queryReturn = self.cursor.fetchone()

        self.name = queryReturn[0]
        self.image = queryReturn[1]
        self.savePicture()
        self.close()
    def savePicture(self):
        print("savePicture")
        BASE_DIR = os.path.dirname(__file__)
        PIC_DIR = os.path.join(BASE_DIR, 'Picture')
        save_path = os.path.join(PIC_DIR,self.name)
        open(save_path,'wb').write(self.image)
        self.savePath = save_path
    def initialiseCamera(self):
        self.vision.Recording(self.savePath)
    def updateStatusReport(self):
        self.connect()
        updateQuery = ''' UPDATE statusreport SET cameraStatus = %s WHERE id = %s'''
        rowNo = 1
        updateCameraStatus = 1111
        self.cursor.execute(updateQuery, (updateCameraStatus, rowNo,))
        self.conn.commit()
        self.close()
    def resetDb(self):
        self.connect()
        query = ''' UPDATE statusreport SET cameraStatus = %s WHERE id = %s '''
        tableRow = 1
        resetValue = 1111
        self.cursor.execute(query, (resetValue,tableRow,))
        self.conn.commit()
        self.close()
        self.loop = True
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
        #self.updateStatusReport()
        print("******* ", " Idle State ", " ***********")
        #self.resetDb()
        self.vision.__del__()
        self.FSM.ToTransition("Idle")
    def __del__(self):
        pass

Char = type("Char",(object,),{})

class StateMachine(Char):
    """docstring for StateMachine.Char  def __init__(self, arg):
        super(StateMachine,Char._
        _init__()
        self.arg = arg"""
    def __init__(self):
        self.FSM = FSM(self)
        ## state
        self.FSM.AddState("Idle", Idle(self.FSM))
        self.FSM.AddState("Navigation", Navigation(self.FSM))
        # Transitions
        self.FSM.AddTransition("Idle", Transitions("Idle"))
        self.FSM.AddTransition("Navigation", Transitions("Navigation"))

        self.FSM.SetState("Idle")

    def Execute(self):
        self.FSM.Execute()

if __name__ == '__main__':
    s = StateMachine()
    while(True):
        s.Execute()
