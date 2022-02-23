
from __future__ import print_function
from re import L, M
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from numpy import argsort                    
from can.interface import Bus
import time
from FingerRobot import FingerRobot
from visionSys import visionSys
  
def mainFunc():

  vision = visionSys()
  robot = FingerRobot(DoControl=True)
  
  robot.x_offset = vision.x_offset
  robot.y_offset = vision.y_offset
  robot.z_offset = vision.z_offset
  
  # 0: Tele-Operation Ctrl 
  # 1: Vision-Based Ctrl
  # 2: Vision-Based FK - PD Ctrl

  desPose = np.zeros(3)
  #curPose = np.zeros(3)
  
  robot.inputMode = 2
      
  lastCommand = time.time()
  
  l1 = 0
  l2 = 0
  deltaL = 0
  
  # robot.visualizeFK()
  # robot.visualizeFKnew()
  robot.visualizePD()
  
  while (True):
        
        deltaTime = time.time() - start_time
        
        key = vision.update(vis=True,freq=20)
        targetPose = vision.targetPose
        endTip_PosError = robot.endTip_PosError
        
        robot.updateKeyboardCtrl(key)
        
        # getting marker pose
        #markerPose = vision.getMarkerPose(1)
        
        # robot.updateMarkerBasedCtrl(targetPose)
        robot.updateMarkerBasedCtrl2(targetPose)
        
        if robot.inputMode == 0: # teleoperation
          robot.robotCtrlTeleOp(robot.lengthKey,robot.phiKey,robot.thetaKey)
        elif robot.inputMode == 1:  # vision based control
          if (time.time()-lastCommand>0.01):  
            lastCommand = time.time()          
            robot.robotIK(robot.lengthMarker,robot.phiMarker,robot.thetaMarker)
        elif robot.inputMode == 2:  # vision based PD-control
          if (time.time()-lastCommand>0.01):  
            lastCommand = time.time()          
            # robot.robotCtrlPD(robot.lengthMarker,robot.phiMarker,robot.thetaMarker)
            desPose[0] = 5
            desPose[1] = 0
            desPose[2] = 15
            curPose = robot.robotFK(robot.M1,robot.M2,robot.M3,robot.M4)[0]
            robot.robotCtrlPD(desPose-curPose)
            
            robot.robotFKnew(l1,l2,deltaL)
            
        robot.printFunc()
        #print('Time = ', f"{deltaTime:3.3f}")
        #print('targetPose = ', targetPose)

        
if __name__ == '__main__':
  start_time = time.time()    
  mainFunc()