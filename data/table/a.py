#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import pybullet as p
import math

#from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
from sawyerEnv import sawyerEnv
import time

def main():

  # replace the values of following 3 variables to recall the last configurations
  # the values are the 3 output of the last run of this program
  handInitial = [0.22152291659747883, 0.9817759152539315, 0.16999407449044338, 0.2215143810144337, 0.9823135574132803, 0.1699989274565554, 0.5739483012567052, 0.5902239466515086, 0.1699767141257003, 0.5739640367999089, 0.5899904373506896, 0.17000733012736757, 0.9361331354216027, 0.26548528122516185, 0.17000335607201394, 0.93630346854007, 0.2656232807802453, 0.17001488487220198, 0.906821961784849, 0.25019764084658785, 0.17007600796776692, 0.906618391328536, 0.25115462875714, 0.16999999912072705, 1.5698416936818693, 0.3400976317456705, 0.3400749247584485]
  orientation =  [-1.1904981136322021, 1.4715882102635245, -1.984163761138916]
  palmPosition = [0.7, 0.17, 0.25]


  environment = sawyerEnv(renders=True, isDiscrete=False, maxSteps=10000000, palmPosition = palmPosition, orientation = orientation)
  readings = [0] * 35
  motorsIds = []
 
  dv = 0.01
  motorsIds.append(environment._p.addUserDebugParameter("posX", -dv, dv, 0))
  motorsIds.append(environment._p.addUserDebugParameter("posY", -dv, dv, 0))
  motorsIds.append(environment._p.addUserDebugParameter("posZ", -dv, dv, 0))

  # orientation of the palm 
  motorsIds.append(environment._p.addUserDebugParameter("orienX", -math.pi, math.pi, 0))
  motorsIds.append(environment._p.addUserDebugParameter("orienY", -math.pi, math.pi, 0))
  motorsIds.append(environment._p.addUserDebugParameter("orienZ", -math.pi, math.pi, 0))

  #low [0.17 - 1.57], mid [0.34, 1.5]
  motorsIds.append(environment._p.addUserDebugParameter("thumbLow", 0.85, 1.57, handInitial[24]))
  motorsIds.append(environment._p.addUserDebugParameter("thumbMid", 0.34, 1.5, handInitial[25]))
  #[0.17 - 1.57]
  motorsIds.append(environment._p.addUserDebugParameter("indexLow", 0.17, 1.57, handInitial[18]))
  motorsIds.append(environment._p.addUserDebugParameter("indexMid", 0.17, 1.57, handInitial[19]))
  motorsIds.append(environment._p.addUserDebugParameter("middleLow", 0.17, 1.57, handInitial[12]))
  motorsIds.append(environment._p.addUserDebugParameter("middleMid", 0.17, 1.57, handInitial[13]))
  motorsIds.append(environment._p.addUserDebugParameter("ringLow", 0.17, 1.57, handInitial[6]))
  motorsIds.append(environment._p.addUserDebugParameter("ringMid", 0.17, 1.57, handInitial[7]))
  motorsIds.append(environment._p.addUserDebugParameter("pinkyLow", 0.17, 1.57, handInitial[0]))
  motorsIds.append(environment._p.addUserDebugParameter("pinkyMid", 0.17, 1.57, handInitial[1]))
  done = False
  action = []
  while (not done):

    action = []
    for motorId in motorsIds:
      action.append(environment._p.readUserDebugParameter(motorId))

    #print (action)
    #break
    #state, reward, done, info = environment.step2(action)
    state, reward, info = environment.step2(action)
    #environment.step2(action)
    #done = True
    #obs = environment.getExtendedObservation()
    handReading = environment.handReading()
    orientation1 = environment.o()
    palmPosition1 = environment.p()
    qKey = ord('q')
    keys = p.getKeyboardEvents()
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
      break;

  print("==============================================================================") 
  print("handReading = ", handReading) 
  print("==============================================================================") 
  print("orientation = ", orientation1) 
  print("==============================================================================") 
  print("palmPosition = ", palmPosition1) 
  print("==============================================================================") 



if __name__ == "__main__":
  main()


