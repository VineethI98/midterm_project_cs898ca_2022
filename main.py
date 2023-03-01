import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

##################################################a####### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot


tableId = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3],
                     p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)


######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
#objectId = p.loadURDF("random_urdfs/001/001.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

xpos =1.05
ypos = 0.1
ang = 3.14 * 0.7
orn = p.getQuaternionFromEuler([0, 0, ang])

object_path ="random_urdfs/067/067.urdf"
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])


######################################################### Load tray Here!!!!#############################################################################

tray_x =  0.923681586762877
tray_y = 0.3



trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])



###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]

#hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
# handReading = [0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 
# 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 
# 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])




##################################################################### Input Value Here############################################################

handInitial =[0.22758715119487366, 0.9757903273209749, 0.24161750372834237, 0.22710371244591762, 0.976509480725665, 0.18225672671637788, 0.5776262116231128, 0.5856521577290977, 0.21449415241902586, 0.573393395852755, 0.5911037708666577, 0.1700040673710855, 0.7352621390539142, 0.26659978467036205, 0.1100028233065044, 0.9354455405240106, 0.26708167250766834, 0.16999999999999985, 0.9059983743861906, 0.25177778770414255, 0.1699999881540996, 0.9058464398347609, 0.25171720259452235, 0.16999998371672256, 1.5697207347951239, 0.3404695043046988, 0.34043465669512]
grasp_orientation = [1.2000694370269775, 3.125212983289037, 2.9182329177856445]
grasp_palmPosition = [0.95, 0.0, -0.12]
handClose = [1.12480102678082, 1.1174286363945584, 0.4501254160114216, 1.070137938832957, 1.0197104455390975, 0.7745033523113237, 1.0683163790871566, 0.5708775271550952, 0.5757548570033821, 0.5700983141257734, 0.4719512211355926, 0.21249483619302775, 0.4656718548668054, 0.3678661934795882, 0.1223057790631204, 0.5770245276016757, 0.5198695989225133, 0.17475162717424994, 0.4647899340831268, 0.4543353698483426, 0.16977190417135682, 0.261570376689427, 0.3501097917585298, 0.10432314164096327, 0.3654125849327554, 0.0210722111081236, 1.008368311270562]
pu_palmPosition = [0.95, 0.0, -0.12]
pu_orientation = [1.1900694370269775, 3.141592653589793, 2.513482093811035]
final_palmPosition = [0.78, 0.13, 0.20557952895760536]
final_orientation = [1.1900694370269775, 3.191592653589793, 1.9843716621398926]
handOpen = [0.23000367121159443, 0.17928183325095823, 0.18252231419640877, 0.2314778761045323, 0.17076121327487434, 0.2698718194983618, 0.17466333560458921, 0.17773815470804644, 0.16996967943951483, 0.17026359035183278, 0.17010979230323622, 0.17000894276182174, 0.1861425095569482, 0.2682374606886315, 0.17000767235156536, 0.18479440522153157, 0.27044691292944883, 0.16999999999999996, 0.17001433550052092, 0.18055525701742214, 0.16999863524073175, 0.17009507059822154, 0.17047493043534048, 0.17002637445955215, 1.5694965920109887, 0.3407138835756754, 0.3405554420643023]
##################################################################################################################################################################################################
initial_palmPosition = [0.85, -0.05, 0.1]
initial_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

initial_palmPosition = [initial_palmPosition[0]-0.1,initial_palmPosition[1]-0.05,initial_palmPosition[2]]
##################################################################################################################################################################################################

# write the code for step 9-12


''' Step 1: move robot to the initial position '''

# Parameters: 
	# arm: initial_palmPosition, initial_orientation
	# hand: handInitial


''' Step 2: move robot to the grasping position '''

# Parameters: 
	# arm: grasp_orientation, grasp_palmPosition


''' Step 3: grasp object '''

# Parameters: 
	# hand: handClose

''' Step 4: pick up  the object '''

# Parameters: 
	# arm: pu_palmPosition, pu_orientation

''' Step 4: move object to the tray '''

# Parameters: 
	# arm: final_palmPosition, final_orientation


''' Step 5: put the object in the tray '''

# Parameters: 
	# hand: handOpen

p.setGravity(0,0,-10)
k = 0
while 1:
    k = k + 1

    # move palm to target postion
    i = 0
    while 1:
        i += 1
        # p.stepSimulation()
        currentP = palmP([initial_palmPosition[0], initial_palmPosition[1], initial_palmPosition[2]],
                         p.getQuaternionFromEuler([initial_orientation[0], initial_orientation[1], initial_orientation[2]]))
        time.sleep(0.09)
        p.stepSimulation()

        if (i == 300):
            print('reached initial position')
            break
    i = 0
    while 1:
        i += 1
        p.stepSimulation()
        currentp = palmP([grasp_palmPosition[0], grasp_palmPosition[1], grasp_palmPosition[2]],
                         p.getQuaternionFromEuler([grasp_orientation[0], grasp_orientation[1], grasp_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 150):
            print('reached target')
            break

    i = 0
    while 1:
        i += 1
        p.stepSimulation()
        startpos = handClose
        final = handClose

        thumb(startpos[0], final[1])
        if (i == 200):
            print('grasping the object')
            indexF(final[2], final[3])
            midF(final[4], final[5])
            ringF(final[6], final[7])
            pinkyF(final[8], final[9])

            time.sleep(0.03)
        if (i == 300):
            print('grasped the object')
            break



    i = 0
    while 1:
        i += 1
        p.stepSimulation()
        time.sleep(0.03)
        currentp = palmP([pu_palmPosition[0], pu_palmPosition[1], pu_palmPosition[2]],
                         p.getQuaternionFromEuler([pu_orientation[0], pu_orientation[1], pu_orientation[2]]))
        time.sleep(0.03)

        if (i == 100):
            print('picking up the object')
            break


    i = 0
    while 1:
        i += 1
        p.stepSimulation()
        time.sleep(0.03)
        currentp = palmP([final_palmPosition[0], final_palmPosition[1], final_palmPosition[2] + 0.02],
                         p.getQuaternionFromEuler([final_orientation[0], final_orientation[1], final_orientation[2]]))
        time.sleep(0.03)

        if (i == 100):
            print('top of the box')
            break


    i = 0
    while 1:
           i += 1
           p.stepSimulation()
           time.sleep(0.03)
           open_hand = handOpen
           indexF(open_hand[2], open_hand[3])
           midF(open_hand[4], open_hand[5])
           ringF(open_hand[6], open_hand[7])
           pinkyF(open_hand[8], open_hand[9])



           time.sleep(0.03)

           if (i == 300):
               print('open hand')
               break
    



p.disconnect()
print("disconnected")







