import serial
import bpy
import mathutils
from time import *


class ImuDataManipulator:
    def __init__(self, port = 'com3', baud = 115200, scene_mode = '', rot_mode = '', armatureName = '', boneName = '', boneFingerNames = []):
        self.port = port
        self.baud = baud
        self.scene_mode = scene_mode
        self.rot_mode = rot_mode
        self.armatureName = armatureName
        self.boneName = boneName
        self.boneFingerNames = boneFingerNames
        self.setModes()
        self.setBone()
        
    def readSerial(self):
        imuData = serial.Serial(self.port, self.baud)
        sleep(1) # give time to read serial port
        return imuData
    
    def setModes(self):
        # enter pose mode
        bpy.ops.object.mode_set(mode = self.scene_mode)
        # set rotation mode
        bpy.ops.pose.rotation_mode_set(type = self.rot_mode)
        
    def setBone(self):
        # accesses the armature for the hand
        armature = bpy.data.objects[self.armatureName]
        # gets the quaternion values for the middle joint bone called hand
        targetBone = armature.pose.bones[self.boneName].rotation_quaternion
        pinkyBone = armature.pose.bones[self.boneFingerNames[0]].rotation_quaternion
        ringBone = armature.pose.bones[self.boneFingerNames[1]].rotation_quaternion
        middleFingerBone = armature.pose.bones[self.boneFingerNames[2]].rotation_quaternion
        indexFingerBone = armature.pose.bones[self.boneFingerNames[3]].rotation_quaternion
        thumbFingerBone = armature.pose.bones[self.boneFingerNames[4]].rotation_quaternion
        
        # armature = bpy.data.objects[self.armatureName]
        # armatureWorld = armature.matrix_world
        # targetBone = armatureWorld.to_quaternion()

        # self.targetBone = aramatureWorldQuaternion
        self.targetBone = targetBone
        self.pinkyBone = pinkyBone
        self.ringBone = ringBone
        self.middleFingerBone = middleFingerBone
        self.indexFingerBone = indexFingerBone
        self.thumbFingerBone = thumbFingerBone
        
        
    def retargetQuaternionData(self, q0, q1, q2, q3):
        self.targetBone[0] = q0
        self.targetBone[1] = -q2
        self.targetBone[2] = q1
        self.targetBone[3] = q3
        
    def retargetFingerBones(self, fingerAngles):
        quatDownScale = -100 # to get it suitable range for quaternion rotation.
        pinkyFingerBend = fingerAngles[0] / quatDownScale # get the angle in a corresponding quaternion representation.
        ringFingerBend = fingerAngles[1] / quatDownScale
        middleFingerBend = fingerAngles[2] / quatDownScale
        indexFingerBone = fingerAngles[3] / quatDownScale
        thumbFingerBone = fingerAngles[4] / quatDownScale
        self.pinkyBone[1] = pinkyFingerBend
        self.ringBone[1] = ringFingerBend
        self.middleFingerBone[1] = middleFingerBend
        self.indexFingerBone[1] = indexFingerBone
        self.thumbFingerBone[1] = thumbFingerBone
        
    def readAndTargetQuaternionData(self):
        imuData = self.readSerial()
        
        while True: # continuously read data from IMU
            while (imuData.inWaiting() == 0): # do nothing if no data being read
                pass
            
            packet = imuData.readline()
            packet = str(packet,'utf-8')
            packet = packet.split(',')
            
            self.retargetQuaternionData(float(packet[5]), float(packet[6]), float(packet[7]), float(packet[8]))
            self.retargetFingerBones([float(packet[0]), float(packet[1]), float(packet[2]), float(packet[3]), float(packet[4])])
            
            # allows for live viewing of the updating scene, while reading the data
            bpy.ops.wm.redraw_timer(type = 'DRAW_WIN_SWAP', iterations = 1)