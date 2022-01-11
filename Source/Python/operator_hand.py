import bpy
from bpy.props import * # allows to create properties in blender (meta data).
from time import *
import math
import serial
import numpy as np
import sys
sys.path.append("C:\\Users\\Jon_A\\Desktop\\COMP-477-Project\\Project\\Source\\Python")
from imuDataManipulator import ImuDataManipulator

class ActivateHand(bpy.types.Operator): 
    bl_idname = "object.hand_operator" # defining function name
    bl_label = "Activate Hand" # name of addon
    bl_options = {'REGISTER', 'UNDO'} # makes it capatible with blender's undo system.

    def execute(self, context):
        quaternionData = ImuDataManipulator('com3', 115200, 'POSE', 'QUATERNION', "HandsRig", "hand", ["pinky", "ring", "middle", "index", "thumb"])
        quaternionData.readAndTargetQuaternionData()

        return {'Program finished executing...'}
    
# allows for the addon to be searchable in blender's menu.
def menu_func(self, context):
        self.layout.operator(ActivateHand.bl_idname)
        
def register():
    bpy.utils.register_class(ActivateHand)
    bpy.types.VIEW3D_MT_object.append(menu_func)


def unregister():
    bpy.utils.unregister_class(ActivateHand)


if __name__ == "__main__":
    register()

    # test call
    # bpy.ops.object.hand_operator()
