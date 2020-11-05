#  /---Amaan Rahman--\
# /----3D POSE MODEL--\
#/-----VISUALIZER------\

import bpy
import json
import mathutils 
import math 
import time



class ModalTimerOperator(bpy.types.Operator):
    #Operator which runs its self from a timer
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"

    _timer = None
    
    f = open('C:\\Users\\Amaan\\Documents\\GitHub\\openpose\\out_new.json')
    scn = bpy.context.scene
    arm = bpy.data.objects.get("Arm_NEW")
    bpy.context.view_layer.objects.active = arm
    arm.select_set(True)
    bpy.ops.object.mode_set(mode='POSE', toggle=False)
    bones = arm.pose.bones
    
    frame = 0
    trigg = True
    old_frame = 0
    delta_fr = 0
    points = {}
    
    def lerp(self, q0, q1, h): #linear interpoate from q0 to q1 by a factor of h
        h_c = 1 - h
        qa = mathutils.Quaternion((q0.w * h_c, q0.x * h_c, q0.y * h_c, q0.z * h_c))
        qb = mathutils.Quaternion((q1.w * h, q1.x * h, q1.y * h, q1.z * h))
        q = qa + qb
        return q
    
    def new_pose(self, name, h, t):
        b = self.bones[name]
        u = b.vector.normalized() #vector of current limb pose from preset skeleton
        p0 = mathutils.Vector(self.points[str(h)])
        p1 = mathutils.Vector(self.points[str(t)])
        v = p1 - p0 #vector of new limb pose from point cloud
        #v.z *= -1
        v.normalized()
        if "right" in name:
            right_rot = mathutils.Matrix.Rotation(math.radians(-90), 3, 'Z')
            u.rotate(right_rot)
            v.rotate(right_rot)
        if "left" in name:
            left_rot = mathutils.Matrix.Rotation(math.radians(90), 3, 'Z')
            u.rotate(left_rot)
            v.rotate(left_rot)
        theta = u.angle(v)
        print("\n---" + name + "----\n")    
        print(theta*(180/3.1415))
        print("-------\n")
        axis = u.cross(v).normalized() #axis of rotation 
        quat = mathutils.Quaternion(axis, theta) #axis-angle -> quaternion
        init_q = b.rotation_quaternion.copy()
        quat = self.lerp(init_q.normalized(), quat.normalized(), 0.5)
        b.rotation_mode = 'QUATERNION'
        b.rotation_quaternion = quat.normalized()
        b.keyframe_insert(data_path="rotation_quaternion", index=-1)   
        
    def reset(self):
        for b in self.bones: 
            b.rotation_quaternion = (1, 0, 0, 0) #identity quaternion
            b.keyframe_insert(data_path="rotation_quaternion", index=-1)
         
    def update(self):
        #self.new_pose("body", 8, 1)
        
        self.new_pose("neck", 1, 0)
        
        self.new_pose("leftShoulder", 1, 5)
        self.new_pose("rightShoulder", 1, 2)
        self.new_pose("leftArm", 5, 6)
        self.new_pose("rightArm", 2, 3)
        self.new_pose("leftForearm", 6, 7)
        self.new_pose("rightForearm", 3, 4)
    
    
    def modal(self, context, event):
        if event.type in {'RIGHTMOUSE', 'ESC'}:
            self.cancel(context)
            return {'CANCELLED'}

        if event.type == 'TIMER':
            print("=================\n")
            #self.scn.frame_set(frame)
            #time.sleep(3)
            prev_frame = self.frame
            try:
                self.f.seek(0,0)
                if(self.f.closed):
                    raise StopIteration
                data = json.load(self.f)
                if self.trigg:
                    self.old_frame = data['frame_num']
                    self.trigg = False
        
                new_frame = data['frame_num']
                self.frame = new_frame - self.old_frame
                self.delta_fr = self.frame - prev_frame
                del data['frame_num']
                for key, coordinates in data.items():
                    self.points[str(key)] = coordinates['xyz']            
                if self.delta_fr > 0: 
                    self.update()
                #self.scn.frame_set(frame*3)
            except ValueError:
                print("\nCORRUPTION!\n")

        return {'PASS_THROUGH'}
    
    def execute(self, context):
        time.sleep(3)
        self.reset()
        wm = context.window_manager
        self._timer = wm.event_timer_add(0.16, window=context.window)
        wm.modal_handler_add(self)
        return {'RUNNING_MODAL'}

    def cancel(self, context):
        wm = context.window_manager
        wm.event_timer_remove(self._timer)


def register():
    bpy.utils.register_class(ModalTimerOperator)


def unregister():
    bpy.utils.unregister_class(ModalTimerOperator)


if __name__ == "__main__":
    register()
    # test call
    bpy.ops.wm.modal_timer_operator()
    
    