bl_info = {
    "name": "OpenVR Streamer_v2",
    "author": "Steve Miller",
    "version": (1, 2),
    "blender": (2, 80, 0),
    "location": "View3D > UI > OpenVR",
    "description": "Streams OpenVR tracking data into blender to bind to objects",
    "warning": "",
    "wiki_url": "",
    "category": "Animation",
}

import bpy, bpy_extras
from bpy.app.handlers import persistent
#import openvr
import re
import math
from mathutils import Matrix, Euler, Vector
import sys
import subprocess
import os



##Install Openvr module##
try:
    import openvr
except:
    python_exe = os.path.join(sys.prefix, 'bin', 'python.exe')
    #upgrade pip
    subprocess.call([python_exe, "-m", "ensurepip"])
    subprocess.call([python_exe, "-m", "pip", "install", "--upgrade", "pip"])
    #install required packages
    subprocess.call([python_exe, "-m", "pip", "install", "openvr"])
    import openvr

#TODOs:

#make modal when animation not playing?
#make preview empties when not bound
#operator to auto pair/offset trackers to nearest bones in bone set

#consider reduced polling frequency on get_trackers()
#collapsable subpanels for button mappings?
#mask tracking to just loc/rot/scale
#convenience function to auto generate an object for a given tracker?

#handle tracking for a given tracker
def handle_tracking(poses, tracker, context):
    if not tracker.target: return

    target = bpy.data.objects[tracker.target]

    if target.type == 'ARMATURE' and tracker.use_subtarget:
        if not tracker.subtarget: return
        target = target.pose.bones[tracker.subtarget]
        
    settings = context.scene.OVRSettings    
    
    
    mat = poses[tracker.index].mDeviceToAbsoluteTracking
    mat = Matrix([list(mat[0]),list(mat[1]),list(mat[2]),[0,0,0,1]])
    #matrix 
    m_location_offset = Matrix.Translation(target.OVRBind.location)* settings.location_scale
    m_rotation_offset = Euler((math.radians(target.OVRBind.rotation[0]),math.radians(target.OVRBind.rotation[1]),math.radians(target.OVRBind.rotation[2]))).to_matrix().to_4x4() 
    m_scale_offset = Matrix.Scale(target.OVRBind.scale[0],4,(1,0,0)) @ Matrix.Scale(target.OVRBind.scale[1],4,(0,1,0)) @ Matrix.Scale(target.OVRBind.scale[2],4,(0,0,1))
    mat_world = bpy_extras.io_utils.axis_conversion('Z','Y','Y','Z').to_4x4() @ mat @ m_location_offset @ m_rotation_offset @ m_scale_offset
    mat_world.translation = (mat_world.translation*settings.location_scale) + Vector(settings.floor_pos)
    old_rot = target.rotation_quaternion.copy() if target.rotation_mode == 'QUATERNION' else target.rotation_euler.copy()
    
    additional_rotation = Euler((math.radians(settings.floor_ros[0]), math.radians(settings.floor_ros[1]), math.radians(settings.floor_ros[2]))).to_matrix().to_4x4()
    mat_world = additional_rotation @ mat_world
    
    if target.bl_rna.name == 'Pose Bone':
        mat_rot = target.id_data.convert_space(pose_bone=target, matrix=mat_world, from_space='WORLD', to_space='LOCAL')
        target.matrix = mat_world
    else:
        mat_rot = mat_world
        target.matrix_world = mat_world
        
    if target.rotation_mode == 'QUATERNION':
        rot = mat_rot.to_quaternion()
        rot.make_compatible(old_rot)
        target.rotation_quaternion = rot
    else:
        rot = mat_rot.to_euler(target.rotation_euler.order, old_rot)
        target.rotation_euler = rot
        
    if bpy.context.scene.tool_settings.use_keyframe_insert_auto:
        target.keyframe_insert('location')
        if target.rotation_mode == 'QUATERNION':
            target.keyframe_insert('rotation_quaternion')
        else:
            target.keyframe_insert('rotation_euler')
        target.keyframe_insert('scale')

#handle controller inputs for a given tracker (assumes its a valid controller)        
def handle_controller(tracker):
    def set_property(property, val, range_old, range_new):
        try:
            #item = prop = bpy.data.objects[target]
            item = prop = bpy.context.scene
        except:
            return
        #prop_path = property.split('.')
        prop_path = re.split('\.(?![^\[]*\])',property)
        for p in prop_path:
            parent_item = item
            item = prop
            p_split = re.split('\[|\]',p)
            prop = getattr(item, p_split[0], None)
            if prop == None: return
        
            if len(p_split) > 1: #there's an index on the property to handle
                if re.findall('\'|\"', p_split[1]): #case its a named index
                    prop = prop[re.sub('\'|\"','',p_split[1])]
                    
                elif p_split[1].isnumeric: #case its a integer index
                    prop = prop[int(p_split[1])]
                    
                else: #otherwise treat as invalid
                    return
            
        #abandon when target property is not a float
        if type(prop) is not float: return
    
        remap = (((val-range_old[0]) * (range_new[1]-range_new[0])) / (range_old[1]-range_old[0])) + range_new[0]
        setattr(item, prop_path[-1],remap)
        
        if bpy.context.scene.tool_settings.use_keyframe_insert_auto:
            if type(item) is Vector:
                parent_item.keyframe_insert(prop_path[-2])
            else:
                item.keyframe_insert(prop_path[-1])
    
    #get controller state
    result,pControllerState = openvr.VRSystem().getControllerState(tracker.index)
    #handle trigger
    set_property(tracker.trigger_property, pControllerState.rAxis[1].x, [0,1], [tracker.trigger_min,tracker.trigger_max])
    #handle trackpad
    set_property(tracker.trackpad_x_property, pControllerState.rAxis[0].x, [-1,1], [tracker.trackpad_x_min, tracker.trackpad_x_max])
    set_property(tracker.trackpad_y_property, pControllerState.rAxis[0].y, [-1,1], [tracker.trackpad_y_min, tracker.trackpad_y_max])
    #handle grip
    set_property(tracker.grip_property, pControllerState.ulButtonPressed >> 2 & 1, [0,1], [tracker.grip_min, tracker.grip_max])
    #handle menu button
    set_property(tracker.menu_property, pControllerState.ulButtonPressed >> 1 & 1, [0,1], [tracker.menu_min, tracker.menu_max])


#on frame update, update tracking as well as controller inputs
@persistent
def puppet(scene):
    settings = scene.OVRSettings
    if not settings.streaming: return
    
    get_trackers(bpy.context)
    poses = []
    poses, _ = openvr.VRCompositor().waitGetPoses(poses, None)
    for tracker in settings.trackers:
        if not tracker.connected: continue
        handle_tracking(poses, tracker, bpy.context)
        if tracker.type == 'Controller': handle_controller(tracker)
    return


#updates scene tracker list
def get_trackers(context):
    settings = context.scene.OVRSettings
    vrsys = openvr.VRSystem()
    types = {
        str(openvr.TrackedDeviceClass_HMD): "HMD",
        str(openvr.TrackedDeviceClass_Controller): "Controller", 
        str(openvr.TrackedDeviceClass_GenericTracker): "GenericTracker"
    }
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        if str(vrsys.getTrackedDeviceClass(i)) in types:
            #add any new trackers
            type = types[str(vrsys.getTrackedDeviceClass(i))]
            #name = type+'_%03d' %i
            name = vrsys.getStringTrackedDeviceProperty(i, openvr.Prop_SerialNumber_String)
            if name not in settings.trackers:
                t = settings.trackers.add()
                t.friendly_name = name
            else:
                t = settings.trackers[name]
            t.name = name
            t.type = type
            t.index = i
            t.connected = bool(vrsys.isTrackedDeviceConnected(i))

def add_empty_object(object_name, display_size):
    bpy.ops.object.empty_add(type='CUBE', location=(0,0,0))
    empty_object = bpy.context.object
    empty_object.name = object_name
    empty_object.empty_display_size = display_size
    
    

#toggles streaming, by initializing and shutting down openvr session    
class ToggleStreaming(bpy.types.Operator):
    """Toggle OpenVR Streaming"""
    bl_idname = "id.toggle_streaming"
    bl_label = "Toggle OpenVR Streaming"

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        settings = context.scene.OVRSettings
        if settings.streaming:
            openvr.shutdown()
            settings.streaming = False
            #settings.trackers.clear()
        else:
            openvr.init(openvr.VRApplication_Scene)
            get_trackers(context)
            settings.streaming = True
        return {'FINISHED'}

#Create Empty and bind to target
class CreateEmpty(bpy.types.Operator):
    """Toggle OpenVR Streaming"""
    bl_idname = "id.add_empty"
    bl_label = "Create empty target"

    @classmethod
    def poll(cls, context):
        return True

    def execute(self, context):
        settings = context.scene.OVRSettings
        #add_empty_object("A", 0.1)
        for tracker in settings.trackers:

            existing_target = bpy.data.objects.get(tracker.name)
            if existing_target:
                tracker.target = tracker.name
            else:
                bpy.ops.object.empty_add(type='CUBE', location=(0,0,0))
                empty_object = bpy.context.object
                empty_object.name = tracker.name
                
                empty_object.empty_display_size = 0.1
                tracker.target = empty_object.name
            
                # Add a parent empty object
                existing_object = bpy.data.objects.get("Root")
                
                if existing_object:
                    parent_empty_object = existing_object
                else:
                    bpy.ops.object.empty_add(type='CUBE', location=(0, 0, 0))
                    parent_empty_object = bpy.context.object
                    parent_empty_object.name = "Root"
                    parent_empty_object.empty_display_size = 0.1
                
                # Make the newly added empty object a child of the parent empty object
                empty_object.parent = parent_empty_object
            
        # if settings.streaming:
            # openvr.shutdown()
            # settings.streaming = False
            # #settings.trackers.clear()
        # else:
            # openvr.init(openvr.VRApplication_Scene)
            # get_trackers(context)
            # settings.streaming = True
        return {'FINISHED'}


#user interface panel    
class OVRStreamPanel(bpy.types.Panel):
    """Stream from OpenVR"""
    bl_category = "OpenVR"
    bl_label = "OpenVR Streaming"
    bl_idname = "OPENVR_PT_main"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"

    def draw(self, context):
        layout = self.layout
        
        settings = context.scene.OVRSettings
        
        label = 'Stop Streaming' if settings.streaming else 'Start Streaming'
        layout.operator('id.toggle_streaming', text=label)
        
        layout.label(text='Origin :')
        layout.prop(settings, 'location_scale')
        layout.prop(settings, 'floor_pos')
        layout.prop(settings, 'floor_ros')        
        
        if settings.streaming:
            if not settings.trackers: return
            layout.template_list("TrackerList", "", settings, "trackers", settings, "active_tracker",rows=len(settings.trackers))
            t = settings.trackers[settings.active_tracker]
            if not t.connected:
                layout.label(text='Not Connected')
                return
            layout.operator('id.add_empty', text="Create targets")    
            layout.prop_search(t,'target',context.scene,'objects')
            if t.target:
                item = bpy.data.objects[t.target]
                if item.type == 'ARMATURE':
                    row = layout.row()
                    row.prop(t,'use_subtarget')
                    if t.use_subtarget:
                        row.prop_search(t,'subtarget',item.pose,'bones',text='')
                        if not t.subtarget: return
                        item = item.pose.bones[t.subtarget]                
                layout.label(text='Offset:')
                row = layout.row(align=True)
                col = row.column(align=True)
                col.prop(item.OVRBind, 'location')
                col = row.column()
                col.prop(item.OVRBind, 'rotation')
                col = row.column()
                col.prop(item.OVRBind, 'scale')
            if t.type == 'Controller':
                def draw_input(label, prop, min, max):
                    if label: layout.label(text=label + ' Scene Property:')
                    layout.prop(t,prop,text='')
                    row = layout.row(align=True)
                    row.prop(t,min,text='Min')
                    row.prop(t,max,text='Max')
                    
                draw_input('Trigger', 'trigger_property', 'trigger_min','trigger_max')
                draw_input('Trackpad X','trackpad_x_property','trackpad_x_min','trackpad_x_max')
                draw_input('Trackpad Y','trackpad_y_property','trackpad_y_min','trackpad_y_max')
                draw_input('Grip Button', 'grip_property', 'grip_min', 'grip_max')
                draw_input('Menu Button', 'menu_property', 'menu_min', 'menu_max')
                

#listview for trackers           
class TrackerList(bpy.types.UIList):
    def draw_item(self,context,layout,data,item,icon,active_data,active_propname):
        trackers = data
        t = item
        if self.layout_type in {'DEFAULT', 'COMPACT'}:
            layout.prop(t, 'friendly_name',text="",emboss=False,icon_value=icon)
        elif self.layout_type in {'GRID'}:
            layout.label(text="",translate=False,icon_value=icon)            
        
#class containing relevant data about a given tracker
class OVRTrackerItem(bpy.types.PropertyGroup):
    name: bpy.props.StringProperty(name='Tracker Name')
    friendly_name: bpy.props.StringProperty(name='Friendly Name')
    index: bpy.props.IntProperty(name='Index')
    type: bpy.props.StringProperty(name='Tracker Type')
    connected: bpy.props.BoolProperty(name='Connected', default = False)
    
    target: bpy.props.StringProperty(name='Target')
    use_subtarget: bpy.props.BoolProperty(name="Use Bone", default=True)
    subtarget: bpy.props.StringProperty(name='Subtarget')
    
    #trigger_target: bpy.props.StringProperty(name='Trigger Target')
    trigger_property: bpy.props.StringProperty(name='Trigger Property')
    trigger_min: bpy.props.FloatProperty(name='Trigger Range Minimum', default=0)
    trigger_max: bpy.props.FloatProperty(name='Trigger Range Maximum', default=1)
    
    #trackpad_x_target: bpy.props.StringProperty(name='Target X',default='')
    trackpad_x_property: bpy.props.StringProperty(default='')
    trackpad_x_min: bpy.props.FloatProperty(default=-1)
    trackpad_x_max: bpy.props.FloatProperty(default=1)
    #trackpad_y_target: bpy.props.StringProperty(name='Target Y',default='')
    trackpad_y_property: bpy.props.StringProperty(default='')
    trackpad_y_min: bpy.props.FloatProperty(default=-1)
    trackpad_y_max: bpy.props.FloatProperty(default=1)
    
    grip_property: bpy.props.StringProperty(default='')
    grip_min: bpy.props.FloatProperty(default=0)
    grip_max: bpy.props.FloatProperty(default=1)
    
    menu_property: bpy.props.StringProperty(default='')
    menu_min: bpy.props.FloatProperty(default=0)
    menu_max: bpy.props.FloatProperty(default=1)
    
#class that serves as container for all of this addon's data
class OVRSettings(bpy.types.PropertyGroup):
    streaming: bpy.props.BoolProperty(name='Streaming', default=False)
    trackers: bpy.props.CollectionProperty(type=OVRTrackerItem)
    active_tracker: bpy.props.IntProperty(name='Active Tracker')
    location_scale: bpy.props.FloatProperty(name='Scale', default=(1.0))
    floor_pos: bpy.props.FloatVectorProperty(name='Location', default=(0.0,0.0,0.0))
    floor_ros: bpy.props.FloatVectorProperty(name='Rotation', default=(0.0,0.0,0.0))

#class that stores the offset data for any object bound to a tracker
class OVRBind(bpy.types.PropertyGroup):
    location: bpy.props.FloatVectorProperty(name="Loc")
    rotation: bpy.props.FloatVectorProperty(name="Rot")
    scale: bpy.props.FloatVectorProperty(name="Scale", default=(1,1,1))
        
#bpy.app.handlers.frame_change_post.clear()
def register():
    bpy.utils.register_class(OVRTrackerItem)
    bpy.utils.register_class(OVRSettings)
    bpy.utils.register_class(OVRBind)
    
    bpy.types.Scene.OVRSettings = bpy.props.PointerProperty(type=OVRSettings)
    bpy.types.Object.OVRBind = bpy.props.PointerProperty(type=OVRBind)
    bpy.types.PoseBone.OVRBind = bpy.props.PointerProperty(type=OVRBind)
    
    bpy.utils.register_class(TrackerList)
    bpy.utils.register_class(ToggleStreaming)
    bpy.utils.register_class(CreateEmpty)
    bpy.utils.register_class(OVRStreamPanel)
    
    #bpy.app.handlers.frame_change_post.clear()
    bpy.app.handlers.frame_change_post.append(puppet)
    
def unregister():
    bpy.utils.unregister_class(OVRTrackerItem)
    bpy.utils.unregister_class(OVRSettings)
    bpy.utils.unregister_class(OVRBind)
    
    bpy.utils.unregister_class(TrackerList)
    bpy.utils.unregister_class(ToggleStreaming)
    bpy.utils.unregister_class(CreateEmpty)
    bpy.utils.unregister_class(OVRStreamPanel)
    
    bpy.app.handlers.frame_change_post.remove(puppet)
    

if __name__ == "__main__":
    register()