import bpy
import csv
from mathutils import *
from math import *

# source: https://blender.stackexchange.com/a/28474
# though it doesn't seem to help much :(
class QuaternionStabilizer:
    def __init__(self):
        self.old=None

    def stabilize(self, q):
        if self.old is None:
            rval = q
        else:
            d1 = (self.old-q).magnitude
            d2 = (self.old+q).magnitude
            if (d1<d2):
                rval = q
            else:
                rval = -q
        self.old = rval
        return rval

# this script writes the position and orientation of the camera
# at every frame into a CSV file named 'camera_trajectory.csv'

T_OpenCV_Blender = Matrix.Rotation(-radians(90), 4, 'Z')

def writeIntrinsics():
    cam = bpy.context.scene.camera
    r = bpy.context.scene.render
    with open(bpy.path.abspath('//camera_intrinsics.csv'), 'w', newline='') as csvfile:
        intr_file = csv.writer(csvfile, delimiter=',')
        intr_file.writerow([ \
            'focal length', \
            'focal length mm', \
            'image width', \
            'image height', \
            'sensor width mm', \
            'sensor height mm', \
            'clip start', \
            'clip end', \
            'color depth', \
            'depth depth'])
            
        W = r.resolution_x * r.resolution_percentage / 100
        H = r.resolution_y * r.resolution_percentage / 100
        
        focal = cam.data.lens * W / cam.data.sensor_width
            
        intr_file.writerow([ \
            focal, \
            cam.data.lens, \
            W, \
            H, \
            cam.data.sensor_width, \
            cam.data.sensor_height, \
            cam.data.clip_start, \
            cam.data.clip_end, \
            255, \
            255])

def writeTrajectory():
    
    scene = bpy.context.scene
    fps = scene.render.fps
    cam = bpy.context.scene.camera
    
    orig_frame = scene.frame_current
    
    qs = QuaternionStabilizer()
    
    # overwrite existing CSV file and write CSV header
    with open(bpy.path.abspath('//camera_trajectory.csv'), 'w', newline='') as csvfile:
        posfile = csv.writer(csvfile, delimiter=',')
        
        #posfile.writerow(['# timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        
        # esim expects spaces between header fields -.-
        csvfile.write(', '.join(['# timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']) + '\n')
        
        for frame in range(scene.frame_start, scene.frame_end+1):
            scene.frame_set(frame)
            
            T_WC = T_OpenCV_Blender * cam.matrix_world
            
            t = T_WC.translation
            q = qs.stabilize(T_WC.to_quaternion())
            
            #print(frame, "->", t, q)
            #print(frame/fps, "s ->", q.angle, q.axis)
            print(frame, frame/fps, "s ->", q)
            
            posfile.writerow([
                frame / fps * 1000 * 1000 * 1000, # in ns
                t.x, t.y, t.z,
                q.x, q.y, q.z, q.w])
                
    scene.frame_set(orig_frame)


writeIntrinsics()
writeTrajectory()
