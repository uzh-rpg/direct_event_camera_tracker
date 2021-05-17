track_filename = "//path/to/output/track.csv"
gt_filename =    "//path/to/output/ground_truth.csv"

skip = 10 # how often to draw a camera
scale = 0.3 # size of camera frustrums

#################################################################

import csv
import bpy
from mathutils import Quaternion, Vector, Matrix
import math
import bmesh

ctx = bpy.context
obj = bpy.ops.object


T_OpenCV_Blender = Matrix.Rotation(-math.radians(90), 3, 'Z')
T_Blender_OpenCV = Matrix.Rotation( math.radians(90), 3, 'Z')


def import_path(filename, traj_name):
    bpy.ops.group.create(name=traj_name)
    all_pos = []

    cur_skip = skip
    with open(filename, newline='') as csvfile:
        for row in csv.DictReader(csvfile):
            
            # load and convert pose
            ####################################################
            
            pos = Vector([
                float(row['Position X']),
                float(row['Position Y']),
                float(row['Position Z'])
            ])
            pos.rotate(T_Blender_OpenCV)
            
            
            rot = Quaternion([
                    float(row['Rotation W']),
                    float(row['Rotation X']),
                    float(row['Rotation Y']),
                    float(row['Rotation Z']),
                ])
            rot.rotate(T_Blender_OpenCV)
            
            
            # insert path
            ####################################################
            
            all_pos.append(pos)
            
            # skip poses if configured
            ####################################################
            
            if cur_skip == 0:
                cur_skip = skip
            else:
                cur_skip -= 1
                continue
            
            # create new camera
            ####################################################
            
            obj.camera_add(view_align = False, location=pos)
            c = ctx.object
            
            # appearantly, Blender wants the orientation of the
            # back of the camera O.o
            c.scale = (-scale, -scale, -scale)
            
            c.rotation_mode = 'QUATERNION'
            c.rotation_quaternion = rot
            
            if 'step' in row:
                c.name = 'Traj Camera ' + str(row['step'])
                
            # add to group
            ####################################################
            
            c.select = True
            obj.group_link(group=traj_name)
            c.select = False
            
            
            
            
    # draw a line with all the poses
    ####################################################

    mesh = bpy.data.meshes.new(traj_name)
    line = bpy.data.objects.new(traj_name, mesh)

    ctx.scene.objects.link(line) # insert into scene

    line_data = bmesh.new()

    # create vertices
    verts = []
    for v in all_pos:
        verts.append(line_data.verts.new(v))

    # and edges
    for v1, v2 in zip(verts[:-1], verts[1:]):
        line_data.edges.new( (v1, v2) )

    line_data.to_mesh(line.data)
    line_data.free()
    
    # color vertices
    #colors = line.data.vertex_colors.new()
    #print(colors)
    #for v in line.data.vertices:
        #print("got v:", v)


#################################################################
    
import_path(track_filename, "imported_camera_trajectory")

if gt_filename:
    import_path(gt_filename, "ground_truth")
