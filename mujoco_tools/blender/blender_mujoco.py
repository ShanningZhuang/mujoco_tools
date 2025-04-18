import bpy
import numpy as np

class BlenderRecorder:
    def __init__(self, output_data_freq=50, record_types=None):
        """Initialize recorder for Blender data export
        
        Args:
            output_data_freq: Output data frequency
            record_types: Types of data to record 
            (geom, tendon) can be used to record geom and tendon data
        """
        self.model_info = None
        self.output_data_freq = output_data_freq
        self.record_types = record_types or ["geom", "tendon"]
        self.objects = {}  # Store created Blender objects
        
    def record_frame(self, model, data):
        """Record frame data for Blender import
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
        """
        # Initialize model info and create objects on first frame
        if self.model_info is None:
            self.model_info = self._extract_model_info(model)
            self.generate_blender_objects()
        
        # Update object positions and orientations
        if 'geom' in self.record_types:
            self.update_geom_positions(model, data)
        if 'tendon' in self.record_types:
            self.record_tendon(model, data)
    
    def initialize(self, output_path, output_prefix):
        """Initialize Blender recorder"""
        pass
    
    def generate_blender_objects(self):
        """Generate Blender objects from model info"""
        from mathutils import Matrix
        
        if not self.model_info:
            print("No model info available")
            return
        
        # Create objects for each geom
        for geom_info in self.model_info['geoms']:
            geom_id = geom_info['id']
            geom_name = geom_info['name']
            geom_type = geom_info['type_name']
            size = geom_info['size']
            
            # Create mesh based on geom type
            mesh_obj = None
            
            if geom_type == "sphere":
                radius = size[0]
                bpy.ops.mesh.primitive_uv_sphere_add(
                    segments=32, 
                    ring_count=16, 
                    radius=radius,
                    location=(0, 0, 0)
                )
                mesh_obj = bpy.context.active_object
                
            elif geom_type == "capsule":
                radius = size[0]
                half_length = size[1]
                length = half_length * 2
                
                mesh_obj = self._create_capsule_mesh(
                    radius=radius, 
                    height=length, 
                    location=(0, 0, 0),
                    name=geom_name
                )
            
            elif geom_type == "cylinder":
                radius = size[0]
                half_length = size[1]
                
                bpy.ops.mesh.primitive_cylinder_add(
                    radius=radius,
                    depth=half_length * 2,
                    vertices=32,
                    location=(0, 0, 0)
                )
                mesh_obj = bpy.context.active_object
                
            elif geom_type == "box":
                bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, 0))
                mesh_obj = bpy.context.active_object
                mesh_obj.scale = (size[0], size[1], size[2])
                
            elif geom_type == "ellipsoid":
                # Create sphere and scale it to ellipsoid dimensions
                bpy.ops.mesh.primitive_uv_sphere_add(
                    segments=32, 
                    ring_count=16, 
                    radius=1.0,
                    location=(0, 0, 0)
                )
                mesh_obj = bpy.context.active_object
                mesh_obj.scale = (size[0], size[1], size[2])
            
            # Set material if rgba color is available
            if mesh_obj and geom_info['rgba'] is not None:
                self._set_material(mesh_obj, geom_info['rgba'])
            
            # Store the object for later animation
            if mesh_obj:
                mesh_obj.name = geom_name
                self.objects[geom_id] = mesh_obj

    def update_geom_positions(self, model, data):
        """Update geom positions and orientations
        
        Args:
            model: MuJoCo model
            data: MuJoCo data
        """
        # Get current frame
        frame = bpy.context.scene.frame_current
        
        # Update each geom object
        for i in range(model.ngeom):
            if i in self.objects:
                obj = self.objects[i]
                
                # Get position and orientation
                pos = data.geom_xpos[i]
                rot_mat = data.geom_xmat[i].reshape(3, 3)
                
                # Apply transform
                transform = self._transform_matrix_from_pos_mat(pos, rot_mat)
                obj.matrix_world = transform
                
                # Insert keyframes
                obj.keyframe_insert("location", frame=frame)
                obj.rotation_mode = "QUATERNION"
                obj.rotation_quaternion = transform.to_quaternion()
                obj.keyframe_insert("rotation_quaternion", frame=frame)

    def _set_material(self, obj, rgba):
        """Set material with given RGBA color
        
        Args:
            obj: Blender object
            rgba: RGBA color values [r, g, b, a]
        """
        import bpy
        
        mat_name = f"material_{obj.name}"
        mat = bpy.data.materials.get(mat_name) or bpy.data.materials.new(mat_name)
        mat.use_nodes = True
        
        # Get the Principled BSDF node
        principled = mat.node_tree.nodes.get('Principled BSDF')
        if principled:
            # Set color
            principled.inputs['Base Color'].default_value = rgba
            # Set alpha if using transparency
            if rgba[3] < 1.0:
                principled.inputs['Alpha'].default_value = rgba[3]
                mat.blend_method = 'BLEND'
        
        # Assign material to object
        if obj.data.materials:
            obj.data.materials[0] = mat
        else:
            obj.data.materials.append(mat)

    def _create_capsule_mesh(self, radius, height, location=(0, 0, 0), name="capsule"):
        """Create a capsule mesh using cylinder and uv_sphere primitives"""
        # Create the cylindrical part
        bpy.ops.mesh.primitive_cylinder_add(
            radius=radius,
            depth=height,
            vertices=32,
            location=location
        )
        cylinder = bpy.context.active_object
        cylinder.name = f"{name}_cylinder"
        
        # Create the top hemisphere
        top_loc = (location[0], location[1], location[2] + height/2)
        bpy.ops.mesh.primitive_uv_sphere_add(
            radius=radius,
            segments=32,
            ring_count=16,
            location=top_loc
        )
        top_sphere = bpy.context.active_object
        top_sphere.name = f"{name}_top"
        
        # Create the bottom hemisphere
        bottom_loc = (location[0], location[1], location[2] - height/2)
        bpy.ops.mesh.primitive_uv_sphere_add(
            radius=radius,
            segments=32,
            ring_count=16,
            location=bottom_loc
        )
        bottom_sphere = bpy.context.active_object
        bottom_sphere.name = f"{name}_bottom"
        
        # Join objects
        bpy.ops.object.select_all(action='DESELECT')
        top_sphere.select_set(True)
        bottom_sphere.select_set(True)
        cylinder.select_set(True)
        bpy.context.view_layer.objects.active = cylinder
        bpy.ops.object.join()
        
        # Clean up the mesh
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.select_all(action='SELECT')
        bpy.ops.mesh.remove_doubles(threshold=0.0001)
        bpy.ops.object.mode_set(mode='OBJECT')
        
        cylinder.name = name
        return cylinder

    def _transform_matrix_from_pos_mat(self, pos, rot_mat):
        """Convert MuJoCo position and rotation matrix to Blender transform matrix"""
        from mathutils import Matrix, Vector
        
        # Create rotation matrix (3x3 -> 4x4)
        R = Matrix((
            (rot_mat[0][0], rot_mat[0][1], rot_mat[0][2], 0),
            (rot_mat[1][0], rot_mat[1][1], rot_mat[1][2], 0),
            (rot_mat[2][0], rot_mat[2][1], rot_mat[2][2], 0),
            (0, 0, 0, 1)
        ))
        
        # Create translation matrix
        T = Matrix.Translation(pos)
        
        # Combine translation and rotation
        return T @ R

    def record_tendon(self, model, data):
        """Record tendon data for Blender import"""
        pass
    
    def _extract_model_info(self, model):
        """Extract relevant information from the MuJoCo model for Blender
        
        Args:
            model: MuJoCo model
            
        Returns:
            dict: Model information
        """
        # Extract model structure information
        model_info = {
            'geoms': [],
            'tendons': [],
            'meshes': [],
        }
        
        # Geoms and their properties with detailed type information
        for i in range(model.ngeom):
            geom_type = int(model.geom_type[i])
            geom_type_name = self._get_geom_type_name(geom_type)
            if geom_type_name not in ['sphere', 'cylinder', 'ellipsoid', 'box', 'mesh']:
                print(f"Unsupported geom type: {geom_type_name}")
            geom = {
                'id': i,
                'name': model.geom(i).name,
                'body_id': model.geom_bodyid[i],
                'type': geom_type,
                'type_name': geom_type_name,
                'size': model.geom_size[i].copy(),
                'rgba': model.geom_rgba[i].copy() if model.geom_rgba is not None else None,
                'mesh_id': model.geom_dataid[i] if geom_type == 7 else -1,  # 7 is mjGEOM_MESH
                }
            model_info['geoms'].append(geom)
        
        # Extract mesh data if present
        if model.nmesh > 0:
            for i in range(model.nmesh):
                mesh = {
                    'id': i,
                    'name': model.mesh(i).name,
                    'vertadr': model.mesh_vertadr[i],
                    'vertnum': model.mesh_vertnum[i],
                    'faceadr': model.mesh_faceadr[i],
                    'facenum': model.mesh_facenum[i],
                }
                
                # Extract vertices and faces for this mesh
                verts = []
                for v in range(mesh['vertnum']):
                    idx = mesh['vertadr'] + v
                    verts.append(model.mesh_vert[idx].copy())
                
                faces = []
                for f in range(mesh['facenum']):
                    idx = mesh['faceadr'] + f * 3
                    faces.append([
                        model.mesh_face[idx],
                        model.mesh_face[idx + 1],
                        model.mesh_face[idx + 2]
                    ])
                
                mesh['vertices'] = np.array(verts)
                mesh['faces'] = np.array(faces)
                model_info['meshes'].append(mesh)
        
        return model_info
    
    def _get_geom_type_name(self, geom_type):
        """Get the name of the geom type from its numeric value
        
        Args:
            geom_type: Numeric geom type value
            
        Returns:
            str: Name of the geom type
        """
        # Based on mjtGeom enum in MuJoCo
        geom_types = {
            0: 'plane',
            1: 'hfield',
            2: 'sphere',
            3: 'capsule',
            4: 'ellipsoid',
            5: 'cylinder',
            6: 'box',
            7: 'mesh',
            8: 'sdf'
        }
        return geom_types.get(geom_type, 'unknown')


#!/usr/bin/env python3
"""
Command-line interface for importing MuJoCo models and animation data into Blender
"""

import os
import numpy as np
import mujoco
from pathlib import Path
from mujoco_tools.player import MujocoPlayer

def main():
    # Set default values directly
    model_path = '/home/zsn/research/blender_project/blender_mujoco_chengtian/assets/ant.xml'
    input_data = 'qpos /home/zsn/research/blender_project/blender_mujoco_chengtian/ant_qpos.txt'
    mode = 'kinematics'
    input_data_freq = 50
    output_path = None
    output_prefix = None
    
    # Initialize player
    player = MujocoPlayer(
        model_path=model_path,
        mode=mode,
        input_data_freq=input_data_freq,
        output_path=output_path,
        output_prefix=output_prefix,
        input_data=input_data
    )
    
    # Initialize the blender recorder
    blender_recorder = BlenderRecorder(
        output_data_freq=input_data_freq,
        record_types=["geom"]
    )
    
    
    
    print(f"Blender import completed. Objects created from model: {model_path}")

main()