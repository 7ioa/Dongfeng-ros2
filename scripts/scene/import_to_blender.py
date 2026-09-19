"""Run in Blender or: blender --background --python scripts/scene/import_to_blender.py

Creates an editable .blend from the same OBJ that Gazebo and the preview use.
"""
from pathlib import Path
import bpy
root=Path(__file__).resolve().parents[2]
export_dir=root/'exports/scene'
scene=bpy.data.scenes.new('Dongfeng sandbox')
bpy.context.window.scene=scene
bpy.context.scene.unit_settings.system='METRIC'
bpy.context.scene.unit_settings.scale_length=1.0
bpy.context.scene.unit_settings.length_unit='METERS'
bpy.ops.wm.obj_import(filepath=str(export_dir/'dongfeng_sandbox.obj'),forward_axis='Y',up_axis='Z')
for obj in bpy.context.selected_objects:
    obj['source']='Measured dimensions and photo approximation; see config/scene/scene_config.json'
bpy.ops.wm.save_as_mainfile(filepath=str(export_dir/'dongfeng_sandbox.blend'))
print('Saved',export_dir/'dongfeng_sandbox.blend')
