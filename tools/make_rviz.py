#!/usr/bin/env python3
"""Build rviz/painting_cell.rviz from the UR package's known-good config (same schema), keeping only the displays we need."""
import copy, sys, yaml
src = '/opt/ros/jazzy/share/ur_moveit_config/config/moveit.rviz'
dst = sys.argv[1]
cfg = yaml.safe_load(open(src))
vm = cfg['Visualization Manager']
grid = [d for d in vm['Displays'] if d['Class'] == 'rviz_default_plugins/Grid'][0]
grid['Plane Cell Count'] = 14
grid['Color'] = '120; 130; 140'
scene = {
    'Alpha': 1, 'Class': 'moveit_rviz_plugin/PlanningScene', 'Enabled': True, 'Move Group Namespace': '',
    'Name': 'PlanningScene', 'Planning Scene Topic': '/monitored_planning_scene', 'Robot Description': 'robot_description',
    'Scene Geometry': {'Scene Alpha': 1, 'Show Scene Geometry': True, 'Voxel Coloring': 'Z-Axis', 'Voxel Rendering': 'Occupied Voxels'},
    'Scene Robot': {'Attached Body Color': '150; 50; 150', 'Links': {'All Links Enabled': True, 'Expand Joint Details': False,
                    'Expand Link Details': False, 'Expand Tree': False, 'Link Tree Style': 'Links in Alphabetic Order'},
                    'Robot Alpha': 1, 'Show Robot Collision': False, 'Show Robot Visual': True},
    'Value': True,
}
markers = {
    'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': True, 'Name': 'PaintMarkers', 'Namespaces': {},
    'Topic': {'Depth': 5, 'Durability Policy': 'Volatile', 'History Policy': 'Keep Last',
              'Reliability Policy': 'Reliable', 'Value': '/paint_markers'},
    'Value': True,
}
vm['Displays'] = [grid, scene, markers]
vm['Global Options']['Fixed Frame'] = 'world'
vm['Global Options']['Background Color'] = '38; 42; 52'
view = vm['Views']['Current']
view.update({'Class': 'rviz_default_plugins/Orbit', 'Name': 'Current View', 'Distance': 10.5, 'Pitch': 0.5, 'Yaw': 5.55,
             'Focal Point': {'X': 0.0, 'Y': -0.4, 'Z': 0.9}, 'Target Frame': '<Fixed Frame>',
             'Invert Z Axis': False, 'Near Clip Distance': 0.01, 'Focal Shape Fixed Size': True, 'Focal Shape Size': 0.05,
             'Value': 'Orbit (rviz)'})
for k in [k for k in view if k.startswith('Enable Stereo')]:
    pass
vm['Tools'] = [t for t in vm.get('Tools', [])]
cfg['Window Geometry'] = {'Displays': {'collapsed': True}, 'Height': 900, 'Hide Left Dock': True, 'Hide Right Dock': True,
                          'Selection': {'collapsed': False}, 'Tool Properties': {'collapsed': False},
                          'Views': {'collapsed': True}, 'Width': 1600, 'X': 0, 'Y': 0}
cfg['Panels'] = [p for p in cfg['Panels'] if p['Class'] in ('rviz_common/Displays', 'rviz_common/Views')]
yaml.safe_dump(cfg, open(dst, 'w'), sort_keys=False)
print('wrote', dst, '; displays:', [d['Name'] for d in vm['Displays']])
# fast variant: draw the low-poly collision meshes instead of the detailed visual meshes (about 25x lighter)
fast = copy.deepcopy(cfg)
ps = [d for d in fast['Visualization Manager']['Displays'] if d['Class'] == 'moveit_rviz_plugin/PlanningScene'][0]
ps['Scene Robot']['Show Robot Visual'] = False
ps['Scene Robot']['Show Robot Collision'] = True
fdst = dst.replace('.rviz', '_fast.rviz')
yaml.safe_dump(fast, open(fdst, 'w'), sort_keys=False)
print('wrote', fdst)
