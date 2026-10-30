"""Generate a self-contained Gazebo Classic world without downloaded models."""
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from simulation import LANDMARKS

world = '''<?xml version="1.0"?>
<sdf version="1.6"><world name="slam_lab">
<gravity>0 0 -9.81</gravity>
<physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
<scene><ambient>0.6 0.6 0.6 1</ambient><background>0.85 0.9 0.95 1</background></scene>
<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><direction>-0.5 0.1 -1</direction><cast_shadows>true</cast_shadows></light>
<plugin name="world_state" filename="libgazebo_ros_state.so"><ros><namespace>/slam_demo</namespace></ros><update_rate>20</update_rate></plugin>
<model name="floor"><static>true</static><link name="floor_link"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision><visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry><material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.7 0.7 0.7 1</diffuse></material></visual></link></model>
<model name="slam_robot"><pose>0 0 0.11 0 0 0</pose>
<link name="base_link"><inertial><mass>3</mass><inertia><ixx>0.06</ixx><iyy>0.08</iyy><izz>0.1</izz></inertia></inertial><collision name="body_collision"><pose>0 0 0.09 0 0 0</pose><geometry><box><size>0.45 0.30 0.12</size></box></geometry></collision><visual name="body_visual"><pose>0 0 0.09 0 0 0</pose><geometry><box><size>0.45 0.30 0.12</size></box></geometry><material><ambient>0.1 0.3 0.9 1</ambient><diffuse>0.1 0.3 0.9 1</diffuse></material></visual>
<visual name="heading"><pose>0.22 0 0.17 0 0 0</pose><geometry><box><size>0.10 0.06 0.03</size></box></geometry><material><ambient>1 0.2 0 1</ambient><diffuse>1 0.2 0 1</diffuse></material></visual></link>
'''
for side, y in [('left', .2), ('right', -.2)]:
    world += f'''<link name="{side}_wheel"><pose>0 {y} 0 1.57079632679 0 0</pose><inertial><mass>0.3</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial><collision name="c"><geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision><visual name="v"><geometry><cylinder><radius>0.1</radius><length>0.05</length></cylinder></geometry><material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse></material></visual></link>
<joint name="{side}_joint" type="revolute"><parent>base_link</parent><child>{side}_wheel</child><axis><xyz>0 1 0</xyz><use_parent_model_frame>true</use_parent_model_frame><limit><lower>-1e16</lower><upper>1e16</upper></limit><dynamics><damping>0.001</damping></dynamics></axis></joint>'''
for name, x in [('front', .17), ('back', -.17)]:
    world += f'''<link name="{name}_caster"><pose>{x} 0 -0.06 0 0 0</pose><inertial><mass>0.05</mass><inertia><ixx>0.0001</ixx><iyy>0.0001</iyy><izz>0.0001</izz></inertia></inertial><collision name="c"><geometry><sphere><radius>0.04</radius></sphere></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision><visual name="v"><geometry><sphere><radius>0.04</radius></sphere></geometry></visual></link><joint name="{name}_fixed" type="fixed"><parent>base_link</parent><child>{name}_caster</child></joint>'''
world += '''<plugin name="drive" filename="libgazebo_ros_diff_drive.so"><ros><namespace>/slam_demo</namespace><remapping>cmd_vel:=cmd_vel</remapping><remapping>odom:=wheel_odom</remapping></ros><update_rate>10</update_rate><left_joint>left_joint</left_joint><right_joint>right_joint</right_joint><wheel_separation>0.4</wheel_separation><wheel_diameter>0.2</wheel_diameter><max_wheel_torque>20</max_wheel_torque><max_wheel_acceleration>2</max_wheel_acceleration><odometry_source>0</odometry_source><publish_odom>true</publish_odom><publish_odom_tf>false</publish_odom_tf><publish_wheel_tf>false</publish_wheel_tf><odometry_frame>map</odometry_frame><robot_base_frame>base_link</robot_base_frame></plugin></model>'''
for i, (x, y) in enumerate(LANDMARKS):
    world += f'''<model name="landmark_{i}"><static>true</static><pose>{x} {y} 0.75 0 0 0</pose><link name="post"><collision name="c"><geometry><cylinder><radius>0.12</radius><length>1.5</length></cylinder></geometry></collision><visual name="v"><geometry><cylinder><radius>0.12</radius><length>1.5</length></cylinder></geometry><material><ambient>0.9 0.4 0.1 1</ambient><diffuse>0.9 0.4 0.1 1</diffuse></material></visual></link></model>'''
world += '<gui><camera name="overview"><pose>15 -12 18 0 0.8 2.15</pose></camera></gui></world></sdf>'
(ROOT/'gazebo'/'slam_lab.world').write_text(world)
