#!/usr/bin/env python3

'''
spawn sdf objects in gazebo based on a config file that specifies the poses
'''

import yaml

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

class SpawnGazeboObjects:
    def __init__(self):
        self.path_to_sdf_files = rospy.get_param('~path_to_sdf_files', None)
        if not self.path_to_sdf_files:
            rospy.signal_shutdown('required param path_to_sdf_files not found, will exit')
        self.yaml_path = rospy.get_param('~object_poses', None)
        if not self.yaml_path:
            rospy.signal_shutdown('required param object_poses not found, will exit')
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        self.spawn_model = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

    def read_yaml(self):
        with open(self.yaml_path, 'r') as stream:
            try:
                self.yaml_data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return

    def remove_id(self, scene_name):
        split_name = scene_name.split('_')
        return '_'.join(split_name[:-1]) if split_name[-1].isdigit() else scene_name

    def spawn_boxes(self):
        for item in self.yaml_data['planning_scene_boxes']:
            sdf_file_path = self.path_to_sdf_files + "/" + self.remove_id(item['scene_name']) + ".sdf"
            with open(sdf_file_path, 'r') as f:
                model_xml = f.read()

            self.spawn_model(
                model_name=item['scene_name'],
                model_xml=model_xml,
                robot_namespace="",
                initial_pose=Pose(
                    position=Point(
                        x=item['box_position_x'],
                        y=item['box_position_y'],
                        z=item['box_position_z']),
                    orientation=Quaternion(
                        x=item['box_orientation_x'],
                        y=item['box_orientation_y'],
                        z=item['box_orientation_z'],
                        w=item['box_orientation_w'])),
                reference_frame=item['frame_id']
            )

if __name__ == '__main__':
    rospy.init_node('spawn_gazebo_objects_node')
    spawner = SpawnGazeboObjects()
    spawner.read_yaml()
    spawner.spawn_boxes()
