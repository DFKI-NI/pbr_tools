#!/usr/bin/env python3

"""
Node that subscribes to the /gazebo/model_states topic, retrieves the pose of
models in the Gazebo simulation, and allows the user to query specific objects by entering their index.

For each queried object, the script prints:
1. Its pose in x, y, z, roll, pitch, yaw format (rounded to 4 decimal places).
2. A formatted XML snippet suitable for including in a ROS launch file to spawn the model.

The XML snippet includes:
- The object's full name (with numbers) for the -model field.
- A base description name (without numbers) for the -param field.

Upon exiting the script with Ctrl+C, all queried objects are printed in XML format, ready for copy-pasting
into a launch file.
"""

import rospy
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
import signal
import sys

class GazeboModelInfo:
    def __init__(self):
        rospy.init_node('gazebo_model_info', anonymous=True)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.model_data = None
        self.queried_objects = []
        signal.signal(signal.SIGINT, self.signal_handler)  # Handle Ctrl+C
        rospy.spin()

    def callback(self, msg):
        self.model_data = msg
        self.print_available_models()

    def print_available_models(self):
        if self.model_data is None:
            return

        print('\nAvailable models:')
        for idx, name in enumerate(self.model_data.name):
            print(f'{idx + 1}) {name}')

        try:
            obj_idx = int(input('\nEnter object number to inspect: ')) - 1
            if 0 <= obj_idx < len(self.model_data.name):
                self.print_model_info(obj_idx)
            else:
                print('Invalid number!')
        except ValueError:
            print('Invalid input! Please enter a number.')

    def print_model_info(self, obj_idx):
        name = self.model_data.name[obj_idx]
        pose = self.model_data.pose[obj_idx]

        position = pose.position
        orientation = pose.orientation

        # Convert quaternion to roll, pitch, yaw
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        
        # Derive the common description name (e.g., remove suffix like "_1" only for -param)
        description_name = self.get_description_name(name)

        # Save XML format
        xml_entry = f'''
  <!-- spawn {name} -->
  <node name="spawn_{name}" pkg="gazebo_ros" type="spawn_model"
        args="-urdf -param {description_name}_description -model {name}
              -x {round(position.x, 4)} -y {round(position.y, 4)} -z {round(position.z, 4)}
              -R {round(roll, 4)} -P {round(pitch, 4)} -Y {round(yaw, 4)}"
        respawn="false" output="screen" />'''
        self.queried_objects.append(xml_entry)

        # Print pose in x, y, z, roll, pitch, yaw format
        print(f'\nPose of {name}:')
        print(f'x: {round(position.x, 4)}, y: {round(position.y, 4)}, z: {round(position.z, 4)}, '
              f'roll: {round(roll, 4)}, pitch: {round(pitch, 4)}, yaw: {round(yaw, 4)}')

        # Print XML format
        print('\nXML Format:')
        print(xml_entry)

    def get_description_name(self, model_name):
        # Split by "_" and remove trailing numbers only for description
        base_name = model_name.split('_')
        if base_name[-1].isdigit():
            return '_'.join(base_name[:-1])
        return model_name

    def signal_handler(self, sig, frame):
        print('\nExiting... Here are the XML entries for queried objects:')
        for xml_entry in self.queried_objects:
            print(xml_entry)
        sys.exit(0)

if __name__ == '__main__':
    try:
        GazeboModelInfo()
    except rospy.ROSInterruptException:
        pass
