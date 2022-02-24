#!/usr/bin/env python3

import rospy
import tf, tf2_ros

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from nav_msgs.srv import GetPlan
from pbr_gazebo.srv import MoveGazeboModel, MoveGazeboModelResponse

import numpy as np

class GazeboModelNavigation:
    '''
    Navigate a model in gazebo by making a move base robot plan
    and applying it to the model using teletransportation ("set model pose" gazebo service calls)
    Usage: run the node and publish a geometry_msgs/PoseStamped msg on the topic: '/<model_name>/goal_pose
    e.g. /worker/goal_pose'
    Alternatively call the service offered by this node of type "MoveGazeboModel", with name : <node_name>/start_navigation
    '''
    def __init__(self):
        self.model_name = rospy.get_param('~model_name', 'my_gazebo_model')
        # offset at start: if the model is detected by the robot laser scanner the plan would not be possible
        # therefore a small offset can be introduced to make the path planning possible
        x_offset_at_start = rospy.get_param('~x_offset_at_start', 0.0)
        y_offset_at_start = rospy.get_param('~y_offset_at_start', 0.0)
        z_offset_at_start = rospy.get_param('~z_offset_at_start', 0.0)
        roll_offset_at_start = rospy.get_param('~roll_offset_at_start', 0.0)
        pitch_offset_at_start = rospy.get_param('~pitch_offset_at_start', 0.0)
        yaw_offset_at_start = rospy.get_param('~yaw_offset_at_start', 0.0)
        self.offset_at_start=[x_offset_at_start, y_offset_at_start, z_offset_at_start,
                              roll_offset_at_start, pitch_offset_at_start, yaw_offset_at_start]
        # offset during execution: useful to apply a constant offset (usually angular) to the model
        # throghout the entire trajectory
        x_offset_during_execution = rospy.get_param('~x_offset_during_execution', 0.0)
        y_offset_during_execution = rospy.get_param('~y_offset_during_execution', 0.0)
        z_offset_during_execution = rospy.get_param('~z_offset_during_execution', 0.0)
        roll_offset_during_execution = rospy.get_param('~roll_offset_during_execution', 0.0)
        pitch_offset_during_execution = rospy.get_param('~pitch_offset_during_execution', 0.0)
        yaw_offset_during_execution = rospy.get_param('~yaw_offset_during_execution', 0.0)
        self.offset_during_execution = [x_offset_during_execution, y_offset_during_execution,
                                        z_offset_during_execution, roll_offset_during_execution,
                                        pitch_offset_during_execution, yaw_offset_during_execution]
        
        self.robot_base_frame = rospy.get_param('~robot_base_frame', 'mobipick/base_footprint')
        self.robot_navigation_frame = rospy.get_param('~robot_navigation_frame', 'map')
        self.robot_move_base_make_plan_srv_name = rospy.get_param('~robot_move_base_make_plan_srv_name',
                                                                  '/mobipick/move_base_node/make_plan')
        self.robot_gazebo_model_name = rospy.get_param('~robot_gazebo_model_name', 'mobipick')
        # flag to indicate that a model goal pose was received in the topic
        self.model_goal_request_received = False
        self.model_states_msg_received = False
        self.model_states_msg = None
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.ModelStatesCB, queue_size=1)
        self.pub_start_pose = rospy.Publisher('~start_pose', PoseStamped, queue_size=1)
        self.pub_path_pose = rospy.Publisher('~path_pose', PoseStamped, queue_size=1)
        # subscribe to model goal pose
        rospy.Subscriber(f'~goal_pose', PoseStamped, self.ModelGoalPoseCB, queue_size=1)
        self.tf_listener = tf.TransformListener()
        # setup service to move gazebo model
        rospy.Service(f'~start_navigation', MoveGazeboModel, self.handle_gazebo_model_navigation)
        rospy.sleep(0.5)
        # get transform from world to map and publish to tf
        self.compute_tf_world_2_map()

    def handle_gazebo_model_navigation(self, req):
        rospy.loginfo(f'received (service) request to navigate the model: {self.model_name}')
        self.model_navigation(self.model_name, req.goal_model_pose,
                                      offset_at_start=self.offset_at_start,
                                      offset_during_execution=self.offset_during_execution,
                                      speed=req.speed)
        resp = MoveGazeboModelResponse()
        resp.success = True
        return resp

    def ModelGoalPoseCB(self, msg):
        rospy.loginfo(f'received (topic) request to navigate model: {self.model_name}')
        self.model_goal_request_received = True
        self.model_goal_pose = msg

    def ModelStatesCB(self, msg):
        self.model_states_msg_received = True
        self.model_states_msg = msg

    def get_robot_pose(self, robot_frame='base_link', global_frame='map'):
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(global_frame, robot_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(global_frame, robot_frame, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn('failed to get robot pose')
            return None
        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        robot_pose.header.frame_id = global_frame
        robot_pose.pose.position.x = trans[0]
        robot_pose.pose.position.y = trans[1]
        robot_pose.pose.position.z = trans[2]
        robot_pose.pose.orientation.x = rot[0]
        robot_pose.pose.orientation.y = rot[1]
        robot_pose.pose.orientation.z = rot[2]
        robot_pose.pose.orientation.w = rot[3]
        return robot_pose

    def get_model_pose(self, model_name):
        '''
        query from gazebo model states the ground truth pose of a model
        '''
        if not self.model_states_msg_received:
            return False
        # iterate over all gazebo models
        model_found = False
        for index, name in enumerate(self.model_states_msg.name):
            if name == model_name:
                index_of_requested_model = index
                model_found = True
        if not model_found:
            rospy.logerr('get model start pose: model NOT found')
            return False
        model_pose_stamped = PoseStamped()
        model_pose_stamped.header.frame_id = 'world'
        model_pose_stamped.pose = self.model_states_msg.pose[index_of_requested_model]
        return model_pose_stamped

    def set_model_pose(self, model_name, desired_pose):
        # transform to world reference frame
        desired_pose_in_world_frame = self.transform_pose(desired_pose, 'world')
        srv_name = '/gazebo/set_model_state'
        rospy.wait_for_service(srv_name)
        try:
            set_model_state = rospy.ServiceProxy(srv_name, SetModelState)
            request_msg = ModelState()
            request_msg.model_name = model_name
            request_msg.pose = desired_pose_in_world_frame.pose
            request_msg.reference_frame = 'world'

            resp1 = set_model_state(request_msg)
            return resp1.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def make_plan(self, start_pose, goal_pose, tolerance = 0.5):
        '''
        call move base srv to make a navigation plan for the robot and return it if one was found
        '''
        rospy.loginfo(f'making a plan for the {self.model_name} to navigate around')
        # publish both poses for debugging purposes
        self.pub_start_pose.publish(start_pose)

        move_base_make_plan_srv = self.robot_move_base_make_plan_srv_name
        rospy.wait_for_service(move_base_make_plan_srv)

        try:
            move_base_make_plan_srv = rospy.ServiceProxy(move_base_make_plan_srv, GetPlan)
            resp = move_base_make_plan_srv(start_pose, goal_pose, tolerance)
            return resp.plan
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None

    def make_transform_from_pose_stamped(self, pose_stamped_msg):
        rot = [pose_stamped_msg.pose.orientation.x, pose_stamped_msg.pose.orientation.y,
               pose_stamped_msg.pose.orientation.z, pose_stamped_msg.pose.orientation.w]
        euler_rot = tf.transformations.euler_from_quaternion(rot)
        transform = tf.transformations.euler_matrix(euler_rot[0], euler_rot[1], euler_rot[2])
        transform[0][3] = pose_stamped_msg.pose.position.x # pborg x
        transform[1][3] = pose_stamped_msg.pose.position.y # pborg y
        transform[2][3] = pose_stamped_msg.pose.position.z # pborg z
        return transform

    def make_pose_stamped_from_transform(self, transform, frame_id):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = frame_id
        pose_stamped_msg.pose.position.x = transform[0][3]
        pose_stamped_msg.pose.position.y = transform[1][3]
        pose_stamped_msg.pose.position.z = transform[2][3]
        roll, pitch, yaw = tf.transformations.euler_from_matrix(transform)
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose_stamped_msg.pose.orientation.x = quat[0]
        pose_stamped_msg.pose.orientation.y = quat[1]
        pose_stamped_msg.pose.orientation.z = quat[2]
        pose_stamped_msg.pose.orientation.w = quat[3]
        return pose_stamped_msg

    def broadcast_tf(self, transform, parent_frame_id, child_frame_id):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = parent_frame_id
        static_transformStamped.child_frame_id = child_frame_id
        static_transformStamped.transform.translation.x = transform[0][3]
        static_transformStamped.transform.translation.y = transform[1][3]
        static_transformStamped.transform.translation.z = transform[2][3]
        roll, pitch, yaw = tf.transformations.euler_from_matrix(transform)
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]
        broadcaster.sendTransform(static_transformStamped)

    def compute_tf_world_2_map(self):
        amcl_robot_pose = self.get_robot_pose(robot_frame=self.robot_base_frame,
                                              global_frame=self.robot_navigation_frame)
        gazebo_robot_pose = self.get_model_pose(self.robot_gazebo_model_name)
        # make transforms
        robot2map_tf = self.make_transform_from_pose_stamped(amcl_robot_pose)
        robot2world_tf = self.make_transform_from_pose_stamped(gazebo_robot_pose)

        world2robot_tf = np.linalg.inv(robot2world_tf)
        map2robot_tf = np.linalg.inv(robot2map_tf)

        self.map2world_tf = np.dot(robot2world_tf, map2robot_tf)
        self.world2map_tf = np.dot(robot2map_tf, world2robot_tf)

        # send static transform to ros (world to map)
        self.broadcast_tf(self.map2world_tf, parent_frame_id='world',
                          child_frame_id=self.robot_navigation_frame)

    def make_pose_stamped_msg(self, frame_id, px, py, pz, ox, oy, oz, ow):
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.frame_id = frame_id
        pose_stamped_msg.pose.position.x = px
        pose_stamped_msg.pose.position.y = py
        pose_stamped_msg.pose.position.z = pz
        pose_stamped_msg.pose.orientation.x = ox
        pose_stamped_msg.pose.orientation.y = oy
        pose_stamped_msg.pose.orientation.z = oz
        pose_stamped_msg.pose.orientation.w = ow
        return pose_stamped_msg

    def transform_pose(self, input_pose, target_reference_frame):
        if input_pose == None:
            rospy.logerr('failed to transform pose: input pose cannot be None')
            return None
        # transform to target reference frame
        current_reference_frame = input_pose.header.frame_id
        try:
            now = rospy.Time.now()
            self.tf_listener.waitForTransform(current_reference_frame,
                            target_reference_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(
                            current_reference_frame, target_reference_frame, now)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn(
                f'failed to lookup transform from {current_reference_frame} to {target_reference_frame}')
            return None
        return self.tf_listener.transformPose(target_reference_frame, input_pose)

    def execute_path(self, path, model_name, offset_during_execution=[0,0,0,0,0,0], speed=0.07):
        '''
        let a model in gazebo to execute a navigation path
        model_offset = [x, y, z, roll, pitch, yaw]
        '''
        for pose in path.poses:
            self.set_model_pose(model_name, self.offset_pose(pose, offset_during_execution))
            # publish pose for debugging purposes
            self.pub_path_pose.publish(pose)
            # interrupt if new goal is received
            if self.model_goal_request_received:
                return
            rospy.sleep(speed)

    def offset_pose(self, pose, offset):
        '''
        offset = [x, y, z, roll, pitch, yaw]
        '''
        model_to_world_tf = self.make_transform_from_pose_stamped(pose)
        transform = tf.transformations.euler_matrix(0.0, 0.0, 0.0)
        transform[0][3] = offset[0]
        transform[1][3] = offset[1]
        transform[2][3] = offset[2]
        pose_stamped_msg = self.make_pose_stamped_from_transform(
                                np.dot(model_to_world_tf, transform), pose.header.frame_id)
        # apply rotation to a pose
        rot = [pose_stamped_msg.pose.orientation.x, pose_stamped_msg.pose.orientation.y,
               pose_stamped_msg.pose.orientation.z, pose_stamped_msg.pose.orientation.w]
        euler_rot = list(tf.transformations.euler_from_quaternion(rot))
        euler_rot[0] += offset[3] # roll
        euler_rot[1] += offset[4] # pitch
        euler_rot[2] += offset[5] # yaw
        q = tf.transformations.quaternion_from_euler(euler_rot[0], euler_rot[1], euler_rot[2])
        pose_stamped_msg.pose.orientation.x = q[0]
        pose_stamped_msg.pose.orientation.y = q[1]
        pose_stamped_msg.pose.orientation.z = q[2]
        pose_stamped_msg.pose.orientation.w = q[3]
        return pose_stamped_msg

    def model_navigation(self, model_name, goal_pose, offset_at_start=[0,0,0,0,0,0],
                               offset_during_execution=[0,0,0,0,0,0], speed=0.07):
        '''
        make a gazebo model to navigate in the environment by calling teletransport method multiple times
        offset_at_start = [x, y, z, roll, pitch, yaw]
        offset_during_execution = [x, y, z, roll, pitch, yaw]
        '''
        goal_reference_frame = goal_pose.header.frame_id
        # get model current pose
        model_pose_in_world_frame = self.get_model_pose(model_name)
        # apply offset at the start pose of the model
        model_pose_in_world_frame = self.offset_pose(model_pose_in_world_frame, offset_at_start)
        # transform model pose to goal pose frame
        model_pose_in_map_frame = self.transform_pose(model_pose_in_world_frame, goal_reference_frame)
        # make plan
        path = self.make_plan(model_pose_in_map_frame, self.transform_pose(goal_pose, goal_reference_frame))
        # execute plan with model
        self.execute_path(path, model_name, offset_during_execution=offset_during_execution, speed=speed)

    def start(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.model_goal_request_received:
                self.model_goal_request_received = False
                self.model_navigation(self.model_name, self.model_goal_pose,
                                      offset_at_start=self.offset_at_start,
                                      offset_during_execution=self.offset_during_execution, speed=0.07)
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('gazebo_model_navigation')
    gazebo_model_navigation = GazeboModelNavigation()
    gazebo_model_navigation.start()
