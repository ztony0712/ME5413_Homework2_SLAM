#!/usr/bin/env python

import os
import rosbag
import argparse

import tf.transformations as tft
import geometry_msgs.msg

class Converter:
    def __init__(self, bag_path, txt_path, topic_name, body_frame, camera_frame):
        self.file_path = txt_path
        if os.path.exists(self.file_path):
            os.remove(self.file_path)
        self.file = open(self.file_path, 'w')
        self.body_frame = body_frame
        self.camera_frame = camera_frame
        self.first_timestamp = 1317354879.24 # get from the have_fun.bag
        self.transform = None
        self.process_bag(bag_path, topic_name)

    def process_bag(self, bag_path, topic_name):
        bag = rosbag.Bag(bag_path)
        for _, msg, _ in bag.read_messages(topics=['/tf_static']):
            for trans in msg.transforms:
                if trans.child_frame_id == self.camera_frame and trans.header.frame_id == self.body_frame:
                    self.transform = trans
                    break

        if self.transform is None:
            print("Required static transform not found.")
            return
    
        for _, msg, _ in bag.read_messages(topics=[topic_name]):
            self.callback(msg)
        bag.close()

    def callback(self, msg):
        trans = self.transform.transform.translation
        rot = self.transform.transform.rotation
        pose = geometry_msgs.msg.PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Adjust position based on static transform
        pose.pose.position.x -= trans.x
        pose.pose.position.y -= trans.y
        pose.pose.position.z -= trans.z
        msg_quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )
        euler = tft.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        q_rot = tft.quaternion_from_euler(euler[0], euler[1], euler[2])
        q_pose = tft.quaternion_multiply(msg_quaternion, q_rot)
        pose.pose.orientation = geometry_msgs.msg.Quaternion(*q_pose)

        time = msg.header.stamp.to_sec() - self.first_timestamp
        msg_quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )
        self.file.write(f"{time} {-pose.pose.position.x} {pose.pose.position.y} {pose.pose.position.z} {msg_quaternion[0]} {msg_quaternion[1]} {msg_quaternion[2]} {msg_quaternion[3]}\n")

    def run(self):
        self.file.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', help='Input the path of the target rosbag')
    parser.add_argument('txt_path', help='Input the path of the target txt file')
    parser.add_argument('--odom_topic', default='/vins_estimator/odometry', help='Odom topic name in the bag file')
    parser.add_argument('--body_frame', default='imu_link', help='body tf frame name')
    parser.add_argument('--camera_frame', default='camera_gray_left', help='camera tf frame name')
    args = parser.parse_args()

    node = Converter(args.bag_path, args.txt_path, args.odom_topic, args.body_frame, args.camera_frame)
    node.run()
