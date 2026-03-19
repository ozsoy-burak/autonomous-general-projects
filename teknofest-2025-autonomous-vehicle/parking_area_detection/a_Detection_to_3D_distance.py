#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import ast
from math import sqrt
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

"""
Kameradan tespit edilen Park-Park Yasak Tabelaları, 3D map üzerinde konumlandırılabilmesi için bu düğüm ile 
tabelanın X-Y-Z pozisyonlarındaki araca olan uzaklıkları hesaplanır.
"""

class TrafficSignTFBroadcaster:
    def __init__(self):
        rospy.init_node('zed2_3d_object_detector', anonymous=True)

        self.target_width = 720
        self.target_height = 480
        self.dist_threshold = 0.6
        self.max_detection_dist = 12.0
        
        self.camera_info = None
        self.depth_image = None
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.counters = {
            "park": 0,
            "park_yasak": 0
        }
        
        self.prev_poses = {
            "park": [],
            "park_yasak": []
        }

        self.valid_classes = ['park', 'park_yasak']

        rospy.Subscriber('/zed2/depth/depth_registered', Image, self.depth_callback)
        rospy.Subscriber('/zed2/left/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/sign_detection', String, self.detected_object_callback)

    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.scale_x = self.target_width / msg.width
            self.scale_y = self.target_height / msg.height
            self.fx = msg.K[0] * self.scale_x
            self.fy = msg.K[4] * self.scale_y
            self.cx = msg.K[2] * self.scale_x
            self.cy = msg.K[5] * self.scale_y

    def depth_callback(self, msg):
        try:
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_image = cv2.resize(depth_raw, (self.target_width, self.target_height))
        except CvBridgeError as e:
            rospy.logerr_throttle(2, f"CvBridge Error: {e}")

    def get_existing_frame_id(self, obj_class, x, y):
        for px, py, fid in self.prev_poses[obj_class]:
            dist = np.sqrt((px - x) ** 2 + (py - y) ** 2)
            if dist < self.dist_threshold:
                return fid
        return None

    def get_robust_depth(self, u, v, window_size=5):
        half_w = window_size // 2
        v_min = max(0, v - half_w)
        v_max = min(self.depth_image.shape[0], v + half_w + 1)
        u_min = max(0, u - half_w)
        u_max = min(self.depth_image.shape[1], u + half_w + 1)
        
        roi = self.depth_image[v_min:v_max, u_min:u_max]
        valid_depths = roi[(roi > 0.0) & (~np.isnan(roi))]
        
        if len(valid_depths) == 0:
            return 0.0
        return np.median(valid_depths)

    def detected_object_callback(self, msg):
        if self.camera_info is None or self.depth_image is None:
            return

        try:
            data_list = ast.literal_eval(msg.data)
            
            if data_list[0] == "NONE" or data_list[0] is None:
                return
                
            cl = data_list[0]
            x_center = float(data_list[2])
            y_center = float(data_list[3])
        except (ValueError, SyntaxError, IndexError):
            return

        if cl not in self.valid_classes:
            return

        u, v = int(x_center), int(y_center)

        if v >= self.depth_image.shape[0] or u >= self.depth_image.shape[1]:
            return

        Z = self.get_robust_depth(u, v)

        if Z <= 0.0:
            return

        X = ((u - self.cx) * Z) / self.fx
        Y = ((v - self.cy) * Z) / self.fy

        distanc = sqrt(X**2 + Z**2)

        if distanc >= self.max_detection_dist:
            return

        existing_id = self.get_existing_frame_id(cl, X, Z)

        if existing_id is not None:
            frame_id = existing_id
        else:
            self.counters[cl] += 1
            frame_id = f"{cl}_{self.counters[cl]}"
            self.prev_poses[cl].append((X, Z, frame_id))

        self.publish_tf(frame_id, X, Z)

    def publish_tf(self, frame_id, X, Z):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "chassiszed2_camera_center"
        transform.child_frame_id = frame_id

        transform.transform.translation.x = Z
        transform.transform.translation.y = -X
        transform.transform.translation.z = 0.0
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

if __name__ == '__main__':
    try:
        node = TrafficSignTFBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
