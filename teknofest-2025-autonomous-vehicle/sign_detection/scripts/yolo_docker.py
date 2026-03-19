#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS Node for 3D Traffic Sign Detection using YOLO and ZED Stereo Camera Depth Map.
Fuses 2D bounding boxes with depth information to calculate relative XYZ coordinates.
"""

import cv2
import rospy
import numpy as np
from math import sqrt
from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String, Int16
from sensor_msgs.msg import Image, CameraInfo
import torch

class TrafficSignDetector3D:
    def __init__(self):
        rospy.init_node('yolo_3d_sign_detector', anonymous=True)

        # Parametreler (Hardcoded)
        self.model_path = 'detecrion_model.pt'
        self.target_width = 720
        self.target_height = 480
        self.default_distance_threshold = 10.0
        
        # Sınıflar ve Güvenilirlik Eşikleri
        self.classes = [
            'dur', 'durak', 'duz', 'engelli_park', 'girilmez', 'ilerden_sag', 
            'ilerden_sol', 'ileri_veya_sag', 'ileri_veya_sol', 'kavsak', 'kirmizi', 
            'park', 'park_yasak', 'sag_mecburi', 'saga_donulmez', 'sol_mecburi', 
            'sola_donulmez', 'tunel', 'yaya_gecidi', 'yesil', 'yol_calismasi', 'zmecburi_yon'
        ]
        self.confidence_threshold = 0.7
        self.zone_distance_thresholds = {14: 20.0}

        self.current_zone = 0
        self.camera_info = None
        self.depth_image = None
        self.bridge = CvBridge()
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        rospy.loginfo(f"YOLO Modeli Yükleniyor... Cihaz: {self.device}")
        self.model = YOLO(self.model_path).to(self.device)

        self.detection_pub = rospy.Publisher('/sign_detection', String, queue_size=10)
        
        rospy.Subscriber("/zone", Int16, self.zone_callback)
        rospy.Subscriber("/zed2/zed_node/left/camera_info", CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, self.depth_callback)
        rospy.Subscriber("/zed2/zed_node/left/image_rect_color", Image, self.image_callback)

        rospy.loginfo("3D Tabela Tespit Düğümü Başarıyla Başlatıldı.")

    def zone_callback(self, msg):
        self.current_zone = msg.data

    def camera_info_callback(self, msg):
        """Kamera matrisini dinamik olarak çeker. Çözünürlük değişirse FX/FY'yi ölçekler."""
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
            depth_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1") # Derinlik genelde Float32 olur
            self.depth_image = cv2.resize(depth_raw, (self.target_width, self.target_height))
        except CvBridgeError as e:
            rospy.logerr_throttle(2, f"Depth görüntüsü çevri süremedi: {e}")

    def get_robust_depth(self, u, v, window_size=5):
        """Tek pikseldeki gürültüyü (NaN/0) önlemek için merkez etrafındaki medyanı alır."""
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

    def image_callback(self, msg):
        if self.depth_image is None or self.camera_info is None:
            rospy.logwarn_throttle(2, "Depth veya CameraInfo bekleniyor...")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (self.target_width, self.target_height))
        except CvBridgeError as e:
            rospy.logerr(f"RGB görüntü çevrilemedi: {e}")
            return

        results = self.model(frame, verbose=False)
        detections = results[0].boxes

        detect_any = False

        for box in detections:
            conf = box.conf.item()
            cls = int(box.cls.item())

            if conf >= self.confidence_threshold:
                label = self.classes[cls]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                
                u = int((x1 + x2) / 2)
                v = int((y1 + y2) / 2)

                Z = self.get_robust_depth(u, v)
                
                if Z <= 0.0 or np.isnan(Z):
                    continue

                X = ((u - self.cx) * Z) / self.fx
                Y = ((v - self.cy) * Z) / self.fy
                distance = sqrt(X**2 + Y**2 + Z**2)

                # Burası bölge bazlı filtreleme, haritadaki bazı zone noktalarında dinamik güncellenir.
                threshold_distance = self.zone_distance_thresholds.get(self.current_zone, self.default_distance_threshold)
                if distance > threshold_distance:
                    continue

                detect_any = True
                
                detection_msg = f"{label},{conf:.2f},{u},{v},{X:.2f},{Z:.2f},{distance:.2f}"
                self.detection_pub.publish(detection_msg)
                
                rospy.loginfo(f"[{label}] tespit edildi | Mesafe: {distance:.2f}m | Yanal(X): {X:.2f}m | Derinlik(Z): {Z:.2f}m")

        if not detect_any:
            self.detection_pub.publish("NONE")

if __name__ == '__main__':
    try:
        node = TrafficSignDetector3D()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
