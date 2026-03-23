#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_msgs.msg
import re
import math
from std_msgs.msg import Int16

"""
Bu düğümde, Tabelanın TF dönüşü yardımıyla harita üzerindeki konum bilgisi alınır ve önceden belirlenmiş 7 adet
park alanlarından hangisinde olduğu eşleştirilir. Ve yol planı yapılacak olan park noktası belirlenmiş olur.
"""

class ParkingTFMatcher:
    def __init__(self):
        rospy.init_node("park_tf_matcher", anonymous=True)

        self.park_areas = {
            1: (-7.55, 6.42),
            2: (-7.55, 5.48),
            3: (-7.47, 4.48),
            4: (-7.28, 3.51),
            5: (-7.06, 2.55),
            6: (-7.0, 1.57),
            7: (-6.7, 0.58),
        }
        
        self.dist_threshold = 3.0
        self.izin = False
        self.published = False
        self.printed_frames = set()
        self.matched_parks = dict()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.pub = rospy.Publisher("PARK_NUMARA", Int16, queue_size=10)
        
        rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, self.tf_callback)
        rospy.Subscriber("/park_aktif", Int16, self.izin_callback)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_smallest_once)

    def euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def find_nearest_parking_area(self, x, y):
        min_dist = float('inf')
        matched_area = None
        
        for area_id, (px, py) in self.park_areas.items():
            dist = self.euclidean_distance(x, y, px, py)
            if dist < min_dist and dist < self.dist_threshold:
                min_dist = dist
                matched_area = area_id
                
        return matched_area

    def publish_smallest_once(self, event):
        if self.published or not self.matched_parks:
            return

        min_park_area = min(self.matched_parks.values())
        msg = Int16(data=min_park_area)
        self.pub.publish(msg)
        
        rospy.loginfo(f"[ZAMANLAYICI] En kucuk park alani: park_{min_park_area}")
        
        self.published = True
        self.timer.shutdown()

    def izin_callback(self, msg):
        if msg.data == 2:
            self.izin = True

    def tf_callback(self, msg):
        if not self.izin:
            return

        for transform in msg.transforms:
            child = transform.child_frame_id
            
            if re.match(r'^park_\d+$', child):
                if child in self.printed_frames:
                    continue
                    
                try:
                    if self.tf_buffer.can_transform("map", child, rospy.Time(0), rospy.Duration(0.1)):
                        trans = self.tf_buffer.lookup_transform("map", child, rospy.Time(0))
                    else:
                        continue

                    x = trans.transform.translation.x
                    y = trans.transform.translation.y

                    matched_area = self.find_nearest_parking_area(x, y)
                    
                    if matched_area is not None:
                        rospy.loginfo(f"{child} tf'i -> park alani {matched_area}'e ait")
                        self.printed_frames.add(child)
                        self.matched_parks[child] = matched_area
                    else:
                        rospy.logwarn(f"{child} icin eslesen park alani bulunamadi.")

                except Exception as e:
                    rospy.logwarn(f"{child} icin TF alinamadi: {e}")

if __name__ == "__main__":
    try:
        matcher = ParkingTFMatcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
