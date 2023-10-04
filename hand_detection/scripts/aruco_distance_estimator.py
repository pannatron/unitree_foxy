#!/usr/bin/python3
import cv2
import cv2.aruco as aruco
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64
import numpy as np
from ros2_unitree_legged_msgs.msg import HandCheck
class ArucoDistanceEstimator(Node):
    def __init__(self):
        super().__init__('aruco_distance_estimator')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10)
        self.distance_pub = self.create_publisher(
            Float64,
            '/aruco_distance',
            10)
        self.handcheck = self.create_publisher(
            HandCheck,
            'handcheck',
            10)
    def image_callback(self, img_msg):
    
        handcheck_msg = HandCheck()
        camera_matrix = np.array([[217.5264044684359, 0, 244.50197112613543], [0, 219.40238512076758, 203.8519876672617], [0, 0, 1]],dtype=np.float64)
        dist_coeffs = np.array([0.009707144781793632, -0.03883462816962419, 0.0038944909172163008, 0.007438806944911315, 0.013900489524656383], dtype=np.float64)
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        parameters = aruco.DetectorParameters()
        corners, ids, _ = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)

        if len(corners) > 0:
            # Assuming the marker size is 0.0070 meters
            marker_size = 0.20
        
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)
            distance = tvec[0][0][2]  # Distance to the first marker found
            distance_msg = Float64()            	
            distance_msg.data = distance * 100
            self.distance_pub.publish(distance_msg)
            if (distance * 100) > 80:
            	handcheck_msg.hand_type=7
            	self.handcheck.publish(handcheck_msg)
            elif (distance * 100) <60:
            	handcheck_msg.hand_type=6
            	self.handcheck.publish(handcheck_msg)
            else:
                handcheck_msg.hand_type=8
                self.handcheck.publish(handcheck_msg)

def main(args=None):
    rclpy.init(args=args)
    aruco_distance_estimator = ArucoDistanceEstimator()
    rclpy.spin(aruco_distance_estimator)
    aruco_distance_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

