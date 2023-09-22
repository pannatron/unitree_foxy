import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mediapipe as mp
import cv2

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class HandDetector(Node):
    def __init__(self):
        super().__init__('hand_detector')
        self.subscriber = self.create_subscription(
            Image,
            '/front_camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.hands = mp_hands.Hands(static_image_mode=False, max_num_hands=2, min_detection_confidence=0.5)

    def image_callback(self, msg):
        # Convert sensor_msgs/Image to an OpenCV Image
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Process the image
        results = self.hands.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        
        # Draw hand landmarks
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        cv2.imshow('Hand Detection', img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    hand_detector = HandDetector()
    rclpy.spin(hand_detector)
    hand_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
