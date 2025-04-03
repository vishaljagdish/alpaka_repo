import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class Model(Node):
    def __init__(self):
        super().__init__('ml_model')

        self.hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
            model_complexity=1
        )

        self.mp_draw = mp.solutions.drawing_utils
        self.mp_hands = mp.solutions.hands

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg)       
        results = self.hands.process(cv_image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                x = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x * 1280)
                y = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y * 720)

                cv2.circle(cv_image, (x, y), 5, (0, 255, 0), -1)

        cv2.imshow("Hand Tracking", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    model = Model()
    rclpy.spin(model)
    model.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
