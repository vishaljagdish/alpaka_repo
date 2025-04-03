import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import curses
import numpy as np
import mediapipe as mp
from collections import deque
from geometry_msgs.msg import Point

# Constants
BUFFER_SIZE = 10

fx = 603.3972778320312
fy = 603.15185546875
cx = 324.9955749511719
cy = 253.40223693847656

# Helper Function
def deproject(x, y, depth):
    x_buf = (x - cx)/fx
    y_buf = (y - cy)/fy
    x_proj = x_buf * depth
    y_proj = y_buf * depth
    return [x_proj, y_proj, depth]


# Class definition
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

        self.color_sub = self.create_subscription(
            Image,
            '/camera/sensor_d435i/color/image_raw',
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/sensor_d435i/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.publish_points = self.create_publisher(
            Point,
            '/points',
            10
        )
        self.latest_depth_image = None
        self.x = None
        self.y = None
        self.z_buffer = deque(maxlen=BUFFER_SIZE)
        self.cv_image = None
        self.saved_points = []  # To store points where 's' was pressed
        self.mouse_points = []  # To store points from mouse clicks
        
        # Initialize curses
        self.stdscr = curses.initscr()
        curses.cbreak()
        self.stdscr.keypad(True)
        curses.noecho()
        self.stdscr.nodelay(True)

        # Create OpenCV window and set mouse callback
        cv2.namedWindow("Hand Tracking")
        cv2.setMouseCallback("Hand Tracking", self.mouse_callback)

    def __del__(self):
        curses.nocbreak()
        self.stdscr.keypad(False)
        curses.echo()
        curses.endwin()

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.latest_depth_image is not None:
                depth_value = self.latest_depth_image[y, x] * 0.001
                if depth_value > 0:
                    self.mouse_points.append((x, y))
                    deprojected_point = deproject(x, y, depth_value)
                    
                    msg = Point()
                    msg.x = deprojected_point[0]
                    msg.y = deprojected_point[1]
                    msg.z = deprojected_point[2]
                    
                    self.publish_points.publish(msg)

    def image_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        display_image = self.cv_image.copy()

        results = self.hands.process(display_image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.x = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].x * 640)
                self.y = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP].y * 480)

                cv2.circle(display_image, (self.x, self.y), 5, (0, 255, 0), -1)

        # Draw all saved points (from 's' key)
        for point in self.saved_points:
            cv2.circle(display_image, (point[0], point[1]), 4, (255, 0, 0), -1)
            
        # Draw all mouse click points
        for point in self.mouse_points:
            cv2.circle(display_image, (point[0], point[1]), 4, (0, 0, 255), -1)  # Red circles for mouse points

        cv2.imshow("Hand Tracking", display_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg)

        if self.x is not None and self.y is not None:
            depth_values = []
            depth_value = self.latest_depth_image[self.y, self.x]

            if depth_value > 0:
                depth_values.append(depth_value * 0.001)
            
            if depth_values:
                z = np.median(depth_values)
                self.z_buffer.append(z)
                z_smooth = np.mean(self.z_buffer)

                deprojected_point = deproject(self.x, self.y, z_smooth)

                msg2 = Point()
                msg2.x = deprojected_point[0]
                msg2.y = deprojected_point[1]
                msg2.z = deprojected_point[2]

                key = self.stdscr.getch()
                if key == ord('s'):
                    self.publish_points.publish(msg2)
                    # Save the point to be drawn in future frames
                    self.saved_points.append((self.x, self.y))

def main(args=None):
    rclpy.init(args=args)
    node = Model()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()