#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineFollowingNode(Node):

    br_: CvBridge

    def __init__(self):

        super().__init__("line_following")
        self.create_subscription(Image, "/image_raw", self.receive_image_data, 10)
        self.br_ = CvBridge()
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.current_vector = (0.0,0.0)
        self.latest_frame = None
        self.imgray = None
        self.high = None
        self.low = None
        self.dilated= None
        

    def receive_image_data(self, msg: Image):

        # self.low = np.uint([0,0,200])
        # self.high = np.uint([172,111,255])
        
        # Convert ROS Image message to OpenCV image and store it
        img = self.br_.imgmsg_to_cv2(msg)
        kernel = np.ones((3,3), np.float32)/25
        dilated_img = cv2.erode(img, kernel, iterations=1)
        self.imgray = cv2.cvtColor(dilated_img, cv2.COLOR_BGR2HSV)

    def get_mask(self, image, hsv_lower= np.uint([0,0,200]), hsv_upper=np.uint([172,111,255])):
        mask = cv2.inRange(image, hsv_lower, hsv_upper)
        return mask

    def contour_regression(self, contour):
        xs = np.array([float(item[0]) for item in contour])
        ys = np.array([float(item[1]) for item in contour])

        x_mean = np.mean(xs)
        y_mean = np.mean(ys)

        xy_mean = np.mean(xs(ys)) 

        x_squared_mean = np.mean(xs**2) 

        m = (x_mean*y_mean - xy_mean)/(x_mean*2 - x_squared_mean) 

        b = y_mean - m*x_mean 

        return (m,b)

    

    def timer_callback(self, msg: Image):
        mask=self.get_mask(self.imgray)
        contours, hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_NONE)

        if len(contours)==0: 
            return None

        c = max(contours, key=cv2.contourArea)

        # Find center of the largest contour (where the line should intersect)
        M = cv2.moments(c)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])    

        cv2.drawContours(self.imgray, c, -1, (0,255,0), 1)
        cv2.imshow("Mask",mask)
        cv2.imshow("Frame", self.imgray)


        # self.curre

        self.get_logger().info(f"Going to vector: {self.current_vector}")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()