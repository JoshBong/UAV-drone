#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2 as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#import txt file file_path = poop
#ar_dict = {}
test_dict = {99:(23,20,0)}
'''
with open(file_path, 'r') as file:
                for line in file:
                    parts = line.strip().split(':')
'''

class ARTagsDetectNode(Node):

    br_: CvBridge
    LIBRARY_: cv.aruco.Dictionary
    PARAMETERS_ = cv.aruco.DetectorParameters
    DETECTOR_: cv.aruco.ArucoDetector

    def __init__(self):
        super().__init__("camera_read_ar_tags")

        # Create a subscriber.
        self.create_subscription(Image, "/image_raw", self.receive_image_data, 10)

        # Create the bridge for exporting images.
        self.br_ = CvBridge()

        self.LIBRARY_ = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_100)
        self.PARAMETERS_ = cv.aruco.DetectorParameters()
        self.DETECTOR_ = cv.aruco.ArucoDetector(self.LIBRARY_, self.PARAMETERS_)

        self.get_logger().info("\nSuccessfully launched the camera display module!")

    def receive_image_data(self, msg: Image):

        ar_detect = False
        location = (0,0,0)
        tag_id = -1
        # Convert ROS Image message to OpenCV image.
        img = self.br_.imgmsg_to_cv2(msg)

        # Detect AR tags in the image.
        markerCorners, markerIDs, rejectedCandidates = self.DETECTOR_.detectMarkers(cv.cvtColor(img, cv.COLOR_BGR2GRAY))

        # Log the obtained corners and detected AR tag IDs.
        #self.get_logger().info(f"Obtained corners: {markerCorners}")
        #self.get_logger().info(f"Detected AR Tag Label: {markerIDs}")

        # Check if marker IDs are in the test dictionary and log their locations.
        if markerIDs is not None:
            ar_detect = True
            for markerID in markerIDs:
                tag_id = markerID[0]  # markerID is a 2D array, so get the first element
                if tag_id in test_dict:
                    location = test_dict[tag_id]
                    #self.get_logger().info(f"AR Tag ID {tag_id} is located at: {location}")
                #else:
                #   self.get_logger().info(f"AR Tag ID {tag_id} is at an unknown location")
            self.get_logger().info(f"\nAR Tag detected: {ar_detect} \nAR Tag ID {tag_id} is located at: {location}")
        else:
            self.get_logger().info("\nNo AR Tags detected.")


def main(args=None):
    
    rclpy.init(args=args)

    # Create a node
    node = ARTagsDetectNode()
    rclpy.spin(node)

    # Shutdown the rclpy
    rclpy.shutdown()


if __name__ == "__main__":
    main()
