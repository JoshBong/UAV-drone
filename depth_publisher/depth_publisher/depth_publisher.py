#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import depthai as dai
import numpy as np
import math
import time

class HostSpatialsCalc:
    def __init__(self, device):
        self.calibData = device.readCalibration()
        self.DELTA = 5
        self.THRESH_LOW = 200
        self.THRESH_HIGH = 30000

    def setLowerThreshold(self, threshold_low):
        self.THRESH_LOW = threshold_low

    def setUpperThreshold(self, threshold_high):
        self.THRESH_HIGH = threshold_high

    def setDeltaRoi(self, delta):
        self.DELTA = delta

    def _check_input(self, roi, frame):
        if len(roi) == 4: return roi
        if len(roi) != 2: raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
        self.DELTA = 5
        x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
        y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
        return (x-self.DELTA, y-self.DELTA, x+self.DELTA, y+self.DELTA)

    def _calc_angle(self, frame, offset, HFOV):
        return math.atan(math.tan(HFOV / 2.0) * offset / (frame.shape[1] / 2.0))

    def calc_spatials(self, depthData, roi, averaging_method=np.mean):
        depthFrame = depthData.getFrame()
        roi = self._check_input(roi, depthFrame)
        xmin, ymin, xmax, ymax = roi
        depthROI = depthFrame[ymin:ymax, xmin:xmax]
        inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)
        HFOV = np.deg2rad(self.calibData.getFov(dai.CameraBoardSocket(depthData.getInstanceNum()), useSpec=False))
        averageDepth = averaging_method(depthROI[inRange])
        centroid = {
            'x': int((xmax + xmin) / 2),
            'y': int((ymax + ymin) / 2)
        }
        midW = int(depthFrame.shape[1] / 2)
        midH = int(depthFrame.shape[0] / 2)
        bb_x_pos = centroid['x'] - midW
        bb_y_pos = centroid['y'] - midH
        angle_x = self._calc_angle(depthFrame, bb_x_pos, HFOV)
        angle_y = self._calc_angle(depthFrame, bb_y_pos, HFOV)
        spatials = {
            'z': averageDepth,
            'x': averageDepth * math.tan(angle_x),
            'y': -averageDepth * math.tan(angle_y)
        }
        return spatials, centroid

class DepthPublisher(Node):
    def __init__(self):
        super().__init__('depth_publisher')
        self.publisher_ = self.create_publisher(Float32, 'distance', 10)
        self.create_timer(0.1, self.timer_callback)
        self.pipeline = dai.Pipeline()
        self.monoLeft = self.pipeline.create(dai.node.MonoCamera)
        self.monoRight = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        self.monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self.monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        self.stereo.initialConfig.setConfidenceThreshold(255)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setSubpixel(False)
        self.monoLeft.out.link(self.stereo.left)
        self.monoRight.out.link(self.stereo.right)
        self.xoutDepth = self.pipeline.create(dai.node.XLinkOut)
        self.xoutDepth.setStreamName("depth")
        self.stereo.depth.link(self.xoutDepth.input)
        self.device = dai.Device(self.pipeline)
        self.depthQueue = self.device.getOutputQueue(name="depth")
        self.hostSpatials = HostSpatialsCalc(self.device)
        self.y = 200
        self.x = 300
        self.delta = 5
        self.hostSpatials.setDeltaRoi(self.delta)

    def timer_callback(self):
        depthData = self.depthQueue.get()
        spatials, centroid = self.hostSpatials.calc_spatials(depthData, (self.x, self.y))
        distance = spatials['z'] / 1000
        self.get_logger().info(f"Distance: {distance:.2f}m")
        msg = Float32()
        msg.data = distance
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    depth_publisher = DepthPublisher()
    try:
        rclpy.spin(depth_publisher)
    except KeyboardInterrupt:
        pass
    depth_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()