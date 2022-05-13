#!/usr/bin/env python

from __future__ import print_function
# import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

import numpy as np
import argparse
import imutils
import time

IMAGE_WIDTH=1241
IMAGE_HEIGHT=376

# def compute_semantics():


class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/segmentation_output",Image)
    self.image_pub2 = rospy.Publisher("/segmentation_visual",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/kitti/camera_color_left/image_raw",Image,self.callback)

    # initialization for semantics segmentation
    classed_path = "/home/yasaburo3/life_long/deremove_ws/src/semantic_seg/enet-cityscapes/enet-classes.txt"
    model_path = "/home/yasaburo3/life_long/deremove_ws/src/semantic_seg/enet-cityscapes/enet-model.net"
    color_path = "/home/yasaburo3/life_long/deremove_ws/src/semantic_seg/enet-cityscapes/enet-colors.txt"

    # load the class label names
    CLASSES = open(classed_path).read().strip().split("\n")

    self.COLORS = open(color_path).read().strip().split("\n")
    self.COLORS = [np.array(c.split(",")).astype("int") for c in self.COLORS]
    self.COLORS = np.array(self.COLORS, dtype="uint8")

    # initialize the legend visualization
    legend = np.zeros(((len(CLASSES) * 25) + 25, 300, 3), dtype="uint8")

    # loop over the class names + colors
    for (i, (className, color)) in enumerate(zip(CLASSES, self.COLORS)):
        # draw the class name + color on the legend
        color = [int(c) for c in color]
        cv2.putText(legend, className, (5, (i * 25) + 17),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.rectangle(legend, (100, (i * 25)), (300, (i * 25) + 25),
            tuple(color), -1)

    # load our serialized model from disk
    print("[INFO] loading model...")
    self.net = cv2.dnn.readNet(model_path)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # time = data.header.stamp
    # print(data.header.stamp)
    # exit()
    

    (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)

    # image = cv_image            ###
    blob = cv2.dnn.blobFromImage(cv_image, 1 / 255.0, (1024, 512), 0,
	    swapRB=True, crop=False)

    # perform a forward pass using the segmentation model
    self.net.setInput(blob)
    start = time.time()
    output = self.net.forward()
    end = time.time()
    print("[INFO] inference took {:.4f} seconds".format(end - start))

    (numClasses, height, width) = output.shape[1:4]
    classMap = np.argmax(output[0], axis=0)
    mask = self.COLORS[classMap]

    mask = cv2.resize(mask, (cv_image.shape[1], cv_image.shape[0]),
	interpolation=cv2.INTER_NEAREST)
    classMap = cv2.resize(classMap, (cv_image.shape[1], cv_image.shape[0]),
	    interpolation=cv2.INTER_NEAREST)

    output = ((0.4 * cv_image) + (0.6 * mask)).astype("uint8")

    # cv2.imshow("Image window", output)
    # cv2.waitKey(3)
    
    dst = Image()
    header = Header(stamp=data.header.stamp)
    dst.height = IMAGE_HEIGHT
    dst.width = IMAGE_WIDTH
    dst.encoding = 'mono8'
    dst.header = header
    dst.step = 1241*1
    mask = cv2.cvtColor(mask, cv2.COLOR_RGB2GRAY)
    dst.data = np.array(mask).tostring()
    

    try:
        self.image_pub.publish(dst)
        self.image_pub2.publish(self.bridge.cv2_to_imgmsg(output, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)