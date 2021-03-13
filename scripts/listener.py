#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32
import numpy as np

def callback(data):
    pub = rospy.Publisher('min_depth', Float32, queue_size=10)
    try:
        br = CvBridge()
        depth_image = br.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except CvBridgeError, e:
        print e
    height, width = depth_image.shape
    x = int(0.5 * height)
    y = int(0.5 * width)
    croped_image = depth_image[x-10:x+10, y-10:y+10]
    filtered_image = croped_image[croped_image > 0]
    if filtered_image.size != 0:
        max_data = np.amax(filtered_image)
        min_data = np.amin(filtered_image)
        pub.publish(min_data)
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s %s', max_data, min_data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    #rospy.init_node('talker', anonymous=True)
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
