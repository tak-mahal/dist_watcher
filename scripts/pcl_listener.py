#! /usr/bin/env python

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import rospy
import time
import math

def callback_pointcloud(data):
    assert isinstance(data, PointCloud2)
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    time.sleep(1)
    #print type(gen)
    min_dist = None
    for p in gen:
        dist = math.sqrt(float(p[0])**2 + float(p[1])**2 + float(p[2])**2)
        if min_dist == None or dist < min_dist:
            min_dist = dist
            #print " x : %.3f  y: %.3f  z: %.3f" %(p[0],p[1],p[2])
    rospy.loginfo(min_dist)

def main():
    rospy.init_node('pcl_listener', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, callback_pointcloud)
    rospy.spin()

if __name__ == "__main__":
    main()

