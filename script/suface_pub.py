#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
import threading
import time
import numpy as np

from sensor_msgs.msg import PointCloud2,PointField



class MarkerPub(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        rospy.init_node('marker_pub')
        self.map_pub = rospy.Publisher("map_pub", PointCloud2, queue_size = 10)
        self.raw_pub = rospy.Publisher("raw_pub", PointCloud2, queue_size = 10)


    def run(self):
        rospy.spin()

    def pub_data(self, points, intensity, data_type = 'map'):
        #msg = self.surface_to_pcl(data,.1)

        msg = PointCloud2()
        buf = []
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.height = points.shape[0]
        msg.width = 1
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)]

        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16*msg.height
        msg.is_dense = True
        if(intensity is None):
            points = np.hstack([points, np.zeros([points.shape[0], 1])])
        else:
            points = np.hstack([points,intensity.reshape([-1,1])])
        msg.data = np.asarray(points, np.float32).tostring()
        if(data_type == 'map'):
            self.map_pub.publish(msg)
        else:
            self.raw_pub.publish(msg)



    def surface_to_pcl(self, surface, resolution):
        msg = PointCloud2()
        buf = []
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.height = surface.shape[0] * surface.shape[1]
        msg.width = 1
        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]

        x, y = np.meshgrid(range(surface.shape[0]),range(surface.shape[1]))
        points = np.dstack([x*resolution,y*resolution,surface])
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12*msg.height
        msg.is_dense = True
        msg.data = np.asarray(points, np.float32).tostring()
        return msg

def main():
    mp = MarkerPub()
    mp.start()
    for i in range(1000):
        time.sleep(0.05)
        mp.pub_data()
        print('pub')


if __name__ == '__main__':
    main()

