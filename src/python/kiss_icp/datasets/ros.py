# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
import importlib
import os
from pathlib import Path
import sys
from typing import List, Optional

import rospy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry

from kiss_icp.pipeline import OdometryPipeline

import numpy as np
import yaml


class Ros_node:
    def __init__(
        self,
        topic: str,
        config: Path,
        deskew: bool = False,
        max_range: Optional[float] = None,
        visualize: bool = False,
        *_,
        **__):

        self.topic = topic
        self.node=rospy.init_node('kissicpnode', anonymous=True)
        self.pc2_subscriber = rospy.Subscriber(self.topic, PointCloud2, self.callback)
        self.odom_publisher = rospy.Publisher('/odom_test', Odometry, queue_size=10)
        self.pc2 = importlib.import_module("sensor_msgs.point_cloud2")
        
        self.odometry_pipeline = OdometryPipeline(
            dataset=None,
            config=config,
            deskew=deskew,
            max_range=max_range,
            visualize=visualize,
            ros=True
        )

    def callback(self, msg):
        self.data = msg
        dataframe=self.read_point_cloud(self.topic)
        pose = self.odometry_pipeline.run_once(dataframe)
        # TODO: convert pose to odometry message and publish
        #odometry = Odometry()
        #self.odom_publisher.publish(odometry)
        
        #print(self.odometry_pipeline.poses[-1])

    def run(self):
        rospy.spin()

    def read_point_cloud(self, topic: str):
        # TODO: implemnt [idx], expose field_names
        msg = self.data
        points = np.array(list(self.pc2.read_points(msg, field_names=["x", "y", "z"])))

        t_field = None
        for field in msg.fields:
            if field.name in ["t", "timestamp", "time"]:
                t_field = field.name
        timestamps = np.ones(points.shape[0])
        if t_field:
            timestamps = np.array(list(self.pc2.read_points(msg, field_names=t_field)))
            timestamps = timestamps / np.max(timestamps) if t_field != "time" else timestamps

        return points.astype(np.float64), timestamps#, msg.header
