#!/usr/bin/env python
# -*- coding: utf-8 -*-
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************
# Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

# import tf
# import rospy
# from unittest import TestCase


# class TestRGBDLaunch(TestCase):
#     def testKinectFrames(self):
#         tfListener = tf.TransformListener()
#         try:
#             tfListener.waitForTransform("camera_depth_optical_frame",
#                                         "camera_rgb_optical_frame",
#                                         rospy.Time(), rospy.Duration(20))
#         except Exception as e:
#             self.fail(str(e))
#         try:
#             trans = tfListener.lookupTransform("camera_depth_optical_frame",
#                                                "camera_rgb_optical_frame",
#                                                rospy.Time())
#             self.assertIsNotNone(trans)
#         except Exception as e:
#             self.fail(str(e))

# if __name__ == '__main__':
#     rospy.init_node("test_kinect_frames")
#     import rostest
#     rostest.rosrun("rgbd_launch", "test_kinect_frames", TestRGBDLaunch)
import rclpy
import tf2_ros
from unittest import TestCase

class TestRGBDLaunch(TestCase):
    @classmethod
    def setUp(cls):
        cls.context = rclpy.context.Context()
        rclpy.init(context=cls.context)
        cls.node = rclpy.create_node(
            'test_foxy_frames',
            context=cls.context
        )
    
    @classmethod
    def tearDown(cls):
        cls.node.destroy_node()
        rclpy.shutdown(context=cls.context)
    
    def testFoxyFrames(self):
        tfBuffer = tf2_ros.Buffer()
        tfListener = tf2_ros.TransformListener(tfBuffer)
        try:
            tfBuffer.waitForTransform(camera_depth_optical_frame,
                                      camera_rgb_optical_frame,
                                      rclpy.Time(),
                                      rclpy.Duration(20))
        except Exception as e:
            self.fail(str(e))
        try:
            trans = tfBuffer.lookupTransform(camera_depth_optical_frame,
                                     camera_rgb_optical_frame,
                                     rclpy.Time())
            self.assertIsNotNone(trans)
        except Exception as e:
            self.fail(str(e))

if __name__ == '__main__':
    unittest.main()