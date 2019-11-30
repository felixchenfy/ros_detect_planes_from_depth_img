#!/usr/bin/env python
# -*- coding: utf-8 -*-

from ros_detect_planes_from_depth_img.msg import PlanesResults
from plane_detector import PlaneDetector, PlaneParam
from utils.lib_ros_rgbd_pub_and_sub import DepthImageSubscriber, ColorImageSubscriber, ColorImagePublisher

import rospy

import cv2
import numpy as np
import argparse
import yaml
import os
import sys


def parse_command_line_argumetns():

    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)

    parser.add_argument("-c", "--config_file", required=False,
                        default='config/plane_detector_config.yaml',
                        help="Path to the plane detecton configuration file. ")

    parser.add_argument("-d", "--depth_topic", required=True,
                        help="Topic name of depth image (no distortion).")

    parser.add_argument("-m", "--camera_info", required=True,
                        help="Path to camera info file. "
                        "The distortion must be zero."
                        "Depth and color images must share the same parameters.")

    parser.add_argument("-i", "--color_topic", required=False,
                        default="",
                        help="Topic name of color image (no distortion). "
                        "This topic can be empty, "
                        "because color image is only for visualization purpose."
                        "If empty, a black image will be used instead.")

    args = parser.parse_args(rospy.myargv()[1:])

    return args


def read_config_file(config_file_path):
    if not os.path.exists(config_file_path):
        raise RuntimeError("Config file doesn't exist: " + config_file_path)
    rospy.loginfo("Read config from: " + config_file_path)

    def read_yaml_file(file_path):
        with open(file_path, 'r') as stream:
            data = yaml.safe_load(stream)
        return data
    config = read_yaml_file(config_file_path)
    return config


class PlaneResultsPublisher(object):
    def __init__(self, topic_name, queue_size=10):
        self._pub = rospy.Publisher(
            topic_name, PlanesResults, queue_size=queue_size)

    def publish(self, plane_params):
        '''
        Arguments:
            plane_params {list of PlaneParam}
        '''
        res = PlanesResults()
        res.N = len(plane_params)
        for pp in plane_params:
            res.norms.extend(pp.w.tolist())
            res.center_3d.extend(pp.pts_3d_center.tolist())
            res.center_2d.extend(pp.pts_2d_center.tolist())
            res.mask_color.extend(pp.mask_color.tolist())
        self._pub.publish(res)
        return


def main(args):

    # -- Plane detector.
    detector = PlaneDetector(args.config_file, args.camera_info)

    # -- Set up color/depth image subscribers.
    sub_depth = DepthImageSubscriber(args.depth_topic)
    if args.color_topic:
        sub_color = ColorImageSubscriber(args.color_topic)
    else:
        sub_color = None

    # -- Set up mask/viz/results publishers.
    config = read_config_file(args.config_file)
    pub_colored_mask = ColorImagePublisher(config["topic_colored_mask"])
    pub_image_viz = ColorImagePublisher(config["topic_image_viz"])
    pub_results = PlaneResultsPublisher(config["topic_result"])

    # -- Wait for subscribing image and detect.
    while not rospy.is_shutdown():
        if sub_depth.has_image():

            # -- Read next color and depth image.
            depth = sub_depth.get_image()
            if sub_color is not None:
                while not sub_color.has_image():
                    rospy.sleep(0.005)
                color = sub_color.get_image()
            else:
                color = None

            # -- Detect plane.
            rospy.loginfo("=================================================")
            rospy.loginfo("Received an image. Start plane detection.")

            list_plane_params, colored_mask, img_viz = detector.detect_planes(
                depth, color)

            # -- Print result.
            for i, plane_param in enumerate(list_plane_params):
                plane_param.print_params(index=i+1)

            # -- Publish result.
            pub_colored_mask.publish(colored_mask)
            pub_image_viz.publish(img_viz)
            pub_results.publish(list_plane_params)
            rospy.loginfo("Publish results completes.")
            rospy.loginfo("-------------------------------------------------")
            rospy.loginfo("")


if __name__ == '__main__':
    node_name = "plane_detector_server"
    rospy.init_node(node_name)
    args = parse_command_line_argumetns()
    main(args)
    rospy.logwarn("Node `{}` stops.".format(node_name))
