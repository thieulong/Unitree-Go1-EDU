#!/usr/bin/env python

import os
import sys
import csv
import torch
import numpy as np
import scipy.io
import cv2
import rospy
from sensor_msgs.msg import Image
import std_msgs.msg
import torchvision.transforms
from mit_semseg.models import ModelBuilder, SegmentationModule
from mit_semseg.utils import colorEncode

class SemanticSegmentationNode:
    def __init__(self):
        rospy.init_node('semantic_segmentation_node')

        self.color_pub = rospy.Publisher('/color_output', Image, queue_size=10)
        self.grayscale_pub = rospy.Publisher('/grayscale_output', Image, queue_size=10)
        self.preprocessed_pub = rospy.Publisher('/preprocessed_image', Image, queue_size=10)

        colors_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'data/color150.mat')
        self.colors = scipy.io.loadmat(colors_file)['colors']
        self.grouped_class_indices = [3, 6, 9, 11, 13, 28, 46, 54]

        encoder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ckpt/ade20k-resnet50dilated-ppm_deepsup/encoder_epoch_20.pth')
        decoder_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ckpt/ade20k-resnet50dilated-ppm_deepsup/decoder_epoch_20.pth')
        self.net_encoder = ModelBuilder.build_encoder(arch='resnet50dilated', fc_dim=2048, weights=encoder_path)
        self.net_decoder = ModelBuilder.build_decoder(arch='ppm_deepsup', fc_dim=2048, num_class=150, weights=decoder_path, use_softmax=True)
        self.crit = torch.nn.NLLLoss(ignore_index=-1)
        self.segmentation_module = SegmentationModule(self.net_encoder, self.net_decoder, self.crit)
        self.segmentation_module.eval()

        rospy.Subscriber("/camera/front/left/image_rect_color", Image, self.image_callback)

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("Unsupported image encoding: {}".format(img_msg.encoding))
            return None
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def cv2_to_imgmsg(self, cv_image, encoding):
        image = Image()
        image.height = cv_image.shape[0]
        image.width = cv_image.shape[1]
        image.encoding = encoding
        image.is_bigendian = 0
        image.step = cv_image.shape[1] * cv_image.shape[2]
        image.data = cv_image.tobytes()
        return image

    def image_callback(self, msg):
        cv_image = self.imgmsg_to_cv2(msg)
        if cv_image is None:
            return

        img_data = torchvision.transforms.ToTensor()(cv_image)[None].numpy()

        with torch.no_grad():
            scores = self.segmentation_module({'img_data': img_data}, segSize=img_data.shape[2:])

        _, pred = torch.max(scores, dim=1)
        pred = pred.cpu().numpy()

        pred_filtered = np.zeros_like(pred)
        for index in self.grouped_class_indices:
            pred_filtered[pred == index] = 1

        # Uncomment the following line if you want to publish the preprocessed image
        # preprocessed_image_msg = self.cv2_to_imgmsg(cv_image, encoding="bgr8")
        # self.preprocessed_pub.publish(preprocessed_image_msg)

        color_segmented = colorEncode(scores, self.colors)
        color_segmented_image_msg = self.cv2_to_imgmsg(color_segmented, encoding="bgr8")
        self.color_pub.publish(color_segmented_image_msg)

        grayscale_segmented_image_msg = self.cv2_to_imgmsg(pred_filtered, encoding="mono8")
        self.grayscale_pub.publish(grayscale_segmented_image_msg)

if __name__ == "__main__":
    segmentation_node = SemanticSegmentationNode()
    rospy.spin()
