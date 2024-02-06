#!/usr/bin/env python

import os
import sys
import torch
import numpy as np
import scipy.io
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
import torchvision.transforms
from mit_semseg.models import ModelBuilder, SegmentationModule
from mit_semseg.utils import colorEncode

class SemanticSegmentationNode:
    def __init__(self):
        rospy.init_node('semantic_segmentation_node')

        self.grayscale_topic = "/grayscale_output"
        self.grayscale_pub = rospy.Publisher(self.grayscale_topic, Image, queue_size=10)
        self.grayscale_array_topic = "/grayscale_array"
        self.grayscale_array_pub = rospy.Publisher(self.grayscale_array_topic, UInt8MultiArray, queue_size=10)
        self.preprocess_topic = "/preprocess_image"
        self.preprocess_pub = rospy.Publisher(self.preprocess_topic, Image, queue_size=10)
        self.camera_sub = "/camera/front/left/image_rect_color"
        rospy.Subscriber(self.camera_sub, Image, self.image_callback)

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

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "bgr8":
            print('Invalid image type, expect "bgr8" but received {}'.format(img_msg.encoding))
        dtype = np.dtype("uint8")
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), dtype=dtype, buffer=img_msg.data)

        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv
    
    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        image = Image()
        image.height = cv_image.shape[0]
        image.width = cv_image.shape[1]
        image.encoding = encoding 
        image.is_bigendian = 0
        image.step = len(image.data) // image.height
        image.data = cv_image.tostring()
        return image

    def image_callback(self, msg):
        cv_image = self.imgmsg_to_cv2(msg)
        if not cv_image:
            print('Error while retrieving image from "{}"'.format(self.camera_sub))
        else:
            print('Successful retrieving image from "{}"'.format(self.camera_sub))

        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        if not rgb_image:
            print('Raw image converted successfully')
        else:
            print('Error while converting raw image (BGR) to (RBG) format')
        
        preprocess_image_msg = self.cv2_to_imgmsg(cv_image)
        self.preprocessed_pub.publish(preprocess_image_msg)
        print('Preprocessed image has been published at "{}"'.format(self.preprocess_topic))

        print("Transforming image to Pytorch Tensor ...")
        img_data = torchvision.transforms.ToTensor()(rgb_image)
        singleton_batch = {'img_data': img_data[None]}
        output_size = img_data.shape[1:]

        with torch.no_grad():
            scores = self.segmentation_module(singleton_batch, segSize=output_size)

        if scores:
            print("Image transform finished successfully")
        else:
            print("Image transform failed")

        _, pred = torch.max(scores, dim=1)
        pred = pred.cpu()[0].numpy()
        pred_filtered = np.zeros_like(pred)
        for index in self.grouped_class_indices:
            pred_filtered[pred == index] = 1

        print("Semantic Segmentation Grayscale:")
        print(pred_filtered)

        pred_filtered_msg = UInt8MultiArray(data=pred_filtered.flatten().tolist())
        self.grayscale_array_pub.publish(pred_filtered_msg)

        print("Grayscale array published to {}".format(self.grayscale_array_topic))

        pred_filtered_image = (pred_filtered * 255).astype(np.uint8)
        grayscale_output_msg = self.cv2_to_imgmsg(pred_filtered_image, encoding="mono8")
        
        self.grayscale_pub.publish(grayscale_output_msg)
        print('Grayscale output image has been published at "{}"'.format(self.grayscale_topic))

if __name__ == "__main__":
    segmentation_node = SemanticSegmentationNode()
    rospy.spin()
