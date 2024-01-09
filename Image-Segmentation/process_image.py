import os
import csv
import torch
import numpy as np
import scipy.io
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import std_msgs.msg
import torchvision.transforms
from mit_semseg.models import ModelBuilder, SegmentationModule
from mit_semseg.utils import colorEncode

# ROS initialization
rospy.init_node('semantic_segmentation_node')
bridge = CvBridge()
color_pub = rospy.Publisher('/color_output', Image, queue_size=1)
grayscale_pub = rospy.Publisher('/grayscale_output', Image, queue_size=1)

# Load the color map and class names
colors = scipy.io.loadmat('data/color150.mat')['colors']
names = {}
with open('data/object150_info.csv') as f:
    reader = csv.reader(f)
    next(reader)
    for row in reader:
        names[int(row[0])] = row[5].split(";")[0]

# Network Builders
net_encoder = ModelBuilder.build_encoder(
    arch='resnet50dilated',
    fc_dim=2048,
    weights='ckpt/ade20k-resnet50dilated-ppm_deepsup/encoder_epoch_20.pth')
net_decoder = ModelBuilder.build_decoder(
    arch='ppm_deepsup',
    fc_dim=2048,
    num_class=150,
    weights='ckpt/ade20k-resnet50dilated-ppm_deepsup/decoder_epoch_20.pth',
    use_softmax=True)

crit = torch.nn.NLLLoss(ignore_index=-1)
segmentation_module = SegmentationModule(net_encoder, net_decoder, crit)
segmentation_module.eval()
segmentation_module.cuda()

grouped_class_indices = [3, 6, 9, 11, 13, 28, 46, 54]

def image_callback(msg):
    # Convert ROS image message to OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Apply transformations
    img_data = torchvision.transforms.ToTensor()(cv_image)
    singleton_batch = {'img_data': img_data[None].cuda()}
    output_size = img_data.shape[1:]

    # Run the segmentation at the highest resolution.
    with torch.no_grad():
        scores = segmentation_module(singleton_batch, segSize=output_size)

    # Get the predicted scores for each pixel
    _, pred = torch.max(scores, dim=1)[3, 9, 11, 13, 28, 46, 54]
    pred = pred.cpu()[0].numpy()

    # Visualize the result for the grouped classes in color
    color_segmented = colorEncode(pred, colors).astype(np.uint8)
    color_segmented_image_msg = bridge.cv2_to_imgmsg(color_segmented, encoding="bgr8")
    color_pub.publish(color_segmented_image_msg)

    # Visualize the result for the grouped classes in grayscale
    grayscale_segmented = pred.copy()
    grayscale_segmented[np.isin(grayscale_segmented, grouped_class_indices, invert=True)] = 0
    grayscale_segmented[grayscale_segmented > 0] = 1
    grayscale_segmented_image_msg = bridge.cv2_to_imgmsg(grayscale_segmented.astype(np.uint8), encoding="mono8")
    grayscale_pub.publish(grayscale_segmented_image_msg)

# Subscribe to the raw_image topic
rospy.Subscriber("/raw_image", Image, image_callback)

# Spin to keep the node alive
rospy.spin()
