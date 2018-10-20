#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

#OUTLIER FILTER
MEAN_K = 10
THRESHOLD_SCALE_FACTOR = 0.1
#Downsample
LEAF_SIZE = 0.005
#Passthrough
FILTER_AXIS = 'z'
AXIS_MIN = 0.6
AXIS_MAX = 1.1
#RANSAC
MAX_DISTANCE = 0.01
#Clustering
CLUSTER_TOLERANCE = 0.05
MIN_CLUSTER_SIZE = 100
MAX_CLUSTER_SIZE = 100000

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(MEAN_K)
    outlier_filter.set_std_dev_mul_thresh(THRESHOLD_SCALE_FACTOR)
    cloud = outlier_filter.filter()

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud = vox.filter()

    # TODO: PassThrough Filter
    passthrough_z = cloud.make_passthrough_filter()
    passthrough_z.set_filter_field_name(FILTER_AXIS)
    passthrough_z.set_filter_limits(AXIS_MIN, AXIS_MAX)
    cloud = passthrough_z.filter()

    passthrough_x = cloud.make_passthrough_filter()
    passthrough_x.set_filter_field_name('x')
    passthrough_x.set_filter_limits(0.3, 2.0)
    cloud = passthrough_x.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(MAX_DISTANCE)

    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = cloud.extract(inliers, negative=False)
    cloud_objects = cloud.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(CLUSTER_TOLERANCE)
    ec.set_MinClusterSize(MIN_CLUSTER_SIZE)
    ec.set_MaxClusterSize(MAX_CLUSTER_SIZE)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                            white_cloud[indice][1],
                                            white_cloud[indice][2],
                                            rgb_to_float(cluster_color[j])])

    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_cluster_pub.publish(ros_cluster_cloud)

    # Exercise-3 TODOs:

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)

        # Compute the associated feature vector
        ros_cluster = pcl_to_ros(pcl_cluster)
        color_hist = compute_color_histograms(ros_cluster, using_hsv=True)
        normals = get_normals(ros_cluster)
        norm_hist = compute_normal_histograms(normals)
        feature = np.concatenate((color_hist, norm_hist))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

        rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)


    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
        # pr2_mover(detected_objects_list)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # TODO: Initialize variables
    labels = []
    centroids = []
    test_scene_num = Int32()
    test_scene_num.data = 2
    arm_name = String()
    pick_pose = Pose()
    place_pose = Pose()

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    place_pose_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    pick_list = []
    for i in range(0, len(object_list_param)):
        object_name = String()
        object_name.data = object_list_param[i]['name']
        object_group = object_list_param[i]['group']
        pick_list.append((object_name, object_group))

    for object in object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    yaml_dict_list = []
    # TODO: Loop through the pick list

    for object_name, object_group in pick_list:
        object_index = 0
        if object_name.data in labels:
            object_index = labels.index(object_name.data)

            # TODO: Get the PointCloud for a given object and obtain it's centroid
            pick_pose.position.x = np.asscalar(centroids[object_index][0])
            pick_pose.position.y = np.asscalar(centroids[object_index][1])
            pick_pose.position.z = np.asscalar(centroids[object_index][2])

            # TODO: Create 'place_pose' for the object
            dropbox = 0 if object_group == 'red' else 1
            place_pose.position.x = place_pose_param[dropbox]['position'][0]
            place_pose.position.y = place_pose_param[dropbox]['position'][1]
            place_pose.position.z = place_pose_param[dropbox]['position'][2]


            # TODO: Assign the arm to be used for pick_place
            arm_name.data = 'left' if object_group == 'red' else 'right'

            # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            yaml_dict_list.append(yaml_dict)

            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            # try:
            #     pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
            #
            #     # TODO: Insert your message variables to be sent as a service request
            #     resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
            #
            #     print ("Response: ", resp.success)
            #
            # except rospy.ServiceException, e:
            #     print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    send_to_yaml('output_2.yaml', yaml_dict_list)


if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscribers
    # pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)


    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']


    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
