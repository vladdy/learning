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


#  Get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster


# Create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"] = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict


# Write yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


# Callback for your Point Cloud Subscriber
def pcl_callback(pcl_msg):
    pcl_data = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    vox_filtered = vox.filter()

    # PassThrough Filter
    passthrough_z = vox_filtered.make_passthrough_filter()
    filter_axis = 'z'
    axis_min = 0.5
    axis_max = 1.1
    passthrough_z.set_filter_field_name(filter_axis)
    passthrough_z.set_filter_limits(axis_min, axis_max)
    passthrough_z_filtered = passthrough_z.filter()

    passthrough_x = passthrough_z_filtered.make_passthrough_filter()
    filter_axis = 'y'
    axis_min = -0.5
    axis_max = 0.5
    passthrough_x.set_filter_field_name(filter_axis)
    passthrough_x.set_filter_limits(axis_min, axis_max)
    passthrough_filtered = passthrough_x.filter()

    # RANSAC Plane Segmentation
    seg = passthrough_filtered.make_segmenter()
    max_distance = 0.03
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(max_distance)

    # Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = passthrough_filtered.extract(inliers, negative=False)
    cloud_objects = passthrough_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    xyz_cloud = XYZRGB_to_XYZ(cloud_objects)
    kd_tree = xyz_cloud.make_kdtree()
    ec = xyz_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(50)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(kd_tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([xyz_cloud[indice][0],
                                             xyz_cloud[indice][1],
                                             xyz_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    # Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_clusters = pcl_to_ros(cluster_cloud)

    # Publish ROS messages
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_clusters_pub.publish(ros_cloud_clusters)

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster from the extracted outliers (cloud_objects)
        pcl_cluster = cloud_objects.extract(pts_list)
        ros_cluster = pcl_to_ros(pcl_cluster)

        # Extract histogram features similarly to capture_features.py
        chists = compute_color_histograms(ros_cluster, using_hsv=False)
        normals = get_normals(ros_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result
        # and add it to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(xyz_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cluster
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    # This is the output you'll need to complete the upcoming project!
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass


# function to load parameters and request PickPlace service
def pr2_mover(detected_object_list):
    # Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # Initialize variables
    test_scene_num = Int32()
    object_name = String()
    arm_name = String()
    place_pose = Pose()
    pick_pose = Pose()

    # Parse parameters into individual variables
    if dropbox_param[0]['group'] == 'green':
        green_position = dropbox_param[0]['position']
        green_arm = dropbox_param[0]['name']
        red_position = dropbox_param[1]['position']
        red_arm = dropbox_param[1]['name']
    else:
        red_position = dropbox_param[0]['position']
        red_arm = dropbox_param[0]['name']
        green_position = dropbox_param[1]['position']
        green_arm = dropbox_param[1]['name']

    # TODO: Rotate PR2 in place to capture side tables for the collision map
    left_angle = Float64()
    left_angle.data = 90.0 / 180 * np.pi
    world_joint_pub.publish(left_angle)

    right_angle = Float64()
    right_angle.data = -90.0 / 180 * np.pi
    world_joint_pub.publish(right_angle)

    original_angle = Float64()
    original_angle.data = 0
    world_joint_pub.publish(original_angle)

    labels = []
    centroids = []
    for object in detected_object_list:
        labels.append(object.label)
        points_arr = ros_to_pcl(object.cloud).to_array()
        centroids.append(np.mean(points_arr, axis=0)[:3])

    # Loop through the pick list
    dict_list = []
    for i, object_param in enumerate(object_list_param):
        object_to_find = object_param['name']
        if object_to_find in labels:
            # Get the PointCloud for a given object and obtain it's centroid
            object_index = labels.index(object_to_find)
            object_centroids = centroids[object_index]

            test_scene_num.data = 1  # Change according to world

            object_name.data = object_param['name']

            pick_pose.position.x = np.asscalar(object_centroids[0])
            pick_pose.position.y = np.asscalar(object_centroids[1])
            pick_pose.position.z = np.asscalar(object_centroids[2])
            pick_pose.orientation.w = 1

            object_group = object_param['group']

            # Create 'place_pose' for the object
            # Assign the arm to be used for pick_place
            # TODO: Add random offsets in the x and y directions (cater to multiple objects in same dropbox)
            if object_group == 'green':
                arm_name.data = green_arm
                place_pose.position.x = green_position[0]
                place_pose.position.y = green_position[1]
                place_pose.position.z = green_position[2]
                place_pose.orientation.w = 1
            else:
                arm_name.data = red_arm
                place_pose.position.x = red_position[0]
                place_pose.position.y = red_position[1]
                place_pose.position.z = red_position[2]
                place_pose.orientation.w = 1

            # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
            yaml_msg = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
            dict_list.append(yaml_msg)

            # Wait for 'pick_place_routine' service to come up
            rospy.wait_for_service('pick_place_routine')

            try:
                pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

                # Insert message variables to be sent as a service request
                resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

                print("Response: ", resp.success)

            except rospy.ServiceException, e:
                print "Service call failed: %s" % e

        else:
            print object_param['name'], "not found in perception"

    # Output request parameters into output yaml file
    send_to_yaml("output_1_test.yaml", dict_list)  # Change according to world


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node("object_recognition", anonymous=True)

    # Create Subscribers
    point_cloud_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback)

    # Create Publishers
    pcl_objects_pub = rospy.Publisher("pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("pcl_clusters", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("detected_objects", DetectedObjectsArray, queue_size=1)
    world_joint_pub = rospy.Publisher("/pr2/world_joint_controller/command", Float64,
                                      queue_size=3)  # To rotate PR2 for collision analysis
    map_3d_pub = rospy.Publisher("/pr2/3d_map/points", PointCloud2,
                                 queue_size=1)  # To publish surrounding collisions for motion planning

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()