#!/usr/bin/env python

# Import modules
from pcl_helper import *
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import rospy

# TODO: Define functions as required
# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # TODO: Convert ROS msg to PCL data
    pcl_data = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = pcl_data.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    vox_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = vox_filtered.make_passthrough_filter()
    filter_axis = 'z'
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_field_name(filter_axis)
    passthrough.set_filter_limits(axis_min, axis_max)
    passthrough_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = passthrough_filtered.make_segmenter()
    max_distance = 0.01
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    cloud_table = passthrough_filtered.extract(inliers, negative=False)
    cloud_objects = passthrough_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    xyz_cloud = XYZRGB_to_XYZ(cloud_objects)
    kd_tree = xyz_cloud.make_kdtree()
    ec = xyz_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(kd_tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([xyz_cloud[indice][0],
                                            xyz_cloud[indice][1],
                                            xyz_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_clusters = pcl_to_ros(cluster_cloud)

    # TODO: Publish ROS messages
    pcl_table_pub.publish(ros_cloud_table)
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_clusters_pub.publish(ros_cloud_clusters)


if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node("clustering", anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber("/sensor_stick/point_cloud", pc2.PointCloud2, pcl_callback, queue_size=1)

    # TODO: Create Publishers
    pcl_objects_pub = rospy.Publisher("pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("pcl_table", PointCloud2, queue_size=1)
    pcl_clusters_pub = rospy.Publisher("pcl_clusters", PointCloud2, queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()