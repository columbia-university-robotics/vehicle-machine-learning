//
// Created by neil on 9/15/20.
//


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/console.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointXYZ PointT;

ros::Publisher pub;
ros::Publisher position_pub;

float distance_threshold;       //: 0.25    (done)


void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input){

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr outputCloud(new pcl::PointCloud<PointT>);
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);

    // Compute the normals
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    ne.setInputCloud (outputCloud);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::search::KdTree<PointT>::Ptr tree_n (new pcl::search::KdTree<PointT>());
    ne.setSearchMethod (tree_n);
    ne.setRadiusSearch (0.25);
    ne.compute (*cloud_normals);

    // Creating the kdtree object for the search method of the extraction
    pcl::KdTree<PointT>::Ptr tree_ec  (new pcl::KdTreeFLANN<PointT> ());
    tree_ec->setInputCloud (outputCloud);

    // Extracting Euclidean clusters using cloud and its normals
    std::vector<pcl::PointIndices> cluster_indices;
    const float tolerance = distance_threshold; // tolerance in (x, y, z) coordinate system
    const double eps_angle = 15 * (M_PI / 180.0); // degree tolerance in normals
    const unsigned int min_cluster_size = 10;
    const unsigned int max_cluster_size = 2000;

    pcl::extractEuclideanClusters (*outputCloud,
                                   *cloud_normals,
                                   tolerance,
                                   tree_ec,
                                   cluster_indices,
                                   eps_angle,
                                   min_cluster_size, max_cluster_size);

    std::cout << "No of clusters formed are " << cluster_indices.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    // Looping through the clusters
    for (const auto & cluster_indice : cluster_indices){

        pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const int &index : cluster_indice.indices)
            cloud_cluster->push_back ((*outputCloud)[index]);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud_cluster,centroid);

        if ((-1 * centroid[1]) > 5 && (-1 * centroid[1]) < 27) {
            std::cout << "cluster is are " << cloud_cluster->size() << std::endl;
            cloud_filtered = cloud_cluster;
            // publish filtered cloud data
            sensor_msgs::PointCloud2 cloud_publish;
            pcl::toROSMsg(*cloud_filtered, cloud_publish);
            cloud_publish.header = input->header;
            cloud_publish.is_dense = false;
            pub.publish(cloud_publish);

            geometry_msgs::PointStamped point = geometry_msgs::PointStamped();
            point.point.x = centroid[0];
            point.point.y = centroid[1];
            point.point.z = centroid[2];
            point.header = cloud_publish.header;
            position_pub.publish(point);

            std::cout << "center" << std::endl;
            std::cout << centroid << std::endl;
            std::cout << "===" << std::endl;
        }
    }
}


int main (int argc, char** argv){

    // Initialize ROS
    ros::init (argc, argv, "cube_detection");
    ros::NodeHandle nh;

    distance_threshold = 3.00;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/stereo/points2", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    position_pub = nh.advertise<geometry_msgs::PointStamped> ("/cube_position", 2);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/cube_points2", 2);

    // Spin
    ros::spin ();

    return 0;
}

