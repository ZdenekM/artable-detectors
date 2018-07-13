#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/voxel_grid.h>

#include <visualization_msgs/Marker.h>
#include <pcl/segmentation/extract_clusters.h>
#include <geometry_msgs/PointStamped.h>
#include <art_msgs/ObjectsCentroids.h>
#include <std_msgs/Bool.h>



typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

class ClusterDetector {
public:
    ClusterDetector() : spinner(4){
        sub = nh.subscribe ("/kinect2/qhd/points", 1, &ClusterDetector::cloud_cb, this);

        // Create a ROS publisher for the output point cloud
        pub = nh.advertise<sensor_msgs::PointCloud2> ("/output", 1);
        vis_pub = nh.advertise<visualization_msgs::Marker> ("/visualization", 1);
        user_pub = nh.advertise<std_msgs::Bool> ("/art/interface/user/present", 1);
        spinner.start();
    }

private:
    ros::Publisher pub;
    ros::Publisher user_pub;
    ros::Publisher vis_pub;
    ros::Subscriber sub;
    tf::TransformListener listener_;
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner;



    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
    {
        ros::Time begin = ros::Time::now();

        ROS_INFO_STREAM_ONCE("First point cloud arrived");
        // Create a container for the data.

        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2 temp_cloud;
        pcl::IndicesPtr indices1 (new std::vector <int>), indices2 (new std::vector <int>);
        pcl_conversions::toPCL(*input, temp_cloud);
        //temp_cloud.header.stamp = ros::Time::now().toSec()*1000000;
        PointCloudPtr pc(new PointCloud);
        pcl::fromPCLPointCloud2(temp_cloud, *pc);
        PointCloudPtr pc_transformed(new PointCloud);


        listener_.waitForTransform(input->header.frame_id, "/marker",
                                  input->header.stamp, ros::Duration(2));

        pcl_ros::transformPointCloud("/marker", *pc, *pc_transformed, listener_);


        pcl::search::Search <PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointType> > (new pcl::search::KdTree<PointType>);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointType> ec;
        PointCloudPtr pc_filtered(new PointCloud);

        pcl::PassThrough<PointType> pass;
        pass.setInputCloud(pc_transformed);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(-0.2,1.2);
        pass.filter(*indices1);

        pass.setIndices(indices1);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-2,0);
        pass.filter(*indices2);

        pass.setIndices(indices2);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-1,1);
        pass.filter(*indices1);



        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(pc);
        extract.setIndices(indices1);
        extract.filter(*pc_filtered);

        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(pc_filtered);
        sor.setLeafSize (0.01f, 0.01f, 0.01f);
        sor.filter (*pc_filtered);


        ec.setClusterTolerance (0.05);
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (10500);
        ec.setSearchMethod (tree);
        ec.setInputCloud (pc_filtered);
        ec.extract (cluster_indices);

        art_msgs::ObjectsCentroids centroids;
        centroids.header.frame_id = "/marker";

        std::cout << "Clusters: " << cluster_indices.size() << std::endl;
        if (cluster_indices.size() > 0) {
            user_pub.publish(true);
        } else {
            user_pub.publish(false);
        }

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {

            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*pc_filtered, it->indices, centroid);
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/kinect2_ir_optical_frame";
            marker.header.stamp = ros::Time::now();
            marker.ns = "my_namespace";
            marker.id = j;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = centroid[0];
            marker.pose.position.y = centroid[1];
            marker.pose.position.z = centroid[2];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.scale.y = 0.03;
            marker.scale.z = 0.03;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            vis_pub.publish( marker );
            j++;




         }


        //ROS_INFO_STREAM("size: " << pc_filtered->size());
        if (pc_filtered->size() > 0) {
            pcl::toROSMsg(*pc_filtered, output);
            pub.publish(output);
        }

        std::cout << "Processing time: " << ros::Time::now() - begin << std::endl;
    }

};


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "art_cluster_detector_node");


  ClusterDetector detector;

  // Create a ROS subscriber for the input point cloud


  // Spin
  ros::waitForShutdown();

}
