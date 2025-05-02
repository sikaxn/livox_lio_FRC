#include "LidarFeatureExtractor/LidarFeatureExtractor.h"

typedef pcl::PointXYZINormal PointType;

std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubFullLaserCloud;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubSharpCloud;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubFlatCloud;
std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubNonFeature;

LidarFeatureExtractor* lidarFeatureExtractor;
pcl::PointCloud<PointType>::Ptr laserCloud;
pcl::PointCloud<PointType>::Ptr laserConerCloud;
pcl::PointCloud<PointType>::Ptr laserSurfCloud;
pcl::PointCloud<PointType>::Ptr laserNonFeatureCloud;

int Lidar_Type    = 0;
int N_SCANS       = 6;
bool Feature_Mode = false;
bool Use_seg      = false;

void lidarCallBackHorizon(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {
    sensor_msgs::msg::PointCloud2 msg2;

    if (Use_seg) {
        lidarFeatureExtractor->FeatureExtract_with_segment(
            msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, msg2, N_SCANS);
    } else {
        lidarFeatureExtractor->FeatureExtract(
            msg, laserCloud, laserConerCloud, laserSurfCloud, N_SCANS, Lidar_Type);
    }

    sensor_msgs::msg::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*laserCloud, laserCloudMsg);
    laserCloudMsg.header       = msg->header;
    laserCloudMsg.header.stamp = rclcpp::Time(msg->timebase + msg->points.back().offset_time);
    pubFullLaserCloud->publish(laserCloudMsg);
}

void lidarCallBackHAP(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr& msg) {

    sensor_msgs::msg::PointCloud2 msg2;

    if (Use_seg) {
        lidarFeatureExtractor->FeatureExtract_with_segment_hap(
            msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, msg2, N_SCANS);
    } else {
        lidarFeatureExtractor->FeatureExtract_hap(
            msg, laserCloud, laserConerCloud, laserSurfCloud, laserNonFeatureCloud, N_SCANS);
    }

    sensor_msgs::msg::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*laserCloud, laserCloudMsg);
    laserCloudMsg.header       = msg->header;
    laserCloudMsg.header.stamp = rclcpp::Time(msg->timebase + msg->points.back().offset_time);
    pubFullLaserCloud->publish(laserCloudMsg);
}

void lidarCallBackPc2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr laser_cloud_custom(
        new pcl::PointCloud<pcl::PointXYZINormal>());

    pcl::fromROSMsg(*msg, *laser_cloud);

    for (uint64_t i = 0; i < laser_cloud->points.size(); i++) {
        auto p = laser_cloud->points.at(i);
        pcl::PointXYZINormal p_custom;
        if (Lidar_Type == 0 || Lidar_Type == 1) {
            if (p.x < 0.01)
                continue;
        } else if (Lidar_Type == 2) {
            if (std::fabs(p.x) < 0.01)
                continue;
        }
        p_custom.x         = p.x;
        p_custom.y         = p.y;
        p_custom.z         = p.z;
        p_custom.intensity = p.intensity;
        p_custom.normal_x  = float(i) / float(laser_cloud->points.size());
        p_custom.normal_y  = i % 4;
        laser_cloud_custom->points.push_back(p_custom);
    }

    lidarFeatureExtractor->FeatureExtract_Mid(laser_cloud_custom, laserConerCloud, laserSurfCloud);

    sensor_msgs::msg::PointCloud2 laserCloudMsg;
    pcl::toROSMsg(*laser_cloud_custom, laserCloudMsg);
    laserCloudMsg.header = msg->header;
    pubFullLaserCloud->publish(laserCloudMsg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto option = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true);
    auto node   = std::make_shared<rclcpp::Node>("ScanRegistration", option);

    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> customCloud;
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pc2Cloud;

    int msg_type = 0;
    node->get_parameter("msg_type", msg_type);

    Lidar_Type   = node->get_parameter_or("Lidar_Type", 0);
    N_SCANS      = node->get_parameter_or("Used_Line", 4);
    Feature_Mode = node->get_parameter_or("Feature_Mode", false);
    Use_seg      = node->get_parameter_or("Use_seg", false);

    int NumCurvSize               = node->get_parameter_or("NumCurvSize", 0);
    int NumFlat                   = node->get_parameter_or("NumFlat", 0);
    int PartNum                   = node->get_parameter_or("PartNum", 0);
    double DistanceFaraway        = node->get_parameter_or("DistanceFaraway", 0.);
    double FlatThreshold          = node->get_parameter_or("FlatThreshold", 0.);
    double BreakCornerDis         = node->get_parameter_or("BreakCornerDis", 0.);
    double LidarNearestDis        = node->get_parameter_or("LidarNearestDis", 0.);
    double KdTreeCornerOutlierDis = node->get_parameter_or("KdTreeCornerOutlierDis", 0.);

    laserCloud           = std::make_shared<pcl::PointCloud<PointType>>();
    laserConerCloud      = std::make_shared<pcl::PointCloud<PointType>>();
    laserSurfCloud       = std::make_shared<pcl::PointCloud<PointType>>();
    laserNonFeatureCloud = std::make_shared<pcl::PointCloud<PointType>>();

    if (Lidar_Type == 0) {
        customCloud = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 100, &lidarCallBackHorizon);
    } else if (Lidar_Type == 1) {
        customCloud = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 100, &lidarCallBackHAP);
    } else if (Lidar_Type == 2) {
        if (msg_type == 0)
            customCloud = node->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 100, &lidarCallBackHorizon);
        else if (msg_type == 1)
            pc2Cloud = node->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 100, &lidarCallBackPc2);
    }

    pubFullLaserCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_livox/full_cloud", 10);
    pubSharpCloud = node->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_livox/less_sharp_cloud", 10);
    pubFlatCloud  = node->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_livox/less_flat_cloud", 10);
    pubNonFeature = node->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_livox/nonfeature_cloud", 10);

    lidarFeatureExtractor = new LidarFeatureExtractor(
        N_SCANS, NumCurvSize, static_cast<float>(DistanceFaraway), NumFlat, PartNum,
        static_cast<float>(FlatThreshold), static_cast<float>(BreakCornerDis),
        static_cast<float>(LidarNearestDis), static_cast<float>(KdTreeCornerOutlierDis));

    rclcpp::spin(node);

    return 0;
}
