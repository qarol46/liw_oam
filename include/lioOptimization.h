#pragma once
// c++
#include <iostream>
#include <cmath>
#include <thread>
#include <fstream>
#include <vector>
#include <queue>
#include <memory>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Boost для random
#include <boost/random.hpp>

// eigen 
#include <Eigen/Core>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>	
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include "cloudMap.h"
#include "cloudProcessing.h"
#include "imuProcessing.h"
#include "utility.h"
#include "parameters.h"
#include "imuFactor.h"
#include "lidarFactor.h"
#include "poseParameterization.h"

class cloudFrame
{
public:
    double time_sweep_begin, time_sweep_end;
    double time_frame_begin, time_frame_end;

    int id;   // the index in all_cloud_frame
    int sub_id;    // the index of segment >= 0 && <= sweep_cut_num - 1
    int frame_id;

    state *p_state;

    std::vector<point3D> point_frame;
    std::vector<point3D> const_frame;

    double offset_begin;
    double offset_end;
    double dt_offset;

    bool success;

    cloudFrame(std::vector<point3D> &point_frame_, std::vector<point3D> &const_frame_, state *p_state_);

    cloudFrame(cloudFrame *p_cloud_frame);

    void release();
};

struct Neighborhood {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d center = Eigen::Vector3d::Zero();

    Eigen::Vector3d normal = Eigen::Vector3d::Zero();

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();

    double a2D = 1.0; // Planarity coefficient
};

struct ResidualBlock {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d point_closest;

    Eigen::Vector3d pt_imu;

    Eigen::Vector3d normal;

    double alpha_time;

    double weight;
};

class estimationSummary {

public:

    state *state_frame = nullptr;

    int sample_size = 0; // The number of points sampled

    int number_of_residuals = 0; // The number of keypoints used for ICP registration

    int robust_level = 0;

    double distance_correction = 0.0; // The correction between the last frame's end, and the new frame's beginning

    double relative_distance = 0.0; // The distance between the beginning of the new frame and the end

    double relative_orientation = 0.0; // The distance between the beginning of the new frame and the end

    double ego_orientation = 0.0; // The angular distance between the beginning and the end of the frame

    bool success = true; // Whether the registration was a success

    int number_of_attempts = 0; // The number of attempts at registering the new frame

    std::string error_message;

    std::vector<point3D> corrected_points; // Sampled points expressed in the initial frame

    std::vector<point3D> all_corrected_points; // Initial points expressed in the initial frame

    std::vector<point3D> keypoints; // Last Keypoints selected

    estimationSummary();

    void release();

};

struct optimizeSummary {

    bool success = false; // Whether the registration succeeded

    int num_residuals_used = 0;

    std::string error_log;
};

class lioOptimization : public rclcpp::Node
{
private:
    // ROS2 Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_body;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;

    // ROS2 Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_ori;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_ori;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_wheel_ori;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_gps_ori;

    // TF2 Broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::string lidar_topic;
    std::string imu_topic;
    std::string wheel_topic;

    cloudProcessing *cloud_pro;
    imuProcessing *imu_pro;

    bool extrin_enable;
    bool Odom_enble = true;

    double laser_point_cov;

    std::vector<double> v_G;
    std::vector<double> v_extrin_t;
    std::vector<double> v_extrin_R;
    std::vector<double> v_extrin_t_odom;
    std::vector<double> v_extrin_R_odom;

    Eigen::Matrix3d R_imu_lidar;
    Eigen::Vector3d t_imu_lidar;
    Eigen::Matrix3d R_imu_odom;
    Eigen::Vector3d t_imu_odom;

    Eigen::Vector3d pose_lid;

    std::queue<std::pair<double, double>> time_buffer;
    std::queue<std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> feature_buffer;
    std::queue<std::vector<std::vector<point3D>>> lidar_buffer;
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
    std::queue<geometry_msgs::msg::TwistStamped::SharedPtr> wheel_buffer;
    std::vector<std::vector<geometry_msgs::msg::TwistStamped::SharedPtr>> vWheel_msg;

    std::vector<cloudFrame*> all_cloud_frame;

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr down_cloud_body;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr down_cloud_world;

    std::vector<std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>>>  nearest_points;

    double last_time_lidar;
    double last_time_imu;
    double last_time_velo;
    double last_time_frame;
    double current_time;

    int sweep_cut_num;
    int index_frame;

    double time_max_solver;
    int num_max_iteration;

    // lidar range
    float det_range;
    double fov_deg;

    voxelHashMap voxel_map;

    odometryOptions options;

    int next_robust_level;

    int registered_frames;

    int robust_num_consecutive_failures;

    bool suspect_registration_error;

    double time_init;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> v_acc_static;
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> v_gyr_static;

    geometry_msgs::msg::Quaternion geoQuat;
    geometry_msgs::msg::PoseStamped msg_body_pose;
    nav_msgs::msg::Path path;
    nav_msgs::msg::Odometry odomAftMapped;

public:
    explicit lioOptimization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void readParameters();

    void allocateMemory();

    void initialValue();

    void standardCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);

    void wheelHandler(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    std::vector<std::pair<std::pair<std::vector<sensor_msgs::msg::Imu::SharedPtr>, std::vector<std::vector<point3D>>>, std::pair<double, double>>> 
    getMeasurements(std::vector<std::vector<geometry_msgs::msg::TwistStamped::SharedPtr>>& vWheel_msg);

    // main loop
    void stateEstimation(std::vector<std::vector<point3D>> &v_cut_sweep, double timestamp_begin, double timestamp_offset);

    void run();
    // main loop

    // data pre-processing
    cloudFrame* buildFrame(std::vector<point3D> &const_frame, state *cur_state, double timestamp_begin, double timestamp_offset);

    void makePointTimestamp(std::vector<point3D> &sweep, double time_sweep_begin, double time_sweep_end);

    void stateInitialization(state *cur_state);
    // data pre-processing

    // initialization
    void staticInitialization(cloudFrame* p_frame);

    void motionInitialization();

    bool initialLidarStructure(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose);

    bool lidarImuAlignment(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose, Eigen::Vector3d &g, Eigen::VectorXd &x);

    void solveGyroscopeBias(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose);

    bool linearAlignment(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose, Eigen::Vector3d &g, Eigen::VectorXd &x);

    Eigen::Matrix<double, 3, 2> tangentBasis(Eigen::Vector3d &g0);

    void refineGravity(std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &v_lidar_pose, Eigen::Vector3d &g, Eigen::VectorXd &x);
    // initialization

    // state estimation
    estimationSummary poseEstimation(cloudFrame *p_frame);

    estimationSummary optimize(cloudFrame *p_frame, const icpOptions &cur_icp_options, estimationSummary &summary, double sample_voxel_size);

    optimizeSummary optimizeByAnalyticLio(const icpOptions &cur_icp_options, const voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame);

    optimizeSummary optimizeByAnalyticLidar(const icpOptions &cur_icp_options, const voxelHashMap &voxel_map_temp, std::vector<point3D> &keypoints, cloudFrame *p_frame);
    // state estimation

    // search neighbors
    Neighborhood computeNeighborhoodDistribution(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> &points);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> searchNeighbors(const voxelHashMap &map, const Eigen::Vector3d &point,
        int nb_voxels_visited, double size_voxel_map, int max_num_neighbors, int threshold_voxel_capacity = 1, std::vector<voxel> *voxels = nullptr);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> selectClosestNeighbors(const std::vector<std::vector<Eigen::Vector3d> const *> &neighbors_ptr, 
        const Eigen::Vector3d &pt_keypoint, int num_neighbors, int max_num_neighbors);
    // search neighbors

    // map update
    void addPointToMap(voxelHashMap &map, const Eigen::Vector3d &point, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points, cloudFrame* p_frame);

    void addPointsToMap(voxelHashMap &map, cloudFrame* p_frame, double voxel_size, int max_num_points_in_voxel, double min_distance_points, int min_num_points = 0);

    bool assessRegistration(const cloudFrame *p_frame, estimationSummary &summary);

    void removePointsFarFromLocation(voxelHashMap &map, const Eigen::Vector3d &location, double distance);

    size_t mapSize(const voxelHashMap &map);
    // map update

    // save state for evaluation
    void recordSinglePose(cloudFrame *p_frame);
    // save state for evaluation

    // visualization
    void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path, cloudFrame *p_frame);

    void set_posestamp(geometry_msgs::msg::PoseStamped &body_pose_out, cloudFrame *p_frame);

    void addPointToPcl(pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_points, const Eigen::Vector3d& point, cloudFrame *p_frame);
    
    void publishCLoudWorld(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub_cloud_world, 
                          pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudFullRes, cloudFrame* p_frame);

    pcl::PointCloud<pcl::PointXYZI>::Ptr points_world;
    
    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped, cloudFrame *p_frame);

    // visualization
};