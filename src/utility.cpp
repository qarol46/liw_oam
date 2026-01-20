#include "utility.h"
#include "cloudProcessing.h"

// Глобальные переменные
bool debug_output = false;
std::string output_path = "";

bool time_diff_enable = false;
double time_diff = 0.0;

float mov_threshold = 1.5f;

bool initial_flag = false;

Eigen::Vector3d G;

bool time_list(point3D &point_1, point3D &point_2)
{
    return (point_1.relative_time < point_2.relative_time);
}

// bool time_list_velodyne(const velodyne_ros::Point& point_1, const velodyne_ros::Point& point_2)
// {
//     return (point_1.time < point_2.time);
// }

void point3DtoPCL(std::vector<point3D> &v_point_temp, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp)
{
    for(size_t i = 0; i < v_point_temp.size(); i++)
    {
        pcl::PointXYZINormal cloud_temp;
        cloud_temp.x = v_point_temp[i].raw_point.x();
        cloud_temp.y = v_point_temp[i].raw_point.y();
        cloud_temp.z = v_point_temp[i].raw_point.z();
        cloud_temp.normal_x = 0;
        cloud_temp.normal_y = 0;
        cloud_temp.normal_z = 0;
        cloud_temp.intensity = v_point_temp[i].alpha_time;
        cloud_temp.curvature = v_point_temp[i].relative_time;

        p_cloud_temp->points.push_back(cloud_temp);
    }
}

Eigen::Matrix3d mat33FromArray(std::vector<double> &array)
{
    assert(array.size() == 9);
    Eigen::Matrix3d mat33;
    mat33(0, 0) = array[0]; mat33(0, 1) = array[1]; mat33(0, 2) = array[2];
    mat33(1, 0) = array[3]; mat33(1, 1) = array[4]; mat33(1, 2) = array[5];
    mat33(2, 0) = array[6]; mat33(2, 1) = array[7]; mat33(2, 2) = array[8];

    return mat33;
}

Eigen::Vector3d vec3FromArray(std::vector<double> &array)
{
    assert(array.size() == 3);
    Eigen::Vector3d vec3;
    vec3(0) = array[0]; 
    vec3(1) = array[1]; 
    vec3(2) = array[2];

    return vec3;
}

void pointBodyToWorld(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, 
                      Eigen::Matrix3d &R_world_cur, Eigen::Vector3d &t_world_cur, 
                      Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    Eigen::Vector3d point_body(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_global(R_world_cur * (R_imu_lidar * point_body + t_imu_lidar) + t_world_cur);

    po->x = point_global(0);
    po->y = point_global(1);
    po->z = point_global(2);
    po->intensity = pi->intensity;
}

void RGBpointLidarToIMU(pcl::PointXYZINormal const * const pi, pcl::PointXYZINormal * const po, 
                        Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    Eigen::Vector3d pt_lidar(pi->x, pi->y, pi->z);
    Eigen::Vector3d pt_imu = R_imu_lidar * pt_lidar + t_imu_lidar;

    po->x = pt_imu(0);
    po->y = pt_imu(1);
    po->z = pt_imu(2);
    po->intensity = pi->intensity;
}

bool planeFitting(Eigen::Matrix<double, 4, 1> &plane_parameter, 
                  const std::vector<pcl::PointXYZINormal, Eigen::aligned_allocator<pcl::PointXYZINormal>> &point, 
                  const double &threshold)
{
    Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0;

    for (int i = 0; i < NUM_MATCH_POINTS; i++)
    {
        A(i, 0) = point[i].x;
        A(i, 1) = point[i].y;
        A(i, 2) = point[i].z;
    }

    Eigen::Matrix<double, 3, 1> norm_vector = A.colPivHouseholderQr().solve(b);

    double n = norm_vector.norm();
    if (n == 0) return false;  // Добавлена проверка на нулевую норму
    
    plane_parameter(0) = norm_vector(0) / n;
    plane_parameter(1) = norm_vector(1) / n;
    plane_parameter(2) = norm_vector(2) / n;
    plane_parameter(3) = 1.0 / n;

    for (int i = 0; i < NUM_MATCH_POINTS; i++)
    {
        if (fabs(plane_parameter(0) * point[i].x + 
                 plane_parameter(1) * point[i].y + 
                 plane_parameter(2) * point[i].z + 
                 plane_parameter(3)) > threshold)
            return false;
    }

    return true;
}

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb)
{
    double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
    norm = std::acos(std::clamp(norm, -1.0, 1.0)) * 180.0 / M_PI;  // Добавлен clamp для численной стабильности
    return norm;
}

double AngularDistance(const Eigen::Quaterniond &q_a, const Eigen::Quaterniond &q_b)
{
    Eigen::Matrix3d rota = q_a.toRotationMatrix();
    Eigen::Matrix3d rotb = q_b.toRotationMatrix();
    
    double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
    norm = std::acos(std::clamp(norm, -1.0, 1.0)) * 180.0 / M_PI;  // Добавлен clamp для численной стабильности
    return norm;
}

float calculateDist2(pcl::PointXYZINormal point1, pcl::PointXYZINormal point2)
{
    float dx = point1.x - point2.x;
    float dy = point1.y - point2.y;
    float dz = point1.z - point2.z;
    return dx*dx + dy*dy + dz*dz;
}

void saveCutCloud(std::string &str, pcl::PointCloud<pcl::PointXYZINormal>::Ptr &p_cloud_temp)
{
    if (!p_cloud_temp->empty())
    {
        pcl::io::savePCDFileBinary(str, *p_cloud_temp);
    }
}

struct VoxelKey {
    short x, y, z;
    
    bool operator==(const VoxelKey &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelHash {
    std::size_t operator()(const VoxelKey& k) const {
        return ((std::hash<short>()(k.x) ^ (std::hash<short>()(k.y) << 1)) >> 1) ^ (std::hash<short>()(k.z) << 1);
    }
};

void subSampleFrame(std::vector<point3D> &frame, double size_voxel)
{
    if (frame.empty() || size_voxel <= 0) return;
    
    std::unordered_map<VoxelKey, std::vector<point3D>, VoxelHash> grid;
    
    for (size_t i = 0; i < frame.size(); i++) {
        VoxelKey key;
        key.x = static_cast<short>(frame[i].point[0] / size_voxel);
        key.y = static_cast<short>(frame[i].point[1] / size_voxel);
        key.z = static_cast<short>(frame[i].point[2] / size_voxel);
        grid[key].push_back(frame[i]);
    }
    
    std::vector<point3D> result;
    result.reserve(grid.size());
    
    for (const auto &pair : grid) {
        if (!pair.second.empty()) {
            result.push_back(pair.second[0]);
        }
    }
    
    frame = std::move(result);
}

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling)
{
    keypoints.clear();
    
    if (frame.empty() || size_voxel_subsampling <= 0) {
        return;
    }
    
    std::vector<point3D> frame_sub = frame;
    subSampleFrame(frame_sub, size_voxel_subsampling);
    
    keypoints = std::move(frame_sub);
}

void distortFrame(std::vector<point3D> &points, 
                  Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, 
                  Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end,
                  Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    if (points.empty()) return;
    
    Eigen::Quaterniond q_end_inv = q_end.inverse();
    Eigen::Vector3d t_end_inv = -1.0 * (q_end_inv * t_end);
    
    for (auto &point_temp : points)
    {
        double alpha_time = point_temp.alpha_time;
        Eigen::Quaterniond q_alpha = q_begin.slerp(alpha_time, q_end);
        q_alpha.normalize();
        Eigen::Vector3d t_alpha = (1.0 - alpha_time) * t_begin + alpha_time * t_end;

        point_temp.raw_point = R_imu_lidar.transpose() * (q_end_inv * (q_alpha * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t_alpha) + t_end_inv) 
                             - R_imu_lidar.transpose() * t_imu_lidar;
    }
}

void transformPoint(MotionCompensation compensation, point3D &point_temp, 
                    Eigen::Quaterniond &q_begin, Eigen::Quaterniond &q_end, 
                    Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end,
                    Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar)
{
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    double alpha_time = point_temp.alpha_time;
    
    switch (compensation) {
        case MotionCompensation::NONE:
        case MotionCompensation::CONSTANT_VELOCITY:
            R = q_end.toRotationMatrix();
            t = t_end;
            break;
        case MotionCompensation::CONTINUOUS:
        case MotionCompensation::ITERATIVE:
            R = q_begin.slerp(alpha_time, q_end).normalized().toRotationMatrix();
            t = (1.0 - alpha_time) * t_begin + alpha_time * t_end;
            break;
    }
    
    point_temp.point = R * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t;
}