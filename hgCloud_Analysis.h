#ifndef HG_CLOUD_ANALYSIS_H
#define HG_CLOUD_ANALYSIS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <limits>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
#include <utility>

namespace hgCloud
{
    using PointT = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<PointT>;
    using PointCloudPtr = PointCloud::Ptr;

    // ==================== 点云与Eigen互转 ====================
    std::vector<Eigen::Vector3f> pointCloudToEigen(const PointCloud& cloud);
    std::vector<Eigen::Vector3f> pointCloudToEigen(const PointCloudPtr& cloud);
    PointCloudPtr eigenToPointCloud(const std::vector<Eigen::Vector3f>& vec);
    PointCloudPtr contourToPointCloud(const std::vector<Eigen::Vector3f>& contour3d, float sample_step = 0.01f, bool closed = true);

    // ==================== 点云基础属性计算 ====================
    void getRange(const PointCloud& cloud, Eigen::Vector3f& min_pt, Eigen::Vector3f& max_pt);
    void getRange(const PointCloudPtr& cloud, Eigen::Vector3f& min_pt, Eigen::Vector3f& max_pt);
    Eigen::Vector3f Centroid(const PointCloud& input);
    Eigen::Vector3f Centroid(const PointCloudPtr& input);
    Eigen::Vector3f Center(const PointCloud& input);
    Eigen::Vector3f Center(const PointCloudPtr& input);

    // ==================== 距离与几何相关计算 ====================
    double pt2Line(const Eigen::Vector3d& pt, const Eigen::Vector3d& line_point, const Eigen::Vector3d& line_dir);
    double pt2LineSegment(const Eigen::Vector3d& pt, const Eigen::Vector3d& seg_start, const Eigen::Vector3d& seg_end);
    double ptsDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);

    // ==================== 点云投影 ====================
    void projectXY(const PointCloud& input, PointCloud& output);
    void projectXY(const PointCloudPtr& input, PointCloudPtr& output);

    // ==================== 点云下采样相关 ====================
    void downsampleCentroid(const PointCloud& input, PointCloud& output, float voxel_size);
    void downsampleCentroid(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size);
    void downsampleCentroid_keepSize(const PointCloud& input, PointCloud& output, float voxel_size);
    void downsampleCentroid_keepSize(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size);
    void downsampleCentroid_keepAll(const PointCloud& input, PointCloud& output, float voxel_size);
    void downsampleCentroid_keepAll(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size);
    void downsampleCenter(const PointCloud& input, PointCloud& output, float voxel_size);
    void downsampleCenter(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size);
    void downsampleCenter_keepSize(const PointCloud& input, PointCloud& output, float voxel_size);
    void downsampleCenter_keepSize(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size);

    // ==================== 点云滤波 ====================
    void filterSOR(const PointCloud& input, PointCloud& output, int meanK = 50, double stddevMulThresh = 1.0);
    void filterSOR(const PointCloudPtr& input, PointCloudPtr& output, int meanK = 50, double stddevMulThresh = 1.0);
    void filterROR(const PointCloud& input, PointCloud& output, double radiusSearch = 0.8, int minNeighborsInRadius = 2);
    void filterROR(const PointCloudPtr& input, PointCloudPtr& output, double radiusSearch = 0.8, int minNeighborsInRadius = 2);
    void filterPassThrough(const PointCloud& input, PointCloud& output, const std::string& field_name, float limit_min, float limit_max);
    void filterPassThrough(const PointCloudPtr& input, PointCloudPtr& output, const std::string& field_name, float limit_min, float limit_max);

    // ==================== 主方向分析（PCA） ====================
    std::vector<Eigen::Vector3f> directionsPCA(const PointCloud& input);
    std::vector<Eigen::Vector3f> directionsPCA(const PointCloudPtr& input);

    // ==================== 点云分割与几何拟合 ====================
    bool ransacLineFit(const PointCloud& input, pcl::PointIndices& inliers, pcl::ModelCoefficients& coefficients, double distance_threshold = 0.01);
    bool ransacLineFit(const PointCloudPtr& input, pcl::PointIndices& inliers, pcl::ModelCoefficients& coefficients, double distance_threshold = 0.01);
    std::vector<Eigen::Vector3d> ransacPolyFit(const PointCloudPtr& input, int max_lines, double distance_threshold, int min_inliers);
    bool fitConvexHull3D(const std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& hull_pts, std::vector<Eigen::Vector3i>& faces);
    bool fitConvexHull2D(const std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& hull_pts, std::vector<int>& hull_indices);

    // ==================== 点云分离与特征识别 ====================
    void separateClouds(const PointCloudPtr& input, std::vector<PointCloudPtr>& separated_clouds, const std::string& axis = "y", double minZ = -199);
    bool grooveRecognition_Rotation(const PointCloudPtr& cloud, Eigen::Vector3f& pt_left_up, Eigen::Vector3f& pt_right_up, Eigen::Vector3f& pt_down, double ransac_dist_thresh = 0.05, double rotation_angle_deg = 30.0);
    int getKernelSize(const PointCloudPtr& cloud, int bin_count, double k);
    bool grooveRecognition_Convolution(const PointCloudPtr& cloud, Eigen::Vector3f& pt_left_up, Eigen::Vector3f& pt_right_up, Eigen::Vector3f& pt_left_down, Eigen::Vector3f& pt_right_down, int kernel_size = 35, double ransac_dist_thresh = 0.05, double rotation_angle_deg = 5.0);

    // ==================== 点云图像化 ====================
    cv::Mat cloudToGrayImage(const PointCloud& cloud, int width, int height, float z_min, float z_max, float margin_ratio = 0.05f);
    cv::Mat cloudToGrayImage(const PointCloudPtr& cloud, int width, int height, float z_min, float z_max, float margin_ratio = 0.05f);
    void cloudToBinary(const PointCloud& cloud, cv::Mat& binary, int width, int height, float z_min, float z_max);
	void cloudToBinary(const PointCloudPtr& cloud, cv::Mat& binary, int width, int height, float z_min, float z_max);

    // ==================== 曲线简化 ====================
    std::vector<Eigen::Vector3d> douglasPeucker(const std::vector<Eigen::Vector3d>& points, double epsilon);

    // ==================== 曲线平滑 ====================
    std::vector<Eigen::Vector3f> smoothCurve(const std::vector<Eigen::Vector3f>& points, int window_size = 5);
    std::vector<Eigen::Vector3f> smoothCurveGaussian(const std::vector<Eigen::Vector3f>& points, int window_size = 5, double sigma = -1.0);
} // namespace hgCloud

#endif // HG_CLOUD_ANALYSIS_H
