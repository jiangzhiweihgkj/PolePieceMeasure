#include "hgCloud_Analysis.h"

namespace hgCloud {
    // ==================== 点云与Eigen互转 ====================

    // 将PCL点云转换为Eigen矩阵（n x 3）
    std::vector<Eigen::Vector3f> pointCloudToEigen(const PointCloud& cloud)
    {
        std::vector<Eigen::Vector3f> vec;
        vec.reserve(cloud.points.size());
        for (const auto& pt : cloud.points)
        {
            vec.emplace_back(pt.x, pt.y, pt.z);
        }
        return vec;
    }
    std::vector<Eigen::Vector3f> pointCloudToEigen(const PointCloudPtr& cloud)
    {
        if (!cloud) return {};
        return pointCloudToEigen(*cloud);
    }
    // 将Eigen矩阵转换为PCL点云智能指针
    PointCloudPtr eigenToPointCloud(const std::vector<Eigen::Vector3f>& vec)
    {
        auto cloud = std::make_shared<PointCloud>();
        cloud->points.reserve(vec.size());
        for (const auto& pt : vec)
        {
            pcl::PointXYZ pcl_pt(pt.x(), pt.y(), pt.z());
            cloud->points.push_back(pcl_pt);
        }
        cloud->width = static_cast<uint32_t>(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;
        return cloud;
    }
    // 沿三维轨迹点集均匀插值采样
    PointCloudPtr contourToPointCloud(const std::vector<Eigen::Vector3f>& contour3d, float sample_step, bool closed)
    {
        const size_t n_points = contour3d.size();
        if (n_points < 2)
            return {};

        std::vector<Eigen::Vector3f> dense_points;

        // 如果闭合，最后一段连接尾首；否则只到倒数第二点
        size_t seg_count = closed ? n_points : n_points - 1;

        for (size_t i = 0; i < seg_count; ++i)
        {
            const Eigen::Vector3f& pt_start = contour3d[i];
            const Eigen::Vector3f& pt_end = contour3d[(i + 1) % n_points];

            Eigen::Vector3f diff = pt_end - pt_start;
            float length = diff.norm();
            if (length < 1e-6f)
                continue;

            int num_samples = static_cast<int>(std::ceil(length / sample_step));
            num_samples = std::max(1, num_samples);

            for (int s = 0; s <= num_samples; ++s)
            {
                float t = static_cast<float>(s) / num_samples;
                Eigen::Vector3f pt = pt_start + t * diff;
                dense_points.push_back(pt);
            }
        }

        return eigenToPointCloud(dense_points);
    }


    // ==================== 点云基础属性计算 ====================

    // 计算点云xyz最小最大边界
    void getRange(const PointCloud& cloud, Eigen::Vector3f& min_pt, Eigen::Vector3f& max_pt)
    {
        if (cloud.empty()) {
            min_pt = Eigen::Vector3f::Zero();
            max_pt = Eigen::Vector3f::Zero();
            return;
        }

        float x_min = std::numeric_limits<float>::max();
        float y_min = std::numeric_limits<float>::max();
        float z_min = std::numeric_limits<float>::max();
        float x_max = -std::numeric_limits<float>::max();
        float y_max = -std::numeric_limits<float>::max();
        float z_max = -std::numeric_limits<float>::max();

        for (const auto& pt : cloud.points) {
            if (pt.x < x_min) x_min = pt.x;
            if (pt.y < y_min) y_min = pt.y;
            if (pt.z < z_min) z_min = pt.z;

            if (pt.x > x_max) x_max = pt.x;
            if (pt.y > y_max) y_max = pt.y;
            if (pt.z > z_max) z_max = pt.z;
        }

        min_pt = Eigen::Vector3f(x_min, y_min, z_min);
        max_pt = Eigen::Vector3f(x_max, y_max, z_max);
    }
    void getRange(const PointCloudPtr& cloud, Eigen::Vector3f& min_pt, Eigen::Vector3f& max_pt)
    {
        if (!cloud || cloud->empty()) {
            min_pt = Eigen::Vector3f::Zero();
            max_pt = Eigen::Vector3f::Zero();
            return;
        }

        getRange(*cloud, min_pt, max_pt);
    }
    // 计算点云质心
    Eigen::Vector3f Centroid(const PointCloud& input)
    {
        if (input.empty())
            return Eigen::Vector3f::Zero();

        Eigen::Vector4f centroid4f;
        pcl::compute3DCentroid(input, centroid4f);

        return centroid4f.head<3>();
    }
    Eigen::Vector3f Centroid(const PointCloudPtr& input)
    {
        if (!input || input->empty())
            return Eigen::Vector3f::Zero();

        return Centroid(*input);
    }
    // 计算点云外轮廓中心
    Eigen::Vector3f Center(const PointCloud& input)
    {
        if (input.empty())
            return Eigen::Vector3f::Zero();

        float x_min = std::numeric_limits<float>::max();
        float y_min = std::numeric_limits<float>::max();
        float z_min = std::numeric_limits<float>::max();

        float x_max = -std::numeric_limits<float>::max();
        float y_max = -std::numeric_limits<float>::max();
        float z_max = -std::numeric_limits<float>::max();

        for (const auto& pt : input.points) {
            if (pt.x < x_min) x_min = pt.x;
            if (pt.y < y_min) y_min = pt.y;
            if (pt.z < z_min) z_min = pt.z;

            if (pt.x > x_max) x_max = pt.x;
            if (pt.y > y_max) y_max = pt.y;
            if (pt.z > z_max) z_max = pt.z;
        }

        Eigen::Vector3f center;
        center.x() = 0.5f * (x_min + x_max);
        center.y() = 0.5f * (y_min + y_max);
        center.z() = 0.5f * (z_min + z_max);

        return center;
    }
    Eigen::Vector3f Center(const PointCloudPtr& input)
    {
        if (!input || input->empty())
            return Eigen::Vector3f::Zero();

        return Center(*input);
    }
    // 计算点到直线的距离
    inline double pt2Line(const Eigen::Vector3d& pt, const Eigen::Vector3d& line_point, const Eigen::Vector3d& line_dir)
    {
        // 直线的方向向量必须非零
        double dir_norm = line_dir.norm();
        if (dir_norm < 1e-12) return -1.0;

        Eigen::Vector3d v = pt - line_point;
        Eigen::Vector3d cross = v.cross(line_dir);
        // 距离公式：| (pt - line_point) × line_dir | / |line_dir|
        return cross.norm() / dir_norm;
    }
    // 计算点到线段距离
    inline double pt2LineSegment(const Eigen::Vector3d& pt, const Eigen::Vector3d& seg_start, const Eigen::Vector3d& seg_end)
    {
        Eigen::Vector3d v = seg_end - seg_start;
        Eigen::Vector3d w = pt - seg_start;

        double c1 = v.dot(w);
        if (c1 <= 0.0)
            return (pt - seg_start).norm();

        double c2 = v.dot(v);
        if (c2 < 1e-12) // 线段退化成点
            return (pt - seg_start).norm();

        if (c1 >= c2)
            return (pt - seg_end).norm();

        double b = c1 / c2;
        Eigen::Vector3d pb = seg_start + b * v;
        return (pt - pb).norm();
    }
    // 计算两点间距离
    inline double ptsDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
    {
        return (p1 - p2).norm();
    }


    // ==================== 点云下采样相关 ====================

    //  体素质心下采样（调用pcl版本，保留点云密度信息
    void downsampleCentroid(const PointCloud& input, PointCloud& output, float voxel_size)
    {
        if (input.empty() || voxel_size <= 0) {
            output.clear();
            return;
        }

        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(input.makeShared());
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(output);
    }
    void downsampleCentroid(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size)
    {
        if (!input || input->empty()) {
            if (output) output->clear();
            return;
        }
        if (!output) output.reset(new PointCloud);

        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(input);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        vg.filter(*output);
    }
    //  体素质心下采样（还原原始点云尺寸，保持外轮廓中心位置不变）
    void downsampleCentroid_keepSize(const PointCloud& input, PointCloud& output, float voxel_size)
    {
        if (input.empty() || voxel_size <= 0) {
            output.clear();
            return;
        }

        // 计算输入点云范围
        Eigen::Vector3f input_min, input_max;
        getRange(input, input_min, input_max);

        // 调用 downsampleCentroid 得到下采样点云（体素质心）
        PointCloud downsampled;
        downsampleCentroid(input, downsampled, voxel_size);
        if (downsampled.empty()) {
            output.clear();
            return;
        }

        // 计算下采样点云范围
        Eigen::Vector3f ds_min, ds_max;
        getRange(downsampled, ds_min, ds_max);

        // 计算坐标缩放和平移参数，实现 ds 点云坐标 -> input 点云坐标的线性映射
        Eigen::Vector3f ds_range = ds_max - ds_min;
        Eigen::Vector3f input_range = input_max - input_min;

        for (int i = 0; i < 3; ++i) {
            if (ds_range[i] < 1e-6f) ds_range[i] = 1.0f;
        }

        output.points.resize(downsampled.points.size());
        for (size_t i = 0; i < downsampled.points.size(); ++i) {
            const auto& pt = downsampled.points[i];

            float nx = (pt.x - ds_min.x()) / ds_range.x();
            float ny = (pt.y - ds_min.y()) / ds_range.y();
            float nz = (pt.z - ds_min.z()) / ds_range.z();

            PointT new_pt;
            new_pt.x = nx * input_range.x() + input_min.x();
            new_pt.y = ny * input_range.y() + input_min.y();
            new_pt.z = nz * input_range.z() + input_min.z();

            output.points[i] = new_pt;
        }

        output.width = static_cast<uint32_t>(output.points.size());
        output.height = 1;
        output.is_dense = true;
    }
    void downsampleCentroid_keepSize(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size)
    {
        if (!input || input->empty()) {
            if (output) output->clear();
            return;
        }
        if (!output) output.reset(new PointCloud);

        downsampleCentroid_keepSize(*input, *output, voxel_size);
    }
    //  体素质心下采样（迭代求解，还原原始点云的外轮廓、质心和中心） 
    void downsampleCentroid_keepAll(const PointCloud& input, PointCloud& output, float voxel_size)
    {
        if (input.empty() || voxel_size <= 0) {
            output.clear();
            return;
        }

        // 1. 计算输入点云范围和质心
        Eigen::Vector3f input_min, input_max;
        getRange(input, input_min, input_max);
        Eigen::Vector3f input_centroid = Centroid(input);

        // 2. 体素质心下采样
        PointCloud downsampled;
        downsampleCentroid(input, downsampled, voxel_size);
        if (downsampled.empty()) {
            output.clear();
            return;
        }

        // 3. 计算下采样点云范围
        Eigen::Vector3f ds_min, ds_max;
        getRange(downsampled, ds_min, ds_max);

        Eigen::Vector3f ds_range = ds_max - ds_min;
        Eigen::Vector3f input_range = input_max - input_min;

        // 防止ds_range接近0
        for (int i = 0; i < 3; ++i) {
            if (ds_range[i] < 1e-6f) ds_range[i] = 1.0f;
        }

        size_t M = downsampled.points.size();

        // 4. 计算下采样点云归一化坐标 t 以及每维的目标归一化质心 C
        std::vector<std::vector<float>> t_xyz(3, std::vector<float>(M, 0.0f));
        for (size_t k = 0; k < M; ++k) {
            const auto& pt = downsampled.points[k];
            t_xyz[0][k] = (pt.x - ds_min.x()) / ds_range.x();
            t_xyz[1][k] = (pt.y - ds_min.y()) / ds_range.y();
            t_xyz[2][k] = (pt.z - ds_min.z()) / ds_range.z();
        }

        Eigen::Vector3f C;
        for (int i = 0; i < 3; ++i) {
            C[i] = (input_centroid[i] - input_min[i]) / input_range[i];
            // 防止C超界
            C[i] = std::min(std::max(C[i], 0.0f), 1.0f);
        }

        // 5. 二分法求解 gamma，使 mean(t^gamma) = C
        auto solveGamma = [&](const std::vector<float>& t_vals, float target) -> float {
            if (target <= 0.0f) return 10.0f;  // 质心很靠近最小边界，拉伸指数大
            if (target >= 1.0f) return 0.01f; // 质心靠近最大边界，拉伸指数小

            float low = 0.01f, high = 10.0f;
            for (int iter = 0; iter < 25; ++iter) {
                float mid = 0.5f * (low + high);
                double val = 0.0;
                for (float v : t_vals) val += std::pow(v, mid);
                val /= t_vals.size();

                if (val < target) high = mid;
                else low = mid;
            }
            return 0.5f * (low + high);
            };

        Eigen::Vector3f gamma;
        for (int i = 0; i < 3; ++i) {
            gamma[i] = solveGamma(t_xyz[i], C[i]);
        }

        // 6. 根据 gamma 映射点云坐标
        output.points.resize(M);
        for (size_t k = 0; k < M; ++k) {
            PointT new_pt;
            new_pt.x = input_min.x() + input_range.x() * std::pow(t_xyz[0][k], gamma[0]);
            new_pt.y = input_min.y() + input_range.y() * std::pow(t_xyz[1][k], gamma[1]);
            new_pt.z = input_min.z() + input_range.z() * std::pow(t_xyz[2][k], gamma[2]);
            output.points[k] = new_pt;
        }

        output.width = static_cast<uint32_t>(output.points.size());
        output.height = 1;
        output.is_dense = true;
    }
    void downsampleCentroid_keepAll(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size)
    {
        if (!input || input->empty()) {
            if (output) output->clear();
            return;
        }
        if (!output) output.reset(new PointCloud);

        downsampleCentroid_keepAll(*input, *output, voxel_size);
    }
    //  体素中心下采样（无点云密度信息，纯外轮廓几何信息） 
    void downsampleCenter(const PointCloud& input, PointCloud& output, float voxel_size)
    {
        if (input.empty() || voxel_size <= 0) {
            output.clear();
            return;
        }

        struct VoxelKey {
            int x, y, z;
            bool operator<(const VoxelKey& other) const {
                if (x != other.x) return x < other.x;
                if (y != other.y) return y < other.y;
                return z < other.z;
            }
        };

        std::map<VoxelKey, int> voxel_map;

        for (const auto& pt : input.points) {
            int ix = static_cast<int>(std::floor(pt.x / voxel_size));
            int iy = static_cast<int>(std::floor(pt.y / voxel_size));
            int iz = static_cast<int>(std::floor(pt.z / voxel_size));
            voxel_map[VoxelKey{ ix, iy, iz }] = 1;
        }

        output.clear();
        output.points.reserve(voxel_map.size());

        for (const auto& kv : voxel_map) {
            const VoxelKey& key = kv.first;
            PointT center_pt;
            center_pt.x = (key.x + 0.5f) * voxel_size;
            center_pt.y = (key.y + 0.5f) * voxel_size;
            center_pt.z = (key.z + 0.5f) * voxel_size;
            output.points.push_back(center_pt);
        }

        output.width = static_cast<uint32_t>(output.points.size());
        output.height = 1;
        output.is_dense = true;
    }
    void downsampleCenter(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size)
    {
        if (!input || input->empty()) {
            if (output) output->clear();
            return;
        }
        if (!output) output.reset(new PointCloud);

        downsampleCenter_keepSize(*input, *output, voxel_size);
    }
    //  体素中心下采样（还原原始点云尺寸，保持外轮廓中心位置不变） 
    void downsampleCenter_keepSize(const PointCloud& input, PointCloud& output, float voxel_size)
    {
        if (input.empty() || voxel_size <= 0) {
            output.clear();
            return;
        }

        // 计算输入点云范围
        Eigen::Vector3f input_min, input_max;
        getRange(input, input_min, input_max);

        // 调用 downsampleCenter 得到下采样点云（体素质心）
        PointCloud downsampled;
        downsampleCenter(input, downsampled, voxel_size);
        if (downsampled.empty()) {
            output.clear();
            return;
        }

        // 计算下采样点云范围
        Eigen::Vector3f ds_min, ds_max;
        getRange(downsampled, ds_min, ds_max);

        // 计算坐标缩放和平移参数，实现 ds 点云坐标 -> input 点云坐标的线性映射
        Eigen::Vector3f ds_range = ds_max - ds_min;
        Eigen::Vector3f input_range = input_max - input_min;

        for (int i = 0; i < 3; ++i) {
            if (ds_range[i] < 1e-6f) ds_range[i] = 1.0f;
        }

        output.points.resize(downsampled.points.size());
        for (size_t i = 0; i < downsampled.points.size(); ++i) {
            const auto& pt = downsampled.points[i];

            float nx = (pt.x - ds_min.x()) / ds_range.x();
            float ny = (pt.y - ds_min.y()) / ds_range.y();
            float nz = (pt.z - ds_min.z()) / ds_range.z();

            PointT new_pt;
            new_pt.x = nx * input_range.x() + input_min.x();
            new_pt.y = ny * input_range.y() + input_min.y();
            new_pt.z = nz * input_range.z() + input_min.z();

            output.points[i] = new_pt;
        }

        output.width = static_cast<uint32_t>(output.points.size());
        output.height = 1;
        output.is_dense = true;
    }
    void downsampleCenter_keepSize(const PointCloudPtr& input, PointCloudPtr& output, float voxel_size)
    {
        if (!input || input->empty()) {
            if (output) output->clear();
            return;
        }
        if (!output) output.reset(new PointCloud);

        downsampleCenter(*input, *output, voxel_size);
    }


    // ==================== 点云滤波相关 ====================

    //  SOR - 统计离群点移除滤波 
    void filterSOR(const PointCloud& input, PointCloud& output, int meanK, double stddevMulThresh)
    {
        if (input.empty() || meanK <= 0 || stddevMulThresh <= 0) {
            output.clear();
            return;
        }

        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(input.makeShared());
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(stddevMulThresh);
        sor.filter(output);
    }
    void filterSOR(const PointCloudPtr& input, PointCloudPtr& output, int meanK, double stddevMulThresh)
    {
        if (!input || input->empty() || meanK <= 0 || stddevMulThresh <= 0) {
            if (output) output->clear();
            return;
        }
        if (!output) output.reset(new PointCloud);

        filterSOR(*input, *output, meanK, stddevMulThresh);
    }
    //  ROR - 半径离群点移除滤波  
    void filterROR(const PointCloud& input, PointCloud& output, double radiusSearch, int minNeighborsInRadius)
    {
        if (input.empty() || radiusSearch <= 0 || minNeighborsInRadius < 1) {
            output.clear();
            return;
        }

        pcl::RadiusOutlierRemoval<PointT> ror;
        ror.setInputCloud(input.makeShared());
        ror.setRadiusSearch(radiusSearch);
        ror.setMinNeighborsInRadius(minNeighborsInRadius);
        ror.filter(output);
    }
    void filterROR(const PointCloudPtr& input, PointCloudPtr& output, double radiusSearch, int minNeighborsInRadius)
    {
        if (!input || input->empty() || radiusSearch <= 0 || minNeighborsInRadius < 1) {
            if (output) output->clear();
            return;
        }
        if (!output) output.reset(new PointCloud);

        filterROR(*input, *output, radiusSearch, minNeighborsInRadius);
    }
    //  直通滤波，裁剪指定坐标轴（field_name）范围内的点（field_name常用"x","y","z"） 
    void filterPassThrough(const PointCloud& input, PointCloud& output, const std::string& field_name, float limit_min, float limit_max)
    {
        if (input.empty()) {
            output.clear();
            return;
        }
        if (limit_min > limit_max) {
            std::swap(limit_min, limit_max);
        }

        pcl::PassThrough<PointT> pass;
        pass.setInputCloud(input.makeShared());
        pass.setFilterFieldName(field_name);
        pass.setFilterLimits(limit_min, limit_max);
        pass.filter(output);
    }
    void filterPassThrough(const PointCloudPtr& input, PointCloudPtr& output, const std::string& field_name, float limit_min, float limit_max)
    {
        if (!input || input->empty()) {
            if (output) output->clear();
            return;
        }
        if (limit_min > limit_max) {
            std::swap(limit_min, limit_max);
        }
        if (!output) output.reset(new PointCloud);

        filterPassThrough(*input, *output, field_name, limit_min, limit_max);
    }


    // ==================== 点云投影与插值 ====================

    // 计算点云在XY平面上的投影
    void projectXY(const PointCloud& input, PointCloud& output)
    {
        if (input.empty()) {
            output.clear();
            return;
        }

        output.points.resize(input.points.size());
        output.width = input.width;
        output.height = input.height;
        output.is_dense = input.is_dense;

        for (size_t i = 0; i < input.points.size(); ++i)
        {
            const auto& pt = input.points[i];
            PointT projected_pt;
            projected_pt.x = pt.x;
            projected_pt.y = pt.y;
            projected_pt.z = 0.0f;  // 投影到XY平面
            output.points[i] = projected_pt;
        }
    }
    void projectXY(const PointCloudPtr& input, PointCloudPtr& output)
    {
        if (!input || input->empty()) {
            if (output) output->clear();
            return;
        }

        if (!output) output.reset(new PointCloud);

        projectXY(*input, *output);
    }


    // ==================== PCA主方向识别 ====================

    //  PCA 分析点云主方向 
    std::vector<Eigen::Vector3f> directionsPCA(const PointCloud& input)
    {
        std::vector<Eigen::Vector3f> directions;

        if (input.empty())
            return directions;

        pcl::PCA<PointT> pca;
        pca.setInputCloud(input.makeShared());

        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3f dir = eigen_vectors.col(i);
            directions.push_back(dir.normalized());
        }

        return directions;
    }
    std::vector<Eigen::Vector3f> directionsPCA(const PointCloudPtr& input)
    {
        if (!input || input->empty())
            return {};

        return directionsPCA(*input);
    }


    // ==================== RANSAC直线拟合 ====================

    //  RANSAC 直线拟合 
    bool ransacLineFit(const PointCloud& input, pcl::PointIndices& inliers, pcl::ModelCoefficients& coefficients, double distance_threshold) {
        if (input.empty() || distance_threshold <= 0.0) {
            return false;
        }

        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);
        seg.setInputCloud(input.makeShared());
        seg.segment(inliers, coefficients);

        if (inliers.indices.empty()) {
            return false;
        }
        return true;
    }
    bool ransacLineFit(const PointCloudPtr& input, pcl::PointIndices& inliers, pcl::ModelCoefficients& coefficients, double distance_threshold) {
        if (!input || input->empty() || distance_threshold <= 0.0) {
            return false;
        }
        return ransacLineFit(*input, inliers, coefficients, distance_threshold);
    }

    std::vector<Eigen::Vector3d> ransacPolyFit(const PointCloudPtr& input, int max_lines = 10, double distance_threshold = 1.0, int min_inliers = 10)
    {
        // 计算两条直线最近点
        auto computeLineLineClosestPoint = [](const pcl::ModelCoefficients& line1, const pcl::ModelCoefficients& line2) -> Eigen::Vector3d {
            Eigen::Vector3d p1(line1.values[0], line1.values[1], line1.values[2]);
            Eigen::Vector3d d1(line1.values[3], line1.values[4], line1.values[5]);
            Eigen::Vector3d p2(line2.values[0], line2.values[1], line2.values[2]);
            Eigen::Vector3d d2(line2.values[3], line2.values[4], line2.values[5]);

            Eigen::Vector3d r = p1 - p2;
            double a = d1.dot(d1);
            double b = d1.dot(d2);
            double c = d2.dot(d2);
            double d = d1.dot(r);
            double e = d2.dot(r);

            double denom = a * c - b * b;
            double s = 0, t = 0;
            if (std::abs(denom) > 1e-12) {
                s = (b * e - c * d) / denom;
                t = (a * e - b * d) / denom;
            }

            Eigen::Vector3d cp1 = p1 + s * d1;
            Eigen::Vector3d cp2 = p2 + t * d2;
            return 0.5 * (cp1 + cp2);
            };

        // 过滤距离过近点
        auto filterClosePoints = [](const std::vector<Eigen::Vector3d>& points, double minDist) -> std::vector<Eigen::Vector3d> {
            std::vector<Eigen::Vector3d> filtered;
            for (const auto& pt : points) {
                bool tooClose = false;
                for (const auto& fpt : filtered) {
                    if ((pt - fpt).norm() < minDist) {
                        tooClose = true;
                        break;
                    }
                }
                if (!tooClose) filtered.push_back(pt);
            }
            return filtered;
            };

        auto sortPointsClockwise = [](std::vector<Eigen::Vector3d>& pts) {
            if (pts.size() < 3) return;

            Eigen::Vector2d centroid(0, 0);
            for (const auto& pt : pts) {
                centroid.x() += pt.x();
                centroid.y() += pt.y();
            }
            centroid /= pts.size();

            std::vector<std::pair<double, Eigen::Vector3d>> anglePts;
            anglePts.reserve(pts.size());
            for (const auto& pt : pts) {
                double angle = std::atan2(pt.y() - centroid.y(), pt.x() - centroid.x());
                anglePts.emplace_back(angle, pt);
            }

            std::sort(anglePts.begin(), anglePts.end(), [](auto& a, auto& b) { return a.first < b.first; });

            size_t startIdx = 0;
            for (size_t i = 1; i < anglePts.size(); ++i) {
                const auto& pt = anglePts[i].second;
                const auto& minPt = anglePts[startIdx].second;
                if (pt.x() < minPt.x() || (pt.x() == minPt.x() && pt.y() < minPt.y())) {
                    startIdx = i;
                }
            }

            std::vector<Eigen::Vector3d> sorted;
            sorted.reserve(pts.size());
            for (size_t i = 0; i < anglePts.size(); ++i) {
                sorted.push_back(anglePts[(startIdx + i) % anglePts.size()].second);
            }
            pts = std::move(sorted);
            };

        // 获取点云范围
        Eigen::Vector3f min_pt, max_pt;
        getRange(input, min_pt, max_pt);

        PointCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZ>(*input));

        std::vector<pcl::ModelCoefficients> lines_coeffs;
        std::vector<pcl::PointIndices> lines_inliers;

        for (int i = 0; i < max_lines; ++i) {
            pcl::PointIndices inliers;
            pcl::ModelCoefficients coeff;

            bool success = ransacLineFit(cloud, inliers, coeff, distance_threshold);
            if (!success || (int)inliers.indices.size() < min_inliers)
                break;

            lines_coeffs.push_back(coeff);
            lines_inliers.push_back(inliers);

            PointCloudPtr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
            std::vector<bool> is_inlier(cloud->size(), false);
            for (int idx : inliers.indices) is_inlier[idx] = true;
            for (size_t idx = 0; idx < cloud->size(); ++idx)
                if (!is_inlier[idx])
                    filtered_cloud->push_back(cloud->at(idx));
            cloud.swap(filtered_cloud);

            if (cloud->size() < min_inliers)
                break;
        }

        if (lines_coeffs.size() < 2) return {};

        std::vector<Eigen::Vector3d> candidateCorners;

        for (size_t i = 0; i < lines_coeffs.size(); ++i) {
            for (size_t j = i + 1; j < lines_coeffs.size(); ++j) {
                Eigen::Vector3d interPt = computeLineLineClosestPoint(lines_coeffs[i], lines_coeffs[j]);

                // 判断交点是否在点云范围附近（这里放宽一点以防边界问题）
                constexpr double margin = 5.0; // 10个单位的容忍边界，可调
                if (interPt.x() < min_pt.x() - margin || interPt.x() > max_pt.x() + margin ||
                    interPt.y() < min_pt.y() - margin || interPt.y() > max_pt.y() + margin ||
                    interPt.z() < min_pt.z() - margin || interPt.z() > max_pt.z() + margin)
                {
                    continue; // 跳过范围外点
                }

                // 你已有的 pt2Line 判断距离
                double distToLineI = pt2Line(interPt,
                    Eigen::Vector3d(lines_coeffs[i].values[0], lines_coeffs[i].values[1], lines_coeffs[i].values[2]),
                    Eigen::Vector3d(lines_coeffs[i].values[3], lines_coeffs[i].values[4], lines_coeffs[i].values[5]));
                double distToLineJ = pt2Line(interPt,
                    Eigen::Vector3d(lines_coeffs[j].values[0], lines_coeffs[j].values[1], lines_coeffs[j].values[2]),
                    Eigen::Vector3d(lines_coeffs[j].values[3], lines_coeffs[j].values[4], lines_coeffs[j].values[5]));
                constexpr double maxDistToLine = 2.0;
                if (distToLineI < maxDistToLine && distToLineJ < maxDistToLine) {
                    candidateCorners.push_back(interPt);
                }
            }
        }

        candidateCorners = filterClosePoints(candidateCorners, 1.0);

        if (candidateCorners.size() < 3) return {};

        sortPointsClockwise(candidateCorners);

        return candidateCorners;
    }


    // ==================== 凸包计算相关 ====================

    // 3D凸包计算
    bool fitConvexHull3D(const std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& hull_pts, std::vector<Eigen::Vector3i>& faces)
    {
        if (pts.size() < 4) return false;

        // 转PCL点云
        PointCloudPtr cloud(new pcl::PointCloud<PointT>());
        cloud->reserve(pts.size());
        for (const auto& p : pts)
            cloud->push_back(PointT(p.x(), p.y(), p.z()));

        // 计算凸包
        pcl::ConvexHull<PointT> chull;
        chull.setInputCloud(cloud);
        pcl::PointCloud<PointT> pcl_hull_points;
        std::vector<pcl::Vertices> polygons;
        chull.reconstruct(pcl_hull_points, polygons);

        if (pcl_hull_points.size() < 4 || polygons.empty())
            return false;

        // 输出顶点
        hull_pts.resize(pcl_hull_points.size());
        for (size_t i = 0; i < pcl_hull_points.size(); ++i)
        {
            hull_pts[i] = Eigen::Vector3d(
                pcl_hull_points[i].x,
                pcl_hull_points[i].y,
                pcl_hull_points[i].z);
        }

        // 输出三角面片
        faces.clear();
        for (const auto& poly : polygons)
        {
            if (poly.vertices.size() == 3)
            {
                faces.emplace_back(
                    Eigen::Vector3i(poly.vertices[0], poly.vertices[1], poly.vertices[2]));
            }
        }

        return !faces.empty();
    }
    // 2D凸包计算（投影到XY平面, 若有其他平面需求可扩展）
    bool fitConvexHull2D(const std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& hull_pts, std::vector<int>& hull_indices)
    {
        if (pts.size() < 3) return false;

        // 投影到XY平面并转换为cv::Point2f
        std::vector<cv::Point2f> pts2d;
        pts2d.reserve(pts.size());
        for (const auto& p : pts)
            pts2d.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()));

        if (pts2d.size() < 3) return false;

        // 计算2D凸包（索引）
        std::vector<int> indices;
        cv::convexHull(pts2d, indices, false, false);

        if (indices.empty())
            return false;

        // 输出凸包点（保持z不变）
        hull_pts.resize(indices.size());
        hull_indices = indices;
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            hull_pts[i] = pts[idx];
        }

        return true;
    }


    // ==================== 点云分离与特征识别 ====================

    //  按指定轴分离点云（axis: "x","y","z"） 
    void separateClouds(const PointCloudPtr& input, std::vector<PointCloudPtr>& separated_clouds, const std::string& axis, double minZ)
    {
        separated_clouds.clear();
        if (!input || input->empty())
            return;

        // 用map按key分离点云
        std::map<float, PointCloudPtr> separatedMap;

        for (const auto& pt : input->points) {
            float key = 0.f;
            if (axis == "x")
                key = pt.x;
            else if (axis == "z")
                key = pt.z;
            else
                key = pt.y;

            if (pt.z <= minZ)  continue;// 超深度过滤

            if (separatedMap.find(key) == separatedMap.end()) {
                separatedMap[key] = PointCloudPtr(new PointCloud);
            }
            separatedMap[key]->points.push_back(pt);
        }

        for (auto& kv : separatedMap) {
            PointCloudPtr& pc_ptr = kv.second;
            pc_ptr->width = static_cast<uint32_t>(pc_ptr->points.size());
            pc_ptr->height = 1;
            pc_ptr->is_dense = true;
            separated_clouds.emplace_back(pc_ptr);
        }
    }
    //  旋转法识别单帧点云凹槽特征位置 
    bool grooveRecognition_Rotation(const PointCloudPtr& cloud, Eigen::Vector3f& pt_left_up, Eigen::Vector3f& pt_right_up, Eigen::Vector3f& pt_down, double ransac_dist_thresh, double rotation_angle_deg)
    {
        if (!cloud || cloud->empty()) return false;

        //  1. 统计离群点移除 
        pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
        pcl::StatisticalOutlierRemoval<PointT> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(50);
        sor.setStddevMulThresh(1.0);
        sor.filter(*cloud_filtered);
        if (cloud_filtered->empty()) return false;

        //  2. RANSAC直线拟合 
        pcl::PointIndices inliers;
        pcl::ModelCoefficients coefficients;
        bool fit_success = ransacLineFit(cloud_filtered, inliers, coefficients, ransac_dist_thresh);
        if (!fit_success) return false;
        if (inliers.indices.size() < 10) return false;

        //  3. 计算旋转变换（绕Y轴）使直线方向与X轴对齐 
        Eigen::Vector3d line_dir(coefficients.values[3], coefficients.values[4], coefficients.values[5]);
        line_dir.normalize();
        Eigen::Vector3f line_dir_f = line_dir.cast<float>();

        float angle = std::atan2(line_dir_f.z(), line_dir_f.x());
        Eigen::AngleAxisf angle_axis(angle, Eigen::Vector3f::UnitY());
        Eigen::Matrix3f rotation = angle_axis.toRotationMatrix();

        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation;

        pcl::PointCloud<PointT>::Ptr cloud_rotated(new pcl::PointCloud<PointT>);
        pcl::transformPointCloud(*cloud, *cloud_rotated, transform);

        //  4. 找最低点  
        int valid_min_idx = -1;
        float valid_min_z = std::numeric_limits<float>::max();

        // 按z排序，找前20个点中第一个 z >= -100 的点
        std::vector<std::pair<int, float>> idx_z_vec;
        idx_z_vec.reserve(cloud_rotated->size());
        for (int i = 0; i < static_cast<int>(cloud_rotated->size()); ++i)
            idx_z_vec.emplace_back(i, cloud_rotated->points[i].z);

        std::sort(idx_z_vec.begin(), idx_z_vec.end(),
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second < b.second;
            });

        int limit = std::min(20, static_cast<int>(idx_z_vec.size()));
        for (int i = 0; i < limit; ++i) {
            if (idx_z_vec[i].second >= -100.f) {
                valid_min_idx = idx_z_vec[i].first;
                valid_min_z = idx_z_vec[i].second;
                break;
            }
        }
        if (valid_min_idx == -1 || valid_min_idx == 0) return false; // 异常或超深度

        // 最低点（在旋转点云中）
        PointT low_pt_rot = cloud_rotated->points[valid_min_idx];
        pt_down = low_pt_rot.getVector3fMap();

        //  5. 分割左右点云（以最低点x为分界） 
        std::vector<int> left_indices, right_indices;
        for (size_t i = 0; i < cloud_rotated->size(); ++i) {
            if (cloud_rotated->points[i].x < low_pt_rot.x) left_indices.push_back((int)i);
            else right_indices.push_back((int)i);
        }
        if (left_indices.empty() || right_indices.empty()) return false;

        // 构造左右子点云
        pcl::PointCloud<PointT>::Ptr left_cloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr right_cloud(new pcl::PointCloud<PointT>);
        left_cloud->resize(left_indices.size());
        right_cloud->resize(right_indices.size());
        for (size_t i = 0; i < left_indices.size(); ++i)
            left_cloud->points[i] = cloud_rotated->points[left_indices[i]];
        for (size_t i = 0; i < right_indices.size(); ++i)
            right_cloud->points[i] = cloud_rotated->points[right_indices[i]];

        //  6. 旋转左右点云（绕Y轴，角度为±rotation_angle_deg） 
        float rad_left = -rotation_angle_deg * static_cast<float>(M_PI) / 180.f;
        float rad_right = rotation_angle_deg * static_cast<float>(M_PI) / 180.f;

        Eigen::Matrix3f R_left, R_right;
        R_left << cos(rad_left), 0, sin(rad_left), 0, 1, 0, -sin(rad_left), 0, cos(rad_left);
        R_right << cos(rad_right), 0, sin(rad_right), 0, 1, 0, -sin(rad_right), 0, cos(rad_right);

        pcl::PointCloud<PointT>::Ptr left_cloud_rotated(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr right_cloud_rotated(new pcl::PointCloud<PointT>);
        left_cloud_rotated->resize(left_cloud->size());
        right_cloud_rotated->resize(right_cloud->size());

        for (size_t i = 0; i < left_cloud->size(); ++i) {
            Eigen::Vector3f pt = left_cloud->points[i].getVector3fMap();
            pt = R_left * pt;
            left_cloud_rotated->points[i].x = pt.x();
            left_cloud_rotated->points[i].y = pt.y();
            left_cloud_rotated->points[i].z = pt.z();
        }
        for (size_t i = 0; i < right_cloud->size(); ++i) {
            Eigen::Vector3f pt = right_cloud->points[i].getVector3fMap();
            pt = R_right * pt;
            right_cloud_rotated->points[i].x = pt.x();
            right_cloud_rotated->points[i].y = pt.y();
            right_cloud_rotated->points[i].z = pt.z();
        }

        //  7. 找左右点云中z最大点作为左右上点 
        auto findMaxZPoint = [](const pcl::PointCloud<PointT>::Ptr& pCloud) -> PointT {
            if (pCloud->empty()) return PointT();
            PointT max_pt = pCloud->points[0];
            for (const auto& pt : pCloud->points) {
                if (pt.z > max_pt.z)
                    max_pt = pt;
            }
            return max_pt;
            };

        PointT left_max_z_rot = findMaxZPoint(left_cloud_rotated);
        PointT right_max_z_rot = findMaxZPoint(right_cloud_rotated);

        //  8. 将关键点从旋转坐标系映射回原始坐标系（旋转矩阵转置） 
        Eigen::Matrix3f rot_inv = rotation.transpose();

        // 因为旋转是绕原点，点云未平移，直接做逆旋转即可
        pt_left_up = rot_inv * left_max_z_rot.getVector3fMap();
        pt_right_up = rot_inv * right_max_z_rot.getVector3fMap();

        // pt_down已经是旋转后的，逆旋转回原始坐标系
        pt_down = rot_inv * pt_down;

        return true;
    }
	//  依据点云分布自适应计算卷积核大小
    int getKernelSize(const PointCloudPtr& cloud, int bin_count, double k) {
        Eigen::VectorXf input_source_cloud_z(cloud->points.size());
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            input_source_cloud_z(i) = cloud->points[i].z;
        }
        float z_min = input_source_cloud_z.minCoeff(); // z坐标的最小值
        float z_max = input_source_cloud_z.maxCoeff(); // z坐标的最大值
        float bin_width = (z_max - z_min) / bin_count;

        // 创建一个vector存储每个bin的点数
        std::vector<int> histogram(bin_count, 0);

        // 填充直方图
        for (int i = 0; i < input_source_cloud_z.size(); ++i) {
            int bin_index = static_cast<int>((input_source_cloud_z[i] - z_min) / bin_width);
            if (bin_index >= 0 && bin_index < bin_count) {
                histogram[bin_index]++;
            }
        }

        // 定义计算均值的lambda
        auto calculateMean = [](const std::vector<int>& histogram) -> double {
            double sum = std::accumulate(histogram.begin(), histogram.end(), 0.0);
            return sum / histogram.size();
            };

        // 定义计算标准差的lambda，传入mean
        auto calculateStandardDeviation = [](const std::vector<int>& histogram, double mean) -> double {
            double variance = 0.0;
            for (int count : histogram) {
                variance += (count - mean) * (count - mean);
            }
            variance /= histogram.size();
            return std::sqrt(variance);
            };

        // 计算均值和标准差
        double mean = calculateMean(histogram);
        double std_dev = calculateStandardDeviation(histogram, mean);

        // 动态计算稀疏区间的阈值
        double sparse_threshold = mean - k * std_dev;

        // 累加点数小于稀疏阈值的区间的点数
        int sparse_point_count = 0;
        for (int i = 0; i < bin_count; ++i) {
            if (histogram[i] < sparse_threshold) { // 判断是否是稀疏区间
                sparse_point_count += histogram[i];
            }
        }
        int kernel_size = sparse_point_count / 2 + 10; // 除最密集区间点数之和 + 10
        return std::max(kernel_size, 35);
    }
	//  滑动卷积法识别单帧点云凹槽特征位置
    bool grooveRecognition_Convolution(const PointCloudPtr& cloud, Eigen::Vector3f& pt_left_up, Eigen::Vector3f& pt_right_up, Eigen::Vector3f& pt_left_down, Eigen::Vector3f& pt_right_down, int kernel_size, double ransac_dist_thresh, double rotation_angle_deg) {
        if (!cloud || cloud->empty()) {
            return false;
        }
        Eigen::VectorXf input_source_cloud_z(cloud->points.size());
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            input_source_cloud_z(i) = cloud->points[i].z;
        }
        Eigen::Index min_index;
        float min_z = input_source_cloud_z.minCoeff(&min_index); // 获取最小z值及其索引
        Eigen::VectorXf z_vals = input_source_cloud_z;
        std::nth_element(z_vals.data(), z_vals.data() + 2, z_vals.data() + z_vals.size());
        float third_min_z = z_vals(2); // 取得第三小值
        input_source_cloud_z = input_source_cloud_z.array().max(third_min_z);// 替换小于等于第三小值的z坐标，避免归一化时出现错误

        Eigen::VectorXf input_source_cloud_x = Eigen::VectorXf::LinSpaced(cloud->points.size(), -1, 1);// 创建均匀分布的 x 向量，用于旋转
        Eigen::Index center_index = static_cast<Eigen::Index>(std::round(input_source_cloud_z.size() / 2.0));// 定义索引中心，向下舍入
		float offset_distance = center_index - min_index; // 计算最小值索引与中心索引的偏移距离
        float cz = min_z + 3 * offset_distance / 4;
        Eigen::VectorXf rotated_z(input_source_cloud_z.size());
        Eigen::VectorXf shifted_z = input_source_cloud_z.array() - cz;
        float rad_left = -rotation_angle_deg * static_cast<float>(M_PI) / 180.f;
        float rad_right = rotation_angle_deg * static_cast<float>(M_PI) / 180.f;
        Eigen::Matrix2f R_left, R_right;
        R_left << cos(rad_left), sin(rad_left), -sin(rad_left), cos(rad_left);
        R_right << cos(rad_right), sin(rad_right), -sin(rad_right), cos(rad_right);
        for (size_t i = 0; i <= min_index; ++i) {
            rotated_z(i) = R_left(1, 1) * shifted_z(i) + R_left(1, 0) * input_source_cloud_x(i); // z 变换
        }
        for (size_t i = min_index + 1; i < cloud->points.size(); ++i) {
            rotated_z(i) = R_right(1, 1) * shifted_z(i) + R_right(1, 0) * input_source_cloud_x(i); // z 变换
        }
		rotated_z.array() += cz; // 平移回原始位置

		// 第一次卷积运算
        Eigen::VectorXf conv_kernel = Eigen::VectorXf::Zero(kernel_size);// 创建一个全零的向量

        int elements_count = 5; // 开头非零元素的数量
        for (int i = 0; i < elements_count && i < kernel_size; ++i) {
            conv_kernel[i] = 1.0f;
        }
        for (int i = 0; i < elements_count && kernel_size - elements_count + i < kernel_size; ++i) {
            conv_kernel[kernel_size - elements_count + i] = -1.0f * i / (elements_count - 1);// 设置结尾的非零元素为 [0, -1] 的等距，下方是倾斜向下的
        }
        Eigen::VectorXf conv_results = Eigen::VectorXf::Zero(rotated_z.size() - conv_kernel.size() + 1);// 初始化卷积结果数组
        std::vector<int> non_zero_indices;
        for (int i = 0; i < kernel_size; ++i) {
            if (conv_kernel(i) != 0) {
                non_zero_indices.push_back(i);// 获取非零元素的索引
            }
        }
        for (int i = 0; i < conv_results.size(); ++i) {
            for (int idx : non_zero_indices) {
                conv_results(i) += rotated_z(i + idx) * conv_kernel(idx);// 进行稀疏卷积运算，只对非零元素的部分进行计算
            }
        }
        auto max_it = std::max_element(conv_results.data(), conv_results.data() + conv_results.size());
        auto min_it = std::min_element(conv_results.data(), conv_results.data() + conv_results.size());
        int max_idx = std::distance(conv_results.data(), max_it);// 获取最大值和最小值的索引
        int min_idx = std::distance(conv_results.data(), min_it);
        Eigen::Vector2i max_conv_position(max_idx, max_idx + kernel_size - 1);// 定义最大和最小卷积块
        Eigen::Vector2i min_conv_position(min_idx, min_idx + kernel_size - 1);
        size_t left_index = max_conv_position(0);
        size_t right_index = min_conv_position(0);

        if (left_index < 0) left_index = max_conv_position(0);
        if (left_index >= cloud->points.size()) left_index = cloud->points.size() - 1;
        if (right_index < 0) right_index = 0;
        if (right_index >= cloud->points.size()) right_index = min_conv_position(1);

		pt_left_up.x() = cloud->points[left_index].x;
		pt_left_up.y() = cloud->points[left_index].y;
		pt_left_up.z() = cloud->points[left_index].z;
		pt_right_down.x() = cloud->points[right_index].x;
		pt_right_down.y() = cloud->points[right_index].y;
        pt_right_down.z() = cloud->points[right_index].z;

		// 第二次卷积运算
        conv_kernel = Eigen::VectorXf::Zero(kernel_size);
        Eigen::VectorXf conv_results2 = Eigen::VectorXf::Zero(rotated_z.size() - conv_kernel.size() + 1);

        for (int i = 0; i < elements_count && i < kernel_size; ++i) {
            conv_kernel[i] = 1.0f - 1.0f * i / (elements_count - 1); // 从 1 等距递减到 0
        }
        for (int i = 0; i < elements_count && kernel_size - elements_count + i < kernel_size; ++i) {
            conv_kernel[kernel_size - elements_count + i] = -1.0f; // 直接设置为 -1
        }

        for (int i = 0; i < conv_results2.size(); ++i) {
            for (int idx : non_zero_indices) {
                conv_results2(i) += rotated_z(i + idx) * conv_kernel(idx);
            }
        }
        max_it = std::max_element(conv_results2.data(), conv_results2.data() + conv_results2.size());
        min_it = std::min_element(conv_results2.data(), conv_results2.data() + conv_results2.size());
        max_idx = std::distance(conv_results2.data(), max_it);
        min_idx = std::distance(conv_results2.data(), min_it);
        Eigen::Vector2i max_conv_position2(max_idx, max_idx + kernel_size - 1);
        Eigen::Vector2i min_conv_position2(min_idx, min_idx + kernel_size - 1);
        left_index = max_conv_position2(1);
        right_index = min_conv_position2(1);

        if (left_index < 0) left_index = max_conv_position(0);
        if (left_index >= cloud->points.size()) left_index = cloud->points.size() - 1;
        if (right_index < 0) right_index = 0;
        if (right_index >= cloud->points.size()) right_index = min_conv_position(1);

		pt_right_up.x() = cloud->points[right_index].x;
        pt_right_up.y() = cloud->points[right_index].y;
		pt_right_up.z() = cloud->points[right_index].z;
        pt_left_down.x() = cloud->points[left_index].x;
        pt_left_down.y() = cloud->points[left_index].y;
        pt_left_down.z() = cloud->points[left_index].z;
		return true;
    }


    // ==================== 点云图像化 ====================

    //  点云转灰度图（z值映射为灰度，x,y按范围均匀分布）
    cv::Mat cloudToGrayImage(const PointCloud& cloud, int width, int height, float z_min, float z_max, float margin_ratio)
    {
        if (cloud.empty() || width <= 0 || height <= 0)
            return cv::Mat();

        // 计算点云x,y范围
        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_min = std::numeric_limits<float>::max();
        float y_max = std::numeric_limits<float>::lowest();

        for (const auto& pt : cloud.points) {
            if (std::isfinite(pt.x)) {
                if (pt.x < x_min) x_min = pt.x;
                if (pt.x > x_max) x_max = pt.x;
            }
            if (std::isfinite(pt.y)) {
                if (pt.y < y_min) y_min = pt.y;
                if (pt.y > y_max) y_max = pt.y;
            }
        }

        if (x_max <= x_min || y_max <= y_min)
            return cv::Mat();

        // 计算范围和边距
        float range_x = x_max - x_min;
        float range_y = y_max - y_min;

        // 扩展边距，保证比例正确
        float margin_x = range_x * margin_ratio;
        float margin_y = range_y * margin_ratio;

        // 扩大范围
        x_min -= margin_x;
        x_max += margin_x;
        y_min -= margin_y;
        y_max += margin_y;

        // 重新计算扩展后范围
        range_x = x_max - x_min;
        range_y = y_max - y_min;

        // 创建灰度图，初始化为0，8位单通道
        cv::Mat img = cv::Mat::zeros(height, width, CV_8UC1);

        // 计数矩阵，求均值
        cv::Mat count = cv::Mat::zeros(height, width, CV_32SC1);
        cv::Mat sum_z = cv::Mat::zeros(height, width, CV_32FC1);

        for (const auto& pt : cloud.points) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
                continue;

            // 归一化映射x,y到[0,width-1]和[0,height-1]
            int px = static_cast<int>((pt.x - x_min) / range_x * (width - 1));
            int py = static_cast<int>((pt.y - y_min) / range_y * (height - 1));
            // y轴翻转，保证y向下
            py = height - 1 - py;

            if (px >= 0 && px < width && py >= 0 && py < height) {
                sum_z.at<float>(py, px) += pt.z;
                count.at<int>(py, px) += 1;
            }
        }

        // 计算平均z值映射为灰度
        for (int r = 0; r < height; ++r) {
            for (int c = 0; c < width; ++c) {
                int cnt = count.at<int>(r, c);
                if (cnt > 0) {
                    float avg_z = sum_z.at<float>(r, c) / cnt;
                    float norm_z = (avg_z - z_min) / (z_max - z_min);
                    norm_z = std::min(std::max(norm_z, 0.f), 1.f);
                    img.at<uchar>(r, c) = static_cast<uchar>(norm_z * 255);
                }
                else {
                    img.at<uchar>(r, c) = 0;
                }
            }
        }

        return img;
    }
    cv::Mat cloudToGrayImage(const PointCloudPtr& cloud, int width, int height, float z_min, float z_max, float margin_ratio)
    {
        if (!cloud)
            return cv::Mat();
        return cloudToGrayImage(*cloud, width, height, z_min, z_max, margin_ratio);
    }
    //  点云转二值图（z值映射为灰度后自动阈值二值化）
    void cloudToBinary(const PointCloud& cloud, cv::Mat& binary, int width, int height, float z_min, float z_max)
    {
        cv::Mat gray = cloudToGrayImage(cloud, width, height, z_min, z_max);
        if (gray.empty()) {
            binary.release();
            return;
        }
        // 自动阈值二值化
        cv::threshold(gray, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    }
    void cloudToBinary(const PointCloudPtr& cloud, cv::Mat& binary, int width, int height, float z_min, float z_max)
    {
        if (!cloud) {
            binary.release();
            return;
        }
        cloudToBinary(*cloud, binary, width, height, z_min, z_max);
    }


    // ==================== 点云曲线处理 ====================

    // Douglas-Peucker递归辅助
    static void douglasPeuckerRecursive(const std::vector<Eigen::Vector3d>& points, int start_idx, int end_idx, double epsilon, std::vector<int>& keep_indices)
    {
        if (end_idx <= start_idx + 1)
            return;

        double max_dist = -1.0;
        int index = start_idx;

        for (int i = start_idx + 1; i < end_idx; ++i)
        {
            double dist = pt2LineSegment(points[i], points[start_idx], points[end_idx]);
            if (dist > max_dist)
            {
                max_dist = dist;
                index = i;
            }
        }

        if (max_dist > epsilon)
        {
            keep_indices.push_back(index);
            douglasPeuckerRecursive(points, start_idx, index, epsilon, keep_indices);
            douglasPeuckerRecursive(points, index, end_idx, epsilon, keep_indices);
        }
        // 否则舍弃中间点，使用起止点直线代替
    }
    // Douglas-Peucker主入口
    std::vector<Eigen::Vector3d> douglasPeucker(const std::vector<Eigen::Vector3d>& points, double epsilon)
    {
        if (points.size() < 3)
            return points;

        bool is_closed = ptsDistance(points.front(), points.back()) < 1e-6;

        std::vector<Eigen::Vector3d> open_points;
        if (is_closed) open_points.assign(points.begin(), points.end() - 1);// 移除最后一个点，断开闭合
        else open_points = points;

        if (open_points.size() < 3)
            return points;

        std::vector<int> keep_indices;
        keep_indices.reserve(open_points.size());
        keep_indices.push_back(0);
        keep_indices.push_back(static_cast<int>(open_points.size() - 1));

        douglasPeuckerRecursive(open_points, 0, static_cast<int>(open_points.size() - 1), epsilon, keep_indices);

        std::sort(keep_indices.begin(), keep_indices.end());

        std::vector<Eigen::Vector3d> result;
        result.reserve(keep_indices.size() + (is_closed ? 1 : 0));

        for (int idx : keep_indices)
        {
            result.push_back(open_points[idx]);
        }

        if (is_closed) result.push_back(result.front());// 如果原始闭合，加回首点闭合

        return result;
    }
    // 平滑曲线（均值滤波）
    std::vector<Eigen::Vector3f> smoothCurve(const std::vector<Eigen::Vector3f>& points, int window_size) {
        const int n = static_cast<int>(points.size());
        if (n == 0 || window_size < 1) return points;
        if (window_size > n) window_size = n;
        if (window_size % 2 == 0) ++window_size; // 强制奇数窗口

        std::vector<Eigen::Vector3f> result(n, Eigen::Vector3f::Zero());
        int half = window_size / 2;

        for (int i = 0; i < n; ++i) {
            int left = std::max(0, i - half);
            int right = std::min(n - 1, i + half);
            int count = right - left + 1;

            Eigen::Vector3f sum = Eigen::Vector3f::Zero();
            for (int j = left; j <= right; ++j) {
                sum += points[j];
            }
            result[i] = sum / static_cast<float>(count);
        }
        return result;
    }
    // 高斯加权平滑曲线
    std::vector<Eigen::Vector3f> smoothCurveGaussian(const std::vector<Eigen::Vector3f>& points, int window_size, double sigma) {
        const int n = static_cast<int>(points.size());
        if (n == 0 || window_size < 1) return points;
        if (window_size > n) window_size = n;
        if (window_size % 2 == 0) ++window_size; // 确保奇数窗口
        if (sigma <= 0) sigma = window_size / 3.0; // 经验值

        std::vector<Eigen::Vector3f> result(n, Eigen::Vector3f::Zero());
        int half = window_size / 2;

        std::vector<double> weights(window_size);
        double sum_weights = 0.0;
        for (int i = 0; i < window_size; ++i) {
            int offset = i - half;
            double w = std::exp(-(offset * offset) / (2.0 * sigma * sigma));
            weights[i] = w;
            sum_weights += w;
        }
        for (auto& w : weights) w /= sum_weights;

        for (int i = 0; i < n; ++i) {
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            double w_sum = 0.0;

            int left = std::max(0, i - half);
            int right = std::min(n - 1, i + half);

            for (int j = left; j <= right; ++j) {
                int w_idx = j - (i - half);
                if (w_idx < 0 || w_idx >= window_size) continue;

                sum += weights[w_idx] * points[j].cast<double>();
                w_sum += weights[w_idx];
            }

            if (w_sum > 1e-12) sum /= w_sum;
            result[i] = sum.cast<float>();
        }
        return result;
    }
}
