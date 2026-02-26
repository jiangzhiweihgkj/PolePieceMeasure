#include "PolePieceMeasureWin.h" 
   
 void PolePieceMeasureWin::roi_Setting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 相对宽度百分比 roiPercent
    double roiPercent = InfoSuanFa.editRoiPercent.toDouble();//ui->editROI->text().toDouble();
    roiPercent = std::max(1.0, std::min(roiPercent, 100.0)); // 限制1~100

    // 中心位置百分比 centerPercent
    double centerPercent = InfoSuanFa.editCentrePoint.toDouble(); //ui->editROI_2->text().toDouble();
    centerPercent = std::max(0.0, std::min(centerPercent, 100.0)); // 限制0~100

    if (!cloud->empty())
    {
        double xMin = std::numeric_limits<double>::max();
        double xMax = -std::numeric_limits<double>::max();
        for (const auto& pt : cloud->points) {
            xMin = std::min(xMin, static_cast<double>(pt.x));
            xMax = std::max(xMax, static_cast<double>(pt.x));
        }

        double xRange = xMax - xMin;
        double roiRatio = roiPercent / 100.0;
        double centerRatio = centerPercent / 100.0;

        double roiWidth = roiRatio * xRange;
        double roiCenter = xMin + centerRatio * xRange;

        // 计算ROI起点和终点
        double roiStart = roiCenter - roiWidth / 2.0;
        double roiEnd = roiCenter + roiWidth / 2.0;

        // 清空之前点
        source_cloud->clear();

         //过滤符合条件的点
        for (const auto& pt : cloud->points) {
            if (pt.x >= roiStart && pt.x <= roiEnd) {
                source_cloud->points.push_back(pt);
            }
        }
    }
}
 void  PolePieceMeasureWin::Multify_Recognition(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud)
{
     pointsClear();
     for (int j=0; j < cloud.size(); ++j)
     {
         int Nret = Single_Recognition(cloud[j]);
         if (Nret == 0)
         {
             ID.push_back(static_cast<int>(j));
         }
         if (InfoResult_Calculate()!=0)
         {
             RecordData(QStringLiteral("点云识别错误"),2);
             return;
         }   
     }
     auto ToPoint2Data = [](double v) -> QString {
         std::ostringstream o;
         o << std::fixed << std::setprecision(4) << v << " mm";
         return QString::fromStdString(o.str());
         };
     QString meanWidth = ToPoint2Data(InfoResult.meanWidth);
     QString maxWidth = ToPoint2Data(InfoResult.maxWidth);
     QString minWidth = ToPoint2Data(InfoResult.minWidth);
     ui->labelMeanWidth->setText(meanWidth);
     ui->labelMaxWidth->setText(maxWidth);
     ui->labelMinWidth->setText(minWidth);

     QString meanDepth = ToPoint2Data(InfoResult.meanDepth);
     QString maxDepth = ToPoint2Data(InfoResult.maxDepth);
     QString minDepth = ToPoint2Data(InfoResult.minDepth);
     ui->labelMeanDepth->setText(meanDepth);
     ui->labelMaxDepth->setText(maxDepth);
     ui->labelMinDepth->setText(minDepth);
}
 int  PolePieceMeasureWin::Single_Recognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if (!cloud || cloud->empty())
    {
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_filter = *cloud;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//对点云进行 统计离群点移除;创建统计离群点移除滤波器对象
    sor.setInputCloud(cloud_filter);//设置输入点云为 cloud_filtered
    sor.setMeanK(50);//// 计算每个点邻域点数;用于均值估计的邻居点数量为 50。
    sor.setStddevMulThresh(1.0);//// 标准差倍数阈值，越小越严格;越小（如 0.5）越严格，越多点会被视为离群点；越大（如 2.0）越宽松，保留的点越多。
    sor.filter(*cloud_filter);//执行滤波操作，结果直接覆盖写入 cloud_filtered。
//#ifdef _DEBUG
//    if (!cloud_filter->empty())
//    {
//       /* pcl::PointCloud<pcl::PointXYZ>::Ptr debugCloud(new pcl::PointCloud<pcl::PointXYZ>());
//        for (const auto& pt : *cloud_filter)
//            debugCloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));*/
//
//        //把 cloud_filter 中的点拷贝到 debugCloud，然后用 PCLVisualizer 可视化
//        pcl::PointCloud<pcl::PointXYZ>::Ptr debugCloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud_filter));
//        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Debug"));
//        //addPointCloud 默认白色点，背景也是黑色，可能看不清。你可以设置颜色：
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(debugCloud, 0, 255, 0); // 绿色
//        viewer->addPointCloud(debugCloud, color_handler, "Cloud");
//      //  viewer->setBackgroundColor(0, 0, 0);
//       // viewer->addPointCloud(debugCloud, "Cloud");
//
//        // 添加坐标系，参数是坐标轴长度，单位与点云单位保持一致
//        float axis_length = 1.0; // 根据你的点云尺度调整
//        viewer->addCoordinateSystem(axis_length);
//
//        viewer->spin();
//
//    }
//#endif
    // RANSAC直线拟合；用 RANSAC 从滤波后的点云中 拟合直线模型
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    double dist = InfoSuanFa.editRANSAC.toDouble(); // 直线拟合距离阈值//ui->editRANSAC->text().toDouble(); // 直线拟合距离阈值
    if (dist < 0.01)//加保护，防止操作人员设置数据超限
        dist = 0.01;
    if (dist > 100)
        dist = 100;
    //RANSAC 每次都会随机抽几个点去“猜”一条直线，猜完再数有多少点支持这条直线。
    // ---如果 5000 次里都没猜到一个足够好的解，就停手，把当前最好的那条线当作结果。
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);//使用 SACMODEL_LINE 模型，
    seg.setMethodType(pcl::SAC_RANSAC);//RANSAC 方法；
    seg.setDistanceThreshold(dist);//设置距离阈值 dist（用户输入）；
    //点云只有几千个点	1000 次以内即可
    seg.setMaxIterations(3000);//(5000);  // 最大迭代次数；1、抽到“全是无效点”的概率高，可能永远凑不齐一条合格直线。2、没有上限，线程会一直被占着，GUI 无响应。3、用户输入的阈值太苛刻，比如 dist = 0.001，几乎没点能落在直线上，算法就一直在“瞎猜”。
    seg.setInputCloud(cloud_filter);
    seg.segment(*inliers, *coefficients);

    if (coefficients->values.size() < 6) {
        //ui->statusLabel->setAnimatedText(QStringLiteral("RANSAC拟合失败"));
        return -1; // 拟合失败
    }
    if (inliers->indices.size() < 10) {
        //ui->statusLabel->setAnimatedText(QStringLiteral("内点数过少"));
        return -1; // 内点数过少
    }
    //提取直线方向向量 
    // 粗拟合直线方向（近似主方向）
    Eigen::Vector3d line_dir(coefficients->values[3], coefficients->values[4], coefficients->values[5]);//表示直线的初始方向
    line_dir.normalize();//通过normalize()方法将line_dir转换为单位向量（模长为 1），确保方向向量的一致性。
    //将双精度向量line_dir的三个分量赋值给单精度向量line_dir_f（Eigen::Vector3f），并再次调用normalize()。
    Eigen::Vector3f line_dir_f(line_dir[0], line_dir[1], line_dir[2]); // double->float转换
    line_dir_f.normalize();

    // 计算夹角阈值的余弦值
    Eigen::Vector3f ref_dir(1.0f, 0.0f, 0.0f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //这几行代码就是把“斜着的直线”扳成“水平朝 + X”，方便后续对齐、裁切或测量长度。
    float angle = std::atan2(line_dir_f.z(), line_dir_f.x());
    Eigen::AngleAxisf angle_axis(angle, Eigen::Vector3f::UnitY());
    Eigen::Matrix3f rotation = angle_axis.toRotationMatrix();// 绕 Y 轴旋转矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // 构造齐次变换矩阵
    transform.block<3, 3>(0, 0) = rotation;
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height=1;
    cloud->is_dense=true;//声明“不存在 NaN/Inf 无效点”，后续算法可跳过 NaN 检查，加速运算。

    // 应用变换
    pcl::transformPointCloud(*cloud, *Cloud, transform);

    // 找到z坐标最小点及其索引
        // 找到z坐标最小点及其索引
    int min_z_idx = 0;
    float min_z_val = Cloud->points[0].z;//“把点云 Cloud 中第 0 个点的 z 坐标取出来，当成当前的最小 z 值。”
    //能存储 “整数索引 + 浮点型 z 值” 组合的动态列表，方便按顺序管理或访问这些关联数据。
    std::vector<std::pair<int, float>> idx_z_vec;
    idx_z_vec.reserve(Cloud->size());
    //这段代码的作用是遍历点云数据（Cloud），将每个点的索引（i）和该点的 z 坐标值组成键值对，存入之前定义的 idx_z_vec 容器中。
    for (int i = 0; i < static_cast<int>(Cloud->size()); ++i) {
        idx_z_vec.emplace_back(i, Cloud->points[i].z);
    }
    //这段代码的作用是对 idx_z_vec 容器中的元素进行排序，排序规则是按照每个元素中 float 类型的第二个值（即点的 z 坐标）从小到大升序排列。
    //idx_z_vec.begin() 和 idx_z_vec.end()：指定排序的范围（整个容器）；
    //自定义排序规则的lambda 表达式（匿名函数）
    std::sort(idx_z_vec.begin(), idx_z_vec.end(),
        [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
            return a.second < b.second;//返回 a.second < b.second 意味着：当 a 的 z 坐标小于 b 的 z 坐标时，a 应该排在 b 前面（升序排序）。
        });
    // 遍历前20个最小点，找到第一个 z >= -100 的点
    int valid_min_idx = -1;
    float valid_min_val = 0.0f;
    int limit = std::min(20, static_cast<int>(idx_z_vec.size()));
    for (int i = 0; i < limit; ++i)
    {
        int idx = idx_z_vec[i].first;
        float z_val = idx_z_vec[i].second;
        if (z_val >= -100.0f) {
            valid_min_idx = idx;
            valid_min_val = z_val;
            break;
        }
    }
    if (valid_min_idx == -1) {
      //  ui->statusLabel->setAnimatedText(QStringLiteral("点云超深度"));
        return -4; // 超深度帧
    }
    else if (valid_min_idx == 0) {
       // ui->statusLabel->setAnimatedText(QStringLiteral("最低点位置异常（平板）"));
        return -3; // 最小点位置异常(平板)
    }
    else {
        // 有效最小点正常，更新对应索引和值
        min_z_idx = valid_min_idx;
        min_z_val = valid_min_val;
    }
    pcl::PointXYZ min_z_point = cloud->points[min_z_idx];
    // 分割点云，记录原始索引
    std::vector<int> left_indices, right_indices;
    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (cloud->points[i].x < min_z_point.x) {
            left_indices.push_back(static_cast<int>(i));
        }
        else {
            right_indices.push_back(static_cast<int>(i));
        }

    }
    // 构造左右子点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_Cloud(new pcl::PointCloud<pcl::PointXYZ>);
    left_Cloud->resize(left_indices.size());//分配存储空间
    right_Cloud->resize(right_indices.size());
    for (size_t i = 0; i < left_indices.size(); ++i)//根据存储的索引将原始点云中的对应索引点 存储到左子点云；
        left_Cloud->points[i] = Cloud->points[left_indices[i]];
    for (size_t i = 0; i < right_indices.size(); ++i)//根据存储的索引将原始点云中的对应索引点 存储到左子点云；
        right_Cloud->points[i] = Cloud->points[right_indices[i]];

    // 定义旋转角度（度）及旋转矩阵（绕Y轴，xz平面旋转）
    float rotation_angle_deg = InfoSuanFa.editAngle.toDouble();// ui->editAngle->text().toDouble();
    if (rotation_angle_deg < 5.0) rotation_angle_deg = 5.0;
    if (rotation_angle_deg > 90.0) rotation_angle_deg = 90.0;

    float rad = -rotation_angle_deg * M_PI / 180.0f;
    //用 Eigen 库定义并初始化一个 3×3 的旋转矩阵 R_left，它表示绕 Y 轴 旋转角度为 rad 的旋转矩阵。
    Eigen::Matrix3f R_left, R_right;
    R_left << cos(rad), 0, sin(rad),//θ = rad，表示向左旋转（假设 rad 为正值时，绕 y 轴逆时针旋转）。
        0, 1, 0,
        -sin(rad), 0, cos(rad);

    R_right << cos(-rad), 0, sin(-rad),//θ = rad，表示向右旋转（假设 rad 为负值时，绕 y 轴顺时针旋转）。
        0, 1, 0,
        -sin(-rad), 0, cos(-rad);
    // 对左右子点云分别旋转（绕 min_z_point 点）
    pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud_rotated(new pcl::PointCloud<pcl::PointXYZ>);
    left_cloud_rotated->resize(left_Cloud->size());
    right_cloud_rotated->resize(right_Cloud->size());

    for (size_t i = 0; i < left_Cloud->size(); ++i) {
        Eigen::Vector3f pt = left_Cloud->points[i].getVector3fMap();//这是 PCL 提供的一个方法，用于将点的 x, y, z 数据 映射为 Eigen 的 3D 向量类型（即 Eigen::Vector3f），不拷贝数据，只是做映射。
        pt = R_left * pt;
        left_cloud_rotated->points[i].x = pt.x();
        left_cloud_rotated->points[i].y = pt.y();
        left_cloud_rotated->points[i].z = pt.z();
    }
    for (size_t i = 0; i < right_Cloud->size(); ++i) {
        Eigen::Vector3f pt = right_Cloud->points[i].getVector3fMap();
        pt = R_right * pt;
        right_cloud_rotated->points[i].x = pt.x();
        right_cloud_rotated->points[i].y = pt.y();
        right_cloud_rotated->points[i].z = pt.z();
    }
    // 找旋转后左右点云中 z 最大的点索引及坐标
    int left_max_z_idx = 0;
    float left_max_z = left_cloud_rotated->points[0].z;
    for (size_t i = 1; i < left_cloud_rotated->size(); ++i) {
        if (left_cloud_rotated->points[i].z > left_max_z) {//通过层层遍历比较，直到找到Z最大值，并把z值最大的时候的索引保存下来；
            left_max_z = left_cloud_rotated->points[i].z;
            left_max_z_idx = static_cast<int>(i);
        }
    }
    pcl::PointXYZ left_max_z_point = left_cloud_rotated->points[left_max_z_idx];


    int right_max_z_idx = 0;
    float right_max_z = right_cloud_rotated->points[0].z;
    for (size_t i = 1; i < right_cloud_rotated->size(); ++i) {
        if (right_cloud_rotated->points[i].z > right_max_z) {//通过层层遍历比较，直到找到Z最大值，并把z值最大的时候的索引保存下来；
            right_max_z = right_cloud_rotated->points[i].z;
            right_max_z_idx = static_cast<int>(i);
        }
    }
    pcl::PointXYZ right_max_z_point = right_cloud_rotated->points[right_max_z_idx];

    // 映射回原始点云坐标（用分割时记录的索引）
    pcl::PointXYZ left_max_z_point_original= cloud->points[left_indices[left_max_z_idx]];
    pcl::PointXYZ right_max_z_point_original = cloud->points[right_indices[right_max_z_idx]];
    // 左上特征点（红色）绿色
    pcl::PointXYZRGB pt_left;
    pt_left.x = left_max_z_point_original.x;
    pt_left.y = left_max_z_point_original.y;
    pt_left.z = left_max_z_point_original.z;
    pt_left.r = 0; pt_left.g = 255; pt_left.b = 0;
    up_Points_left->points.push_back(pt_left);

    // 右上特征点（红色）绿色
    pcl::PointXYZRGB pt_right;
    pt_right.x = right_max_z_point_original.x;
    pt_right.y = right_max_z_point_original.y;
    pt_right.z = right_max_z_point_original.z;
    pt_right.r = 0; pt_right.g = 255; pt_right.b = 0;
    up_Points_right->points.push_back(pt_right);
    // 底部特征点（蓝色，即min_z_point）
    pcl::PointXYZRGB pt_down;
    pt_down.x = min_z_point.x;
    pt_down.y = min_z_point.y;
    pt_down.z = min_z_point.z;
    pt_down.r = 0; pt_down.g = 0; pt_down.b = 255;
    down_Points->points.push_back(pt_down);
    // 计算点到直线距离的lambda
    //这段代码定义了一个名为 pt_line 的 lambda 表达式，用于计算3D 空间中一个点到一条直线的距离。
    // 通过直线的参数方程和向量叉乘，计算点到直线的垂直距离（最短距离）。
    //const pcl::PointXYZRGB& pt：输入的 3D 点（包含 x,y,z 坐标）；
    //const pcl::ModelCoefficients::Ptr& coeffs：直线的系数指针（PCL 库中常用的直线参数存储结构）；；
    auto pt_line = [](const pcl::PointXYZRGB& pt, const pcl::ModelCoefficients::Ptr& coeffs) -> double {
        //提取直线参数
        float x0 = coeffs->values[0], y0 = coeffs->values[1], z0 = coeffs->values[2]; // 直线上一点 P₀ 的坐标
        float dx = coeffs->values[3], dy = coeffs->values[4], dz = coeffs->values[5];// 直线的方向向量 v
        //计算向量 P₀P
        float vx = pt.x - x0;// P.x - P₀.x
        float vy = pt.y - y0;// P.y - P₀.y
        float vz = pt.z - z0;// P.z - P₀.z

        // 叉乘计算距离分子部分；计算叉乘 P₀P × v；叉乘结果是一个向量，其模长与两点之间的平行四边形面积相等，
        // 也等于点到直线的距离与方向向量模长的乘积：
        float cx = dy * vz - dz * vy;
        float cy = dz * vx - dx * vz;
        float cz = dx * vy - dy * vx;
        //计算距离--点到直线的距离 = |P₀P × v| / |v|（叉乘的模长除以方向向量的模长）
        double numerator = std::sqrt(cx * cx + cy * cy + cz * cz); // 叉乘向量的模长 |P₀P × v|
        double denominator = std::sqrt(dx * dx + dy * dy + dz * dz); // 方向向量的模长 |v|
        return numerator / denominator;
        };
    //计算左右两特征点的距离
    double distWidth = calculateDistance(pt_left, pt_right);
    double distDepth = pt_line(pt_down, coefficients);

     // 判断距离是否超过直线拟合的距离阈值（判断挂渣）
    bool left = pt_line(pt_left, coefficients) > dist;
    bool right = pt_line(pt_right, coefficients) > dist;
    if (left || right)
    {
        // 创建左右子点云指针
        //pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr left_cloud_2(new pcl::PointCloud<pcl::PointXYZ>());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr right_cloud_2(new pcl::PointCloud<pcl::PointXYZ>());
        //// 遍历 cloud_filter计算 x 的最大最小值
        //float cloud_min_x = std::numeric_limits<float>::max();
        //float cloud_max_x = std::numeric_limits<float>::lowest();
        //for (const auto& pt : cloud_filter->points)
        //{
        //    if (pt.x < cloud_min_x) cloud_min_x = pt.x;
        //    if (pt.x > cloud_max_x) cloud_max_x = pt.x;
        //}  

        //// 计算修正后的边界值，考虑越界情况
        //double fix_x = 0.5;
        //double left_bound = pt_left.x - fix_x;
        //if (left_bound < cloud_min_x) left_bound = pt_left.x;
        //double right_bound = pt_right.x + fix_x;
        //if (right_bound > cloud_max_x) right_bound = pt_right.x; 
        //// 创建左右直线拟合点云（RANSAC直线拟合逼近圆弧用）
        //for (const auto& pt : cloud_filter->points)
        //{
        //    if (pt.x < left_bound) left_cloud->points.push_back(pt);
        //    if (pt.x > right_bound) right_cloud->points.push_back(pt);
        //} 
        //// 创建邻域递归点云（局部收敛用）
        //for (const auto& pt : cloud->points)
        //{
        //    if (pt.x > pt_left.x && pt.x < pt_down.x) left_cloud_2->points.push_back(pt);
        //    if (pt.x < pt_right.x && pt.x > pt_down.x) right_cloud_2->points.push_back(pt);
        //}

        //// 左子点云拟合直线
        //pcl::ModelCoefficients::Ptr left_coefficients(new pcl::ModelCoefficients);
        //pcl::PointIndices::Ptr left_inliers(new pcl::PointIndices);
        //seg.setDistanceThreshold(0.1);

        //if (left_cloud->points.size() >= 2)
        //{
        //    seg.setInputCloud(left_cloud);
        //    seg.segment(*left_inliers, *left_coefficients);

        //    if (left_inliers->indices.size() < 200)
        //        left_coefficients = coefficients; // 内点数过少则用整体直线参数
        //}
        //else
        //{
        //    left_coefficients = coefficients; // 子点云点数不足则用整体直线参数
        //}

        //// 右子点云拟合直线
        //pcl::ModelCoefficients::Ptr right_coefficients(new pcl::ModelCoefficients);
        //pcl::PointIndices::Ptr right_inliers(new pcl::PointIndices);
        //if (right_cloud->points.size() >= 2)
        //{
        //    seg.setInputCloud(right_cloud);
        //    seg.segment(*right_inliers, *right_coefficients);

        //    if (right_inliers->indices.size() < 200)
        //        right_coefficients = coefficients; // 内点数过少则用整体直线参数
        //}
        //else
        //{
        //    right_coefficients = coefficients; // 子点云点数不足则用整体直线参数
        //}
        //// 计算两个直线最近点的lambda
        //auto line_line = [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2,
        //    const pcl::ModelCoefficients::Ptr& coeffs, pcl::PointXYZRGB& intersect_pt) -> bool {
        //        float x0 = coeffs->values[0], y0 = coeffs->values[1], z0 = coeffs->values[2];
        //        float dx = coeffs->values[3], dy = coeffs->values[4], dz = coeffs->values[5];

        //        float ex = p2.x - p1.x;
        //        float ey = p2.y - p1.y;
        //        float ez = p2.z - p1.z;

        //        float sx = p1.x - x0;
        //        float sy = p1.y - y0;
        //        float sz = p1.z - z0;

        //        float a = ex * ex + ey * ey + ez * ez;
        //        float b = ex * dx + ey * dy + ez * dz;
        //        float c = dx * dx + dy * dy + dz * dz;
        //        float d = ex * sx + ey * sy + ez * sz;
        //        float e = dx * sx + dy * sy + dz * sz;

        //        float denom = a * c - b * b;
        //        if (std::abs(denom) < 1e-8) {
        //            // 平行或无解，返回false
        //            return false;
        //        }
        //        float s = (b * e - c * d) / denom;

        //        // 允许延长线
        //        intersect_pt.x = p1.x + s * ex;
        //        intersect_pt.y = p1.y + s * ey;
        //        intersect_pt.z = p1.z + s * ez;

        //        return true;
        //    };

        //// kd 树寻找最近点
        //auto kd_finder = [](const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        //    const pcl::PointXYZRGB& query_pt) -> pcl::PointXYZRGB
        //    {
        //        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        //        kdtree.setInputCloud(cloud);

        //        pcl::PointXYZ search_point{ query_pt.x, query_pt.y, query_pt.z };
        //        std::vector<int> indices(1);
        //        std::vector<float> dists(1);

        //        if (kdtree.nearestKSearch(search_point, 1, indices, dists) > 0) {
        //            const auto& pt = cloud->points[indices[0]];
        //            return pcl::PointXYZRGB{ pt.x, pt.y, pt.z, query_pt.r, query_pt.g, query_pt.b };
        //        }
        //        return query_pt;
        //    };

        //// 计算两点距离
        //auto Distance = [](const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) -> double {
        //    double dx = p1.x - p2.x;
        //    double dy = p1.y - p2.y;
        //    double dz = p1.z - p2.z;
        //    return std::sqrt(dx * dx + dy * dy + dz * dz);
        //    };

        //double dis_nei = 0.2; // 邻域搜索半径

        // 若左点被判定为挂渣，则调整左点位置
        //if (left) {
        //    pcl::PointXYZRGB new_pt_left = pt_left;
        //    // 计算左点-底点直线与左拟合直线的交点：new_pt_left
        //    if (line_line(pt_down, pt_left, left_coefficients, new_pt_left)) {
        //        pcl::PointXYZRGB pt_temp = kd_finder(left_cloud_2, new_pt_left); // kd树找 new_pt_left 的最近点
        //        // 领域点保护：距离小于0.5且 x 坐标不小于 new_pt_left.x 则更新左点
        //        if (Distance(pt_temp, new_pt_left) < 0.5 && pt_temp.x >= new_pt_left.x) {
        //            pt_left = pt_temp;
        //            line_line(pt_down, pt_left, left_coefficients, new_pt_left); // 重新计算左点-底点直线与左拟合直线的交点：new_pt_left
        //        }
        //        pt_left = new_pt_left;

        //        // 邻域搜索（递归到局部最低点）
        //        bool found = false;
        //        float max_x = -std::numeric_limits<float>::max();
        //        pcl::PointXYZ max_x_pt;

        //        for (const auto& pt : left_cloud_2->points)
        //        {
        //            float dx = pt.x - new_pt_left.x;
        //            float dy = pt.y - new_pt_left.y;
        //            float dz = pt.z - new_pt_left.z;
        //            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        //            if (dist < dis_nei)
        //            {
        //                if (!found || pt.x > max_x)
        //                {
        //                    max_x = pt.x;
        //                    max_x_pt = pt;
        //                    found = true;
        //                }
        //            }
        //        }

        //        if (found && max_x > new_pt_left.x)
        //        {
        //            new_pt_left.x = max_x_pt.x;
        //            new_pt_left.y = max_x_pt.y;
        //            new_pt_left.z = max_x_pt.z;
        //            line_line(pt_down, new_pt_left, left_coefficients, pt_left);
        //        }
        //    }
        //}

        //if (right) {
        //    pcl::PointXYZRGB new_pt_right = pt_right;
        //    if (line_line(pt_down, pt_right, right_coefficients, new_pt_right)) {
        //        pcl::PointXYZRGB pt_temp = kd_finder(right_cloud_2, new_pt_right);
        //        if (Distance(pt_temp, new_pt_right) < 0.5 && pt_temp.x <= new_pt_right.x) {
        //            pt_right = pt_temp;
        //            line_line(pt_down, pt_right, right_coefficients, new_pt_right);
        //        }
        //        pt_right = new_pt_right;

        //        bool found = false;
        //        float min_x = std::numeric_limits<float>::max();
        //        pcl::PointXYZ min_x_pt;

        //        for (const auto& pt : right_cloud_2->points)
        //        {
        //            float dx = pt.x - new_pt_right.x;
        //            float dy = pt.y - new_pt_right.y;
        //            float dz = pt.z - new_pt_right.z;
        //            float dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        //            if (dist < dis_nei)
        //            {
        //                if (!found || pt.x < min_x)
        //                {
        //                    min_x = pt.x;
        //                    min_x_pt = pt;
        //                    found = true;
        //                }
        //            }
        //        }

        //        if (found && min_x < new_pt_right.x)
        //        {
        //            new_pt_right.x = min_x_pt.x;
        //            new_pt_right.y = min_x_pt.y;
        //            new_pt_right.z = min_x_pt.z;
        //            line_line(pt_down, new_pt_right, right_coefficients, pt_right);
        //        }
        //    }
        //}



    }

    //if (Reco_Protection(pt_left, pt_right, pt_down)) {
    //    up_Points_left->points.push_back(pt_left);
    //    up_Points_right->points.push_back(pt_right);
    //    down_Points->points.push_back(pt_down);
    //    return 1; // 成功
    //}
   // return -2; // 保护机制触发




    return 0;
 }
 bool PolePieceMeasureWin::Reco_Protection(pcl::PointXYZRGB pt_left, pcl::PointXYZRGB pt_right, pcl::PointXYZRGB pt_down)
 {
     // 0. 上特征点 pt_left 和 pt_right 在X方向上的距离不能过大，超出视场范围
     double length_x = std::fabs(pt_right.x - pt_left.x);
     if (length_x > 16) {
       //  ui->statusLabel->setAnimatedText(QStringLiteral("识别触发保护，上特征点超出视场范围，请减小识别算法旋转角度"));
         return false;
     }

     // 1. pt_left 到 pt_right 的距离 (单帧的宽度)
     double dx = pt_left.x - pt_right.x;
     double dy = pt_left.y - pt_right.y;
     double dz = pt_left.z - pt_right.z;
     double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
     double MinWidth =InfoSuanFa.editMaxWidth.toDouble(); //ui->editMaxWidth->text().toDouble();
     if (MinWidth < 0.04) {
     //if (ui->editMaxWidth->text().isEmpty() || MinWidth < 0.04) {
         MinWidth = 0.04;
     }
     if (distance < MinWidth) {
        // ui->statusLabel->setAnimatedText(QStringLiteral("识别触发保护，请检查最小宽度阈值参数"));
         return false;
     }

     // 2. pt_down.x 是否大致靠近 pt_left.x 和 pt_right.x 中间位置
     double mid_x = (pt_left.x + pt_right.x) / 2.0;
     double Edge = 0.5 - InfoSuanFa.editEdge.toDouble() / 100;//ui->editEdge->text().toDouble() / 100;
     double margin = length_x * Edge;
     if (pt_down.x < mid_x - margin || pt_down.x > mid_x + margin) {
        // ui->statusLabel->setAnimatedText(QStringLiteral("识别触发保护，请检查底点位置阈值参数"));
         return false;
     }

     // 正常情况
     return true;
 }

 void PolePieceMeasureWin::pointsClear() {
     if (up_Points_left != nullptr) up_Points_left->clear();
     if (up_Points_right != nullptr) up_Points_right->clear();
     if (down_Points != nullptr) down_Points->clear();
     InfoResult.width.clear(); InfoResult.depth.clear();
     InfoResult.maxWidth = 0; InfoResult.minWidth = 0;
     InfoResult.maxDepth = 0; InfoResult.minDepth = 0;
     InfoResult.meanWidth = 0; InfoResult.meanDepth = 0;
     ui->labelMeanWidth->clear();
     ui->labelMaxWidth->clear();
     ui->labelMinWidth->clear();
     ui->labelMeanDepth->clear();
     ui->labelMaxDepth->clear();
     ui->labelMinDepth->clear();
     ID.clear();
 }
 int  PolePieceMeasureWin::InfoResult_Calculate()
 {
     InfoResult.width.clear(); InfoResult.depth.clear();

     for (size_t i = 0; i < up_Points_left->size(); ++i)
     {
         const auto& ptLeft = up_Points_left->points[i];
         const auto& ptRight = up_Points_right->points[i];
         double width = calculateDistance(ptLeft, ptRight);
         InfoResult.width.push_back(width);
         const auto& ptDown = down_Points->points[i];
         //求解深度是最下点与左右特征点连线的距离；垂直距离
         Eigen::Vector3d LEFT_POINT(ptLeft.x, ptLeft.y, ptLeft.z);
         Eigen::Vector3d RIGHT_POINT(ptRight.x, ptRight.y, ptRight.z);
         Eigen::Vector3d DOWN_POINT(ptDown.x, ptDown.y, ptDown.z);  
          


         Eigen::Vector3d L2R_LINE = RIGHT_POINT - LEFT_POINT;
         Eigen::Vector3d L2D_LINE = DOWN_POINT - LEFT_POINT;

         double L2R_LINENORMAL = L2R_LINE.norm();//长度
         if (L2R_LINENORMAL<1e-8)
         {
             InfoResult.depth.push_back(0.0);
         }
         else 
         {
             //double distance = AP.cross(AB).norm() / AB.norm();//cross(AB) 得到的是平行四边形面积，除以底边长度 AB.norm() 就是高，即距离。
             double depthDistance = L2D_LINE.cross(L2R_LINE).norm() / L2R_LINE.norm();
             InfoResult.depth.push_back(depthDistance);
         }
     }
     //获取结构体里面宽度容器里面的最大值、最小值以及Mean
     if (!InfoResult.width.empty()) {
         auto minmax_width = std::minmax_element(InfoResult.width.begin(), InfoResult.width.end());
           InfoResult.minWidth = *minmax_width.first;
           InfoResult.maxWidth = *minmax_width.second;
           InfoResult.meanWidth = std::accumulate(InfoResult.width.begin(), InfoResult.width.end(), 0.0) / InfoResult.width.size();
     }
     else {
         // 处理空向量的情况
         return -1;
     }
     //获取结构体里面宽度容器里面的最大值、最小值以及Mean

     if (!InfoResult.depth.empty()) {
         auto minmax_depth = std::minmax_element(InfoResult.depth.begin(), InfoResult.depth.end());
           InfoResult.minDepth = *minmax_depth.first;
           InfoResult.maxDepth = *minmax_depth.second;
           InfoResult.meanDepth = std::accumulate(InfoResult.depth.begin(), InfoResult.depth.end(), 0.0) / InfoResult.depth.size();
     }
     else {
         // 处理空向量的情况
         return -1;
     }
     return 0;




 }

 // 计算两点之间的欧氏距离
 double PolePieceMeasureWin::calculateDistance(const pcl::PointXYZRGB& pt_left, const pcl::PointXYZRGB& pt_right) {
     // 计算坐标差的平方和
     double dx = pt_right.x - pt_left.x;
     double dy = pt_right.y - pt_left.y;
     double dz = pt_right.z - pt_left.z;
     double squared_distance = dx * dx + dy * dy + dz * dz;
     // 开方得到距离
     return std::sqrt(squared_distance);
 }
 // void PolePieceMeasureWin::on_btnRecoCloud_clicked()//识别点云
//{
     //圆筒划线项目的，暂时屏蔽
    //pointsClear();//加载之前清空之前点云
    // if (separate_clouds.empty())
    //{
    //    if (!source_cloud || source_cloud->empty())
    //    {
    //        return ;
    //    }
    //    std::map<float, pcl::PointCloud<pcl::PointXYZ>::Ptr> separateClouds;
    //    
    //    for (const auto&pt:source_cloud->points)
    //    {
    //        if (pt.z < -500)
    //        {
    //            continue;
    //        }
    //        if (separateClouds.find(pt.y) == separateClouds.end())//当前 Y 值第一次出现 → 新建一个空点云。
    //        {
    //            separateClouds[pt.y] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //        }
    //        separateClouds[pt.y]->points.push_back(pt);//把该点放进对应 Y 值的那一层点云。 
    //    }  

    //     //遍历map，设置点云属性，直接把智能指针放入vector
    //    for (   auto& ptr : separateClouds)//按升序遍历 map 里的每一项；kv.first 是 Y 值，kv.second 是点云指针。
    //    {
    //        pcl::PointCloud<pcl::PointXYZ>::Ptr& pr = ptr.second;//起个别名，少打字。
    //        pr->width = static_cast<uint32_t>(pr->points.size());//把宽度设成点的数量。
    //        pr->height = 1;//高度为 1 → 明确这是“无组织”点云（简单列表）。
    //        pr->is_dense = true;
    //        separate_clouds.push_back(pr);
    //    }


    //}
    //Multify_Recognition(separate_clouds);
    //updateViewer();
    //updateTable(); 
 //}
