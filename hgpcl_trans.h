#pragma once
//用来描述旋转的类
#include <vector>
#include <string>
#include <math.h>
#include <time.h>

#include <iostream>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/io/io.h>                      //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                  //PCD文件读取
#include <pcl/io/ply_io.h>					//PLY文件读取
#include <pcl/io/vtk_lib_io.h>				//VTK文件读取			

//滤波类头文件说明
#include <pcl/filters/statistical_outlier_removal.h>//去除离群点
#include <pcl/filters/extract_indices.h>//提取点云索引
#include <pcl/filters/voxel_grid.h>//体素下采样
#include <pcl/filters/passthrough.h>

//一致采样
#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>//一致性采样识别圆柱
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

//可视化
#include <pcl/visualization/cloud_viewer.h> //类cloud_viewer头文件申明

//特征
#include <pcl/features/integral_image_normal.h>  //法线估计类头文件
#include <pcl/features/normal_3d.h>
#include<pcl/features/principal_curvatures.h>//曲率估计类头文件
#include <pcl/features/boundary.h>//边界估计类头文件
#include <pcl/features/fpfh.h>//fpfh特征
#include <pcl/features/vfh.h>//vfh特征
#include <pcl/features/shot.h>//shot特征
#include <pcl/features/shot_omp.h>//shot特征
#include <pcl/features/3dsc.h>//3dsc特征
#include <pcl/features/usc.h>//usc特征
#include <pcl/features/cvfh.h>//cvfh特征
#include <pcl/features/our_cvfh.h>//our_cvfh特征
#include <pcl/features/rsd.h>//rsd特征
#include <pcl/features/rops_estimation.h>//rops特征
#include <pcl/features/shot_lrf.h>//shot_lrf特征
#include <pcl/features/brisk_2d.h>//brisk_2d特征
#include <pcl/features/pfhrgb.h>//pfhrgb特征
#include <pcl/features/gfpfh.h>//gfpfh特征
#include <pcl/features/esf.h>//esf特征
#include <pcl/features/grsd.h>//crh特征

#include <pcl/keypoints/harris_3D.h>//harris特征点估计类头文件声明

//表面
#include <pcl/surface/convex_hull.h>

//聚类
#include <pcl/segmentation/euclidean_cluster_comparator.h>//欧几里得聚类
#include <pcl/segmentation/conditional_euclidean_clustering.h>//条件欧几里得聚类
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>//提取聚类
#include <pcl/segmentation/region_growing.h>

//配准
#include <pcl/registration/icp.h> //ICP配准类相关头文件
#include <pcl/registration/icp_nl.h>//非线性ICP配准类相关头文件
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>  // pcl::transformPointCloud

//opencv 调试用
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace HG
{
	//数组转vector
	template<typename T>
	std::vector<T> array2vector(T* array, int length);
	//数组转matrixXd
	template<typename T>
	Eigen::MatrixXd array2MatrixXd(T* array, int length);

	//根据输入的pcl点云生成齐次向量
	Eigen::Vector4d getHomogeneousVector(const Eigen::Vector3d& point);
	Eigen::Vector4d getHomogeneousVector(const pcl::PointXYZ& point);
	Eigen::Vector3d getVector3d(const pcl::PointXYZ& point);
	Eigen::Vector3d getVector3d(const pcl::Normal& point);
	pcl::PointXYZ getPCLPointXYZ(const Eigen::Vector3d& vector);
	pcl::Normal getPCLNormal(const Eigen::Vector3d& vector);
	std::vector<Eigen::Vector3d> getVector3dSet(const pcl::PointCloud<pcl::PointXYZ> points);
	Eigen::MatrixXd getMatrix(const std::vector<Eigen::Vector3d>& vector);
	Eigen::MatrixXd getMatrix(const pcl::PointCloud<pcl::PointXYZ> points);
	pcl::PointCloud<pcl::PointXYZ> getPCLPointXYZSet(const std::vector<Eigen::Vector3d>& vector);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getPCLPointXYZSet(const std::vector<std::vector<Eigen::Vector3d>>& vector);
	std::vector<Eigen::Vector3d> getVVector3d(Eigen::MatrixXd input);
	std::vector<Eigen::Vector3d> getVVector3d(pcl::PointCloud<pcl::PointXYZ> input);
	//pcl::coefficients 转换为Eigen::Vectorxd
	bool getVectorByCoefficients(pcl::ModelCoefficients::Ptr coefficients, Eigen::VectorXd& output);
	//Eigen::Vectorxd 转换为pcl::coefficients
	bool getCoefficientsByVector(Eigen::VectorXd input, pcl::ModelCoefficients::Ptr& output);

	//cv::Mat 和 Eigen::MatrixXd 相互转化
	cv::Mat getMat(Eigen::MatrixXd& input);
	cv::Mat getMat(Eigen::VectorXd& input);
	std::vector<cv::Mat> getMatFromVector(std::vector<Eigen::MatrixXd>& input);
	std::vector<cv::Mat> getMatFromVector(std::vector<Eigen::VectorXd>& input);

	//矩阵的svd分解
	bool solveSvd(const Eigen::MatrixXd& input, Eigen::MatrixXd& U, Eigen::VectorXd& S, Eigen::MatrixXd& V);

	//随机数相关
	double getRandomNumber(double min, double max);
	//限制范围
	double limitedRange(double input, double rangUp, double rangeDown);
	//
	void limitRange(double& input, double rangUp, double rangeDown);
	//计算中心
	Eigen::Vector3d getVectorCenter(const std::vector<Eigen::Vector3d>& points);

	//排列组合相关
		//求解排列数
	bool getPermutation(int n, int m, std::vector<std::vector<int>>& all);
	//求解组合数
	bool getCombination(int n, int m, std::vector<std::vector<int>>& all);

	//旋转相关的函数
	// 
	// 
	// 
	// 
		//据旋转轴和旋转角度来生成旋转矩阵
	Eigen::Matrix3d getRotationMatrix(double angle, const Eigen::Vector3d& axis);
	Eigen::Matrix4d getRotationMatrix(double angle, const Eigen::Vector3d& axis, Eigen::Vector3d& pole);
	//根据旋转矩阵求解旋转轴和旋转角度
	void getRotationAxisAndAngle(const Eigen::Matrix3d& rotation_matrix, Eigen::Vector3d& axis, double& angle);
	//从一个方向旋转到另一个方向的旋转矩阵
	Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
	//有2个方向旋转到2个方向的旋转矩阵
	Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& from1, const Eigen::Vector3d& from2, const Eigen::Vector3d& to1, const Eigen::Vector3d& to2);


	//根据旋转矩阵求导数；导数矩阵为3*3 为旋转矩阵对欧拉角的导数 表达式 dR(theta)/dtheta
	bool getRotationMatrixDerivative(const Eigen::Matrix3d& rotation_matrix, Eigen::MatrixXd& rotation_matrix_derivative);
	//rotation_matrix *input = output 过程中的导数
	std::vector<Eigen::MatrixXd> getRotationProcessDerivative(const Eigen::Matrix4d& rotation_matrix, const Eigen::MatrixXd& input, const Eigen::MatrixXd& output);


	//据欧拉角来生成旋转矩阵
	Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& euler_angle);
	//据旋转矩阵生成欧拉角
	Eigen::Vector3d getEulerAngle(const Eigen::Matrix3d& rotation_matrix);

	//据旋转矩阵和平移矩阵生成变换矩阵
	Eigen::Matrix4d getTransformMatrix(const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation_matrix);
	//根据变换矩阵求解旋转矩阵和平移矩阵
	void getRotationMatrixAndTranslationMatrix(const Eigen::Matrix4d& transform_matrix, Eigen::Matrix3d& rotation_matrix, Eigen::Vector3d& translation_matrix);
	//使用变换矩阵对点进行变换
	Eigen::Vector3d transformPoint(const Eigen::Matrix4d& transform_matrix, const Eigen::Vector3d& point);
	//使用变换矩阵对矩阵进行变换
	Eigen::MatrixXd transformMatrix(const Eigen::Matrix4d& transform_matrix, const Eigen::MatrixXd& matrix);
	//求解点云集之间的变换矩阵 [R,T]*matrix_base = matrix_regist
	//svd 方法
	Eigen::MatrixXd calTransformMatrix(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist);
	Eigen::MatrixXd calTransformMatrix(const pcl::PointCloud<pcl::PointXYZ>& matrix_base, const pcl::PointCloud<pcl::PointXYZ>& matrix_regist);
	//LM 方法
	Eigen::MatrixXd calTransformMatrixLM(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist);


	bool calTransformMatrix_extendMatrix(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist, Eigen::MatrixXd& matrix_trans, Eigen::MatrixXd& matrix_errror, double sigama_base = (1E1) / 4.0);
	bool calTransformMatrix_extendMatrix(const pcl::PointCloud<pcl::PointXYZ>& matrix_base, const pcl::PointCloud<pcl::PointXYZ>& matrix_regist, Eigen::MatrixXd& matrix_trans, Eigen::MatrixXd& matrix_errror, double sigama_base = (1E1) / 4.0);

	//计算点云与点云之间的转换关系
	//输入：基准点云 pcloud_base，待配准点云 pcloud_regist，变换矩阵 identity； pcloud_regist = identity*pcloud_base
	//算法对点对数量没有要求;对对应点没有要求，也就是说 pcloud_regist 可以是乱序的。
	//构建 待配准的点云的三维高斯模型，需要参数（sigama），物理意义为每个点的三维高斯函数的标准差。代表空间点周围空间圆模型的大小
	//整个算法的物理意义为求解两个点云在构建的三维高斯模型下的重合面积最大值
	bool calTransformMatrix_extendMatrix(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, Eigen::MatrixXd& identity);
	//计算小数据下乱序点云的转换关系
	bool calTransformMatrix_extendMatrix_disorder(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);
	bool calTransformMatrix_extendMatrix_disorder(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);
	bool calTransformMatrix_extendMatrix_disorder(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);

	//特殊情况下的匹配：只旋转z轴匹配；粗匹配
	bool calTransformMatrix_extendMatrix_onlyZ(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);
	bool calTransformMatrix_extendMatrix_onlyZ(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);
	bool calTransformMatrix_extendMatrix_onlyZ(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);
	//三维通用粗匹配；找初始位姿
	bool calTransformMatrix_extendMatrix_3D(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama);
	bool calTransformMatrix_extendMatrix_3D(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama);
	bool calTransformMatrix_extendMatrix_3D(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama);

	//特殊情况下的匹配：只旋转z轴匹配, 精匹配，给定初始位姿
	bool calTransformMatrix_extendMatrix_initialAngle(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, Eigen::MatrixXd& initial_transfrom, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);
	bool calTransformMatrix_extendMatrix_initialAngle(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, Eigen::MatrixXd& initial_transfrom, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);
	bool calTransformMatrix_extendMatrix_initialAngle(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, Eigen::MatrixXd& initial_transfrom, double& score, Eigen::MatrixXd& identity, double sigama = (1E1) / 4.0);



	//排序
	template<typename T>
	std::vector<T> sortVector(std::vector<T>& vector);
	std::vector<Eigen::Vector3d> sortVector(std::vector<Eigen::Vector3d>& vector, Eigen::Vector3d axis);
	pcl::PointCloud<pcl::PointXYZ> sortVector(pcl::PointCloud<pcl::PointXYZ>& vector, Eigen::Vector3d axis);
	//查找
	int findVector(double input, std::vector<double>& vector);
	int findVector(int input, std::vector<int>& vector);
	int findEigenVector(Eigen::Vector3d input, std::vector<Eigen::Vector3d>& vector);
	//叉乘
	Eigen::Vector3d crossProduct(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2);
	//点乘
	double dotProduct(const Eigen::VectorXd& vector1, const Eigen::VectorXd& vector2);
	//求和
	template<typename T>
	T sumVector(std::vector<T> input);
	//求Mean
	template<typename T>
	T meanVector(std::vector<T> input);
	//求方差
	template<typename T>
	T varianceVector(std::vector<T> input);
	//求标准差
	double standardDeviationVector(std::vector<double> input);

	//求向量的模
	double getVectorNorm(const Eigen::VectorXd& vector);
	double getVectorNorm(const Eigen::Vector3d& vector, pcl::PointXYZ v2);


	//求任意两个向量的夹角
	//需要定义旋转轴方向，从旋转轴正方向看去，逆时针为正，顺时针为负
	//求解的夹角范围为[0,2pi]
	//求解的夹角为旋转轴正方向到vector2的方向的夹角
	double getAngleBetweenVectors_2pi(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2, const Eigen::Vector3d& principlenormal);
	//任意两个向量的夹角
	//求解的夹角范围为[0,pi]
	double getAngleBetweenVectors(const Eigen::VectorXd& vector1, const Eigen::VectorXd& vector2);


	//求向量的单位向量
	Eigen::Vector3d getUnitVector(const Eigen::Vector3d& vector);

	//计算向量的中心
	Eigen::Vector3d getCenterPoint(std::vector<Eigen::Vector3d>& points_cloud);
	Eigen::Vector3d getCenterPoint(pcl::PointCloud<pcl::PointXYZ>& points_cloud_pcl);
	//计算点云的高点
	Eigen::Vector3d getTopPoint(std::vector<Eigen::Vector3d>& points_cloud);
	//Eigen::Vector3d getZPoint(pcl::PointCloud<pcl::PointXYZ>& points_cloud_pcl);


	//拼接两个向量
	Eigen::VectorXd concatenateVectors(const Eigen::VectorXd& vector1, const Eigen::VectorXd& vector2);
	//拼接两个矩阵
	//横向拼接
	Eigen::MatrixXd concatenateMatrices_horizon(const Eigen::MatrixXd& matrix1, const Eigen::MatrixXd& matrix2);
	//纵向拼接
	Eigen::MatrixXd concatenateMatrices_vertical(const Eigen::MatrixXd& matrix1, const Eigen::MatrixXd& matrix2);


	//据旋转轴和旋转角度来生成旋转四元数
	Eigen::Quaterniond getRotationQuaternion(const Eigen::Vector3d& axis, double angle);

	//点、线和平面相关的函数
	//
		//平面相关的函数
		// 
		//根据平面上的三个点求解平面的法向量
	Eigen::Vector3d getPlaneNormalVector(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& point3);
	//根据平面上的一个点和平面的法向量求解平面的方程；平面方程为ax+by+cz+d=0 或者 hom(normal_vector)*plane_para=0
	Eigen::Vector4d  getPlaneEquation(const Eigen::Vector3d& point, const Eigen::Vector3d& normal_vector);
	void getPlaneEquation(const Eigen::Vector3d& point, const Eigen::Vector3d& normal_vector, Eigen::Vector4d& plane_para);
	//根据n个平面上的点拟合平面的方程
	Eigen::Vector4d getPlaneEquation(const std::vector<Eigen::Vector3d>& points);
	Eigen::Vector4d getPlaneEquation(const pcl::PointCloud<pcl::PointXYZ>& points);
	//根据n个平面上的点拟合平面的坐标系
	bool getPlaneEquation(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& center, Eigen::Vector3d& normal);
	bool getPlaneEquation(const pcl::PointCloud<pcl::PointXYZ>& points, Eigen::Vector3d& center, Eigen::Vector3d& normal);


	//与法线垂直的平面和yz，xz，xy平面的交线的单位向量
	Eigen::Vector3d getIntersectionLine_yz(const Eigen::Vector3d& normal_vector);
	Eigen::Vector3d getIntersectionLine_xz(const Eigen::Vector3d& normal_vector);
	Eigen::Vector3d getIntersectionLine_xy(const Eigen::Vector3d& normal_vector);

	//点线相关的函数
	//
	//三维直线的参数方程为：x=x0+at,y=y0+bt,z=z0+ct;point = line_pole + t*line_direction；
	//三维线段的参数方程为：x=x0+at,y=y0+bt,z=z0+ct 0<=t<=n;point = line_pole + t*line_direction, 0<=t<=n
	// 参数t的物理意义为点到基础点line_pole的距离（由于向量有正负，所以t总是正的，即取值范围为[0,n]）
	// 线段具有方向，point1 和 point2 为线段的两个端点，point1为基础点，point2为终点，point2-point1为线段的方向向量
	//根据直线上的两个点求解直线的参数方程
	void getLineEquation(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction);
	//根据n个点拟合一个线段的参数方程，初始点为线段的一个端点，方向为线段的方向向量
	void getLineEquation(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction);
	void getLineEquation(const Eigen::MatrixXd& points, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction);
	//dp算法拟合曲线
	bool fitLine_DouglasPeucker(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& output, double threshold);

	//根据线段上的两点求解线段的参数方程
	void getSegmentLineEquation(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction, double& segment_length);

	//计算直线上的点的参数t；point = line_pole + t*line_direction；可以是直线外的点，计算的是投影点的参数t
	double getPointParameter(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction);
	//计算点到直线的距离
	double getDistance_pointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction);
	//计算点到平面的距离
	double getDistance_pointToPlane(const Eigen::Vector3d& point, const Eigen::Vector4d& plane_para);
	//计算点到空间圆的距离
	double getDistance_pointToCircle(const Eigen::Vector3d& point, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& normal, double circle_radius);

	//计算点到直线的投影点
	bool getProjectionPoint_pointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, Eigen::Vector3d& project_point);
	Eigen::Vector3d getProjectionPoint_pointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction);
	//计算点到平面的投影点
	bool getProjectionPoint_pointToPlane(const Eigen::Vector3d& point, const Eigen::Vector4d& plane_para, Eigen::Vector3d& project_point);

	//计算两条直线的交点
	bool getIntersectionPoint(Eigen::Vector3d line1_pole, Eigen::Vector3d line1_direction, Eigen::Vector3d line2_pole, Eigen::Vector3d line2_direction, Eigen::Vector3d& intersect_point);
	Eigen::Vector3d getIntersectionPoint(const Eigen::Vector3d& line1_pole, const Eigen::Vector3d& line1_direction, const Eigen::Vector3d& line2_pole, const Eigen::Vector3d& line2_direction);
	//计算直线与线段的交点
	bool getIntersectionPoint_lineAndSegmentLine(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, Eigen::Vector3d& intersection_point);
	//计算直线与直线段的交点,返回线段上距离直线最近的点
	bool getIntersectionPoint_lineAndSegmentLine_nearest(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, Eigen::Vector3d& intersection_point);
	//计算直线与折线段的交点
	bool getIntersectionPoint_lineAndPolyline(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const std::vector<Eigen::Vector3d>& polyline_points, std::vector<Eigen::Vector3d>& intersection_point);
	//计算直线与折线段的交点,返回线段上距禀直线最近的点
	bool getIntersectionPoint_lineAndPolyline_nearest(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const std::vector<Eigen::Vector3d>& polyline_points, std::vector<Eigen::Vector3d>& intersection_points);

	//计算两条线段的交点
	bool getIntersectionPoint_segmentLineAndSegmentLine(const Eigen::Vector3d& segment1_pole, const Eigen::Vector3d& segment1_direction, double segment1_length, const Eigen::Vector3d& segment2_pole, const Eigen::Vector3d& segment2_direction, double segment2_length, Eigen::Vector3d& intersection_point);
	//计算直线与空间球的交点
	bool getIntersectionPoint_lineAndSphere(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector3d& circle_center, double circle_radius, Eigen::Vector3d& intersection_point1, Eigen::Vector3d& intersection_point2);
	//计算线段与空间球的交点
	int getIntersectionPoint_segmentLineAndSphere(const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, const Eigen::Vector3d& circle_center, double circle_radius, std::vector<Eigen::Vector3d>& intersection_points);
	//计算折线与空间球的交点
	std::vector<Eigen::Vector3d> getIntersectionPoint_polylineAndSphere(const std::vector<Eigen::Vector3d>& polyline_points, const Eigen::Vector3d& circle_center, double circle_radius);

	//计算直线与平面的交点
	bool getIntersectionPoint_lineAndPlane(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector4d& plane_para, Eigen::Vector3d& intersection_point);
	//计算线段与平面的交点
	bool getIntersectionPoint_segmentLineAndPlane(const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, const Eigen::Vector4d& plane_para, Eigen::Vector3d& intersection_point);
	//计算直线与平面的夹角
	double getAngle_lineAndPlane(const Eigen::Vector3d& line_direction, const Eigen::Vector4d& plane_para);
	//计算平面与折线段的交点
	bool getIntersectionPoint_planeAndPolyline(const Eigen::Vector4d& plane_para, const std::vector<Eigen::Vector3d>& polyline_points, std::vector<Eigen::Vector3d>& intersection_point);

	//计算两个平面的交点;与xy平面
	bool getIntersectionPoint_planeAndPlane_xy(const Eigen::Vector4d& plane1_para, const Eigen::Vector4d& plane2_para, Eigen::Vector3d& intersection_point);
	//计算两个平面的交线
	bool getIntersectionLine_planeAndPlane(const Eigen::Vector4d& plane1_para, const Eigen::Vector4d& plane2_para, Eigen::Vector3d& intersection_line_pole, Eigen::Vector3d& intersection_line_direction);
	//计算两个平面的夹角
	double getAngle_planes(const Eigen::Vector4d& plane1_para, const Eigen::Vector4d& plane2_para);

	//空间圆相关函数
	//n个点拟合空间圆的参数方程
	//参数方程为：x=x0+rcos(t),y=y0+rsin(t),z=z0;t为参数，x0,y0,z0为圆心，r为半径
	//参数t的物理意义为点到圆心的距离（由于向量有正负，所以t总是正的，即取值范围为[0,2pi]）
	//根据圆上的三个点求解圆的参数方程
	bool getCircleEquation(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& circle_center, Eigen::Vector3d& circle_normal, double& circle_radius);
	bool getCircleEquation(const pcl::PointCloud<pcl::PointXYZ>& points, Eigen::Vector3d& circle_center, Eigen::Vector3d& circle_normal, double& circle_radius);
	bool getCircleEquation(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& point3, Eigen::Vector3d& circle_center, Eigen::Vector3d& circle_normal, double& circle_radius);

	//计算点在圆上的旋转角
	double getAngle_pointOnCircle(const Eigen::Vector3d& point, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& circle_princicalnormal, const Eigen::Vector3d& circle_binormal, double circle_radius);

	//计算平面与空间圆的交点
	bool getIntersectionPoint_planeAndCircle(const Eigen::Vector4d& plane_para, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& principlenormal, double circle_radius, Eigen::Vector3d& intersection_point1, Eigen::Vector3d& intersection_point2);
	std::vector<Eigen::Vector3d> getIntersectionPoint_planeAndCircle(const Eigen::Vector4d& plane_para, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& principlenormal, double circle_radius);
	//计算平面与空间圆弧的交点
	//假设圆弧的起始角和终止角指的是和x轴副法向量的夹角，取值范围为[0,2pi]
	std::vector<Eigen::Vector3d> getIntersectionPoint_planeAndArc(const Eigen::Vector4d& plane_para, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& principlenormal, const Eigen::Vector3d& binormal, double circle_radius, double arc_start_angle, double arc_end_angle, bool isWiseClock);

	//拟合
		//拟合空间圆
	bool fitCircleRansac(Eigen::MatrixXd& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate = 1000, double threshold = 0.01, int numberOfSelectPoints = 3);
	bool fitCircleRansac(std::vector<Eigen::Vector3d>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate = 1000, double threshold = 0.01, int numberOfSelectPoints = 3);
	bool fitCircleRansac(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate = 1000, double threshold = 0.01, int numberOfSelectPoints = 3);

	bool fitCircleRansacFunctional(Eigen::MatrixXd& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate = 1000, double threshold = 0.01, int numberOfSelectPoints = 3);
	bool fitCircleRansacFunctional(std::vector<Eigen::Vector3d>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate = 1000, double threshold = 0.01, int numberOfSelectPoints = 3);
	bool fitCircleRansacFunctional(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate = 1000, double threshold = 0.01, int numberOfSelectPoints = 3);

	//拟合圆柱
	bool fitCylinder(Eigen::MatrixXd& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius);
	bool fitCylinder(Eigen::MatrixXd& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius, Eigen::Vector3d centerMask, Eigen::Vector3d axisMask, int radiusMask);
	bool fitCylinder(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius);
	bool fitCylinder(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius, Eigen::Vector3d centerMask, Eigen::Vector3d axisMask, int radiusMask);


	//空间立方体 box 相关的函数
		//由八个点确定一个立方体；八个点分别为xyz轴上的最大值和最小值


	//点云处理方法 
		//1搜索
		//计算临近点
	pcl::PointCloud<pcl::PointXYZ> getNearPoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointXYZ point, int ksearch);
	pcl::PointCloud<pcl::PointXYZ> getNearPoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& point, double radius);
	bool getNearPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>, const pcl::PointXYZ point, int ksearch, pcl::PointCloud<pcl::PointXYZ>& output, pcl::PointCloud<pcl::Normal>&);

	//2特征
	// 点特征
	//计算fpfh特征
	pcl::PointCloud<pcl::FPFHSignature33> getFPFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//计算SHOT特征
	pcl::PointCloud<pcl::SHOT352> getSHOT(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//计算RSD特征
	pcl::PointCloud<pcl::PrincipalRadiiRSD> getRSD(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);

	//计算法向量
	//计算单个点的法向量
	pcl::Normal getNormal(const pcl::PointCloud<pcl::PointXYZ>& input);
	pcl::Normal getNormal(const pcl::PointCloud<pcl::PointXYZ>& input, const std::vector<int> pointIdxNKNSearch);
	pcl::PointCloud<pcl::Normal> getNormal(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	pcl::PointCloud<pcl::Normal> getNormal_my(const pcl::PointCloud<pcl::PointXYZ>& input, int ksearch = 19);

	//计算曲率
	pcl::PointCloud<pcl::PrincipalCurvatures> getPrincipalCurvatures(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double radius = 0.2);
	//bool getModelCurvatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int k, std::vector<PCURVATURE>& tempCV);

	//计算中心
	pcl::PointXYZ getCenter(const pcl::PointCloud<pcl::PointXYZ>& input);
	//中心化
	pcl::PointCloud<pcl::PointXYZ> Centralized(const pcl::PointCloud<pcl::PointXYZ>& input);
	pcl::PointCloud<pcl::PointXYZ> Centralized(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointXYZ& center);
	//去中心化
	pcl::PointCloud<pcl::PointXYZ> DeCentralized(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointXYZ& center);

	//局部特征
	//计算3Dshape上下文特征
	pcl::PointCloud<pcl::ShapeContext1980> getShapeContext(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//全局特征
	//计算视点特征直方图
	pcl::PointCloud<pcl::VFHSignature308> getVFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//计算CVFH特征
	pcl::PointCloud<pcl::VFHSignature308> getCVFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//计算OURCVFH特征
	pcl::PointCloud<pcl::VFHSignature308> getOURCVFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//计算gfpfh特征
	pcl::PointCloud<pcl::GFPFHSignature16> getGFPFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//计算ESF特征
	pcl::PointCloud<pcl::ESFSignature640> getESF(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);
	//计算GRSD特征
	pcl::PointCloud<pcl::GRSDSignature21> getGRSD(const pcl::PointCloud<pcl::PointXYZ>& input, double radius = 0.2);

	//计算harris特征点
	pcl::PointCloud<pcl::PointXYZ> getHarrisPoints(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, double r_normal = 0.01, double r_keypoint = 0.01);
	//3滤波
	//去除离群点
	pcl::PointCloud<pcl::PointXYZ> removeOutlierPoints(const pcl::PointCloud<pcl::PointXYZ>& input, int meanK = 50, double stdDevMulThresh = 1.0);
	pcl::PointCloud<pcl::PointXYZ> removeOutlierPoints_density(const pcl::PointCloud<pcl::PointXYZ>& input, int meanK = 50, double stdDevMulThresh = 1.0);
	//采样
	pcl::PointCloud<pcl::PointXYZ> getSamplePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold);
	//passThrough 滤波
	pcl::PointCloud<pcl::PointXYZ> passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>& input, double min, double max, std::string axis);

	//根据某一个方向的法向量滤波
	bool removePoints_oneDirection(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double threshold, Eigen::Vector3d direction, pcl::PointCloud<pcl::PointXYZ>& inCloud, pcl::PointCloud<pcl::PointXYZ>& nonCloud);
	bool removePoints_oneDirection(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double minThreshold, double maxThreshold, Eigen::Vector3d direction, pcl::PointCloud<pcl::PointXYZ>& inCloud, pcl::PointCloud<pcl::PointXYZ>& nonCloud);

	//4聚类
	//点云根据距离聚类
	//输入：点云，聚类的距离阈值
	//输出：聚类后的点云集合
	//std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_EC(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize = 100, double maxSize = 2500);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize = 100, double maxSize = 2500);
	//自定义聚类
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_my(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize = 100, double maxSize = 2500);

	//根据fpfh特征聚类；未完成
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_plane(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> fpfhFeature, double threshold, double minSize = 100, double maxSize = 1000000);
	//基于法向量的聚类
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_normal(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, double threshold, double minSize = 100, double maxSize = 1000000);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_normal(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, std::vector<Eigen::MatrixXd>& Feature, double threshold, double minSize = 100, double maxSize = 1000000);

	//根据曲率聚类
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_curvature(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PrincipalCurvatures> curvatureFeature, double threshold, double minSize = 100, double maxSize = 1000000);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_curvature(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, pcl::PointCloud<pcl::PrincipalCurvatures> curvatureFeature, std::vector<Eigen::MatrixXd>& Feature, double threshold, double minSize = 100, double maxSize = 1000000);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_curvature(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, pcl::PointCloud<pcl::PrincipalCurvatures> curvatureFeature, std::vector<Eigen::MatrixXd>& Feature,
		double threshold, double ruled_theshold, double plane_theshold, double rate, double minSize = 100, double maxSize = 1000000);

	//根据距离聚类
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_distance(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize = 100, double maxSize = 1000000);

	//区域生长聚类
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_RG(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normals, double smoothThreshold, double curevatureThreshold, double minSize = 100, double maxSize = 1000000);

	//5配准
	//icp点云配准
	bool getRegistration_ICP(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& input_target, double threshold, double maxIteration, double& score, Eigen::Matrix4d& transfromMatrix);

	//6提取特征
	//提取平面中的圆
	bool getCirclePoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::PointCloud<pcl::PointXYZ>& circle_points, pcl::PointCloud<pcl::PointXYZ>& noncircle_points);
	bool getCirclePoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>& circle_points, pcl::PointCloud<pcl::PointXYZ>& noncircle_points);

	//提取点云中的平面
	pcl::PointCloud<pcl::PointXYZ> getPlanePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold);
	bool getPlanePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points);
	bool getPlanePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points, pcl::ModelCoefficients&);
	//提取点云中的圆柱
	bool getCylinderPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points);
	bool getCylinderPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points, double minRadius = 0, double maxRadius = 10000);
	//提取点云的凸包
	pcl::PointCloud<pcl::PointXYZ> getConvexHullPoints(const pcl::PointCloud<pcl::PointXYZ>& input);

	//点云的方形box特征；需要计算方形的
	//计算点的坐标
	Eigen::Vector3d getPointCoordination(const Eigen::Vector3d& point, const Eigen::Vector3d& x, const Eigen::Vector3d& y, const Eigen::Vector3d& z);

	// 判断是否在三角形内
	bool isInTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& x, const Eigen::Vector3d& y, const Eigen::Vector3d& z);
	//判断是否在四边形内
	bool isInBox(const Eigen::Vector3d& point, const Eigen::Vector3d& x, const Eigen::Vector3d& y, const Eigen::Vector3d& z);
	bool isInBox(pcl::PointXYZ& inputPoint, Eigen::MatrixXd& box);
	//判断是否在圆柱内
	bool isInCylinder(const Eigen::Vector3d& point, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, double radius, double height);

	//提取点云的box
	Eigen::MatrixXd getBoxPoints(const pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector3d normal, Eigen::Vector3d binormal);
	Eigen::MatrixXd getBoxPoints(const pcl::PointCloud<pcl::PointXYZ>& input);
	bool getBoxPoints(const pcl::PointCloud<pcl::PointXYZ>& input, Eigen::MatrixXd& output, double& score);

	//计算点云到平面的最大值和最小值
	bool getTangencyPoint(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector4d planePara, std::vector<Eigen::Vector3d>& tangencyPoints, double& minDistance, double& maxDistance);

	//提取点云的外圈点
	bool getOuterPoint(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, double interval = 3);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getRoundness(std::vector<pcl::PointCloud<pcl::PointXYZ>>& input, Eigen::Vector3d axis);

	//提取点云内圈点
	bool getInnerPoint(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, Eigen::Vector3d Center, double interval = 3);
	bool getInnerPoint(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, double interval = 3);

	//提取点云的边界点
	bool getBoundaryPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& boundPoints_is, pcl::PointCloud<pcl::PointXYZ>& boundPoints_non, double interval);
	//提取点云中距离某个平面一定距离的点
	bool getPointsByPlaneDistance(const pcl::PointCloud<pcl::PointXYZ>& input, const Eigen::Vector4d& plane_para, double distance, pcl::PointCloud<pcl::PointXYZ>& points);
	//提取点云中距离某条线一定距离的点
	bool getPointsByLineDistance(const pcl::PointCloud<pcl::PointXYZ>& input, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, double distance, pcl::PointCloud<pcl::PointXYZ>& points);

	//根据已知采样点云位置提取对应位置的目标原始点云
	bool getPointCloudByPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& sample_points, double threshold, pcl::PointCloud<pcl::PointXYZ>& output);

	//处理轮廓线函数，分割为直线点集和曲线点集
	//找点云中的直线点集
	bool getLinePoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& line_points, pcl::PointCloud<pcl::PointXYZ>& nonline_points, double interval);
	//找到点云中的圆形点集
	bool getCirclePoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& Circle_points, pcl::PointCloud<pcl::PointXYZ>& nonCircle_points, double interval, double radius, Eigen::Vector3d center);

	//void getLineAndCurvePoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& line_points, pcl::PointCloud<pcl::PointXYZ>& curve_points);

	//判断点云形状
	int getPointCloudShape(const pcl::PointCloud<pcl::PointXYZ>& input);

	//腐蚀点云
	//腐蚀点云，返回腐蚀后的点云和非腐蚀的点云
	bool getLinePoints_erode(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& line_points, pcl::PointCloud<pcl::PointXYZ>& nonline_points, double interval);


	//7点云可视化
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, std::string name);
	void showPointCloud(std::vector<Eigen::Vector3d>& input, std::string name);
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input1, const pcl::PointCloud<pcl::PointXYZ>& input2, std::string name);
	void showPointCloud(std::vector<Eigen::Vector3d>& input1, std::vector<Eigen::Vector3d>& input2, std::string name);
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input1, const pcl::PointCloud<pcl::PointXYZ>& input2, const pcl::PointCloud<pcl::PointXYZ>& input3, std::string name);
	void showPointCloud(std::vector<Eigen::Vector3d>& input1, std::vector<Eigen::Vector3d>& input2, std::vector<Eigen::Vector3d>& input3, std::string name);

	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointXYZ, std::string name);
	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointXYZ, pcl::PointXYZ, std::string name);
	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointXYZ, pcl::PointXYZ, pcl::PointXYZ, std::string name);
	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& input1, std::vector<Eigen::Vector3d> points, std::string name);
	void showPointCloudAndPoints(std::vector<Eigen::Vector3d>& input1, std::vector<Eigen::Vector3d> points, std::string name);

	//显示点云和法向量
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::Normal>& normal, std::string name);

	//根据RSD特征值赋予点云不同的颜色可视化点云
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PrincipalRadiiRSD>, std::string name);
	//根据曲率赋予点云不同的颜色可视化点云
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PrincipalCurvatures>, std::string name);
	//显示vector<pc>中的点云
	void showPointCloud(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input, std::string name);
	//在已有的viewer上画线
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& lin1, pcl::PointCloud<pcl::PointXYZ>& lin2, std::string name);
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input, std::vector<Eigen::Vector3d>& lin1, std::vector<Eigen::Vector3d>& lin2, std::string name);
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::MatrixXd box, std::string name);
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointCloud<pcl::PointXYZ>& input2, pcl::PointCloud<pcl::PointXYZ>& lin1, pcl::PointCloud<pcl::PointXYZ>& lin2, std::string name);
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointCloud<pcl::PointXYZ>& input2, Eigen::MatrixXd box, std::string name);
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input1, Eigen::Vector4d planePara, std::string name);
	//在已有的点云上画点
	void showPoint(pcl::PointCloud<pcl::PointXYZ>& input, std::vector<Eigen::Vector3d>& points, std::string name);

	//生成点云
	pcl::PointCloud<pcl::PointXYZ> genCylinderPointCloud(Eigen::VectorXd coefficients, double length);
	pcl::PointCloud<pcl::PointXYZ> genCylinderPointCloud(pcl::ModelCoefficients::Ptr coefficients, double length);
	pcl::PointCloud<pcl::PointXYZ> genCirclePointCloud(pcl::ModelCoefficients::Ptr coefficients);
	pcl::PointCloud<pcl::PointXYZ> genCirclePointCloud(Eigen::Vector3d para, double radius);
	std::vector<pcl::PointCloud<pcl::PointXYZ>> genCirclePointCloud(std::vector<Eigen::Vector3d>& para, double radius);
	pcl::PointCloud<pcl::PointXYZ> genCirclePointCloud(Eigen::Vector3d para, Eigen::Vector3d normal, double radius, double intervalAngle = 5);

	//B样条曲线
	//二维B样条曲线
	//回转体拟合
	bool BSplineCurve2DgreeBase(double input, double& output, double& derivativeYtoX, double tStart, double tmiddle, double tend, double tend2);
	bool BSplineCureve2D(double input, double& output, double& derivativeYtoX, std::vector<double>& derivativeYtoP, double tStart, double tend, std::vector<double> Para);
	bool BSplineCureve2DTransfer(double input, double& output, double& derivativeYtoX, std::vector<double>& derivativeYtoP, double tStart, double tend, std::vector<double> Para);
	bool RevolvedBodyFitFlowLM(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum, Eigen::MatrixXd& YEerror);
	bool RevolvedBodyFitFlowTransferLM(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum, Eigen::MatrixXd& YEerror);
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2D(Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum);
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2DTransfer(Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum);
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2D(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum);
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2DTransfer(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum);
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2D(Eigen::MatrixXd& inputPoint, pcl::PointCloud<pcl::PointXYZ>& outputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum);
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2DTransfer(Eigen::MatrixXd& inputPoint, pcl::PointCloud<pcl::PointXYZ>& outputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum);
	bool RevolvedBodyFitFlowLMRansac(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum, int selectNum, double threshold);

	//opencv相关
	//Eigen::MatrixXd转cv::Mat
	cv::Mat Eigen2Mat(const Eigen::MatrixXd& eigen);
	cv::Mat Eigen2Mat(const std::vector<Eigen::MatrixXd>& eigen);
	//cv::Mat转Eigen::MatrixXd
	Eigen::MatrixXd Mat2Eigen(const cv::Mat& cv);

	//三维图
	class C3Dimage
	{
	public:
		C3Dimage();
		C3Dimage(std::vector<Eigen::Vector3d>);
		~C3Dimage();
		C3Dimage& operator=(const C3Dimage& other) {
			if (this != &other) {
				T_trans = other.T_trans;
				m_Interval = other.m_Interval;
				length_xyz = other.length_xyz;
				image3Data_cv = other.image3Data_cv;
				image3Fill_cv = other.image3Fill_cv;
				image3Connect_cv = other.image3Connect_cv;
				cluster_clouds = other.cluster_clouds;
			}
			return *this;
		}
		//由点云坐标得到三维图坐标
		Eigen::Vector3d get3DimageCoordinate(Eigen::Vector3d);
		//由点云生成三维图
		void initial(std::vector<Eigen::Vector3d>);
		//计算三维图连通域
		void getConnectRegion();

	private:
		//置0
		void setZero();

	public:
		//三维图的体积，描述对角线上两点的量
		Eigen::Vector3d T_trans;//平移变换量
		double m_Interval;
		Eigen::Vector3d length_xyz;//xyz方向上的长度(数据的三维各位，目前是：x、y、维度)
		//数据
		//最里面是x，中间是y，最外面是z；一般来讲z方向是延伸的方向
		//使用OpenCV调试,数据不保存在这里。
		cv::Mat image3Data_cv;
		cv::Mat image3Fill_cv;
		cv::Mat image3Connect_cv;

		//聚类的点云
		std::vector<std::vector<Eigen::Vector3d>> cluster_clouds;
	};



}