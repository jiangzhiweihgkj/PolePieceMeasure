#include"hgpcl_trans.h"

namespace HG
{
	//数组转换为向量
	template<typename T>
	std::vector<T> array2vector(T* array, int length)
	{
		std::vector<T> vector;
		for (int i = 0; i < length; i++)
		{
			vector.push_back(array[i]);
		}
		return vector;
	}
	template<typename T>
	Eigen::MatrixXd array2MatrixXd(T* array, int length)
	{
		Eigen::MatrixXd matrix(length, 1);
		for (int i = 0; i < length; i++)
		{
			matrix(i, 0) = array[i];
		}
		return matrix;
	}
	//据旋转轴和旋转角度来生成旋转矩阵
	Eigen::Matrix3d getRotationMatrix(double angle, const Eigen::Vector3d& axis)
	{
		Eigen::AngleAxisd rotation_vector(angle, axis);
		Eigen::Matrix3d rotation_matrix = rotation_vector.matrix();
		return rotation_matrix;
	}
	Eigen::Matrix4d getRotationMatrix(double angle, const Eigen::Vector3d& axis, Eigen::Vector3d& pole)
	{
		//根据旋转角和空间一旋转轴，求解旋转矩阵
		//初始向量的起点在原点，绕着旋转轴旋转的变换矩阵
		Eigen::Matrix3d rotation_matrix = getRotationMatrix(angle, axis);
		Eigen::Vector3d T = -rotation_matrix * pole + pole;
		Eigen::Matrix4d rotation_matrix4d = Eigen::Matrix4d::Identity();
		rotation_matrix4d.block(0, 0, 3, 3) = rotation_matrix;
		rotation_matrix4d.block(0, 3, 3, 1) = T;
		return rotation_matrix4d;
	}
	void getRotationAxisAndAngle(const Eigen::Matrix3d& rotation_matrix, Eigen::Vector3d& axis, double& angle)
	{
		Eigen::AngleAxisd rotation_vector(rotation_matrix);
		axis = rotation_vector.axis();
		angle = rotation_vector.angle();
	}
	Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
	{
		Eigen::Vector3d axis = from.cross(to);
		axis.normalize();
		double angle = acos(from.dot(to) / (from.norm() * to.norm()));
		Eigen::Matrix3d rotation_matrix = getRotationMatrix(angle, axis);
		return rotation_matrix;
	}
	Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& from1, const Eigen::Vector3d& from2, const Eigen::Vector3d& to1, const Eigen::Vector3d& to2)
	{
		Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
		rotation_matrix1.block(0, 0, 3, 1) = from1.normalized();
		rotation_matrix1.block(0, 1, 3, 1) = from2.normalized();
		Eigen::Vector3d from3 = from1.cross(from2).normalized();
		rotation_matrix1.block(0, 2, 3, 1) = from3;
		rotation_matrix2.block(0, 0, 3, 1) = to1.normalized();
		rotation_matrix2.block(0, 1, 3, 1) = to2.normalized();
		Eigen::Vector3d to3 = to1.cross(to2).normalized();
		rotation_matrix2.block(0, 2, 3, 1) = to3;
		Eigen::Matrix3d rotation_matrix = rotation_matrix2 * rotation_matrix1.transpose();
		return rotation_matrix;
	}

	bool getRotationMatrixDerivative(const Eigen::Matrix3d& rotation_matrix, Eigen::MatrixXd& rotation_matrix_derivative)
	{
		//由旋转矩阵生成欧拉角
		Eigen::Vector3d euler_angle = rotation_matrix.eulerAngles(0, 1, 2);
		//由欧拉角生成旋转矩阵
		Eigen::Matrix3d rotation_matrix_x, rotation_matrix_y, rotation_matrix_z;
		rotation_matrix_x = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX());
		rotation_matrix_y = Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY());
		rotation_matrix_z = Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ());
		Eigen::Matrix3d skew_x, skew_y, skew_z;
		skew_x << 0, 0, 0,
			0, 0, -1,
			0, 1, 0;
		skew_y << 0, 0, 1,
			0, 0, 0,
			-1, 0, 0;
		skew_z << 0, -1, 0,
			1, 0, 0,
			0, 0, 0;
		Eigen::Matrix3d rotation_matrix_derivative_x, rotation_matrix_derivative_y, rotation_matrix_derivative_z;
		rotation_matrix_derivative_x = skew_x * rotation_matrix_x * rotation_matrix_y * rotation_matrix_z;
		rotation_matrix_derivative_y = rotation_matrix_x * skew_y * rotation_matrix_y * rotation_matrix_z;
		rotation_matrix_derivative_z = rotation_matrix_x * rotation_matrix_y * skew_z * rotation_matrix_z;
		Eigen::MatrixXd output(9, 3);
		for (size_t i = 0; i < 9; i++)
		{
			output(i, 0) = rotation_matrix_derivative_x(i / 3, i % 3);
			output(i, 1) = rotation_matrix_derivative_y(i / 3, i % 3);
			output(i, 2) = rotation_matrix_derivative_z(i / 3, i % 3);
		}
		rotation_matrix_derivative = output;
		return 1;
	}
	std::vector<Eigen::MatrixXd> getRotationProcessDerivative(const Eigen::Matrix4d& rotation_matrix, const Eigen::MatrixXd& input, const Eigen::MatrixXd& output)
	{
		//首先计算 output
		//output = (rotation_matrix.block(0,0,3,3)* input)

		//生成导数
		Eigen::MatrixXd JYX, JYThteta, JYT, JYR;
		JYX = Eigen::MatrixXd::Zero(output.rows() * output.cols(), input.rows() * input.cols());//(3*col)*9
		JYR = Eigen::MatrixXd::Zero(output.rows() * output.cols(), 9);//(3*col)*9
		JYT = Eigen::MatrixXd::Zero(output.rows() * output.cols(), 3);//(3*col)*3
		for (size_t i = 0; i < output.cols(); i++)
		{
			JYX.block(3 * i, 3 * i, 3, 3) = rotation_matrix.block(0, 0, 3, 3);
			JYR.block(3 * i, 0, 1, 3) = input.col(i).transpose();
			JYR.block(3 * i + 1, 3, 1, 3) = input.col(i).transpose();
			JYR.block(3 * i + 2, 6, 1, 3) = input.col(i).transpose();
			JYT.block(3 * i, 0, 3, 3) = Eigen::Matrix3d::Identity();
		}
		getRotationMatrixDerivative(rotation_matrix.block(0, 0, 3, 3), JYThteta);
		JYThteta = JYR * JYThteta;
		std::vector<Eigen::MatrixXd> vout;
		vout.push_back(JYX);
		vout.push_back(JYThteta);
		vout.push_back(JYT);
		return vout;
	}
	std::vector<Eigen::MatrixXd> getRotationProcessDerivativePara(const Eigen::Matrix4d& rotation_matrix, const Eigen::MatrixXd& input, const Eigen::MatrixXd& output)
	{
		//首先计算 output
		//output = (rotation_matrix.block(0,0,3,3)* input)

		//生成导数
		Eigen::MatrixXd JYX, JYThteta, JYT, JYR;
		//JYX = Eigen::MatrixXd::Zero(output.rows() * output.cols(), input.rows() * input.cols());//(3*col)*9
		JYR = Eigen::MatrixXd::Zero(output.rows() * output.cols(), 9);//(3*col)*9
		JYT = Eigen::MatrixXd::Zero(output.rows() * output.cols(), 3);//(3*col)*3
		for (size_t i = 0; i < output.cols(); i++)
		{
			//JYX.block(3 * i, 3 * i, 3, 3) = rotation_matrix.block(0, 0, 3, 3);
			JYR.block(3 * i, 0, 1, 3) = input.col(i).transpose();
			JYR.block(3 * i + 1, 3, 1, 3) = input.col(i).transpose();
			JYR.block(3 * i + 2, 6, 1, 3) = input.col(i).transpose();
			JYT.block(3 * i, 0, 3, 3) = Eigen::Matrix3d::Identity();
		}
		getRotationMatrixDerivative(rotation_matrix.block(0, 0, 3, 3), JYThteta);
		JYThteta = JYR * JYThteta;
		std::vector<Eigen::MatrixXd> vout;
		//vout.push_back(JYX);
		vout.push_back(JYThteta);
		vout.push_back(JYT);
		return vout;
	}
	//s(R*input+T) = output
	std::vector<Eigen::MatrixXd> getRotationProcessDerivativeScale(const Eigen::Matrix4d& rotation_matrix, const Eigen::MatrixXd& input, const Eigen::MatrixXd& output)
	{
		//求尺度
		double scale = rotation_matrix(3, 3);
		Eigen::MatrixXd rotateMatrix = rotation_matrix.block(0, 0, 3, 3) / scale;
		Eigen::MatrixXd translateMatrix = rotation_matrix.block(0, 3, 3, 1) / scale;
		//生成导数
		Eigen::MatrixXd JYX, JYThteta, JYT, JYR, JYS;
		JYX = Eigen::MatrixXd::Zero(output.rows() * output.cols(), input.rows() * input.cols());//(3*col)*9
		JYR = Eigen::MatrixXd::Zero(output.rows() * output.cols(), 9);//(3*col)*9
		JYT = Eigen::MatrixXd::Zero(output.rows() * output.cols(), 3);//(3*col)*3
		JYS = Eigen::MatrixXd::Zero(output.rows() * output.cols(), 1);//(3*col)*1
		for (size_t i = 0; i < output.cols(); i++)
		{
			JYX.block(3 * i, 3 * i, 3, 3) = scale * rotation_matrix.block(0, 0, 3, 3);
			JYR.block(3 * i, 0, 1, 3) = scale * input.col(i).transpose();
			JYR.block(3 * i + 1, 3, 1, 3) = scale * input.col(i).transpose();
			JYR.block(3 * i + 2, 6, 1, 3) = scale * input.col(i).transpose();
			JYT.block(3 * i, 0, 3, 3) = scale * Eigen::Matrix3d::Identity();
			JYS.block(3 * i, 0, 3, 1) = rotateMatrix * input.col(i) + translateMatrix;
		}
		getRotationMatrixDerivative(rotateMatrix, JYThteta);
		JYThteta = JYR * JYThteta;
		std::vector<Eigen::MatrixXd> vout;
		vout.push_back(JYX);
		vout.push_back(JYThteta);
		vout.push_back(JYT);
		vout.push_back(JYS);
		return vout;
	}


	Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& euler_angle)
	{
		Eigen::Matrix3d rotation_matrix;
		rotation_matrix = Eigen::AngleAxisd(euler_angle[0], Eigen::Vector3d::UnitX())
			* Eigen::AngleAxisd(euler_angle[1], Eigen::Vector3d::UnitY())
			* Eigen::AngleAxisd(euler_angle[2], Eigen::Vector3d::UnitZ());
		return rotation_matrix;
	}
	Eigen::Vector3d getEulerAngle(const Eigen::Matrix3d& rotation_matrix)
	{
		Eigen::Vector3d euler_angle = rotation_matrix.eulerAngles(0, 1, 2);
		return euler_angle;
	}
	Eigen::Matrix4d getTransformMatrix(const Eigen::Matrix3d& rotation_matrix, const Eigen::Vector3d& translation_matrix)
	{
		Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
		transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
		transform_matrix.block<3, 1>(0, 3) = translation_matrix;
		return transform_matrix;
	}
	void getRotationMatrixAndTranslationMatrix(const Eigen::Matrix4d& transform_matrix, Eigen::Matrix3d& rotation_matrix, Eigen::Vector3d& translation_matrix)
	{
		rotation_matrix = transform_matrix.block<3, 3>(0, 0);
		translation_matrix = transform_matrix.block<3, 1>(0, 3);
	}
	//把一个 3D 坐标点 (x,y,z) 转成 4D 齐次坐标 (x,y,z,1)，以便后续能用 4×4 变换矩阵一次性完成旋转、平移、缩放、投影等操作。
	Eigen::Vector4d getHomogeneousVector(const Eigen::Vector3d& point)
	{
		Eigen::Vector4d homogeneous_vector;
		homogeneous_vector << point[0], point[1], point[2], 1;
		return homogeneous_vector;
	}
	Eigen::Vector4d getHomogeneousVector(const pcl::PointXYZ& point)
	{
		Eigen::Vector4d homogeneous_vector;
		homogeneous_vector << point.x, point.y, point.z, 1;
		return homogeneous_vector;
	}
	Eigen::Vector3d getVector3d(const pcl::PointXYZ& point)
	{
		Eigen::Vector3d vector;
		vector << point.x, point.y, point.z;
		return vector;
	}
	Eigen::Vector3d getVector3d(const pcl::Normal& point)
	{
		Eigen::Vector3d vector;
		vector << point.normal_x, point.normal_y, point.normal_z;
		return vector;
	}
	pcl::PointXYZ getPCLPointXYZ(const Eigen::Vector3d& vector)
	{
		pcl::PointXYZ point;
		point.x = vector[0];
		point.y = vector[1];
		point.z = vector[2];
		return point;
	}
	pcl::Normal getPCLNormal(const Eigen::Vector3d& vector)
	{
		pcl::Normal normal;
		normal.normal_x = vector[0];
		normal.normal_y = vector[1];
		normal.normal_z = vector[2];
		return normal;
	}
	std::vector<Eigen::Vector3d> getVector3dSet(const pcl::PointCloud<pcl::PointXYZ> points)
	{
		std::vector<Eigen::Vector3d> vectors;
		for (int i = 0; i < points.size(); i++)
		{
			Eigen::Vector3d vector;
			vector << points[i].x, points[i].y, points[i].z;
			vectors.push_back(vector);
		}
		return vectors;
	}

	pcl::PointCloud<pcl::PointXYZ> getPCLPointXYZSet(const std::vector<Eigen::Vector3d>& vector)
	{
		pcl::PointCloud<pcl::PointXYZ> points;
		for (int i = 0; i < vector.size(); i++)
		{
			pcl::PointXYZ point;
			point.x = vector[i][0];
			point.y = vector[i][1];
			point.z = vector[i][2];
			points.push_back(point);
		}
		return points;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getPCLPointXYZSet(const std::vector<std::vector<Eigen::Vector3d>>& vector)
	{
		std::vector<pcl::PointCloud<pcl::PointXYZ>> points;
		for (int i = 0; i < vector.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZ> point;
			for (int j = 0; j < vector[i].size(); j++)
			{
				pcl::PointXYZ point_temp;
				point_temp.x = vector[i][j][0];
				point_temp.y = vector[i][j][1];
				point_temp.z = vector[i][j][2];
				point.push_back(point_temp);
			}
			points.push_back(point);
		}
		return points;
	}
	Eigen::Vector3d transformPoint(const Eigen::Matrix4d& transform_matrix, const Eigen::Vector3d& point)
	{
		Eigen::Vector4d homogeneous_point = getHomogeneousVector(point);
		Eigen::Vector4d transformed_homogeneous_point = transform_matrix * homogeneous_point;
		Eigen::Vector3d transformed_point;
		transformed_point << transformed_homogeneous_point[0], transformed_homogeneous_point[1], transformed_homogeneous_point[2];
		return transformed_point;
	}
	Eigen::MatrixXd transformMatrix(const Eigen::Matrix4d& transform_matrix, const Eigen::MatrixXd& matrix)
	{
		Eigen::MatrixXd transformed_matrix(matrix);
		for (size_t i = 0; i < matrix.cols(); i++)
		{
			transformed_matrix.col(i).block(0, 0, 3, 1) = transform_matrix.block(0, 0, 3, 3) * matrix.block(0, i, 3, 1) + transform_matrix.block(0, 3, 3, 1);
		}
		return transformed_matrix;
	}
	Eigen::MatrixXd calTransformMatrix(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist)
	{
		Eigen::MatrixXd matrix_base_centered = matrix_base.colwise() - matrix_base.rowwise().mean();
		Eigen::MatrixXd matrix_regist_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();
		Eigen::MatrixXd covariance_matrix = matrix_base_centered * matrix_regist_centered.transpose();
		std::cout << "协方差矩阵：covariance_matrix " << covariance_matrix << std::endl;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::MatrixXd rotation_matrix = svd.matrixV() * svd.matrixU().transpose();
		Eigen::Vector3d translation_matrix = matrix_regist.rowwise().mean() - rotation_matrix * matrix_base.rowwise().mean();
		Eigen::MatrixXd transform_matrix = getTransformMatrix(rotation_matrix, translation_matrix);
		Eigen::MatrixXd error = matrix_regist - transformMatrix(transform_matrix, matrix_base);
		//std::cout<< "求解变换矩阵的误差："<<error.norm() << std::endl;
		return transform_matrix;
	}
	Eigen::MatrixXd calTransformMatrix(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		Eigen::MatrixXd transform_matrix = calTransformMatrix(matrix_base, matrix_regist);
		return transform_matrix;
	}
	Eigen::MatrixXd calTransformMatrixLM(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist)
	{
		//参数 
		double stride = 1;
		double gama = 1E-10;
		//使用LM方法迭代求解RT旋转矩阵
		Eigen::MatrixXd transform_matrix = Eigen::Matrix4d::Identity();
		Eigen::MatrixXd transform_matrix_error = Eigen::Matrix4d::Identity();


		//计算残差
		Eigen::MatrixXd Y0 = matrix_regist.reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		Eigen::MatrixXd Y1 = transformMatrix(transform_matrix, matrix_base).reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		//计算雅克比矩阵
		std::vector<Eigen::MatrixXd> J_set = getRotationProcessDerivative(transform_matrix, matrix_base, matrix_regist);
		Eigen::MatrixXd J = Eigen::MatrixXd::Zero(Y0.rows(), 6);
		J = concatenateMatrices_horizon(J_set[1], J_set[2]);

		//变换J
		//J.block(0, 0, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将旋转变量置为0
		//J.block(0, 3, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将平移变量置为0
		//std::cout << "J：" << J << std::endl;


		Eigen::MatrixXd detaA = (J.transpose() * J + gama * Eigen::MatrixXd::Identity(J.cols(), J.cols())).inverse() * J.transpose() * (Y0 - Y1);
		detaA = stride * detaA;
		//std::cout << "detaA：" << detaA << std::endl;

		Eigen::MatrixXd detaR = detaA.block(0, 0, 3, 1);
		Eigen::Vector3d detaT(detaA(3), detaA(4), detaA(5));
		Eigen::MatrixXd detaR_matrix = getRotationMatrix(Eigen::Vector3d(detaR(0), detaR(1), detaR(2)));
		Eigen::Vector3d detaT_matrix = detaT;
		transform_matrix = getTransformMatrix(detaR_matrix, detaT_matrix) * transform_matrix;
		Eigen::MatrixXd error = matrix_regist - transformMatrix(transform_matrix, matrix_base);
		double error_norm = error.norm();
		//std::cout<<"LM 方法求解角度的误差："<< error_norm << std::endl;
		return transform_matrix;
	}
	Eigen::MatrixXd calTransformMatrixLMScale(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist)
	{
		//参数 
		double stride = 1;
		double gama = 1E-10;
		//使用LM方法迭代求解RT旋转矩阵
		Eigen::MatrixXd transform_matrix = Eigen::Matrix4d::Identity();
		Eigen::MatrixXd transform_matrix_error = Eigen::Matrix4d::Identity();


		//计算残差
		Eigen::MatrixXd Y0 = matrix_regist.reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		Eigen::MatrixXd Y1 = transformMatrix(transform_matrix, matrix_base).reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		//计算雅克比矩阵
		std::vector<Eigen::MatrixXd> J_set = getRotationProcessDerivativeScale(transform_matrix, matrix_base, matrix_regist);
		Eigen::MatrixXd J, J1;// = Eigen::MatrixXd::Zero(Y0.rows(), 7);
		J1 = concatenateMatrices_horizon(J_set[1], J_set[2]);
		J = concatenateMatrices_horizon(J1, J_set[3]);

		//变换J
		//J.block(0, 0, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将旋转变量置为0
		//J.block(0, 3, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将平移变量置为0
		//std::cout << "J：" << J << std::endl;


		Eigen::MatrixXd detaA = (J.transpose() * J + gama * Eigen::MatrixXd::Identity(J.cols(), J.cols())).inverse() * J.transpose() * (Y0 - Y1);
		detaA = stride * detaA;
		//std::cout << "detaA：" << detaA << std::endl;

		Eigen::MatrixXd detaR = detaA.block(0, 0, 3, 1);
		Eigen::Vector3d detaT(detaA(3), detaA(4), detaA(5));
		Eigen::MatrixXd detaR_matrix = getRotationMatrix(Eigen::Vector3d(detaR(0), detaR(1), detaR(2)));
		Eigen::Vector3d detaT_matrix = detaT;
		transform_matrix = getTransformMatrix((1 + detaA(6)) * detaR_matrix, detaT_matrix) * transform_matrix;
		Eigen::MatrixXd error = matrix_regist - transformMatrix(transform_matrix, matrix_base);
		double error_norm = error.sum();
		//std::cout << "LM 方法求解角度的误差：" << error_norm << std::endl;
		return transform_matrix;
	}
	Eigen::MatrixXd calTransformMatrixLMWeight(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist, const std::vector<double> weight)
	{
		//参数 
		double stride = 1;
		double gama = 1E-10;
		//使用LM方法迭代求解RT旋转矩阵
		Eigen::MatrixXd transform_matrix = Eigen::Matrix4d::Identity();
		Eigen::MatrixXd transform_matrix_error = Eigen::Matrix4d::Identity();


		//计算残差
		Eigen::MatrixXd Y0 = matrix_regist.reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		Eigen::MatrixXd Y1 = transformMatrix(transform_matrix, matrix_base).reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		//计算雅克比矩阵
		std::vector<Eigen::MatrixXd> J_set = getRotationProcessDerivative(transform_matrix, matrix_base, matrix_regist);
		Eigen::MatrixXd J = Eigen::MatrixXd::Zero(Y0.rows(), 6);
		J = concatenateMatrices_horizon(J_set[1], J_set[2]);

		//变换J
		//J.block(0, 0, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将旋转变量置为0
		//J.block(0, 3, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将平移变量置为0
		//std::cout << "J：" << J << std::endl;

		//计算权重矩阵
		Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Zero(Y0.rows(), Y0.rows());
		for (size_t i = 0; i < weight.size(); i++)
		{
			weight_matrix(3 * i, 3 * i) = weight[i];
			weight_matrix(3 * i + 1, 3 * i + 1) = weight[i];
			weight_matrix(3 * i + 2, 3 * i + 2) = weight[i];
		}

		Eigen::MatrixXd detaA = (J.transpose() * weight_matrix * J + gama * Eigen::MatrixXd::Identity(J.cols(), J.cols())).inverse() * J.transpose() * weight_matrix * (Y0 - Y1);
		detaA = stride * detaA;
		//std::cout << "detaA："<< detaA << std::endl;

		Eigen::MatrixXd detaR = detaA.block(0, 0, 3, 1);
		Eigen::Vector3d detaT(detaA(3), detaA(4), detaA(5));
		Eigen::MatrixXd detaR_matrix = getRotationMatrix(Eigen::Vector3d(detaR(0), detaR(1), detaR(2)));
		Eigen::Vector3d detaT_matrix = detaT;
		transform_matrix = getTransformMatrix(detaR_matrix, detaT_matrix) * transform_matrix;
		Eigen::MatrixXd error = matrix_regist - transformMatrix(transform_matrix, matrix_base);
		double error_norm = error.norm();
		std::cout << "LM 方法求解角度的误差：" << error_norm << std::endl;
		return transform_matrix;
	}
	Eigen::MatrixXd calTransformMatrixLMScaleWeight(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist, const std::vector<double> weight)
	{
		//参数 
		double stride = 1;
		double gama = 1E-10;
		//使用LM方法迭代求解RT旋转矩阵
		Eigen::MatrixXd transform_matrix = Eigen::Matrix4d::Identity();
		Eigen::MatrixXd transform_matrix_error = Eigen::Matrix4d::Identity();


		//计算残差
		Eigen::MatrixXd Y0 = matrix_regist.reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		Eigen::MatrixXd Y1 = transformMatrix(transform_matrix, matrix_base).reshaped(matrix_regist.rows() * matrix_regist.cols(), 1);
		//计算雅克比矩阵
		std::vector<Eigen::MatrixXd> J_set = getRotationProcessDerivativeScale(transform_matrix, matrix_base, matrix_regist);
		Eigen::MatrixXd J1, J;// = Eigen::MatrixXd::Zero(Y0.rows(), 6);
		J1 = concatenateMatrices_horizon(J_set[1], J_set[2]);
		J = concatenateMatrices_horizon(J1, J_set[3]);

		//变换J
		//J.block(0, 0, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将旋转变量置为0
		//J.block(0, 3, J.rows(), 3) = Eigen::MatrixXd::Zero(Y0.rows(), 3);//将平移变量置为0
		//std::cout << "J：" << J << std::endl;

		//计算权重矩阵
		Eigen::MatrixXd weight_matrix = Eigen::MatrixXd::Zero(Y0.rows(), Y0.rows());
		for (size_t i = 0; i < weight.size(); i++)
		{
			weight_matrix(3 * i, 3 * i) = weight[i];
			weight_matrix(3 * i + 1, 3 * i + 1) = weight[i];
			weight_matrix(3 * i + 2, 3 * i + 2) = weight[i];
		}

		Eigen::MatrixXd detaA = (J.transpose() * weight_matrix * J + gama * Eigen::MatrixXd::Identity(J.cols(), J.cols())).inverse() * J.transpose() * weight_matrix * (Y0 - Y1);
		detaA = stride * detaA;
		//std::cout << "detaA：" << detaA << std::endl;

		Eigen::MatrixXd detaR = detaA.block(0, 0, 3, 1);
		Eigen::Vector3d detaT(detaA(3), detaA(4), detaA(5));
		Eigen::MatrixXd detaR_matrix = getRotationMatrix(Eigen::Vector3d(detaR(0), detaR(1), detaR(2)));
		Eigen::Vector3d detaT_matrix = detaT;
		transform_matrix = (1 + detaA(6)) * getTransformMatrix(detaR_matrix, detaT_matrix) * transform_matrix;
		Eigen::MatrixXd error = matrix_regist - transformMatrix(transform_matrix, matrix_base);
		double error_norm = error.norm();
		std::cout << "LM 方法求解角度的误差：" << error_norm << std::endl;
		return transform_matrix;
	}

	//基础的基于三维高斯函数的配准方法，单次步骤；作为基础的配准方法
	bool calTransformMatrix_extendMatrix(const Eigen::MatrixXd& matrix_base, const Eigen::MatrixXd& matrix_regist, Eigen::MatrixXd& matrix_trans, Eigen::MatrixXd& matrix_error, double sigama_base)
	{
		//原理是将点云中的每个点模拟成一个三维高斯函数。sigama代表的是高斯函数的作用域。
		//整体流程为：
		// 1 将原始点集和待配准点集平移到各自的中心点；(默认输入数据已经旋转到中心)
		// 2 将待配准点集中的每个点模拟成一个三维高斯函数；
		// 3 计算两个点集的加权协方差矩阵；
		// 4 对协方差矩阵进行奇异值分解，求解旋转矩阵；
		// 5 求解平移矩阵；
		// 6 求解变换矩阵
		// 
		// 
		// 
		//当sigama过小时，点与点之间的高斯函数不覆盖，函数优化过程中容易因为导数过小而停止，陷入局部极小值。
		//当sigama过大是，函数的收敛速度就会变慢,且整个模型更接近圆形，而不是点云的形状。
		//总结，sigama越小，模型越锐利，越大则越平滑。
		//sigama的取值推荐为点云的采样间隔的【0.3-1】
		double sigama = sigama_base;// (1E1) / 4.0;
		Eigen::MatrixXd matrix_base_expend(3, matrix_base.cols()), matrix_regist_expend(3, matrix_base.cols());
		//首先平移到中心
		//Eigen::MatrixXd matrix_base_expend_centered = matrix_base.colwise() - matrix_base.rowwise().mean();
		//Eigen::MatrixXd matrix_regist_expend_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();
		//计算 待匹配点云的加权点云
		std::vector<double> weight_set;
		double sum_all = 0;//目标卷积函数的值
		for (int i = 0; i < matrix_base.cols(); i++)
		{
			matrix_base_expend.col(i) = matrix_base.col(i);
			Eigen::MatrixXd matrix_regist_expend_single = Eigen::Matrix<double, 3, 1>::Zero();
			double sum = 0;
			for (int j = 0; j < matrix_regist.cols(); j++)
			{
				//根据距离计算权重
				double distance = (matrix_base.col(i) - matrix_regist.col(j)).norm();
				if (distance > 2 * sigama)
				{
					//continue;
				}
				double weight = exp(-std::pow(distance / (2 * sigama), 2));
				//std::cout << std::pow((matrix_base_expend_centered.col(i) - matrix_regist_expend_centered.col(j)).norm() / (2 * sigama),2) << std::endl;
				sum += weight;
				matrix_regist_expend_single += weight * matrix_regist.col(j);
				//std::cout << "权重：" << weight << std::endl;
			}
			weight_set.push_back(sum);
			matrix_regist_expend_single /= sum;
			matrix_regist_expend.col(i) = matrix_regist_expend_single;
			sum_all += sum;
			//std::cout<< "每个点的权重和："<<i<<"    "<< sum << std::endl;
		}

		//继续计算一次中心
		//Eigen::MatrixXd matrix_regist_expend2 = matrix_regist_expend.colwise() - matrix_regist_expend.rowwise().mean();

		//计算加权协方差矩阵
		Eigen::MatrixXd covariance_matrix = Eigen::Matrix3d::Zero();
		Eigen::MatrixXd covariance_matrix_single = Eigen::Matrix3d::Zero();
		for (size_t i = 0; i < matrix_base_expend.cols(); i++)
		{
			//double weight =  exp(-std::pow((matrix_base_expend_centered.col(i) - matrix_regist_expend_centered.col(i)).norm() / (2 * sigama), 2));
			//sum += weight;
			covariance_matrix += matrix_base_expend.col(i) * matrix_regist_expend.col(i).transpose();
			//std::cout << "covariance_matrix_single:" << covariance_matrix_single << std::endl;
			//std::cout << "权重：" << weight << std::endl;
			//if ((i+1)% matrix_base.cols() == 0)
			{
				//covariance_matrix = covariance_matrix + covariance_matrix_single/sum;
				//std::cout << "covariance_matrix_single:" << covariance_matrix_single << std::endl;
				//std::cout << "covariance_matrix_single_contrast:" << matrix_base_expend_centered.col(i)* matrix_base_expend_centered.col((i) / matrix_regist.cols()).transpose() << std::endl;
				//std::cout<<"sum::"<< sum << std::endl;
				//sum = 0;
				//covariance_matrix_single = Eigen::Matrix3d::Zero();
			}
			//std::cout << "系数：" << (matrix_base_centered.col(i) - matrix_regist_centered.col(i)).norm() / (2 * sigama) << std::endl;
		}

		//Eigen::MatrixXd matrix_base_centered = matrix_base.colwise() - matrix_base.rowwise().mean();
		//Eigen::MatrixXd matrix_regist_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();
		//Eigen::MatrixXd covariance_matrix_contrast = matrix_base_centered * matrix_regist_centered.transpose();
		//std::cout << "对比协方差矩阵：covariance_matrix " << covariance_matrix << std::endl;
		//std::cout << "对比协方差矩阵：covariance_matrix_contrast" << covariance_matrix_contrast << std::endl;
		//std::cout<<"对比协方差矩阵："<<(covariance_matrix_contrast- covariance_matrix ).norm()<< std::endl;

		//svd 方法精确求解R矩阵
		//Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
		//Eigen::MatrixXd rotation_matrix = svd.matrixV() * svd.matrixU().transpose();
		//Eigen::Vector3d translation_matrix = Eigen::MatrixXd::Zero(3, 1);// matrix_regist_expend.rowwise().mean();// -rotation_matrix * matrix_base.rowwise().mean();

		//使用LM方法迭代求解RT旋转矩阵
		Eigen::MatrixXd Trans_matrix_LM = calTransformMatrixLM(matrix_base_expend, matrix_regist_expend);

		//使用LM方法迭代求解尺度变换矩阵
		//Eigen::MatrixXd Trans_matrix_LM = calTransformMatrixLMScale(matrix_base_expend, matrix_regist_expend);

		//使用LM方法迭代求解加权矩阵
		//Eigen::MatrixXd Trans_matrix_LM = calTransformMatrixLMWeight(matrix_base_expend, matrix_regist_expend,weight_set);

		//使用LM方法迭代求解加权尺度变换矩阵
		//Eigen::MatrixXd Trans_matrix_LM = calTransformMatrixLMScaleWeight(matrix_base_expend, matrix_regist_expend,weight_set);


		//Eigen::Vector3d translation_matrix = matrix_regist_expend.rowwise().mean();
		//std::cout<<"平移向量："<< matrix_regist_expend.rowwise().mean() << std::endl;
		//std::cout<<"平移矩阵："<< translation_matrix << std::endl;
		//translation_matrix << 0, 0, 0;


		//方法1：使用SVD分解方法求解变换矩阵
		//Eigen::MatrixXd transform_matrix = getTransformMatrix(rotation_matrix, translation_matrix);
		//方法2：使用LM方法求解变换矩阵
		//Trans_matrix_LM.block(0, 3, 3, 1) += matrix_regist.rowwise().mean() - Trans_matrix_LM.block(0, 0, 3, 3) * matrix_base.rowwise().mean();		
		matrix_trans = Trans_matrix_LM;

		matrix_error = matrix_regist_expend - transformMatrix(Trans_matrix_LM, matrix_base_expend);
		//std::cout << "Trans_matrix_LM：" << Trans_matrix_LM << std::endl;
		//std::cout<<"dY/dR 的偏导数，=0 为函数收敛到极致："<<error.sum() << std::endl;
		//std::cout << "卷积函数函数值（需要求最大值）：" << sum_all << std::endl;
		return 1;
	}
	bool calTransformMatrix_extendMatrix(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, Eigen::MatrixXd& matrix_trans, Eigen::MatrixXd& matrix_error, double sigama_base)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix(matrix_base, matrix_regist, matrix_trans, matrix_error, sigama_base);
	}

	//拓展的基于三维高斯函数的配准方法，多次步骤生成变换位姿；期望在这个函数中得到通用的变换矩阵
	bool calTransformMatrix_extendMatrix(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, Eigen::MatrixXd& identity)
	{
		//首先需要预处理数据
		//1 将数据转换为matrix格式
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		//2 首先生成一些预设的旋转矩阵
		std::vector<Eigen::MatrixXd> rotation_matrix_set;
		int interval = 2;
		for (size_t i = 0; i < interval; i++)
		{
			for (size_t j = 0; j < interval; j++)
			{
				for (size_t k = 0; k < interval; k++)
				{
					Eigen::MatrixXd pre_matrix = getRotationMatrix(Eigen::Vector3d(i * M_PI / interval, j * 2 * M_PI / interval, k * 2 * M_PI / interval));
					rotation_matrix_set.push_back(pre_matrix);
				}
			}
		}
		identity = Eigen::Matrix4d::Identity();
		//3 存储函数值和对应的索引值
		std::vector<double> function_set;
		std::vector<Eigen::MatrixXd> Matrix_set;
		//4 计算每个初始位姿对应的最大函数值
		for (size_t k = 0; k < rotation_matrix_set.size(); k++)
		{
			//4.1 首先设置初始旋转位姿
			identity = Eigen::Matrix4d::Identity();
			identity.block(0, 0, 3, 3) = rotation_matrix_set[k];
			std::cout << "identity:" << identity << std::endl;
			//std::cout << "第" << k << "个位置" << std::endl;

			//4.2 先将点旋转到预定位置
			Eigen::MatrixXd matrix_base1 = transformMatrix(identity, matrix_base);
			//4.3 将点云数据中心化
			Eigen::MatrixXd matrix_base_centered = matrix_base1.colwise() - matrix_base1.rowwise().mean();
			Eigen::MatrixXd matrix_regist_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();

			Eigen::MatrixXd transform_matrix_iterate = Eigen::Matrix4d::Identity();
			//4.4 迭代计算最优的变换矩阵
			for (size_t i = 0; i < 20; i++)
			{
				//迭代的初始矩阵
				Eigen::MatrixXd matrix_iterate = transformMatrix(transform_matrix_iterate, matrix_base_centered);
				//std::cout << "matrix_iterate:" << matrix_iterate << std::endl;
				//std::cout << "第" << i << "次迭代" << std::endl;
				//计算单次迭代的变换矩阵
				Eigen::MatrixXd transform_matrix, error_matrix;
				calTransformMatrix_extendMatrix(matrix_iterate, matrix_regist_centered, transform_matrix, error_matrix);
				transform_matrix_iterate = transform_matrix * transform_matrix_iterate;
				//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
			}
			//4.5 计算最终的变换矩阵
			//去中心化
			//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
			transform_matrix_iterate.block(0, 3, 3, 1) += matrix_regist.rowwise().mean() - transform_matrix_iterate.block(0, 0, 3, 3) * matrix_base1.rowwise().mean();
			//std::cout << "matrix_regist.rowwise().mean():" << matrix_regist.rowwise().mean() << std::endl;
			//std::cout << "matrix_base1.rowwise().mean():" << matrix_base1.rowwise().mean() << std::endl;

			identity = transform_matrix_iterate * identity;
			std::cout << "identity:" << identity << std::endl;

			//4.6 评价标准函数
			Eigen::MatrixXd afterRotate = transformMatrix(identity, matrix_base);
			double sigama = (1E1) / 3.0;
			double sum_all = 0;//目标卷积函数的值
			for (int i = 0; i < matrix_base.cols(); i++)
			{
				double sum = 0;
				for (int j = 0; j < matrix_regist.cols(); j++)
				{
					//根据距离计算权重
					double weight = exp(-std::pow((afterRotate.col(i) - matrix_regist.col(j)).norm() / (2 * sigama), 2));
					//std::cout << std::pow((matrix_base_expend_centered.col(i) - matrix_regist_expend_centered.col(j)).norm() / (2 * sigama),2) << std::endl;
					sum += weight;
					//std::cout << "权重：" << weight << std::endl;
				}
				sum_all += sum;
				//std::cout<< "每个点的权重和："<<i<<"    "<< sum << std::endl;
			}


			function_set.push_back(sum_all);
			Matrix_set.push_back(identity);
			std::cout << "Index and Benchmark： " << k << "    " << sum_all << std::endl;
		}
		//5 找到最大值，并将对应的旋转矩阵返回
		double max = 0;
		int index = 0;
		for (size_t i = 0; i < function_set.size(); i++)
		{
			if (function_set[i] > max)
			{
				max = function_set[i];
				index = i;
			}
		}
		identity = Matrix_set[index];
		std::cout << "最大值： " << max << std::endl;
		return 1;
	}
	bool calTransformMatrix_extendMatrix_disorder(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		//首先需要预处理数据
		//1 将数据转换为matrix格式
		Eigen::MatrixXd matrix_base = pcloud_base;
		Eigen::MatrixXd matrix_regist = pcloud_regist;
		identity = Eigen::Matrix4d::Identity();
		//3 存储函数值和对应的索引值
		std::vector<double> function_set;
		std::vector<Eigen::MatrixXd> Matrix_set;
		double score_threshold = 0.95 * std::max(pcloud_regist.cols(), pcloud_base.cols());//分数阈值，当分数大于阈值时，直接返回
		//4 计算每个初始位姿对应的最大函数值
		//4.3 将点云数据中心化
		Eigen::MatrixXd matrix_base_centered = matrix_base.colwise() - matrix_base.rowwise().mean();
		Eigen::MatrixXd matrix_regist_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();
		Eigen::MatrixXd transform_matrix_iterate = Eigen::Matrix4d::Identity();
		//4.4 迭代计算最优的变换矩阵
		std::vector<double> error_set;
		for (size_t i = 0; i < 40; i++)
		{
			//迭代的初始矩阵
			Eigen::MatrixXd matrix_iterate = transformMatrix(transform_matrix_iterate, matrix_base_centered);
			//std::cout << "matrix_iterate:" << matrix_iterate << std::endl;
			//std::cout << "第" << i << "次迭代" << std::endl;
			//计算单次迭代的变换矩阵
			Eigen::MatrixXd transform_matrix, error_matrix;
			calTransformMatrix_extendMatrix(matrix_iterate, matrix_regist_centered, transform_matrix, error_matrix, sigama);
			transform_matrix_iterate = transform_matrix * transform_matrix_iterate;
			//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
			error_set.push_back(error_matrix.norm());
			if (i > 2)
			{
				if (abs(error_set[i] - error_set[i - 1]) < 1E-5)
					//std::cout << "break: " << i << std::endl;
					break;
			}
		}
		//4.5 计算最终的变换矩阵
		//去中心化
		//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
		transform_matrix_iterate.block(0, 3, 3, 1) += matrix_regist.rowwise().mean() - transform_matrix_iterate.block(0, 0, 3, 3) * matrix_base.rowwise().mean();
		//std::cout << "matrix_regist.rowwise().mean():" << matrix_regist.rowwise().mean() << std::endl;
		//std::cout << "matrix_base1.rowwise().mean():" << matrix_base1.rowwise().mean() << std::endl;

		identity = transform_matrix_iterate * identity;
		//std::cout << "identity:" << identity << std::endl;

		//4.6 评价标准函数
		Eigen::MatrixXd afterRotate = transformMatrix(identity, matrix_base);
		//改变sigma，使其变为以最小值为基准
		//sigama += std::min(pcloud_regist.cols(), pcloud_base.cols())/ std::max(pcloud_regist.cols(), pcloud_base.cols());
		double sum_all = 0;//目标卷积函数的值
		for (int i = 0; i < matrix_base.cols(); i++)
		{
			double sum = 0;
			for (int j = 0; j < matrix_regist.cols(); j++)
			{
				//根据距离计算权重
				double weight = exp(-std::pow((afterRotate.col(i) - matrix_regist.col(j)).norm() / (2 * sigama), 2));
				//std::cout << std::pow((matrix_base_expend_centered.col(i) - matrix_regist_expend_centered.col(j)).norm() / (2 * sigama),2) << std::endl;
				//std::cout << (afterRotate.col(i) - matrix_regist.col(j)).norm() << endl;
				sum += weight;
				//std::cout << "权重：" << weight << std::endl;
			}
			sum_all += sum;
			//std::cout<< "每个点的权重和："<<i<<"    "<< sum << std::endl;
		}
		function_set.push_back(sum_all);
		Matrix_set.push_back(identity);
		std::cout << "Index and Benchmark： " << "    " << sum_all << "    " << sum_all / std::max(pcloud_regist.cols(), pcloud_base.cols()) << std::endl;
		//判断是否满足阈值要求，如果满足则直接中断循环
		if (sum_all > score_threshold)
		{
			//break;
		}
		//5 找到最大值，并将对应的旋转矩阵返回
		double max = 0;
		int index = 0;
		for (size_t i = 0; i < function_set.size(); i++)
		{
			if (function_set[i] > max)
			{
				max = function_set[i];
				index = i;
			}
		}
		identity = Matrix_set[index];
		score = max;
		//std::cout << "最大值： " << max << std::endl;
		return 1;
	}
	bool calTransformMatrix_extendMatrix_disorder(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_disorder(matrix_base, matrix_regist, score, identity, sigama);
	}
	bool calTransformMatrix_extendMatrix_disorder(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_disorder(matrix_base, matrix_regist, score, identity, sigama);
	}
	bool calTransformMatrix_extendMatrix_onlyZ(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		//首先需要预处理数据
		//1 将数据转换为matrix格式
		Eigen::MatrixXd matrix_base = pcloud_base;
		Eigen::MatrixXd matrix_regist = pcloud_regist;
		//2 首先生成一些预设的旋转矩阵,只旋转z轴
		//首先确定最基础的旋转角度，是输入点云和目标点云的副法向量的夹角
		Eigen::Vector3d pole_base, direction_base, pole_regist, direction_regist;
		getLineEquation(pcloud_base, pole_base, direction_base);
		getLineEquation(pcloud_regist, pole_regist, direction_regist);
		double angle_base = acos(dotProduct(direction_base, direction_regist));//一个正向或者反向的角度
		cout << "angle_base: " << angle_base << endl;

		//svd分解
		Eigen::MatrixXd U1, V1, U2, V2;
		Eigen::VectorXd sigama1, sigama2;
		solveSvd(pcloud_base, U1, sigama1, V1);
		solveSvd(pcloud_regist, U2, sigama2, V2);
		std::cout << "sigama1: " << sigama1 << std::endl;
		std::cout << "sigama2: " << sigama2 << std::endl;
		std::cout << "U1: " << U1 << std::endl;
		std::cout << "U2: " << U2 << std::endl;
		//std::cout<<"V1: "<<V1<<std::endl;
		//std::cout<<"V2: "<<V2<<std::endl;

		Eigen::MatrixXd rotate_svd = U2 * U1.transpose();
		Eigen::Vector3d eula_svd;
		//double angle_svd;
		eula_svd = getEulerAngle(rotate_svd);//svd分解的角度，大体方向是对的
		std::cout << "eula_svd: " << eula_svd << std::endl;
		Eigen::Vector3d rotate_normal_line(cos(angle_base), sin(angle_base), 0);
		Eigen::Vector3d rotate_normal_svd(cos(eula_svd[2]), sin(eula_svd[2]), 0);
		if (getAngleBetweenVectors(rotate_normal_line, rotate_normal_svd) > M_PI / 2)
		{
			//两个向量的夹角大于90度，说明line的角度是反向的
			angle_base += M_PI;
		}
		//在基准角度之上进行差分，对差分角度进行排序
		std::vector<Eigen::MatrixXd> rotation_matrix_set;
		std::vector<Eigen::Vector3d> angle_set;
		double interval = 60;
		for (size_t i = 0; i < interval; i++)
		{
			//0角度
			//angle_set.push_back(Eigen::Vector3d(-M_PI/2, 0, 0));
			//svd分解的角度
			//angle_set.push_back(eula_svd);
			//综合的角度
			angle_set.push_back(Eigen::Vector3d(0, 0, i * 2 * M_PI / interval + angle_base));//正向
			angle_set.push_back(Eigen::Vector3d(M_PI, 0, i * 2 * M_PI / interval + M_PI / 4 - angle_base));//镜像
		}
		//angle_set.push_back(Eigen::Vector3d(0, 0, 0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, M_PI + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, M_PI / 2.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 3 * M_PI / 2.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 3 * M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 5 * M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 7 * M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, 0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, M_PI + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, M_PI / 2.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, 3 * M_PI / 2.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, 3 * M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, 5 * M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(M_PI, 0, 7 * M_PI / 4.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 1 * M_PI / 8.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 3 * M_PI / 8.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 5 * M_PI / 8.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 7 * M_PI / 8.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 9 * M_PI / 8.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 11 * M_PI / 8.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 13 * M_PI / 8.0 + angle_base));
		//angle_set.push_back(Eigen::Vector3d(0, 0, 15 * M_PI / 8.0 + angle_base));

		for (size_t k = 0; k < angle_set.size(); k++)
		{
			Eigen::MatrixXd pre_matrix = getRotationMatrix(angle_set[k]);
			rotation_matrix_set.push_back(pre_matrix);
		}
		identity = Eigen::Matrix4d::Identity();
		//3 存储函数值和对应的索引值
		std::vector<double> function_set;
		std::vector<Eigen::MatrixXd> Matrix_set;
		double score_threshold = 0.95 * std::max(pcloud_regist.cols(), pcloud_base.cols());//分数阈值，当分数大于阈值时，直接返回
		//4 计算每个初始位姿对应的最大函数值
		for (size_t k = 0; k < rotation_matrix_set.size(); k++)
		{
			//4.1 首先设置初始旋转位姿
			identity = Eigen::Matrix4d::Identity();
			identity.block(0, 0, 3, 3) = rotation_matrix_set[k];
			//std::cout << "identity:" << identity << std::endl;
			//std::cout << "第" << k << "个位置" << std::endl;

			//4.2 先将点旋转到预定位置
			Eigen::MatrixXd matrix_base1 = transformMatrix(identity, matrix_base);
			//4.3 将点云数据中心化
			Eigen::MatrixXd matrix_base_centered = matrix_base1.colwise() - matrix_base1.rowwise().mean();
			Eigen::MatrixXd matrix_regist_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();

			Eigen::MatrixXd transform_matrix_iterate = Eigen::Matrix4d::Identity();
			//4.4 迭代计算最优的变换矩阵
			std::vector<double> error_set;
			for (size_t i = 0; i < 2; i++)
			{
				//迭代的初始矩阵
				Eigen::MatrixXd matrix_iterate = transformMatrix(transform_matrix_iterate, matrix_base_centered);
				//std::cout << "matrix_iterate:" << matrix_iterate << std::endl;
				//std::cout << "第" << i << "次迭代" << std::endl;
				//计算单次迭代的变换矩阵
				Eigen::MatrixXd transform_matrix, error_matrix;
				calTransformMatrix_extendMatrix(matrix_iterate, matrix_regist_centered, transform_matrix, error_matrix, sigama);
				transform_matrix_iterate = transform_matrix * transform_matrix_iterate;
				//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
				error_set.push_back(error_matrix.norm());
				if (i > 2)
				{
					if (abs(error_set[i] - error_set[i - 1]) < 1E-5)
						//std::cout << "break: " << i << std::endl;
						break;
				}
			}
			//4.5 计算最终的变换矩阵
			//去中心化
			//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
			transform_matrix_iterate.block(0, 3, 3, 1) += matrix_regist.rowwise().mean() - transform_matrix_iterate.block(0, 0, 3, 3) * matrix_base1.rowwise().mean();
			//std::cout << "matrix_regist.rowwise().mean():" << matrix_regist.rowwise().mean() << std::endl;
			//std::cout << "matrix_base1.rowwise().mean():" << matrix_base1.rowwise().mean() << std::endl;

			identity = transform_matrix_iterate * identity;
			//std::cout << "identity:" << identity << std::endl;

			//4.6 评价标准函数
			Eigen::MatrixXd afterRotate = transformMatrix(identity, matrix_base);
			//改变sigma，使其变为以最小值为基准
			//sigama += std::min(pcloud_regist.cols(), pcloud_base.cols())/ std::max(pcloud_regist.cols(), pcloud_base.cols());
			double sum_all = 0;//目标卷积函数的值
			for (int i = 0; i < matrix_base.cols(); i++)
			{
				double sum = 0;
				for (int j = 0; j < matrix_regist.cols(); j++)
				{
					//根据距离计算权重
					double weight = exp(-std::pow((afterRotate.col(i) - matrix_regist.col(j)).norm() / (2 * sigama), 2));
					//std::cout << std::pow((matrix_base_expend_centered.col(i) - matrix_regist_expend_centered.col(j)).norm() / (2 * sigama),2) << std::endl;
					//std::cout << (afterRotate.col(i) - matrix_regist.col(j)).norm() << endl;
					sum += weight;
					//std::cout << "权重：" << weight << std::endl;
				}
				sum_all += sum;
				//std::cout<< "每个点的权重和："<<i<<"    "<< sum << std::endl;
			}
			function_set.push_back(sum_all);
			Matrix_set.push_back(identity);
			std::cout << "Index and Benchmark： " << k << "    " << sum_all << "    " << sum_all / std::max(pcloud_regist.cols(), pcloud_base.cols()) << std::endl;
			//判断是否满足阈值要求，如果满足则直接中断循环
			if (sum_all > score_threshold)
			{
				//break;
			}
		}
		//5 找到最大值，并将对应的旋转矩阵返回
		double max = 0;
		int index = 0;
		for (size_t i = 0; i < function_set.size(); i++)
		{
			if (function_set[i] > max)
			{
				max = function_set[i];
				index = i;
			}
		}
		identity = Matrix_set[index];
		score = max;
		//std::cout << "最大值： " << max << std::endl;
		return 1;
	}
	bool calTransformMatrix_extendMatrix_onlyZ(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_onlyZ(matrix_base, matrix_regist, score, identity, sigama);
	}
	bool calTransformMatrix_extendMatrix_onlyZ(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_onlyZ(matrix_base, matrix_regist, score, identity, sigama);
	}
	bool calTransformMatrix_extendMatrix_3D(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		//首先需要预处理数据
		//1 将数据转换为matrix格式
		Eigen::MatrixXd matrix_base = pcloud_base;
		Eigen::MatrixXd matrix_regist = pcloud_regist;
		//2 首先生成一些预设的旋转矩阵,只旋转z轴
		//首先确定最基础的旋转角度，是输入点云和目标点云的副法向量的夹角
		Eigen::Vector3d pole_base, direction_base, pole_regist, direction_regist;
		getLineEquation(pcloud_base, pole_base, direction_base);
		getLineEquation(pcloud_regist, pole_regist, direction_regist);
		double angle_base = acos(dotProduct(direction_base, direction_regist));//一个正向或者反向的角度
		cout << "angle_base: " << angle_base << endl;

		//在基准角度之上进行差分，对差分角度进行排序
		std::vector<Eigen::MatrixXd> rotation_matrix_set;
		std::vector<Eigen::Vector3d> angle_set;
		double interval = 10;
		for (size_t i = 0; i < interval; i++)
		{
			for (size_t j = 0; j < interval; j++)
			{
				for (size_t l = 0; l < interval; l++)
				{
					angle_set.push_back(Eigen::Vector3d(i * 2 * M_PI / interval + angle_base, j * 2 * M_PI / interval + angle_base, l * 2 * M_PI / interval + angle_base));//正向
				}
			}
		}

		for (size_t k = 0; k < angle_set.size(); k++)
		{
			Eigen::MatrixXd pre_matrix = getRotationMatrix(angle_set[k]);
			rotation_matrix_set.push_back(pre_matrix);
		}
		identity = Eigen::Matrix4d::Identity();
		//3 存储函数值和对应的索引值
		std::vector<double> function_set;
		std::vector<Eigen::MatrixXd> Matrix_set;
		double score_threshold = 0.95 * std::max(pcloud_regist.cols(), pcloud_base.cols());//分数阈值，当分数大于阈值时，直接返回
		//4 计算每个初始位姿对应的最大函数值
		for (size_t k = 0; k < rotation_matrix_set.size(); k++)
		{
			//4.1 首先设置初始旋转位姿
			identity = Eigen::Matrix4d::Identity();
			identity.block(0, 0, 3, 3) = rotation_matrix_set[k];
			//std::cout << "identity:" << identity << std::endl;
			//std::cout << "第" << k << "个位置" << std::endl;

			//4.2 先将点旋转到预定位置
			Eigen::MatrixXd matrix_base1 = transformMatrix(identity, matrix_base);
			//4.3 将点云数据中心化
			Eigen::MatrixXd matrix_base_centered = matrix_base1.colwise() - matrix_base1.rowwise().mean();
			Eigen::MatrixXd matrix_regist_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();

			Eigen::MatrixXd transform_matrix_iterate = Eigen::Matrix4d::Identity();
			//4.4 迭代计算最优的变换矩阵
			std::vector<double> error_set;
			for (size_t i = 0; i < 2; i++)
			{
				//迭代的初始矩阵
				Eigen::MatrixXd matrix_iterate = transformMatrix(transform_matrix_iterate, matrix_base_centered);
				//std::cout << "matrix_iterate:" << matrix_iterate << std::endl;
				//std::cout << "第" << i << "次迭代" << std::endl;
				//计算单次迭代的变换矩阵
				Eigen::MatrixXd transform_matrix, error_matrix;
				calTransformMatrix_extendMatrix(matrix_iterate, matrix_regist_centered, transform_matrix, error_matrix, sigama);
				transform_matrix_iterate = transform_matrix * transform_matrix_iterate;
				//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
				error_set.push_back(error_matrix.norm());
				if (i > 2)
				{
					if (abs(error_set[i] - error_set[i - 1]) < 1E-5)
						//std::cout << "break: " << i << std::endl;
						break;
				}
			}
			//4.5 计算最终的变换矩阵
			//去中心化
			//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
			transform_matrix_iterate.block(0, 3, 3, 1) += matrix_regist.rowwise().mean() - transform_matrix_iterate.block(0, 0, 3, 3) * matrix_base1.rowwise().mean();
			//std::cout << "matrix_regist.rowwise().mean():" << matrix_regist.rowwise().mean() << std::endl;
			//std::cout << "matrix_base1.rowwise().mean():" << matrix_base1.rowwise().mean() << std::endl;

			identity = transform_matrix_iterate * identity;
			//std::cout << "identity:" << identity << std::endl;

			//4.6 评价标准函数
			Eigen::MatrixXd afterRotate = transformMatrix(identity, matrix_base);
			//改变sigma，使其变为以最小值为基准
			//sigama += std::min(pcloud_regist.cols(), pcloud_base.cols())/ std::max(pcloud_regist.cols(), pcloud_base.cols());
			double sum_all = 0;//目标卷积函数的值
			for (int i = 0; i < matrix_base.cols(); i++)
			{
				double sum = 0;
				for (int j = 0; j < matrix_regist.cols(); j++)
				{
					//根据距离计算权重
					double weight = exp(-std::pow((afterRotate.col(i) - matrix_regist.col(j)).norm() / (2 * sigama), 2));
					//std::cout << std::pow((matrix_base_expend_centered.col(i) - matrix_regist_expend_centered.col(j)).norm() / (2 * sigama),2) << std::endl;
					//std::cout << (afterRotate.col(i) - matrix_regist.col(j)).norm() << endl;
					sum += weight;
					//std::cout << "权重：" << weight << std::endl;
				}
				sum_all += sum;
				//std::cout<< "每个点的权重和："<<i<<"    "<< sum << std::endl;
			}
			function_set.push_back(sum_all);
			Matrix_set.push_back(identity);
			std::cout << "Index and Benchmark： " << k << "    " << sum_all << "    " << sum_all / std::max(pcloud_regist.cols(), pcloud_base.cols()) << std::endl;
			//判断是否满足阈值要求，如果满足则直接中断循环
			if (sum_all > score_threshold)
			{
				//break;
			}
		}
		//5 找到最大值，并将对应的旋转矩阵返回
		double max = 0;
		int index = 0;
		for (size_t i = 0; i < function_set.size(); i++)
		{
			if (function_set[i] > max)
			{
				max = function_set[i];
				index = i;
			}
		}
		identity = Matrix_set[index];
		score = max;
		//std::cout << "最大值： " << max << std::endl;
		return 1;
	}
	bool calTransformMatrix_extendMatrix_3D(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_3D(matrix_base, matrix_regist, score, identity, sigama);
	}
	bool calTransformMatrix_extendMatrix_3D(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_3D(matrix_base, matrix_regist, score, identity, sigama);
	}
	bool calTransformMatrix_extendMatrix_initialAngle(const Eigen::MatrixXd& pcloud_base, const Eigen::MatrixXd& pcloud_regist, Eigen::MatrixXd& initial_transfrom, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		//只使用initial_angle 的初始角度
		//首先需要预处理数据
		//1 将数据转换为matrix格式
		Eigen::MatrixXd matrix_base = pcloud_base;
		Eigen::MatrixXd matrix_regist = pcloud_regist;
		//2 首先生成一些预设的旋转矩阵,只旋转z轴

		//在基准角度之上进行差分，对差分角度进行排序
		std::vector<Eigen::MatrixXd> rotation_matrix_set;
		rotation_matrix_set.push_back(initial_transfrom.block(0, 0, 3, 3));
		identity = Eigen::Matrix4d::Identity();
		//3 存储函数值和对应的索引值
		std::vector<double> function_set;
		std::vector<Eigen::MatrixXd> Matrix_set;
		double score_threshold = 0.95 * std::max(pcloud_regist.cols(), pcloud_base.cols());//分数阈值，当分数大于阈值时，直接返回
		//4 计算每个初始位姿对应的最大函数值
		for (size_t k = 0; k < rotation_matrix_set.size(); k++)
		{
			//4.1 首先设置初始旋转位姿
			identity = Eigen::Matrix4d::Identity();
			identity.block(0, 0, 3, 3) = rotation_matrix_set[k];
			//std::cout << "identity:" << identity << std::endl;
			//std::cout << "第" << k << "个位置" << std::endl;

			//4.2 先将点旋转到预定位置
			Eigen::MatrixXd matrix_base1 = transformMatrix(identity, matrix_base);
			//4.3 将点云数据中心化
			Eigen::MatrixXd matrix_base_centered = matrix_base1.colwise() - matrix_base1.rowwise().mean();
			Eigen::MatrixXd matrix_regist_centered = matrix_regist.colwise() - matrix_regist.rowwise().mean();

			Eigen::MatrixXd transform_matrix_iterate = Eigen::Matrix4d::Identity();
			//4.4 迭代计算最优的变换矩阵
			std::vector<double> error_set;
			for (size_t i = 0; i < 40; i++)
			{
				//迭代的初始矩阵
				Eigen::MatrixXd matrix_iterate = transformMatrix(transform_matrix_iterate, matrix_base_centered);
				//std::cout << "matrix_iterate:" << matrix_iterate << std::endl;
				//std::cout << "第" << i << "次迭代" << std::endl;
				//计算单次迭代的变换矩阵
				Eigen::MatrixXd transform_matrix, error_matrix;
				calTransformMatrix_extendMatrix(matrix_iterate, matrix_regist_centered, transform_matrix, error_matrix, sigama);
				transform_matrix_iterate = transform_matrix * transform_matrix_iterate;
				//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
				error_set.push_back(error_matrix.norm());
				if (i > 2)
				{
					if (abs(error_set[i] - error_set[i - 1]) < 1E-5)
						break;
				}
			}
			//4.5 计算最终的变换矩阵
			//去中心化
			//std::cout << "transform_matrix_iterate:" << transform_matrix_iterate << std::endl;
			transform_matrix_iterate.block(0, 3, 3, 1) += matrix_regist.rowwise().mean() - transform_matrix_iterate.block(0, 0, 3, 3) * matrix_base1.rowwise().mean();
			//std::cout << "matrix_regist.rowwise().mean():" << matrix_regist.rowwise().mean() << std::endl;
			//std::cout << "matrix_base1.rowwise().mean():" << matrix_base1.rowwise().mean() << std::endl;

			identity = transform_matrix_iterate * identity;
			//std::cout << "identity:" << identity << std::endl;

			//4.6 评价标准函数
			Eigen::MatrixXd afterRotate = transformMatrix(identity, matrix_base);
			//改变sigma，使其变为以最小值为基准
			//sigama += std::min(pcloud_regist.cols(), pcloud_base.cols())/ std::max(pcloud_regist.cols(), pcloud_base.cols());
			double sum_all = 0;//目标卷积函数的值
			for (int i = 0; i < matrix_base.cols(); i++)
			{
				double sum = 0;
				for (int j = 0; j < matrix_regist.cols(); j++)
				{
					//根据距离计算权重
					double weight = exp(-std::pow((afterRotate.col(i) - matrix_regist.col(j)).norm() / (2 * sigama), 2));
					//std::cout << std::pow((matrix_base_expend_centered.col(i) - matrix_regist_expend_centered.col(j)).norm() / (2 * sigama),2) << std::endl;
					//std::cout << (afterRotate.col(i) - matrix_regist.col(j)).norm() << endl;
					sum += weight;
					//std::cout << "权重：" << weight << std::endl;
				}
				sum_all += sum;
				//std::cout<< "每个点的权重和："<<i<<"    "<< sum << std::endl;
			}
			function_set.push_back(sum_all);
			Matrix_set.push_back(identity);
			std::cout << "Index and Benchmark： " << k << "    " << sum_all << "    " << sum_all / std::max(pcloud_regist.cols(), pcloud_base.cols()) << std::endl;
			//判断是否满足阈值要求，如果满足则直接中断循环
			if (sum_all > score_threshold)
			{
				//break;
			}
		}
		//5 找到最大值，并将对应的旋转矩阵返回
		double max = 0;
		int index = 0;
		for (size_t i = 0; i < function_set.size(); i++)
		{
			if (function_set[i] > max)
			{
				max = function_set[i];
				index = i;
			}
		}
		identity = Matrix_set[index];
		score = max;
		//std::cout << "最大值： " << max << std::endl;
		return 1;
	}
	bool calTransformMatrix_extendMatrix_initialAngle(const std::vector<Eigen::Vector3d>& pcloud_base, const std::vector<Eigen::Vector3d>& pcloud_regist, Eigen::MatrixXd& initial_transfrom, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_initialAngle(matrix_base, matrix_regist, initial_transfrom, score, identity, sigama);
	}
	bool calTransformMatrix_extendMatrix_initialAngle(const pcl::PointCloud<pcl::PointXYZ>& pcloud_base, const pcl::PointCloud<pcl::PointXYZ>& pcloud_regist, Eigen::MatrixXd& initial_transfrom, double& score, Eigen::MatrixXd& identity, double sigama)
	{
		Eigen::MatrixXd matrix_base = getMatrix(pcloud_base);
		Eigen::MatrixXd matrix_regist = getMatrix(pcloud_regist);
		return calTransformMatrix_extendMatrix_initialAngle(matrix_base, matrix_regist, initial_transfrom, score, identity, sigama);
	}
	//据旋转轴和旋转角度来生成旋转四元数
	Eigen::Quaterniond getRotationQuaternion(const Eigen::Vector3d& axis, double angle)
	{
		Eigen::AngleAxisd rotation_vector(angle, axis);
		Eigen::Quaterniond rotation_quaternion(rotation_vector);
		return rotation_quaternion;
	}
	template <typename T>
	std::vector<T> sortVector(std::vector<T>& vector)
	{
		std::vector<T> tempt_vector = vector;
		if (vector.size() == 0)
		{
			return tempt_vector;
		}
		std::sort(tempt_vector.begin(), tempt_vector.end());
		return tempt_vector;
	}
	std::vector<Eigen::Vector3d> sortVector(std::vector<Eigen::Vector3d>& vector, Eigen::Vector3d axis)
	{
		std::vector<Eigen::Vector3d> tempt_vector = vector;
		if (vector.size() == 0)
		{
			return tempt_vector;
		}
		//根据axis的方向，对vector进行排序,从小到大
		std::sort(tempt_vector.begin(), tempt_vector.end(), [axis](Eigen::Vector3d a, Eigen::Vector3d b) {return dotProduct(a, axis) < dotProduct(b, axis); });
		return tempt_vector;
	}
	pcl::PointCloud<pcl::PointXYZ> sortVector(pcl::PointCloud<pcl::PointXYZ>& vector, Eigen::Vector3d axis)
	{
		if (vector.size() == 0)
		{
			return vector;
		}
		//根据axis的方向，对vector进行排序
		pcl::PointCloud<pcl::PointXYZ> tempt_vector = vector;
		std::sort(tempt_vector.begin(), tempt_vector.end(), [axis](pcl::PointXYZ a, pcl::PointXYZ b) {return dotProduct(Eigen::Vector3d(a.x, a.y, a.z), axis) < dotProduct(Eigen::Vector3d(b.x, b.y, b.z), axis); });
		return tempt_vector;
	}
	int findVector(double input, std::vector<double>& vector)
	{
		for (int i = 0; i < vector.size(); i++)
		{
			if (input == vector[i])
			{
				return i;
			}
		}
		return -1;
	}
	int findVector(int input, std::vector<int>& vector)
	{
		for (int i = 0; i < vector.size(); i++)
		{
			if (input == vector[i])
			{
				return i;
			}
		}
		return -1;
	}
	int findEigenVector(Eigen::Vector3d input, std::vector<Eigen::Vector3d>& vector)
	{
		for (int i = 0; i < vector.size(); i++)
		{
			if (input == vector[i])
			{
				return i;
			}
		}
		return -1;
	}
	Eigen::Vector3d crossProduct(const Eigen::Vector3d& vector1, const Eigen::Vector3d& vector2)
	{
		Eigen::Vector3d cross_product;
		cross_product << vector1[1] * vector2[2] - vector1[2] * vector2[1],
			vector1[2] * vector2[0] - vector1[0] * vector2[2],
			vector1[0] * vector2[1] - vector1[1] * vector2[0];
		return cross_product;
	}

	double dotProduct(const Eigen::VectorXd& vector1, const Eigen::VectorXd& vector2)
	{
		double dot_product = 0;
		for (int i = 0; i < vector1.size(); i++)
		{
			dot_product += vector1[i] * vector2[i];
		}
		return dot_product;
	}
	template<typename T>
	T sumVector(std::vector<T> input)
	{
		T sum = 0;
		for (int i = 0; i < input.size(); i++)
		{
			sum += input[i];
		}
		return sum;
	}
	template<typename T>
	T meanVector(std::vector<T> input)
	{
		T sum = sumVector(input);
		if (input.size() != 0)
		{
			return sum / input.size();
		}
		return 0;
	}
	template<typename T>
	T varianceVector(std::vector<T> input)
	{
		T mean = meanVector(input);
		T variance = 0;
		for (int i = 0; i < input.size(); i++)
		{
			variance += (input[i] - mean) * (input[i] - mean);
		}
		if (input.size() != 0)
		{
			return variance / input.size();
		}
		return 0;
	}
	double limitedRange(double input, double rangUp, double rangeDown)
	{
		if (input > rangUp)
		{
			return rangUp;
		}
		else if (input < rangeDown)
		{
			return rangeDown;
		}
		else
		{
			return input;
		}
	}

	void limitRange(double& input, double rangUp, double rangeDown)
	{
		if (input > rangUp)
		{
			input = rangUp;
		}
		else if (input < rangeDown)
		{
			input = rangeDown;
		}
	}
	double standardDeviationVector(std::vector<double> input)
	{
		double variance = varianceVector(input);
		return sqrt(variance);

	}

	double getVectorNorm(const Eigen::VectorXd& vector)
	{
		double vector_norm = 0;
		for (int i = 0; i < vector.size(); i++)
		{
			vector_norm += vector[i] * vector[i];
		}
		vector_norm = sqrt(vector_norm);
		return vector_norm;
	}
	double getVectorNorm(const Eigen::Vector3d& vector, pcl::PointXYZ v2)
	{
		double vector_norm = 0;
		vector_norm = sqrt((vector[0] - v2.x) * (vector[0] - v2.x) + (vector[1] - v2.y) * (vector[1] - v2.y) + (vector[2] - v2.z) * (vector[2] - v2.z));
		return vector_norm;
	}
	Eigen::Vector3d getUnitVector(const Eigen::Vector3d& vector)
	{
		double vector_norm = getVectorNorm(vector);
		Eigen::Vector3d unit_vector;
		unit_vector << vector[0] / vector_norm, vector[1] / vector_norm, vector[2] / vector_norm;
		return unit_vector;
	}
	Eigen::Vector3d getCenterPoint(std::vector<Eigen::Vector3d>& points_cloud)
	{
		Eigen::Vector3d center_point;
		center_point << 0, 0, 0;
		for (int i = 0; i < points_cloud.size(); i++)
		{
			center_point[0] += points_cloud[i][0];
			center_point[1] += points_cloud[i][1];
			center_point[2] += points_cloud[i][2];
		}
		if (points_cloud.size() != 0)
		{
			center_point = center_point / points_cloud.size();
		}
		return center_point;
	}
	Eigen::Vector3d getCenterPoint(pcl::PointCloud<pcl::PointXYZ>& points_cloud_pcl)
	{
		Eigen::Vector3d center_point;
		center_point << 0, 0, 0;
		for (int i = 0; i < points_cloud_pcl.size(); i++)
		{
			center_point[0] += points_cloud_pcl[i].x;
			center_point[1] += points_cloud_pcl[i].y;
			center_point[2] += points_cloud_pcl[i].z;
		}
		if (points_cloud_pcl.size() != 0)
		{
			center_point = center_point / points_cloud_pcl.size();
		}
		return center_point;
	}
	/*Eigen::Vector3d getTopPoint(std::vector<Eigen::Vector3d>& points_cloud)
	{
		Eigen::Vector3d top_point;
		top_point << 0, 0, 0;
		double max = -100000;
		for (int i = 0; i < points_cloud.size(); i++)
		{
			if (points_cloud[i][2] > max)
			{
				max = points_cloud[i][2];
				top_point = points_cloud[i];
			}
		}
		return top_point;
	}*/
	Eigen::Vector3d getTopPoint(pcl::PointCloud<pcl::PointXYZ>& points_cloud_pcl)
	{
		Eigen::Vector3d top_point;
		top_point << 0, 0, 0;
		double max = -100000;
		for (int i = 0; i < points_cloud_pcl.size(); i++)
		{
			if (points_cloud_pcl[i].z > max)
			{
				max = points_cloud_pcl[i].z;
				top_point << points_cloud_pcl[i].x, points_cloud_pcl[i].y, points_cloud_pcl[i].z;
			}
		}
		return top_point;
	}
	Eigen::MatrixXd getMatrix(const std::vector<Eigen::Vector3d>& vector)
	{
		Eigen::MatrixXd matrix(3, vector.size());
		for (int i = 0; i < vector.size(); i++)
		{
			matrix.col(i) = vector[i];
		}
		return matrix;
	}
	Eigen::MatrixXd getMatrix(const pcl::PointCloud<pcl::PointXYZ> points)
	{
		Eigen::MatrixXd matrix(3, points.size());
		for (int i = 0; i < points.size(); i++)
		{
			matrix.col(i) << points[i].x, points[i].y, points[i].z;
		}
		return matrix;
	}
	std::vector<Eigen::Vector3d> getVVector3d(Eigen::MatrixXd input)
	{
		std::vector<Eigen::Vector3d> output;
		for (int i = 0; i < input.cols(); i++)
		{
			Eigen::Vector3d temp;
			temp << input(0, i), input(1, i), input(2, i);
			output.push_back(temp);
		}
		return output;
	}
	std::vector<Eigen::Vector3d> getVVector3d(pcl::PointCloud<pcl::PointXYZ> input)
	{
		std::vector<Eigen::Vector3d> output;
		for (int i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d temp;
			temp << input[i].x, input[i].y, input[i].z;
			output.push_back(temp);
		}
		return output;
	}
	bool getVectorByCoefficients(pcl::ModelCoefficients::Ptr coefficients, Eigen::VectorXd& output)
	{
		output.resize(coefficients->values.size());
		for (int i = 0; i < coefficients->values.size(); i++)
		{
			output[i] = coefficients->values[i];
		}
		return 1;
	}
	bool getCoefficientsByVector(Eigen::VectorXd input, pcl::ModelCoefficients::Ptr& output)
	{
		output->values.clear();
		for (int i = 0; i < input.size(); i++)
		{
			output->values.push_back(input[i]);
		}
		return 1;
	}
	cv::Mat getMat(Eigen::MatrixXd& input)
	{
		cv::Mat output(input.rows(), input.cols(), CV_64F, cv::Scalar(0));
		for (size_t i = 0; i < input.rows(); i++)
		{
			for (size_t j = 0; j < input.cols(); j++)
			{
				output.at<double>(i, j) = input(i, j);
			}
		}
		return output;
	}
	cv::Mat getMat(Eigen::VectorXd& input)
	{
		cv::Mat output(input.rows(), input.cols(), CV_64F, cv::Scalar(0));
		for (size_t i = 0; i < input.rows(); i++)
		{
			for (size_t j = 0; j < input.cols(); j++)
			{
				output.at<double>(i, j) = input(i, j);
			}
		}
		return output;
	}
	std::vector<cv::Mat> getMatFromVector(std::vector<Eigen::MatrixXd>& input)
	{
		std::vector<cv::Mat> output;
		for (size_t i = 0; i < input.size(); i++)
		{
			output.push_back(getMat(input[i]));
		}
		return output;

	}
	std::vector<cv::Mat> getMatFromVector(std::vector<Eigen::VectorXd>& input)
	{
		std::vector<cv::Mat> output;
		for (size_t i = 0; i < input.size(); i++)
		{
			output.push_back(getMat(input[i]));
		}
		return output;
	}
	bool solveSvd(const Eigen::MatrixXd& input, Eigen::MatrixXd& U, Eigen::VectorXd& S, Eigen::MatrixXd& V)
	{
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(input, Eigen::ComputeThinU | Eigen::ComputeThinV);
		U = svd.matrixU();
		S = svd.singularValues();
		V = svd.matrixV();
		return 1;
	}
	double getRandomNumber(double min, double max)
	{
		srand(time(NULL));
		double random_number = min + (max - min) * rand() / (RAND_MAX + 1.0);
		return random_number;
	}
	Eigen::Vector3d getVectorCenter(const std::vector<Eigen::Vector3d>& points)
	{
		Eigen::Vector3d center;
		center << 0, 0, 0;
		for (size_t i = 0; i < points.size(); i++)
		{
			center += points[i];
		}
		if (points.size() != 0)
		{
			center = center / points.size();
		}
		return center;
	}
	bool getCombination(int n, int m, std::vector<std::vector<int>>& all)
	{
		std::vector<int> temp;
		for (size_t i = 0; i < n; i++)
		{
			temp.push_back(i);
		}
		std::vector<int> flags(n, 0);
		for (size_t i = 0; i < m; i++)
		{
			flags[i] = 1;
		}
		std::sort(flags.begin(), flags.end());
		do
		{
			std::vector<int> temp1;
			for (size_t i = 0; i < n; i++)
			{
				if (flags[i] == 1)
				{
					temp1.push_back(temp[i]);
				}
			}
			all.push_back(temp1);
		} while (std::next_permutation(flags.begin(), flags.end()));
		return 1;
	}
	double getAngleBetweenVectors_2pi(const Eigen::Vector3d& vector1_input, const Eigen::Vector3d& vector2_bi, const Eigen::Vector3d& vector3_prin)
	{
		//角度
		double angle1 = acos(dotProduct(vector1_input, vector2_bi) / (getVectorNorm(vector1_input) * getVectorNorm(vector2_bi)));
		//旋转轴1
		Eigen::Vector3d axis1 = crossProduct(vector2_bi, vector1_input);
		axis1.normalize();
		double angle2 = dotProduct(axis1, vector3_prin);

		double angle;
		if (angle2 >= 0)
		{
			angle = angle1;
		}
		else
		{
			angle = 2 * M_PI - angle1;
		}
		return angle;
	}
	double getAngleBetweenVectors(const Eigen::VectorXd& vector1, const Eigen::VectorXd& vector2)
	{
		double angle = acos(dotProduct(vector1, vector2) / (getVectorNorm(vector1) * getVectorNorm(vector2)));
		return angle;
	}
	Eigen::VectorXd concatenateVectors(const Eigen::VectorXd& vector1, const Eigen::VectorXd& vector2)
	{
		Eigen::VectorXd concatenated_vector(vector1.size() + vector2.size());
		for (int i = 0; i < vector1.size(); i++)
		{
			concatenated_vector[i] = vector1[i];
		}
		for (int i = 0; i < vector2.size(); i++)
		{
			concatenated_vector[i + vector1.size()] = vector2[i];
		}
		return concatenated_vector;
	}
	Eigen::MatrixXd concatenateMatrices_horizon(const Eigen::MatrixXd& matrix1, const Eigen::MatrixXd& matrix2)
	{
		Eigen::MatrixXd concatenated_matrix(matrix1.rows(), matrix1.cols() + matrix2.cols());
		concatenated_matrix << matrix1, matrix2;
		return concatenated_matrix;
	}
	Eigen::MatrixXd concatenateMatrices_vertical(const Eigen::MatrixXd& matrix1, const Eigen::MatrixXd& matrix2)
	{
		Eigen::MatrixXd concatenated_matrix(matrix1.rows() + matrix2.rows(), matrix1.cols());
		concatenated_matrix << matrix1, matrix2;
		return concatenated_matrix;
	}
	Eigen::Vector3d getPlaneNormalVector(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& point3)
	{
		Eigen::Vector3d vector1 = point2 - point1;
		Eigen::Vector3d vector2 = point3 - point1;
		Eigen::Vector3d normal_vector = crossProduct(vector1, vector2);
		normal_vector.normalize();
		return normal_vector;
	}
	Eigen::Vector4d  getPlaneEquation(const Eigen::Vector3d& point, const Eigen::Vector3d& normal_vector)
	{
		Eigen::Vector4d plane_para;
		Eigen::Vector3d normal_vector_unit = getUnitVector(normal_vector);
		plane_para << normal_vector_unit[0], normal_vector_unit[1], normal_vector_unit[2], -normal_vector_unit[0] * point[0] - normal_vector_unit[1] * point[1] - normal_vector_unit[2] * point[2];
		return plane_para;
	}
	void getPlaneEquation(const Eigen::Vector3d& point, const Eigen::Vector3d& normal_vector, Eigen::Vector4d& plane_para)
	{
		Eigen::Vector3d normal_vector_unit = getUnitVector(normal_vector);
		plane_para << normal_vector_unit[0], normal_vector_unit[1], normal_vector_unit[2], -normal_vector_unit[0] * point[0] - normal_vector_unit[1] * point[1] - normal_vector_unit[2] * point[2];
	}
	Eigen::Vector4d getPlaneEquation(const std::vector<Eigen::Vector3d>& points)
	{
		Eigen::Vector4d plane_para;
		if (points.size() < 4)
		{
			return plane_para;
		}
		Eigen::Vector3d normal_vector = getPlaneNormalVector(points[0], points[1], points[2]);
		Eigen::Vector3d center;
		Eigen::MatrixXd points_matrix = getMatrix(points);
		center = points_matrix.rowwise().mean();
		points_matrix = points_matrix.colwise() - center;
		Eigen::MatrixXd nx = (points_matrix.transpose()).block(0, 0, points_matrix.cols(), 2).colPivHouseholderQr().solve(-points_matrix.transpose().block(0, 2, points_matrix.cols(), 1));
		//Eigen::MatrixXd A = (points_matrix.transpose()).block(0, 0, points_matrix.cols(), 2);
		//Eigen::MatrixXd B = (- points_matrix.transpose()).block(0, 2, points_matrix.cols(), 1);
		//Eigen::MatrixXd nx = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
		Eigen::Vector3d normal;
		normal << nx(0, 0), nx(1, 0), 1;
		normal = getUnitVector(normal);
		plane_para << normal[0], normal[1], normal[2], -normal[0] * center[0] - normal[1] * center[1] - normal[2] * center[2];
		return plane_para;
	}
	bool getPlaneEquation(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& center, Eigen::Vector3d& normal)
	{
		if (points.size() < 3)
		{
			return false;
		}
		Eigen::Vector3d normal_vector = getPlaneNormalVector(points[0], points[1], points[2]);
		Eigen::MatrixXd points_matrix = getMatrix(points);
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(points_matrix);
		if (svd.rank() < 2)
		{
			return false;
		}
		if (svd.rank() < 3)
		{
			center = points_matrix.rowwise().mean();
			Eigen::Vector3d p1, p2;
			for (size_t i = 0; i < points_matrix.cols(); i++)
			{
				p1 = points_matrix.col(0) - points_matrix.col(i);
				if (p1.norm() != 0)
				{
					break;
				}
			}
			for (size_t i = 0; i < points_matrix.cols(); i++)
			{
				p2 = points_matrix.col(0) - points_matrix.col(i);
				if (p2.norm() != 0 && p1 != p2)
				{
					break;
				}
			}
			normal = p1.cross(p2);
			normal = getUnitVector(normal);
			return 1;
		}
		center = points_matrix.rowwise().mean();
		points_matrix = points_matrix.colwise() - center;
		Eigen::MatrixXd x1 = (points_matrix.transpose()).block(0, 0, points_matrix.cols(), 2);
		Eigen::MatrixXd x2 = (points_matrix.transpose()).block(0, 2, points_matrix.cols(), 1);
		Eigen::MatrixXd x3 = x1.colPivHouseholderQr().solve(-x2);
		normal << x3(0, 0), x3(1, 0), 1;
		normal = getUnitVector(normal);
		return 1;
	}
	bool getPlaneEquation(const pcl::PointCloud<pcl::PointXYZ>& points, Eigen::Vector3d& center, Eigen::Vector3d& normal)
	{
		std::vector<Eigen::Vector3d> points_vector = getVVector3d(points);
		return getPlaneEquation(points_vector, center, normal);
	}
	Eigen::Vector4d getPlaneEquation(const pcl::PointCloud<pcl::PointXYZ>& points)
	{
		std::vector<Eigen::Vector3d> points_vector = getVVector3d(points);
		Eigen::Vector4d plane_para = getPlaneEquation(points_vector);
		return plane_para;
	}
	Eigen::Vector3d getIntersectionLine_xy(const Eigen::Vector3d& normal_vector)
	{
		Eigen::Vector3d line_pole;
		line_pole << normal_vector[1], -normal_vector[0], 0;
		line_pole = getUnitVector(line_pole);
		return line_pole;
	}
	Eigen::Vector3d getIntersectionLine_xz(const Eigen::Vector3d& normal_vector)
	{
		Eigen::Vector3d line_pole;
		line_pole << normal_vector[2], 0, -normal_vector[0];
		line_pole = getUnitVector(line_pole);
		return line_pole;
	}
	Eigen::Vector3d getIntersectionLine_yz(const Eigen::Vector3d& normal_vector)
	{
		Eigen::Vector3d line_pole;
		line_pole << 0, normal_vector[2], -normal_vector[1];
		line_pole = getUnitVector(line_pole);
		return line_pole;
	}
	void getLineEquation(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction)
	{
		line_pole = point1;
		line_direction = point2 - point1;
		line_direction = getUnitVector(line_direction);
	}
	void getLineEquation(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction)
	{
		//所有输入点的matrix
		Eigen::MatrixXd points_matrix = getMatrix(points);
		//计算中心点
		line_pole = points_matrix.rowwise().mean();
		Eigen::MatrixXd points_matrix_center = points_matrix.colwise() - line_pole;
		//初始参数
		Eigen::MatrixXd  Y1(3 * points_matrix.cols(), 1);
		Y1.setZero();
		Eigen::MatrixXd parameter(3, 1);
		parameter << 1, 1, 1;
		std::vector<double> error;
		for (size_t k = 0; k < 100; k++)
		{
			Eigen::MatrixXd J(3 * points_matrix.cols(), 3);
			J.setZero();
			for (size_t i = 0; i < points_matrix.cols(); i++)
			{
				Eigen::MatrixXd t_value = (parameter.transpose() * parameter).inverse() * parameter.transpose() * points_matrix_center.block(0, i, 3, 1);
				double t = t_value(0, 0);
				J(3 * i, 0) = t;
				J(3 * i + 1, 1) = t;
				J(3 * i + 2, 2) = t;

				Y1(3 * i, 0) = parameter(0, 0) * t;
				Y1(3 * i + 1, 0) = parameter(1, 0) * t;
				Y1(3 * i + 2, 0) = parameter(2, 0) * t;

			}
			Eigen::MatrixXd x1 = (J.transpose() * J).inverse() * J.transpose();
			Eigen::MatrixXd x2 = (points_matrix_center.reshaped(3 * points_matrix.cols(), 1) - Y1);

			Eigen::MatrixXd thetaP = (J.transpose() * J).inverse() * J.transpose() * (points_matrix_center.reshaped(3 * points_matrix.cols(), 1) - Y1);
			parameter += thetaP;
			error.push_back(x2.norm());
			if (k > 2)
			{
				if (std::abs(error[k] - error[k - 1]) < 1E-5)
					break;
			}
			//std::cout<<"Y1-Y0:"<<x2.norm()<<std::endl;
		}
		/*double xz = 0, yz = 0, xy = 0, z = 0, x = 0, y = 0,xx = 0, yy = 0, zz = 0;
		double n = points.size();
		for (size_t i = 0; i < points.size(); i++)
		{
			x += points[i][0];
			y += points[i][1];
			z += points[i][2];
			xz += points[i][0] * points[i][2];
			yz += points[i][1] * points[i][2];
			xy += points[i][0] * points[i][1];
			xx += points[i][0] * points[i][0];
			yy += points[i][1] * points[i][1];
			zz += points[i][2] * points[i][2];
		}
		double a, b, c, d;
		Eigen::Vector3d line_direction1;
		if ((n * zz - z * z) !=0)
		{
			a = (n * xz - x * z) / (n * zz - z * z);
			b = (x - a * z) / n;
			c = (n * yz - y * z) / (n * zz - z * z);
			d = (y - c * z) / n;
			line_direction1 << a, c, 1;
		}
		else if((n * yy - y * y)!=0)
		{
			a = (n * xy - x * y) / (n * yy - y * y);
			b = (x - a * y) / n;
			c = (n * yz - y * z) / (n * yy - y * y);
			d = (x - c * y) / n;
			line_direction1 << a, 1, 0;
		}
		else
		{
			line_direction1 << 1, 0, 0;
		}
		line_direction = getUnitVector(line_direction1);*/
		line_direction = getUnitVector(parameter.block(0, 0, 3, 1));
	}
	void getLineEquation(const Eigen::MatrixXd& points, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction)
	{
		std::vector<Eigen::Vector3d> points_vv = getVVector3d(points);
		getLineEquation(points_vv, line_pole, line_direction);
	}
	bool fitLine_DouglasPeucker(const std::vector<Eigen::Vector3d>& points, std::vector<Eigen::Vector3d>& output, double threshold)
	{
		//输入要按照顺序排列
		//1.首先找到最大的点
		if (points.size() == 0)
		{
			return false;
		}
		double dmax = 0;
		int index = 0;
		int end = points.size() - 1;
		Eigen::Vector3d point1 = points[0];
		Eigen::Vector3d point2 = points[end];
		Eigen::Vector3d line_pole, line_direction;
		getLineEquation(point1, point2, line_pole, line_direction);

		for (size_t i = 1; i < end; i++)
		{
			double d = getDistance_pointToLine(points[i], line_pole, line_direction);
			if (d > dmax)
			{
				index = i;
				dmax = d;
			}
		}
		//2.如果最大距离大于阈值，则递归调用
		if (dmax > threshold)
		{
			std::vector<Eigen::Vector3d> output1, output2;
			std::vector<Eigen::Vector3d> points1, points2;
			for (size_t i = 0; i < index; i++)
			{
				points1.push_back(points[i]);
			}
			for (size_t i = index; i < points.size(); i++)
			{
				points2.push_back(points[i]);
			}
			fitLine_DouglasPeucker(points1, output1, threshold);
			fitLine_DouglasPeucker(points2, output2, threshold);
			output.insert(output.end(), output1.begin(), output1.end() - 1);
			output.insert(output.end(), output2.begin(), output2.end());
		}
		else
		{
			output.push_back(points[0]);
			output.push_back(points[end]);
		}
		return true;
	}

	void getSegmentLineEquation(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, Eigen::Vector3d& line_pole, Eigen::Vector3d& line_direction, double& segment_length)
	{
		line_pole = point1;
		line_direction = point2 - point1;
		line_direction = getUnitVector(line_direction);
		segment_length = getVectorNorm(point2 - point1);
	}
	double getPointParameter(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction)
	{
		double parameter = dotProduct(point - line_pole, line_direction);
		return parameter;
	}
	double getDistance_pointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction)
	{
		double distance = getVectorNorm(crossProduct(point - line_pole, line_direction));
		return distance;
	}
	double getDistance_pointToPlane(const Eigen::Vector3d& point, const Eigen::Vector4d& plane_para)
	{
		double distance = dotProduct(point, Eigen::Vector3d(plane_para[0], plane_para[1], plane_para[2])) + plane_para[3];
		return distance;
	}
	double getDistance_pointToCircle(const Eigen::Vector3d& point, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& normal, double circle_radius)
	{
		double distance1 = getVectorNorm(point - circle_center) - circle_radius;
		Eigen::Vector4d planPara = getPlaneEquation(circle_center, normal);
		double distance2 = getDistance_pointToPlane(point, planPara);
		return distance1 + distance2;
	}
	bool getIntersectionPoint(Eigen::Vector3d line1_pole, Eigen::Vector3d line1_direction, Eigen::Vector3d line2_pole, Eigen::Vector3d line2_direction, Eigen::Vector3d& intersect_point)
	{
		double dot = line1_direction.dot(line2_direction);
		if (dot == 1)
		{
			return false;
		}
		Eigen::Matrix<double, 3, 2> coefficient_matrix;
		coefficient_matrix << line1_direction[0], -line2_direction[0],
			line1_direction[1], -line2_direction[1],
			line1_direction[2], -line2_direction[2];
		Eigen::Vector3d constant_vector;
		constant_vector << line2_pole[0] - line1_pole[0],
			line2_pole[1] - line1_pole[1],
			line2_pole[2] - line1_pole[2];

		Eigen::Vector2d solution_vector = coefficient_matrix.colPivHouseholderQr().solve(constant_vector);
		intersect_point = line1_pole + solution_vector[0] * line1_direction;
		return true;
	}

	Eigen::Vector3d getIntersectionPoint(const Eigen::Vector3d& line1_pole, const Eigen::Vector3d& line1_direction, const Eigen::Vector3d& line2_pole, const Eigen::Vector3d& line2_direction)
	{
		Eigen::Vector3d intersection_point;
		Eigen::Matrix<double, 3, 2> coefficient_matrix;
		coefficient_matrix << line1_direction[0], -line2_direction[0],
			line1_direction[1], -line2_direction[1],
			line1_direction[2], -line2_direction[2];
		Eigen::Vector3d constant_vector;
		constant_vector << line2_pole[0] - line1_pole[0],
			line2_pole[1] - line1_pole[1],
			line2_pole[2] - line1_pole[2];

		Eigen::Vector2d solution_vector = coefficient_matrix.colPivHouseholderQr().solve(constant_vector);
		intersection_point = line1_pole + solution_vector[0] * line1_direction;
		return intersection_point;
	}
	bool getIntersectionPoint_lineAndSegmentLine(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, Eigen::Vector3d& intersection_point)
	{
		intersection_point = getIntersectionPoint(line_pole, line_direction, segment_pole, segment_direction);
		double parameter = getPointParameter(intersection_point, segment_pole, segment_direction);
		if (parameter < 0 || parameter > segment_length)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	bool getIntersectionPoint_lineAndSegmentLine_nearest(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, Eigen::Vector3d& intersection_point)
	{
		intersection_point = getIntersectionPoint(line_pole, line_direction, segment_pole, segment_direction);
		double parameter = getPointParameter(intersection_point, segment_pole, segment_direction);
		if (parameter < 0)
		{
			intersection_point = segment_pole;
		}
		else if (parameter > segment_length)
		{
			intersection_point = segment_pole + segment_length * segment_direction;
		}
		return 1;
	}
	bool getIntersectionPoint_lineAndPolyline(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const std::vector<Eigen::Vector3d>& polyline_points, std::vector<Eigen::Vector3d>& intersection_points)
	{
		intersection_points.clear();
		for (size_t i = 0; i < polyline_points.size() - 1; i++)
		{
			Eigen::Vector3d segment_pole = polyline_points[i];
			Eigen::Vector3d segment_direction = polyline_points[i + 1] - polyline_points[i];
			double segment_length = getVectorNorm(segment_direction);
			segment_direction = getUnitVector(segment_direction);
			Eigen::Vector3d intersection_point;
			if (getIntersectionPoint_lineAndSegmentLine(line_pole, line_direction, segment_pole, segment_direction, segment_length, intersection_point))
			{
				intersection_points.push_back(intersection_point);
			}
		}
		return intersection_points.size();
	}
	bool getIntersectionPoint_lineAndPolyline_nearest(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const std::vector<Eigen::Vector3d>& polyline_points, std::vector<Eigen::Vector3d>& intersection_points)
	{
		intersection_points.clear();
		for (size_t i = 0; i < polyline_points.size() - 1; i++)
		{
			Eigen::Vector3d segment_pole = polyline_points[i];
			Eigen::Vector3d segment_direction = polyline_points[i + 1] - polyline_points[i];
			double segment_length = getVectorNorm(segment_direction);
			segment_direction = getUnitVector(segment_direction);
			Eigen::Vector3d intersection_point;
			if (getIntersectionPoint_lineAndSegmentLine_nearest(line_pole, line_direction, segment_pole, segment_direction, segment_length, intersection_point))
			{
				intersection_points.push_back(intersection_point);
			}
		}
		return intersection_points.size();

	}
	bool getIntersectionPoint_segmentLineAndSegmentLine(const Eigen::Vector3d& segment1_pole, const Eigen::Vector3d& segment1_direction, double segment1_length, const Eigen::Vector3d& segment2_pole, const Eigen::Vector3d& segment2_direction, double segment2_length, Eigen::Vector3d& intersection_point)
	{
		intersection_point = getIntersectionPoint(segment1_pole, segment1_direction, segment2_pole, segment2_direction);
		double parameter1 = getPointParameter(intersection_point, segment1_pole, segment1_direction);
		double parameter2 = getPointParameter(intersection_point, segment2_pole, segment2_direction);
		if (parameter1 < 0 || parameter1 > segment1_length || parameter2 < 0 || parameter2 > segment2_length)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	bool getProjectionPoint_pointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, Eigen::Vector3d& project_point)
	{
		double parameter = getPointParameter(point, line_pole, line_direction);
		project_point = line_pole + parameter * line_direction;
		return 1;
	}
	Eigen::Vector3d getProjectionPoint_pointToLine(const Eigen::Vector3d& point, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction)
	{
		Eigen::Vector3d projection_point;
		double parameter = getPointParameter(point, line_pole, line_direction);
		projection_point = line_pole + parameter * line_direction;
		return projection_point;
	}
	bool getProjectionPoint_pointToPlane(const Eigen::Vector3d& point, const Eigen::Vector4d& plane_para, Eigen::Vector3d& project_point)
	{
		Eigen::Vector3d normal_vector;
		normal_vector << plane_para[0], plane_para[1], plane_para[2];
		double distance = dotProduct(point, normal_vector) + plane_para[3];
		project_point = point - distance * normal_vector;
		return 1;

	}
	bool getIntersectionPoint_lineAndSphere(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector3d& circle_center, double circle_radius, Eigen::Vector3d& intersection_point1, Eigen::Vector3d& intersection_point2)
	{
		Eigen::Vector3d line_direction_temp = getUnitVector(line_direction);
		Eigen::Vector4d vertical_plane;
		//计算圆心在直线上的投影点
		Eigen::Vector3d projection_point = getProjectionPoint_pointToLine(circle_center, line_pole, line_direction);
		//计算圆心在直线上的投影点到圆心的距离
		double distance = getVectorNorm(circle_center - projection_point);
		if (distance > circle_radius)
		{
			return 0;
		}
		else
		{
			//计算圆心在直线上的投影点到圆心的距离与圆半径的差值
			double difference = std::pow(pow(circle_radius, 2) - pow(distance, 2), 0.5);
			intersection_point1 = projection_point + difference * line_direction_temp;
			intersection_point2 = projection_point - difference * line_direction_temp;
			return 1;
		}
	}
	int getIntersectionPoint_segmentLineAndSphere(const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, const Eigen::Vector3d& circle_center, double circle_radius, std::vector<Eigen::Vector3d>& intersection_points)
	{
		intersection_points.clear();
		Eigen::Vector3d line_direction_temp = getUnitVector(segment_direction);
		Eigen::Vector4d vertical_plane;
		//计算圆心在直线上的投影点
		Eigen::Vector3d projection_point = getProjectionPoint_pointToLine(circle_center, segment_pole, segment_direction);
		//计算圆心在直线上的投影点到圆心的距离
		double distance = getVectorNorm(circle_center - projection_point);
		if (distance > circle_radius)
		{
			return 0;
		}
		else
		{
			//计算圆心在直线上的投影点到圆心的距离与圆半径的差值
			double difference = std::pow(pow(circle_radius, 2) - pow(distance, 2), 0.5);
			Eigen::Vector3d intersection_point1 = projection_point + difference * line_direction_temp;
			Eigen::Vector3d intersection_point2 = projection_point - difference * line_direction_temp;
			//判断交点是否在线段上
			double parameter1 = getPointParameter(intersection_point1, segment_pole, segment_direction);
			double parameter2 = getPointParameter(intersection_point2, segment_pole, segment_direction);
			if (parameter1 >= 0 && parameter1 <= segment_length)
			{
				intersection_points.push_back(intersection_point1);
			}
			if (parameter2 >= 0 && parameter2 <= segment_length)
			{
				intersection_points.push_back(intersection_point2);
			}
		}

		return intersection_points.size();
	}
	std::vector<Eigen::Vector3d> getIntersectionPoint_polylineAndSphere(const std::vector<Eigen::Vector3d>& polyline_points, const Eigen::Vector3d& circle_center, double circle_radius)
	{
		std::vector<Eigen::Vector3d> intersection_points;
		if (polyline_points.size() - 1 < 1)
		{
			return intersection_points;
		}
		for (size_t i = 0; i < polyline_points.size() - 1; i++)
		{
			Eigen::Vector3d segment_pole = polyline_points[i];
			Eigen::Vector3d segment_direction = polyline_points[i + 1] - polyline_points[i];
			double segment_length = getVectorNorm(segment_direction);
			segment_direction = getUnitVector(segment_direction);
			std::vector<Eigen::Vector3d> intersection_point;
			if (getIntersectionPoint_segmentLineAndSphere(segment_pole, segment_direction, segment_length, circle_center, circle_radius, intersection_point) > 0)
			{
				intersection_points.insert(intersection_points.end(), intersection_point.begin(), intersection_point.end());
			}
		}
		return intersection_points;

	}
	bool getIntersectionPoint_lineAndPlane(const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, const Eigen::Vector4d& plane_para, Eigen::Vector3d& intersection_point)
	{
		Eigen::Vector3d normal_vector;
		normal_vector << plane_para[0], plane_para[1], plane_para[2];
		double denominator = dotProduct(line_direction, normal_vector);
		if (denominator == 0)
		{
			return 0;
		}
		else
		{
			double numerator = -plane_para[3] - dotProduct(line_pole, normal_vector);
			double parameter = numerator / denominator;
			intersection_point = line_pole + parameter * line_direction;
			return 1;
		}
	}
	bool getIntersectionPoint_segmentLineAndPlane(const Eigen::Vector3d& segment_pole, const Eigen::Vector3d& segment_direction, double segment_length, const Eigen::Vector4d& plane_para, Eigen::Vector3d& intersection_point)
	{
		Eigen::Vector3d normal_vector;
		normal_vector << plane_para[0], plane_para[1], plane_para[2];
		double denominator = dotProduct(segment_direction, normal_vector);
		if (denominator == 0)//法线相互垂直，线与面平行，无交点
		{
			return 0;
		}
		else
		{
			double numerator = -plane_para[3] - dotProduct(segment_pole, normal_vector);
			double parameter = numerator / denominator;
			intersection_point = segment_pole + parameter * segment_direction;
			double parameter1 = getPointParameter(intersection_point, segment_pole, segment_direction);
			if (parameter1 < 0 || parameter1 > segment_length)
			{
				return 0;
			}
			else
			{
				return 1;
			}
		}
	}
	double getAngle_lineAndPlane(const Eigen::Vector3d& line_direction, const Eigen::Vector4d& plane_para)
	{
		Eigen::Vector3d normal_vector;
		normal_vector << plane_para[0], plane_para[1], plane_para[2];
		double angle = getAngleBetweenVectors(line_direction, normal_vector);
		return angle;
	}
	bool getIntersectionPoint_planeAndPolyline(const Eigen::Vector4d& plane_para, const std::vector<Eigen::Vector3d>& polyline_points, std::vector<Eigen::Vector3d>& intersection_point)
	{
		intersection_point.clear();
		Eigen::Vector3d normal_vector;
		normal_vector << plane_para[0], plane_para[1], plane_para[2];
		if (polyline_points.size() < 2)
		{
			return false;
		}
		for (size_t i = 0; i < polyline_points.size() - 1; i++)
		{
			Eigen::Vector3d segment_pole = polyline_points[i];
			Eigen::Vector3d segment_direction = polyline_points[i + 1] - polyline_points[i];
			double segment_length = getVectorNorm(segment_direction);
			segment_direction = getUnitVector(segment_direction);
			Eigen::Vector3d intersection_point_temp;
			if (getIntersectionPoint_segmentLineAndPlane(segment_pole, segment_direction, segment_length, plane_para, intersection_point_temp))
			{
				intersection_point.push_back(intersection_point_temp);
			}
		}
		return intersection_point.size();

	}
	bool getIntersectionPoint_planeAndPlane_xy(const Eigen::Vector4d& plane1_para, const Eigen::Vector4d& plane2_para, Eigen::Vector3d& intersection_point)
	{
		Eigen::Matrix2d coefficient_matrix;
		coefficient_matrix << plane1_para[0], plane1_para[1],
			plane2_para[0], plane2_para[1];
		Eigen::Vector2d constant_vector;
		constant_vector << -plane1_para[3], -plane2_para[3];
		Eigen::Vector2d solution_vector = coefficient_matrix.colPivHouseholderQr().solve(constant_vector);
		intersection_point << solution_vector[0], solution_vector[1], 0;
		return 1;
	}
	bool getIntersectionLine_planeAndPlane(const Eigen::Vector4d& plane1_para, const Eigen::Vector4d& plane2_para, Eigen::Vector3d& intersection_line_pole, Eigen::Vector3d& intersection_line_direction)
	{
		Eigen::Vector3d normal_vector1;
		normal_vector1 << plane1_para[0], plane1_para[1], plane1_para[2];
		Eigen::Vector3d normal_vector2;
		normal_vector2 << plane2_para[0], plane2_para[1], plane2_para[2];
		Eigen::Vector3d intersection_line_direction_temp = crossProduct(normal_vector1, normal_vector2);
		if (getVectorNorm(intersection_line_direction_temp) == 0)
		{
			return 0;
		}
		else
		{
			intersection_line_direction = getUnitVector(intersection_line_direction_temp);
			Eigen::Vector3d intersection_point;
			getIntersectionPoint_planeAndPlane_xy(plane1_para, plane2_para, intersection_point);
			intersection_line_pole = intersection_point;
			return 1;
		}
	}
	double getAngle_planes(const Eigen::Vector4d& plane1_para, const Eigen::Vector4d& plane2_para)
	{
		Eigen::Vector3d normal_vector1;
		normal_vector1 << plane1_para[0], plane1_para[1], plane1_para[2];
		Eigen::Vector3d normal_vector2;
		normal_vector2 << plane2_para[0], plane2_para[1], plane2_para[2];
		double angle = getAngleBetweenVectors(normal_vector1, normal_vector2);
		return angle;
	}
	bool getCircleEquation(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const Eigen::Vector3d& point3, Eigen::Vector3d& circle_center, Eigen::Vector3d& circle_normal, double& circle_radius)
	{
		std::vector<Eigen::Vector3d> points;
		points.push_back(point1);
		points.push_back(point2);
		points.push_back(point3);
		return getCircleEquation(points, circle_center, circle_normal, circle_radius);
	}

	bool getCircleEquation(const std::vector<Eigen::Vector3d>& points, Eigen::Vector3d& circle_center, Eigen::Vector3d& circle_normal, double& circle_radius)
	{
		//输入为 至少三个点
		//首先先将点云投影到平面上
		Eigen::MatrixXd Y0 = getMatrix(points);
		getPlaneEquation(points, circle_center, circle_normal);
		Eigen::Vector3d circle_normal_unit = crossProduct(circle_normal, Eigen::Vector3d(0, 0, 1));
		circle_normal_unit.normalize();
		Eigen::MatrixXd rotate_matrix = getRotationMatrix(acos(dotProduct(circle_normal, Eigen::Vector3d(0, 0, 1))), circle_normal_unit);
		//Eigen::MatrixXd Y0_ = (rotate_matrix * Y0).colwise() - circle_center;//旋转到xy平面上，应该是先平移后旋转
		Eigen::MatrixXd Y0_ = rotate_matrix * (Y0.colwise() - circle_center);//旋转到xy平面上，应该是先平移后旋转

		//Ax2+Ay2+Bx+Cy+D=0
		Eigen::MatrixXd J(points.size(), 3);
		Eigen::MatrixXd Y1(points.size(), 1);
		for (size_t k = 0; k < points.size(); k++)
		{
			J(k, 0) = Y0_(0, k) * Y0_(0, k) + Y0_(1, k) * Y0_(1, k);
			J(k, 1) = Y0_(0, k);
			J(k, 2) = Y0_(1, k);
			Y1(k, 0) = -1;
		}
		Eigen::MatrixXd Para = J.colPivHouseholderQr().solve(Y1);
		double A = Para(0, 0);
		double B = Para(1, 0);
		double C = Para(2, 0);
		circle_radius = pow(pow(B / A / 2.0, 2) + pow(C / A / 2.0, 2) - 1 / A, 0.5);
		Eigen::Vector3d center_xy;
		center_xy << -B / A / 2.0, -C / A / 2.0, 0;
		circle_center = rotate_matrix.inverse() * center_xy + circle_center;//旋转到原来的坐标系
		return 1;
	}
	bool getCircleEquation(const pcl::PointCloud<pcl::PointXYZ>& points, Eigen::Vector3d& circle_center, Eigen::Vector3d& circle_normal, double& circle_radius)
	{
		std::vector<Eigen::Vector3d> points_vector = getVVector3d(points);
		return getCircleEquation(points_vector, circle_center, circle_normal, circle_radius);
	}
	double getAngle_pointOnCircle(const Eigen::Vector3d& point, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& circle_princicalnormal, const Eigen::Vector3d& circle_binormal, double circle_radius)
	{
		Eigen::Vector3d point_vector = point - circle_center;
		double angle = getAngleBetweenVectors_2pi(point_vector, circle_binormal, circle_princicalnormal);
		return angle;
	}


	std::vector<Eigen::Vector3d> getIntersectionPoint_planeAndCircle(const Eigen::Vector4d& plane_para, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& principlenormal, double circle_radius)
	{
		Eigen::Vector3d intersection_point1, intersection_point2;
		std::vector<Eigen::Vector3d> intersection_points;
		//圆所在的平面的参数
		Eigen::Vector4d circle_plane_para;
		getPlaneEquation(circle_center, principlenormal, circle_plane_para);
		//圆所在平面与给定平面的交线
		Eigen::Vector3d intersection_line_pole, intersection_line_direction;
		getIntersectionLine_planeAndPlane(plane_para, circle_plane_para, intersection_line_pole, intersection_line_direction);
		//圆所在平面与给定平面的交线与圆的交点
		if (getIntersectionPoint_lineAndSphere(intersection_line_pole, intersection_line_direction, circle_center, circle_radius, intersection_point1, intersection_point2))
		{
			intersection_points.push_back(intersection_point1);
			intersection_points.push_back(intersection_point2);
		}
		return intersection_points;
	}
	bool getIntersectionPoint_planeAndCircle(const Eigen::Vector4d& plane_para, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& principlenormal, double circle_radius, Eigen::Vector3d& intersection_point1, Eigen::Vector3d& intersection_point2)
	{
		//圆所在的平面的参数
		Eigen::Vector4d circle_plane_para;
		getPlaneEquation(circle_center, principlenormal, circle_plane_para);
		//圆所在平面与给定平面的交线
		Eigen::Vector3d intersection_line_pole, intersection_line_direction;
		getIntersectionLine_planeAndPlane(plane_para, circle_plane_para, intersection_line_pole, intersection_line_direction);
		//圆所在平面与给定平面的交线与圆的交点
		return getIntersectionPoint_lineAndSphere(intersection_line_pole, intersection_line_direction, circle_center, circle_radius, intersection_point1, intersection_point2);
	}
	std::vector<Eigen::Vector3d> getIntersectionPoint_planeAndArc(const Eigen::Vector4d& plane_para, const Eigen::Vector3d& circle_center, const Eigen::Vector3d& principlenormal, const Eigen::Vector3d& binormal, double circle_radius, double arc_start_angle, double arc_end_angle, bool isWiseClock)
	{
		//圆所在的平面的参数
		Eigen::Vector4d circle_plane_para;
		getPlaneEquation(circle_center, principlenormal, circle_plane_para);
		//圆所在平面与给定平面的交线
		Eigen::Vector3d intersection_line_pole, intersection_line_direction;
		getIntersectionLine_planeAndPlane(plane_para, circle_plane_para, intersection_line_pole, intersection_line_direction);
		//圆所在平面与给定平面的交线与圆的交点
		Eigen::Vector3d intersection_point1, intersection_point2;
		std::vector<Eigen::Vector3d> intersection_points;
		bool isIntersectWithCircle = getIntersectionPoint_lineAndSphere(intersection_line_pole, intersection_line_direction, circle_center, circle_radius, intersection_point1, intersection_point2);
		if (isIntersectWithCircle)//是否和圆相交
		{
			//圆平面的副法线
			//圆所在平面与给定平面的交线与圆的交点与圆心的夹角
			if (isWiseClock == 1)
			{
				//double angle1 = getAngleBetweenVectors_2pi(intersection_point1 - circle_center, binormal, principlenormal);
				//double angle2 = getAngleBetweenVectors_2pi(intersection_point2 - circle_center, binormal, principlenormal);
				//向量
				Eigen::Vector3d point_start_angle = getRotationMatrix(arc_start_angle - 1E-6, principlenormal) * binormal;
				Eigen::Vector3d point_end_angle = getRotationMatrix(arc_end_angle + 1E-6, principlenormal) * binormal;

				Eigen::Vector3d intersection_point1_temp = intersection_point1 - circle_center;
				Eigen::Vector3d intersection_point2_temp = intersection_point2 - circle_center;

				//圆所在平面与给定平面的交线与圆的交点与圆心的夹角与圆的起始角度和终止角度的关系
				double a = dotProduct(crossProduct(point_start_angle, intersection_point1_temp), principlenormal);
				double b = dotProduct(crossProduct(intersection_point1_temp, point_end_angle), principlenormal);
				if (a >= 0 &&
					b >= 0)
				{
					intersection_points.push_back(intersection_point1);
				}
				if (dotProduct(crossProduct(point_start_angle, intersection_point2_temp), principlenormal) >= 0 &&
					dotProduct(crossProduct(intersection_point2_temp, point_end_angle), principlenormal) >= 0 &&
					(intersection_point1 - intersection_point2).norm() > 1E-6)
				{
					intersection_points.push_back(intersection_point2);
				}

			}
			else
			{
				Eigen::Vector3d point_start_angle = getRotationMatrix(arc_start_angle, principlenormal) * binormal;
				Eigen::Vector3d point_end_angle = getRotationMatrix(arc_end_angle, principlenormal) * binormal;

				Eigen::Vector3d intersection_point1_temp = intersection_point1 - circle_center;
				Eigen::Vector3d intersection_point2_temp = intersection_point2 - circle_center;

				//圆所在平面与给定平面的交线与圆的交点与圆心的夹角与圆的起始角度和终止角度的关系
				if (dotProduct(crossProduct(point_start_angle, intersection_point1_temp), principlenormal) <= 0 &&
					dotProduct(crossProduct(intersection_point1_temp, point_end_angle), principlenormal) <= 0)
				{
					intersection_points.push_back(intersection_point1);
				}
				if (dotProduct(crossProduct(point_start_angle, intersection_point2_temp), principlenormal) <= 0 &&
					dotProduct(crossProduct(intersection_point2_temp, point_end_angle), principlenormal) <= 0 &&
					(intersection_point1 - intersection_point2).norm() > 1E-6)
				{
					intersection_points.push_back(intersection_point2);
				}
			}
		}
		return intersection_points;
	}

	bool fitCircleRansac(Eigen::MatrixXd& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate, double threshold, int numberOfSelectPoints)
	{
		//用RANSAC拟合圆
		std::vector<Eigen::Vector3d> points = getVVector3d(input);
		if (points.size() < numberOfSelectPoints)
		{
			return false;
		}
		std::vector<Eigen::Vector3d> circle_points;
		std::vector<Eigen::Vector3d> circle_points_temp;
		double circle_radius = 0;
		double circle_residual = 1E10;
		Eigen::Vector3d circle_center;
		Eigen::Vector3d circle_normal;
		double circle_radius_temp;
		Eigen::Vector3d circle_center_temp;
		Eigen::Vector3d circle_normal_temp;
		double circle_radius_max = 0;
		Eigen::Vector3d circle_center_max;
		Eigen::Vector3d circle_normal_max;
		circle_points.clear();
		for (size_t i = 0; i < maxIterate; i++)
		{
			circle_points_temp.clear();
			std::vector<Eigen::Vector3d> pointsSet;
			for (size_t i = 0; i < numberOfSelectPoints; i++)
			{
				pointsSet.push_back(points[rand() % points.size()]);
			}
			/*int index1 = rand() % points.size();
			int index2 = rand() % points.size();
			int index3 = rand() % points.size();
			Eigen::Vector3d point1 = points[index1];
			Eigen::Vector3d point2 = points[index2];
			Eigen::Vector3d point3 = points[index3];*/
			Eigen::Vector3d center_temp;
			Eigen::Vector3d normal_temp;
			double radius_temp;
			std::vector<double> distanceSet;
			if (getCircleEquation(pointsSet, center_temp, normal_temp, radius_temp))
			{
				for (size_t j = 0; j < points.size(); j++)
				{
					double distance = getDistance_pointToCircle(points[j], center_temp, normal_temp, radius_temp);
					if (abs(distance) < threshold)
					{
						distanceSet.push_back(distance);
						circle_points_temp.push_back(points[j]);
					}
				}
				//pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
				//cloud1 = getPCLPointXYZSet(points);
				//cloud2 = getPCLPointXYZSet(circle_points_temp);
				//showPointCloud(cloud1, cloud2, "cloud1");
				double sum_temp = varianceVector(distanceSet);
				if (circle_points_temp.size() > circle_points.size())
				{
					circle_points = circle_points_temp;
					circle_center = center_temp;
					circle_normal = normal_temp;
					circle_radius = radius_temp;
					circle_residual = sum_temp;
				}
				if (circle_points_temp.size() == circle_points.size())
				{
					if (sum_temp < circle_residual)
					{
						circle_points = circle_points_temp;
						circle_center = center_temp;
						circle_normal = normal_temp;
						circle_radius = radius_temp;
						circle_residual = sum_temp;
					}
				}
			}
		}
		getCircleEquation(circle_points, circle_center, circle_normal, circle_radius);
		center = circle_center;
		normal = circle_normal;
		radius = circle_radius;
		return 1;
	}
	bool fitCircleRansac(std::vector<Eigen::Vector3d>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate, double threshold, int numberOfSelectPoints)
	{
		Eigen::MatrixXd input_matrix = getMatrix(input);
		return fitCircleRansac(input_matrix, center, normal, radius, maxIterate, threshold, numberOfSelectPoints);
	}
	bool fitCircleRansac(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate, double threshold, int numberOfSelectPoints)
	{
		Eigen::MatrixXd input_matrix = getMatrix(input);
		return fitCircleRansac(input_matrix, center, normal, radius, maxIterate, threshold, numberOfSelectPoints);

	}
	bool fitCircleRansacFunctional(Eigen::MatrixXd& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate, double threshold, int numberOfSelectPoints)
	{
		//用RANSAC拟合圆
		std::vector<Eigen::Vector3d> points = getVVector3d(input);
		if (points.size() < numberOfSelectPoints)
		{
			return false;
		}
		std::vector<Eigen::Vector3d> circle_points;
		std::vector<Eigen::Vector3d> circle_points_temp;
		double circle_radius = 0;
		double circle_residual = 1E10;
		Eigen::Vector3d circle_center;
		Eigen::Vector3d circle_normal;
		double circle_radius_temp;
		Eigen::Vector3d circle_center_temp;
		Eigen::Vector3d circle_normal_temp;
		double circle_radius_max = 0;
		Eigen::Vector3d circle_center_max;
		Eigen::Vector3d circle_normal_max;
		circle_points.clear();
		cv::Mat data(maxIterate, 5, CV_64F, cv::Scalar(0));
		for (size_t i = 0; i < maxIterate; i++)
		{
			circle_points_temp.clear();
			std::vector<Eigen::Vector3d> pointsSet;
			for (size_t i = 0; i < numberOfSelectPoints; i++)
			{
				pointsSet.push_back(points[rand() % points.size()]);
			}
			/*int index1 = rand() % points.size();
			int index2 = rand() % points.size();
			int index3 = rand() % points.size();
			Eigen::Vector3d point1 = points[index1];
			Eigen::Vector3d point2 = points[index2];
			Eigen::Vector3d point3 = points[index3];*/
			Eigen::Vector3d center_temp;
			Eigen::Vector3d normal_temp;
			double radius_temp;
			std::vector<double> distanceSet;
			if (getCircleEquation(pointsSet, center_temp, normal_temp, radius_temp))
			{
				for (size_t j = 0; j < points.size(); j++)
				{
					double distance = getDistance_pointToCircle(points[j], center_temp, normal_temp, radius_temp);
					double rEstimate = distance + radius_temp;
					//distance = (pow(rEstimate, 2) - pow(radius_temp, 2)) / radius_temp;
					if (abs(distance) < threshold)
					{
						distanceSet.push_back(distance);
						circle_points_temp.push_back(points[j]);
					}
				}
				//pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
				//cloud1 = getPCLPointXYZSet(points);
				//cloud2 = getPCLPointXYZSet(circle_points_temp);
				//showPointCloud(cloud1, cloud2, "cloud1");
				double sum_temp = varianceVector(distanceSet);
				data.at<double>(i, 0) = i;
				data.at<double>(i, 1) = radius_temp;
				data.at<double>(i, 2) = sum_temp;
				data.at<double>(i, 3) = circle_points_temp.size();
				data.at<double>(i, 4) = 0;

				if (circle_points_temp.size() > 0.5 * points.size())//circle_points.size())
				{
					//circle_points = circle_points_temp;
					//circle_center = center_temp;
					//circle_normal = normal_temp;
					//circle_radius = radius_temp;
					//circle_residual = sum_temp;
					//if (circle_points_temp.size() == circle_points.size())
					{
						if (sum_temp < circle_residual)
						{
							circle_points = circle_points_temp;
							circle_center = center_temp;
							circle_normal = normal_temp;
							circle_radius = radius_temp;
							circle_residual = sum_temp;
						}
					}
				}
			}
		}
		getCircleEquation(circle_points, circle_center, circle_normal, circle_radius);
		center = circle_center;
		normal = circle_normal;
		radius = circle_radius;
		return 1;
	}
	bool fitCircleRansacFunctional(std::vector<Eigen::Vector3d>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate, double threshold, int numberOfSelectPoints)
	{
		Eigen::MatrixXd input_matrix = getMatrix(input);
		return fitCircleRansacFunctional(input_matrix, center, normal, radius, maxIterate, threshold, numberOfSelectPoints);
	}
	bool fitCircleRansacFunctional(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector3d& center, Eigen::Vector3d& normal, double& radius, int maxIterate, double threshold, int numberOfSelectPoints)
	{
		Eigen::MatrixXd input_matrix = getMatrix(input);
		return fitCircleRansacFunctional(input_matrix, center, normal, radius, maxIterate, threshold, numberOfSelectPoints);
	}
	std::vector<Eigen::MatrixXd> circleProcessDerivative(Eigen::Vector<double, 7>& coeifficient_temp, Eigen::MatrixXd& input, Eigen::MatrixXd& output)
	{
		double x0original = coeifficient_temp[0];
		double y0original = coeifficient_temp[1];
		double z0original = coeifficient_temp[2];
		double aoriginal = coeifficient_temp[3];
		double boriginal = coeifficient_temp[4];
		double coriginal = coeifficient_temp[5];
		double r = coeifficient_temp[6];
		double norm = pow(aoriginal, 2) + pow(boriginal, 2) + pow(coriginal, 2);
		double a = aoriginal / pow(norm, 0.5);
		double b = boriginal / pow(norm, 0.5);
		double c = coriginal / pow(norm, 0.5);
		double distance = (a * x0original + b * y0original + c * z0original) / (a + b + c);
		double x0 = x0original;// -distance;
		double y0 = y0original;// -distance;
		double z0 = z0original;// -distance;

		double A0 = 1 - a * a;
		double B0 = 1 - b * b;
		double C0 = 1 - c * c;
		double D0 = -2 * a * b;
		double E0 = -2 * a * c;
		double F0 = -2 * b * c;
		double G0 = -2 * A0 * x0 - D0 * y0 - E0 * z0;
		double H0 = -2 * B0 * y0 - D0 * x0 - F0 * z0;
		double I0 = -2 * C0 * z0 - E0 * x0 - F0 * y0;
		double P = A0 * x0 * x0 + B0 * y0 * y0 + C0 * z0 * z0 + D0 * x0 * y0 + E0 * x0 * z0 + F0 * y0 * z0;
		double J = P - r * r;

		double A = A0;
		double B = B0;
		double C = C0;
		double D = D0;
		double E = E0;
		double F = F0;
		double G = G0;
		double H = H0;
		double I = I0;

		std::vector<Eigen::MatrixXd> J_set;
		Eigen::MatrixXd JP1(input.cols(), 10);
		Eigen::MatrixXd Y(input.cols(), 1);
		JP1.setZero();
		Y.setZero();
		for (size_t i = 0; i < input.cols(); i++)
		{
			Eigen::Vector3d point = input.col(i);
			double x = point[0];
			double y = point[1];
			double z = point[2];

			JP1(i, 0) = x * x;
			JP1(i, 1) = y * y;
			JP1(i, 2) = z * z;
			JP1(i, 3) = x * y;
			JP1(i, 4) = x * z;
			JP1(i, 5) = y * z;
			JP1(i, 6) = x;
			JP1(i, 7) = y;
			JP1(i, 8) = z;
			JP1(i, 9) = 1;
			double tempt1 = pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2) - pow(a * (x - x0) + b * (y - y0) + c * (z - z0), 2) - pow(r, 2);
			double tempt2 = A * x * x + B * y * y + C * z * z + D * x * y + E * x * z + F * y * z + G * x + H * y + I * z + J;


			Y(i, 0) = A * x * x + B * y * y + C * z * z + D * x * y + E * x * z + F * y * z + G * x + H * y + I * z + J;
		}


		Eigen::MatrixXd JP2(10, 7);
		JP2.setZero();
		double dja = -2 * a * x0 * x0 - 2 * b * x0 * y0 - 2 * c * x0 * z0;
		double djb = -2 * b * y0 * y0 - 2 * a * x0 * y0 - 2 * c * y0 * z0;
		double djc = -2 * c * z0 * z0 - 2 * a * x0 * z0 - 2 * b * y0 * z0;
		double djx0 = 2 * A0 * x0 - 2 * a * b * y0 - 2 * a * c * z0;
		double djy0 = 2 * B0 * y0 - 2 * a * b * x0 - 2 * b * c * z0;
		double djz0 = 2 * C0 * z0 - 2 * a * c * x0 - 2 * b * c * y0;
		double djr = -2 * r;
		JP2(0, 0) = 0;
		JP2(0, 1) = 0;
		JP2(0, 2) = 0;
		JP2(0, 3) = -2 * a;
		JP2(0, 4) = 0;
		JP2(0, 5) = 0;
		JP2(0, 6) = 0;

		JP2(1, 0) = 0;
		JP2(1, 1) = 0;
		JP2(1, 2) = 0;
		JP2(1, 3) = 0;
		JP2(1, 4) = -2 * b;
		JP2(1, 5) = 0;
		JP2(1, 6) = 0;

		JP2(2, 0) = 0;
		JP2(2, 1) = 0;
		JP2(2, 2) = 0;
		JP2(2, 3) = 0;
		JP2(2, 4) = 0;
		JP2(2, 5) = -2 * c;
		JP2(2, 6) = 0;

		JP2(3, 0) = 0;
		JP2(3, 1) = 0;
		JP2(3, 2) = 0;
		JP2(3, 3) = -2 * b;
		JP2(3, 4) = -2 * a;
		JP2(3, 5) = 0;
		JP2(3, 6) = 0;

		JP2(4, 0) = 0;
		JP2(4, 1) = 0;
		JP2(4, 2) = 0;
		JP2(4, 3) = -2 * c;
		JP2(4, 4) = 0;
		JP2(4, 5) = -2 * a;
		JP2(4, 6) = 0;

		JP2(5, 0) = 0;
		JP2(5, 1) = 0;
		JP2(5, 2) = 0;
		JP2(5, 3) = 0;
		JP2(5, 4) = -2 * c;
		JP2(5, 5) = -2 * b;
		JP2(5, 6) = 0;

		JP2(6, 0) = (-2 * A0);
		JP2(6, 1) = (2 * a * b);
		JP2(6, 2) = (2 * a * c);
		JP2(6, 3) = (4 * a * x0 + 2 * b * y0 + 2 * c * z0);
		JP2(6, 4) = (2 * a * y0);
		JP2(6, 5) = (2 * a * z0);
		JP2(6, 6) = 0;

		JP2(7, 0) = (2 * a * b);
		JP2(7, 1) = (-2 * B0);
		JP2(7, 2) = (2 * b * c);
		JP2(7, 3) = (2 * b * x0);
		JP2(7, 4) = (4 * b * y0 + 2 * a * x0 + 2 * c * z0);
		JP2(7, 5) = (2 * b * z0);
		JP2(7, 6) = 0;

		JP2(8, 0) = (2 * a * c);
		JP2(8, 1) = (2 * b * c);
		JP2(8, 2) = (-2 * C0);
		JP2(8, 3) = (2 * c * x0);
		JP2(8, 4) = (2 * c * y0);
		JP2(8, 5) = (4 * c * z0 + 2 * a * x0 + 2 * b * y0);
		JP2(8, 6) = 0;

		JP2(9, 0) = djx0;
		JP2(9, 1) = djy0;
		JP2(9, 2) = djz0;
		JP2(9, 3) = dja;
		JP2(9, 4) = djb;
		JP2(9, 5) = djc;
		JP2(9, 6) = djr;
		Eigen::MatrixXd JPUP = JP1 * JP2;

		Eigen::Matrix<double, 7, 7> JPDOWN;//对参数进行限制。就如同激活函数一样。
		JPDOWN.setZero();
		JPDOWN(0, 0) = 0;// (b + c) / (a + b + c);
		JPDOWN(0, 1) = 0;// (-b) / (a + b + c);
		JPDOWN(0, 2) = 0;// (-c) / (a + b + c);
		JPDOWN(0, 3) = 0;// (b* x0original - b * y0original + c * x0original - c * z0original) * -1 * pow(a + b + c, -2);
		JPDOWN(0, 4) = 0;// (b* x0original - b * y0original + c * x0original - c * z0original) * -1 * pow(a + b + c, -2) + (x0original - y0original) * pow(a + b + c, -1);
		JPDOWN(0, 5) = 0;// (b* x0original - b * y0original + c * x0original - c * z0original) * -1 * pow(a + b + c, -2) + (x0original - z0original) * pow(a + b + c, -1);

		JPDOWN(1, 0) = 0;// (-a) / (a + b + c);
		JPDOWN(1, 1) = 1;// (a + c) / (a + b + c);
		JPDOWN(1, 2) = 0;// (-c) / (a + b + c);
		JPDOWN(1, 3) = 0;// (a* y0original - a * x0original + c * y0original - c * z0original) * -1 * pow(a + b + c, -2) + (y0original - x0original) * pow(a + b + c, -1);
		JPDOWN(1, 4) = 0;// (a* y0original - a * x0original + c * y0original - c * z0original) * -1 * pow(a + b + c, -2);
		JPDOWN(1, 5) = 0;// (a* y0original - a * x0original + c * y0original - c * z0original) * -1 * pow(a + b + c, -2) + (y0original - z0original) * pow(a + b + c, -1);

		JPDOWN(2, 0) = 0;// (-a) / (a + b + c);
		JPDOWN(2, 1) = 0;// (-b) / (a + b + c);
		JPDOWN(2, 2) = 1;// (a + b) / (a + b + c);
		JPDOWN(2, 3) = 0;// (a* z0original - a * x0original + b * z0original - b * y0original) * -1 * pow(a + b + c, -2) + (z0original - x0original) * pow(a + b + c, -1);
		JPDOWN(2, 4) = 0;// (a* z0original - a * x0original + b * z0original - b * y0original) * -1 * pow(a + b + c, -2) + (z0original - y0original) * pow(a + b + c, -1);
		JPDOWN(2, 5) = 0;// (a* z0original - a * x0original + b * z0original - b * y0original) * -1 * pow(a + b + c, -2);

		JPDOWN(3, 3) = 0;// pow(norm, -0.5) - aoriginal * aoriginal * pow(norm, -1.5);
		JPDOWN(3, 4) = -aoriginal * boriginal * pow(norm, -1.5);
		JPDOWN(3, 5) = -aoriginal * coriginal * pow(norm, -1.5);
		JPDOWN(4, 3) = 0;// -aoriginal * boriginal * pow(norm, -1.5);
		JPDOWN(4, 4) = pow(norm, -0.5) - boriginal * boriginal * pow(norm, -1.5);
		JPDOWN(4, 5) = -boriginal * coriginal * pow(norm, -1.5);
		JPDOWN(5, 3) = 0;// -aoriginal * coriginal * pow(norm, -1.5);
		JPDOWN(5, 4) = -boriginal * coriginal * pow(norm, -1.5);
		JPDOWN(5, 5) = pow(norm, -0.5) - coriginal * coriginal * pow(norm, -1.5);
		JPDOWN(6, 6) = 1;
		JPUP = JPUP * JPDOWN;
		cv::Mat JP2_mat = Eigen2Mat(JP2);
		cv::Mat JPDOWN_mat = Eigen2Mat(static_cast<Eigen::MatrixXd>(JPDOWN));

		output = Y;
		J_set.push_back(JPUP);//JX
		return J_set;

	}
	std::vector<Eigen::MatrixXd> circleProcessDerivative(Eigen::Vector<double, 7>& coeifficient_temp, Eigen::MatrixXd& input, Eigen::MatrixXd& output, double minRadius, double maxRadius)
	{
		double x0 = coeifficient_temp[0];
		double y0 = coeifficient_temp[1];
		double z0 = coeifficient_temp[2];
		double a = coeifficient_temp[3];
		double b = coeifficient_temp[4];
		double c = coeifficient_temp[5];
		double roriginal = coeifficient_temp[6];
		double r;
		if (roriginal < minRadius)
		{
			r = minRadius;
		}
		else if (roriginal > maxRadius)
		{
			r = maxRadius;
		}
		else
		{
			r = roriginal;
		}

		double A0 = 1 - a * a;
		double B0 = 1 - b * b;
		double C0 = 1 - c * c;
		double D0 = -2 * a * b;
		double E0 = -2 * a * c;
		double F0 = -2 * b * c;
		double G0 = -2 * A0 * x0 - D0 * y0 - E0 * z0;
		double H0 = -2 * B0 * y0 - D0 * x0 - F0 * z0;
		double I0 = -2 * C0 * z0 - E0 * x0 - F0 * y0;
		double P = A0 * x0 * x0 + B0 * y0 * y0 + C0 * z0 * z0 + D0 * x0 * y0 + E0 * x0 * z0 + F0 * y0 * z0;
		double J = P - r * r;

		double A = A0;
		double B = B0;
		double C = C0;
		double D = D0;
		double E = E0;
		double F = F0;
		double G = G0;
		double H = H0;
		double I = I0;

		std::vector<Eigen::MatrixXd> J_set;
		Eigen::MatrixXd JP1(input.cols(), 10);
		Eigen::MatrixXd Y(input.cols(), 1);
		JP1.setZero();
		Y.setZero();
		for (size_t i = 0; i < input.cols(); i++)
		{
			Eigen::Vector3d point = input.col(i);
			double x = point[0];
			double y = point[1];
			double z = point[2];

			JP1(i, 0) = x * x;
			JP1(i, 1) = y * y;
			JP1(i, 2) = z * z;
			JP1(i, 3) = x * y;
			JP1(i, 4) = x * z;
			JP1(i, 5) = y * z;
			JP1(i, 6) = x;
			JP1(i, 7) = y;
			JP1(i, 8) = z;
			JP1(i, 9) = 1;
			double tempt1 = pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2) - pow(a * (x - x0) + b * (y - y0) + c * (z - z0), 2) - pow(r, 2);
			double tempt2 = A * x * x + B * y * y + C * z * z + D * x * y + E * x * z + F * y * z + G * x + H * y + I * z + J;


			Y(i, 0) = A * x * x + B * y * y + C * z * z + D * x * y + E * x * z + F * y * z + G * x + H * y + I * z + J;
		}


		Eigen::MatrixXd JP2(10, 7);
		JP2.setZero();
		double dja = -2 * a * x0 * x0 - 2 * b * x0 * y0 - 2 * c * x0 * z0;
		double djb = -2 * b * y0 * y0 - 2 * a * x0 * y0 - 2 * c * y0 * z0;
		double djc = -2 * c * z0 * z0 - 2 * a * x0 * z0 - 2 * b * y0 * z0;
		double djx0 = 2 * A0 * x0 - 2 * a * b * y0 - 2 * a * c * z0;
		double djy0 = 2 * B0 * y0 - 2 * a * b * x0 - 2 * b * c * z0;
		double djz0 = 2 * C0 * z0 - 2 * a * c * x0 - 2 * b * c * y0;
		double djr = -2 * r;
		JP2(0, 0) = 0;
		JP2(0, 1) = 0;
		JP2(0, 2) = 0;
		JP2(0, 3) = -2 * a;
		JP2(0, 4) = 0;
		JP2(0, 5) = 0;
		JP2(0, 6) = 0;

		JP2(1, 0) = 0;
		JP2(1, 1) = 0;
		JP2(1, 2) = 0;
		JP2(1, 3) = 0;
		JP2(1, 4) = -2 * b;
		JP2(1, 5) = 0;
		JP2(1, 6) = 0;

		JP2(2, 0) = 0;
		JP2(2, 1) = 0;
		JP2(2, 2) = 0;
		JP2(2, 3) = 0;
		JP2(2, 4) = 0;
		JP2(2, 5) = -2 * c;
		JP2(2, 6) = 0;

		JP2(3, 0) = 0;
		JP2(3, 1) = 0;
		JP2(3, 2) = 0;
		JP2(3, 3) = -2 * b;
		JP2(3, 4) = -2 * a;
		JP2(3, 5) = 0;
		JP2(3, 6) = 0;

		JP2(4, 0) = 0;
		JP2(4, 1) = 0;
		JP2(4, 2) = 0;
		JP2(4, 3) = -2 * c;
		JP2(4, 4) = 0;
		JP2(4, 5) = -2 * a;
		JP2(4, 6) = 0;

		JP2(5, 0) = 0;
		JP2(5, 1) = 0;
		JP2(5, 2) = 0;
		JP2(5, 3) = 0;
		JP2(5, 4) = -2 * c;
		JP2(5, 5) = -2 * b;
		JP2(5, 6) = 0;

		JP2(6, 0) = (-2 * A0);
		JP2(6, 1) = (2 * a * b);
		JP2(6, 2) = (2 * a * c);
		JP2(6, 3) = (4 * a * x0 + 2 * b * y0 + 2 * c * z0);
		JP2(6, 4) = (2 * a * y0 + 2 * c * z0);
		JP2(6, 5) = (2 * a * z0);
		JP2(6, 6) = 0;

		JP2(7, 0) = (2 * a * b);
		JP2(7, 1) = (-2 * B0);
		JP2(7, 2) = (2 * b * c);
		JP2(7, 3) = (2 * b * x0);
		JP2(7, 4) = (4 * b * y0 + 2 * a * x0 + 2 * c * z0);
		JP2(7, 5) = (2 * b * z0);
		JP2(7, 6) = 0;

		JP2(8, 0) = (2 * a * c);
		JP2(8, 1) = (2 * b * c);
		JP2(8, 2) = (-2 * C0);
		JP2(8, 3) = (2 * c * x0);
		JP2(8, 4) = (2 * c * y0);
		JP2(8, 5) = (4 * c * z0 + 2 * a * x0 + 2 * b * y0);
		JP2(8, 6) = 0;

		JP2(9, 0) = djx0;
		JP2(9, 1) = djy0;
		JP2(9, 2) = djz0;
		JP2(9, 3) = dja;
		JP2(9, 4) = djb;
		JP2(9, 5) = djc;
		JP2(9, 6) = djr;
		Eigen::MatrixXd JPUP = JP1 * JP2;

		Eigen::Matrix<double, 7, 7> JPDOWN;//对参数进行限制。就如同激活函数一样。
		double norm = pow(a, 2) + pow(b, 2) + pow(c, 2);
		JPDOWN.setZero();
		JPDOWN(0, 0) = 1;
		JPDOWN(1, 1) = 1;
		JPDOWN(2, 2) = 1;
		JPDOWN(3, 3) = pow(norm, 0.5) - a * a * pow(norm, -1.5);
		JPDOWN(4, 4) = pow(norm, 0.5) - b * b * pow(norm, -1.5);
		JPDOWN(5, 5) = pow(norm, 0.5) - b * b * pow(norm, -1.5);
		if (roriginal < minRadius)
		{
			JPDOWN(6, 6) = 0;
		}
		else if (roriginal > maxRadius)
		{
			JPDOWN(6, 6) = 0;
		}
		else
		{
			JPDOWN(6, 6) = 1;
		}
		JPUP = JPUP * JPDOWN;

		output = Y;
		J_set.push_back(JPUP);//JX
		return J_set;

	}

	bool fitCylinder(Eigen::MatrixXd& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius)
	{
		double minRadius = 0.01;
		double maxRadius = 500;

		Eigen::Vector<double, 7> coeifficient_temp;
		//需要根据input确定初始值
		Eigen::Vector3d cnerter = preCenter;


		coeifficient_temp = Eigen::Vector<double, 7>(cnerter[0], cnerter[1], cnerter[2], preAxis[0], preAxis[1], preAxis[2], preRadius);
		Eigen::MatrixXd E_identity = Eigen::MatrixXd::Identity(7, 7);
		Eigen::MatrixXd Y0 = Eigen::MatrixXd::Zero(input.cols(), 1);
		for (size_t i = 0; i < 10; i++)
		{
			Eigen::MatrixXd Y;
			std::vector<Eigen::MatrixXd> J_set = circleProcessDerivative(coeifficient_temp, input, Y);
			Eigen::MatrixXd J = J_set[0];
			Eigen::MatrixXd JT = J.transpose();
			Eigen::MatrixXd theta = (JT * J + 1E-10 * E_identity).inverse() * JT * (Y0 - Y);
			cv::Mat Y_mat = getMat(Y);
			cv::Mat J_mat = getMat(J);
			cv::Mat theta_mat = getMat(theta);
			double Y_norm = (Y0 - Y).norm();
			coeifficient_temp = coeifficient_temp + 0.1 * theta;
			if (theta.norm() < 1E-5)
			{
				break;
			}
			//cv::Mat coeifficient_temp_mat = getMat(coeifficient_temp);
		}
		double x0 = coeifficient_temp[0];
		double y0 = coeifficient_temp[1];
		double z0 = coeifficient_temp[2];
		double a = coeifficient_temp[3];
		double b = coeifficient_temp[4];
		double c = coeifficient_temp[5];
		double r = coeifficient_temp[6];
		Eigen::Vector3d axisNormal = Eigen::Vector3d(a, b, c);
		Eigen::Vector3d axisCenter = Eigen::Vector3d(x0, y0, z0);
		axisNormal.normalize();

		Eigen::Vector3d centerProjectPoint = HG::getProjectionPoint_pointToLine(cnerter, axisCenter, axisNormal);

		coeifficient = Eigen::VectorXd(7);
		coeifficient << centerProjectPoint[0], centerProjectPoint[1], centerProjectPoint[2], axisNormal[0], axisNormal[1], axisNormal[2], abs(r);

		return 0;
	}
	bool fitCylinder(Eigen::MatrixXd& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius, Eigen::Vector3d centerMask, Eigen::Vector3d axisMask, int radiusMask)
	{
		double minRadius = 0.01;
		double maxRadius = 500;

		Eigen::Vector<double, 7> coeifficient_temp;
		//需要根据input确定初始值
		Eigen::Vector3d cnerter = preCenter;


		coeifficient_temp = Eigen::Vector<double, 7>(cnerter[0], cnerter[1], cnerter[2], preAxis[0], preAxis[1], preAxis[2], preRadius);
		Eigen::MatrixXd E_identity = Eigen::MatrixXd::Identity(7, 7);
		Eigen::MatrixXd Y0 = Eigen::MatrixXd::Zero(input.cols(), 1);
		Eigen::MatrixXd Mask = E_identity;
		Mask(0, 0) = centerMask[0];
		Mask(1, 1) = centerMask[1];
		Mask(2, 2) = centerMask[2];
		Mask(3, 3) = axisMask[0];
		Mask(4, 4) = axisMask[1];
		Mask(5, 5) = axisMask[2];
		Mask(6, 6) = radiusMask;
		for (size_t i = 0; i < 10; i++)
		{
			Eigen::MatrixXd Y;
			std::vector<Eigen::MatrixXd> J_set = circleProcessDerivative(coeifficient_temp, input, Y);
			Eigen::MatrixXd J = J_set[0];
			Eigen::MatrixXd JT = J.transpose();
			Eigen::MatrixXd theta = (JT * J + 1E-10 * E_identity).inverse() * JT * (Y0 - Y);
			cv::Mat Y_mat = getMat(Y);
			cv::Mat J_mat = getMat(J);
			cv::Mat theta_mat = getMat(theta);
			double Y_norm = (Y0 - Y).norm();
			coeifficient_temp = coeifficient_temp + 0.1 * Mask * theta;
			if (theta.norm() < 1E-5)
			{
				break;
			}
			//cv::Mat coeifficient_temp_mat = getMat(coeifficient_temp);
		}
		double x0 = coeifficient_temp[0];
		double y0 = coeifficient_temp[1];
		double z0 = coeifficient_temp[2];
		double a = coeifficient_temp[3];
		double b = coeifficient_temp[4];
		double c = coeifficient_temp[5];
		double r = coeifficient_temp[6];
		Eigen::Vector3d axisNormal = Eigen::Vector3d(a, b, c);
		Eigen::Vector3d axisCenter = Eigen::Vector3d(x0, y0, z0);
		axisNormal.normalize();

		Eigen::Vector3d centerProjectPoint = HG::getProjectionPoint_pointToLine(cnerter, axisCenter, axisNormal);

		coeifficient = Eigen::VectorXd(7);
		coeifficient << centerProjectPoint[0], centerProjectPoint[1], centerProjectPoint[2], axisNormal[0], axisNormal[1], axisNormal[2], abs(r);
		return 0;
	}
	bool fitCylinder(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius)
	{
		Eigen::MatrixXd input_matrix = getMatrix(input);
		return fitCylinder(input_matrix, coeifficient, preCenter, preAxis, preRadius);
	}
	bool fitCylinder(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::VectorXd& coeifficient, Eigen::Vector3d preCenter, Eigen::Vector3d preAxis, double preRadius, Eigen::Vector3d centerMask, Eigen::Vector3d axisMask, int radiusMask)
	{
		Eigen::MatrixXd input_matrix = getMatrix(input);
		Eigen::Vector3d center = preCenter;
		Eigen::Vector3d axis = preAxis;
		double radius = preRadius;
		return fitCylinder(input_matrix, coeifficient, center, axis, radius, centerMask, axisMask, radiusMask);
	}
	pcl::PointCloud<pcl::PointXYZ> getNearPoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointXYZ point, int ksearch)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		if (kdtree.nearestKSearch(point, ksearch, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				cloud_filtered->push_back(input[pointIdxRadiusSearch[i]]);
			}
		}
		return *cloud_filtered;
	}
	pcl::PointCloud<pcl::PointXYZ> getNearPoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& point, double radius)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		std::vector<int> input_process(input.size(), 0);
		for (size_t i = 0; i < point.size(); i++)
		{
			if (kdtree.radiusSearch(point[i], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
			{
				for (size_t i = 0; i < pointIdxRadiusSearch.size(); i++)
				{
					if (input_process[pointIdxRadiusSearch[i]] == 0)
					{
						cloud_filtered.push_back(input[pointIdxRadiusSearch[i]]);
						input_process[pointIdxRadiusSearch[i]] = 1;
					}
				}
			}
		}
		return cloud_filtered;

	}
	bool getNearPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> input_normal, const pcl::PointXYZ point, int ksearch, pcl::PointCloud<pcl::PointXYZ>& output, pcl::PointCloud<pcl::Normal>& output_normal)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::Normal>::Ptr cloud_filtered_normal(new pcl::PointCloud<pcl::Normal>);
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		if (kdtree.nearestKSearch(point, ksearch, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
		{
			for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			{
				cloud_filtered->push_back(input[pointIdxRadiusSearch[i]]);
				cloud_filtered_normal->push_back(input_normal[pointIdxRadiusSearch[i]]);
			}
		}
		output = *cloud_filtered;
		output_normal = *cloud_filtered_normal;
		return 1;
	}

	pcl::PointCloud<pcl::FPFHSignature33> getFPFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		normalEstimation.setRadiusSearch(radius);
		normalEstimation.compute(*normals);
		pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
		fpfhEstimation.setInputCloud(input.makeShared());
		fpfhEstimation.setInputNormals(normals);
		fpfhEstimation.setSearchMethod(tree);
		fpfhEstimation.setRadiusSearch(radius);
		fpfhEstimation.compute(*fpfhs);
		return *fpfhs;
	}
	pcl::PointCloud<pcl::SHOT352> getSHOT(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::SHOT352>::Ptr shots(new pcl::PointCloud<pcl::SHOT352>());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		normalEstimation.setKSearch(16);
		//normalEstimation.setRadiusSearch(radius);
		normalEstimation.compute(*normals);
		pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shotEstimation;
		shotEstimation.setInputCloud(input.makeShared());
		shotEstimation.setInputNormals(normals);
		shotEstimation.setSearchMethod(tree);
		shotEstimation.setRadiusSearch(radius);
		shotEstimation.compute(*shots);
		return *shots;
	}
	pcl::PointCloud<pcl::PrincipalRadiiRSD> getRSD(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsds(new pcl::PointCloud<pcl::PrincipalRadiiRSD>());
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		//normalEstimation.setKSearch(16);
		normalEstimation.setRadiusSearch(radius);
		normalEstimation.compute(*normals);
		rsd.setInputCloud(input.makeShared());
		rsd.setInputNormals(normals);
		rsd.setSearchMethod(tree);
		// Search radius, to look for neighbors. Note: the value given here has to be
	// larger than the radius used to estimate the normals.
		rsd.setRadiusSearch(1.1 * radius);
		// Plane radius. Any radius larger than this is considered infinite (a plane).
		rsd.setPlaneRadius(1000);
		// Do we want to save the full distance-angle histograms?
		rsd.setSaveHistograms(true);
		rsd.compute(*rsds);
		return *rsds;
	}
	pcl::PointCloud<pcl::VFHSignature308> getVFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs(new pcl::PointCloud<pcl::VFHSignature308>());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		normalEstimation.setKSearch(16);
		//normalEstimation.setRadiusSearch(radius);
		normalEstimation.compute(*normals);
		pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfhEstimation;
		vfhEstimation.setInputCloud(input.makeShared());
		vfhEstimation.setInputNormals(normals);
		vfhEstimation.setSearchMethod(tree);
		//vfhEstimation.setRadiusSearch(radius);
		vfhEstimation.compute(*vfhs);
		return *vfhs;
	}
	pcl::PointCloud<pcl::VFHSignature308> getCVFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		// Object for storing the normals.
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		// Object for storing the CVFH descriptors.
		pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptors(new pcl::PointCloud<pcl::VFHSignature308>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setKSearch(16);
		//normalEstimation.setRadiusSearch(0.03);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setSearchMethod(kdtree);
		normalEstimation.compute(*normals);

		// CVFH estimation object.
		pcl::CVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh;
		cvfh.setInputCloud(input.makeShared());
		cvfh.setInputNormals(normals);
		cvfh.setSearchMethod(kdtree);
		// Set the maximum allowable deviation of the normals,
		// for the region segmentation step.
		cvfh.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
		// Set the curvature threshold (maximum disparity between curvatures),
		// for the region segmentation step.
		cvfh.setCurvatureThreshold(1.0);
		// Set to true to normalize the bins of the resulting histogram,
		// using the total number of points. Note: enabling it will make CVFH
		// invariant to scale just like VFH, but the authors encourage the opposite.
		cvfh.setNormalizeBins(false);
		cvfh.compute(*descriptors);
		return *descriptors;
	}
	pcl::PointCloud<pcl::VFHSignature308> getOURCVFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::VFHSignature308>::Ptr ourcvfhs(new pcl::PointCloud<pcl::VFHSignature308>());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		normalEstimation.setKSearch(16);
		//normalEstimation.setRadiusSearch(radius);
		normalEstimation.compute(*normals);
		pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfhEstimation;
		ourcvfhEstimation.setInputCloud(input.makeShared());
		ourcvfhEstimation.setInputNormals(normals);
		ourcvfhEstimation.setSearchMethod(tree);
		ourcvfhEstimation.setEPSAngleThreshold(5.0 / 180.0 * M_PI); // 5 degrees.
		ourcvfhEstimation.setCurvatureThreshold(1.0);
		ourcvfhEstimation.setNormalizeBins(false);
		// Set the minimum axis ratio between the SGURF axes. At the disambiguation phase,
		// this will decide if additional Reference Frames need to be created, if ambiguous.
		ourcvfhEstimation.setAxisRatio(0.8);
		ourcvfhEstimation.compute(*ourcvfhs);
		return *ourcvfhs;
	}
	pcl::PointCloud<pcl::GFPFHSignature16> getGFPFH(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::GFPFHSignature16>::Ptr gfphs(new pcl::PointCloud<pcl::GFPFHSignature16>());
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::GFPFHEstimation<pcl::PointXYZL, pcl::PointXYZL, pcl::GFPFHSignature16> gfphEstimation;
		pcl::PointCloud<pcl::PointXYZL>::Ptr object(new pcl::PointCloud<pcl::PointXYZL>);
		object->width = input.width;
		object->height = input.height;
		object->is_dense = false;
		object->points.resize(input.width * input.height);
		for (size_t i = 0; i < object->points.size(); ++i)
		{
			object->points[i].x = input.points[i].x;
			object->points[i].y = input.points[i].y;
			object->points[i].z = input.points[i].z;
			object->points[i].label = 1 + i % 4;
		}
		gfphEstimation.setInputCloud(object);
		gfphEstimation.setInputLabels(object);
		gfphEstimation.setOctreeLeafSize(radius);
		gfphEstimation.setNumberOfClasses(4);
		gfphEstimation.compute(*gfphs);
		return *gfphs;
	}
	pcl::PointCloud<pcl::ESFSignature640> getESF(const pcl::PointCloud<pcl::PointXYZ>& input, double)
	{
		pcl::PointCloud<pcl::ESFSignature640>::Ptr esfs(new pcl::PointCloud<pcl::ESFSignature640>());
		pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esfEstimation;
		esfEstimation.setInputCloud(input.makeShared());
		esfEstimation.compute(*esfs);
		return *esfs;
	}
	pcl::PointCloud<pcl::GRSDSignature21> getGRSD(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::GRSDSignature21>::Ptr grsds(new pcl::PointCloud<pcl::GRSDSignature21>());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		normalEstimation.setKSearch(16);
		//normalEstimation.setRadiusSearch(radius);
		normalEstimation.compute(*normals);
		pcl::GRSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::GRSDSignature21> grsdEstimation;
		grsdEstimation.setInputCloud(input.makeShared());
		grsdEstimation.setInputNormals(normals);
		grsdEstimation.setSearchMethod(tree);
		grsdEstimation.setRadiusSearch(radius);
		grsdEstimation.compute(*grsds);
		return *grsds;
	}
	pcl::PointCloud<pcl::PointXYZ> getHarrisPoints(const pcl::PointCloud<pcl::PointXYZ>& input_cloud, double r_normal, double r_keypoint)
	{
		//r_normal = 0.1/*stof(argv[2])*/;
		//r_keypoint = 0.1/*stof(argv[3])*/;
		typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ColorHandlerT3;

		pcl::PointCloud<pcl::PointXYZI>::Ptr Harris_keypoints(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>* harris_detector = new pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal>;

		//harris_detector->setNonMaxSupression(true);
		harris_detector->setRadius(r_normal);
		harris_detector->setRadiusSearch(r_keypoint);
		harris_detector->setInputCloud(input_cloud.makeShared());
		//harris_detector->setNormals(normal_source);
		//harris_detector->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::LOWE);
		harris_detector->compute(*Harris_keypoints);
		//cout << "Harris_keypoints的大小是" << Harris_keypoints->size() << endl;
		pcl::PointCloud<pcl::PointXYZ> ::Ptr cloud_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t i = 0; i < Harris_keypoints->size(); i++)
		{
			cloud_keypoints->push_back(pcl::PointXYZ(Harris_keypoints->points[i].x, Harris_keypoints->points[i].y, Harris_keypoints->points[i].z));
		}
		return *cloud_keypoints;
	}
	pcl::PointCloud<pcl::ShapeContext1980> getShapeContext(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::ShapeContext1980>::Ptr shapeContexts(new pcl::PointCloud<pcl::ShapeContext1980>());
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		normalEstimation.setKSearch(16);
		//normalEstimation.setRadiusSearch(radius);
		normalEstimation.compute(*normals);
		pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> shapeContextEstimation;
		shapeContextEstimation.setInputCloud(input.makeShared());
		shapeContextEstimation.setInputNormals(normals);
		shapeContextEstimation.setSearchMethod(tree);
		shapeContextEstimation.setRadiusSearch(radius);
		shapeContextEstimation.compute(*shapeContexts);
		return *shapeContexts;
	}
	Eigen::Vector3d getCenterPoint(const pcl::PointCloud<pcl::PointXYZ>& input)
	{
		Eigen::Vector3d center_point;
		center_point << 0, 0, 0;
		for (size_t i = 0; i < input.size(); i++)
		{
			center_point += Eigen::Vector3d(input[i].x, input[i].y, input[i].z);
		}
		center_point = center_point / input.size();
		return center_point;
	}

	pcl::PointCloud<pcl::PointXYZ> removeOutlierPoints(const pcl::PointCloud<pcl::PointXYZ>& input, int meanK, double stdDevMulThresh)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud(input.makeShared());
		sor.setMeanK(meanK);
		sor.setStddevMulThresh(stdDevMulThresh);
		sor.filter(*cloud_filtered);
		return *cloud_filtered;
	}
	pcl::PointCloud<pcl::PointXYZ> removeOutlierPoints_density(const pcl::PointCloud<pcl::PointXYZ>& input, int meanK, double stdDevMulThresh)
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
		std::vector<bool> isProcessed(input.size(), false);
		double distance_square_theshold = pow(stdDevMulThresh, 2);
		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类		
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;

				if (kdtree.radiusSearch(input[i], 2 * stdDevMulThresh, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
				{
					bool isProcessed_temp = false;
					if (pointIdxRadiusSearch.size() >= meanK)
					{
						isProcessed_temp = true;
					}
					if (isProcessed_temp)
					{
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
								isProcessed[pointIdxRadiusSearch[j]] = true;
							}
						}
					}

				}
			}
		}
		return cloud_cluster;
	}
	pcl::Normal getNormal(const pcl::PointCloud<pcl::PointXYZ>& input)
	{
		//构建matrix
		Eigen::Vector4d plana_para = getPlaneEquation(input);
		pcl::Normal normal_pcl;
		normal_pcl.normal_x = plana_para[0];
		normal_pcl.normal_y = plana_para[1];
		normal_pcl.normal_z = plana_para[2];
		return normal_pcl;
	}
	pcl::Normal getNormal(const pcl::PointCloud<pcl::PointXYZ>& input, const std::vector<int> pointIdxNKNSearch)
	{

		pcl::PointCloud<pcl::PointXYZ> plane_points;
		for (size_t i = 0; i < pointIdxNKNSearch.size(); i++)
		{
			plane_points.push_back(input[pointIdxNKNSearch[i]]);
		}
		//构建matrix
		Eigen::Vector4d plana_para = getPlaneEquation(plane_points);
		pcl::Normal normal_pcl;
		normal_pcl.normal_x = plana_para[0];
		normal_pcl.normal_y = plana_para[1];
		normal_pcl.normal_z = plana_para[2];
		return normal_pcl;
	}
	pcl::PointCloud<pcl::Normal> getNormal(const pcl::PointCloud<pcl::PointXYZ>& input, double radius)
	{
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		normalEstimation.setInputCloud(input.makeShared());
		normalEstimation.setSearchMethod(tree);
		normalEstimation.setKSearch(8);
		//normalEstimation.setRadiusSearch(2*radius);
		normalEstimation.compute(*cloud_normals);
		return *cloud_normals;
	}
	pcl::PointCloud<pcl::Normal> getNormal_my(const pcl::PointCloud<pcl::PointXYZ>& input, int ksearch)
	{
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());

		for (size_t i = 0; i < input.size(); i++)
		{
			//k近邻
			std::vector<int> pointIdxNKNSearch(ksearch);
			std::vector<float> pointNKNSquaredDistance(ksearch);
			kdtree.nearestKSearch(input[i], ksearch, pointIdxNKNSearch, pointNKNSquaredDistance);
			pcl::Normal normal_pcl = getNormal(input, pointIdxNKNSearch);
			cloud_normals->push_back(normal_pcl);
		}
		return *cloud_normals;
	}
	pcl::PointCloud<pcl::PrincipalCurvatures> getPrincipalCurvatures(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double radius)
	{
		pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvature(new pcl::PointCloud<pcl::PrincipalCurvatures>);
		pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> principalCurvaturesEstimation;
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		principalCurvaturesEstimation.setInputCloud(input.makeShared());
		principalCurvaturesEstimation.setInputNormals(normal.makeShared());
		principalCurvaturesEstimation.setSearchMethod(tree);
		principalCurvaturesEstimation.setKSearch(4);
		//principalCurvaturesEstimation.setRadiusSearch(2*radius);
		principalCurvaturesEstimation.compute(*cloud_curvature);
		return *cloud_curvature;
	}
	pcl::PointXYZ getCenter(const pcl::PointCloud<pcl::PointXYZ>& input)
	{
		pcl::PointXYZ center;
		center.x = 0;
		center.y = 0;
		center.z = 0;
		if (input.size() == 0)
		{
			return center;
		}
		for (size_t i = 0; i < input.size(); i++)
		{
			center.x += input[i].x;
			center.y += input[i].y;
			center.z += input[i].z;
		}
		center.x = center.x / input.size();
		center.y = center.y / input.size();
		center.z = center.z / input.size();
		return center;
	}
	pcl::PointCloud<pcl::PointXYZ> Centralized(const pcl::PointCloud<pcl::PointXYZ>& input)
	{
		//中心化点云
		//输出 output 中心化之后的点云，center 中心点
		pcl::PointXYZ center = getCenter(input);
		pcl::PointCloud<pcl::PointXYZ> output;
		for (size_t i = 0; i < input.size(); i++)
		{
			pcl::PointXYZ point;
			point.x = input[i].x - center.x;
			point.y = input[i].y - center.y;
			point.z = input[i].z - center.z;
			output.push_back(point);
		}
		return output;
	}
	pcl::PointCloud<pcl::PointXYZ> Centralized(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointXYZ& center)
	{
		//中心化点云
		//输出 output 中心化之后的点云，center 中心点
		pcl::PointCloud<pcl::PointXYZ> output;
		for (size_t i = 0; i < input.size(); i++)
		{
			pcl::PointXYZ point;
			point.x = input[i].x - center.x;
			point.y = input[i].y - center.y;
			point.z = input[i].z - center.z;
			output.push_back(point);
		}
		return output;
	}
	pcl::PointCloud<pcl::PointXYZ> DeCentralized(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointXYZ& center)
	{
		//去中心化点云
		//输入 input 中心化之后的点云，center 中心点
		pcl::PointCloud<pcl::PointXYZ> output;
		for (size_t i = 0; i < input.size(); i++)
		{
			pcl::PointXYZ point;
			point.x = input[i].x + center.x;
			point.y = input[i].y + center.y;
			point.z = input[i].z + center.z;
			output.push_back(point);
		}
		return output;
	}
	pcl::PointCloud<pcl::PointXYZ> getSamplePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_voxel(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> sor_VoxelGrid;
		sor_VoxelGrid.setInputCloud(input.makeShared());
		sor_VoxelGrid.setLeafSize(threshold, threshold, threshold);//体素采样的参数，指的是采样体素的大小
		sor_VoxelGrid.filter(*cloud_filtered_voxel);
		return *cloud_filtered_voxel;
	}
	pcl::PointCloud<pcl::PointXYZ> passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>& input, double min, double max, std::string axis)
	{
		// 创建PassThrough滤波器对象 
		pcl::PointCloud<pcl::PointXYZ> output;
		pcl::PassThrough<pcl::PointXYZ> pass;
		pass.setInputCloud(input.makeShared());
		pass.setFilterFieldName(axis);    // 指定滤波维度
		pass.setFilterLimits(min, max);  //  指定滤波范围（0.0,1.0）
		pass.setNegative(false);          // false: 滤除范围外的点（默认）；true:  滤除范围内的点
		pass.filter(output);
		return output;
	}
	bool removePoints_oneDirection(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double threshold, Eigen::Vector3d direction, pcl::PointCloud<pcl::PointXYZ>& inCloud, pcl::PointCloud<pcl::PointXYZ>& nonCloud)
	{
		inCloud.clear();
		nonCloud.clear();
		//根据某一个方向判断是否删除点，如果点的法向量与该方向的夹角大于阈值，则删除
		pcl::PointCloud<pcl::PointXYZ> cloud_segments;//采集的结果
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d normal_vector;
			normal_vector << normal[i].normal_x, normal[i].normal_y, normal[i].normal_z;
			double angle = getAngleBetweenVectors(normal_vector, direction);
			if (angle > M_PI / 2.0)
			{
				angle = M_PI - angle;
			}
			if (angle < threshold)
			{
				inCloud.push_back(input[i]);
			}
			else
			{
				nonCloud.push_back(input[i]);
			}
		}
		return 1;
	}
	bool removePoints_oneDirection(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double minThreshold, double maxThreshold, Eigen::Vector3d direction, pcl::PointCloud<pcl::PointXYZ>& inCloud, pcl::PointCloud<pcl::PointXYZ>& nonCloud)
	{
		//根据某一个方向判断是否删除点，如果点的法向量与该方向的夹角大于阈值，则删除
		pcl::PointCloud<pcl::PointXYZ> cloud_segments;//采集的结果
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d normal_vector;
			normal_vector << normal[i].normal_x, normal[i].normal_y, normal[i].normal_z;
			double angle = getAngleBetweenVectors(normal_vector, direction);
			if (angle > M_PI / 2.0)
			{
				angle = M_PI - angle;
			}
			if (angle < maxThreshold && angle > minThreshold)
			{
				inCloud.push_back(input[i]);
			}
			else
			{
				nonCloud.push_back(input[i]);
			}
		}
		return 1;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_EC(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize, double maxSize)
	{
		//创建一个kd树对象
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud(input.makeShared());
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setClusterTolerance(2 * threshold); // 设置近邻搜索的搜索半径为2cm
		ec.setMinClusterSize(minSize); //设置一个聚类需要的最少的点数目为100
		ec.setMaxClusterSize(maxSize); //设置一个聚类需要的最大点数目为25000
		ec.setSearchMethod(tree); //设置点云的搜索机制
		ec.setInputCloud(input.makeShared());
		ec.extract(cluster_indices); //从点云中提取聚类，并将点云索引保存在cluster_indices中
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cluster_points;
		std::vector<int> cluster_points_size;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
				cloud_cluster.push_back(input[*pit]); //*
			cloud_cluster.width = cloud_cluster.size();
			cloud_cluster.height = 1;
			cloud_cluster.is_dense = true;
			cluster_points.push_back(cloud_cluster);
		}
		//按照点云数量排序，从小到大
		for (size_t i = 0; i < cluster_points.size(); i++)
		{
			for (size_t j = i + 1; j < cluster_points.size(); j++)
			{
				if (cluster_points[i].size() > cluster_points[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp = cluster_points[i];
					cluster_points[i] = cluster_points[j];
					cluster_points[j] = temp;
				}
			}
		}
		return cluster_points;
	}
	bool enforceIntensitySimilarity(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
	{
		//if (std::abs(point_a.intensity - point_b.intensity) < 1E-6f &&
		//	abs(point_b.intensity) < 1E-5f)
		return (true);
		//else
			//return (false);
	}
	bool enforceIntensitySimilarity_plane(const pcl::PointXYZI& point_a, const pcl::PointXYZI& point_b, float squared_distance)
	{
		if (point_a.intensity != 0)
		{
			if (std::abs((point_a.intensity - point_b.intensity) / point_a.intensity) < 1E-2)
				return (true);
			else
				return (false);
		}
		else
		{
			if (abs(point_b.intensity) < 1E-5)
			{
				return (true);
			}
			return (false);
		}
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize, double maxSize)
	{
		//构建 PointXYZI 特征值

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzipoints(new pcl::PointCloud< pcl::PointXYZI>);
		cloud_xyzipoints->width = input.width;
		cloud_xyzipoints->height = input.height;
		//is_dense is True if no points have NaN or Inf in any of their floating points field
		cloud_xyzipoints->is_dense = false;
		cloud_xyzipoints->points.resize(cloud_xyzipoints->width * cloud_xyzipoints->height);
		for (size_t i = 0; i < cloud_xyzipoints->width; ++i)
		{
			cloud_xyzipoints->points[i].x = input.points[i].x;
			cloud_xyzipoints->points[i].y = input.points[i].y;
			cloud_xyzipoints->points[i].z = input.points[i].z;
			cloud_xyzipoints->points[i].intensity = 0;// cloud_curv[i].pc1* cloud_curv[i].pc2;
			/*
			cout << i << "    " << cloud_xyzipoints->points[i].x <<
				"    " << cloud_xyzipoints->points[i].y <<
				"    " << cloud_xyzipoints->points[i].z <<
				"    " << cloud_xyzipoints->points[i].intensity << endl;
			 */
		}


		//点云聚类
		pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
		pcl::ConditionalEuclideanClustering< pcl::PointXYZI> cec(true);
		cec.setInputCloud(cloud_xyzipoints);
		cec.setConditionFunction(&enforceIntensitySimilarity);
		cec.setClusterTolerance(2 * threshold);
		cec.setMinClusterSize(minSize);//最小的聚类点数
		cec.setMaxClusterSize(maxSize);//最大的聚类点数
		pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_XYZI(new pcl::search::KdTree<pcl::PointXYZI>());
		cec.setSearchMethod(tree_XYZI); //设置点云的搜索机制
		// The resulting clusters (an array of pointindices):
		//std::vector<pcl::PointIndices> clusters;
		cec.segment(*clusters);
		// The clusters that are too small or too large in size can also be extracted separately:
		cec.getRemovedClusters(small_clusters, large_clusters);
		//对每个聚类结果处理
		int currentClusterNum = 1;
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		for (auto i = clusters->begin(); i < clusters->end(); ++i)
		{
			// ...add all its points to a new cloud...
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				cluster->points.push_back(input.points[*point]);
			cluster->width = cluster->points.size();
			cluster->height = 1;
			cluster->is_dense = true;

			// ...and save it to disk.
			if (cluster->points.size() <= 0)
				break;
			//保存点云
			cloud_segments.push_back(*cluster);
			//std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
			//std::string fileName = "cluster" + std::to_string(currentClusterNum) + ".pcd";
			//pcl::io::savePCDFileASCII(fileName, *cluster);
			currentClusterNum++;
		}

		//按照点云数量排序，从小到大
		for (size_t i = 0; i < cloud_segments.size(); i++)
		{
			for (size_t j = i + 1; j < cloud_segments.size(); j++)
			{
				if (cloud_segments[i].size() > cloud_segments[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp = cloud_segments[i];
					cloud_segments[i] = cloud_segments[j];
					cloud_segments[j] = temp;
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_plane(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> Feature, double threshold, double minSize, double maxSize)
	{
		//构建 PointXYZI 特征值

		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzipoints(new pcl::PointCloud< pcl::PointXYZI>);
		cloud_xyzipoints->width = input.width;
		cloud_xyzipoints->height = input.height;
		//is_dense is True if no points have NaN or Inf in any of their floating points field
		cloud_xyzipoints->is_dense = false;
		cloud_xyzipoints->points.resize(cloud_xyzipoints->width * cloud_xyzipoints->height);
		for (size_t i = 0; i < cloud_xyzipoints->width; ++i)
		{
			cloud_xyzipoints->points[i].x = input.points[i].x;
			cloud_xyzipoints->points[i].y = input.points[i].y;
			cloud_xyzipoints->points[i].z = input.points[i].z;
			cloud_xyzipoints->points[i].intensity = Feature[i].curvature;// cloud_curv[i].pc1* cloud_curv[i].pc2;
			/*
			cout << i << "    " << cloud_xyzipoints->points[i].x <<
				"    " << cloud_xyzipoints->points[i].y <<
				"    " << cloud_xyzipoints->points[i].z <<
				"    " << cloud_xyzipoints->points[i].intensity << endl;
			 */
		}
		//点云聚类
		pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters), small_clusters(new pcl::IndicesClusters), large_clusters(new pcl::IndicesClusters);
		pcl::ConditionalEuclideanClustering< pcl::PointXYZI> cec(true);
		cec.setInputCloud(cloud_xyzipoints);
		cec.setConditionFunction(&enforceIntensitySimilarity_plane);
		cec.setClusterTolerance(2 * threshold);
		cec.setMinClusterSize(minSize);//最小的聚类点数
		cec.setMaxClusterSize(maxSize);//最大的聚类点数
		pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_XYZI(new pcl::search::KdTree<pcl::PointXYZI>());
		cec.setSearchMethod(tree_XYZI); //设置点云的搜索机制
		// The resulting clusters (an array of pointindices):
		//std::vector<pcl::PointIndices> clusters;
		cec.segment(*clusters);
		// The clusters that are too small or too large in size can also be extracted separately:
		cec.getRemovedClusters(small_clusters, large_clusters);
		//对每个聚类结果处理
		int currentClusterNum = 1;
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		for (auto i = clusters->begin(); i < clusters->end(); ++i)
		{
			// ...add all its points to a new cloud...
			pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
			for (std::vector<int>::const_iterator point = i->indices.begin(); point != i->indices.end(); point++)
				cluster->points.push_back(input.points[*point]);
			cluster->width = cluster->points.size();
			cluster->height = 1;
			cluster->is_dense = true;

			// ...and save it to disk.
			if (cluster->points.size() <= 0)
				break;
			//保存点云
			cloud_segments.push_back(*cluster);
			//std::cout << "Cluster " << currentClusterNum << " has " << cluster->points.size() << " points." << std::endl;
			//std::string fileName = "cluster" + std::to_string(currentClusterNum) + ".pcd";
			//pcl::io::savePCDFileASCII(fileName, *cluster);
			currentClusterNum++;
		}

		//按照点云数量排序，从小到大
		for (size_t i = 0; i < cloud_segments.size(); i++)
		{
			for (size_t j = i + 1; j < cloud_segments.size(); j++)
			{
				if (cloud_segments[i].size() > cloud_segments[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp = cloud_segments[i];
					cloud_segments[i] = cloud_segments[j];
					cloud_segments[j] = temp;
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_my(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize, double maxSize)
	{
		/*
		1为点云P 创建KD - Tree的输入表征
		2创建空的聚类列表C 和 点云的检查队列Q
		3对于P中的每一个点Pi，执行如下操作：
		3.1- 将Pi添加到当前队列Q(并标记为已处理)；
		3.2- while处理 Q 中的每一个Pi：
		3.2.1- 对Pi进行近邻搜索，查找满足半径 < d 的点集合;
		3.2.2-检查上一步中每一个邻居点，如果标记是未被处理的点，则加入到队列Q中；
		3.3- 直到Q中的所有点都被标记为已处理，将Q加入到聚类列表C中，将Q重新置为空
		4当所有的Pi都被处理过之后结束，聚类结果为列表 C
		*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		std::vector<bool> isProcessed(input.size(), false);
		//计算点的fpfh特征
		pcl::PointCloud<pcl::FPFHSignature33> fpfh = getFPFH(input, threshold);
		//计算点的法向量
		//pcl::PointCloud<pcl::Normal> normal = getNormal(input, threshold);
		//计算点的曲率
		//pcl::PointCloud<pcl::PrincipalCurvatures> cloud_curv = getPrincipalCurvatures(input, normal, threshold);

		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类
				pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
				cloud_cluster.push_back(input[i]);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				//流程：1找到聚类中还没处理的点 2将符合条件的点加入到聚类中，循环，直到所有点都处理完了
				int k_tempt = 0;
				while (1)
				{
					if (k_tempt >= cloud_cluster.size())
					{
						break;
					}
					if (kdtree.radiusSearch(cloud_cluster[k_tempt], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						//聚类判断函数
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								Eigen::MatrixXd historgram0 = array2MatrixXd(fpfh[pointIdxRadiusSearch[0]].histogram, 33);
								Eigen::MatrixXd historgram1 = array2MatrixXd(fpfh[pointIdxRadiusSearch[j]].histogram, 33);
								double distance_fpfh = (historgram0 - historgram1).norm();
								bool isProcessed_temp = distance_fpfh < 1E0;
								if (isProcessed_temp)
								{
									cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
									isProcessed[pointIdxRadiusSearch[j]] = true;
								}
							}
						}
					}
					k_tempt++;
				}

				if (cloud_cluster.size() >= minSize && cloud_cluster.size() <= maxSize)
				{
					cloud_segments.push_back(cloud_cluster);
				}
			}
		}
		//按照点云数量排序，从小到大
		for (size_t i = 0; i < cloud_segments.size(); i++)
		{
			for (size_t j = i + 1; j < cloud_segments.size(); j++)
			{
				if (cloud_segments[i].size() > cloud_segments[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp = cloud_segments[i];
					cloud_segments[i] = cloud_segments[j];
					cloud_segments[j] = temp;
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_normal(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, double threshold, double minSize, double maxSize)
	{
		/*
		1为点云P 创建KD - Tree的输入表征
		2创建空的聚类列表C 和 点云的检查队列Q
		3对于P中的每一个点Pi，执行如下操作：
			3.1- 将Pi添加到当前队列Q(并标记为已处理)；
			3.2- while处理 Q 中的每一个Pi：
				3.2.1- 对Pi进行近邻搜索，查找满足半径 < d 的点集合;
				3.2.2-检查上一步中每一个邻居点，如果标记是未被处理的点，则加入到队列Q中；
			3.3- 直到Q中的所有点都被标记为已处理，将Q加入到聚类列表C中，将Q重新置为空
		4当所有的Pi都被处理过之后结束，聚类结果为列表 C
		*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		std::vector<bool> isProcessed(input.size(), false);

		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类
				pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
				cloud_cluster.push_back(input[i]);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				//流程：1找到聚类中还没处理的点 2将符合条件的点加入到聚类中，循环，直到所有点都处理完了
				//设计一个平均曲率，用来限制曲率过大的偏离
				double mean_curvature = 0;
				int k_tempt = 0;
				while (1)
				{
					if (k_tempt >= cloud_cluster.size())
					{
						break;
					}
					if (kdtree.radiusSearch(cloud_cluster[k_tempt], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						//聚类判断函数
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								//计算判断的值
								double pc1 = normal[pointIdxRadiusSearch[0]].curvature;
								double pc1j = normal[pointIdxRadiusSearch[j]].curvature;
								if (pc1 == 0)
								{
									pc1 += 1E-6;
								}
								double distance_curvature_pattern = abs((pc1 - pc1j) / pc1);//变化率；用来约束每个cluster中的曲率连续性
								double distance_curvature_mean = abs(pc1j * cloud_cluster.size() / mean_curvature);//绝对值；用来约束每个cluster中的曲率方差
								double alpha = 0.1;//alpha是【0，1】之间的一个数，用来约束曲率的变化率
								bool isProcessed_temp = distance_curvature_mean > alpha && distance_curvature_mean < pow(alpha, -1);
								if (j == 0)
								{
									isProcessed_temp = true;
								}
								if (isProcessed_temp)
								{
									cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
									isProcessed[pointIdxRadiusSearch[j]] = true;
									mean_curvature += pc1j;
								}
							}
						}
					}
					k_tempt++;
				}

				if (cloud_cluster.size() >= minSize && cloud_cluster.size() <= maxSize)
				{
					cloud_segments.push_back(cloud_cluster);
				}
			}
		}
		//按照点云数量排序，从小到大
		for (size_t i = 0; i < cloud_segments.size(); i++)
		{
			for (size_t j = i + 1; j < cloud_segments.size(); j++)
			{
				if (cloud_segments[i].size() > cloud_segments[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp = cloud_segments[i];
					cloud_segments[i] = cloud_segments[j];
					cloud_segments[j] = temp;
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_normal(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, std::vector<Eigen::MatrixXd>& Feature, double threshold, double minSize, double maxSize)
	{
		/*
		1为点云P 创建KD - Tree的输入表征
		2创建空的聚类列表C 和 点云的检查队列Q
		3对于P中的每一个点Pi，执行如下操作：
		3.1- 将Pi添加到当前队列Q(并标记为已处理)；
		3.2- while处理 Q 中的每一个Pi：
		3.2.1- 对Pi进行近邻搜索，查找满足半径 < d 的点集合;
		3.2.2-检查上一步中每一个邻居点，如果标记是未被处理的点，则加入到队列Q中；
		3.3- 直到Q中的所有点都被标记为已处理，将Q加入到聚类列表C中，将Q重新置为空
		4当所有的Pi都被处理过之后结束，聚类结果为列表 C
		*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		std::vector<bool> isProcessed(input.size(), false);

		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类
				pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
				cloud_cluster.push_back(input[i]);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				//创建一个特征向量
				Eigen::MatrixXd Feature_cluster(1, 9);
				//流程：1找到聚类中还没处理的点 2将符合条件的点加入到聚类中，循环，直到所有点都处理完了
				//设计一个平均曲率，用来限制曲率过大的偏离
				double mean_curvature = 0;//平均主曲率
				double mean_curvature_b = 0;//平均副曲率
				double mean_normal_x = 0;//平均法向量x
				double mean_normal_y = 0;//平均法向量y
				double mean_normal_z = 0;//平均法向量z
				double mean_x = 0;//平均x
				double mean_y = 0;//平均y
				double mean_z = 0;//平均z
				int k_tempt = 0;
				while (1)
				{
					if (k_tempt >= cloud_cluster.size())
					{
						break;
					}
					if (kdtree.radiusSearch(cloud_cluster[k_tempt], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						//聚类判断函数
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								//计算判断的值
								double pc1 = normal[pointIdxRadiusSearch[0]].curvature;
								double pc1j = normal[pointIdxRadiusSearch[j]].curvature;
								double pc2j = 0;// normal[pointIdxRadiusSearch[j]].pc2;
								double nxj = normal[pointIdxRadiusSearch[j]].normal_x;
								double nyj = normal[pointIdxRadiusSearch[j]].normal_y;
								double nzj = normal[pointIdxRadiusSearch[j]].normal_z;
								double xj = input[pointIdxRadiusSearch[j]].x;
								double yj = input[pointIdxRadiusSearch[j]].y;
								double zj = input[pointIdxRadiusSearch[j]].z;
								if (pc1 == 0)
								{
									pc1 += 1E-7;
								}
								double distance_curvature_pattern = abs((pc1 - pc1j) / pc1);//变化率；用来约束每个cluster中的曲率连续性
								double distance_curvature_mean = abs(pc1j * cloud_cluster.size() / mean_curvature);//绝对值；用来约束每个cluster中的曲率方差
								double alpha = 0.5;//alpha是【0，1】之间的一个数，用来约束曲率的变化率
								bool isProcessed_temp = false;// distance_curvature_mean > alpha&& distance_curvature_mean < pow(alpha, -1);
								if (j == 0)
								{
									isProcessed_temp = true;
								}
								if (abs(pc1j) < 1E-2 && abs(mean_curvature / cloud_cluster.size()) < 1E-2)//平面
								{
									isProcessed_temp = true;
								}
								else if (distance_curvature_mean > alpha && distance_curvature_mean < pow(alpha, -1))
								{
									//isProcessed_temp = true;
								}
								if (isProcessed_temp)
								{
									cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
									isProcessed[pointIdxRadiusSearch[j]] = true;
									mean_curvature += pc1j;
									mean_curvature_b += pc2j;
									mean_normal_x += nxj;
									mean_normal_y += nyj;
									mean_normal_z += nzj;
									mean_x += xj;
									mean_y += yj;
									mean_z += zj;
								}
							}
						}
					}
					//阶段结束，处理特征向量
					Feature_cluster(0, 0) = cloud_cluster.size();
					Feature_cluster(0, 1) = mean_curvature / cloud_cluster.size();//平均主曲率
					Feature_cluster(0, 2) = mean_curvature_b / cloud_cluster.size();//平均副曲率
					Feature_cluster(0, 3) = mean_normal_x / cloud_cluster.size();//平均法向量x
					Feature_cluster(0, 4) = mean_normal_y / cloud_cluster.size();//平均法向量y
					Feature_cluster(0, 5) = mean_normal_z / cloud_cluster.size();//平均法向量z
					Feature_cluster(0, 6) = mean_x / cloud_cluster.size();//平均x
					Feature_cluster(0, 7) = mean_y / cloud_cluster.size();//平均y
					Feature_cluster(0, 8) = mean_z / cloud_cluster.size();//平均z
					k_tempt++;
				}

				if (cloud_cluster.size() >= minSize && cloud_cluster.size() <= maxSize)
				{
					cloud_segments.push_back(cloud_cluster);
					Feature.push_back(Feature_cluster);
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_curvature(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PrincipalCurvatures> curvatureFeature, double threshold, double minSize, double maxSize)
	{
		/*
		1为点云P 创建KD - Tree的输入表征
		2创建空的聚类列表C 和 点云的检查队列Q
		3对于P中的每一个点Pi，执行如下操作：
		3.1- 将Pi添加到当前队列Q(并标记为已处理)；
		3.2- while处理 Q 中的每一个Pi：
		3.2.1- 对Pi进行近邻搜索，查找满足半径 < d 的点集合;
		3.2.2-检查上一步中每一个邻居点，如果标记是未被处理的点，则加入到队列Q中；
		3.3- 直到Q中的所有点都被标记为已处理，将Q加入到聚类列表C中，将Q重新置为空
		4当所有的Pi都被处理过之后结束，聚类结果为列表 C
		*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		std::vector<bool> isProcessed(input.size(), false);

		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类
				pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
				cloud_cluster.push_back(input[i]);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				//流程：1找到聚类中还没处理的点 2将符合条件的点加入到聚类中，循环，直到所有点都处理完了
				//设计一个平均曲率，用来限制曲率过大的偏离
				double mean_curvature = 0;
				int k_tempt = 0;
				while (1)
				{
					if (k_tempt >= cloud_cluster.size())
					{
						break;
					}
					if (kdtree.radiusSearch(cloud_cluster[k_tempt], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						//聚类判断函数
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								//计算判断的值
								double pc1 = curvatureFeature[pointIdxRadiusSearch[0]].pc1;
								double pc1j = curvatureFeature[pointIdxRadiusSearch[j]].pc1;
								if (pc1 == 0)
								{
									pc1 += 1E-6;
								}
								double distance_curvature_pattern = abs((pc1 - pc1j) / pc1);//变化率；用来约束每个cluster中的曲率连续性
								double distance_curvature_mean = abs(pc1j * cloud_cluster.size() / mean_curvature);//绝对值；用来约束每个cluster中的曲率方差
								double alpha = 0.1;//alpha是【0，1】之间的一个数，用来约束曲率的变化率
								bool isProcessed_temp = distance_curvature_mean > alpha && distance_curvature_mean < pow(alpha, -1);
								if (j == 0)
								{
									isProcessed_temp = true;
								}
								if (isProcessed_temp)
								{
									cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
									isProcessed[pointIdxRadiusSearch[j]] = true;
									mean_curvature += pc1j;
								}
							}
						}
					}
					k_tempt++;
				}

				if (cloud_cluster.size() >= minSize && cloud_cluster.size() <= maxSize)
				{
					cloud_segments.push_back(cloud_cluster);
				}
			}
		}
		//按照点云数量排序，从小到大
		for (size_t i = 0; i < cloud_segments.size(); i++)
		{
			for (size_t j = i + 1; j < cloud_segments.size(); j++)
			{
				if (cloud_segments[i].size() > cloud_segments[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp = cloud_segments[i];
					cloud_segments[i] = cloud_segments[j];
					cloud_segments[j] = temp;
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_curvature(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, pcl::PointCloud<pcl::PrincipalCurvatures> curvatureFeature, std::vector<Eigen::MatrixXd>& Feature, double threshold, double minSize, double maxSize)
	{
		/*
		1为点云P 创建KD - Tree的输入表征
		2创建空的聚类列表C 和 点云的检查队列Q
		3对于P中的每一个点Pi，执行如下操作：
		3.1- 将Pi添加到当前队列Q(并标记为已处理)；
		3.2- while处理 Q 中的每一个Pi：
		3.2.1- 对Pi进行近邻搜索，查找满足半径 < d 的点集合;
		3.2.2-检查上一步中每一个邻居点，如果标记是未被处理的点，则加入到队列Q中；
		3.3- 直到Q中的所有点都被标记为已处理，将Q加入到聚类列表C中，将Q重新置为空
		4当所有的Pi都被处理过之后结束，聚类结果为列表 C
		*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		std::vector<bool> isProcessed(input.size(), false);

		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类
				pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
				cloud_cluster.push_back(input[i]);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				//创建一个特征向量
				Eigen::MatrixXd Feature_cluster(1, 9);
				//流程：1找到聚类中还没处理的点 2将符合条件的点加入到聚类中，循环，直到所有点都处理完了
				//设计一个平均曲率，用来限制曲率过大的偏离
				double mean_curvature = 0;//平均主曲率
				double mean_curvature_b = 0;//平均副曲率
				double mean_normal_x = 0;//平均法向量x
				double mean_normal_y = 0;//平均法向量y
				double mean_normal_z = 0;//平均法向量z
				double mean_x = 0;//平均x
				double mean_y = 0;//平均y
				double mean_z = 0;//平均z
				int k_tempt = 0;
				while (1)
				{
					if (k_tempt >= cloud_cluster.size())
					{
						break;
					}
					if (kdtree.radiusSearch(cloud_cluster[k_tempt], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						//聚类判断函数
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								//计算判断的值
								double pc1 = curvatureFeature[pointIdxRadiusSearch[0]].pc1;
								double pc2 = curvatureFeature[pointIdxRadiusSearch[0]].pc2;
								double pc1j = curvatureFeature[pointIdxRadiusSearch[j]].pc1;
								double pc2j = curvatureFeature[pointIdxRadiusSearch[j]].pc2;
								double nxj = normal[pointIdxRadiusSearch[j]].normal_x;
								double nyj = normal[pointIdxRadiusSearch[j]].normal_y;
								double nzj = normal[pointIdxRadiusSearch[j]].normal_z;
								double xj = input[pointIdxRadiusSearch[j]].x;
								double yj = input[pointIdxRadiusSearch[j]].y;
								double zj = input[pointIdxRadiusSearch[j]].z;
								bool isProcessed_temp = false;//初始值为否
								double ruled_theshold = 1E-2;//直纹面阈值，区分pc2，主要用来区分直纹面和球面
								double plane_theshold = 1E-2;//平面阈值，区分pc1，阈值以上主要用来区分不同半径的圆柱面，阈值以下为平面
								if (j == 0)
								{
									isProcessed_temp = true;//第一个点直接加入
								}
								//判断是否为直纹面
								//就是高斯曲率为0
								else if (mean_curvature_b / cloud_cluster.size() < ruled_theshold)//当前聚类为直纹面
								{
									if (mean_curvature / cloud_cluster.size() < plane_theshold)//当前聚类为平面
									{
										if (pc2j < ruled_theshold && pc1j < plane_theshold)//当前点为直纹面，当前点为平面
										{
											isProcessed_temp = true;
										}
									}
									else//当前聚类为圆柱面
									{
										if (pc2j < ruled_theshold)//当前点为直纹面，
										{
											//根据圆柱半径判断是否为同一种圆柱面
											double radio = pc1j * cloud_cluster.size() / mean_curvature;
											double rate = 0.1;//半径的变化率
											if (radio < 1 / rate && radio > rate)//按照10倍左右区分每一个区域
											{
												//isProcessed_temp = true;
											}
										}
									}
								}
								else if (1)//当前聚类为曲面
								{
									//isProcessed_temp = true;//直接加入
								}

								if (isProcessed_temp)
								{
									cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
									isProcessed[pointIdxRadiusSearch[j]] = true;
									mean_curvature += pc1j;
									mean_curvature_b += pc2j;
									mean_normal_x += nxj;
									mean_normal_y += nyj;
									mean_normal_z += nzj;
									mean_x += xj;
									mean_y += yj;
									mean_z += zj;
								}
							}
						}
					}
					//阶段结束，处理特征向量
					Feature_cluster(0, 0) = cloud_cluster.size();
					Feature_cluster(0, 1) = mean_curvature / cloud_cluster.size();//平均主曲率
					Feature_cluster(0, 2) = mean_curvature_b / cloud_cluster.size();//平均副曲率
					Feature_cluster(0, 3) = mean_normal_x / cloud_cluster.size();//平均法向量x
					Feature_cluster(0, 4) = mean_normal_y / cloud_cluster.size();//平均法向量y
					Feature_cluster(0, 5) = mean_normal_z / cloud_cluster.size();//平均法向量z
					Feature_cluster(0, 6) = mean_x / cloud_cluster.size();//平均x
					Feature_cluster(0, 7) = mean_y / cloud_cluster.size();//平均y
					Feature_cluster(0, 8) = mean_z / cloud_cluster.size();//平均z
					k_tempt++;
				}

				if (cloud_cluster.size() >= minSize && cloud_cluster.size() <= maxSize)
				{
					cloud_segments.push_back(cloud_cluster);
					Feature.push_back(Feature_cluster);
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_curvature(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal> normal, pcl::PointCloud<pcl::PrincipalCurvatures> curvatureFeature, std::vector<Eigen::MatrixXd>& Feature,
		double threshold, double ruled_theshold, double plane_theshold, double rate, double minSize, double maxSize)
	{
		/*
		1为点云P 创建KD - Tree的输入表征
		2创建空的聚类列表C 和 点云的检查队列Q
		3对于P中的每一个点Pi，执行如下操作：
		3.1- 将Pi添加到当前队列Q(并标记为已处理)；
		3.2- while处理 Q 中的每一个Pi：
		3.2.1- 对Pi进行近邻搜索，查找满足半径 < d 的点集合;
		3.2.2-检查上一步中每一个邻居点，如果标记是未被处理的点，则加入到队列Q中；
		3.3- 直到Q中的所有点都被标记为已处理，将Q加入到聚类列表C中，将Q重新置为空
		4当所有的Pi都被处理过之后结束，聚类结果为列表 C
		*/
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		std::vector<bool> isProcessed(input.size(), false);

		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类
				pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
				cloud_cluster.push_back(input[i]);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				//创建一个特征向量
				Eigen::MatrixXd Feature_cluster(1, 9);
				//流程：1找到聚类中还没处理的点 2将符合条件的点加入到聚类中，循环，直到所有点都处理完了
				//设计一个平均曲率，用来限制曲率过大的偏离
				double mean_curvature = 0;//平均主曲率
				double mean_curvature_b = 0;//平均副曲率
				double mean_normal_x = 0;//平均法向量x
				double mean_normal_y = 0;//平均法向量y
				double mean_normal_z = 0;//平均法向量z
				double mean_x = 0;//平均x
				double mean_y = 0;//平均y
				double mean_z = 0;//平均z
				int k_tempt = 0;
				while (1)
				{
					if (k_tempt >= cloud_cluster.size())
					{
						break;
					}
					if (kdtree.radiusSearch(cloud_cluster[k_tempt], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						//聚类判断函数
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								//计算判断的值
								double pc1 = curvatureFeature[pointIdxRadiusSearch[0]].pc1;
								double pc2 = curvatureFeature[pointIdxRadiusSearch[0]].pc2;
								double pc1j = curvatureFeature[pointIdxRadiusSearch[j]].pc1;
								double pc2j = curvatureFeature[pointIdxRadiusSearch[j]].pc2;
								double nxj = normal[pointIdxRadiusSearch[j]].normal_x;
								double nyj = normal[pointIdxRadiusSearch[j]].normal_y;
								double nzj = normal[pointIdxRadiusSearch[j]].normal_z;
								double xj = input[pointIdxRadiusSearch[j]].x;
								double yj = input[pointIdxRadiusSearch[j]].y;
								double zj = input[pointIdxRadiusSearch[j]].z;
								bool isProcessed_temp = false;//初始值为否
								//double ruled_theshold = 1E-2;//直纹面阈值，区分pc2，主要用来区分直纹面和球面
								//double plane_theshold = 1E-2;//平面阈值，区分pc1，阈值以上主要用来区分不同半径的圆柱面，阈值以下为平面
								if (j == 0)
								{
									isProcessed_temp = true;//第一个点直接加入
								}
								//判断是否为直纹面
								//就是高斯曲率为0
								else if (mean_curvature_b / cloud_cluster.size() < ruled_theshold)//当前聚类为直纹面
								{
									if (mean_curvature / cloud_cluster.size() < plane_theshold)//当前聚类为平面
									{
										if (pc2j < ruled_theshold && pc1j < plane_theshold)//当前点为直纹面，当前点为平面
										{
											isProcessed_temp = true;
										}
									}
									else//当前聚类为圆柱面
									{
										if (pc2j < ruled_theshold)//当前点为直纹面，
										{
											//根据圆柱半径判断是否为同一种圆柱面
											double radio = pc1j * cloud_cluster.size() / mean_curvature;
											//double rate = 0.1;//半径的变化率
											if (radio < 1 / rate && radio > rate)//按照10倍左右区分每一个区域
											{
												//isProcessed_temp = true;
											}
										}
									}
								}
								else if (1)//当前聚类为曲面
								{
									//isProcessed_temp = true;//直接加入
								}

								if (isProcessed_temp)
								{
									cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
									isProcessed[pointIdxRadiusSearch[j]] = true;
									mean_curvature += pc1j;
									mean_curvature_b += pc2j;
									mean_normal_x += nxj;
									mean_normal_y += nyj;
									mean_normal_z += nzj;
									mean_x += xj;
									mean_y += yj;
									mean_z += zj;
								}
							}
						}
					}
					//阶段结束，处理特征向量
					//Eigen::Vector3d mean_normal(mean_normal_x, mean_normal_y, mean_normal_z);
					//mean_normal.normalize();
					Feature_cluster(0, 0) = cloud_cluster.size();
					Feature_cluster(0, 1) = mean_curvature / cloud_cluster.size();//平均主曲率
					Feature_cluster(0, 2) = mean_curvature_b / cloud_cluster.size();//平均副曲率
					Feature_cluster(0, 3) = mean_normal_x / cloud_cluster.size();//平均法向量x
					Feature_cluster(0, 4) = mean_normal_y / cloud_cluster.size();//平均法向量y
					Feature_cluster(0, 5) = mean_normal_z / cloud_cluster.size();//平均法向量z
					Feature_cluster(0, 6) = mean_x / cloud_cluster.size();//平均x
					Feature_cluster(0, 7) = mean_y / cloud_cluster.size();//平均y
					Feature_cluster(0, 8) = mean_z / cloud_cluster.size();//平均z
					k_tempt++;
				}

				if (cloud_cluster.size() >= minSize && cloud_cluster.size() <= maxSize)
				{
					cloud_segments.push_back(cloud_cluster);
					Feature.push_back(Feature_cluster);
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_CEC_distance(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, double minSize, double maxSize)
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;//采集的结果
		std::vector<bool> isProcessed(input.size(), false);
		double distance_square_theshold = pow(threshold, 2);
		for (size_t i = 0; i < input.size(); i++)
		{
			if (isProcessed[i] == false)
			{
				//构建一个新的聚类
				pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
				cloud_cluster.push_back(input[i]);
				std::vector<int> pointIdxRadiusSearch;
				std::vector<float> pointRadiusSquaredDistance;
				//创建一个特征向量
				Eigen::MatrixXd Feature_cluster(1, 9);
				//流程：1找到聚类中还没处理的点 2将符合条件的点加入到聚类中，循环，直到所有点都处理完了
				//设计一个平均曲率，用来限制曲率过大的偏离
				double mean_curvature = 0;//平均主曲率
				double mean_curvature_b = 0;//平均副曲率
				double mean_normal_x = 0;//平均法向量x
				double mean_normal_y = 0;//平均法向量y
				double mean_normal_z = 0;//平均法向量z
				double mean_x = 0;//平均x
				double mean_y = 0;//平均y
				double mean_z = 0;//平均z
				int k_tempt = 0;
				while (1)
				{
					if (k_tempt >= cloud_cluster.size())
					{
						break;
					}
					if (kdtree.radiusSearch(cloud_cluster[k_tempt], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
					{
						//聚类判断函数
						for (size_t j = 0; j < pointIdxRadiusSearch.size(); ++j)
						{
							if (isProcessed[pointIdxRadiusSearch[j]] == false)
							{
								//计算判断的值
								double x0 = input[pointIdxRadiusSearch[0]].x;
								double y0 = input[pointIdxRadiusSearch[0]].y;
								double z0 = input[pointIdxRadiusSearch[0]].z;
								double xj = input[pointIdxRadiusSearch[j]].x;
								double yj = input[pointIdxRadiusSearch[j]].y;
								double zj = input[pointIdxRadiusSearch[j]].z;
								bool isProcessed_temp = false;//初始值为否
								//if (pow(x0- xj,2) + pow(y0 - yj,2)+ pow(z0 - zj,2) < distance_square_theshold)
								{
									isProcessed_temp = true;
								}
								if (isProcessed_temp)
								{
									cloud_cluster.push_back(input[pointIdxRadiusSearch[j]]);
									isProcessed[pointIdxRadiusSearch[j]] = true;
								}
							}
						}
					}
					//阶段结束，处理特征向量
					k_tempt++;
				}

				if (cloud_cluster.size() >= minSize && cloud_cluster.size() <= maxSize)
				{
					cloud_segments.push_back(cloud_cluster);
				}
			}
		}
		//按照点云数量排序，从大到小
		for (size_t i = 0; i < cloud_segments.size(); i++)
		{
			for (size_t j = i + 1; j < cloud_segments.size(); j++)
			{
				if (cloud_segments[i].size() < cloud_segments[j].size())
				{
					pcl::PointCloud<pcl::PointXYZ> temp = cloud_segments[i];
					cloud_segments[i] = cloud_segments[j];
					cloud_segments[j] = temp;
				}
			}
		}
		return cloud_segments;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getClusterPoints_RG(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normals, double smoothThreshold, double curevatureThreshold, double minSize, double maxSize)
	{
		//region growing
		pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize(minSize);
		reg.setMaxClusterSize(maxSize);
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30);
		reg.setInputCloud(input.makeShared());
		//reg.setIndices (indices);
		reg.setInputNormals(normals.makeShared());
		reg.setSmoothnessThreshold(smoothThreshold / 180.0 * M_PI);
		reg.setCurvatureThreshold(curevatureThreshold);
		std::vector <pcl::PointIndices> clusters;
		reg.extract(clusters);
		std::vector<pcl::PointCloud<pcl::PointXYZ>> cloud_segments;
		for (size_t i = 0; i < clusters.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZ> cloud_cluster;
			for (size_t j = 0; j < clusters[i].indices.size(); j++)
			{
				cloud_cluster.push_back(input[clusters[i].indices[j]]);
			}
			cloud_segments.push_back(cloud_cluster);
		}
		return cloud_segments;
	}

	bool getRegistration_ICP(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& input_target, double threshold, double maxIteration, double& score, Eigen::Matrix4d& transfromMatrix)
	{
		//配准
		 // 创建IterativeClosestPoint的实例
		// setInputSource将cloud_in作为输入点云
		// setInputTarget将平移后的cloud_out作为目标点云
		pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setTransformationEpsilon(1e-6);
		icp.setMaxCorrespondenceDistance(2 * threshold);
		icp.setInputSource(input.makeShared());
		icp.setInputTarget(input_target.makeShared()); //cloud_segments[i]
		icp.setMaximumIterations(maxIteration);
		pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
		icp.align(*Final);
		score = icp.getFitnessScore();
		transfromMatrix = icp.getFinalTransformation().cast<double>();
		return 1;
	}
	bool getCirclePoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::PointCloud<pcl::PointXYZ>& circle_points, pcl::PointCloud<pcl::PointXYZ>& noncircle_points)
	{
		//pcl ransac 方法提取点云圆
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setNormalDistanceWeight(0.1);// 设置表面法线权重系数
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CIRCLE3D);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(2 * threshold);//距离阈值；自适应阈值，为采样间隔的2倍
		seg.setRadiusLimits(0, 1E3);
		seg.setInputCloud(cloud);
		seg.setInputNormals(normal.makeShared());
		seg.segment(*inliers, *coefficients);
		//提取平面点云
		pcl::PointCloud<pcl::PointXYZ> cloud_circle;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(circle_points);
		extract.setNegative(true);
		extract.filter(noncircle_points);
		return bool(circle_points.size());
	}
	bool getCirclePoints(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>& circle_points, pcl::PointCloud<pcl::PointXYZ>& noncircle_points)
	{
		//pcl ransac 方法提取点云圆
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setNormalDistanceWeight(0.1);// 设置表面法线权重系数
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CIRCLE3D);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(2 * threshold);//距离阈值；自适应阈值，为采样间隔的2倍
		seg.setRadiusLimits(0, 1E3);
		seg.setInputCloud(cloud);
		seg.setInputNormals(normal.makeShared());
		seg.segment(*inliers, *coefficients);
		//提取平面点云
		pcl::PointCloud<pcl::PointXYZ> cloud_circle;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(circle_points);
		extract.setNegative(true);
		extract.filter(noncircle_points);
		return bool(circle_points.size());
	}
	pcl::PointCloud<pcl::PointXYZ> getPlanePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold)
	{
		//pcl ransac 方法提取点云平面
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(2 * threshold);//距离阈值；自适应阈值，为采样间隔的2倍
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		//提取平面点云
		pcl::PointCloud<pcl::PointXYZ> cloud_plane;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(cloud_plane);
		return cloud_plane;
	}
	bool getPlanePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points)
	{
		plane_points.clear();
		nonplane_points.clear();
		//pcl ransac 方法提取点云平面
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(2 * threshold);//距离阈值；自适应阈值，为采样间隔的2倍
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		//提取平面点云
		pcl::PointCloud<pcl::PointXYZ> cloud_plane;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(plane_points);
		extract.setNegative(true);
		extract.filter(nonplane_points);
		return bool(plane_points.size());
	}
	bool getPlanePoints(const pcl::PointCloud<pcl::PointXYZ>& input, double threshold, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points, pcl::ModelCoefficients& coefficients)
	{
		//pcl ransac 方法提取点云平面
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(2 * threshold);//距离阈值；自适应阈值，为采样间隔的2倍
		seg.setInputCloud(cloud);
		seg.segment(*inliers, coefficients);
		//提取平面点云
		pcl::PointCloud<pcl::PointXYZ> cloud_plane;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(plane_points);
		extract.setNegative(true);
		extract.filter(nonplane_points);
		return bool(plane_points.size());
	}

	bool getCylinderPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points)
	{
		//pcl ransac 方法提取点云平面
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input;
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setNormalDistanceWeight(0.1);           // 设置表面法线权重系数
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(2 * threshold);//距离阈值；自适应阈值，为采样间隔的2倍
		seg.setRadiusLimits(0, 1000);
		seg.setInputCloud(cloud);
		seg.setInputNormals(normal.makeShared());
		seg.segment(*inliers, *coefficients);
		//提取平面点云
		pcl::PointCloud<pcl::PointXYZ> cloud_plane;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(plane_points);
		extract.setNegative(true);
		extract.filter(nonplane_points);
		return bool(plane_points.size());
	}
	bool getCylinderPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::Normal>& normal, double threshold, pcl::ModelCoefficients::Ptr coefficients, pcl::PointCloud<pcl::PointXYZ>& plane_points, pcl::PointCloud<pcl::PointXYZ>& nonplane_points, double minRadius, double maxRadius)
	{
		//pcl ransac 方法提取点云平面
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		*cloud = input;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		seg.setNormalDistanceWeight(0.1);           // 设置表面法线权重系数
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CYLINDER);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(2 * threshold);//距离阈值；自适应阈值，为采样间隔的2倍
		seg.setRadiusLimits(minRadius, maxRadius);
		seg.setInputCloud(cloud);
		seg.setInputNormals(normal.makeShared());
		seg.segment(*inliers, *coefficients);
		//提取平面点云
		pcl::PointCloud<pcl::PointXYZ> cloud_plane;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(plane_points);
		extract.setNegative(true);
		extract.filter(nonplane_points);
		return bool(plane_points.size());
	}
	pcl::PointCloud<pcl::PointXYZ> getConvexHullPoints(const pcl::PointCloud<pcl::PointXYZ>& input)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud_hull;
		pcl::ConvexHull<pcl::PointXYZ> chull;
		chull.setInputCloud(input.makeShared());
		chull.reconstruct(cloud_hull);
		return cloud_hull;
	}
	Eigen::Vector3d getPointCoordination(const Eigen::Vector3d& point, const Eigen::Vector3d& x, const Eigen::Vector3d& y, const Eigen::Vector3d& z)
	{
		//x,y,z 为轴
		//求解 p = a*x+b*y + c*z	
		Eigen::Matrix3d A;
		A.col(0) = x;
		A.col(1) = y;
		A.col(2) = z;
		Eigen::Vector3d b = point;
		Eigen::Vector3d V = A.colPivHouseholderQr().solve(b);
		return V;
	}
	bool isInTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& x, const Eigen::Vector3d& y, const Eigen::Vector3d& z)
	{
		//x,y,z 为轴
		//求解 p = a*x+b*y + c*z	
		Eigen::Matrix3d A;
		A.col(0) = x;
		A.col(1) = y;
		A.col(2) = z;
		Eigen::Vector3d b = point;
		Eigen::Vector3d V = A.colPivHouseholderQr().solve(b);
		if (V[0] > 0 && V[1] > 0 && V[2] > 0 && V[0] + V[1] + V[2] < 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	bool isInBox(const Eigen::Vector3d& point, const Eigen::Vector3d& x, const Eigen::Vector3d& y, const Eigen::Vector3d& z)
	{
		//x,y,z 为轴
		//求解 p = a*x+b*y + c*z	
		Eigen::Matrix3d A;
		A.col(0) = x;
		A.col(1) = y;
		A.col(2) = z;
		Eigen::Vector3d b = point;
		Eigen::Vector3d V = A.colPivHouseholderQr().solve(b);
		if (V[0] > 0 && V[1] > 0 && V[2] > 0 && V[0] < 1 && V[1] < 1 && V[2] < 1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	bool isInBox(pcl::PointXYZ& inputPoint, Eigen::MatrixXd& output)
	{
		Eigen::Vector3d p0 = output.row(0);
		Eigen::Vector3d p1 = output.row(1);
		Eigen::Vector3d p2 = output.row(2);
		Eigen::Vector3d p3 = output.row(3);
		Eigen::Vector3d p4 = output.row(4);
		Eigen::Vector3d p5 = output.row(5);
		Eigen::Vector3d p6 = output.row(6);
		Eigen::Vector3d p7 = output.row(7);
		//判断输入点云是否在box内
		//三个轴
		Eigen::Vector3d x_axis1, y_axis1, z_axis1, x_axis2, y_axis2, z_axis2, x_axis3, y_axis3, z_axis3, x_axis4, y_axis4, z_axis4;
		x_axis1 = (p3 - p7);
		y_axis1 = (p5 - p7);
		z_axis1 = (p6 - p7);
		x_axis2 = (p5 - p1);
		y_axis2 = (p3 - p1);
		z_axis2 = (p0 - p1);
		x_axis3 = (p6 - p4);
		y_axis3 = (p0 - p4);
		z_axis3 = (p5 - p4);
		x_axis4 = (p6 - p2);
		y_axis4 = (p0 - p2);
		z_axis4 = (p3 - p2);

		int count = 0;
		int count1 = 0;
		int count2 = 0;
		int count3 = 0;
		int count4 = 0;

		//pcl::PointCloud<pcl::PointXYZ> point1_cloud, point2_cloud;
		Eigen::Vector3d point(inputPoint.x, inputPoint.y, inputPoint.z);
		Eigen::Vector3d point1 = point - p7;
		Eigen::Vector3d point2 = point - p1;
		Eigen::Vector3d point3 = point - p4;
		Eigen::Vector3d point4 = point - p2;


		if (isInBox(point1, x_axis1, y_axis1, z_axis1) &&
			isInBox(point2, x_axis2, y_axis2, z_axis2) &&
			isInBox(point3, x_axis3, y_axis3, z_axis3) &&
			isInBox(point4, x_axis4, y_axis4, z_axis4))
		{
			return true;
		}
		else
		{
			return false;

		}
	}
	bool isInCylinder(const Eigen::Vector3d& point, const Eigen::Vector3d& center, const Eigen::Vector3d& normal, double radius, double height)
	{
		//计算点到直线的距禝
		double distance = getDistance_pointToLine(point, center, normal);
		//计算点到圆心的距禝
		double distance_center = (point - center).norm();
		if (distance < radius && distance_center < height)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	Eigen::MatrixXd getBoxPoints(const pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector3d normal, Eigen::Vector3d binormal)
	{
		Eigen::MatrixXd box_points(8, 3);
		Eigen::Vector3d center;
		Eigen::Vector3d x_axis;
		Eigen::Vector3d y_axis;
		Eigen::Vector3d z_axis;
		//计算x轴
		x_axis = normal;
		x_axis.normalize();
		//计算y轴
		y_axis = binormal;
		y_axis.normalize();
		//计算z轴
		z_axis = normal.cross(binormal);
		z_axis.normalize();
		double x_max = 0, x_min = 0, y_max = 0, y_min = 0, z_max = 0, z_min = 0;
		//计算中心点
		center = Eigen::Vector3d::Zero();
		for (int i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point(input.points[i].x, input.points[i].y, input.points[i].z);
			center += point;
		}
		center = center / input.size();
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point(input.points[i].x, input.points[i].y, input.points[i].z);
			point -= center;
			double x = point.dot(x_axis);
			double y = point.dot(y_axis);
			double z = point.dot(z_axis);
			if (x > 0 && x > x_max)
			{
				x_max = x;
			}
			if (x < 0 && x < x_min)
			{
				x_min = x;
			}
			if (y > 0 && y > y_max)
			{
				y_max = y;
			}
			if (y < 0 && y < y_min)
			{
				y_min = y;
			}
			if (z > 0 && z > z_max)
			{
				z_max = z;
			}
			if (z < 0 && z < z_min)
			{
				z_min = z;
			}
		}
		//计算box的8个点
		box_points.row(0) = center + x_max * x_axis + y_max * y_axis + z_max * z_axis;//(1,1,1)
		box_points.row(1) = center + x_max * x_axis + y_max * y_axis + z_min * z_axis;//(1,1,0)
		box_points.row(2) = center + x_max * x_axis + y_min * y_axis + z_max * z_axis;//(1,0,1)
		box_points.row(3) = center + x_max * x_axis + y_min * y_axis + z_min * z_axis;//(1,0,0)
		box_points.row(4) = center + x_min * x_axis + y_max * y_axis + z_max * z_axis;//(0,1,1)
		box_points.row(5) = center + x_min * x_axis + y_max * y_axis + z_min * z_axis;//(0,1,0)
		box_points.row(6) = center + x_min * x_axis + y_min * y_axis + z_max * z_axis;//(0,0,1)
		box_points.row(7) = center + x_min * x_axis + y_min * y_axis + z_min * z_axis;//(0,0,0)
		return box_points.transpose();
	}
	Eigen::MatrixXd getBoxPoints(const pcl::PointCloud<pcl::PointXYZ>& input)
	{
		//先计算点云法向量
				//将点云拟合为一个平面
				//计算平面的法向量
				//计算平面的中心
		Eigen::Vector4d plan_para = getPlaneEquation(input);
		Eigen::Vector3d normal = Eigen::Vector3d(plan_para[0], plan_para[1], plan_para[2]);
		normal.normalize();
		//计算平面的中心
		Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
		for (size_t i = 0; i < input.size(); i++)
		{
			center[0] += input.points[i].x;
			center[1] += input.points[i].y;
			center[2] += input.points[i].z;
		}
		center = center / input.size();
		//将所有点云投影到平面上,计算最远点
		std::vector<double> distance_vectorz, distance_vectorz_sort, distance_vectorxy, distance_vectorxy_sort;
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point(input.points[i].x, input.points[i].y, input.points[i].z);
			double distance_temptz = getPointParameter(point, center, normal);
			double distance_temptxy = getDistance_pointToLine(point, center, normal);
			distance_vectorz.push_back(distance_temptz);
			distance_vectorxy.push_back(distance_temptxy);
		}
		distance_vectorz_sort = sortVector(distance_vectorz);
		distance_vectorxy_sort = sortVector(distance_vectorxy);
		double z_min, z_max, x_min, x_max;
		z_min = distance_vectorz_sort[0];
		z_max = distance_vectorz_sort[distance_vectorz_sort.size() - 1];
		x_min = distance_vectorxy_sort[0];
		x_max = distance_vectorxy_sort[distance_vectorxy_sort.size() - 1];
		double indexx_min, indexx_max, indexz_min, indexz_max;
		indexx_min = findVector(x_min, distance_vectorxy);
		indexx_max = findVector(x_max, distance_vectorxy);
		indexz_min = findVector(z_min, distance_vectorz);
		indexz_max = findVector(z_max, distance_vectorz);
		Eigen::Vector3d far_point_x_min = Eigen::Vector3d(input.points[indexx_min].x, input.points[indexx_min].y, input.points[indexx_min].z);
		Eigen::Vector3d far_point_x_max = Eigen::Vector3d(input.points[indexx_max].x, input.points[indexx_max].y, input.points[indexx_max].z);
		Eigen::Vector3d far_point_x_min_project, far_point_x_max_project;
		Eigen::Vector4d plane_para = getPlaneEquation(center, normal);
		getProjectionPoint_pointToPlane(far_point_x_min, plane_para, far_point_x_min_project);
		getProjectionPoint_pointToPlane(far_point_x_max, plane_para, far_point_x_max_project);
		//计算x,y,z轴
		Eigen::Vector3d x_axis = (far_point_x_max_project - center).normalized();
		Eigen::Vector3d z_axis = normal;
		Eigen::Vector3d y_axis = z_axis.cross(x_axis);
		//计算x轴的最远点
		std::vector<double> distance_vectorx_axis, distance_vectorx_axis_sort;
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point(input.points[i].x, input.points[i].y, input.points[i].z);
			double distance_tempt = getPointParameter(point, center, x_axis);
			distance_vectorx_axis.push_back(distance_tempt);
		}
		distance_vectorx_axis_sort = sortVector(distance_vectorx_axis);
		x_min = distance_vectorx_axis_sort[0];
		x_max = distance_vectorx_axis_sort[distance_vectorx_axis_sort.size() - 1];
		indexx_min = findVector(x_min, distance_vectorx_axis);
		indexx_max = findVector(x_max, distance_vectorx_axis);
		far_point_x_min = Eigen::Vector3d(input.points[indexx_min].x, input.points[indexx_min].y, input.points[indexx_min].z);
		far_point_x_max = Eigen::Vector3d(input.points[indexx_max].x, input.points[indexx_max].y, input.points[indexx_max].z);
		getProjectionPoint_pointToPlane(far_point_x_min, plane_para, far_point_x_min_project);
		getProjectionPoint_pointToPlane(far_point_x_max, plane_para, far_point_x_max_project);



		//计算y轴的最远点
		std::vector<double> distance_vectory, distance_vectory_sort;
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point(input.points[i].x, input.points[i].y, input.points[i].z);
			double distance_tempt = getPointParameter(point, center, y_axis);
			distance_vectory.push_back(distance_tempt);
		}


		distance_vectory_sort = sortVector(distance_vectory);
		double y_max, y_min;
		y_min = distance_vectory_sort[0];
		y_max = distance_vectory_sort[distance_vectory_sort.size() - 1];
		int indexy_min, indexy_max;
		indexy_min = findVector(y_min, distance_vectory);
		indexy_max = findVector(y_max, distance_vectory);
		Eigen::Vector3d far_point_y_min = Eigen::Vector3d(input.points[indexy_min].x, input.points[indexy_min].y, input.points[indexy_min].z);
		Eigen::Vector3d far_point_y_max = Eigen::Vector3d(input.points[indexy_max].x, input.points[indexy_max].y, input.points[indexy_max].z);
		Eigen::Vector3d far_point_y_min_project, far_point_y_max_project;
		getProjectionPoint_pointToPlane(far_point_y_min, plane_para, far_point_y_min_project);
		getProjectionPoint_pointToPlane(far_point_y_max, plane_para, far_point_y_max_project);
		//计算y轴
		y_axis = (far_point_y_max_project - far_point_y_min_project).normalized();
		//计算x轴和y轴交点
		center = getIntersectionPoint(far_point_y_max_project, y_axis, far_point_x_max_project, x_axis);


		//计算box的8个点
		Eigen::MatrixXd box_points(8, 3);
		//box_points.row(0) = center + x_max * x_axis + 0*y_max * y_axis + z_max * z_axis;//(1,1,1)
		//box_points.row(1) = center + x_max * x_axis + 0*y_max * y_axis - z_min * z_axis;//(1,1,0)
		//box_points.row(2) = center + 0*x_max * x_axis - y_min * y_axis + z_max * z_axis;//(1,0,1)
		//box_points.row(3) = center + 0*x_max * x_axis - y_min * y_axis - z_min * z_axis;//(1,0,0)
		//box_points.row(4) = center - 0*x_min * x_axis + y_max * y_axis + z_max * z_axis;//(0,1,1)
		//box_points.row(5) = center - 0*x_min * x_axis + y_max * y_axis - z_min * z_axis;//(0,1,0)
		//box_points.row(6) = center - x_min * x_axis - 0* y_min * y_axis + z_max * z_axis;//(0,0,1)
		//box_points.row(7) = center - x_min * x_axis - 0* y_min * y_axis - z_min * z_axis;//(0,0,0)
		Eigen::Vector3d p0 = far_point_y_max_project + z_max * z_axis;//(1,1,1)
		Eigen::Vector3d p1 = far_point_y_max_project + z_min * z_axis;//(1,1,0)
		Eigen::Vector3d p2 = far_point_x_max_project + z_max * z_axis;//(1,0,1)
		Eigen::Vector3d p3 = far_point_x_max_project + z_min * z_axis;//(1,0,0)
		Eigen::Vector3d p4 = far_point_x_min_project + z_max * z_axis;//(0,1,1)
		Eigen::Vector3d p5 = far_point_x_min_project + z_min * z_axis;//(0,1,0)	
		Eigen::Vector3d p6 = far_point_y_min_project + z_max * z_axis;//(0,0,1)
		Eigen::Vector3d p7 = far_point_y_min_project + z_min * z_axis;//(0,0,0)

		box_points.row(0) = p0;
		box_points.row(1) = p1;
		box_points.row(2) = p2;
		box_points.row(3) = p3;
		box_points.row(4) = p4;
		box_points.row(5) = p5;
		box_points.row(6) = p6;
		box_points.row(7) = p7;

		return box_points;
	}
	bool getBoxPoints(const pcl::PointCloud<pcl::PointXYZ>& input, Eigen::MatrixXd& output, double& score)
	{
		//score 为覆盖率
		output = getBoxPoints(input);
		Eigen::Vector3d p0 = output.row(0);
		Eigen::Vector3d p1 = output.row(1);
		Eigen::Vector3d p2 = output.row(2);
		Eigen::Vector3d p3 = output.row(3);
		Eigen::Vector3d p4 = output.row(4);
		Eigen::Vector3d p5 = output.row(5);
		Eigen::Vector3d p6 = output.row(6);
		Eigen::Vector3d p7 = output.row(7);
		//判断输入点云是否在box内
		//三个轴
		Eigen::Vector3d x_axis1, y_axis1, z_axis1, x_axis2, y_axis2, z_axis2, x_axis3, y_axis3, z_axis3, x_axis4, y_axis4, z_axis4;
		x_axis1 = (p3 - p7);
		y_axis1 = (p5 - p7);
		z_axis1 = (p6 - p7);
		x_axis2 = (p5 - p1);
		y_axis2 = (p3 - p1);
		z_axis2 = (p0 - p1);
		x_axis3 = (p6 - p4);
		y_axis3 = (p0 - p4);
		z_axis3 = (p5 - p4);
		x_axis4 = (p6 - p2);
		y_axis4 = (p0 - p2);
		z_axis4 = (p3 - p2);

		int count = 0;
		int count1 = 0;
		int count2 = 0;
		int count3 = 0;
		int count4 = 0;

		//pcl::PointCloud<pcl::PointXYZ> point1_cloud, point2_cloud;
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point(input.points[i].x, input.points[i].y, input.points[i].z);
			Eigen::Vector3d point1 = point - p7;
			Eigen::Vector3d point2 = point - p1;
			Eigen::Vector3d point3 = point - p4;
			Eigen::Vector3d point4 = point - p2;
			//if (isInBox(point1, x_axis1, y_axis1, z_axis1))
			//{
			//	point1_cloud.push_back(input[i]);
			//	count1++;
			//}
			//if (isInBox(point2, x_axis2, y_axis2, z_axis2))
			//{
			//	point2_cloud.push_back(input[i]);
			//	count2++;
			//}


			if (isInBox(point1, x_axis1, y_axis1, z_axis1) &&
				isInBox(point2, x_axis2, y_axis2, z_axis2) &&
				isInBox(point3, x_axis3, y_axis3, z_axis3) &&
				isInBox(point4, x_axis4, y_axis4, z_axis4))
			{
				count++;
				//point1_cloud.push_back(input.points[i]);
			}
		}
		//showPointCloud(input, point1_cloud, point2_cloud,  "point1_cloud");
		score = double(count) / double(input.size());
		return 1;
	}
	bool getTangencyPoint(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::Vector4d planePara, std::vector<Eigen::Vector3d>& tangencyPoints, double& minDistance, double& maxDistance)
	{
		//计算点云距离平面的最大值和最小值的点
		maxDistance = -1E10;
		minDistance = 1E10;
		Eigen::Vector3d maxDistancePoint, minDistancePoint;
		if (input.size() == 0)
		{
			return false;
		}
		for (size_t i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point = Eigen::Vector3d(input[i].x, input[i].y, input[i].z);
			double distance = HG::getDistance_pointToPlane(point, planePara);
			if (distance < minDistance)
			{
				minDistance = distance;
				minDistancePoint = point;
			}
			if (distance > maxDistance)
			{
				maxDistance = distance;
				maxDistancePoint = point;
			}
		}
		tangencyPoints.clear();
		tangencyPoints.push_back(minDistancePoint);
		tangencyPoints.push_back(maxDistancePoint);
		return true;
	}
	bool getOuterPoint(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, double interval)
	{
		//计算外围点
		//计算中心
		output.clear();
		std::vector<double> minDistancePoints, maxDistancePoints;
		std::vector<std::vector<Eigen::Vector3d>> points;
		double interval_angle = 64;
		for (size_t i = 0; i < interval_angle; i++)
		{
			double theta = M_PI / interval_angle * double(i);
			Eigen::Vector4d planePara = Eigen::Vector4d(sin(theta), cos(theta), 0, 0);
			std::vector<Eigen::Vector3d> points_tempt;
			double minDistance, maxDistance;
			if (getTangencyPoint(input, planePara, points_tempt, minDistance, maxDistance))
			{
				minDistancePoints.push_back(minDistance);
				maxDistancePoints.push_back(maxDistance);
				pcl::PointCloud<pcl::PointXYZ> pointsPCL_tempt = HG::getPCLPointXYZSet(points_tempt);
				output.insert(output.end(), pointsPCL_tempt.begin(), pointsPCL_tempt.end());
			}
		}
		//去掉output中的重复点
		pcl::PointCloud<pcl::PointXYZ> output_tempt;
		for (size_t i = 0; i < output.size(); i++)
		{
			bool isExist = false;
			for (size_t j = 0; j < output_tempt.size(); j++)
			{
				Eigen::Vector3d point1 = Eigen::Vector3d(output[i].x, output[i].y, output[i].z);
				Eigen::Vector3d point2 = Eigen::Vector3d(output_tempt[j].x, output_tempt[j].y, output_tempt[j].z);
				if ((point1 - point2).norm() < interval * 2)
				{
					isExist = true;
					break;
				}
			}
			if (!isExist)
			{
				output_tempt.push_back(output[i]);
			}
		}
		output = output_tempt;

		return true;

	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> getRoundness(std::vector<pcl::PointCloud<pcl::PointXYZ>>& input, Eigen::Vector3d axis)
	{
		//首先将点云投影到平面上，然后计算外接圆，然后计算每个点到圆的距离，然后计算方差
		std::vector<pcl::PointCloud<pcl::PointXYZ>> roundness;
		pcl::PointCloud<pcl::PointXYZ> outPoints;
		for (size_t i = 0; i < input.size(); i++)
		{
			getOuterPoint(input[i], outPoints);
			roundness.push_back(outPoints);
		}
		return roundness;
	}
	bool getInnerPoint(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, Eigen::Vector3d Center, double interval)
	{
		output.clear();
		//计算内圈点
		//计算输入点云的中心,并中心化
		pcl::PointCloud<pcl::PointXYZ> inputCenteralized;
		pcl::PointXYZ inputCenter = getPCLPointXYZ(Center);
		inputCenteralized = Centralized(input, inputCenter);
		std::vector<std::vector<Eigen::Vector3d>> points;
		double interval_angle = 64;
		for (size_t i = 0; i < interval_angle; i++)
		{
			//将角度分为64份
			double theta = M_PI / interval_angle * double(i);
			Eigen::Vector4d planePara = Eigen::Vector4d(sin(theta), cos(theta), 0, 0);
			//计算切面
			pcl::PointCloud<pcl::PointXYZ> pointsSliceTemp;
			getPointsByPlaneDistance(inputCenteralized, planePara, 2 * interval, pointsSliceTemp);
			//计算切面的距离原点正向和反向的最大值和最小值
			double maxPositive = -1E10, minPositive = 1E10, maxNegative = -1E10, minNegative = 1E10;
			pcl::PointXYZ maxPositivePoint, minPositivePoint, maxNegativePoint, minNegativePoint;
			bool isExistMaxPositive = false, isExistMinPositive = false, isExistMaxNegative = false, isExistMinNegative = false;
			for (size_t j = 0; j < pointsSliceTemp.size(); j++)
			{
				Eigen::Vector3d point = Eigen::Vector3d(pointsSliceTemp[j].x, pointsSliceTemp[j].y, pointsSliceTemp[j].z);
				double distance = point.dot(Eigen::Vector3d(planePara[1], -planePara[0], 0)) + planePara[3];
				if (distance > 0)
				{
					if (distance > maxPositive)
					{
						maxPositive = distance;
						isExistMaxPositive = true;
						maxPositivePoint = pointsSliceTemp[j];
					}
					if (distance < minPositive)
					{
						minPositive = distance;
						isExistMinPositive = true;
						minPositivePoint = pointsSliceTemp[j];
					}
				}
				else
				{
					if (distance > maxNegative)
					{
						maxNegative = distance;
						isExistMaxNegative = true;
						maxNegativePoint = pointsSliceTemp[j];
					}
					if (distance < minNegative)
					{
						minNegative = distance;
						isExistMinNegative = true;
						minNegativePoint = pointsSliceTemp[j];
					}
				}
			}
			if (isExistMinPositive)
			{
				output.push_back(minPositivePoint);
			}
			if (isExistMaxNegative)
			{
				output.push_back(maxNegativePoint);
			}
		}
		//去掉output中的重复点
		pcl::PointCloud<pcl::PointXYZ> output_tempt;
		for (size_t i = 0; i < output.size(); i++)
		{
			bool isExist = false;
			for (size_t j = 0; j < output_tempt.size(); j++)
			{
				Eigen::Vector3d point1 = Eigen::Vector3d(output[i].x, output[i].y, output[i].z);
				Eigen::Vector3d point2 = Eigen::Vector3d(output_tempt[j].x, output_tempt[j].y, output_tempt[j].z);
				if ((point1 - point2).norm() < interval * 2)
				{
					isExist = true;
					break;
				}
			}
			if (!isExist)
			{
				output_tempt.push_back(output[i]);
			}
		}
		pcl::PointCloud<pcl::PointXYZ> output_tempt1;
		output_tempt1 = DeCentralized(output_tempt, inputCenter);
		output = output_tempt1;

		return true;
	}
	bool getInnerPoint(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& output, double interval)
	{
		pcl::PointXYZ inputCenter = getCenter(input);
		return getInnerPoint(input, output, Eigen::Vector3d(inputCenter.x, inputCenter.y, inputCenter.z), interval);
	}
	bool getBoundaryPoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& boundPoints_is, pcl::PointCloud<pcl::PointXYZ>& boundPoints_non, double interval)
	{
		//计算法向量
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_line;
		ne_line.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		ne_line.setKSearch(16);
		ne_line.setInputCloud(input.makeShared());
		ne_line.compute(*cloud_normals);
		//计算边界
		pcl::PointCloud<pcl::Boundary> boundaries;
		pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> est;
		est.setInputCloud(input.makeShared());
		est.setInputNormals(cloud_normals);
		//est.setRadiusSearch(2*8);
		est.setRadiusSearch(2 * interval);
		est.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		est.compute(boundaries);
		//  pcl::PointCloud<pcl::PointXYZ> boundPoints;
		int countBoundaries = 0;
		for (int i = 0; i < input.size(); i++) {
			uint8_t x = (boundaries.points[i].boundary_point);
			int a = static_cast<int>(x); //该函数的功能是强制类型转换
			if (a == 1)
			{
				boundPoints_is.push_back(input.points[i]);
				countBoundaries++;
			}
			else
				boundPoints_non.push_back(input.points[i]);

		}
		return bool(boundPoints_is.size());
	}
	bool getPointsByPlaneDistance(const pcl::PointCloud<pcl::PointXYZ>& input, const Eigen::Vector4d& plane_para, double distance, pcl::PointCloud<pcl::PointXYZ>& points)
	{
		points.clear();
		for (int i = 0; i < input.size(); i++)
		{
			double x = abs(plane_para[0] * input.points[i].x + plane_para[1] * input.points[i].y + plane_para[2] * input.points[i].z + plane_para[3]);
			if (x < distance)
			{
				points.push_back(input.points[i]);
			}
		}
		return bool(points.size());
	}
	bool getPointsByLineDistance(const pcl::PointCloud<pcl::PointXYZ>& input, const Eigen::Vector3d& line_pole, const Eigen::Vector3d& line_direction, double distance, pcl::PointCloud<pcl::PointXYZ>& points)
	{
		for (int i = 0; i < input.size(); i++)
		{
			Eigen::Vector3d point(input.points[i].x, input.points[i].y, input.points[i].z);
			double x = getDistance_pointToLine(point, line_pole, line_direction);
			if (x < distance)
			{
				points.push_back(input.points[i]);
			}
		}
		return bool(points.size());
	}
	bool getPointCloudByPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::PointXYZ>& sample_points, double threshold, pcl::PointCloud<pcl::PointXYZ>& output)
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(sample_points.makeShared());
		for (size_t i = 0; i < input.size(); i++)
		{
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			if (kdtree.radiusSearch(input[i], 2 * threshold, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
			{
				output.push_back(input[i]);
			}
		}
		return bool(output.size());

	}
	bool getLinePoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& line_points, pcl::PointCloud<pcl::PointXYZ>& nonline_points, double interval)
	{

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		*cloud = input;
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);//线点云索引
		pcl::ModelCoefficients::Ptr coefficients_line(new pcl::ModelCoefficients);//模型系数
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_line(new pcl::PointCloud<pcl::Normal>());
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_line;
		ne_line.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		ne_line.setKSearch(16);
		ne_line.setInputCloud(cloud);
		ne_line.compute(*cloud_normals_line);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cylinder(new pcl::search::KdTree<pcl::PointXYZ>());
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE);//设置模型类型
		//seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(2 * interval);//最大允许偏离误差
		//seg.setRadiusLimits(0, 6);
		seg.setInputCloud(cloud);
		seg.setInputNormals(cloud_normals_line);
		seg.segment(*inliers_line, *coefficients_line);
		// Extract the cylinder inliers from the input cloud

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(inliers_line);
		extract.setNegative(false);
		extract.filter(line_points);
		extract.setNegative(true);
		extract.filter(nonline_points);

		if (line_points.size() <= 1)
		{
			return 0;
		}
		//拟合直线，按顺序输出所有点在直线上投影
		Eigen::Vector3d line_pole, line_direction;
		std::vector<Eigen::Vector3d> line_points_vv = getVector3dSet(line_points);
		getLineEquation(line_points_vv, line_pole, line_direction);
		std::vector <Eigen::Vector2d> parameters;
		for (int i = 0; i < line_points_vv.size(); i++)
		{
			parameters.push_back(Eigen::Vector2d(getPointParameter(line_points_vv[i], line_pole, line_direction), i));
		}

		//按照参数值排序点云
		//找到最大最小值
		pcl::PointCloud <pcl::PointXYZ> line_points_temp;
		for (int i = 0; i < parameters.size(); i++)
		{
			for (size_t j = i + 1; j < parameters.size(); j++)
			{
				if (parameters[j][0] < parameters[i][0])
				{
					Eigen::Vector2d tempt = parameters[i];
					parameters[i] = parameters[j];
					parameters[j] = tempt;
				}
			}
			line_points_temp.push_back(line_points[parameters[i][1]]);
			//segment_points.push_back(line_points_vv[parameters[i][1]]);
		}
		line_points = line_points_temp;
		return 1;
	}
	bool getCirclePoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& Circle_points, pcl::PointCloud<pcl::PointXYZ>& nonCircle_points, double interval, double radius, Eigen::Vector3d center)
	{
		pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
		pcl::PointIndices::Ptr inliers_line(new pcl::PointIndices);//线点云索引
		pcl::ModelCoefficients::Ptr coefficients_line(new pcl::ModelCoefficients);//模型系数
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line(new pcl::PointCloud<pcl::PointXYZ>());//线点云
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nonline(new pcl::PointCloud<pcl::PointXYZ>());//非线点云
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_line(new pcl::PointCloud<pcl::Normal>());
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne_line;
		ne_line.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
		ne_line.setKSearch(16);
		ne_line.setInputCloud(input.makeShared());
		ne_line.compute(*cloud_normals_line);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cylinder(new pcl::search::KdTree<pcl::PointXYZ>());
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_CIRCLE2D);//设置模型类型
		//seg.setNormalDistanceWeight(0.1);
		seg.setMaxIterations(10000);
		seg.setDistanceThreshold(2 * interval);
		seg.setRadiusLimits(0, radius);
		seg.setInputCloud(input.makeShared());
		seg.setInputNormals(cloud_normals_line);
		seg.segment(*inliers_line, *coefficients_line);
		// Extract the cylinder inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(input.makeShared());
		extract.setIndices(inliers_line);
		extract.setNegative(false);
		extract.filter(*cloud_line);
		extract.setNegative(true);
		extract.filter(*cloud_nonline);
		Circle_points = *cloud_line;
		nonCircle_points = *cloud_nonline;
		if (Circle_points.size() < 3)
		{
			return 0;
		}
		//拟合圆
		Eigen::Vector3d circle_normal;
		getCircleEquation(Circle_points, center, circle_normal, radius);
		//给出一个副法向量
		Eigen::Vector3d binormal = crossProduct(circle_normal, Eigen::Vector3d(Circle_points[0].x, Circle_points[0].y, Circle_points[0].z));
		//按照角度排序点云
		//首先计算所有点云与副法向量在主法向量下的夹角，然后按照角度排序
		std::vector<Eigen::Vector3d> Circle_points_vv = getVector3dSet(Circle_points);
		std::vector <Eigen::Vector2d> angles;
		for (int i = 0; i < Circle_points_vv.size(); i++)
		{
			angles.push_back(Eigen::Vector2d(getAngleBetweenVectors_2pi(Circle_points_vv[i] - center, binormal, circle_normal), i));
		}
		//按照参数值排序点云
		//找到最大最小值
		pcl::PointCloud <pcl::PointXYZ> Circle_points_temp;
		for (int i = 0; i < angles.size(); i++)
		{
			for (size_t j = i + 1; j < angles.size(); j++)
			{
				if (angles[j][0] < angles[i][0])
				{
					Eigen::Vector2d tempt = angles[i];
					angles[i] = angles[j];
					angles[j] = tempt;
				}
			}
			Circle_points_temp.push_back(Circle_points[angles[i][1]]);
		}
		Circle_points = Circle_points_temp;

		return bool(Circle_points.size());
	}
	bool getLineAndCurvePoints(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& line_points, pcl::PointCloud<pcl::PointXYZ>& nonline_points, double interval)
	{
		//default
		return 1;
	}
	int getPointCloudShape(const pcl::PointCloud<pcl::PointXYZ>& input)
	{
		//首先计算点云凸包
		pcl::PointCloud<pcl::PointXYZ> convex = getConvexHullPoints(input);

		//计算点云中心
		Eigen::Vector3d center;
		for (size_t i = 0; i < input.size(); i++)
		{
			center[0] += input.points[i].x;
			center[1] += input.points[i].y;
			center[2] += input.points[i].z;
		}
		center = center / input.size();
		//计算点云凸包上点到中心距离的直方图
		return 0;

	}
	bool getLinePoints_erode(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& line_points, pcl::PointCloud<pcl::PointXYZ>& nonline_points, double interval)
	{
		pcl::PointCloud<pcl::PointXYZ> line_pointsTemp, nonline_pointsTemp;
		//只保留点云中线性的部分
		//首先建立kd树
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(input.makeShared());
		//计算每个点的4邻域
		std::vector<std::vector<int>> point_neighbour;
		std::vector<std::vector<float>> point_neighbour_distance;
		for (size_t i = 0; i < input.size(); i++)
		{
			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			std::vector<Eigen::Vector3d> point_neighbour_temp;
			if (kdtree.radiusSearch(input[i], 2 * interval, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3)
			{
				//point_neighbour.push_back(pointIdxRadiusSearch);
				//point_neighbour_distance.push_back(pointRadiusSquaredDistance);
				for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
				{
					point_neighbour_temp.push_back(Eigen::Vector3d(input[pointIdxRadiusSearch[j]].x, input[pointIdxRadiusSearch[j]].y, input[pointIdxRadiusSearch[j]].z));
				}
				// 拟合直线point_neighbour_temp
				Eigen::Vector3d line_pole, line_direction;
				getLineEquation(point_neighbour_temp, line_pole, line_direction);
				//计算点到直线的距离
				std::vector<double> distance;
				for (size_t j = 0; j < point_neighbour_temp.size(); j++)
				{
					distance.push_back(getDistance_pointToLine(point_neighbour_temp[j], line_pole, line_direction));
				}
				//计算距离的方差
				double variance = varianceVector(distance);
				if (variance < 0.1 * interval)
				{
					line_pointsTemp.push_back(input[i]);
				}
				else
				{
					nonline_pointsTemp.push_back(input[i]);
				}
			}
		}
		line_points = line_pointsTemp;
		nonline_points = nonline_pointsTemp;
		return true;
	}
	// 定义一个结构体来表示RGB颜色
	struct RGB {
		int red;
		int green;
		int blue;
	};

	// 生成光谱颜色的函数
	std::vector<RGB> generateSpectrumColors(int steps) {
		std::vector<RGB> colors;

		// 从红色到黄色
		for (int i = 0; i < steps; ++i) {
			colors.push_back({ 255, i * 255 / steps, 0 });
		}

		// 从黄色到绿色
		for (int i = 0; i < steps; ++i) {
			colors.push_back({ 255 - i * 255 / steps, 255, 0 });
		}

		// 从绿色到青色
		for (int i = 0; i < steps; ++i) {
			colors.push_back({ 0, 255, i * 255 / steps });
		}

		// 从青色到蓝色
		for (int i = 0; i < steps; ++i) {
			colors.push_back({ 0, 255 - i * 255 / steps, 255 });
		}

		// 从蓝色到紫色
		for (int i = 0; i < steps; ++i) {
			colors.push_back({ i * 255 / steps, 0, 255 });
		}

		// 从紫色到红色
		for (int i = 0; i < steps; ++i) {
			colors.push_back({ 255, 0, 255 - i * 255 / steps });
		}

		return colors;
	}
	//键盘响应函数
	int callback_stop = 0;
	void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
		void* viewer_void)
	{
		pcl::visualization::PCLVisualizer* viewer = static_cast<pcl::visualization::PCLVisualizer*>(viewer_void);
		if (event.getKeyCode() == 27 && event.keyDown())
		{
			callback_stop = 1;
		}
	}
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, std::string name)
	{
		pcl::visualization::PCLVisualizer viewer(name);
		viewer.addPointCloud<pcl::PointXYZ>(input.makeShared(), "cloud");
		//红色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
		viewer.setBackgroundColor(0, 0, 0);
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showPointCloud(std::vector<Eigen::Vector3d>& input, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> input_pcl = getPCLPointXYZSet(input);
		showPointCloud(input_pcl, name);
	}
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input1, const pcl::PointCloud<pcl::PointXYZ>& input2, std::string name)
	{
		pcl::visualization::PCLVisualizer viewer(name);
		viewer.addPointCloud<pcl::PointXYZ>(input1.makeShared(), "cloud1");
		//红色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud1");
		viewer.addPointCloud<pcl::PointXYZ>(input2.makeShared(), "cloud2");
		//绿色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud2");
		viewer.setBackgroundColor(0, 0, 0);
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showPointCloud(std::vector<Eigen::Vector3d>& input1, std::vector<Eigen::Vector3d>& input2, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> input1_pcl = getPCLPointXYZSet(input1);
		pcl::PointCloud<pcl::PointXYZ> input2_pcl = getPCLPointXYZSet(input2);
		showPointCloud(input1_pcl, input2_pcl, name);
	}
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input1, const pcl::PointCloud<pcl::PointXYZ>& input2, const pcl::PointCloud<pcl::PointXYZ>& input3, std::string name)
	{
		pcl::visualization::PCLVisualizer viewer(name);
		//红色
		viewer.addPointCloud<pcl::PointXYZ>(input1.makeShared(), "cloud1");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud1");
		//绿色
		viewer.addPointCloud<pcl::PointXYZ>(input2.makeShared(), "cloud2");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud2");
		//蓝色
		viewer.addPointCloud<pcl::PointXYZ>(input3.makeShared(), "cloud3");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "cloud3");
		viewer.setBackgroundColor(0, 0, 0);
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showPointCloud(std::vector<Eigen::Vector3d>& input1, std::vector<Eigen::Vector3d>& input2, std::vector<Eigen::Vector3d>& input3, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> input1_pcl = getPCLPointXYZSet(input1);
		pcl::PointCloud<pcl::PointXYZ> input2_pcl = getPCLPointXYZSet(input2);
		pcl::PointCloud<pcl::PointXYZ> input3_pcl = getPCLPointXYZSet(input3);
		showPointCloud(input1_pcl, input2_pcl, input3_pcl, name);
	}
	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& inputCloud, pcl::PointXYZ inputPoint, std::string name)
	{
		std::vector<Eigen::Vector3d> seperateCenter;
		for (size_t i = 0; i < 1; i++)
		{
			seperateCenter.push_back(Eigen::Vector3d(inputPoint.x, inputPoint.y, inputPoint.z));
		}
		std::vector<pcl::PointCloud<pcl::PointXYZ>> seperateCircle = HG::genCirclePointCloud(seperateCenter, 5);
		seperateCircle.push_back(inputCloud);
		HG::showPointCloud(seperateCircle, "cloud");
	}
	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& inputCloud, pcl::PointXYZ inputPoint1, pcl::PointXYZ inputPoint2, std::string name)
	{
		std::vector<Eigen::Vector3d> seperateCenter;
		seperateCenter.push_back(Eigen::Vector3d(inputPoint1.x, inputPoint1.y, inputPoint1.z));
		seperateCenter.push_back(Eigen::Vector3d(inputPoint2.x, inputPoint2.y, inputPoint2.z));

		std::vector<pcl::PointCloud<pcl::PointXYZ>> seperateCircle = HG::genCirclePointCloud(seperateCenter, 5);
		seperateCircle.push_back(inputCloud);
		HG::showPointCloud(seperateCircle, "cloud");
	}
	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& inputCloud, pcl::PointXYZ inputPoint1, pcl::PointXYZ inputPoint2, pcl::PointXYZ inputPoint3, std::string name)
	{
		std::vector<Eigen::Vector3d> seperateCenter;
		seperateCenter.push_back(Eigen::Vector3d(inputPoint1.x, inputPoint1.y, inputPoint1.z));
		seperateCenter.push_back(Eigen::Vector3d(inputPoint2.x, inputPoint2.y, inputPoint2.z));
		seperateCenter.push_back(Eigen::Vector3d(inputPoint3.x, inputPoint3.y, inputPoint3.z));

		std::vector<pcl::PointCloud<pcl::PointXYZ>> seperateCircle = HG::genCirclePointCloud(seperateCenter, 5);
		seperateCircle.push_back(inputCloud);
		HG::showPointCloud(seperateCircle, "cloud");
	}
	void showPointCloudAndPoints(const pcl::PointCloud<pcl::PointXYZ>& input1, std::vector<Eigen::Vector3d> points, std::string name)
	{
		std::vector<pcl::PointCloud<pcl::PointXYZ>> seperateCircle;
		for (size_t i = 0; i < points.size(); i++)
		{
			std::vector<Eigen::Vector3d> seperateCenter;
			seperateCenter.push_back(points[i]);
			seperateCircle.push_back(HG::genCirclePointCloud(seperateCenter, 5)[0]);
		}
		seperateCircle.push_back(input1);
		HG::showPointCloud(seperateCircle, "cloud");
	}
	void showPointCloudAndPoints(std::vector<Eigen::Vector3d>& input1, std::vector<Eigen::Vector3d> points, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> input1_pcl = getPCLPointXYZSet(input1);
		std::vector<pcl::PointCloud<pcl::PointXYZ>> seperateCircle;
		for (size_t i = 0; i < points.size(); i++)
		{
			std::vector<Eigen::Vector3d> seperateCenter;
			seperateCenter.push_back(points[i]);
			seperateCircle.push_back(HG::genCirclePointCloud(seperateCenter, 5)[0]);
		}
		seperateCircle.push_back(input1_pcl);
		HG::showPointCloud(seperateCircle, "cloud");
	}
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, const pcl::PointCloud<pcl::Normal>& normal, std::string name)
	{
		pcl::visualization::PCLVisualizer viewer(name);
		viewer.addPointCloud<pcl::PointXYZ>(input.makeShared(), "cloud");
		//红色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
		viewer.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(input.makeShared(), normal.makeShared(), 1000, 10, "normals");
		viewer.setBackgroundColor(0, 0, 0);
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PrincipalRadiiRSD> RSDFeature, std::string name)
	{
		//颜色盘
		double color[10][3] = { { 255,0,0 },{ 0,255,0 },{ 0,0,255 },{ 255,255,0 },{ 0,255,255 },{ 255,0,255 },{ 255,255,255 },{ 0,0,0 },{ 255,128,0 },{ 255,0,128 } };

		//构建带颜色的点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud->width = input.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		for (size_t i = 0; i < input.size(); i++)
		{
			double r = (RSDFeature.points[i].r_min) > 1000.0 ? 1000.0 : (RSDFeature.points[i].r_max);
			int rgb = floor(RSDFeature.points[i].r_max / 300);
			cloud->points[i].x = input[i].x;
			cloud->points[i].y = input[i].y;
			cloud->points[i].z = input[i].z;
			cloud->points[i].r = color[rgb][0];
			cloud->points[i].g = color[rgb][1];
			cloud->points[i].b = color[rgb][2];
		}
		//显示点云自带的颜色
		pcl::visualization::PCLVisualizer viewer(name);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer.addPointCloud(cloud, rgb, name);
		//设置背景颜色
		viewer.setBackgroundColor(0, 0, 0);
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PrincipalCurvatures> principleCurve, std::string name)
	{
		//颜色盘
		double color[10][3] = { { 255,0,0 },{ 0,255,0 },{ 0,0,255 },{ 255,255,0 },{ 0,255,255 },{ 255,0,255 },{ 255,255,255 },{ 0,0,0 },{ 255,128,0 },{ 255,0,128 } };

		//构建带颜色的点云
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		cloud->width = input.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		for (size_t i = 0; i < input.size(); i++)
		{
			//double curve = (principleCurve[i].pc1* principleCurve[i].pc2)*10000000;//高斯曲率
			//double curve = (principleCurve[i].pc1 + principleCurve[i].pc2) *1000;//平均曲率
			double curve = (principleCurve[i].pc1) * 1000;//主曲率

			int k = 0;
			if (curve > 0)
			{
				while (curve > 10)
				{
					curve = (curve / 10);
					k++;
				}
			}
			int rgb = k;
			cloud->points[i].x = input[i].x;
			cloud->points[i].y = input[i].y;
			cloud->points[i].z = input[i].z;
			cloud->points[i].r = color[rgb][0];
			cloud->points[i].g = color[rgb][1];
			cloud->points[i].b = color[rgb][2];
		}
		//显示点云自带的颜色
		pcl::visualization::PCLVisualizer viewer(name);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer.addPointCloud(cloud, rgb, name);
		//设置背景颜色
		viewer.setBackgroundColor(0, 0, 0);
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showPointCloud(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& input, std::string name)
	{
		//颜色盘
		//double color[10][3] = { { 255,0,0 },{ 0,255,0 },{ 0,0,255 },{ 255,255,0 },{ 0,255,255 },{ 255,0,255 },{ 255,255,255 },{ 128,128,0 },{ 255,128,0 },{ 255,0,128 } };
		std::vector<RGB> spectrumColors = generateSpectrumColors(ceil(input.size() / 6.0));
		pcl::visualization::PCLVisualizer viewer(name);
		for (size_t i = 0; i < input.size(); i++)
		{
			std::string cloud_name = "cloud" + std::to_string(i);
			viewer.addPointCloud<pcl::PointXYZ>(input[i].makeShared(), cloud_name);
			//int rgb = rand()/(1.0+RAND_MAX);
			//红色
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, spectrumColors[i].red / 255.0, spectrumColors[i].green / 255.0, spectrumColors[i].blue / 255.0, cloud_name);
		}
		viewer.setBackgroundColor(0, 0, 0);
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input, pcl::PointCloud<pcl::PointXYZ>& lin1, pcl::PointCloud<pcl::PointXYZ>& lin2, std::string name)
	{
		pcl::visualization::PCLVisualizer viewer(name);
		viewer.addPointCloud<pcl::PointXYZ>(input.makeShared(), "cloud");
		//红色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
		if (lin1.size() == lin2.size())
		{
			for (size_t i = 0; i < lin1.size(); i++)
			{
				double randx = rand() / (1.0 + RAND_MAX);
				double randy = rand() / (1.0 + RAND_MAX);
				double randz = rand() / (1.0 + RAND_MAX);
				std::string id = "line_" + std::to_string(i);
				viewer.addLine(lin1.points[i], lin2.points[i], randx, randy, randz, id);
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, id);

				std::string text_id = "text_" + std::to_string(i);
				viewer.addText("line " + std::to_string(i), 10, 10 + i * 16, 16, randx, randy, randz, text_id);

			}
		}
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input, std::vector<Eigen::Vector3d>& lin1, std::vector<Eigen::Vector3d>& lin2, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> lin1_pcl, lin2_pcl;
		for (size_t i = 0; i < lin1.size(); i++)
		{
			pcl::PointXYZ point1, point2;
			point1.x = lin1[i][0];
			point1.y = lin1[i][1];
			point1.z = lin1[i][2];
			point2.x = lin2[i][0];
			point2.y = lin2[i][1];
			point2.z = lin2[i][2];
			lin1_pcl.push_back(point1);
			lin2_pcl.push_back(point2);
		}
		showLine(input, lin1_pcl, lin2_pcl, name);
	}
	void transBoxToLine(Eigen::MatrixXd& box, pcl::PointCloud<pcl::PointXYZ>& lin1, pcl::PointCloud<pcl::PointXYZ>& lin2)
	{
		lin1.clear();
		lin2.clear();
		pcl::PointXYZ p1 = pcl::PointXYZ(box(0, 0), box(0, 1), box(0, 2));
		pcl::PointXYZ p2 = pcl::PointXYZ(box(1, 0), box(1, 1), box(1, 2));
		pcl::PointXYZ p3 = pcl::PointXYZ(box(2, 0), box(2, 1), box(2, 2));
		pcl::PointXYZ p4 = pcl::PointXYZ(box(3, 0), box(3, 1), box(3, 2));
		pcl::PointXYZ p5 = pcl::PointXYZ(box(4, 0), box(4, 1), box(4, 2));
		pcl::PointXYZ p6 = pcl::PointXYZ(box(5, 0), box(5, 1), box(5, 2));
		pcl::PointXYZ p7 = pcl::PointXYZ(box(6, 0), box(6, 1), box(6, 2));
		pcl::PointXYZ p8 = pcl::PointXYZ(box(7, 0), box(7, 1), box(7, 2));
		lin1.push_back(p1);
		lin1.push_back(p2);
		lin1.push_back(p3);
		lin1.push_back(p4);
		lin1.push_back(p5);
		lin1.push_back(p6);
		lin1.push_back(p7);
		lin1.push_back(p8);
		lin2.push_back(p2);
		lin2.push_back(p4);
		lin2.push_back(p1);
		lin2.push_back(p3);
		lin2.push_back(p6);
		lin2.push_back(p8);
		lin2.push_back(p5);
		lin2.push_back(p7);
		lin1.push_back(p1);
		lin1.push_back(p2);
		lin1.push_back(p3);
		lin1.push_back(p4);
		lin2.push_back(p5);
		lin2.push_back(p6);
		lin2.push_back(p7);
		lin2.push_back(p8);
	}
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input, Eigen::MatrixXd box, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> lin1, lin2;
		transBoxToLine(box, lin1, lin2);
		showLine(input, lin1, lin2, name);
	}
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointCloud<pcl::PointXYZ>& input2, pcl::PointCloud<pcl::PointXYZ>& lin1, pcl::PointCloud<pcl::PointXYZ>& lin2, std::string name)
	{
		pcl::visualization::PCLVisualizer viewer(name);
		viewer.addPointCloud<pcl::PointXYZ>(input1.makeShared(), "cloud");
		//红色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "cloud");
		viewer.addPointCloud<pcl::PointXYZ>(input2.makeShared(), "cloud2");
		//绿色
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "cloud2");
		if (lin1.size() == lin2.size())
		{
			for (size_t i = 0; i < lin1.size(); i++)
			{
				double randx = rand() / (1.0 + RAND_MAX);
				double randy = rand() / (1.0 + RAND_MAX);
				double randz = rand() / (1.0 + RAND_MAX);
				std::string id = "line_" + std::to_string(i);
				viewer.addLine(lin1.points[i], lin2.points[i], randx, randy, randz, id);
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 6, id);

				std::string text_id = "text_" + std::to_string(i);
				viewer.addText("line " + std::to_string(i), 10, 10 + i * 16, 16, randx, randy, randz, text_id);

			}
		}
		viewer.registerKeyboardCallback(keyboardEventOccurred, &viewer);
		callback_stop = 0;
		while (!viewer.wasStopped() && callback_stop == 0)
		{
			viewer.spinOnce();
		}
		viewer.close();
	}
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointCloud<pcl::PointXYZ>& input2, Eigen::MatrixXd box, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> lin1, lin2;
		transBoxToLine(box, lin1, lin2);
		showLine(input1, input2, lin1, lin2, name);
	}
	void showLine(pcl::PointCloud<pcl::PointXYZ>& input1, Eigen::Vector4d planePara, std::string name)
	{
		pcl::PointCloud<pcl::PointXYZ> plane;
		//求两个点
		double length = 1000;
		for (size_t i = 0; i < length; i++)
		{
			double x, y, z;
			if (planePara[2] != 0)
			{
				x = rand() / (1.0 + RAND_MAX) * 1000;
				y = rand() / (1.0 + RAND_MAX) * 1000;
				z = (-planePara[3] - planePara[0] * x - planePara[1] * y) / planePara[2];
			}
			else if (planePara[1] != 0)
			{
				x = rand() / (1.0 + RAND_MAX) * 1000;
				z = rand() / (1.0 + RAND_MAX) * 1000;
				y = (-planePara[3] - planePara[0] * x - planePara[2] * z) / planePara[1];
			}
			else
			{
				y = rand() / (1.0 + RAND_MAX) * 1000;
				z = rand() / (1.0 + RAND_MAX) * 1000;
				x = (-planePara[3] - planePara[1] * y - planePara[2] * z) / planePara[0];
			}
			pcl::PointXYZ point;
			point.x = x;
			point.y = y;
			point.z = z;
			plane.push_back(point);
		}
		showPointCloud(input1, plane, name);
	}
	void showPoint(pcl::PointCloud<pcl::PointXYZ>& input, std::vector<Eigen::Vector3d>& points, std::string name)
	{
		std::vector<pcl::PointCloud<pcl::PointXYZ>> circlePoints = genCirclePointCloud(points, 10);
		circlePoints.push_back(input);
		showPointCloud(circlePoints, name);
	}
	pcl::PointCloud<pcl::PointXYZ> genCylinderPointCloud(Eigen::VectorXd coefficients, double length)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		if (coefficients.size() < 7)
		{
			return cloud;
		}
		//中心点
		Eigen::Vector3d center(coefficients[0], coefficients[1], coefficients[2]);
		//法向量
		Eigen::Vector3d normal(coefficients[3], coefficients[4], coefficients[5]);
		normal.normalize();
		//半径
		double radius = coefficients[6];
		// Create a cylinder with chosen coefficients
		for (float z = -length; z <= length; z += 1)
		{
			for (float angle = 0.0f; angle <= 360.0f; angle += 5.0f)
			{
				pcl::PointXYZ point;
				point.x = radius * cosf(pcl::deg2rad(angle));
				point.y = radius * sinf(pcl::deg2rad(angle));
				point.z = z;
				cloud.points.push_back(point);
			}
		}
		cloud.width = cloud.points.size();
		cloud.height = 1;
		//将点云旋转到法向量方向
		Eigen::Vector3d z_axis(0, 0, 1);
		Eigen::Vector3d rotation_axis = crossProduct(z_axis, normal);
		rotation_axis.normalize();
		double rotation_angle = acos(normal.dot(z_axis));
		//旋转矩阵
		Eigen::Matrix3d rotation_matrix = getRotationMatrix(rotation_angle, rotation_axis);
		//旋转
		for (size_t i = 0; i < cloud.size(); i++)
		{
			Eigen::Vector3d point(cloud[i].x, cloud[i].y, cloud[i].z);
			point = rotation_matrix * point;
			cloud[i].x = point[0] + center[0];
			cloud[i].y = point[1] + center[1];
			cloud[i].z = point[2] + center[2];
		}
		return cloud;
	}
	pcl::PointCloud<pcl::PointXYZ> genCylinderPointCloud(pcl::ModelCoefficients::Ptr coefficients, double length)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		Eigen::Vector<double, 7> input;
		if (coefficients->values.size() < 7)
		{
			return cloud;
		}
		for (size_t i = 0; i < coefficients->values.size(); i++)
		{
			input[i] = coefficients->values[i];
		}
		cloud = genCylinderPointCloud(input, length);
		return cloud;
	}
	pcl::PointCloud<pcl::PointXYZ> genCirclePointCloud(pcl::ModelCoefficients::Ptr coefficients)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud;
		//中心点
		Eigen::Vector3d center(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
		//半径
		double radius = coefficients->values[3];
		// Create a cylinder with chosen coefficients
		for (float angle1 = 0.0f; angle1 <= 360.0f; angle1 += 5.0f)
		{
			for (float angle2 = 0.0f; angle2 <= 360.0f; angle2 += 5.0f)
			{
				pcl::PointXYZ point;
				point.x = radius * sinf(pcl::deg2rad(angle1)) * cosf(pcl::deg2rad(angle2)) + center[0];
				point.y = radius * sinf(pcl::deg2rad(angle1)) * sinf(pcl::deg2rad(angle2)) + center[1];
				point.z = radius * cosf(pcl::deg2rad(angle1)) + center[2];
				cloud.points.push_back(point);
			}
		}
		cloud.width = cloud.points.size();
		cloud.height = 1;
		return cloud;
	}
	pcl::PointCloud<pcl::PointXYZ> genCirclePointCloud(Eigen::Vector3d para, double radius)
	{
		pcl::ModelCoefficients::Ptr circle_para(new pcl::ModelCoefficients);
		circle_para->values.push_back(para[0]);
		circle_para->values.push_back(para[1]);
		circle_para->values.push_back(para[2]);
		circle_para->values.push_back(radius);
		pcl::PointCloud<pcl::PointXYZ> output = genCirclePointCloud(circle_para);
		return output;
	}
	std::vector<pcl::PointCloud<pcl::PointXYZ>> genCirclePointCloud(std::vector<Eigen::Vector3d>& para, double radius)
	{
		//输入为圆心坐标和半径，输出为圆的点云
		std::vector<pcl::PointCloud<pcl::PointXYZ>> output;
		for (size_t i = 0; i < para.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZ> cloud = genCirclePointCloud(para[i], radius);
			output.push_back(cloud);
		}
		return output;

	}
	pcl::PointCloud<pcl::PointXYZ> genCirclePointCloud(Eigen::Vector3d center, Eigen::Vector3d normal, double radius, double intervalAngle)
	{
		pcl::PointCloud<pcl::PointXYZ> cloud, output;
		// Create a cylinder with chosen coefficients
		for (float angle1 = 0.0f; angle1 <= 360.0f; angle1 += intervalAngle)
		{
			pcl::PointXYZ point;
			point.x = radius * cosf(pcl::deg2rad(angle1));
			point.y = radius * sinf(pcl::deg2rad(angle1));
			point.z = 0;
			cloud.push_back(point);
		}
		Eigen::Vector3d zAxis = Eigen::Vector3d(0, 0, 1);
		Eigen::MatrixXd rotateMatrix = getRotationMatrix(zAxis, normal);
		Eigen::MatrixXd transformMatrix = getTransformMatrix(rotateMatrix, center);
		pcl::transformPointCloud(cloud, output, static_cast<Eigen::Matrix4d>(transformMatrix));
		return output;
	}
	cv::Mat Eigen2Mat(const Eigen::MatrixXd& eigen)
	{
		cv::Mat output(eigen.rows(), eigen.cols(), CV_64F);
		for (size_t i = 0; i < eigen.rows(); i++)
		{
			for (size_t j = 0; j < eigen.cols(); j++)
			{
				output.at<double>(i, j) = eigen(i, j);
			}
		}
		return output;
	}
	cv::Mat Eigen2Mat(const std::vector<Eigen::MatrixXd>& eigen)
	{
		cv::Mat output(eigen.size() * eigen[0].rows(), eigen[0].cols(), CV_64F);
		for (size_t k = 0; k < eigen.size(); k++)
		{
			for (size_t i = 0; i < eigen[k].rows(); i++)
			{
				for (size_t j = 0; j < eigen[k].cols(); j++)
				{
					output.at<double>(k * eigen[0].rows() + i, j) = eigen[k](i, j);
				}
			}
		}
		return output;
	}
	Eigen::MatrixXd Mat2Eigen(const cv::Mat& cv)
	{
		Eigen::MatrixXd output(cv.rows, cv.cols);
		for (size_t i = 0; i < cv.rows; i++)
		{
			for (size_t j = 0; j < cv.cols; j++)
			{
				output(i, j) = cv.at<double>(i, j);
			}
		}
		return output;
	}
	std::vector<Eigen::MatrixXd> RotorChange(Eigen::MatrixXd inputXYZ, Eigen::MatrixXd& outputRZ)
	{
		//输入为XYZ坐标系下的点，输出为RZ坐标系下的点
		//输入为n*3，输出为n*3
		std::vector<Eigen::MatrixXd> output;
		Eigen::MatrixXd input = inputXYZ;
		Eigen::MatrixXd outputY(2, input.cols());
		Eigen::MatrixXd JYX(2 * input.cols(), 3);
		JYX.setZero();
		for (size_t i = 0; i < input.cols(); i++)
		{
			outputY(0, i) = pow(pow(input(0, i), 2) + pow(input(1, i), 2), 0.5);
			outputY(1, i) = input(2, i);
		}
		for (size_t i = 0; i < input.cols(); i++)//2 * input.cols()
		{
			double r = pow(pow(input(0, i), 2) + pow(input(1, i), 2), 0.5);
			JYX(2 * i + 0, 0) = input(0, i) / r;
			JYX(2 * i + 0, 1) = input(1, i) / r;
			JYX(2 * i + 1 + 0, 2) = 1;
		}
		outputRZ = outputY;
		output.push_back(JYX);
		return output;
	}
	bool BSplineCurve2DgreeBase(double input, double& output, double& derivativeYtoX, double tStart, double tmiddle, double tend, double tend2)
	{
		//用来处理单个B样条的基类
		//首先先算3个y值，3个导数值
		double t = input;
		double t0 = tStart;
		double t1 = tmiddle;
		double t2 = tend;
		double t3 = tend2;
		if (t2 - t0 == 0 || t1 - t0 == 0 || t2 - t1 == 0 || t3 - t1 == 0 || t3 - t2 == 0)
		{
			return false;
		}
		double output1, output2, output3, derivative1, derivative2, derivative3;
		if (t >= t0 && t < t1)
		{
			output1 = pow(t - t0, 2) / (t2 - t0) / (t1 - t0);
			derivative1 = 2 * (t - t0);
			output = output1;
			derivativeYtoX = derivative1;
		}
		else if (t >= t1 && t < t2)
		{
			output2 = (t - t0) * (t2 - t) / (t2 - t0) / (t2 - t1) + (t3 - t) * (t - t1) / (t3 - t1) / (t2 - t1);
			derivative2 = (t2 + t0 - 2 * t) / (t2 - t0) / (t2 - t1) + (t3 + t1 - 2 * t) / (t3 - t1) / (t2 - t1);
			output = output2;
			derivativeYtoX = derivative2;
		}
		else if (t >= t2 && t < t3)
		{
			output3 = pow(t3 - t, 2) / (t3 - t1) / (t3 - t2);
			derivative3 = 2 * (t - t3);
			output = output3;
			derivativeYtoX = derivative3;
		}
		else
		{
			output = 0;
			derivativeYtoX = 0;
		}
		return true;
	}
	bool BSplineCureve2D(double input, double& output, double& derivativeYtoX, std::vector<double>& derivativeYtoP, double tStart, double tend, std::vector<double> Para)
	{
		if (Para.size() == 0)
		{
			return false;
		}
		derivativeYtoP.clear();
		int num = Para.size() + 2;
		double interval = (tend - tStart) / num;
		double outputSum = 0, derivativeYtoXSum = 0;
		for (size_t i = 0; i < Para.size(); i++)
		{
			double outputTemp = 0, derivativeYtoXTemp = 0;
			if (BSplineCurve2DgreeBase(input, outputTemp, derivativeYtoXTemp, tStart + i * interval, tStart + (i + 1) * interval, tStart + (i + 2) * interval, tStart + (i + 3) * interval))
			{
				outputSum += outputTemp * Para[i];
				derivativeYtoXSum += derivativeYtoXTemp * Para[i];
				derivativeYtoP.push_back(outputTemp);
			}
		}
		output = outputSum;
		derivativeYtoX = derivativeYtoXSum;
		return true;
	}
	bool BSplineCureve2DTransfer(double input, double& output, double& derivativeYtoX, std::vector<double>& derivativeYtoP, double tStart, double tend, std::vector<double> Para)
	{
		//带旋转的B样条曲线
		//输入为2*n的点，输出为n的点云，输入的Para 为3 + num，其中前3项为a，d，theta，分别对应r和z方向的平移和旋转，后num项为2次B样条曲线的参数
		//需要重新确认Para,现在先写平移变量
		if (Para.size() < 2)
		{
			return false;
		}
		std::vector<double> para;
		for (size_t i = 0; i < Para.size(); i++)
		{
			if (i > 1)
			{
				para.push_back(Para[i]);
			}
		}
		double a = Para[0];
		double d = Para[1];
		double outputBS, derivativeYtoXBS;
		std::vector<double> derivativeYtoPBS;
		double inputX = input + a;
		if (BSplineCureve2D(inputX, outputBS, derivativeYtoXBS, derivativeYtoPBS, tStart, tend, para))
		{
			outputBS = outputBS + d;
			derivativeYtoPBS.insert(derivativeYtoPBS.begin(), 1);//对d的导数
			derivativeYtoPBS.insert(derivativeYtoPBS.begin(), derivativeYtoXBS);//对a的导数
		}
		else
		{
			return false;
		}
		output = outputBS;
		derivativeYtoX = derivativeYtoXBS;
		derivativeYtoP = derivativeYtoPBS;
		return true;
	}

	std::vector<Eigen::MatrixXd> BSplineCureve2DChange(Eigen::MatrixXd input, Eigen::MatrixXd& output, double tStart, double tend, Eigen::MatrixXd Para)
	{
		//输入为2*n的点，输出为n的点云
		Eigen::MatrixXd JYX(input.cols(), 2);
		JYX.setZero();
		Eigen::MatrixXd JYP(input.cols(), Para.size());
		JYP.setZero();
		Eigen::MatrixXd Y1(input.cols(), 1);
		Y1.setZero();
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			para.push_back(Para(i, 0));
		}
		for (size_t i = 0; i < input.cols(); i++)
		{
			double outputTemp = 0, derivativeYtoXTemp = 0;
			std::vector<double> derivativeYtoP;
			if (BSplineCureve2D(input(0, i), outputTemp, derivativeYtoXTemp, derivativeYtoP, tStart, tend, para))
			{
				Y1(i, 0) = outputTemp - input(1, i);
				for (size_t j = 0; j < derivativeYtoP.size(); j++)
				{
					JYP(i, j) = derivativeYtoP[j];
				}
				JYX(i, 0) = derivativeYtoXTemp;
				JYX(i, 0 + 1) = -1;
			}
		}
		output = Y1;
		std::vector<Eigen::MatrixXd> outputSet;
		outputSet.push_back(JYX);
		outputSet.push_back(JYP);
		return outputSet;
	}
	std::vector<Eigen::MatrixXd> BSplineCureve2DTransferChange(Eigen::MatrixXd input, Eigen::MatrixXd& output, double tStart, double tend, Eigen::MatrixXd Para)
	{
		//输入为2*n的点，输出为n的点云
		Eigen::MatrixXd JYX(input.cols(), 2);
		JYX.setZero();
		Eigen::MatrixXd JYP(input.cols(), Para.size());
		JYP.setZero();
		Eigen::MatrixXd Y1(input.cols(), 1);
		Y1.setZero();
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			para.push_back(Para(i, 0));
		}
		for (size_t i = 0; i < input.cols(); i++)
		{
			double outputTemp = 0, derivativeYtoXTemp = 0;
			std::vector<double> derivativeYtoP;
			if (BSplineCureve2DTransfer(input(0, i), outputTemp, derivativeYtoXTemp, derivativeYtoP, tStart, tend, para))
			{
				Y1(i, 0) = outputTemp - input(1, i);
				for (size_t j = 0; j < derivativeYtoP.size(); j++)
				{
					JYP(i, j) = derivativeYtoP[j];
				}
				JYX(i, 0) = derivativeYtoXTemp;
				JYX(i, 0 + 1) = -1;
			}
		}
		output = Y1;
		std::vector<Eigen::MatrixXd> outputSet;
		outputSet.push_back(JYX);
		outputSet.push_back(JYP);
		return outputSet;
	}
	Eigen::MatrixXd dialogMultify(Eigen::MatrixXd input1, int row, int col, Eigen::MatrixXd input2)
	{
		//对角线矩阵相乘
		//输入 input1 为对角线矩阵，其block为row*col的矩阵
		//输入 input2 为(n*row)*(m*col)的矩阵
		Eigen::MatrixXd output(input1.rows(), input2.cols());
		output.setZero();
		int rowNum = input1.rows() / row;
		for (size_t i = 0; i < rowNum; i++)
		{
			Eigen::MatrixXd block1 = input1.block(i * row, 0, row, col);
			Eigen::MatrixXd block2 = input2.block(i * col, 0, col, input2.cols());
			output.block(i * row, 0, row, input2.cols()) = block1 * block2;
		}
		return output;
	}
	std::vector<Eigen::MatrixXd> RevolvedBodyFitFlowSingle(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Y, Eigen::MatrixXd& Para, double tStart, double tend)
	{
		//用来计算单步的输入，输出，以及雅可比矩阵
		//inputCloud: 3*n,参数； Y: n*1；J: n * (6+num)；para: 6 + num;
		//其中参数中的前6项为旋转矩阵，后num项为2次B样条曲线的参数值
		//整体流程为前向传递，反向传播，需要计算每一个步骤中的X，Y，DYX，DYP
		//流程1，旋转
		//输入 X，输出Y，参数：旋转矩阵，
		//生成旋转矩阵
		Eigen::Vector3d theta = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d Trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(theta);
		Eigen::Matrix4d transMatrix = HG::getTransformMatrix(rotationMatrix, Trans);
		Eigen::MatrixXd XRotation, YRotation;
		XRotation = inputPoint;
		YRotation = transformMatrix(transMatrix, XRotation);
		std::vector<Eigen::MatrixXd> JRotationSet = getRotationProcessDerivativePara(transMatrix, XRotation, YRotation);
		//Eigen::MatrixXd JXRotation = JRotationSet[0];
		Eigen::MatrixXd JThetaRotation = JRotationSet[0];
		Eigen::MatrixXd JTRotation = JRotationSet[1];
		cv::Mat XRotationMat = HG::getMat(XRotation);
		cv::Mat YRotationMat = HG::getMat(YRotation);
		cv::Mat JThetaRotationMat = HG::getMat(JThetaRotation);
		cv::Mat JTRotationMat = HG::getMat(JTRotation);
		//流程2，x，y转r
		//输入 YRotation， 输出YRZ，参数无
		Eigen::MatrixXd YRZ;
		std::vector<Eigen::MatrixXd> JXYtoRZSet = RotorChange(YRotation, YRZ);
		Eigen::MatrixXd JXtoRZ = JXYtoRZSet[0];
		cv::Mat JXtoRZMat = HG::getMat(JXtoRZ);
		//流程3,，B样条曲线
		//输入 YRZ，输出 YB，参数：B样条曲线的参数
		Eigen::MatrixXd YB;
		Eigen::MatrixXd JPtoB;
		std::vector<Eigen::MatrixXd> JBSplineSet = BSplineCureve2DChange(YRZ, YB, tStart, tend, Para.block(6, 0, Para.rows() - 6, 1));
		Eigen::MatrixXd JXtoB = JBSplineSet[0];
		JPtoB = JBSplineSet[1];
		cv::Mat JBMat = HG::getMat(YB);
		cv::Mat JXtoBMat = HG::getMat(JXtoB);
		cv::Mat JPtoBMat = HG::getMat(JPtoB);
		//构建整体流程的JYX，JYP，YB
		std::vector<Eigen::MatrixXd> outputSet;
		Eigen::MatrixXd JYX, JYTheta, JYTrans, JYP, JAll;
		//JYX = JXRotation * JXtoRZ * JXtoB;
		JYTheta = dialogMultify(JXtoB, 1, 2, dialogMultify(JXtoRZ, 2, 3, JThetaRotation));
		JYTrans = dialogMultify(JXtoB, 1, 2, dialogMultify(JXtoRZ, 2, 3, JTRotation));
		JYP = JPtoB;
		JAll = Eigen::MatrixXd::Zero(YB.rows(), Para.rows());
		for (size_t i = 0; i < YB.rows(); i++)
		{
			for (size_t j = 0; j < JYTheta.cols(); j++)
			{
				JAll(i, j) = JYTheta(i, j);
			}
			for (size_t j = 0; j < JYTrans.cols(); j++)
			{
				JAll(i, JYTheta.cols() + j) = JYTrans(i, j);
			}
			for (size_t j = 0; j < JYP.cols(); j++)
			{
				JAll(i, JYTheta.cols() + JYTrans.cols() + j) = JYP(i, j);
			}
		}
		cv::Mat JAllMat = HG::getMat(JAll);
		outputSet.push_back(JYX);
		outputSet.push_back(JAll);
		outputSet.push_back(YB);
		outputSet.push_back(JYTheta);
		outputSet.push_back(JYTrans);
		outputSet.push_back(JYP);
		return outputSet;
	}
	std::vector<Eigen::MatrixXd> RevolvedBodyFitFlowTransferSingle(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Y, Eigen::MatrixXd& Para, double tStart, double tend)
	{
		//用来计算单步的输入，输出，以及雅可比矩阵
		//inputCloud: 3*n,参数； Y: n*1；J: n * (6+num)；para: 6 + num;
		//其中参数中的前6项为旋转矩阵，后num项为2次B样条曲线的参数值
		//整体流程为前向传递，反向传播，需要计算每一个步骤中的X，Y，DYX，DYP
		//流程1，旋转
		//输入 X，输出Y，参数：旋转矩阵，
		//生成旋转矩阵
		Eigen::Vector3d theta = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d Trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(theta);
		Eigen::Matrix4d transMatrix = HG::getTransformMatrix(rotationMatrix, Trans);
		Eigen::MatrixXd XRotation, YRotation;
		XRotation = inputPoint;
		YRotation = transformMatrix(transMatrix, XRotation);
		std::vector<Eigen::MatrixXd> JRotationSet = getRotationProcessDerivativePara(transMatrix, XRotation, YRotation);
		//Eigen::MatrixXd JXRotation = JRotationSet[0];
		Eigen::MatrixXd JThetaRotation = JRotationSet[0];
		Eigen::MatrixXd JTRotation = JRotationSet[1];
		cv::Mat XRotationMat = HG::getMat(XRotation);
		cv::Mat YRotationMat = HG::getMat(YRotation);
		cv::Mat JThetaRotationMat = HG::getMat(JThetaRotation);
		cv::Mat JTRotationMat = HG::getMat(JTRotation);
		//流程2，x，y转r
		//输入 YRotation， 输出YRZ，参数无
		Eigen::MatrixXd YRZ;
		std::vector<Eigen::MatrixXd> JXYtoRZSet = RotorChange(YRotation, YRZ);
		Eigen::MatrixXd JXtoRZ = JXYtoRZSet[0];
		cv::Mat JXtoRZMat = HG::getMat(JXtoRZ);
		//流程3,，B样条曲线
		//输入 YRZ，输出 YB，参数：B样条曲线的参数
		Eigen::MatrixXd YB;
		Eigen::MatrixXd JPtoB;
		std::vector<Eigen::MatrixXd> JBSplineSet = BSplineCureve2DTransferChange(YRZ, YB, tStart, tend, Para.block(6, 0, Para.rows() - 6, 1));
		Eigen::MatrixXd JXtoB = JBSplineSet[0];
		JPtoB = JBSplineSet[1];
		cv::Mat JBMat = HG::getMat(YB);
		cv::Mat JXtoBMat = HG::getMat(JXtoB);
		cv::Mat JPtoBMat = HG::getMat(JPtoB);
		//构建整体流程的JYX，JYP，YB
		std::vector<Eigen::MatrixXd> outputSet;
		Eigen::MatrixXd JYX, JYTheta, JYTrans, JYP, JAll;
		//JYX = JXRotation * JXtoRZ * JXtoB;
		JYTheta = dialogMultify(JXtoB, 1, 2, dialogMultify(JXtoRZ, 2, 3, JThetaRotation));
		JYTrans = dialogMultify(JXtoB, 1, 2, dialogMultify(JXtoRZ, 2, 3, JTRotation));
		JYP = JPtoB;
		JAll = Eigen::MatrixXd::Zero(YB.rows(), Para.rows());
		for (size_t i = 0; i < YB.rows(); i++)
		{
			for (size_t j = 0; j < JYTheta.cols(); j++)
			{
				JAll(i, j) = JYTheta(i, j);
			}
			for (size_t j = 0; j < JYTrans.cols(); j++)
			{
				JAll(i, JYTheta.cols() + j) = JYTrans(i, j);
			}
			for (size_t j = 0; j < JYP.cols(); j++)
			{
				JAll(i, JYTheta.cols() + JYTrans.cols() + j) = JYP(i, j);
			}
		}
		cv::Mat JAllMat = HG::getMat(JAll);
		outputSet.push_back(JYX);
		outputSet.push_back(JAll);
		outputSet.push_back(YB);
		outputSet.push_back(JYTheta);
		outputSet.push_back(JYTrans);
		outputSet.push_back(JYP);
		return outputSet;
	}

	bool RevolvedBodyFitFlowLM(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum, Eigen::MatrixXd& YEerror)
	{
		//用来计算单步的输入，输出，以及雅可比矩阵
		//inputCloud: 3*n,参数； Y: n*1；J: n * (6+num)；para: 6 + num;
		//其中参数中的前6项为旋转矩阵，后num项为2次B样条曲线的参数值
		//整体流程为前向传递，反向传播，需要计算每一个步骤中的X，Y，DYX，DYP
		//流程1，旋转
		//输入 X，输出Y，参数：初始旋转平移矩阵，B样条函数参数，B样条其实点，终点，迭代次数
		double gama = 1E-10;
		double stride = 0.1;
		//测试一下参数的不同权重
		Eigen::MatrixXd E = Eigen::MatrixXd::Identity(Para.rows(), Para.rows());
		E = gama * E;
		Eigen::MatrixXd EStride = Eigen::MatrixXd::Identity(Para.rows(), Para.rows());
		EStride = stride * EStride;
		Eigen::MatrixXd EStrideTrans = Eigen::MatrixXd::Identity(3, 3);
		EStrideTrans = 1 * EStrideTrans;
		EStride.block(2, 2, 3, 3) = EStrideTrans;
		Eigen::MatrixXd EStrideRotate = Eigen::MatrixXd::Identity(3, 3);
		EStrideRotate = 0.2 * EStrideRotate;
		EStride.block(0, 0, 3, 3) = EStrideRotate;

		Eigen::MatrixXd Y1, Y0;
		Eigen::MatrixXd error = Eigen::MatrixXd::Zero(iterateNum, 1);
		std::vector<Eigen::MatrixXd> ParaSet;
		for (size_t i = 0; i < iterateNum; i++)
		{
			Eigen::MatrixXd JYX, JYP;
			std::vector<Eigen::MatrixXd> outputSet = RevolvedBodyFitFlowSingle(inputPoint, Y1, Para, tStart, tend);
			JYX = outputSet[0];
			JYP = outputSet[1];
			Y1 = outputSet[2];
			Eigen::MatrixXd JYPZero = Eigen::MatrixXd::Zero(JYP.rows(), 1);
			//JYP.block(0, 0, JYP.rows(), 1) = JYPZero;
			//JYP.block(0, 1, JYP.rows(), 1) = JYPZero;
			JYP.block(0, 2, JYP.rows(), 1) = JYPZero;
			JYP.block(0, 5, JYP.rows(), 1) = JYPZero;
			cv::Mat JYXMat, JYPMat, Y1Mat;
			JYXMat = Eigen2Mat(JYX);
			JYPMat = Eigen2Mat(JYP);
			Y1Mat = Eigen2Mat(Y1);
			//Y0 = Eigen::MatrixXd::Zero(Y1.rows(), Y1.cols());
			Eigen::MatrixXd detaA = (JYP.transpose() * JYP + E).inverse() * JYP.transpose() * (-Y1);
			detaA = EStride * detaA;
			cv::Mat detaAMat = Eigen2Mat(detaA);
			ParaSet.push_back(Para);
			//更新参数
			Para += detaA;
			error(i, 0) = (-Y1).norm();
			if (error(i, 0) < 1E-5)
			{
				break;
			}
			//if (i > 1&& error(i, 0) > error(i - 1, 0))
			{
				//stride *= 0.1;
			}
		}
		YEerror = Y1;
		cv::Mat Y1Mat = Eigen2Mat(Y1);
		cv::Mat paraMat = Eigen2Mat(Para);
		cv::Mat errorMat = Eigen2Mat(error);
		double minError = 1E10;
		for (size_t i = 0; i < error.rows(); i++)
		{
			if (abs(error(i, 0)) < minError)
			{
				minError = abs(error(i, 0));
				Para = ParaSet[i];
			}
		}
		return true;
	}
	bool RevolvedBodyFitFlowTransferLM(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum, Eigen::MatrixXd& YEerror)
	{
		//用来计算单步的输入，输出，以及雅可比矩阵
		//inputCloud: 3*n,参数； Y: n*1；J: n * (6+num)；para: 6 + num;
		//其中参数中的前6项为旋转矩阵，后num项为2次B样条曲线的参数值
		//整体流程为前向传递，反向传播，需要计算每一个步骤中的X，Y，DYX，DYP
		//流程1，旋转
		//输入 X，输出Y，参数：初始旋转平移矩阵，B样条函数参数，B样条其实点，终点，迭代次数
		double gama = 1E-10;
		double stride = 0.1;
		//测试一下参数的不同权重
		Eigen::MatrixXd E = Eigen::MatrixXd::Identity(Para.rows(), Para.rows());
		E = gama * E;
		Eigen::MatrixXd EStride = Eigen::MatrixXd::Identity(Para.rows(), Para.rows());
		EStride = 0.1 * EStride;
		Eigen::MatrixXd EStrideTrans = Eigen::MatrixXd::Identity(3, 3);
		EStrideTrans = 1E-0 * EStrideTrans;
		EStride.block(2, 2, 3, 3) = EStrideTrans;
		Eigen::MatrixXd EStrideRotate = Eigen::MatrixXd::Identity(3, 3);
		EStrideRotate = 1E-2 * EStrideRotate;
		EStride.block(0, 0, 3, 3) = EStrideRotate;

		Eigen::MatrixXd Y1, Y0;
		Eigen::MatrixXd error = Eigen::MatrixXd::Zero(iterateNum, 1);
		std::vector<Eigen::MatrixXd> ParaSet;
		for (size_t i = 0; i < iterateNum; i++)
		{
			Eigen::MatrixXd JYX, JYP;
			std::vector<Eigen::MatrixXd> outputSet = RevolvedBodyFitFlowTransferSingle(inputPoint, Y1, Para, tStart, tend);
			JYX = outputSet[0];
			JYP = outputSet[1];
			Y1 = outputSet[2];
			Eigen::MatrixXd JYPZero = Eigen::MatrixXd::Zero(JYP.rows(), 1);
			//JYP.block(0, 0, JYP.rows(), 1) = JYPZero;
			//JYP.block(0, 1, JYP.rows(), 1) = JYPZero;
			JYP.block(0, 2, JYP.rows(), 1) = JYPZero;
			JYP.block(0, 5, JYP.rows(), 1) = JYPZero;
			cv::Mat JYXMat, JYPMat, Y1Mat;
			JYXMat = Eigen2Mat(JYX);
			JYPMat = Eigen2Mat(JYP);
			Y1Mat = Eigen2Mat(Y1);
			//Y0 = Eigen::MatrixXd::Zero(Y1.rows(), Y1.cols());
			Eigen::MatrixXd detaA = (JYP.transpose() * JYP + E).inverse() * JYP.transpose() * (-Y1);
			detaA = EStride * detaA;
			cv::Mat detaAMat = Eigen2Mat(detaA);
			ParaSet.push_back(Para);
			//更新参数
			Para += detaA;
			error(i, 0) = (-Y1).norm();
			if (error(i, 0) < 1E-5)
			{
				break;
			}
			//if (i > 1&& error(i, 0) > error(i - 1, 0))
			{
				//stride *= 0.1;
			}
		}
		YEerror = Y1;
		cv::Mat Y1Mat = Eigen2Mat(Y1);
		cv::Mat paraMat = Eigen2Mat(Para);
		cv::Mat errorMat = Eigen2Mat(error);
		double minError = 1E10;
		for (size_t i = 0; i < error.rows(); i++)
		{
			if (abs(error(i, 0)) < minError)
			{
				minError = abs(error(i, 0));
				Para = ParaSet[i];
			}
		}
		return true;
	}
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2D(Eigen::MatrixXd& Para, double tStart, double tend, int interval)
	{
		//输入 Para (6 + n)*1
		//1 先按照BSplineCurve2D的方式生成点云，从tStart到tend
		//生成点云
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			if (i >= 6)
			{
				para.push_back(Para(i, 0));
			}
		}
		Eigen::Vector3d eularAngle = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		int num = (tend - tStart) / interval;
		pcl::PointCloud<pcl::PointXYZ> BSplineCloud;
		for (size_t i = 0; i < num; i++)
		{
			double t = tStart + i * interval;
			double output, derivativeYtoX;
			std::vector<double> derivativeYtoP;
			BSplineCureve2D(t, output, derivativeYtoX, derivativeYtoP, tStart, tend, para);
			BSplineCloud.push_back(pcl::PointXYZ(t, 0, output));
		}
		//2 逆向旋转生成的点云
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(eularAngle);
		Eigen::MatrixXd TransMatrix = HG::getTransformMatrix(rotationMatrix, trans);
		pcl::PointCloud<pcl::PointXYZ> outputCloud;
		pcl::transformPointCloud(BSplineCloud, outputCloud, static_cast<Eigen::Matrix4d>(TransMatrix.inverse()));
		return outputCloud;
	}
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2DTransfer(Eigen::MatrixXd& Para, double tStart, double tend, int interval)
	{
		//输入 Para (6 + n)*1
		//1 先按照BSplineCurve2D的方式生成点云，从tStart到tend
		//生成点云
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			if (i >= 6)
			{
				para.push_back(Para(i, 0));
			}
		}
		Eigen::Vector3d eularAngle = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		int num = (tend - tStart) / interval;
		pcl::PointCloud<pcl::PointXYZ> BSplineCloud;
		for (size_t i = 0; i < num; i++)
		{
			double t = tStart + i * interval;
			double output, derivativeYtoX;
			std::vector<double> derivativeYtoP;
			BSplineCureve2DTransfer(t, output, derivativeYtoX, derivativeYtoP, tStart, tend, para);
			BSplineCloud.push_back(pcl::PointXYZ(t, 0, output));
		}
		//2 逆向旋转生成的点云
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(eularAngle);
		Eigen::MatrixXd TransMatrix = HG::getTransformMatrix(rotationMatrix, trans);
		pcl::PointCloud<pcl::PointXYZ> outputCloud;
		pcl::transformPointCloud(BSplineCloud, outputCloud, static_cast<Eigen::Matrix4d>(TransMatrix.inverse()));
		return outputCloud;
	}

	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2D(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int interval)
	{
		//输入 Para (6 + n)*1
		//1 先按照BSplineCurve2D的方式生成点云，从tStart到tend
		//生成点云
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			if (i >= 6)
			{
				para.push_back(Para(i, 0));
			}
		}
		Eigen::Vector3d eularAngle = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(eularAngle);
		Eigen::MatrixXd TransMatrix = HG::getTransformMatrix(rotationMatrix, trans);
		int num = (tend - tStart) / interval;
		pcl::PointCloud<pcl::PointXYZ> BSplineCloud;
		for (size_t i = 0; i < inputPoint.cols(); i++)
		{
			Eigen::MatrixXd X = inputPoint.col(i);
			Eigen::MatrixXd XRotation = rotationMatrix * X + TransMatrix.block(0, 3, 3, 1);
			Eigen::Vector2d RZ = Eigen::Vector2d(pow(pow(XRotation(0, 0), 2) + pow(XRotation(1, 0), 2), 0.5), XRotation(2, 0));
			double t = RZ(0);
			double output, derivativeYtoX;
			std::vector<double> derivativeYtoP;
			BSplineCureve2D(t, output, derivativeYtoX, derivativeYtoP, tStart, tend, para);
			BSplineCloud.push_back(pcl::PointXYZ(t, 0, output));
		}
		//2 逆向旋转生成的点云

		pcl::PointCloud<pcl::PointXYZ> outputCloud = BSplineCloud;
		return outputCloud;
	}
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2DTransfer(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int interval)
	{
		//输入 Para (6 + n)*1
		//1 先按照BSplineCurve2D的方式生成点云，从tStart到tend
		//生成点云
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			if (i >= 6)
			{
				para.push_back(Para(i, 0));
			}
		}
		Eigen::Vector3d eularAngle = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(eularAngle);
		Eigen::MatrixXd TransMatrix = HG::getTransformMatrix(rotationMatrix, trans);
		int num = (tend - tStart) / interval;
		pcl::PointCloud<pcl::PointXYZ> BSplineCloud;
		for (size_t i = 0; i < inputPoint.cols(); i++)
		{
			Eigen::MatrixXd X = inputPoint.col(i);
			Eigen::MatrixXd XRotation = rotationMatrix * X + TransMatrix.block(0, 3, 3, 1);
			Eigen::Vector2d RZ = Eigen::Vector2d(pow(pow(XRotation(0, 0), 2) + pow(XRotation(1, 0), 2), 0.5), XRotation(2, 0));
			double t = RZ(0);
			double output, derivativeYtoX;
			std::vector<double> derivativeYtoP;
			BSplineCureve2DTransfer(t, output, derivativeYtoX, derivativeYtoP, tStart, tend, para);
			BSplineCloud.push_back(pcl::PointXYZ(t, 0, output));
		}
		//2 逆向旋转生成的点云

		pcl::PointCloud<pcl::PointXYZ> outputCloud = BSplineCloud;
		return outputCloud;
	}
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2D(Eigen::MatrixXd& inputPoint, pcl::PointCloud<pcl::PointXYZ>& outputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int interval)
	{
		outputPoint.clear();
		//输入 Para (6 + n)*1
		//1 先按照BSplineCurve2D的方式生成点云，从tStart到tend
		//生成点云
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			if (i >= 6)
			{
				para.push_back(Para(i, 0));
			}
		}
		Eigen::Vector3d eularAngle = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(eularAngle);
		Eigen::MatrixXd TransMatrix = HG::getTransformMatrix(rotationMatrix, trans);
		int num = (tend - tStart) / interval;
		pcl::PointCloud<pcl::PointXYZ> BSplineCloud;
		cv::Mat Y1Mat = cv::Mat::zeros(inputPoint.cols(), 1, CV_64F);
		for (size_t i = 0; i < inputPoint.cols(); i++)
		{
			Eigen::MatrixXd X = inputPoint.col(i);
			Eigen::MatrixXd XRotation = TransMatrix.block(0, 0, 3, 3) * X + TransMatrix.block(0, 3, 3, 1);
			Eigen::Vector2d RZ = Eigen::Vector2d(pow(pow(XRotation(0, 0), 2) + pow(XRotation(1, 0), 2), 0.5), XRotation(2, 0));
			double t = RZ(0);
			double output, derivativeYtoX;
			std::vector<double> derivativeYtoP;
			BSplineCureve2D(t, output, derivativeYtoX, derivativeYtoP, tStart, tend, para);
			BSplineCloud.push_back(pcl::PointXYZ(t, 0, output));
			outputPoint.push_back(pcl::PointXYZ(t, 0, XRotation(2, 0)));
			Y1Mat.at<double>(i, 0) = output - XRotation(2, 0);
		}
		//2 逆向旋转生成的点云

		pcl::PointCloud<pcl::PointXYZ> BPlineCloud = BSplineCloud;
		return BPlineCloud;
	}
	pcl::PointCloud<pcl::PointXYZ> genBSplineCurve2DTransfer(Eigen::MatrixXd& inputPoint, pcl::PointCloud<pcl::PointXYZ>& outputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int interval)
	{
		outputPoint.clear();
		//输入 Para (6 + n)*1
		//1 先按照BSplineCurve2D的方式生成点云，从tStart到tend
		//生成点云
		std::vector<double> para;
		for (size_t i = 0; i < Para.rows(); i++)
		{
			if (i >= 6)
			{
				para.push_back(Para(i, 0));
			}
		}
		Eigen::Vector3d eularAngle = Eigen::Vector3d(Para(0, 0), Para(1, 0), Para(2, 0));
		Eigen::Vector3d trans = Eigen::Vector3d(Para(3, 0), Para(4, 0), Para(5, 0));
		Eigen::Matrix3d rotationMatrix = HG::getRotationMatrix(eularAngle);
		Eigen::MatrixXd TransMatrix = HG::getTransformMatrix(rotationMatrix, trans);
		int num = (tend - tStart) / interval;
		pcl::PointCloud<pcl::PointXYZ> BSplineCloud;
		cv::Mat Y1Mat = cv::Mat::zeros(inputPoint.cols(), 1, CV_64F);
		for (size_t i = 0; i < inputPoint.cols(); i++)
		{
			Eigen::MatrixXd X = inputPoint.col(i);
			Eigen::MatrixXd XRotation = TransMatrix.block(0, 0, 3, 3) * X + TransMatrix.block(0, 3, 3, 1);
			Eigen::Vector2d RZ = Eigen::Vector2d(pow(pow(XRotation(0, 0), 2) + pow(XRotation(1, 0), 2), 0.5), XRotation(2, 0));
			double t = RZ(0);
			double output, derivativeYtoX;
			std::vector<double> derivativeYtoP;
			BSplineCureve2DTransfer(t, output, derivativeYtoX, derivativeYtoP, tStart, tend, para);
			BSplineCloud.push_back(pcl::PointXYZ(t, 0, output));
			outputPoint.push_back(pcl::PointXYZ(t, 0, XRotation(2, 0)));
			Y1Mat.at<double>(i, 0) = output - XRotation(2, 0);
		}
		//2 逆向旋转生成的点云

		pcl::PointCloud<pcl::PointXYZ> BPlineCloud = BSplineCloud;
		return BPlineCloud;
	}

	bool RevolvedBodyFitFlowLMRansac(Eigen::MatrixXd& inputPoint, Eigen::MatrixXd& Para, double tStart, double tend, int iterateNum, int selectNum, double threshold)
	{
		//用ransac来拟合回转体
		if (inputPoint.cols() < selectNum)
		{
			return false;
		}
		std::vector<Eigen::Vector3d> revolve_points_temp;
		std::vector<Eigen::Vector3d> revolve_points;
		double revolvedRasidue = 0, revolvedRasidueTemp;
		Eigen::MatrixXd ParaSelect;
		for (size_t i = 0; i < iterateNum; i++)
		{
			revolve_points_temp.clear();
			Eigen::MatrixXd pointsSet = Eigen::MatrixXd(3, selectNum);
			for (size_t i = 0; i < selectNum; i++)
			{
				pointsSet.col(i) = inputPoint.col(rand() % inputPoint.cols());
			}
			Eigen::MatrixXd ParaTemp = Para;
			Eigen::MatrixXd YErrorTemp;
			/*int index1 = rand() % points.size();
			int index2 = rand() % points.size();
			int index3 = rand() % points.size();
			Eigen::Vector3d point1 = points[index1];
			Eigen::Vector3d point2 = points[index2];
			Eigen::Vector3d point3 = points[index3];*/
			Eigen::Vector3d center_temp;
			Eigen::Vector3d normal_temp;
			double radius_temp;
			std::vector<double> distanceSet;
			if (RevolvedBodyFitFlowLM(pointsSet, ParaTemp, tStart, tend, 100, YErrorTemp))
			{
				pcl::PointCloud<pcl::PointXYZ> revolvedPointsZ;
				pcl::PointCloud<pcl::PointXYZ> revolvedPointsR = genBSplineCurve2D(inputPoint, revolvedPointsZ, ParaTemp, tStart, tend, ParaTemp.rows() - 6);
				for (size_t j = 0; j < revolvedPointsZ.size(); j++)
				{
					double distance = (Eigen::Vector3d(revolvedPointsZ[j].x, revolvedPointsZ[j].y, revolvedPointsZ[j].z) -
						Eigen::Vector3d(revolvedPointsR[j].x, revolvedPointsR[j].y, revolvedPointsR[j].z)).norm();
					if (abs(distance) < threshold)
					{
						distanceSet.push_back(distance);
						revolve_points_temp.push_back(Eigen::Vector3d(revolvedPointsZ[i].x, revolvedPointsZ[i].y, revolvedPointsZ[i].z));
					}
				}
				//pcl::PointCloud<pcl::PointXYZ> cloud1, cloud2;
				//cloud1 = getPCLPointXYZSet(points);
				//cloud2 = getPCLPointXYZSet(circle_points_temp);
				//showPointCloud(cloud1, cloud2, "cloud1");
				double sum_temp = varianceVector(distanceSet);
				if (revolve_points_temp.size() > revolve_points.size())
				{
					ParaSelect = ParaTemp;
					revolve_points = revolve_points_temp;
					revolvedRasidue = sum_temp;
				}
				if (revolve_points_temp.size() == revolve_points.size())
				{
					if (sum_temp <= revolvedRasidue)
					{
						ParaSelect = ParaTemp;
						revolve_points = revolve_points_temp;
						revolvedRasidue = sum_temp;
					}
				}
			}
		}
		Para = ParaSelect;
		return true;
	}

	//C3Dimage function
	//------------------
	C3Dimage::C3Dimage()
	{
		m_Interval = 0.2;
		length_xyz = { 3000, 4000, 3 };
		T_trans = { 400, 0, 0 };
		image3Data_cv.create(int(length_xyz[0]), int(length_xyz[1]), CV_32FC3);
		image3Fill_cv.create(int(length_xyz[0]), int(length_xyz[1]), CV_8U);
		image3Connect_cv.create(int(length_xyz[0]), int(length_xyz[1]), CV_16U);
		setZero();
	}
	C3Dimage::C3Dimage(std::vector<Eigen::Vector3d> cloud_vv)
	{
		*this = C3Dimage();
		initial(cloud_vv);
	}
	C3Dimage::~C3Dimage()
	{
	}
	Eigen::Vector3d C3Dimage::get3DimageCoordinate(Eigen::Vector3d point)
	{
		Eigen::Vector3d output;
		output[0] = int((point[0] + T_trans[0]) / m_Interval);
		output[1] = int((point[1] + T_trans[1]) / m_Interval);
		output[2] = int((point[2] + T_trans[2]) / m_Interval);
		if (output[0] < length_xyz[0] && output[0] >= 0 &&
			output[1] < length_xyz[1] && output[1] >= 0)
		{
			return output;
		}
		else
		{
			return Eigen::Vector3d(0, 0, 0);
		}
	}
	void C3Dimage::initial(std::vector<Eigen::Vector3d> cloud)
	{
		for (size_t i = 0; i < cloud.size(); i++)
		{
			Eigen::Vector3d coordinate = get3DimageCoordinate(cloud[i]);
			if (coordinate != Eigen::Vector3d(0, 0, 0))
			{
				image3Data_cv.at<cv::Vec<float, 3>>((int)coordinate[0], (int)coordinate[1])[0] = cloud[i][0];
				image3Data_cv.at<cv::Vec<float, 3>>((int)coordinate[0], (int)coordinate[1])[1] = cloud[i][1];
				image3Data_cv.at<cv::Vec<float, 3>>((int)coordinate[0], (int)coordinate[1])[2] = cloud[i][2];
				image3Fill_cv.at<uchar>((int)coordinate[0], (int)coordinate[1]) = 1;
			}
		}
	}
	void C3Dimage::setZero()
	{
		image3Fill_cv.setTo(0);
		image3Connect_cv.setTo(0);
	}

	void C3Dimage::getConnectRegion()
	{
		cv::Mat temtp = image3Fill_cv.clone();
		cv::Mat riceBW, out;
		cv::threshold(image3Fill_cv, riceBW, 0, 255, cv::THRESH_BINARY);
		int number = cv::connectedComponents(riceBW, out, 8, CV_16U);
		cluster_clouds.clear();
		for (size_t i = 0; i < number; i++)
		{
			std::vector<Eigen::Vector3d> tempt;
			cluster_clouds.push_back(tempt);
		}
		for (size_t i = 0; i < length_xyz[0]; i++)
		{
			for (size_t j = 0; j < length_xyz[1]; j++)
			{
				if (image3Fill_cv.at<uchar>(i, j) == 1)
				{
					Eigen::Vector3d tempt(image3Data_cv.at<cv::Vec<float, 3>>(i, j)[0], image3Data_cv.at<cv::Vec<float, 3>>(i, j)[1], image3Data_cv.at<cv::Vec<float, 3>>(i, j)[2]);
					cluster_clouds[out.at<unsigned short>(i, j)].push_back(tempt);
				}
			}
		}


	}
}