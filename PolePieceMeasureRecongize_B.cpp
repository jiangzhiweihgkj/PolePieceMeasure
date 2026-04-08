#include "PolePieceMeasureWin.h"
PolePieceMeasureWin::ThreePoint PolePieceMeasureWin::YIJie_B(pcl::PointCloud<pcl::PointXYZ>::Ptr  yijieCloud, int pinghuaxishu)//单独处理三节分成一节的情况
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pinghua_cloud_B = slidingAverageZ(yijieCloud, pinghuaxishu);
	pcl::PointCloud<pcl::PointXYZ>::Ptr nihe_cloud_B(new pcl::PointCloud<pcl::PointXYZ>);
	pinghua_cloud_B->width = yijieCloud->size();
	pinghua_cloud_B->height = 1;
	pinghua_cloud_B->is_dense = true;
	nihe_cloud_B = nihe_Roate(pinghua_cloud_B);
	//思路：判断凹槽最低点，然后找到凹槽的肩点；
	PolePieceMeasureWin::ThreePoint single_threePoint_B;
	double min_z = std::numeric_limits<double>::max();
	int min_index = -1;
	int down_idx = -1;


	//左肩点和右肩点的索引与底点的差值
	int left_2_bottom = -1;
	int right_2_bottom = -1;
	// 遍历所有点
	for (size_t i = 0; i < nihe_cloud_B->size(); ++i) {
		if (nihe_cloud_B->points[i].z < min_z) {
			min_z = nihe_cloud_B->points[i].z;
			min_index = i;
		}
	}
	down_idx = min_index;
	pcl::PointXYZ down_point;
	//if ((yijieCloud->points[0].z - yijieCloud->points[down_idx].z) / (yijieCloud->points[0].x - yijieCloud->points[down_idx].x)>-0.005)//-0.084
	bool ok = false;
	float didianThreshold = InfoSuanFa.editGaoDuChaThreshold.toFloat(); //ui->editbottom->text().toFloat(&ok);
	if ((nihe_cloud_B->points[0].z - nihe_cloud_B->points[down_idx].z) < didianThreshold)//-0.008
	{
		down_point = yijieCloud->points[down_idx];
		down_point.z = 0;
	}
	else
	{
		if (down_idx < 2)
		{
			down_idx = 2;
		}
		int max_index = -1;
		if (down_idx > nihe_cloud_B->points.size() - 2)
		//if (down_idx > 178)
		{
			max_index = nihe_cloud_B->points.size() - 1;
			//max_index = 179;
		}
		else
		{
			max_index = down_idx + 2;
		}
		//加一个二次定位的功能，那就是比较一下前面和后面的点的Z值，找Z值的最小值索引
		std::vector<double> min_Z;
		for (int i = down_idx - 2; i < max_index; i++)
		{
			min_Z.push_back(yijieCloud->points[i].z);
		}
		double z_min = std::numeric_limits<double>::max();
		int    i_min = -1;                 // 记录最小值对应的下标

		for (int i = down_idx - 2; i < max_index; ++i)
		{
			double z = yijieCloud->points[i].z;
			if (z < z_min)                 // 遇到更小的就更新
			{
				z_min = z;
				down_idx = i;
			}
		}
		down_point = yijieCloud->points[down_idx];
		sandian_suoyin_B.down_index.push_back(down_idx);
		single_threePoint_B.downShoulder = down_point;//*************低点***************** 
		
	    }
	     std::vector<double> left_xielv;
		int left_idx = 0;                 // 默认值 
		int bianlijiandianshu = ParamUi->editSlope->text().toInt();

		if (down_idx < bianlijiandianshu)
		{
			for (int i = down_idx; i > 1; --i)
			{

				double left_kerial = (nihe_cloud_B->points[down_idx].z - nihe_cloud_B->points[i - 1].z) / (nihe_cloud_B->points[down_idx].x - nihe_cloud_B->points[i - 1].x);
				left_xielv.push_back(left_kerial);
			}
		}
		else
		{
			for (int i = down_idx; i > down_idx - bianlijiandianshu; --i)
			{
				double left_kerial = (nihe_cloud_B->points[down_idx].z - nihe_cloud_B->points[i - 1].z) / (nihe_cloud_B->points[down_idx].x - nihe_cloud_B->points[i - 1].x);
				left_xielv.push_back(left_kerial);

			}
		}
		if (down_point.z != 0)
		{
			auto it = std::min_element(left_xielv.begin(), left_xielv.end());

			// 2. 最小值
			double min_val = *it;

			// 3. 对应的序列号（下标）
			left_idx = down_idx - std::distance(left_xielv.begin(), it);
			left_2_bottom = down_idx - left_idx; // 左肩到底的索引差值

			if (left_idx < 3)
			{
				LeftMostIndex_B = true;
				//left_idx = 0;
			}
			sandian_suoyin_B.left_index.push_back(left_idx); 
			pcl::PointXYZ left_point = yijieCloud->points[left_idx];//***********左点*******************
			single_threePoint_B.leftShoulder = left_point;
		}
		std::vector<double> right_xielv;
		int right_idx = nihe_cloud_B->size() - 1;                 // 默认值 
		if (down_idx + bianlijiandianshu < nihe_cloud_B->size() - 1)
		{
			for (int i = down_idx; i < down_idx + bianlijiandianshu; ++i)
			{
				double right_kerial = (nihe_cloud_B->points[down_idx].z - nihe_cloud_B->points[i + 1].z) / (nihe_cloud_B->points[down_idx].x - nihe_cloud_B->points[i + 1].x);
				right_xielv.push_back(right_kerial); 
			}
		}
		else
		{
			for (int i = down_idx; i < nihe_cloud_B->size() - 1; ++i)
			{
				double right_kerial = (nihe_cloud_B->points[down_idx].z - nihe_cloud_B->points[i + 1].z) / (nihe_cloud_B->points[down_idx].x - nihe_cloud_B->points[i + 1].x);
				right_xielv.push_back(right_kerial); 
			}
		}
		if (down_point.z != 0)
		{
			auto it_r = std::max_element(right_xielv.begin(), right_xielv.end());

			double max_val = *it_r;                       // 最大值
			right_idx = down_idx + std::distance(right_xielv.begin(), it_r); // 对应下标
			//增加一个二次定位，利用夹角进行判断
			right_2_bottom = right_idx - down_idx;
			/*if (right_2_bottom<8)
			{
				right_idx = right_idx + 6;
			}*/
			if (right_idx > nihe_cloud_B->size() - 4)
				//if (right_idx==nihe_cloud->size()-1)
			{
				RightMostIndex_B = true;
				//right_idx = nihe_cloud->size() - 1;
			}
			sandian_suoyin_B.right_index.push_back(right_idx);
			pcl::PointXYZ right_point = yijieCloud->points[right_idx];//***********右点*******************
			single_threePoint_B.rightShoulder = right_point;
		}
		return single_threePoint_B;

}

PolePieceMeasureWin::ThreePoint  PolePieceMeasureWin::singleRecongize_B(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  threeJieCloud, int pinghuaxishu)//逐帧处理
{
	PolePieceMeasureWin::ThreePoint vectorThreePoints_B;
	// 左肩点和右肩点的距离
	auto pointDistance = [](const pcl::PointXYZ& p1,
		const pcl::PointXYZ& p2) -> float
		{
			return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
				(p1.y - p2.y) * (p1.y - p2.y) +
				(p1.z - p2.z) * (p1.z - p2.z));
		};
	//点p到a,b连线的距离
	auto pointToLineDistance3D =
		[](const pcl::PointXYZ& a,
			const pcl::PointXYZ& b,
			const pcl::PointXYZ& p) -> float
		{
			// 向量 ab
			float abx = b.x - a.x;
			float aby = b.y - a.y;
			float abz = b.z - a.z;

			// 向量 ap
			float apx = p.x - a.x;
			float apy = p.y - a.y;
			float apz = p.z - a.z;

			// 点积 ab·ap
			float dot = abx * apx + aby * apy + abz * apz;

			// ab 长度平方
			float len2 = abx * abx + aby * aby + abz * abz;
			if (len2 < std::numeric_limits<float>::epsilon())
				return std::sqrt(apx * apx + apy * apy + apz * apz); // a==b，退化为到点距离

			// 投影比例 t
			float t = dot / len2;
			t = std::fmax(0.0f, std::fmin(1.0f, t));               // 钳位到线段内

			// 垂足 q = a + t*ab
			float qx = a.x + t * abx;
			float qy = a.y + t * aby;
			float qz = a.z + t * abz;

			// 返回 p 到 q 的距离
			float dx = p.x - qx;
			float dy = p.y - qy;
			float dz = p.z - qz;
			return std::sqrt(dx * dx + dy * dy + dz * dz);
		};
	for (int i = 0; i < threeJieCloud.size(); ++i)
	{
		PolePieceMeasureWin::ThreePoint single_threePoint_B;
		single_threePoint_B = YIJie_B(threeJieCloud[i], pinghuaxishu);
		if (single_threePoint_B.downShoulder.z != 0)
		{
			vectorThreePoints_B = single_threePoint_B;
			//屏蔽掉  后面会算一遍   粗定位先不算了
			double width_B = pointDistance(single_threePoint_B.leftShoulder, single_threePoint_B.rightShoulder);
			double depth_B = pointToLineDistance3D(single_threePoint_B.leftShoulder, single_threePoint_B.rightShoulder, single_threePoint_B.downShoulder);
			width_B =1000 * width_B + InfoSuanFa.edit_Width_B_Coefficient.toFloat();
			depth_B = 1000 * depth_B + InfoSuanFa.edit_Depth_B_Coefficient.toFloat();
			Calucate_Width_Depth_B.m_width.push_back(width_B);
			Calucate_Width_Depth_B.m_depth.push_back(depth_B);
			_callOneTimes->success_B = true;
			break; //找到了就跳出循环
		}
		else
		{
			_callOneTimes->success_B = false; 
		}
	} 
	return vectorThreePoints_B;
}
bool PolePieceMeasureWin::multifyRecognize_B(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  multifyCloud, int pinghuaxishu)//多帧处理
{
	if (multifyCloud.empty())
	{
		return false;
	}
	int sizenum = multifyCloud.size();
	if (ui->checkbox_roi->isChecked())
	{
		sizenum = multifyCloud.size() - InfoSuanFa.editCheckThreshold.toInt();
	}
	else
	{
		sizenum = multifyCloud.size() - InfoSuanFa.editCheckThreshold.toInt();
	}
	std::vector<PolePieceMeasureWin::ThreePoint> vectorThreePoints_B;
	vectorThreePoints_B.clear();
	for (int i = 0; i < sizenum; ++i)
	{
		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> san_jie;
		san_jie.clear();
		san_jie = NineJie(multifyCloud[i]);//一帧分9块
		PolePieceMeasureWin::ThreePoint single_threePoint = singleRecongize_B(san_jie, pinghuaxishu);//单帧识别
		if (_callOneTimes->success_B)
		{
			ID_SELECT_B.push_back(i);

			//二次定位  
			auto pointDistance = [](const pcl::PointXYZ& p1,
				const pcl::PointXYZ& p2) -> float
				{
					return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
						(p1.y - p2.y) * (p1.y - p2.y) +
						(p1.z - p2.z) * (p1.z - p2.z));
				};
			//点p到a,b连线的距离
			auto pointToLineDistance3D =
				[](const pcl::PointXYZ& a,
					const pcl::PointXYZ& b,
					const pcl::PointXYZ& p) -> float
				{
					// 向量 ab
					float abx = b.x - a.x;
					float aby = b.y - a.y;
					float abz = b.z - a.z;

					// 向量 ap
					float apx = p.x - a.x;
					float apy = p.y - a.y;
					float apz = p.z - a.z;

					// 点积 ab·ap
					float dot = abx * apx + aby * apy + abz * apz;

					// ab 长度平方
					float len2 = abx * abx + aby * aby + abz * abz;
					if (len2 < std::numeric_limits<float>::epsilon())
						return std::sqrt(apx * apx + apy * apy + apz * apz); // a==b，退化为到点距离

					// 投影比例 t
					float t = dot / len2;
					t = std::fmax(0.0f, std::fmin(1.0f, t));               // 钳位到线段内

					// 垂足 q = a + t*ab
					float qx = a.x + t * abx;
					float qy = a.y + t * aby;
					float qz = a.z + t * abz;

					// 返回 p 到 q 的距离
					float dx = p.x - qx;
					float dy = p.y - qy;
					float dz = p.z - qz;
					return std::sqrt(dx * dx + dy * dy + dz * dz);
				};



			int left_x_index = single_threePoint.leftShoulder.x / 0.003;
			int right_x_index = single_threePoint.rightShoulder.x / 0.003;
			pcl::PointCloud<pcl::PointXYZ>::Ptr ErCi_Calculate(new pcl::PointCloud<pcl::PointXYZ>);
			ErCi_Calculate = separate_clouds_B[i];
			std::vector<double> left_xielv;
			std::vector<double> right_xielv;
			for (int j = left_x_index; j > left_x_index - 10; --j)
			{
				double left_kerial = (ErCi_Calculate->points[j].z - ErCi_Calculate->points[j - 1].z) / (ErCi_Calculate->points[j].x - ErCi_Calculate->points[j - 1].x);
				left_xielv.push_back(left_kerial);
			}
			for (int j = right_x_index; j < right_x_index + 10; ++j)
			{
				double right_kerial = (ErCi_Calculate->points[j].z - ErCi_Calculate->points[j + 1].z) / (ErCi_Calculate->points[j].x - ErCi_Calculate->points[j + 1].x);
				right_xielv.push_back(right_kerial);
			}
			double min_left_kerial = *std::min_element(left_xielv.begin(), left_xielv.end());
			int min_left_index = std::distance(left_xielv.begin(), std::min_element(left_xielv.begin(), left_xielv.end()));
			double	min_right_kerial = *std::min_element(right_xielv.begin(), right_xielv.end());
			int min_right_index = 0;
			double left_xielv_control = ParamUi->editRadius->text().toDouble();
			left_xielv_control = left_xielv_control * M_PI / 180.0;   // 度→弧度
			left_xielv_control = std::tan(left_xielv_control);        // 0.5773502691896256

			double right_xielv_control = ParamUi->editTorlance->text().toDouble();
			right_xielv_control = right_xielv_control * M_PI / 180.0;   // 度→弧度
			right_xielv_control = std::tan(right_xielv_control);        // 0.5773502691896256
			//找到left_xielv最小值的下标>=0.008
			for (int j = 0; j < left_xielv.size(); ++j)
			{
				if (left_xielv[j] >= left_xielv_control)
				{
					min_left_kerial = left_xielv[j];
					min_left_index = left_x_index - j;
					break;
				}
				else
				{
					auto it = std::max_element(left_xielv.begin(), left_xielv.end());
					int idx_min = std::distance(left_xielv.begin(), it);
					min_left_index = left_x_index - idx_min;
				}
			}
			for (int i = 0; i < right_xielv.size(); i++)
			{
				if (right_xielv[i] <= right_xielv_control)
				{
					min_right_kerial = right_xielv[i];
					min_right_index = right_x_index + i;
					break;
				}
				else
				{
					auto it = std::min_element(right_xielv.begin(), right_xielv.end());
					int idx_max = std::distance(right_xielv.begin(), it);
					min_right_index = right_x_index + idx_max;
				}
			}
			single_threePoint.leftShoulder = separate_clouds_B[i]->points[min_left_index];
			single_threePoint.rightShoulder = separate_clouds_B[i]->points[min_right_index];
			single_threePoint.downShoulder = single_threePoint.downShoulder; 
			vectorThreePoints_B.push_back(single_threePoint);

		} 
	}
	if (vectorThreePoints_B.empty())
	{
		return false;//识别失败
	}
	_multifyRecognize_B = vectorThreePoints_B;
	return true; //成功识别 
}
bool PolePieceMeasureWin::StartRecongize_B(pcl::PointCloud<pcl::PointXYZ>::Ptr source_Cloud)
{
	Init_Clear_B();//后面再加上B这边需要初始化的量
	int pinghuaxishu = InfoSuanFa.editPingHuaXiShu.toInt(); //ui->editPingHuaData->text().toInt();  
	separate_clouds_B = separateCloud(source_cloud_B);

	_separateCloud_B = separateCloud(source_Cloud);//将一张带有多行点云分离成y值一致的单行点云 
	bool multyResult_B = multifyRecognize_B(_separateCloud_B, pinghuaxishu);//多组点云循环依次执行 
	return multyResult_B;
}
void PolePieceMeasureWin::on_btnRecoCloud_clicked()//导入点云B
{
	// 打开文件选择对话框，支持的格式为PLY和PCD
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Point Cloud File"), "", tr("Point Cloud Files (*.ply *.pcd)"));

	// 如果未选择文件，则返回
	if (fileName.isEmpty()) {
		return;
	}

	// 获取文件扩展名，以区分文件格式
	QFileInfo fileInfo(fileName);
	QString ext = fileInfo.suffix().toLower();

	 Init_Clear_B();
	source_cloud_B.reset(new pcl::PointCloud<pcl::PointXYZ>);
	source_cloud_B->clear(); separate_clouds_B.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ui->Single_B_PointShow->setPointCloud(cloud);

	// 根据扩展名加载相应格式的点云文件
	int result = -1;
	if (ext == "ply")
	{
		result = pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toStdString(), *cloud);
	}
	else if (ext == "pcd")
	{
		result = pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud);
	}
	else
	{
		return;
	}

	// 检查文件加载是否成功
	if (result != 0)
	{
		QMessageBox::critical(this, tr("Error"), tr("无法打开点云文件（路径需要全英文）"));
		return;
	}
	source_cloud_B = cloud; 
	roi_cloud_B = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	roiPercent_B = InfoSuanFa.editRoiPercent.toDouble();
	centerPercent_B = InfoSuanFa.editCentrePoint.toDouble();
	roi_cloud_B = roiAcquire(source_cloud_B, roiPercent_B, centerPercent_B);//划定一个ROI范围
	ui->tableFrame_B->clearContents();    // 清空单元格内容，表头不变
	ui->tableFrame_B->setRowCount(0);    // 设置为0行，彻底清空内容，表头依然会显示
	if (ui->checkbox_roi->isChecked())
	{
		//ui->openGLWidget_Result_B->setCameraUpVector(0,1,0);
	//	ui->openGLWidget_Result_B->showPointCloud(roi_cloud_B);
		updateViewer_B(roi_cloud_B);
	}
	else
	{
		//ui->openGLWidget_Result_B->setCameraUpVector(0, 1, 0);
		//ui->openGLWidget_Result_B->showPointCloud(source_cloud_B);
		updateViewer_B(source_cloud_B);
	}
	if (ui->checkBox_auto->isChecked())
	{
		on_btnTestShow_clicked();
	}
}
void PolePieceMeasureWin::on_btnTestShow_clicked()//识别
{
	/* Show_CurrentAB_Table_Show();*/
	 //show_Check_Result_OK_NG();
	// tableAB_Show();
   //  Point_Show();
	if (source_cloud_B->points.empty())
	{
		QMessageBox::warning(this, tr("Warning"), tr("Please load a point cloud B."));
		return;
	}
	iniSettingParam();
	roiPercent_B = InfoSuanFa.editRoiPercent.toDouble();
	centerPercent_B = InfoSuanFa.editCentrePoint.toDouble();
	for (int i = 0; i < 5; i++)
	{
		roi_cloud_B = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		roi_cloud_B = roiAcquire(source_cloud_B, roiPercent_B, centerPercent_B);//划定一个ROI范围
		bool result = StartRecongize_B(roi_cloud_B);//改成void类型
		//bool RightMostIndex = false;
		if (RightMostIndex_B)
		{
			centerPercent_B = centerPercent_B + 5;
		}

		if (LeftMostIndex_B)
		{
			centerPercent_B = centerPercent_B - 5;
		}
		if (!RightMostIndex_B && !LeftMostIndex_B)
		{
			break;
		}
	}
	if (Calucate_Width_Depth_B.m_width.size() < 80)
	{
		ui->Result_B->setText("NG");
		ui->Result_B->setStyleSheet("color: rgb(255,0,0);");          // 红色 
		return;
	}
	Result_B_Show();
	new_B_updateTable(); tableAB_Show();
	ShowThreeVectorPoint_B(ThreePointToRGBcloud_B(_multifyRecognize_B));
	Show_CurrentAB_Table_Show_B();
}
PolePieceMeasureWin::ThreePoint2RGBcloud  PolePieceMeasureWin::ThreePointToRGBcloud_B(std::vector<PolePieceMeasureWin::ThreePoint> _vectorThreePoint)
{
	for (int i = 0; i < _vectorThreePoint.size(); ++i)
	{
		const auto& _ThreePoint = _vectorThreePoint[i];
		pcl::PointXYZRGB left_array_RGB;
		pcl::PointXYZRGB right_array_RGB;
		pcl::PointXYZRGB down_array_RGB;
		left_array_RGB.x = _ThreePoint.leftShoulder.x;
		left_array_RGB.y = _ThreePoint.leftShoulder.y;
		left_array_RGB.z = _ThreePoint.leftShoulder.z;
		left_array_RGB.r = 0;
		left_array_RGB.g = 255;
		left_array_RGB.b = 0;

		right_array_RGB.x = _ThreePoint.rightShoulder.x;
		right_array_RGB.y = _ThreePoint.rightShoulder.y;
		right_array_RGB.z = _ThreePoint.rightShoulder.z;
		right_array_RGB.r = 0;
		right_array_RGB.g = 255;
		right_array_RGB.b = 0;

		down_array_RGB.x = _ThreePoint.downShoulder.x;
		down_array_RGB.y = _ThreePoint.downShoulder.y;
		down_array_RGB.z = _ThreePoint.downShoulder.z;
		down_array_RGB.r = 255;
		down_array_RGB.g = 0;
		down_array_RGB.b = 0;

		_ThreePoint2RGBcloud_B.RGB_left_Cloud->points.push_back(left_array_RGB);
		_ThreePoint2RGBcloud_B.RGB_right_Cloud->points.push_back(right_array_RGB);
		_ThreePoint2RGBcloud_B.RGB_down_Cloud->points.push_back(down_array_RGB);
	}
 	return _ThreePoint2RGBcloud_B;
}
void PolePieceMeasureWin::ShowThreeVectorPoint_B(ThreePoint2RGBcloud _ThreePoint2RGBcloud)
{
	if (ui->checkbox_roi->isChecked())
	{
		updateViewer_B(roi_cloud_B, _ThreePoint2RGBcloud.RGB_left_Cloud, _ThreePoint2RGBcloud.RGB_right_Cloud, _ThreePoint2RGBcloud.RGB_down_Cloud);
	}
	else
	{
		updateViewer_B(source_cloud_B, _ThreePoint2RGBcloud.RGB_left_Cloud, _ThreePoint2RGBcloud.RGB_right_Cloud, _ThreePoint2RGBcloud.RGB_down_Cloud);
	}
}
void PolePieceMeasureWin::on_btnStartCollectPoint_B_clicked()//开始采集
{

}
void PolePieceMeasureWin::on_btnStopCollectPoint_B_clicked()//停止采集
{
	if (ParamUi->CameraConnectState->text() != QStringLiteral("connected"))
	{
		QMessageBox::warning(this, QStringLiteral("Tips"), QStringLiteral("A相机未连接"));
		return;
	}

	if (ParamUi->CameraConnectState_B->text() != QStringLiteral("connected"))
	{
		QMessageBox::warning(this, QStringLiteral("Tips"), QStringLiteral("B相机未连接"));
		return;
	}
	_callOneTimes->ThreadStatus = CallOneTimes::SYS_CAMERA_STOP_COLLECT;
	_callOneTimes_B->ThreadStatus = CallOneTimes_B::SYS_CAMERA_STOP_COLLECT;
}
void PolePieceMeasureWin::updateViewer_B(
	pcl::PointCloud<pcl::PointXYZ>::Ptr show_source_cloud ,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_left,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_right,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_down_Points)
{
	if (!viewer_B) return; // 防止空指针崩溃
	viewer_B->removeAllPointClouds();//将窗口里面所有点云清空

	// 优先使用传入的点云参数，否则用成员变量（默认显示）
	pcl::PointCloud<pcl::PointXYZ>::Ptr sc = show_source_cloud ? show_source_cloud : source_cloud_B;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr upl = show_up_Points_left ? show_up_Points_left : _ThreePoint2RGBcloud_B.RGB_left_Cloud;//点云类型是pcl::PointXYZRGB（包含XYZ坐标和RGB颜色信息）
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr upr = show_up_Points_right ? show_up_Points_right : _ThreePoint2RGBcloud_B.RGB_right_Cloud;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr dp = show_down_Points ? show_down_Points : _ThreePoint2RGBcloud_B.RGB_down_Cloud;

	if (sc && !sc->empty())
	{
		if (!viewer_B->contains("source_cloud"))
		{
			viewer_B->addPointCloud<pcl::PointXYZ>(sc, "source_cloud");//给这个点云设置一个标识符"source_cloud";使用viewer->addPointCloud将点云数据添加到可视化窗口中
		}
	}

	int ptSize = 15;
	// 使用setPointCloudRenderingProperties设置这个点云的渲染属性，具体是点的大小（PCL_VISUALIZER_POINT_SIZE）为ptSize变量指定的值

	if (upl && !upl->empty())
	{
		if (!viewer_B->contains("up_Points_left"))
		{
			viewer_B->addPointCloud<pcl::PointXYZRGB>(upl, "up_Points_left");//给这个点云设置一个标识符"up_Points_left";使用viewer->addPointCloud将点云数据添加到可视化窗口中
			viewer_B->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, "up_Points_left");
		}
	}
	if (upr && !upr->empty())
	{
		if (!viewer_B->contains("up_Points_right"))
		{
			viewer_B->addPointCloud<pcl::PointXYZRGB>(upr, "up_Points_right");//给这个点云设置一个标识符"up_Points_right";使用viewer->addPointCloud将点云数据添加到可视化窗口中
			viewer_B->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, "up_Points_right");
		}
	}
	if (dp && !dp->empty())
	{
		if (!viewer_B->contains("down_Points"))
		{
			viewer_B->addPointCloud<pcl::PointXYZRGB>(dp, "down_Points");//给这个点云设置一个标识符"down_Points";使用viewer->addPointCloud将点云数据添加到可视化窗口中
			viewer_B->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, "down_Points");
		}
	}

	if (sc && !sc->empty())
	{
		pcl::PointXYZ min_pt, max_pt;
		pcl::getMinMax3D(*sc, min_pt, max_pt);//把点云 *sc 里所有点的 x/y/z 坐标各扫一遍，找出三个方向上的最小值和最大值，结果放进 min_pt 和 max_pt。
		double length_x = (max_pt.x - min_pt.x) / 3.0;
		double length_y = (max_pt.y - min_pt.y) / 3.0;
		double length_z = (max_pt.z - min_pt.z) / 3.0;
		double length = (length_x + length_y + length_z) / 3.0;

		ui->openGLWidget_Result_B->setAxesLength(length, length, length);

		double center_x = (min_pt.x + max_pt.x) / 2.0;
		double center_y = (min_pt.y + max_pt.y) / 2.0;
		double center_z = (min_pt.z + max_pt.z) / 2.0;

		bool specifiedView = (show_source_cloud || show_up_Points_left || show_up_Points_right || show_down_Points);

		if (specifiedView || upl->points.size() == 1)
		{
			// XZ平面视角，从Y轴正方向往下看
			double cam_x = center_x;
			double cam_y = center_y - length * 10; // 适当提高摄像机高度
			double cam_z = center_z;

			// 设置摄像机位置，焦点，up向量
			viewer_B->setCameraPosition(
				cam_x, cam_y, cam_z,        // 摄像机位置在Y轴正方向
				center_x, center_y, center_z, // 目标点仍是中心点
				0.0, 0.0, 1.0);            // up向量设置为正Z方向，保证视图不旋转
		}
		else
		{
			// 默认视角：从Z轴正方向看向点云
			double cam_x = center_x;
			double cam_y = center_y;
			double cam_z = max_pt.z + length_y * 10;

			viewer_B->setCameraPosition(
				cam_x, cam_y, cam_z,
				center_x, center_y, center_z,
				0.0, 1.0, 0.0);
		}
	}

	ui->openGLWidget_Result_B->update();
	viewer_B->getRenderWindow()->Render();


}
void PolePieceMeasureWin::Result_B_Show()//结果显示
{
	int idx_max = 0;
	if (!Calucate_Width_Depth_B.m_width.empty()) {
		auto minmax_width = std::minmax_element(Calucate_Width_Depth_B.m_width.begin(), Calucate_Width_Depth_B.m_width.end());
		Calucate_Width_Depth_B.m_min_width = *minmax_width.first;
		Calucate_Width_Depth_B.m_max_width = *minmax_width.second;
		Calucate_Width_Depth_B.m_mean_width = std::accumulate(Calucate_Width_Depth_B.m_width.begin(), Calucate_Width_Depth_B.m_width.end(), 0.0) / Calucate_Width_Depth_B.m_width.size();
 	}
	if (!Calucate_Width_Depth_B.m_depth.empty()) {
		auto minmax_depth = std::minmax_element(Calucate_Width_Depth_B.m_depth.begin(), Calucate_Width_Depth_B.m_depth.end());
		Calucate_Width_Depth_B.m_min_depth = *minmax_depth.first;
		Calucate_Width_Depth_B.m_max_depth = *minmax_depth.second;
		Calucate_Width_Depth_B.m_mean_depth = std::accumulate(Calucate_Width_Depth_B.m_depth.begin(), Calucate_Width_Depth_B.m_depth.end(), 0.0) / Calucate_Width_Depth_B.m_depth.size();
	}
	auto ToPoint2Data = [](double v) -> QString {
		std::ostringstream o;
		o << std::fixed << std::setprecision(2) << v << "um ";
		return QString::fromStdString(o.str());
		};
	QString meanWidth = ToPoint2Data(1.000 * Calucate_Width_Depth_B.m_mean_width);
	QString maxWidth = ToPoint2Data(1.000 * Calucate_Width_Depth_B.m_max_width);
	QString minWidth = ToPoint2Data(1.000 * Calucate_Width_Depth_B.m_min_width);

	QString meanDepth = ToPoint2Data(1.000 * Calucate_Width_Depth_B.m_mean_depth);
	QString maxDepth = ToPoint2Data(1.000 * Calucate_Width_Depth_B.m_max_depth);
	QString minDepth = ToPoint2Data(1.000 * Calucate_Width_Depth_B.m_min_depth);
	qInfo() << ("meanWidth:" + QString::number(1.000 * Calucate_Width_Depth_B.m_mean_width, 'f', 2))
		<< ("maxWidth:" + QString::number(1.000 * Calucate_Width_Depth_B.m_max_width, 'f', 2))
		<< ("minWidth:" + QString::number(1.000 * Calucate_Width_Depth_B.m_min_width, 'f', 2));

	qInfo() << ("meanDepth:" + QString::number(1.000 * Calucate_Width_Depth_B.m_mean_depth, 'f', 2))
		<< ("maxDepth:" + QString::number(1.000 * Calucate_Width_Depth_B.m_max_depth, 'f', 2))
		<< ("minDepth:" + QString::number(1.000 * Calucate_Width_Depth_B.m_min_depth, 'f', 2));

	_callOneTimes->class_A_B.Result_B.Measure_Depth_Result = 1.000 * Calucate_Width_Depth_B.m_mean_depth;
	_callOneTimes->class_A_B.Result_B.Measure_Width_Result = 1.000 * Calucate_Width_Depth_B.m_mean_width;
	//    /* 第一列：宽度 */ 
	ui->tb_Widget_B->setItem(0, 0, new QTableWidgetItem(QStringLiteral("平均宽度")));
	ui->tb_Widget_B->setItem(1, 0, new QTableWidgetItem(QStringLiteral("最大宽度")));
	ui->tb_Widget_B->setItem(2, 0, new QTableWidgetItem(QStringLiteral("最小宽度")));
	ui->tb_Widget_B->setItem(0, 1, new QTableWidgetItem(meanWidth));
	ui->tb_Widget_B->setItem(1, 1, new QTableWidgetItem(maxWidth));
	ui->tb_Widget_B->setItem(2, 1, new QTableWidgetItem(minWidth)); 
	/* 第二列：空白，不用填 */
	/* 第三列：深度 */
	ui->tb_Widget_B->setItem(0, 2, new QTableWidgetItem(QStringLiteral("平均深度")));
	ui->tb_Widget_B->setItem(1, 2, new QTableWidgetItem(QStringLiteral("最大深度")));
	ui->tb_Widget_B->setItem(2, 2, new QTableWidgetItem(QStringLiteral("最小深度")));
	ui->tb_Widget_B->setItem(0, 3, new QTableWidgetItem(meanDepth));
	ui->tb_Widget_B->setItem(1, 3, new QTableWidgetItem(maxDepth));
	ui->tb_Widget_B->setItem(2, 3, new QTableWidgetItem(minDepth));
	auto setStyle = [](QLabel* label, bool highlight) {
		if (!highlight)
			label->setStyleSheet("color: red; font-weight: bold;");
		else
			label->setStyleSheet("color: rgb(0, 234, 0);");
		};
	bool B_Width_R = InfoSuanFa.editDetectedWidthMin.toDouble() <= 1.000 * Calucate_Width_Depth_B.m_mean_width && 1.000 * Calucate_Width_Depth_B.m_mean_width <= InfoSuanFa.editDetectedWidthMax.toDouble();
	bool B_Depth_R = InfoSuanFa.editDetectedDepthMin.toDouble() <= 1.000 * Calucate_Width_Depth_B.m_mean_depth && 1.000 * Calucate_Width_Depth_B.m_mean_depth <= InfoSuanFa.editDetectedDepthMax.toDouble();
	bool Fina_R = B_Width_R && B_Depth_R;
	if (Fina_R)
	{
		ui->Result_B->setText("OK");
		ui->Result_B->setStyleSheet("color: rgb(0,255,0);");     // 绿色
	}
	else
	{
		ui->Result_B->setText("NG");
		ui->Result_B->setStyleSheet("color: rgb(255,0,0);");          // 红色 
	}

	customPts_B << QPointF(_multifyRecognize_B[ID_SELECT_B[idx_max]].leftShoulder.x, _multifyRecognize_B[ID_SELECT_B[idx_max]].leftShoulder.z)
		<< QPointF(_multifyRecognize_B[ID_SELECT_B[idx_max]].rightShoulder.x, _multifyRecognize_B[ID_SELECT_B[idx_max]].rightShoulder.z);
	customPts_bottom_B << QPointF(_multifyRecognize_B[ID_SELECT_B[idx_max]].downShoulder.x, _multifyRecognize_B[ID_SELECT_B[idx_max]].downShoulder.z);
	//显示单帧的时候先找到最低点那个位置的X值，然后将可以映射回原来的点云的序列号，然后根据序列号来截取ROI来进行显示；//separate_clouds
	int index = qRound(_multifyRecognize_B[ID_SELECT_B[idx_max]].downShoulder.x / 0.003);
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
	/*for (int i = index - 440; i < index + 440;++i)
	{
		roi_show->points.push_back(	separate_clouds[ID_SELECT[0]]->points[i]);
	}*/
	std::size_t start = index - 45;
	std::size_t end = index + 45;
	slice->points.assign(separate_clouds_B[ID_SELECT_B[idx_max]]->points.begin() + start,
		separate_clouds_B[ID_SELECT_B[idx_max]]->points.begin() + end);
	slice->width = slice->points.size();
	slice->height = 1;
	slice->is_dense = separate_clouds_B[ID_SELECT_B[idx_max]]->is_dense;
	ui->Single_B_PointShow->addCustomPoints(customPts_B, Qt::green, 5);
	ui->Single_B_PointShow->addCustomPoints(customPts_bottom_B, Qt::red, 5);
	ui->Single_B_PointShow->setPointCloud(slice);




}
void PolePieceMeasureWin::Show_CurrentAB_Table_Show_B()
{
	
	if (InfoSuanFa.editDetectedWidthMin.toDouble() < 1.000 * Calucate_Width_Depth_B.m_mean_width && 1.000 * Calucate_Width_Depth_B.m_mean_width < InfoSuanFa.editDetectedWidthMax.toDouble()
		&& InfoSuanFa.editDetectedDepthMin.toDouble() < 1.000 * Calucate_Width_Depth_B.m_mean_depth && 1.000 * Calucate_Width_Depth_B.m_mean_depth < InfoSuanFa.editDetectedDepthMax.toDouble())
	{
		_callOneTimes->class_A_B.Result_B.Number_Check_Result_OK = _callOneTimes->class_A_B.Result_B.Number_Check_Result_OK + 1;
	}
	else
	{
		_callOneTimes->class_A_B.Result_B.Number_Check_Result_NG = _callOneTimes->class_A_B.Result_B.Number_Check_Result_NG + 1;
	}
	int all_number_OK = _callOneTimes->class_A_B.Result_B.Number_Check_Result_OK;
	QTableWidgetItem* widthItem = new QTableWidgetItem(QString::number(all_number_OK));
	ui->tableAB_Show_Data->setItem(1, 0, widthItem);

	int all_number_NG = _callOneTimes->class_A_B.Result_B.Number_Check_Result_NG;
	QTableWidgetItem* widthItem1 = new QTableWidgetItem(QString::number(all_number_NG));
	ui->tableAB_Show_Data->setItem(1, 1, widthItem1);
	_callOneTimes->class_A_B.Result_B.Number_Check_Result_ALL = all_number_OK + all_number_NG;
	int all_number = _callOneTimes->class_A_B.Result_B.Number_Check_Result_ALL;
	QTableWidgetItem* widthItem2 = new QTableWidgetItem(QString::number(all_number));
	ui->tableAB_Show_Data->setItem(1, 2, widthItem2);

	_callOneTimes->class_A_B.Result_B.Yield_Check_Result_OK = all_number_OK * 1.0 / all_number;
	double all_number_Yield = _callOneTimes->class_A_B.Result_B.Yield_Check_Result_OK;
	QTableWidgetItem* widthItem3 = new QTableWidgetItem(QString::number(all_number_Yield * 100, 'f', 2) + QStringLiteral("%"));
	ui->tableAB_Show_Data->setItem(1, 3, widthItem3);

	if (1.000 * Calucate_Width_Depth_B.m_mean_depth<InfoSuanFa.editDetectedDepthMin.toDouble()|| InfoSuanFa.editDetectedDepthMax.toDouble()<1.000*Calucate_Width_Depth_B.m_mean_depth)
	{
		_callOneTimes->class_A_B.Result_B.Number_Depth_Result_NG = _callOneTimes->class_A_B.Result_B.Number_Depth_Result_NG + 1;
	}

	int all_number_depth_NG = _callOneTimes->class_A_B.Result_B.Number_Depth_Result_NG;
	QTableWidgetItem* widthItem4 = new QTableWidgetItem(QString::number(all_number_depth_NG));
	ui->tableAB_Show_Data->setItem(1, 4, widthItem4);
	if (1.000 * Calucate_Width_Depth_B.m_mean_width<InfoSuanFa.editDetectedWidthMin.toDouble()  || InfoSuanFa.editDetectedWidthMax.toDouble()<1.000 * Calucate_Width_Depth_B.m_mean_width )
	{
		_callOneTimes->class_A_B.Result_B.Number_Width_Result_NG = _callOneTimes->class_A_B.Result_B.Number_Width_Result_NG + 1;
	}
	int all_number_width_NG = _callOneTimes->class_A_B.Result_B.Number_Width_Result_NG;
	QTableWidgetItem* widthItem5 = new QTableWidgetItem(QString::number(all_number_width_NG));
	ui->tableAB_Show_Data->setItem(1, 5, widthItem5);

	double Measure_Depth_Result = _callOneTimes->class_A_B.Result_B.Measure_Depth_Result;
	QTableWidgetItem* widthItem6 = new QTableWidgetItem(QString::number(Measure_Depth_Result,'f', 2));
	ui->tableAB_Show_Data->setItem(1, 6, widthItem6);

	_callOneTimes->class_A_B.Result_B.Depth_Model = InfoSuanFa.editDetectedDepthMin + "-" + InfoSuanFa.editDetectedDepthMax;
	QString  Depth_Model = _callOneTimes->class_A_B.Result_B.Depth_Model;
	QTableWidgetItem* widthItem7 = new QTableWidgetItem(Depth_Model);
	ui->tableAB_Show_Data->setItem(1, 7, widthItem7);

	double Measure_Width_Result = _callOneTimes->class_A_B.Result_B.Measure_Width_Result;
	QTableWidgetItem* widthItem8 = new QTableWidgetItem(QString::number(Measure_Width_Result, 'f', 2));
	ui->tableAB_Show_Data->setItem(1, 8, widthItem8);

	_callOneTimes->class_A_B.Result_A.Width_Model = InfoSuanFa.editDetectedWidthMin + "-" + InfoSuanFa.editDetectedWidthMax;
	QString  Width_Model = _callOneTimes->class_A_B.Result_B.Width_Model;
	QTableWidgetItem* widthItem9 = new QTableWidgetItem(Width_Model);
	ui->tableAB_Show_Data->setItem(1, 9, widthItem9);

	//// 设置所有单元格内容居中
	for (int row = 0; row < ui->tableAB_Show_Data->rowCount(); ++row)
	{
		for (int col = 0; col < ui->tableAB_Show_Data->columnCount(); ++col)
		{
			QTableWidgetItem* item = ui->tableAB_Show_Data->item(row, col);
			if (item)
				item->setTextAlignment(Qt::AlignCenter);
		}
	}  

	auto ToPoint2Data = [](double v) -> QString {
		std::ostringstream o;
		o << std::fixed << std::setprecision(2) << v;
		return QString::fromStdString(o.str());
		};
	QString B_Max_Width = ToPoint2Data(Calucate_Width_Depth_B.m_max_width);
	QString B_Mean_Width = ToPoint2Data(Calucate_Width_Depth_B.m_mean_width);
	QString B_Min_Width = ToPoint2Data(Calucate_Width_Depth_B.m_min_width);
			 
	QString B_Max_Depth = ToPoint2Data(Calucate_Width_Depth_B.m_max_depth);
	QString B_Mean_Depth = ToPoint2Data(Calucate_Width_Depth_B.m_mean_depth);
	QString B_Min_Depth = ToPoint2Data(Calucate_Width_Depth_B.m_min_depth);

 
	QDateTime date = QDateTime::currentDateTime();
	QString strWorkName = date.toString("yyyy / MM / dd / hh:mm:ss");
	int index_B = all_number - 1;
	DataExcelUi->tableWidgetExcel_B->insertRow(index_B);
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 0, new QTableWidgetItem(QString::number(index_B)));//序号
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 1, new QTableWidgetItem(strWorkName));//日期
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 2, new QTableWidgetItem(B_Mean_Depth));//A面槽深均值
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 3, new QTableWidgetItem(B_Max_Depth));//A面槽深最大值
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 4, new QTableWidgetItem(B_Min_Depth));//A面槽深最小值
	if ( Calucate_Width_Depth_B.m_mean_depth<InfoSuanFa.editDetectedDepthMin.toDouble() ||  Calucate_Width_Depth_B.m_mean_depth > InfoSuanFa.editDetectedDepthMax.toDouble())
	{
		DataExcelUi->tableWidgetExcel_B->setItem(index_B, 5, new QTableWidgetItem(QStringLiteral("NG")));//A面槽深结果

	}
	else
	{
		DataExcelUi->tableWidgetExcel_B->setItem(index_B, 5, new QTableWidgetItem(QStringLiteral("OK")));//A面槽深结果
	}
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 6, new QTableWidgetItem(B_Mean_Width));//A面槽宽均值
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 7, new QTableWidgetItem(B_Max_Width));//A面槽宽最大值
	DataExcelUi->tableWidgetExcel_B->setItem(index_B, 8, new QTableWidgetItem(B_Min_Width));//A面槽宽最小值
	if (InfoSuanFa.editDetectedWidthMin.toDouble() >  Calucate_Width_Depth_B.m_mean_width ||  Calucate_Width_Depth_B.m_mean_width > InfoSuanFa.editDetectedWidthMax.toDouble())
	{
		DataExcelUi->tableWidgetExcel_B->setItem(index_B, 9, new QTableWidgetItem(QStringLiteral("NG")));//A面槽宽结果
	}
	else
	{
		DataExcelUi->tableWidgetExcel_B->setItem(index_B, 9, new QTableWidgetItem(QStringLiteral("OK")));//A面槽宽结果 
	}


	for (int row = 0; row < DataExcelUi->tableWidgetExcel_B->rowCount(); ++row)
	{
		for (int col = 0; col < DataExcelUi->tableWidgetExcel_B->columnCount(); ++col)
		{
			QTableWidgetItem* item = DataExcelUi->tableWidgetExcel_B->item(row, col);
			if (item)
				item->setTextAlignment(Qt::AlignCenter);
		}
	}
}

void PolePieceMeasureWin::Init_Clear_B()
{
	if (!_ThreePoint2RGBcloud_B.RGB_down_Cloud->empty())_ThreePoint2RGBcloud_B.RGB_down_Cloud->clear();
	if (!_ThreePoint2RGBcloud_B.RGB_left_Cloud->empty())_ThreePoint2RGBcloud_B.RGB_left_Cloud->clear();
	if (!_ThreePoint2RGBcloud_B.RGB_right_Cloud->empty())_ThreePoint2RGBcloud_B.RGB_right_Cloud->clear();
 	if (!Calucate_Width_Depth_B.m_width.empty())Calucate_Width_Depth_B.m_width.clear();
	if (!Calucate_Width_Depth_B.m_depth.empty())Calucate_Width_Depth_B.m_depth.clear();
	if (!ID_SELECT_B.empty())ID_SELECT_B.clear();
	if (!ID_NEW_B.empty())ID_NEW_B.clear();
	if (!_multifyRecognize_B.empty())_multifyRecognize_B.clear();
	if (!_separateCloud_B.empty())_separateCloud_B.clear();
	if (!separate_clouds_B.empty())separate_clouds_B.clear();
	if (!customPts_B.empty())customPts_B.clear();
	if (!customPts_bottom_B.empty())customPts_bottom_B.clear();
	if (!showResultPoint_B.empty())showResultPoint_B.clear();
  	ui->Single_B_PointShow->clearCustomPoints();
 	RightMostIndex_B = false;
	LeftMostIndex_B = false;
	//    /* 第一列：宽度 */
	ui->tb_Widget_B->setItem(0, 1, new QTableWidgetItem(""));
	ui->tb_Widget_B->setItem(1, 1, new QTableWidgetItem(""));
	ui->tb_Widget_B->setItem(2, 1, new QTableWidgetItem(""));
	ui->tb_Widget_B->setItem(0, 0, new QTableWidgetItem(QStringLiteral("平均宽度")));
	ui->tb_Widget_B->setItem(1, 0, new QTableWidgetItem(QStringLiteral("最大宽度")));
	ui->tb_Widget_B->setItem(2, 0, new QTableWidgetItem(QStringLiteral("最小宽度")));

	ui->tb_Widget_B->setItem(0, 2, new QTableWidgetItem(QStringLiteral("平均深度")));
	ui->tb_Widget_B->setItem(1, 2, new QTableWidgetItem(QStringLiteral("最大深度")));
	ui->tb_Widget_B->setItem(2, 2, new QTableWidgetItem(QStringLiteral("最小深度")));
	ui->tb_Widget_B->setItem(0, 3, new QTableWidgetItem(""));
	ui->tb_Widget_B->setItem(1, 3, new QTableWidgetItem(""));
	ui->tb_Widget_B->setItem(2, 3, new QTableWidgetItem(""));
	 
	ui->tableFrame_B->clearContents();
	ui->tableFrame_B->setRowCount(0);
}
void  PolePieceMeasureWin::onslotReceiveCloud_B()
{
	 Init_Clear_B();
	source_cloud_B.reset(new pcl::PointCloud<pcl::PointXYZ>);
	source_cloud_B->clear(); separate_clouds_B.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud = _callOneTimes_B->cloudsResultDeal;
	source_cloud_B = cloud;
	separate_clouds_B = separateCloud(source_cloud_B);
	//std::sort(cloud->points.begin(), cloud->points.end(),
	//    [](const pcl::PointXYZ& a, const pcl::PointXYZ& b)
	//    { return a.x < b.x; });
	//for (auto& p : *cloud)        // cloud 是 pcl::PointCloud<pcl::PointXYZ>::Ptr
	//    p.z *= 1.0f;           // 原地放大 1000 倍
	//source_cloud =  roiAcquire(cloud); // ROI设置
	//source_cloud->width= source_cloud->size();
	//source_cloud->height= 1;
	//source_cloud->is_dense  =true;
	roi_cloud_B = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	//  roiPercent = InfoSuanFa.editRoiPercent.toDouble();
	 // centerPercent = InfoSuanFa.editCentrePoint.toDouble();
	//  roi_cloud = roiAcquire(source_cloud, roiPercent, centerPercent);//划定一个ROI范围
	ui->tableFrame_B->clearContents();    // 清空单元格内容，表头不变
	ui->tableFrame_B->setRowCount(0);    // 设置为0行，彻底清空内容，表头依然会显示
	updateViewer(source_cloud_B);
	if (ui->checkBox_auto->isChecked())
	{
		on_btnTestShow_clicked();
	}
}
void  PolePieceMeasureWin::onReceiveStatus_B(QString content)
{
	RecordData((QStringLiteral("B:")+content),1);
}