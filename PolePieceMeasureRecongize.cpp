#include "PolePieceMeasureWin.h"
void PolePieceMeasureWin::on_btnLoadCloud_clicked()
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

	 Init_Clear();
	source_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	source_cloud->clear(); separate_clouds.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	ui->Single_A_PointShow->setPointCloud(cloud);

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
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("无法打开点云文件（路径需要全英文）"));
		return;
	}
	source_cloud = cloud;
	//std::sort(cloud->points.begin(), cloud->points.end(),
	//    [](const pcl::PointXYZ& a, const pcl::PointXYZ& b)
	//    { return a.x < b.x; });
	//for (auto& p : *cloud)        // cloud 是 pcl::PointCloud<pcl::PointXYZ>::Ptr
	//    p.z *= 1.0f;           // 原地放大 1.000 倍
	//source_cloud =  roiAcquire(cloud); // ROI设置
	//source_cloud->width= source_cloud->size();
	//source_cloud->height= 1;
	//source_cloud->is_dense  =true;
	roi_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	roiPercent = InfoSuanFa.editRoiPercent.toDouble();
	centerPercent = InfoSuanFa.editCentrePoint.toDouble();
	roi_cloud = roiAcquire(source_cloud, roiPercent, centerPercent);//划定一个ROI范围
	ui->tableFrame->clearContents();    // 清空单元格内容，表头不变
	ui->tableFrame->setRowCount(0);    // 设置为0行，彻底清空内容，表头依然会显示
	if (ui->checkbox_roi->isChecked())
	{
		ui->openGLWidget_Result->showPointCloud(roi_cloud);
		//updateViewer(roi_cloud); 
	}
	else
	{
		//updateViewer(source_cloud);
		ui->openGLWidget_Result->showPointCloud(source_cloud);
	}
	source_separate_clouds = separateCloud(source_cloud);

	if (ui->checkBox_auto->isChecked())
	{
		on_btnebug_clicked();
	}
 }
void  PolePieceMeasureWin::onslotReceiveCloud()
{
	/* Process.m_Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>(*_callOneTimes->merged_cloud));

   // Process.m_Cloud = _callOneTimes->merged_cloud;
    if (Process.m_Cloud == nullptr || Process.m_Cloud->empty())
    {
        updateStates(QStringLiteral("点云为空，无法处理"));
        updateRecord2(QStringLiteral("点云为空，无法处理"), 1);
        return;
    }*/
	//pointsClar(); 
	Init_Clear();
	source_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	source_cloud->clear(); separate_clouds.clear();
	 source_separate_clouds.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud = _callOneTimes->cloudsResultDeal;
	source_cloud = cloud;
	separate_clouds = separateCloud(source_cloud);
	source_separate_clouds = separateCloud(source_cloud);
	 
	roi_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	 
	ui->tableFrame->clearContents();    // 清空单元格内容，表头不变
	ui->tableFrame->setRowCount(0);    // 设置为0行，彻底清空内容，表头依然会显示
	updateViewer(source_cloud);
	if (ui->checkBox_auto->isChecked())
	{
		on_btnebug_clicked();
	}
}

void PolePieceMeasureWin::on_btnebug_clicked()//debug
{
	/* ui->openGLWidget_Result->setCameraUpVector(0, 1, 0);;
	return;*/
	if (source_cloud->points.empty())
	{
		QMessageBox::warning(this, QStringLiteral("Warning"), QStringLiteral("请加载点云A"));
		return;
	}
	iniSettingParam();
	roiPercent = InfoSuanFa.editRoiPercent.toDouble();
	centerPercent = InfoSuanFa.editCentrePoint.toDouble();
	for (int i = 0; i < 10; i++)
	{ 
		roi_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
 		roi_cloud = roiAcquire(source_cloud, roiPercent, centerPercent);//划定一个ROI范围
		bool result = StartRecongize(roi_cloud);//改成void类型
		 
		if (RightMostIndex)
		{
			centerPercent = centerPercent + 5;
 		}
		
		if (LeftMostIndex)
		{
			centerPercent = centerPercent - 5;
		}
		if(!RightMostIndex && !LeftMostIndex)
		{

			break;
		}
	}
	if (Calucate_Width_Depth.m_width.size()<80)
	{
		ui->Result_A->setText("NG");
		ui->Result_A->setStyleSheet("color: rgb(255,0,0);");          // 红色 
		return;
	}
	Result_A_Show();
	new_A_updateTable(); tableAB_Show();
	ShowThreeVectorPoint(ThreePointToRGBcloud(_multifyRecognize));
	Show_CurrentAB_Table_Show();
	//exportToCSV();
	//on_btnTestShow_clicked();
  }
pcl::PointCloud<pcl::PointXYZ>::Ptr PolePieceMeasureWin::roiAcquire(pcl::PointCloud<pcl::PointXYZ>::Ptr  sourcecloud, double roiPercent, double centerPercent)
{
	// 相对宽度百分比 roiPercent
	//roiPercent = InfoSuanFa.editRoiPercent.toDouble(); //ui->editROI->text().toDouble();
	roiPercent = std::max(1.0, std::min(roiPercent, 100.0)); // 限制1~100

	// 中心位置百分比 centerPercent
   // centerPercent = InfoSuanFa.editCentrePoint.toDouble(); // ui->editROI_2->text().toDouble();
	centerPercent = std::max(0.0, std::min(centerPercent, 100.0)); // 限制0~100
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_clouds(new pcl::PointCloud<pcl::PointXYZ>);

	if (!sourcecloud->empty())
	{
		double xMin = std::numeric_limits<double>::max();
		double xMax = -std::numeric_limits<double>::max();
		for (const auto& pt : sourcecloud->points) {
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
 		//过滤符合条件的点
		for (const auto& pt : sourcecloud->points) {
			if (pt.x >= roiStart && pt.x <= roiEnd) {
				source_clouds->points.push_back(pt);
			}
		} 
	}
	return source_clouds;
}

std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> PolePieceMeasureWin::NineJie(pcl::PointCloud<pcl::PointXYZ>::Ptr  roiCloud)
{
	//点云长度
	size_t point_length = roiCloud->size();
	size_t point_3_length = 0;
	//判断point_length是否为3的倍数
	point_3_length = point_length % 3;
	if (point_3_length!=0)
	{
		point_length = point_length - point_3_length;
	}
	else
	{
		point_length = point_length;
	}
	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> sanJie_Cloud; //sanJie_Cloud.resize(3);
	// 假设 roiCloud 已经填充好;roiCloud->points[i].x 单调递增。
	std::sort(roiCloud->points.begin(),
		roiCloud->points.end(),
		[](const pcl::PointXYZ& a, const pcl::PointXYZ& b)
		{
			return a.x < b.x;   // 按 x 从小到大
		});
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_first(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_second(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_third(new pcl::PointCloud<pcl::PointXYZ>);
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr leng_fourth(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_five(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_six(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_seven(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_eight(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_nine(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_first1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_second1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_third1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_fourth1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_five1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_six1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_seven1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_eight1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr leng_nine1(new pcl::PointCloud<pcl::PointXYZ>);*/
	leng_first->resize(point_length / 3);
	leng_first->width = point_length / 3;
	leng_first->height =1;
	leng_first->is_dense =true;
	
	leng_second->resize(point_length / 3);
	leng_second->width = point_length / 3;
	leng_second->height =1;
	leng_second->is_dense =true;
	
	leng_third->resize(point_3_length+point_length / 3 );
	leng_third->width = point_length / 3 +point_3_length;
	leng_third->height =1;
	leng_third->is_dense =true;
	
	//leng_fourth->resize(point_length / 18);
	//leng_fourth->width = point_length / 18;
	//leng_fourth->height =1;
	//leng_fourth->is_dense =true;
	//
	//leng_five->resize(point_length / 18);
	//leng_five->width = point_length / 18;
	//leng_five->height =1;
	//leng_five->is_dense =true;
	//
	//leng_six->resize(point_length / 18 );
	//leng_six->width = point_length / 18;// +point_3_length;
	//leng_six->height =1;
	//leng_six->is_dense =true;
	//
	//leng_seven->resize(point_length / 18);
	//leng_seven->width = point_length / 18;
	//leng_seven->height =1;
	//leng_seven->is_dense =true;
	//
	//leng_first1->resize(point_length / 18);
	//leng_first1->width = point_length / 18;
	//leng_first1->height =1;
	//leng_first1->is_dense =true;
	//
	//leng_second1->resize(point_length / 18);
	//leng_second1->width = point_length / 18;
	//leng_second1->height =1;
	//leng_second1->is_dense =true;
	//
	//leng_third1->resize(point_length / 18 );
	//leng_third1->width = point_length / 18;// +point_3_length;
	//leng_third1->height =1;
	//leng_third1->is_dense =true;
	//
	//leng_fourth1->resize(point_length / 18);
	//leng_fourth1->width = point_length / 18;
	//leng_fourth1->height =1;
	//leng_fourth1->is_dense =true;
	//
	//leng_five1->resize(point_length / 18);
	//leng_five1->width = point_length / 18;
	//leng_five1->height =1;
	//leng_five1->is_dense =true;
	//
	//leng_six1->resize(point_length / 18 );
	//leng_six1->width = point_length / 18;// +point_3_length;
	//leng_six1->height =1;
	//leng_six1->is_dense =true;
	//
	//leng_seven1->resize(point_length / 18);
	//leng_seven1->width = point_length / 18;
	//leng_seven1->height =1;
	//leng_seven1->is_dense =true;
	//
	//leng_eight1->resize(point_length / 18);
	//leng_eight1->width = point_length / 18;
	//leng_eight1->height =1;
	//leng_eight1->is_dense =true;
	//
	//leng_nine1->resize(point_length / 18 );
	//leng_nine1->width = point_length / 18;// +point_3_length;
	//leng_nine1->height =1;
	//leng_nine1->is_dense =true;

	for (size_t i = 0; i < point_length/3 ;++i)
	{
		leng_first->points[i] = roiCloud->points[i];
	}
	for (size_t i = point_length / 3; i < 2*point_length/3 ;++i)
	{
		leng_second->points[i- point_length / 3] = roiCloud->points[i];
	}

	for (size_t i =2 * point_length / 3; i <point_length+ point_3_length;++i)
	//for (size_t k =2 * point_length / 18; k < point_length+ point_3_length;++k)
	{
		leng_third->points[i - 2 * point_length / 3] = roiCloud->points[i];
	}
	
	//for (size_t i = 3 * point_length / 18; i < 4*point_length/18 ;++i)
	//{
	//	leng_first->points[i- 3 * point_length / 18] = roiCloud->points[i];
	//}
	//for (size_t i = 4 * point_length / 18; i < 5*point_length/18 ;++i)
	//{
	//	leng_second->points[i- 4*point_length / 18] = roiCloud->points[i];
	//}

	//for (size_t i =5 * point_length / 18; i < 6*point_length/18;++i)
	////for (size_t k =2 * point_length / 18; k < point_length+ point_3_length;++k)
	//{
	//	leng_third->points[i - 5 * point_length / 18] = roiCloud->points[i];
	//}
	//for (size_t i = 6 * point_length / 18; i < 7*point_length/18 ;++i)
	//{
	//	leng_first->points[i- 6 * point_length / 18] = roiCloud->points[i];
	//}
	//for (size_t i = 7 * point_length / 18; i < 8*point_length/18 ;++i)
	//{
	//	leng_second->points[i- 7*point_length / 18] = roiCloud->points[i];
	//}

	//for (size_t i =8 * point_length / 18; i < 9*point_length/18+ point_3_length;++i)
	////for (size_t k =2 * point_length / 18; k < point_length+ point_3_length;++k)
	//{
	//	leng_third->points[i - 8 * point_length / 18] = roiCloud->points[i];
	//}

	//for (size_t i = 9 * point_length / 18; i < 10*point_length/18 ;++i)
	//{
	//	leng_first->points[i- 9 * point_length / 18] = roiCloud->points[i];
	//}
	//for (size_t i = 10*point_length / 18; i < 11*point_length/18 ;++i)
	//{
	//	leng_second->points[i- 10*point_length / 18] = roiCloud->points[i];
	//}

	//for (size_t i =11 * point_length / 18; i <12* point_length/18;++i)
	////for (size_t k =2 * point_length / 18; k < point_length+ point_3_length;++k)
	//{
	//	leng_third->points[i - 11 * point_length / 18] = roiCloud->points[i];
	//}
	//
	//for (size_t i = 12 * point_length / 18; i < 13*point_length/18 ;++i)
	//{
	//	leng_first->points[i- 12 * point_length / 18] = roiCloud->points[i];
	//}
	//for (size_t i = 13 * point_length / 18; i < 14*point_length/18 ;++i)
	//{
	//	leng_second->points[i- 13*point_length / 18] = roiCloud->points[i];
	//}

	//for (size_t i =14 * point_length / 18; i < 15*point_length/18;++i)
	////for (size_t k =2 * point_length / 18; k < point_length+ point_3_length;++k)
	//{
	//	leng_third->points[i - 14 * point_length / 18] = roiCloud->points[i];
	//}
	//for (size_t i = 15 * point_length / 18; i < 16*point_length/18 ;++i)
	//{
	//	leng_first->points[i- 15 * point_length / 18] = roiCloud->points[i];
	//}
	//for (size_t i = 16 * point_length / 18; i < 17*point_length/18 ;++i)
	//{
	//	leng_second->points[i- 16*point_length / 18] = roiCloud->points[i];
	//}

	//for (size_t i =17 * point_length / 18; i < 18*point_length/18+ point_3_length;++i)
	////for (size_t k =2 * point_length / 18; k < point_length+ point_3_length;++k)
	//{
	//	leng_third->points[i - 17 * point_length / 18] = roiCloud->points[i];
	//}

	sanJie_Cloud.push_back(leng_first);
	sanJie_Cloud.push_back(leng_second);
	sanJie_Cloud.push_back(leng_third);
	
	/*sanJie_Cloud.push_back(leng_fourth);
	sanJie_Cloud.push_back(leng_five);
	sanJie_Cloud.push_back(leng_six);
	
	sanJie_Cloud.push_back(leng_seven);
	sanJie_Cloud.push_back(leng_eight);
	sanJie_Cloud.push_back(leng_nine);
	
	sanJie_Cloud.push_back(leng_first1);
	sanJie_Cloud.push_back(leng_second1);
	sanJie_Cloud.push_back(leng_third1);
	
	sanJie_Cloud.push_back(leng_fourth1);
	sanJie_Cloud.push_back(leng_five1);
	sanJie_Cloud.push_back(leng_six1);
	
	sanJie_Cloud.push_back(leng_seven1);
	sanJie_Cloud.push_back(leng_eight1);
	sanJie_Cloud.push_back(leng_nine1);*/
	return sanJie_Cloud;
}
//PolePieceMeasureWin::ThreePoint PolePieceMeasureWin::YIJie(pcl::PointCloud<pcl::PointXYZ>::Ptr  yijieCloud, int pinghuaxishu)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pinghua_cloud = slidingAverageZ(yijieCloud, pinghuaxishu);
//	pinghua_cloud->width = source_cloud->size();
//	pinghua_cloud->height = 1;
//	pinghua_cloud->is_dense = true;
//	nihe_cloud = nihe_Roate(pinghua_cloud);
//	//思路：判断凹槽最低点，然后找到凹槽的肩点；
//	PolePieceMeasureWin::ThreePoint single_threePoint;
//
//	double min_z = std::numeric_limits<double>::max();
//	int min_index = -1;
//	int down_idx = -1;
//
//	// 遍历所有点
//	for (size_t i = 0; i < nihe_cloud->size(); ++i) {
//		if (nihe_cloud->points[i].z < min_z) {
//			min_z = nihe_cloud->points[i].z;
//			min_index = i;
//		}
//	}
//	down_idx = min_index;
//	pcl::PointXYZ down_point;
//	//if ((yijieCloud->points[0].z - yijieCloud->points[down_idx].z) / (yijieCloud->points[0].x - yijieCloud->points[down_idx].x)>-0.005)//-0.084
//	bool ok = false;
//	float didianThreshold = InfoSuanFa.editGaoDuChaThreshold.toFloat(); //ui->editbottom->text().toFloat(&ok);
//	if ((nihe_cloud->points[0].z - nihe_cloud->points[down_idx].z) < didianThreshold)//-0.008
//	{
//		down_point = yijieCloud->points[down_idx];
//		down_point.z = 0;
//	}
//	else
//	{
//		down_point = yijieCloud->points[down_idx];
//		sandian_suoyin.down_index.push_back(down_idx);
//		single_threePoint.downShoulder = down_point;//*************低点***************** 
//	}
//
//	//以最低点位置的索引，向两边扩展，向左找到第一次出现 斜率>=0
//	std::vector<double> left_xielv;
//	size_t left_idx = 0;                 // 默认值 
//	for (size_t i = down_idx; i > 1; --i)
//	{
//		double left_kerial = (nihe_cloud->points[i].z - nihe_cloud->points[i - 1].z) / (nihe_cloud->points[i].x - nihe_cloud->points[i - 1].x);
//		if (left_kerial > 0)
//		{
//			left_idx = i;
//			break;
//		}
//		/*else if(i==0)
//		{
//			left_idx = 0;
//			break;
//
//		}*/
//	}
//	if (down_point.z != 0)
//	{
//		sandian_suoyin.left_index.push_back(left_idx);
//		pcl::PointXYZ left_point = yijieCloud->points[left_idx];//***********左点*******************
//		single_threePoint.leftShoulder = left_point;
//	}
//
//	std::vector<double> right_xielv;
//	size_t right_idx = nihe_cloud->size() - 1;                 // 默认值 
//	for (size_t i = down_idx; i < nihe_cloud->size() - 1; ++i)
//	{
//		double right_kerial = (nihe_cloud->points[i].z - nihe_cloud->points[i + 1].z) / (nihe_cloud->points[i].x - nihe_cloud->points[i + 1].x);
//		if (right_kerial <= 0)
//		{
//			right_idx = i;
//			break;
//		}
//		/*else if(i== nihe_cloud->size() - 1)
//		{
//			right_idx = nihe_cloud->size()-1;
//			break;
//		}*/
//	}
//	if (down_point.z != 0)
//	{
//		sandian_suoyin.right_index.push_back(right_idx);
//		pcl::PointXYZ right_point = yijieCloud->points[right_idx];//***********右点*******************
//		single_threePoint.rightShoulder = right_point;
//	}
//	return single_threePoint;
//}
//
//PolePieceMeasureWin::ThreePoint PolePieceMeasureWin::YIJie(pcl::PointCloud<pcl::PointXYZ>::Ptr  yijieCloud, int pinghuaxishu)
//{
//	pcl::PointCloud<pcl::PointXYZ>::Ptr pinghua_cloud = slidingAverageZ(yijieCloud, pinghuaxishu);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr nihe_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	pinghua_cloud->width = yijieCloud->size();
//	pinghua_cloud->height = 1;
//	pinghua_cloud->is_dense = true;
// 	nihe_cloud = nihe_Roate(pinghua_cloud);
//	//思路：判断凹槽最低点，然后找到凹槽的肩点；
//	PolePieceMeasureWin::ThreePoint single_threePoint; 
//	double min_z = std::numeric_limits<double>::max();
//	int min_index = -1;
//	int down_idx = -1;
//
//
//	//左肩点和右肩点的索引与底点的差值
//	int left_2_bottom = -1;
//	int right_2_bottom = -1;
//	// 遍历所有点
//	for (size_t i = 0; i < nihe_cloud->size(); ++i) {
//		if (nihe_cloud->points[i].z < min_z) {
//			min_z = nihe_cloud->points[i].z;
//			min_index = i;
//		}
//	}
//	down_idx = min_index;
//	pcl::PointXYZ down_point;
//	//if ((yijieCloud->points[0].z - yijieCloud->points[down_idx].z) / (yijieCloud->points[0].x - yijieCloud->points[down_idx].x)>-0.005)//-0.084
//	bool ok = false;
//	float didianThreshold = InfoSuanFa.editGaoDuChaThreshold.toFloat(); //ui->editbottom->text().toFloat(&ok);
//	if ((nihe_cloud->points[0].z - nihe_cloud->points[down_idx].z) < didianThreshold)//-0.008
//	{ 
//		down_point = yijieCloud->points[down_idx];
//		down_point.z = 0;
//	}
//	else
//	{
//		if (down_idx<2)
//		{
//			down_idx = 2;
//		}
//		int max_index = -1;
//		if (down_idx>178)
//		{
//			max_index = 179;
// 		}
//		else
//		{
//			max_index = down_idx + 2;
//		}
//		//加一个二次定位的功能，那就是比较一下前面和后面的点的Z值，找Z值的最小值索引
//		std::vector<double> min_Z;
//		for (int i = down_idx - 2; i < max_index; i++)
//		{
//			min_Z.push_back(yijieCloud->points[i].z);
//		}
//		double z_min = std::numeric_limits<double>::max();
//		int    i_min = -1;                 // 记录最小值对应的下标
//
//		for (int i = down_idx - 2; i < max_index; ++i)
//		{
//			double z = yijieCloud->points[i].z;
//			if (z < z_min)                 // 遇到更小的就更新
//			{
//				z_min = z;
//				down_idx = i;
//			}
//		}
// 		down_point = yijieCloud->points[down_idx];
//		sandian_suoyin.down_index.push_back(down_idx);
//		single_threePoint.downShoulder = down_point;//*************低点***************** 
//	}
//
//	//以最低点位置的索引，向两边扩展，向左找到第一次出现 斜率>=0
//	//修改算法：将从最低点为中心，向两边扩展20个点
//	std::vector<double> left_xielv;
//	int left_idx = 0;                 // 默认值 
//	int bianlijiandianshu = ParamUi->editSlope->text().toInt();
//
//	if (down_idx < bianlijiandianshu)
//	{ 
// 		for (int i = down_idx; i > 1; --i)
//		{
//
//			double left_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i - 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i - 1].x);
//			left_xielv.push_back(left_kerial);
//			/*double leftthreshould = ParamUi->editRadius->text().toDouble();
//		if (left_kerial > leftthreshould)
//		{
//			left_idx = i;
//			break;
//		}*/
//			/*double left_kerial = (nihe_cloud->points[i].z - nihe_cloud->points[i-1].z) / (nihe_cloud->points[i].x - nihe_cloud->points[i-1].x);
//		double leftthreshould = ParamUi->editRadius->text().toDouble();
//		if (left_kerial>leftthreshould)
//		{
//			left_idx = i;
//			break;
//		}*/
//			/*else if(i==0)
//		{
//			left_idx = 0;
//			break;
//
//		}*/
//		}
//     }
//	else
//	{
//		for (int i = down_idx; i > down_idx- bianlijiandianshu; --i)
//		{
//			double left_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i - 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i - 1].x);
//			left_xielv.push_back(left_kerial);
//			/*double left_kerial = (nihe_cloud->points[i].z - nihe_cloud->points[i - 1].z) / (nihe_cloud->points[i].x - nihe_cloud->points[i - 1].x);
//			double leftthreshould = ParamUi->editRadius->text().toDouble();
//			if (left_kerial > leftthreshould)
//			{
//				left_idx = i;
//				break;
//			}*/
//			/*double left_kerial = (nihe_cloud->points[i].z - nihe_cloud->points[i-1].z) / (nihe_cloud->points[i].x - nihe_cloud->points[i-1].x);
//			double leftthreshould = ParamUi->editRadius->text().toDouble();
//			if (left_kerial>leftthreshould)
//			{
//				left_idx = i;
//				break;
//			}*/
//			/*else if(i==0)
//			{
//				left_idx = 0;
//				break;
//
//			}*/
//		}
//	}
//	if (down_point.z != 0)
//	{
//		//把数组里面的最小值找出来
//	// 1. 找最小值迭代器
//		auto it = std::min_element(left_xielv.begin(), left_xielv.end());
//
//		// 2. 最小值
//		double min_val = *it;
//
//		// 3. 对应的序列号（下标）
//		left_idx = down_idx - std::distance(left_xielv.begin(), it);
//		left_2_bottom = down_idx-left_idx; // 左肩到底的索引差值
//		/*if (left_2_bottom<9)
//		{
//			left_idx= left_idx-5;
//		}*/
//		//if (left_2_bottom <10)
//		if (left_idx <3)
//		{
//			LeftMostIndex = true;
//			//left_idx = 0;
//		}
//		sandian_suoyin.left_index.push_back(left_idx);
//
//		pcl::PointXYZ left_point = yijieCloud->points[left_idx];//***********左点*******************
//		single_threePoint.leftShoulder = left_point;
//	}
//	
//	//if (down_point.z != 0)
//	//{
//	//	sandian_suoyin.left_index.push_back(left_idx); 
//	//	pcl::PointXYZ left_point = yijieCloud->points[left_idx];//***********左点*******************
//	//	single_threePoint.leftShoulder = left_point;
//	//}	
//	
//	std::vector<double> right_xielv;
//	int right_idx = nihe_cloud->size()-1;                 // 默认值 
//	if (down_idx+ bianlijiandianshu < nihe_cloud->size() - 1)
//	{
//		for (int i = down_idx; i < down_idx+ bianlijiandianshu; ++i)
//		{
//			double right_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i + 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i + 1].x);
//			right_xielv.push_back(right_kerial);
//			/*double rightthreshould = ParamUi->editTorlance->text().toDouble();
//
//			if (right_kerial <= rightthreshould)
//			{
//				right_idx = i;
//				break;
//			}*/
//			/*else if(i== nihe_cloud->size() - 1)
//			{
//				right_idx = nihe_cloud->size()-1;
//				break;
//			}*/
//		}
//	}
//	else
//	{
//		for (int i = down_idx; i < nihe_cloud->size() - 1; ++i)
//		{
//			double right_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i + 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i + 1].x);
//			right_xielv.push_back(right_kerial);
//			/*double rightthreshould = ParamUi->editTorlance->text().toDouble();
//
//			if (right_kerial <= rightthreshould)
//			{
//				right_idx = i;
//				break;
//			}*/
//			/*else if(i== nihe_cloud->size() - 1)
//			{
//				right_idx = nihe_cloud->size()-1;
//				break;
//			}*/
//		}
//	}
//	//for (int i = down_idx; i< nihe_cloud->size()-1; ++i)
//	//{
//	//	double right_kerial = (nihe_cloud->points[i].z - nihe_cloud->points[i+1].z) / (nihe_cloud->points[i].x - nihe_cloud->points[i+1].x);
//	//	double rightthreshould = ParamUi->editTorlance->text().toDouble();
//
//	//	if (right_kerial<= rightthreshould)
//	//	{
//	//		right_idx = i;
//	//		break;
//	//	}
//	//	/*else if(i== nihe_cloud->size() - 1)
//	//	{
//	//		right_idx = nihe_cloud->size()-1;
//	//		break;
//	//	}*/
//	//}
//	if (down_point.z != 0)
//	{
//		auto it_r = std::max_element(right_xielv.begin(), right_xielv.end());
//
//		double max_val = *it_r;                       // 最大值
//		right_idx = down_idx + std::distance(right_xielv.begin(), it_r); // 对应下标
//		//增加一个二次定位，利用夹角进行判断
//		right_2_bottom = right_idx - down_idx;
//		/*if (right_2_bottom<8)
//		{
//			right_idx = right_idx + 6;
//		}*/
//		if (right_idx > nihe_cloud->size() - 4)
//		//if (right_idx==nihe_cloud->size()-1)
//		{
//			RightMostIndex = true;
//			//right_idx = nihe_cloud->size() - 1;
//		}
//		sandian_suoyin.right_index.push_back(right_idx);
//		pcl::PointXYZ right_point = yijieCloud->points[right_idx];//***********右点*******************
//		single_threePoint.rightShoulder = right_point;
//	}
//	//if (down_point.z != 0)
//	//{
//	//	sandian_suoyin.right_index.push_back(right_idx); 
//	//	pcl::PointXYZ right_point = yijieCloud->points[right_idx];//***********右点*******************
//	//	single_threePoint.rightShoulder = right_point;
//	//}  
//	return single_threePoint;
//}
PolePieceMeasureWin::ThreePoint PolePieceMeasureWin::YIJie(pcl::PointCloud<pcl::PointXYZ>::Ptr  yijieCloud, int pinghuaxishu)//单独处理三节分成一节的情况
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pinghua_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
	pinghua_cloud->width = yijieCloud->size();
	pinghua_cloud->height = 1;
	pinghua_cloud->is_dense = true;
	 pinghua_cloud = slidingAverageZ(yijieCloud, pinghuaxishu);
	pcl::PointCloud<pcl::PointXYZ>::Ptr nihe_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	nihe_cloud = nihe_Roate(pinghua_cloud);
	//思路：判断凹槽最低点，然后找到凹槽的肩点；
	PolePieceMeasureWin::ThreePoint single_threePoint;
	double min_z = std::numeric_limits<double>::max();
	int min_index = -1;
	int down_idx = -1;


	//左肩点和右肩点的索引与底点的差值
	int left_2_bottom = -1;
	int right_2_bottom = -1;
	// 遍历所有点
	for (size_t i = 0; i < nihe_cloud->size(); ++i) {
		if (nihe_cloud->points[i].z < min_z) {
			min_z = nihe_cloud->points[i].z;
			min_index = i;
		}
	}
	down_idx = min_index;
	pcl::PointXYZ down_point;
	//if ((yijieCloud->points[0].z - yijieCloud->points[down_idx].z) / (yijieCloud->points[0].x - yijieCloud->points[down_idx].x)>-0.005)//-0.084
	bool ok = false;
	float didianThreshold = InfoSuanFa.editGaoDuChaThreshold.toFloat(); //ui->editbottom->text().toFloat(&ok);
	if ((nihe_cloud->points[0].z - nihe_cloud->points[down_idx].z) < didianThreshold)//-0.008
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
		if (down_idx > nihe_cloud->points.size() - 2)
		//if (down_idx > 178)
		{
			max_index = nihe_cloud->points.size() - 1;
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
		sandian_suoyin.down_index.push_back(down_idx);
		single_threePoint.downShoulder = down_point;//*************低点***************** 

	}
	std::vector<double> left_xielv;
	int left_idx = 0;                 // 默认值 
	int bianlijiandianshu = ParamUi->editSlope->text().toInt();

	if (down_idx < bianlijiandianshu)
	{
		for (int i = down_idx; i > 1; --i)
		{

			double left_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i - 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i - 1].x);
			left_xielv.push_back(left_kerial);
		}
	}
	else
	{
		for (int i = down_idx; i > down_idx - bianlijiandianshu; --i)
		{
			double left_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i - 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i - 1].x);
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

		//if (left_idx < 10)
		if (left_idx < 4)
		{
			LeftMostIndex = true;
			//left_idx = 0;
		}
		sandian_suoyin.left_index.push_back(left_idx);
		pcl::PointXYZ left_point = yijieCloud->points[left_idx];//***********左点*******************
		single_threePoint.leftShoulder = left_point;
	}
	std::vector<double> right_xielv;
	int right_idx = nihe_cloud->size() - 1;                 // 默认值 
	if (down_idx + bianlijiandianshu < nihe_cloud->size() - 1)
	{
		for (int i = down_idx; i < down_idx + bianlijiandianshu; ++i)
		{
			double right_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i + 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i + 1].x);
			right_xielv.push_back(right_kerial);
		}
	}
	else
	{
		for (int i = down_idx; i < nihe_cloud->size() - 1; ++i)
		{
			double right_kerial = (nihe_cloud->points[down_idx].z - nihe_cloud->points[i + 1].z) / (nihe_cloud->points[down_idx].x - nihe_cloud->points[i + 1].x);
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
 		if (right_idx > nihe_cloud->size() - 4)
			//if (right_idx==nihe_cloud->size()-1)
		{
			RightMostIndex = true;
			//right_idx = nihe_cloud->size() - 1;
		}
		sandian_suoyin.right_index.push_back(right_idx);
		pcl::PointXYZ right_point = yijieCloud->points[right_idx];//***********右点*******************
		single_threePoint.rightShoulder = right_point;
	}
	return single_threePoint;

}

PolePieceMeasureWin::ThreePoint PolePieceMeasureWin::singleRecongize(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  threeJieCloud, int pinghuaxishu)
{
	PolePieceMeasureWin::ThreePoint vectorThreePoints; 
	// 左肩点和右肩点的距离
	auto pointDistance1 = [](const pcl::PointXYZ& p1,
		const pcl::PointXYZ& p2) -> float
		{
			return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) +
				(p1.y - p2.y) * (p1.y - p2.y) +
				(p1.z - p2.z) * (p1.z - p2.z));
		};
	//点p到a,b连线的距离
	auto pointToLineDistance3D1 =
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
		PolePieceMeasureWin::ThreePoint single_threePoint;
		single_threePoint = YIJie(threeJieCloud[i], pinghuaxishu);
		if (single_threePoint.downShoulder.z != 0)
		{
			vectorThreePoints = single_threePoint;
			//屏蔽掉  后面会算一遍   粗定位先不算了
			/*double width = pointDistance1(single_threePoint.leftShoulder, single_threePoint.rightShoulder);
			double depth = pointToLineDistance3D1(single_threePoint.leftShoulder, single_threePoint.rightShoulder, single_threePoint.downShoulder);
			width=1000* width+InfoSuanFa.edit_Width_A_Coefficient.toFloat();
			depth =1000* depth+ InfoSuanFa.edit_Depth_A_Coefficient.toFloat();
 			Calucate_Width_Depth.m_width.push_back(width);
			Calucate_Width_Depth.m_depth.push_back(depth);*/
			_callOneTimes->success = true;
			break; //找到了就跳出循环
		}
		else
		{
			 
			_callOneTimes->success = false;
		}
		//PolePieceMeasureWin::ThreePoint single_threePoint;
		//single_threePoint = YIJie(threeJieCloud[i], pinghuaxishu);
		////if (i == threeJieCloud.size() - 1 && single_threePoint.downShoulder.z == 0)
		////{
		////	Calucate_Width_Depth.m_width.clear();
		////	Calucate_Width_Depth.m_depth.clear();
		////	centerPercent = centerPercent - 0.2;
		////	ParamUi->editROI_2->setText(QString::number(centerPercent));
		////	roi_cloud = roiAcquire(source_cloud, roiPercent, centerPercent);
		////	_callOneTimes->success = false;
		////	break;
		////	//return;
		////}
		//if (single_threePoint.downShoulder.z != 0)
		//{
		//	showResultPoint.push_back(threeJieCloud[i]);

		//	vectorThreePoints = single_threePoint;
		//	//屏蔽掉  后面会算一遍   粗定位先不算了
 	//		double width = pointDistance(single_threePoint.leftShoulder, single_threePoint.rightShoulder);
		//	double depth = pointToLineDistance3D(single_threePoint.leftShoulder, single_threePoint.rightShoulder, single_threePoint.downShoulder);

		//	Calucate_Width_Depth.m_width.push_back(width);
		//	Calucate_Width_Depth.m_depth.push_back(depth);
		//	_callOneTimes->success = true; 
		//	break;
		//}else
		//	_callOneTimes->success = false; 
	}
	return vectorThreePoints;

}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PolePieceMeasureWin::separateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sourcedCloud)
{
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> input_vector;
	if (!sourcedCloud->empty())
	{
		if (!sourcedCloud || sourcedCloud->empty())
		{
			//return;51   16
		}
		std::map<float, pcl::PointCloud<pcl::PointXYZ>::Ptr> separateClouds;

		for (const auto& pt : sourcedCloud->points)
		{
			if (pt.z < -10)
			{
				continue;
			}
			if (separateClouds.find(pt.y) == separateClouds.end())//当前 Y 值第一次出现 → 新建一个空点云。
			{
				separateClouds[pt.y] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
			}
			separateClouds[pt.y]->points.push_back(pt);//把该点放进对应 Y 值的那一层点云。 
		}

		//遍历map，设置点云属性，直接把智能指针放入vector
		for (auto& ptr : separateClouds)//按升序遍历 map 里的每一项；kv.first 是 Y 值，kv.second 是点云指针。
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr& pr = ptr.second;//起个别名，少打字。
			pr->width = static_cast<uint32_t>(pr->points.size());//把宽度设成点的数量。
			pr->height = 1;//高度为 1 → 明确这是“无组织”点云（简单列表）。
			pr->is_dense = true;
			input_vector.push_back(pr);
		}


	}
	return input_vector;

}
//多帧识别
 bool PolePieceMeasureWin::multifyRecognize(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  multifyCloud, int pinghuaxishu)
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
	std::vector<PolePieceMeasureWin::ThreePoint> vectorThreePoints;
	vectorThreePoints.clear();
	for (int i = 0; i < sizenum; ++i)
	{
		//nihe_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		//roi_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		//roi_cloud = roiAcquire(multifyCloud[i]);//划定一个ROI范围
		/*pcl::PointCloud<pcl::PointXYZ>::Ptr pinghua_cloud =slidingAverageZ(roi_cloud, pinghuaxishu);
		pinghua_cloud->width = source_cloud->size();
		pinghua_cloud->height = 1;
		pinghua_cloud->is_dense = true;
		nihe_cloud = nihe_Roate(pinghua_cloud);*/
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> nine_jie;
		nine_jie.clear();
		nine_jie = NineJie(multifyCloud[i]);//一帧分3块
		 
		 //PolePieceMeasureWin::ThreePoint single_threePoint = singleRecongize(nine_jie, pinghuaxishu);//单帧识别
		 // 原错误代码：
// PolePieceMeasureWin::ThreePoint single_threePoint = singleRecongize(multifyCloud[i], pinghuaxishu);//单帧识别

// 修正方式：将 multifyCloud[i] 包装为 vector 传递
		 PolePieceMeasureWin::ThreePoint single_threePoint = singleRecongize(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>{multifyCloud[i]}, pinghuaxishu);//单帧识别
		 if (_callOneTimes->success) 
		 { 
			 //if (i == 118)//D:\project\PolePieceMeasure\PolePieceMeasureWin\x64\Debug\ply\HG260114163130\pointCloud
			// {
			//	 int a = 2;
			// }
			 ID_SELECT.push_back(i); 
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
			  
			//int left_x_index = single_threePoint.leftShoulder.x/0.003;
			 double editCloudShowDis = 0.001*InfoSuanFa.editCloudShowDis.toFloat();
			int left_x_index = single_threePoint.leftShoulder.x/ editCloudShowDis;
			//int right_x_index= single_threePoint.rightShoulder.x/0.003;
			int right_x_index= single_threePoint.rightShoulder.x/ editCloudShowDis;
			int down_x_index= single_threePoint.downShoulder.x/ editCloudShowDis;
			int delta_x_down_left = down_x_index - left_x_index;
			int delta_x_right_down = right_x_index - down_x_index;
			pcl::PointCloud<pcl::PointXYZ>::Ptr ErCi_Calculate(new pcl::PointCloud<pcl::PointXYZ>);
			ErCi_Calculate = separate_clouds[i];
			std::vector<double> left_xielv;
			std::vector<double> right_xielv;
			for (int j = left_x_index; j > left_x_index-10; --j)
			{
				double left_kerial = (ErCi_Calculate->points[j].z - ErCi_Calculate->points[j - 1].z) / (ErCi_Calculate->points[j].x - ErCi_Calculate->points[j - 1].x);
				left_xielv.push_back(left_kerial);
			}
			for (int j = right_x_index; j < right_x_index+10; ++j)
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
				if (left_xielv[j]>= left_xielv_control)
				{
					min_left_kerial=left_xielv[j];
					min_left_index = left_x_index-j;
					break;
				}
				else
				{
					auto it = std::max_element(left_xielv.begin(), left_xielv.end());
					int idx_min = std::distance(left_xielv.begin(), it);
					min_left_index = left_x_index- idx_min;
				}
			}
			for (int i = 0; i < right_xielv.size(); i++)
			{
				if (right_xielv[i] <= right_xielv_control)
				{
					min_right_kerial = right_xielv[i];
					min_right_index = right_x_index+i;
					break;
				}
				else
				{
					auto it = std::min_element(right_xielv.begin(), right_xielv.end());
					int idx_max = std::distance(right_xielv.begin(), it);
					min_right_index =right_x_index+ idx_max;
				}
			}
			single_threePoint.leftShoulder = separate_clouds[i]->points[min_left_index];
			single_threePoint.rightShoulder = separate_clouds[i]->points[min_right_index];
			single_threePoint.downShoulder = single_threePoint.downShoulder;
			vectorThreePoints.push_back(single_threePoint);
			double width = pointDistance(single_threePoint.leftShoulder, single_threePoint.rightShoulder);
			double depth = pointToLineDistance3D(single_threePoint.leftShoulder, single_threePoint.rightShoulder, single_threePoint.downShoulder);
			width = 1000 * width + InfoSuanFa.edit_Width_A_Coefficient.toFloat();
			depth = 1000 * depth + InfoSuanFa.edit_Depth_A_Coefficient.toFloat();
			Calucate_Width_Depth.m_width.push_back(width);
			Calucate_Width_Depth.m_depth.push_back(depth);
			//double width = pointDistance(single_threePoint.leftShoulder, single_threePoint.rightShoulder);
			//double depth = pointToLineDistance3D(single_threePoint.leftShoulder, single_threePoint.rightShoulder, single_threePoint.downShoulder);

			//Calucate_Width_Depth.m_width.push_back(width);
			//Calucate_Width_Depth.m_depth.push_back(depth);

		 } 
	}
int size_width =Calucate_Width_Depth.m_width.size();
	if (vectorThreePoints.empty())
	{
		return false;
	}
	_multifyRecognize = vectorThreePoints;//记得下次执行的时候要清零
	return true;
}
//std::vector<PolePieceMeasureWin::ThreePoint> PolePieceMeasureWin::multifyRecognize(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  multifyCloud ,int pinghuaxishu)
//{
//	int sizenum = multifyCloud.size();
//	if (ui->checkbox_roi->isChecked())
//	{
//		if (multifyCloud.size()>3)
//		{
//			sizenum = 3;
//
//		}else
//		{
//			sizenum = multifyCloud.size();
//
//		}
//	} 
//	else
//	{
//		sizenum = multifyCloud.size();
//	}
//	std::vector<PolePieceMeasureWin::ThreePoint> vectorThreePoints;
//	vectorThreePoints.clear();
//	for (size_t i = 0; i < sizenum; ++i)
//	{
// 		nihe_cloud= pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//		//roi_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
//		//roi_cloud = roiAcquire(multifyCloud[i]);//划定一个ROI范围
// 		/*pcl::PointCloud<pcl::PointXYZ>::Ptr pinghua_cloud =slidingAverageZ(roi_cloud, pinghuaxishu);
//  		pinghua_cloud->width = source_cloud->size();
//		pinghua_cloud->height = 1;
//		pinghua_cloud->is_dense = true;
//		nihe_cloud = nihe_Roate(pinghua_cloud);*/
//		std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> three_jie; 
//		three_jie.clear();
// 		three_jie = threeJie(multifyCloud[i]);//一帧分三块
//  		PolePieceMeasureWin::ThreePoint single_threePoint = singleRecongize(three_jie, pinghuaxishu);//单帧识别
//		 
//		ID_NEW.push_back(i);
//		vectorThreePoints.push_back(single_threePoint);   
//	}
//	return vectorThreePoints;
//}
PolePieceMeasureWin::ThreePoint2RGBcloud PolePieceMeasureWin::ThreePointToRGBcloud(std::vector<ThreePoint> _vectorThreePoint) 
{  
	for ( int i = 0; i < _vectorThreePoint.size();++i)
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

			_ThreePoint2RGBcloud.RGB_left_Cloud->points.push_back(left_array_RGB);
			_ThreePoint2RGBcloud.RGB_right_Cloud->points.push_back(right_array_RGB);
			_ThreePoint2RGBcloud.RGB_down_Cloud->points.push_back(down_array_RGB); 
	} 
	return _ThreePoint2RGBcloud;
}
bool PolePieceMeasureWin::StartRecongize(pcl::PointCloud<pcl::PointXYZ>::Ptr source_Cloud)
{ 
 	Init_Clear(); 
	int pinghuaxishu = InfoSuanFa.editPingHuaXiShu.toInt(); //ui->editPingHuaData->text().toInt();  
	separate_clouds = separateCloud(source_cloud);

	_separateCloud =separateCloud(source_Cloud);//将一张带有多行点云分离成y值一致的单行点云 
    bool multyResult = multifyRecognize(_separateCloud, pinghuaxishu);//多组点云循环依次执行 
    return multyResult;
	//return _multifyRecognize;
}
//std::vector<PolePieceMeasureWin::ThreePoint> PolePieceMeasureWin::StartRecongize(pcl::PointCloud<pcl::PointXYZ>::Ptr source_Cloud)
//{
//	Init_Clear();
//	int pinghuaxishu = InfoSuanFa.editPingHuaXiShu.toInt(); //ui->editPingHuaData->text().toInt();  
// 	std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> _separateCloud =	separateCloud(roi_cloud);//将一张带有多行点云分离成y值一致的单行点云
//	std::vector<PolePieceMeasureWin::ThreePoint> _multifyRecognize = multifyRecognize(_separateCloud, pinghuaxishu);//多组点云循环依次执行 
//	Result_Show(); new_A_updateTable();
//	ShowThreeVectorPoint(ThreePointToRGBcloud(_multifyRecognize)); on_btnTestShow_clicked();
//	return _multifyRecognize;
//}
void PolePieceMeasureWin::Init_Clear()  
{
	if (!_ThreePoint2RGBcloud.RGB_down_Cloud->empty())_ThreePoint2RGBcloud.RGB_down_Cloud->clear(); 
	if(!_ThreePoint2RGBcloud.RGB_left_Cloud->empty())_ThreePoint2RGBcloud.RGB_left_Cloud->clear();
	if (!_ThreePoint2RGBcloud.RGB_right_Cloud->empty())_ThreePoint2RGBcloud.RGB_right_Cloud->clear(); 
	if (!Calucate_Width_Depth.m_width.empty())Calucate_Width_Depth.m_width.clear();
	if (!Calucate_Width_Depth.m_depth.empty())Calucate_Width_Depth.m_depth.clear();
	if (!ID_SELECT.empty())ID_SELECT.clear();
	if (!ID_NEW.empty())ID_NEW.clear();
	if (!_multifyRecognize.empty())_multifyRecognize.clear();
	if (!_separateCloud.empty())_separateCloud.clear();
	if (!separate_clouds.empty())separate_clouds.clear();
	if (!customPts.empty())customPts.clear();
	if (!customPts_bottom.empty())customPts_bottom.clear();
	if (!showResultPoint.empty())showResultPoint.clear();
 	ui->Single_A_PointShow->clearCustomPoints();
 	RightMostIndex = false;
	LeftMostIndex = false; 
	//    /* 第一列：宽度 */
	ui->tb_Widget_A->setItem(0, 1, new QTableWidgetItem(""));
	ui->tb_Widget_A->setItem(1, 1, new QTableWidgetItem(""));
	ui->tb_Widget_A->setItem(2, 1, new QTableWidgetItem(""));
	ui->tb_Widget_A->setItem(0, 0, new QTableWidgetItem(QStringLiteral("平均宽度")));
	ui->tb_Widget_A->setItem(1, 0, new QTableWidgetItem(QStringLiteral("最大宽度")));
	ui->tb_Widget_A->setItem(2, 0, new QTableWidgetItem(QStringLiteral("最小宽度")));
  
	ui->tb_Widget_A->setItem(0, 2, new QTableWidgetItem(QStringLiteral("平均深度")));
	ui->tb_Widget_A->setItem(1, 2, new QTableWidgetItem(QStringLiteral("最大深度")));
	ui->tb_Widget_A->setItem(2, 2, new QTableWidgetItem(QStringLiteral("最小深度")));
	ui->tb_Widget_A->setItem(0, 3, new QTableWidgetItem(""));
	ui->tb_Widget_A->setItem(1, 3, new QTableWidgetItem(""));
	ui->tb_Widget_A->setItem(2, 3, new QTableWidgetItem(""));
	ui->labelMeanWidth->setText("");
	ui->labelMaxWidth->setText("");
	ui->labelMinWidth->setText("");
	ui->labelMeanDepth->setText("");
	ui->labelMaxDepth->setText("");
	ui->labelMinDepth->setText(""); 
	ui->tableFrame->clearContents();
	ui->tableFrame->setRowCount(0);
}
void PolePieceMeasureWin::Result_A_Show()//结果显示
{
	int idx_max = 0;
	if (!Calucate_Width_Depth.m_width.empty()) {
		auto minmax_width = std::minmax_element(Calucate_Width_Depth.m_width.begin(), Calucate_Width_Depth.m_width.end());
		Calucate_Width_Depth.m_min_width = *minmax_width.first;
		Calucate_Width_Depth.m_max_width = *minmax_width.second;
		Calucate_Width_Depth.m_mean_width = std::accumulate(Calucate_Width_Depth.m_width.begin(), Calucate_Width_Depth.m_width.end(), 0.0) / Calucate_Width_Depth.m_width.size();
		// 关键：求最大值的索引
		 
	}
	if (!Calucate_Width_Depth.m_depth.empty()) {
		auto minmax_depth = std::minmax_element(Calucate_Width_Depth.m_depth.begin(), Calucate_Width_Depth.m_depth.end());
		Calucate_Width_Depth.m_min_depth = *minmax_depth.first;
		Calucate_Width_Depth.m_max_depth = *minmax_depth.second;
		Calucate_Width_Depth.m_mean_depth = std::accumulate(Calucate_Width_Depth.m_depth.begin(), Calucate_Width_Depth.m_depth.end(), 0.0) / Calucate_Width_Depth.m_depth.size();
	}
	auto ToPoint2Data = [](double v) -> QString {
		std::ostringstream o;
		o << std::fixed << std::setprecision(2) << v << "um ";
		return QString::fromStdString(o.str());
		};
	QString meanWidth = ToPoint2Data(1.000*Calucate_Width_Depth.m_mean_width);
	QString maxWidth = ToPoint2Data(1.000*Calucate_Width_Depth.m_max_width);
	QString minWidth = ToPoint2Data(1.000*Calucate_Width_Depth.m_min_width);
	 
	QString meanDepth = ToPoint2Data(1.000*Calucate_Width_Depth.m_mean_depth);
	QString maxDepth = ToPoint2Data(1.000*Calucate_Width_Depth.m_max_depth);
	QString minDepth = ToPoint2Data(1.000*Calucate_Width_Depth.m_min_depth);
	qInfo() << ("meanWidth:" +QString::number(1.000 * Calucate_Width_Depth.m_mean_width, 'f', 2))
	<< ( "maxWidth:" + QString::number(1.000 * Calucate_Width_Depth.m_max_width, 'f', 2))
		<< ("minWidth:" + QString::number(1.000 * Calucate_Width_Depth.m_min_width, 'f', 2));
	
	qInfo() << ( "meanDepth:" +QString::number(1.000 * Calucate_Width_Depth.m_mean_depth, 'f', 2))
	<< ( "maxDepth:" + QString::number(1.000 * Calucate_Width_Depth.m_max_depth, 'f', 2))
		<< ( "minDepth:" + QString::number(1.000 * Calucate_Width_Depth.m_min_depth, 'f', 2)); 
	
	_callOneTimes->class_A_B.Result_A.Measure_Depth_Result = 1.000 * Calucate_Width_Depth.m_mean_depth; 
	_callOneTimes->class_A_B.Result_A.Measure_Width_Result = 1.000 * Calucate_Width_Depth.m_mean_width;
	//    /* 第一列：宽度 */
	ui->tb_Widget_A->setItem(0, 1, new QTableWidgetItem(meanWidth));
	ui->tb_Widget_A->setItem(1, 1, new QTableWidgetItem(maxWidth));
	ui->tb_Widget_A->setItem(2, 1, new QTableWidgetItem(minWidth));
	ui->tb_Widget_A->setItem(0, 0, new QTableWidgetItem(QStringLiteral("平均宽度")));
	ui->tb_Widget_A->setItem(1, 0, new QTableWidgetItem(QStringLiteral("最大宽度")));
	ui->tb_Widget_A->setItem(2, 0, new QTableWidgetItem(QStringLiteral("最小宽度")));

	/* 第二列：空白，不用填 */ 
	/* 第三列：深度 */
	ui->tb_Widget_A->setItem(0, 2, new QTableWidgetItem(QStringLiteral("平均深度")));
	ui->tb_Widget_A->setItem(1, 2, new QTableWidgetItem(QStringLiteral("最大深度")));
	ui->tb_Widget_A->setItem(2, 2, new QTableWidgetItem(QStringLiteral("最小深度")));
	ui->tb_Widget_A->setItem(0, 3, new QTableWidgetItem(meanDepth));
	ui->tb_Widget_A->setItem(1, 3, new QTableWidgetItem(maxDepth));
	ui->tb_Widget_A->setItem(2, 3, new QTableWidgetItem(minDepth));
	ui->labelMeanWidth->setText(meanWidth);
	ui->labelMaxWidth->setText(maxWidth);
	ui->labelMinWidth->setText(minWidth);
	ui->labelMeanDepth->setText(meanDepth);
	ui->labelMaxDepth->setText(maxDepth);
	ui->labelMinDepth->setText(minDepth);
	// 判断并设置宽度标签样式
	auto setStyle = [](QLabel* label, bool highlight) {
		if (!highlight)
			label->setStyleSheet("color: red; font-weight: bold;");
		else
			label->setStyleSheet("color: rgb(0, 234, 0);");
		};
	//InfoSuanFa.editDetectedWidthMin.toDouble() <= width_avg <= InfoSuanFa.editDetectedWidthMax.toDouble()
	setStyle(ui->labelMeanWidth,  InfoSuanFa.editDetectedWidthMin.toDouble() <= Calucate_Width_Depth.m_mean_width <= InfoSuanFa.editDetectedWidthMax.toDouble());
	setStyle(ui->labelMaxWidth, InfoSuanFa.editDetectedWidthMin.toDouble() <= Calucate_Width_Depth.m_max_width <= InfoSuanFa.editDetectedWidthMax.toDouble());
	setStyle(ui->labelMinWidth, InfoSuanFa.editDetectedWidthMin.toDouble() <= Calucate_Width_Depth.m_min_width <= InfoSuanFa.editDetectedWidthMax.toDouble());

	// 判断并设置深度标签样式
	setStyle(ui->labelMeanDepth, InfoSuanFa.editDetectedDepthMin.toDouble() <= Calucate_Width_Depth.m_mean_depth <= InfoSuanFa.editDetectedDepthMax.toDouble());
	setStyle(ui->labelMaxDepth, InfoSuanFa.editDetectedDepthMin.toDouble() <= Calucate_Width_Depth.m_max_depth <= InfoSuanFa.editDetectedDepthMax.toDouble());
	setStyle(ui->labelMinDepth, InfoSuanFa.editDetectedDepthMin.toDouble() <= Calucate_Width_Depth.m_min_depth <= InfoSuanFa.editDetectedDepthMax.toDouble());
	bool A_Width_R = InfoSuanFa.editDetectedWidthMin.toDouble() <= 1.000*Calucate_Width_Depth.m_mean_width && 1.000*Calucate_Width_Depth.m_mean_width <= InfoSuanFa.editDetectedWidthMax.toDouble();
	bool A_Depth_R = InfoSuanFa.editDetectedDepthMin.toDouble() <= 1.000*Calucate_Width_Depth.m_mean_depth && 1.000*Calucate_Width_Depth.m_mean_depth <= InfoSuanFa.editDetectedDepthMax.toDouble();
	bool Fina_R = A_Width_R && A_Depth_R;
	if(Fina_R)
	{
		ui->Result_A->setText("OK"); 
		ui->Result_A->setStyleSheet("color: rgb(0,255,0);");     // 绿色
		QString path = CreateOkNgTimeDir(ParamUi->editOutPath->text(), true) + "/OK.ply";
		int result = pcl::io::savePLYFileBinary(path.toStdString(), *source_cloud);
		if (result != 0)
		{
			RecordData(QStringLiteral("OK点云保存失败"), 2);
		}
	}
	else
	{ 
		ui->Result_A->setText("NG"); 
		ui->Result_A->setStyleSheet("color: rgb(255,0,0);");          // 红色 
		QString path = CreateOkNgTimeDir(ParamUi->editOutPath->text(), false)+"/NG.ply";//创建 年月日/OK|NG/时刻 目录;//创建 年月日/OK|NG/时刻 目录
		int result = pcl::io::savePLYFileBinary(path.toStdString(), *source_cloud);
		if (result != 0)
		{
			RecordData(QStringLiteral("NG点云保存失败"), 2);
		}
	}
	//// 添加自定义点，颜色绿色，半径8
	//QVector<QPointF> customPts;
	////customPts.clear();
	//QVector<QPointF> customPts_bottom;
	////customPts_bottom.clear();
	customPts << QPointF(_multifyRecognize[ID_SELECT[idx_max]].leftShoulder.x , _multifyRecognize[ID_SELECT[idx_max]].leftShoulder.z) << QPointF(_multifyRecognize[ID_SELECT[idx_max]].rightShoulder.x , _multifyRecognize[ID_SELECT[idx_max]].rightShoulder.z);
	customPts_bottom << QPointF(_multifyRecognize[ID_SELECT[idx_max]].downShoulder.x , _multifyRecognize[ID_SELECT[idx_max]].downShoulder.z);
	//显示单帧的时候先找到最低点那个位置的X值，然后将可以映射回原来的点云的序列号，然后根据序列号来截取ROI来进行显示；//separate_clouds
	//int index = qRound(_multifyRecognize[ID_SELECT[idx_max]].downShoulder.x / 0.003);JZW
	int index = qRound(_multifyRecognize[ID_SELECT[idx_max]].downShoulder.x / (0.001 * InfoSuanFa.editCloudShowDis.toFloat()));
	pcl::PointCloud<pcl::PointXYZ>::Ptr slice(new pcl::PointCloud<pcl::PointXYZ>);
	/*for (int i = index - 440; i < index + 440;++i) 
	{
		roi_show->points.push_back(	separate_clouds[ID_SELECT[0]]->points[i]);
	}*/
	std::size_t start = index - 45;
	std::size_t end = index + 45;
	slice->points.assign(separate_clouds[ID_SELECT[idx_max]]->points.begin() + start,
		separate_clouds[ID_SELECT[idx_max]]->points.begin() + end);
	slice->width = slice->points.size();
	slice->height = 1;
	slice->is_dense = separate_clouds[ID_SELECT[idx_max]]->is_dense;
	ui->Single_A_PointShow->addCustomPoints(customPts, Qt::green, 5);
	ui->Single_A_PointShow->addCustomPoints(customPts_bottom, Qt::red, 5);
	ui->Single_A_PointShow->setPointCloud(slice);
	//ui->Single_A_PointShow->setPointCloud(showResultPoint[0]);
}
void PolePieceMeasureWin::ShowThreeVectorPoint(PolePieceMeasureWin::ThreePoint2RGBcloud _ThreePoint2RGBcloud)
{
	if (ui->checkbox_roi->isChecked())
	{
		updateViewer(roi_cloud, _ThreePoint2RGBcloud.RGB_left_Cloud, _ThreePoint2RGBcloud.RGB_right_Cloud, _ThreePoint2RGBcloud.RGB_down_Cloud); 
	}
	else
	{
		updateViewer(source_cloud, _ThreePoint2RGBcloud.RGB_left_Cloud, _ThreePoint2RGBcloud.RGB_right_Cloud, _ThreePoint2RGBcloud.RGB_down_Cloud); 
	}
}
pcl::PointCloud<pcl::PointXYZ>::Ptr PolePieceMeasureWin::nihe_Roate(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_filtered = *inputCloud;  // 直接赋值复制点云数据
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud_filtered);
	sor.setMeanK(50);            // 计算每个点邻域点数
	sor.setStddevMulThresh(1.0); // 标准差倍数阈值，越小越严格
	sor.filter(*cloud_filtered);


	//// RANSAC直线拟合
	//pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	//pcl::SACSegmentation<pcl::PointXYZ> seg;
	//double dist = ui->editRANSAC->text().toDouble(); // 直线拟合距离阈值
	//if (dist < 0.01)dist = 0.01;
	//if (dist > 100)dist = 100;
	//seg.setOptimizeCoefficients(true);
	//seg.setModelType(pcl::SACMODEL_LINE);
	//seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setDistanceThreshold(dist);
	//seg.setMaxIterations(5000);  // 最大迭代次数
	//seg.setInputCloud(cloud_filtered);
	//seg.segment(*inliers, *coefficients);

	//// coefficients->values: [点x, 点y, 点z, 方向x, 方向y, 方向z]
	//if (coefficients->values.size() < 6) {
	//	//ui->statusLabel->setAnimatedText(QStringLiteral("RANSAC拟合失败"));
	//	return nullptr; // 拟合失败
	//}
	//if (inliers->indices.size() < 10) {
	//	//ui->statusLabel->setAnimatedText(QStringLiteral("内点数过少"));
	//	return nullptr; // 内点数过少
	//}

	//// 粗拟合直线方向（近似主方向）
	//Eigen::Vector3d line_dir(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
	//line_dir.normalize();
	//Eigen::Vector3f line_dir_f(line_dir[0], line_dir[1], line_dir[2]); // double->float转换
	//line_dir_f.normalize();

	//// 计算夹角阈值的余弦值
	//Eigen::Vector3f ref_dir(1.0f, 0.0f, 0.0f);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud(new pcl::PointCloud<pcl::PointXYZ>);
 //
	//// 计算旋转角度绕 Y 轴
	//float angle = std::atan2(line_dir_f.z(), line_dir_f.x());
	//Eigen::AngleAxisf angle_axis(angle, Eigen::Vector3f::UnitY());
	//Eigen::Matrix3f rotation = angle_axis.toRotationMatrix();// 绕 Y 轴旋转矩阵
	//Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
 // 	// 构造齐次变换矩阵
	//transform.block<3, 3>(0, 0) = rotation;

	//// 应用变换
	//pcl::transformPointCloud(*inputCloud, *Cloud, transform);
 //	return Cloud;

	// Step 1: PCA获取主方向（在XZ平面内）
	pcl::PCA<pcl::PointXYZ> pca;
	pca.setInputCloud(cloud_filtered);
	//Eigen::Vector3f eigen_vector = pca.getEigenVectors(0); // 最大方差方向
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();   // 3×3 矩阵，列向量按特征值降序
	Eigen::Vector3f main_direction = eigen_vectors.col(0);
	// Step 2: 投影到XZ平面（Y=0），忽略Y分量
	Eigen::Vector2f direction(main_direction[0], main_direction[2]);
	direction.normalize();

	// Step 3: 计算与X轴的夹角
	Eigen::Vector2f x_axis(1.0f, 0.0f);
	float angle = std::atan2(direction[1], direction[0]); // 弧度
	float rotation_angle = angle; // 反向旋转，使其对齐X轴

	// Step 4: 构造绕Y轴的旋转矩阵
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(Eigen::AngleAxisf(rotation_angle, Eigen::Vector3f::UnitY()));

	// Step 5: 应用变换
	pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*inputCloud, *aligned_cloud, transform);

	return aligned_cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PolePieceMeasureWin::slidingAverageZ(
	const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int half_w) {
	// 创建输出点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>(*cloud));
	int half_window = half_w / 2;
	int n = cloud->size();

	for (int i = 0; i < n; ++i) {
		double sum_z = 0.0;
		int count = 0;

		// 计算前后窗口内点的Z值和
		for (int j = -half_window; j <= half_window; ++j) {
			int idx = i + j;
			if (idx >= 0 && idx < n) {
				sum_z += cloud->points[idx].z;
				count++;
			}
		}

		// 设置平均Z值
		if (count > 0) {
			filtered_cloud->points[i].z = sum_z / count;
		}
	}

	return filtered_cloud;
}
