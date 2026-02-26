#pragma once
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>

#include <iostream>
#include <algorithm>
#include <time.h>

#include <pcl/visualization/cloud_viewer.h> //类cloud_viewer头文件申明
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/io/io.h>                      //I/O相关头文件申明
#include <pcl/io/pcd_io.h>                  //PCD文件读取
#include <pcl/io/ply_io.h>                  //PLY文件读取
#include <pcl/point_types.h>                //点类型定义头文件申明

#include <vtkGenericOpenGLRenderWindow.h>
#include "QVTKOpenGLNativeWidget.h"           //新版本，旧版QVTKWidget
#include "vtkAutoInit.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkProperty.h"
#include "vtkCamera.h"
#include "vtkLight.h"

#include <QtWidgets/QMainWindow>
#include <QFileDialog>
#include <qfileinfo.h>
#include <QMessageBox>
#include <QDir>
#include <qstringlist.h>
#include <qstring.h>
#include <QTextCodec>
#include <QtCharts/qchartview.h>
#include <QtCharts/qsplineseries.h>
#include <QtSerialPort/qserialport.h>
#include <QtSerialPort/qserialportinfo.h>
#include <QMouseEvent>

 #include"hgpcl_trans.h"
 
//QT_CHARTS_USE_NAMESPACE

class ChartView : public QtCharts::QChartView
 {
	 Q_OBJECT
public:
	ChartView(QWidget* parent = nullptr);
	//ChartView(QtCharts::QChart * chart, QWidget * parent = nullptr);
	 ~ChartView();
	// 保存坐标区域，用于复位
	//void saveAxisRange();
	/* void showPointCloud(pcl::PointCloud<pcl::PointXYZ>& input);
	 void showPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ>>& input);
	 void showPointCloud(pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointCloud<pcl::PointXYZ>& input2);
	 void showPointCloud_polyline(pcl::PointCloud<pcl::PointXYZ>& input);
	 void showPointCloud_polyline(std::vector<pcl::PointCloud<pcl::PointXYZ>>& input);
	 void showPointCloud_polyline(pcl::PointCloud<pcl::PointXYZ>& input1, pcl::PointCloud<pcl::PointXYZ>& input2);*/

protected:
	//void mousePressEvent(QMouseEvent * event);
	//void mouseMoveEvent(QMouseEvent * event);
	//void mouseReleaseEvent(QMouseEvent * event);
	////void wheelEvent(QWheelEvent * event);
	////void keyPressEvent(QKeyEvent * event);
	////void keyReleaseEvent(QKeyEvent * event);
	//void resizeEvent(QResizeEvent* event) override;

 public: Q_SIGNALS:
	 /** 鼠标在图表位置的数据改变时触发 **/
	 //void onMousePositionValueChanged(QPointF valueGivenSeries);
	 //void onMousePositionValueChangedLeftPress(QPointF valueGivenSeries);
	 //void onMousePositionValueChangedLeftMove(QPointF valueGivenSeries);
	 //void onMousePositionValueChangedLeftRelease(QPointF valueGivenSeries);
	 //void onMousePositionValueChangedRightPress(QPointF valueGivenSeries);
	 ////图表大小变化信号
	 //void chartSizeChanged(QSize size);

private:
	//QPoint m_lastPoint;//鼠标按下的点
	//bool m_isLeftPress, m_isRightPress;//鼠标是否按下
	////bool m_ctrlPress;
	//bool m_alreadySaveRange;
	//double m_xMin, m_xMax, m_yMin, m_yMax;
	//QGraphicsSimpleTextItem * m_coordItem;

	};