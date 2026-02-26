#include"ChartView.h"
ChartView::ChartView(QWidget* parent) :QtCharts::QChartView(parent)
{
	m_isLeftPress = false;
	m_isRightPress = false;
	m_alreadySaveRange = false;
	m_xMin = 0;
	m_xMax = 0;
	m_yMin = 0;
	m_yMax = 0;
}
ChartView::ChartView(QtCharts::QChart* chart, QWidget* parent) : QtCharts::QChartView(chart, parent)
{

}
ChartView::~ChartView()
{

}

void ChartView::showPointCloud_polyline(pcl::PointCloud<pcl::PointXYZ>& input)
{
	
}

void ChartView::mousePressEvent(QMouseEvent* event)
{
	if (event->button() == Qt::LeftButton)
	{
		auto const widgetPos = event->pos();
		auto const scenePos = this->chart()->mapToScene(QPoint(static_cast<int>(widgetPos.x()), static_cast<int>(widgetPos.y())));
		auto const chartItemPos = chart()->mapFromScene(scenePos);
		auto const valueGivenSeries = chart()->mapToValue(chartItemPos);
		//    qDebug() << "widgetPos:" << widgetPos;
		//    qDebug() << "scenePos:" << scenePos;
		//    qDebug() << "chartItemPos:" << chartItemPos;
			// 鼠标所在位置的图表X，Y坐标值
		//    qDebug() << "valSeries:" << valueGivenSeries;
		emit onMousePositionValueChanged(valueGivenSeries);
		m_isLeftPress = true;
	}
	//鼠标右键按下
	if (event->button() == Qt::RightButton)
	{
		auto const widgetPos = event->pos();
		auto const scenePos = this->chart()->mapToScene(QPoint(static_cast<int>(widgetPos.x()), static_cast<int>(widgetPos.y())));
		auto const chartItemPos = chart()->mapFromScene(scenePos);
		auto const valueGivenSeries = chart()->mapToValue(chartItemPos);
		//    qDebug() << "widgetPos:" << widgetPos;
		//    qDebug() << "scenePos:" << scenePos;
		//    qDebug() << "chartItemPos:" << chartItemPos;
			// 鼠标所在位置的图表X，Y坐标值
		//    qDebug() << "valSeries:" << valueGivenSeries;
		emit onMousePositionValueChangedRightPress(valueGivenSeries);
		m_isRightPress = true;
	}
}
void ChartView::mouseReleaseEvent(QMouseEvent* event)
{
	 if (event->button() == Qt::LeftButton)
	 {
		 m_isLeftPress = false;
		 auto const widgetPos = event->pos();
		 auto const scenePos = this->chart()->mapToScene(QPoint(static_cast<int>(widgetPos.x()), static_cast<int>(widgetPos.y())));
		 auto const chartItemPos = chart()->mapFromScene(scenePos);
		 auto const valueGivenSeries = chart()->mapToValue(chartItemPos);
		 emit onMousePositionValueChangedLeftRelease(valueGivenSeries);
	 }
	 if (event->button() == Qt::RightButton)
	 {
		 m_isRightPress = false;

	 }
}
void ChartView::resizeEvent(QResizeEvent* event)
{
	QChartView::resizeEvent(event);
	// 自定义的尺寸变化逻辑
	chart()->resize(event->size()); // 调整图表大小以适应新尺寸
	emit chartSizeChanged(event->size());
}
void ChartView::mouseMoveEvent(QMouseEvent* event)
{
	//鼠标左键按下触发左键移动信号
	if (m_isLeftPress)
	{
		auto const widgetPos = event->pos();
		auto const scenePos = this->chart()->mapToScene(QPoint(static_cast<int>(widgetPos.x()), static_cast<int>(widgetPos.y())));
		auto const chartItemPos = chart()->mapFromScene(scenePos);
		auto const valueGivenSeries = chart()->mapToValue(chartItemPos);
		//    qDebug() << "widgetPos:" << widgetPos;
		//    qDebug() << "scenePos:" << scenePos;
		//    qDebug() << "chartItemPos:" << chartItemPos;
		// 鼠标所在位置的图表X，Y坐标值
		//    qDebug() << "valSeries:" << valueGivenSeries;
		emit onMousePositionValueChangedLeftMove(valueGivenSeries);
	}
}
