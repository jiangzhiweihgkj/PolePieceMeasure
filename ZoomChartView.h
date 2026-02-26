#pragma once
#include <QtCharts/QChartView>
#include <QtCore/QPointF>

QT_CHARTS_USE_NAMESPACE

class ZoomChartView : public QChartView
{
    Q_OBJECT
public:
    explicit ZoomChartView(QChart* chart, QWidget* parent = nullptr);

protected:
    void wheelEvent(QWheelEvent* ev) override;
    void mousePressEvent(QMouseEvent* ev) override;
    void mouseMoveEvent(QMouseEvent* ev) override;
    void mouseReleaseEvent(QMouseEvent* ev) override;

private:
    bool m_drag = false;
    QPoint m_last;
};
