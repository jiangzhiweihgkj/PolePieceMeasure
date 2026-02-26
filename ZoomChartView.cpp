#include "ZoomChartView.h"
#include <QtCharts/QValueAxis>
#include <QtGui/QMouseEvent>
#include <QtGui/QWheelEvent>

ZoomChartView::ZoomChartView(QChart* chart, QWidget* parent)
    : QChartView(chart, parent)
{
    //setRubberBand(QChartView::RectangleRubberBand); // 框选放大
    //setInteractive(true);
}

/* 滚轮：以鼠标位置为中心缩放 */
void ZoomChartView::wheelEvent(QWheelEvent* ev)
{
    const double factor = ev->angleDelta().y() > 0 ? 0.8 : 1.25; // 放大系数
    chart()->zoom(factor);                                       // Qt 自带缩放
    ev->accept();
}

/* 拖拽平移 */
void ZoomChartView::mousePressEvent(QMouseEvent* ev)
{
    if (ev->button() == Qt::LeftButton) {
        m_drag = true;
        m_last = ev->pos();
        setCursor(Qt::ClosedHandCursor);
    }
    QChartView::mousePressEvent(ev);
}

void ZoomChartView::mouseMoveEvent(QMouseEvent* ev)
{
    if (m_drag) {
        QPoint delta = ev->pos() - m_last;
        chart()->scroll(-delta.x(), delta.y()); // 负号：拖右→左移坐标
        m_last = ev->pos();
    }
    QChartView::mouseMoveEvent(ev);
}

void ZoomChartView::mouseReleaseEvent(QMouseEvent* ev)
{
    if (ev->button() == Qt::LeftButton) {
        m_drag = false;
        setCursor(Qt::ArrowCursor);
    }
    QChartView::mouseReleaseEvent(ev);
}