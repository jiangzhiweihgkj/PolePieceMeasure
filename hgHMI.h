#ifndef HGHMI_H
#define HGHMI_H

// Qt 核心模块和控件
#include <QObject>
#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <QFontMetrics>
#include <QDateTime>
#include <QLineEdit>
#include <QRadioButton>
#include <QCheckBox>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPlainTextEdit>
#include <QTextEdit>
#include <QSettings>
#include <QPair>
#include <QPointer>
#include <QList>
#include <QStack>

// Qt 布局和界面组件
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QScrollArea>
#include <QFrame>
#include <QTableWidget>
#include <QHeaderView>
#include <QDialog>

// Qt 图形视图框架相关
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsWidget>
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsProxyWidget>
#include <QGraphicsPixmapItem>
#include <QGraphicsSceneMouseEvent>

// Qt 绘图相关
#include <QPainter>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QPen>
#include <QBrush>
#include <QPointF>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QGraphicsOpacityEffect>

// VTK 相关
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRendererCollection.h>
#include <vtkObjectFactory.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSmartPointer.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkAxesActor.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkLineSource.h>
#include <vtkPolyDataMapper.h>

// PCL 相关
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>

// 自定义库函数
#include "hgCloud_Analysis.h"

// Boost 库
#include <boost/shared_ptr.hpp>

// C++ 标准库
#include <vector>
#include <stack>
#include <limits>
#include <chrono>
#include <functional>
 
// ********** hgHMI_Ini 配置文件读写类（外部仅需设置映射关系，支持修改检查） **********
class hgHMI_Ini : public QObject
{
    Q_OBJECT
public:
    explicit hgHMI_Ini(const QString& iniFilePath, QObject* parent = nullptr);

    // 设置控件-key映射容器
    // 参数格式：QList<QPair<QWidget*, QString>>
    void setWidgetKeyMap(const QList<QPair<QWidget*, QString>>& widgetKeyPairs);

    // 读取INI到控件
    void load();

    // 保存控件当前值到INI
    void save();

    // 判断是否改变
    bool isChanged() const;

	// 同步读取和保存
    void sync();

private:
    QSettings m_settings;
    QList<QPair<QWidget*, QString>> m_widgetKeyPairs;

    // 内部辅助方法根据控件类型处理
    QVariant readValue(const QString& key) const;
    void writeValue(const QString& key, const QVariant& value);

    bool isWidgetChanged(QWidget* widget, const QString& key) const;
    void loadWidget(QWidget* widget, const QString& key);
    void saveWidget(QWidget* widget, const QString& key);
};

// ********** hgHMI_Pushbutton 控件 (自定义按钮，右上角带小灯显示触发状态) **********
class hgHMI_Pushbutton : public QPushButton
{
    Q_OBJECT
public:
    explicit hgHMI_Pushbutton(QWidget* parent = nullptr);
    explicit hgHMI_Pushbutton(const QString& text, QWidget* parent = nullptr);

    QColor normalColor = QColor(80, 80, 80);      // 按钮背景色
    QColor pressedColor = QColor(60, 60, 60);     // 按下背景色
    QColor lightColorOff = QColor(180, 180, 180); // 指示灯灰色
    QColor lightColorOn = QColor(0, 200, 0);      // 指示灯绿色

    void setLightVisible(bool visible);
    bool isLightVisible() const;

    void setLightOn(bool on);
    bool isLightOn() const;

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    bool indicatorOn = false;
    bool indicatorVisible = true; // 默认显示指示灯
    bool pressed = false;
};

// ********** hgHMI_StatusLabel 控件 （自定义Label，更新文字时带渐变效果） **********
class hgHMI_StatusLabel : public QLabel
{
    Q_OBJECT
public:
    explicit hgHMI_StatusLabel(QWidget* parent = nullptr);
    void set_Text(const QString& newText);
    void onFadeAnimFinished();

private:
    QGraphicsOpacityEffect* opacityEffect;
    QPropertyAnimation* fadeAnim;
    bool fadingOut;
    QString m_newText;
};

// ********** hgHMI_MinimizeLabel 控件（点击后可让窗口最小化） **********
class hgHMI_MinimizeLabel : public QLabel
{
    Q_OBJECT
public:
    explicit hgHMI_MinimizeLabel(QWidget* parent = nullptr);

protected:
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;

private slots:
    void onSingleClickTimeout();

private:
    QTimer m_singleClickTimer;
    bool m_doubleClickFlag = false;
};

// ********** hgHMI_LogWidget 控件（日志表格控件） **********
class hgHMI_LogWidget : public QWidget
{
    Q_OBJECT
public:
    enum AlarmLevel {
        Info,
        Warning,
        Error
    };

    explicit hgHMI_LogWidget(QWidget* parent = nullptr);

    // 添加报警日志
    void addLog(const QString& message, AlarmLevel level = Info);

    // 清空所有日志
    void clearLogs();

    // 获取当前日志条数
    int logCount() const;

private:
    QTableWidget* table;

    QColor levelColor(AlarmLevel level) const;
    QString currentTimestamp() const;
};

// ********** hgHMI_LogTextEdit 控件（带时间和等级区分的日志控件） **********
class hgHMI_LogTextEdit : public QTextEdit
{
    Q_OBJECT
public:
    enum AlarmLevel {
        Info,
        Warning,
        Error
    };

    explicit hgHMI_LogTextEdit(QWidget* parent = nullptr);

    // 添加日志，带等级标记
    void addLog(const QString& message, AlarmLevel level = Info);
    void addLog(const QString& message, int level = 0);
};

// ********** hgHMI_TimerStatus 控件（自定义状态栏，自动刷新时间显示） **********
class hgHMI_TimerStatus : public QWidget
{
    Q_OBJECT
public:
    explicit hgHMI_TimerStatus(QWidget* parent = nullptr);
    ~hgHMI_TimerStatus();

private slots:
    void updateTime();

private:
    QLabel* timeLabel;
    QTimer* timer;
};

// ********** hgHMI_VelueBar 控件（根据传入值和设定阈值显示进度条） **********
class hgHMI_VelueBar : public QWidget
{
    Q_OBJECT
public:
    enum Orientation {
        Horizontal,
        Vertical
    };

    explicit hgHMI_VelueBar(QWidget* parent = nullptr);

    void setMaximum(int max);
    void setValue(int val);
    int value() const { return m_value; }
    int maximum() const { return m_maximum; }

    void setOrientation(Orientation ori);
    Orientation orientation() const { return m_orientation; }

    // 设置报警阈值，value >= warningThreshold时显示黄色，value >= alarmThreshold时显示红色
    void setWarningThreshold(int warningThreshold);
    void setAlarmThreshold(int alarmThreshold);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    int m_value = 0;
    int m_maximum = 100;

    Orientation m_orientation = Horizontal;

    int m_warningThreshold = 70;
    int m_alarmThreshold = 90;

    QColor m_colorNormal = QColor(0, 200, 0);      // 绿色
    QColor m_colorWarning = QColor(255, 165, 0);   // 橙黄
    QColor m_colorAlarm = QColor(255, 0, 0);       // 红色
};

// ********** hgHMI_ProgressBar 控件（进度条控件） **********
class hgHMI_ProgressBar : public QWidget
{
    Q_OBJECT
public:
    explicit hgHMI_ProgressBar(QWidget* parent = nullptr);

    // 设置当前进度，线程安全的槽函数，可以跨线程调用
public slots:
    void setProgress(int value); // 0~100

    // 重置进度条到0，准备下次使用
    void reset();

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    int m_progress;         // 当前进度0~100
    bool m_finished;        // 是否已完成任务

    QColor m_colorRunning = QColor(0, 120, 215); // 运行中蓝色
    QColor m_colorFinished = QColor(0, 200, 0);  // 完成绿色
};

// ********** hgHMI_FoldableGroupBox 控件（可折叠展开的分组框） **********
class hgHMI_FoldableGroupBox : public QGroupBox
{
    Q_OBJECT
public:
    explicit hgHMI_FoldableGroupBox(QWidget* parent = nullptr);
    explicit hgHMI_FoldableGroupBox(const QString& title, QWidget* parent = nullptr);

    bool isFolded() const;               // 返回当前是否折叠状态
    void setFolded(bool folded);         // 设置折叠或展开状态
    int m_foldedExtraHeight = 20;                    // 折叠时额外高度

    // 传入"blue", "yellow", "green"等关键词时应用内置样式，
    void setStyle(const QString& styleNameOrSheet);// 传入其他字符串应用为样式内容，传空字符串清除样式。

protected:
    void mousePressEvent(QMouseEvent* event) override;  // 鼠标点击事件，用于点击标题栏折叠/展开
    void showEvent(QShowEvent* event) override;         // 窗口显示事件，更新高度
    void resizeEvent(QResizeEvent* event) override;     // 窗口大小变化事件，重新计算高度
	void mouseMoveEvent(QMouseEvent* event) override;         // 鼠标移动事件，更新标题栏悬停状态
	void leaveEvent(QEvent* event) override;                     // 鼠标离开事件，更新标题栏悬停状态

private slots:
    void onAnimationFinished();             // 动画结束槽函数，处理折叠后隐藏内容等

private:
    void toggleFolded();                    // 切换折叠状态
    void collectContentWidgets();           // 收集内容子控件，用于隐藏和显示
    int getTitleHeight() const;             // 获取标题栏的高度

    QString styleSheetBlue() const;
    QString styleSheetYellow() const;
    QString styleSheetGreen() const;
    QString styleSheetDark() const;

    QPropertyAnimation* m_animation = nullptr;  // 控制高度变化的动画
    bool m_folded = false;                      // 当前是否折叠状态
    int m_titleHeight = 25;                       // 标题栏高度（动态获取）
    int m_expandedHeight = 0;                     // 展开时的总高度（动态计算）
    bool m_titleHovered = false;

    QList<QPointer<QWidget>> m_contentWidgets;   // 内容控件列表，通过智能指针避免悬空指针
};

// ********** hgHMI_VTKCheck 类（VTK交互样式，支持鼠标单击事件） **********
class hgHMI_VTKCheck : public vtkInteractorStyleTrackballCamera
{
public:
    static hgHMI_VTKCheck* New();
    vtkTypeMacro(hgHMI_VTKCheck, vtkInteractorStyleTrackballCamera);

    hgHMI_VTKCheck();

    // 设置左/右球尺寸，<=0隐藏，>0为半径
    void setSphereSize(bool isLeft, double size);

    // 左键单点拾取回调，参数为3D坐标，失败时为NaN
    std::function<void(double, double, double)> clickCallback;

    // 右键两点距离拾取回调，参数为两点坐标及距离
    std::function<void(double, double, double, double, double, double, double)> distanceCallback;

    // 鼠标左键事件
    virtual void OnLeftButtonDown() override;
    virtual void OnLeftButtonUp() override;

    // 鼠标右键事件，支持两点距离拾取
    virtual void OnRightButtonDown() override;
    virtual void OnRightButtonUp() override;
    virtual void OnMouseMove() override;

	// 鼠标中键事件
    virtual void OnMiddleButtonDown() override;
    virtual void OnMiddleButtonUp() override;


private:
    bool rightDragging = false;
    int rightDownX = 0, rightDownY = 0;
    std::chrono::steady_clock::time_point rightMouseDownTime;
    static constexpr double rightDragThreshold = 5.0;
    bool rightButtonPressed = false;  // 右键是否按下

    vtkSmartPointer<vtkActor> lineActor;          // 两点连线actor
    vtkSmartPointer<vtkActor> leftSphereActor;    // 左球actor（红）
    vtkSmartPointer<vtkActor> rightSphereActor1;  // 右球第1点actor（蓝）
    vtkSmartPointer<vtkActor> rightSphereActor2;  // 右球第2点actor（蓝）

    double leftSphereSize = 0.0;   // 左球半径，0隐藏
    double rightSphereSize = 0.0;  // 右球半径，0隐藏

    int mouseDownX = -1, mouseDownY = -1;  // 左键按下时鼠标位置
    std::chrono::steady_clock::time_point mouseDownTime;

    static constexpr double maxClickMoveDistance = 5.0;  // 左键最大移动距离判定点击
    static constexpr int maxClickDuration = 300;         // 左键最大点击时长（ms）

    double point1[3]{ 0,0,0 };  // 右键第1点坐标
    double point2[3]{ 0,0,0 };  // 右键第2点坐标
    bool hasPoint1 = false;      // 是否已有第1点
    std::chrono::steady_clock::time_point rightClickTime;

    static constexpr int minRightClickIntervalMs = 10;    // 右键两点间最小间隔（ms）
    static constexpr int maxRightClickIntervalMs = 10000; // 右键两点间最大间隔（ms）

    // 新建或更新球actor，center为球心，color为RGB，radius为半径
    void UpdateSphereActor(double center[3], vtkSmartPointer<vtkActor>& actor,
        double color[3], double radius = 5.0);

    // 新建或更新连线actor，连接p1和p2
    void UpdateLineActor(const double p1[3], const double p2[3]);

    // 从渲染器移除actor并置空
    void RemoveActor(vtkSmartPointer<vtkActor>& actor);
};

// ********** hgHMI_VTKdoubleCheck 类（VTK交互样式，支持鼠标双击事件） **********
class hgHMI_VTKdoubleCheck : public hgHMI_VTKCheck
{
public:
    static hgHMI_VTKdoubleCheck* New();
    vtkTypeMacro(hgHMI_VTKdoubleCheck, hgHMI_VTKCheck);

    hgHMI_VTKdoubleCheck();

    // 双击回调接口，只在第二次点击时调用
    std::function<void(double, double, double)> doubleClickCallback;

    // 重写左键抬起事件，实现双击逻辑
    void OnLeftButtonUp() override;
	void OnLeftButtonDown() override;

private:
    // 记录按下时鼠标位置和时间
    int mouseDownX = -1, mouseDownY = -1;
    std::chrono::steady_clock::time_point mouseDownTime;

    // 单击最大移动距离和最大按下时长（复用基类设计）
    static constexpr double maxClickMoveDistance = 5.0;
    static constexpr int maxClickDuration = 500; // ms

    // 双击最大间隔时间
    static constexpr int maxDoubleClickInterval = 500; // ms

    // 记录单击次数和第一次点击时间
    int clickCount = 0;
    std::chrono::steady_clock::time_point firstClickTime;

    // 保存第一次点击坐标（可选）
    double firstClickPos[3];
    bool hasFirstClickPos = false;

protected:
    // 重用基类单击拾取但屏蔽回调的辅助函数
    void PickPointNoCallback(int x, int y);
};

// ********** hgHMI_PCLViewerWidget 控件（集成PCL点云显示/拾取/视角重置等功能） **********
class hgHMI_PCLViewerWidget : public QVTKOpenGLNativeWidget
{
    Q_OBJECT
public:
    explicit hgHMI_PCLViewerWidget(QWidget* parent = nullptr);
    ~hgHMI_PCLViewerWidget();

    // 公开成员，方便外部访问底层渲染器和工具
    vtkSmartPointer<vtkRenderer> m_renderer;          // vtk渲染器
    vtkSmartPointer<vtkAxesActor> m_axesActor;        // 坐标轴actor
    pcl::visualization::PCLVisualizer::Ptr m_viewer;  // PCL可视化接口

    // 坐标轴显示控制接口
    void setAxesVisible(bool visible);
    void setAxesLength(double x_len, double y_len, double z_len);
    void setAxesShaftType(int shaftType); // 1: 线, 2: 圆柱, 3: 用户自定义
    void setAxesShowLabels(bool showLabels);
    void setAxesShowArrow(bool showArrow);

    // 获取当前坐标轴显示状态
    bool isAxesVisible() const { return m_axesVisible; }

    // 清空当前点云和图形
    void clear();

    // 智能指针版本显示函数
    void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
    // 点云版本显示函数
    void showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);
    void showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud);

	// 点拾取回调接口
    using HgHMI_PointPickCallback = std::function<void(double x, double y, double z)>;
    void setPointPickCallback(HgHMI_PointPickCallback cb);

    // 距离回调类型定义
    using HgHMI_DistancePickCallback = std::function<void(
        double x1, double y1, double z1,
        double x2, double y2, double z2,
        double dist)>;
    HgHMI_DistancePickCallback m_distancePickCallback; // 右键两点距离回调
    void setDistancePickCallback(HgHMI_DistancePickCallback cb);

    // 球体尺寸设置接口（true左球，false右球，size为半径，0隐藏）
    void setPickSphereSize(bool isLeft, double size);

	// 视角重置接口
    void resetCameraToTop();
    void resetCameraToBox(double minX, double minY, double minZ, double maxX, double maxY, double maxZ);
    void setCameraUpVector(float up_x, float up_y, float up_z);

    // 方便调试：快速弹窗显示点云（值或智能指针）
    void showDebugCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, bool newWindow = false);
    void showDebugCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool newWindow = false);
    void showDebugCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, bool newWindow = false);
    void showDebugCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, bool newWindow = false);

protected:
    void initializeVTKScene();
    void computeCloudRange(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);
    void computeCloudRange(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);

private:
    Eigen::Vector3f m_cloudCenter{}, m_cloudMin{}, m_cloudMax{};
    HgHMI_PointPickCallback m_pointPickCallback;

    // 坐标轴状态和参数缓存
    bool m_axesVisible = false;
    double m_axesLength[3] = { 100.0, 100.0, 100.0 };
    Eigen::Vector3f m_camPos{ 0,0,0 };   // 当前相机位置
    Eigen::Vector3f m_camView{ 0,0,0 };  // 当前相机目标点
    Eigen::Vector3f m_camUp{ 0,1,0 };    // 当前相机向上向量，默认y正向上
    int m_axesShaftType = vtkAxesActor::LINE_SHAFT;
    bool m_axesShowLabels = false;
    bool m_axesShowArrow = false;

    // debug窗口相关
    QList<QPointer<QDialog>> m_debugDialogs;                 // 存储所有debug窗口
    QList<QPointer<hgHMI_PCLViewerWidget>> m_debugViewers;   // 存储对应viewer
    QList<int> m_debugCloudCounts;                           // 每个窗口的点云计数

    void updateAxes();

	vtkSmartPointer<hgHMI_VTKCheck> m_interactorStyle; // 自定义 hgHMI_VTKCheck 交互样式
};

// ********** hgHMI_QPixelImage 控件（基于QGraphicsView的图像显示和绘制控件） **********
struct Image_Track
{
    QVector<QPointF> points;
    QVector<QGraphicsEllipseItem*> pointItems;
    QGraphicsPathItem* lineItem;

	bool isOut = false; // 标记轨迹是否为外轮廓
    int partIndex = -1;   // 属于哪个零件
    int outerIndex = -1;  // 属于哪个轮廓
    bool isClosed = false; // 是否闭合

    Image_Track() : lineItem(nullptr) {}
    // Image_Track析构函数，负责从scene安全移除然后删除所有图元
    ~Image_Track()
    {
        for (auto item : pointItems)
        {
            if (item && item->scene())
                item->scene()->removeItem(item);
            delete item;
        }
        if (lineItem)
        {
            if (lineItem->scene())
                lineItem->scene()->removeItem(lineItem);
            delete lineItem;
        }
    }
	// 转换为Eigen点序列
    std::vector<Eigen::Vector2d> toEigenPoints() const
    {
        std::vector<Eigen::Vector2d> eigenPoints;
        eigenPoints.reserve(points.size());
        for (const QPointF& pt : points)
        {
            eigenPoints.emplace_back(pt.x(), pt.y());
        }
        return eigenPoints;
    }
    void updateIsClosed(double tolerance = 0.8)
    {
        if (points.size() < 2)
        {
            isClosed = false;
            return;
        }
        isClosed = QLineF(points.first(), points.last()).length() <= tolerance;
    }
};

struct TrackHistoryData
{
    QVector<QPointF> points;
    int partIndex = -1;
    int outerIndex = -1;
    bool isOut = false;
};

class hgHMI_QPixelImage : public QGraphicsView
{
    Q_OBJECT
public:
    explicit hgHMI_QPixelImage(QWidget* parent = nullptr);

    // --- 图片设置与查询 ---
    void setImage(const QImage& image);             // 设置显示的图片
    QSize imageSize() const;                         // 获取当前显示图像尺寸

    // --- 光标相关 ---
    void setCursorRadius(qreal radius);              // 设置光标半径
    void previewCursorRadius(qreal radius);          // 预览光标半径

    // --- 轨迹绘制属性设置 ---
    void setLineWidthAndPointRadius(double lineWidth, double pointRadius); // 设置轨迹线宽和点半径并刷新
    void setTracksColor(const QColor& lineColor, const QColor& pointColor); // 设置所有轨迹线条和点颜色

    // --- 轨迹颜色修改 ---
    void setTrackLineColor(Image_Track* track, const QColor& color); // 通过轨迹指针修改轨迹线条颜色
    void setTrackLineColor(int index, const QColor& color);          // 通过轨迹索引修改轨迹线条颜色

    // --- 轨迹点插入 ---
    void insertPointAtNearestTrack(const QPointF& pos, double maxDistance); // 在最近轨迹插入点，pos为图像坐标，maxDistance为最大距离

    // --- 坐标转换 ---
    QPointF viewPosToImageCoords(const QPointF& viewPos) const;          // 视图坐标转图像坐标
	QVector<QPointF> scenePathToImagePoints(const QPainterPath& scenePath); // 场景路径转图像坐标点序列

    // --- 图形绘制 ---
    void clearDrawings();                 // 清除除图片外所有绘制图形
    void reset();                        // 重置状态并清除所有图形（轨迹、图片等）
	void setImageRotation(qreal angleDegrees);  // 设置图像旋转角度，单位度

    void drawPointMarker(const QPointF& pos);                               // 绘制点标记，图像坐标
    void drawOpenPolyline(const std::vector<Eigen::Vector2d>& points);      // 绘制非闭合折线，图像坐标点序列
    void drawClosedPolyline(const std::vector<Eigen::Vector2d>& points, int partIndex = -1, int outerIndex = -1, bool isOut = false, bool change = true); // 绘制闭合多边形

    void drawTexts(const std::vector<QString>& texts, const std::vector<Eigen::Vector2d>& positions); // 绘制文本，文本串及对应图像坐标

    // --- 模式开关 ---
    void setEraseMode(bool erase);           // 设置擦除模式
    bool isEraseMode() const { return m_isEraseMode; }    // 获取擦除模式状态

    void setPointMoveMode(bool enabled);     // 设置点移动模式
    bool isPointMoveMode() const { return m_isPointMoveMode; }  // 获取点移动模式状态

    void setWholeTrackEraseMode(bool enabled); // 设置整体轨迹擦除模式
    bool isWholeTrackEraseMode() const;         // 获取整体擦除模式状态

    void setLeftDragEnabled(bool enabled);    // 设置是否允许左键拖拽轨迹绘制
    bool isLeftDragEnabled() const { return m_leftDragEnabled; } // 获取左键拖拽状态

    // --- 轨迹管理 ---
    QVector<Image_Track*> getTracks() const { return m_tracks; }             // 获取当前所有轨迹
    void setTracks(const QVector<Image_Track*>& tracks) { m_tracks = tracks; } // 设置轨迹集合

    void setMinDistance(double Distance) { m_minDistance = Distance; }       // 设置轮廓点间最小距离（轮廓点距）
    double getMinDistance() const { return m_minDistance; }                  // 获取轮廓点距

    void setPointTolerance(double Distance) { pointDragTolerance = Distance; } // 设置点拾取阈值
    double getPointTolerance() const { return pointDragTolerance; }            // 获取点拾取阈值

    // --- 轨迹图元刷新与清理 ---
    void updateTrackGraphics(Image_Track* track);   // 重新绘制指定轨迹的图形
    void clearTrackGraphics(Image_Track* track);    // 清除指定轨迹的图形

    // --- 历史记录管理 ---
    void saveTracksToHistory();          // 保存当前轨迹状态到历史记录
    void clearHistory() { m_tracksHistory.clear(); }     // 清空历史记录
    void undo();                         // 撤销操作，恢复上一历史状态

signals:   
    void trackChanged(const QVector<Image_Track*>& tracks);
    void trackDeleted(Image_Track* track);

    // 鼠标左键对应信号
    void onDragPathKeyPointsFinished(const QVector<QPointF>& keyPoints);// 拖拽结束后——路径关键点数组（图像坐标系）  
    void onMousePositionValueChangedLeftPress(const QPointF& pt);// 鼠标左键按下——图像坐标
    void onMousePositionValueChangedLeftRelease(const QPointF& pt);// 鼠标左键释放——图像坐标
    void onMousePositionValueChangedLeftMove(const QPointF& delta);// 鼠标左键移动——图像坐标系差值

    // 鼠标右键对应信号
    void onMousePositionValueChangedRightPress(const QPointF& pt);
    void onMousePositionValueChangedRightRelease(const QPointF& pt);
    void onMousePositionValueChangedRightMove(const QPointF& delta);

    // 鼠标滚轮偏移量，正负整数，单位为滚轮步数
    void onWheelChanged(int delta);

protected:
    // 重写鼠标事件
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

    // 滚轮事件
    void wheelEvent(QWheelEvent* event) override;

    // 鼠标进入和离开事件
    void enterEvent(QEvent* event) override;
    void leaveEvent(QEvent* event) override;

private:
    // --- 图形场景与图元 ---
    QGraphicsScene* m_scene = nullptr;                 // 场景，管理图元
    QGraphicsPixmapItem* m_pixmapItem = nullptr;       // 显示图像的项
    QGraphicsPathItem* m_currentDragPathItem = nullptr;// 实时显示拖拽路径的图元

    // --- 鼠标状态和交互标志 ---
    bool m_isLeftPressed = false;       // 鼠标左键按下标志
    bool m_isRightPressed = false;      // 鼠标右键按下标志
    bool m_isDragging = false;          // 是否处于拖拽状态
    bool m_isRightDragging = false;     // 是否处于右键拖拽状态
    bool m_isPointDragging = false;     // 是否处于点拖拽状态
    bool m_mouseInside = false;         // 鼠标是否进入控件区域

    bool m_leftDragEnabled = false;     // 是否允许左键拖拽绘制轨迹
    bool m_isEraseMode = false;         // 擦除模式开关
    bool m_isPointMoveMode = false;     // 点移动模式开关
    bool m_isWholeTrackEraseMode = false; // 整体擦除模式开关

    // --- 拖拽相关 ---
    Image_Track* m_draggingTrack = nullptr; // 当前被拖拽点所属轨迹
    int m_draggingPointIndex = -1;           // 当前拖拽点索引
    QPointF m_lastPos;                       // 鼠标按下时图像坐标（用于拖拽起点）
    QPoint m_lastRightDragPos;               // 右键拖拽时上次鼠标位置（视图坐标）

    // --- 图像与绘制参数 ---
    QSize m_imageSize;           // 当前图像尺寸（像素）
	qreal angleDegrees = 0.0; 			 // 图像旋转角度，单位度
    double m_minDistance = 0.5;  // 拖拽路径最小轮廓点距（用于绘制）
    double maxClickDistance = 100.0; // 添加点的最大检索范围（用于点击添加点）
    double pointDragTolerance = 1; // 点拖拽距离阈值（图像坐标系单位）
    qreal m_cursorRadius = 15.0;     // 光标圆半径（默认）
    qreal m_cursorRadiusPressed = 10.0; // 鼠标按下时光标半径

    // --- 光标显示 ---
    QGraphicsEllipseItem* m_cursorCircle = nullptr;      // 跟随鼠标的圆形光标
    QGraphicsEllipseItem* m_previewCursorCircle = nullptr; // 预览光标圈（用于渐隐效果）
    QTimer* m_previewFadeTimer = nullptr;                 // 预览渐变定时器

    // 透明度控制
    qreal m_cursorPenAlpha = 1.0;      // 光标边缘透明度
    qreal m_cursorBrushAlpha = 0.196;  // 光标填充透明度（约50/255）
    qreal m_previewAlpha = 1.0;        // 预览光标透明度

    // --- 轨迹绘制参数 ---
    QColor outLineColor = Qt::red;      // 轨迹外轮廓颜色
    QColor outPointColor = Qt::blue;    // 轨迹点颜色
    double outLineWidth = 2;             // 轨迹线宽
    double outPointRadius = 5;           // 轨迹点半径

    // --- 轨迹数据结构 ---
    QPainterPath m_dragPath;             // 当前拖拽绘制的轨迹路径（区域选取）
    QVector<Image_Track*> m_tracks;      // 当前所有轨迹集合
    QStack<QVector<TrackHistoryData>> m_tracksHistory; // 轨迹历史栈，用于撤销

    // 根据图像坐标更新光标圆位置和大小，pressed表示鼠标是否按下
    void updateCursorCirclePosition(const QPointF& imagePos, bool pressed);

    // 从QPainterPath中提取关键点，用于轨迹简化或保存
    QVector<QPointF> extractKeyPointsFromPath(const QPainterPath& inputPath) const;

    // 擦除函数，擦除指定位置附近的轨迹点
    void erasePoints(const QPointF& pos);
};

// ********** hgHMI_PointCloud2DView 控件（点云XZ平面视图显示控件） **********
struct CustomPoint {
    QPointF pos;
    QColor color;
    int radius;
};

class hgHMI_PointCloud2DView : public QWidget {
    Q_OBJECT
public:
    explicit hgHMI_PointCloud2DView(QWidget* parent = nullptr);

    // 点云传入
    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, char axisX = 'x', char axisY = 'z');
    void setPointCloud(const pcl::PointCloud<pcl::PointXYZ>* cloud, char axisX = 'x', char axisY = 'z');

	// 自定义点传入
    void addCustomPoints(const QVector<QPointF>& pts, const QColor& color = Qt::green, int radius = 8);
    void clearCustomPoints();

	// 主题设置
    void setTheme(int themeId);

	// 边距设置
    void setMargins(double left, double bottom, double top, double right);// 整体到控件边距设置
    void getMargins(double& left, double& bottom, double& top, double& right) const;
    void setMarginPointCloud(double margin);// 点云到整体边距设置

	// 颜色设置
    void setBackgroundColor(const QColor& color);// 背景颜色设置
    void setAxisColor(const QColor& color);// 坐标轴颜色设置
    void setGridColor(const QColor& color);// 网格线颜色设置
    void setTickLabelColor(const QColor& color);// 刻度标签颜色设置
    void setAuxiliaryLineColor(const QColor& color);// 辅助线颜色设置

    // 点云设置
    void setPointColor(const QColor& color);// 普通点颜色和大小
    void setPointRadius(int radius);
    int pointRadius() const;
    void setHighlightColor(const QColor& color);// 高亮点颜色和大小
    void setHighlightRadius(int radius);
    int highlightRadius() const;

    // 刻度设置
    void setTickLabelFontSize(int fontSize);// 刻度标签字体大小
    void setTickLabelDecimals(int decimals);// 设置/获取 刻度标签显示的小数位数
    void setNumMainTicks(int count);// 主刻度数设置
    void setNumSubTicks(int count);// 副刻度分割数设置

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    QVector<CustomPoint> customPoints;
    
    QColor backgroundColor = Qt::white;// 背景色
    QColor axisColor = Qt::black;// 坐标轴颜色
    QColor gridColor = Qt::gray;// 网格线颜色
    QColor tickLabelColor = Qt::black;// 刻度标签颜色
    QColor auxiliaryLineColor = Qt::red;// 辅助线颜色（选中点虚线）

    QVector<QPointF> points;         // 点云XZ投影点
    double minX, maxX, minZ, maxZ;  // 点云范围（X,Z）
    double scale;                   // 缩放因子
    QPointF centerScene;            // 视图中心（场景坐标）

    int selectedPointIndex = -1;     // 当前选中点索引，-1表示无选中

    // 拖动相关
    bool dragging = false;
    QPoint lastMousePos;

    // 边距
    double marginLeft = 90;
    double marginBottom = 40;
    double marginTop = 20;
    double marginRight = 40;
    double marginPointCloud = 0.01;

    // 点颜色及大小
    QColor normalPointColor = Qt::blue;
    int normalPointRadius = 3;
    QColor highlightPointColor = Qt::red;
    int highlightPointRadius = 5;

    // 刻度标签字体大小
    int tickFontSize = 8;
    int tickLabelDecimals = 2; // 刻度标签保留小数位数
    int numMainTicks = 10;       // 主刻度数
    int numSubTicks = 4;        // 副刻度分割数

    // 坐标转换
    QPointF mapToWidget(double x, double z) const;
    QPointF mapToScene(const QPoint& widgetPt) const;

    // 包围盒计算
    void updateBoundingRect();

    // 绘制
    void drawGrid(QPainter& painter);
    void drawAxes(QPainter& painter);
    void drawPoints(QPainter& painter);
};


 


#endif // HGHMI_H
