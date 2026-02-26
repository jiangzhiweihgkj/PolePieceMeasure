#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_PolePieceMeasureWin.h"
#include "CallOneTimes.h"
#include "CallOneTimes_B.h"
#include <qmutex.h>
#include <QTextStream>
#include <QSettings>
#include <algorithm>
#include <numeric>
#include <QtCharts/qsplineseries.h>
#include <QtSerialPort/qserialport.h>
#include <QtCharts/QChartView >
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <cmath>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkRenderer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkSmartPointer.h>  // VTK的智能指针
#include <vtkRenderWindow.h>  // 渲染窗口类
#include <vtkAxesActor.h>  // VTK的坐标轴显示类
//表格显示
#include <QtCharts/QLineSeries>
#include <QtCharts/QChart>
#include <QtCharts/QValueAxis>
#include <QScatterSeries>
#include <QMouseEvent>
#include <QDebug>
#include <QRandomGenerator>
//登录窗口
#include "subwindow.h"
#include "ui_subwindow.h"
#include <QCryptographicHash>
#include "login.h"
#include "ui_login.h"
 #include "Communicate.h"
#include "ui_Communicate.h"
//折线图显示窗口
#include "ZoomChartView.h"
#include <QtCharts/QLegend>
#include <QVector>
#include <QPointF>
#include <QGraphicsTextItem>
//数学计算
#include <QtMath>

#include <QFile>
#include "DataExcel.h"
#include "ui_DataExcel.h"

  QT_CHARTS_USE_NAMESPACE
 // mainwindow.h
QT_BEGIN_NAMESPACE
namespace Ui { class PolePieceMeasureWinClass; };
QT_END_NAMESPACE

class PolePieceMeasureWin : public QMainWindow
{
    Q_OBJECT

public:
    PolePieceMeasureWin(QWidget *parent = nullptr);
    ~PolePieceMeasureWin();
    void closeEvent(QCloseEvent* event);
    void  exportToCSV();//将表格数据导出为CSV文件

    void  InitScanTab();
    void  DestroyScanTab();

    QString iniPath = QStringLiteral("./DepthMeasureSetting.ini");
    std::vector<int> ID_SELECT;
    int num_measure_A = 0;   
    QVector<QPointF> depth_data1;
    QVector<QPointF> depth_data1_min;
    QVector<QPointF> depth_data1_max;
    QVector<QPointF> cpk_depth_data1;
    QVector<QPointF> width_data1;
    QVector<QPointF> width_data1_min;
    QVector<QPointF> width_data1_max;
    QVector<QPointF> cpk_width_data1;
     // 添加自定义点，颜色绿色，半径8
    QVector<QPointF> customPts;
    QVector<QPointF> customPts_bottom;

     bool RightMostIndex = false;//右边点
     bool LeftMostIndex = false;//左边点
 public:
    CallOneTimes *_callOneTimes = nullptr;
    CallOneTimes_B *_callOneTimes_B = nullptr;
    void   updateRecord(QString _str);//显示日志
    void  RecordData(QString _str, int level=1);

    void   insertAtTop(QTextEdit* textEdit, const QString& text);//将文本插入到顶部
    void   CreateWorkDir(QString mainDir);//创建工作目录
    void deleteOldFolders(const QString& basePath, int daysToKeep = 7);//删掉一星期以前的文件夹

    QString mainWorkDir = QStringLiteral("");
    QString pointDirName = QStringLiteral("pointCloud");
    void CalculateStatistics();//计算CPK值
    struct
    {
        double meanWidth;
        double meanDepth;
        double standardDeviationDepth;//标准差
        double standardDeviationWidth;//标准差
        double cp_Depth_up;     
        double cp_Depth_down;//// 过程能力指数;
        double cp_Width_up;
        double cp_Width_down;//// 过程能力指数;
        double Cpk_Width;
        double Cpk_Depth;

    }SelectedData;//计算cpk值
     
 private Q_SLOTS :
     void  onReceiveStatus(QString);
     void  onslotReceiveCloud();
 private Q_SLOTS :
        void on_btnConnectCamera_clicked();//连接轮廓仪A
        void on_btnDisConnectCamera_clicked();//断开轮廓仪A
        void on_btnConnectCamera_B_clicked();//连接轮廓仪B
        void on_btnDisConnectCamera_B_clicked();//断开轮廓仪B
        void on_btnStartCollectPoint_clicked();//开始采集
        void on_btnStopCollectPoint_clicked();//开始采集
        void on_btnSavePoint_clicked();//开始采集
        void on_btnOutPutDirectory_clicked();//浏览
        void on_btnOutPutDirectory_Result_clicked();//浏览
        void on_btnSignIn_clicked();//登录
        void on_btnCameraA_clicked();//处理 
        void on_btnCameraB_clicked();//处理 
        void on_btnClearRecord_clicked();//清理日志
        //浏览导出表格的路径包含A B
        void on_btnBrowse_A_clicked();//浏览A
        void on_btnBrowse_B_clicked();//浏览B
        void on_btnInput_B_clicked();//导出B
        void on_btnInput_A_clicked();//导出A        

#pragma region 点云识别  PYF 写目前不用了
public:
    struct RecognizeResult {
        std::vector<double> width;
        std::vector<double> depth;
        double minWidth; double minDepth;
        double maxWidth; double maxDepth;
        double meanWidth; double meanDepth;
    }InfoResult;
    pcl::PointCloud<pcl::PointXYZ>::Ptr slidingAverageZ(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int half_w);
      int  Single_Recognition(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
      void  Multify_Recognition(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud);
      int  InfoResult_Calculate();
      double calculateDistance(const pcl::PointXYZRGB& pt_left, const pcl::PointXYZRGB& pt_right);
      void  updateViewer(
          pcl::PointCloud<pcl::PointXYZ>::Ptr show_source_cloud = nullptr,
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_left = nullptr,
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_right = nullptr,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_down_Points=nullptr);
      void  updateTable();//更新表格显示
      void  new_A_updateTable();//更新表格显示
      bool  Reco_Protection(pcl::PointXYZRGB pt_left, pcl::PointXYZRGB pt_right, pcl::PointXYZRGB pt_down);
#pragma endregion

#pragma region 点云显示
public:
    pcl::ModelCoefficients::Ptr coefficients; // 单帧的平面模型
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> separate_clouds; // 分离后的点云集
    std::vector<int> ID;  // 用来存储识别帧的索引
    std::vector<int> ID_NEW;  // 用来存储识别帧的索引
   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr up_Points_left;      // 左上特征点集
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr up_Points_right;     // 右上特征点集
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr down_Points;         // 底部特征点集
    void  roi_Setting(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void  showPointCloud(pcl::PointCloud<pcl::PointXYZ>& input);
    void  initUIViewer();
    bool actionViewer = false;
    vtkSmartPointer<vtkRenderer> GlobalMyViewRendererXia;
    vtkSmartPointer<vtkRenderWindow> GlobalMyViewRenderWindowXia;
    void  pointsClear();//清空点云
private Q_SLOTS:
    void on_btnLoadCloud_clicked();//导入点云
    void on_btnRecoCloud_clicked();//识别点云
    void on_btnebug_clicked();//debug
    void on_btnSetting_clicked();//设置
    void on_btnCommunication_clicked();//通信窗口
#pragma endregion
private Q_SLOTS:
    void  slotTableSelectionChanged();//表格显示和事件响应
    void  new_slotTableSelectionChanged();//表格显示和事件响应
    void  new_B_slotTableSelectionChanged();//表格显示和事件响应
#pragma region 点云处理5
    public:
        struct sandian_suoyin
        {
            std::vector<double> left_index;
            std::vector<double> right_index;
            std::vector<double> down_index;
        }sandian_suoyin;

        struct Calucate_Width_Depth
        {
          std::vector<double> m_width;
          std::vector<double> m_depth;
          double m_max_width;
          double m_min_width;
          double m_mean_width;
          
          double m_max_depth;
          double m_min_depth;
          double m_mean_depth;
        }Calucate_Width_Depth;

        struct ThreePoint2RGBcloud
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGB_left_Cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGB_right_Cloud;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGB_down_Cloud;
        };
        struct ThreePoint
        {
             pcl::PointXYZ  leftShoulder;
             pcl::PointXYZ  rightShoulder;
             pcl::PointXYZ  downShoulder;
        };
       pcl::PointCloud<pcl::PointXYZ>::Ptr roiAcquire(pcl::PointCloud<pcl::PointXYZ>::Ptr  sourcecloud, double roiPercent, double centerPercent);
       std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr> NineJie(pcl::PointCloud<pcl::PointXYZ>::Ptr  roiCloud);
       ThreePoint YIJie(  pcl::PointCloud<pcl::PointXYZ>::Ptr  yijieCloud, int pinghuaxishu);
       PolePieceMeasureWin::ThreePoint singleRecongize(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  threeJieCloud, int pinghuaxishu); 
       bool multifyRecognize(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  multifyCloud,int pinghuaxishu=10);
       ThreePoint2RGBcloud  ThreePointToRGBcloud(std::vector<ThreePoint> _vectorThreePoint);
       void ShowThreeVectorPoint(ThreePoint2RGBcloud _ThreePoint2RGBcloud);
       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> separateCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr sourcedCloud);
       bool StartRecongize(pcl::PointCloud<pcl::PointXYZ>::Ptr source_Cloud);
       void Init_Clear();
       void Result_A_Show();//结果显示
       PolePieceMeasureWin::ThreePoint2RGBcloud _ThreePoint2RGBcloud;
       std::vector<PolePieceMeasureWin::ThreePoint> _multifyRecognize;
       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _separateCloud;
       //直线拟合近似主方向 RANSC方法
       pcl::PointCloud<pcl::PointXYZ>::Ptr nihe_Roate( pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
       pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud;// (new pcl::PointCloud<pcl::PointXYZ>);
       double roiPercent = 0.0;
       // 中心位置百分比 centerPercent
       double centerPercent = 0.0;
#pragma endregion


#pragma region PLC
 public:

 private Q_SLOTS:
     void on_btnDisConnectPLC_clicked();//PLC 断开
     void on_btnConnectPLC_clicked();//PLC 连接
     void on_btnWriteReister_clicked();//写入值
     void on_btnReadReister_clicked();//读取值
     void slotReceiveOuMoRon(QVector<quint16> val);
#pragma endregion

#pragma region 数据图形化显示
 public:
       void Show_CurrentAB_Table_Show();
       void tableAB_Show();
       void InitLineChartShow();
       void setHeaderText(QGraphicsTextItem* m_headerText, const QStringList& lines);//左上角显示平均值，标准差和Cpk
       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> showResultPoint;
       std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> showResultPoint_B;
       std::vector<PolePieceMeasureWin::ThreePoint>    show3Points;
       QValueAxis * m_axisX = nullptr;
       QValueAxis * m_axisY = nullptr;
       QLineSeries* series_A_Depth_Mean = nullptr;
       QLineSeries* series_A_Depth_Min = nullptr;
       QLineSeries* series_A_Depth_Max = nullptr;
       QLineSeries* series_A_Depth_Signma = nullptr;
       QLineSeries* series_A_Width_Mean = nullptr;
       QLineSeries* series_A_Width_Min = nullptr;
       QLineSeries* series_A_Width_Max = nullptr;
       QLineSeries* series_A_Width_Signma = nullptr;
       QLineSeries* series_B_Depth_Mean = nullptr;
       QLineSeries* series_B_Depth_Min = nullptr;
       QLineSeries* series_B_Depth_Max = nullptr;
       QLineSeries* series_B_Depth_Signma = nullptr;
       QLineSeries* series_B_Width_Mean = nullptr;
       QLineSeries* series_B_Width_Min = nullptr;
       QLineSeries* series_B_Width_Max = nullptr;
       QLineSeries* series_B_Width_Signma = nullptr; 
       QLineSeries* upperSeries = nullptr;//上限
       QLineSeries* lowerSeries = nullptr;//下限
 private Q_SLOTS:
     void on_btnLoadPointsToExcel_clicked();//读取值
     void on_btnTestShow_clicked();//测试 
   
#pragma endregion



#pragma region 折线图形化显示
 public:
     void show_A_Depth_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> depth_min, QVector<QPointF> depth_max);//show_A_Depth_Line_Chart
     void show_A_Width_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> width_min, QVector<QPointF> width_max);//show_A_Width_Line_Chart
     void show_B_Depth_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> depth_min, QVector<QPointF> depth_max);//show_B_Depth_Line_Chart
     void show_B_Width_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> width_min, QVector<QPointF> width_max);//show_B_Width_Line_Chart
      QVector<QPointF> cloudToXZ(const pcl::PointCloud<pcl::PointXYZ>& cloud,double scaleValue=1);//将点云转换成QVector<QPointF>类型，便于图表显示

 private Q_SLOTS:
      void on_btnLength_A_Picture_clicked();
      void on_btnShort_A_Picture_clicked();
#pragma endregion
private:
    Ui::PolePieceMeasureWinClass *ui;
    Ui::SubWindowClass* ParamUi; 
    Ui::DataExcelClass* DataExcelUi;//表格ui
    QDialog* ParametersDialog;
    QDialog* excelDialog;//表格
    ZoomChartView* m_chartView_A_Depth;   // 用来放图
    ZoomChartView* m_chartView_A_Width;   // 用来放图
    ZoomChartView* m_chartView_B_Depth;   // 用来放图
    ZoomChartView* m_chartView_B_Width;   // 用来放图
    pcl::visualization::PCLVisualizer::Ptr viewer;//定义显示
    pcl::visualization::PCLVisualizer::Ptr viewer_B;//定义显示



//d登录
    public:
        //登录
       // void onOpenLoginWindow();
        QString currentPassword;
        QString hashPassword(const QString& password);
        void onLogin();
        void onLogout();


//参数信息 相机的参数信息和PLC的参数信息
#pragma region 参数设置 
 public:
      struct {
         QString Camra_A_IP;//相机IP
         QString Camra_B_IP;//相机IP
         QString PointOutPath;  //点云输出路径
         QString PointOutPath_B;  //点云B输出路径
         QString CameraConnectState_A;  //相机连接状态
         QString CameraConnectState_B;  //相机连接状态
     }InfoCamera;
     struct {
         QString PLC_IP;//PLC IP地址
         QString PLC_PORT;//PLC 端口号 
         QString PLCConnectState;  //PLC连接状态 
     }InfoPLC; 
     struct {
         QString editRoiPercent;//ROI相对宽度(%)：
         QString editCentrePoint;//ROI焦点位置(%)：
         QString editPingHuaXiShu;//点云平滑系数
         QString editGaoDuChaThreshold;//高度差阈值(mm)
         QString editCloudShowDis;//点云间距(mm)

         QString editDetectedDepthMin;//深度阈值Min(um)
         QString editDetectedDepthMax;//深度阈值Max(um)
         QString editDetectedWidthMin;//宽度阈值Min(um)
         QString editDetectedWidthMax;//宽度阈值Max(um)

         QString editMaxWidth;//最小宽度阈值(mm)：
         QString editRANSAC;//距离阈值(mm)：
         QString editAngle;//识别角度(°)：
         QString editEdge;//底点位置阈值(%)：
         QString editCheckThreshold;//底点位置阈值(%)：
         
         QString edit_Depth_A_Coefficient;//A面槽深系数
         QString edit_Width_A_Coefficient;//A面槽宽系数
         QString edit_Depth_B_Coefficient;//B面槽深系数
         QString edit_Width_B_Coefficient;//B面槽宽系数 
     }InfoSuanFa;
     struct {
         QString excel_A_OutPath;  //excel导出输出路径
         QString excel_B_OutPath;  //excel导出输出路径

     }InfoExcel;
     void iniSettingParam();//加载设置参数
     void initIniSettingParam();//加载设置参数
     void destroySettingParam();//保存设置参数
#pragma endregion

     //B面检测头函数和全局变量
#pragma region 点云处理 
     public:
         bool RightMostIndex_B = false;//最右边点
         bool LeftMostIndex_B = false;//最左边点 
         pcl::PointCloud<pcl::PointXYZ>::Ptr slidingAverageZ_B(
             const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int half_w);
         pcl::ModelCoefficients::Ptr coefficients_B; // 单帧的平面模型
         pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_B;//B面处理的原始点云
         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> separate_clouds_B; // 原始ROI分离后的点云集
          ThreePoint YIJie_B(pcl::PointCloud<pcl::PointXYZ>::Ptr  yijieCloud, int pinghuaxishu);//单独处理三节分成一节的情况
         PolePieceMeasureWin::ThreePoint singleRecongize_B(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  threeJieCloud, int pinghuaxishu);//逐帧处理
         bool multifyRecognize_B(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr>  multifyCloud, int pinghuaxishu = 10);//多帧处理
         bool StartRecongize_B(pcl::PointCloud<pcl::PointXYZ>::Ptr source_Cloud);//开始识别
         void Init_Clear_B();
         std::vector<PolePieceMeasureWin::ThreePoint> _multifyRecognize_B;
         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _separateCloud_B;
         pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud_B;// (new pcl::PointCloud<pcl::PointXYZ>);
         double roiPercent_B = 0.0;
         double centerPercent_B = 0.0;

private Q_SLOTS:
    void  onReceiveStatus_B(QString);
    void  onslotReceiveCloud_B();
    void on_btnStartCollectPoint_B_clicked();//开始采集
    void on_btnStopCollectPoint_B_clicked();//停止采集
#pragma endregion

#pragma region 点云处理结果
     public:
         struct RecognizeResult_B {
             std::vector<double> width;
             std::vector<double> depth;
             double minWidth; double minDepth;
             double maxWidth; double maxDepth;
             double meanWidth; double meanDepth;
         }InfoResult_B;
         struct sandian_suoyin_B
         {
             std::vector<double> left_index;
             std::vector<double> right_index;
             std::vector<double> down_index;
         }sandian_suoyin_B;

         struct Calucate_Width_Depth_B
         {
             std::vector<double> m_width;
             std::vector<double> m_depth;
             double m_max_width;
             double m_min_width;
             double m_mean_width;

             double m_max_depth;
             double m_min_depth;
             double m_mean_depth;
         }Calucate_Width_Depth_B;
private Q_SLOTS:
#pragma endregion

#pragma region 点云显示
     public:
         std::vector<int> ID_NEW_B;  // 用来存储识别帧的索引
         struct ThreePoint2RGBcloud_B
         {
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGB_left_Cloud;
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGB_right_Cloud;
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGB_down_Cloud;
         }ThreePoint2RGBcloud_B;
         struct ThreePoint_B
         {
             pcl::PointXYZ  leftShoulder;
             pcl::PointXYZ  rightShoulder;
             pcl::PointXYZ  downShoulder;
         }ThreePoint_B;
         ThreePoint2RGBcloud  ThreePointToRGBcloud_B(std::vector<ThreePoint> _vectorThreePoint);
         void ShowThreeVectorPoint_B(ThreePoint2RGBcloud _ThreePoint2RGBcloud);

         PolePieceMeasureWin::ThreePoint2RGBcloud _ThreePoint2RGBcloud_B;

private Q_SLOTS:
#pragma endregion

#pragma region 控件结果显示
     public:

private Q_SLOTS:
#pragma endregion

#pragma region 图表显示
     public:
         std::vector<int> ID_SELECT_B;//显示索引
         int num_measure_B = 0;//显示OK NG数量
         //显示折线图上面的点
         QVector<QPointF> depth_data1_B;
         QVector<QPointF> depth_data1_B_min;
         QVector<QPointF> depth_data1_B_max;
         QVector<QPointF> cpk_depth_data1_B;
         QVector<QPointF> width_data1_B;
         QVector<QPointF> width_data1_B_min;
         QVector<QPointF> width_data1_B_max;
         QVector<QPointF> cpk_width_data1_B;
         //显示单帧折线图时的肩点和底部点
         QVector<QPointF> customPts_B;
         QVector<QPointF> customPts_bottom_B;
         struct
         {
             double meanWidth;
             double meanDepth;
             double standardDeviationDepth;//标准差
             double standardDeviationWidth;//标准差
             double cp_Depth_up;
             double cp_Depth_down;//// 过程能力指数;
             double cp_Width_up;
             double cp_Width_down;//// 过程能力指数;
             double Cpk_Width;
             double Cpk_Depth;

         }SelectedData_B;//计算cpk值
         void  updateViewer_B(
             pcl::PointCloud<pcl::PointXYZ>::Ptr show_source_cloud = nullptr,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_left = nullptr,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_right = nullptr,
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_down_Points = nullptr);
         void  new_B_updateTable();//更新表格显示
         void Result_B_Show();//结果显示
         void Show_CurrentAB_Table_Show_B();

private Q_SLOTS:
#pragma endregion

#pragma region 按钮功能执行
     public:

private Q_SLOTS:
#pragma endregion





 };

