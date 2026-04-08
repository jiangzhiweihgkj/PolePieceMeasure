#include "PolePieceMeasureWin.h"
 void PolePieceMeasureWinOutputMessage(QtMsgType type, const QMessageLogContext& context,
    const QString& msg)
{
    static QMutex mutex;
    mutex.lock();

    QString text;
    switch (type) {
    case QtInfoMsg:
        text = QString("Info:");
        break;
        if (type!= QtInfoMsg)
        {
    default:
        break;
        }
    /*case QtDebugMsg:
        text = QString("Debug:");
        break;
    case QtWarningMsg:
        text = QString("Warning:");
        break;
    case QtCriticalMsg:
        text = QString("Critical:");
        break;
    case QtFatalMsg:
        text = QString("Fatal:");
        break;*/
   
    }
    // QString context_info = context.line;
    QString current_date_time = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm ddd");
    QString current_date = QString("(%1)").arg(current_date_time);
    QString message = QString("Line:(%1) %2 %3 %4").arg(context.line).arg(current_date).arg(text).arg(msg);

    QDateTime date = QDateTime::currentDateTime();
    QString LogPath = QCoreApplication::applicationDirPath();//在绝对路径下创建
    LogPath.append(QStringLiteral("/log"));
    QDir dir(LogPath);
    if (!dir.exists(LogPath)) {
        dir.mkdir(LogPath);//创建文件夹
    }
    QString strWorkNames = QStringLiteral("Log") + date.toString(QStringLiteral("yyyyMMdd"));
    QString logFile = LogPath + QStringLiteral("/") + strWorkNames;
    logFile.append(QStringLiteral(".log"));
    QFile file(logFile);

    file.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream text_stream(&file);
    text_stream << message << "\r\n";
    file.flush();
    file.close();
    mutex.unlock();
}


PolePieceMeasureWin::PolePieceMeasureWin(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::PolePieceMeasureWinClass())
{
    ui->setupUi(this);
    InitScanTab(); initIniSettingParam(); iniSettingParam(); 
    {
      QString fileName1 = QCoreApplication::applicationDirPath() + "/icon/ctrl.png";
      ui->btnCameraA->setIcon(QIcon(fileName1));   // 注意冒号+前缀
      ui->btnCameraA->setIconSize(QSize(30, 40));

      QString fileName2 = QCoreApplication::applicationDirPath() + "/icon/ctrl.png";
      ui->btnCameraB->setIcon(QIcon(fileName2));   // 注意冒号+前缀
      ui->btnCameraB->setIconSize(QSize(30, 40));

      QString fileName3 = QCoreApplication::applicationDirPath() + "/icon/communicate.png";
      ui->btnCommunication->setIcon(QIcon(fileName3));   // 注意冒号+前缀
      ui->btnCommunication->setIconSize(QSize(30, 40));
      
      QString fileName4 = QCoreApplication::applicationDirPath() + "/icon/paramSet.png";
      ui->btnSetting->setIcon(QIcon(fileName4));   // 注意冒号+前缀
      ui->btnSetting->setIconSize(QSize(30, 40));
      
      QString fileName5 = QCoreApplication::applicationDirPath() + "/icon/start.png";
      ui->btnStartCollectPoint->setIcon(QIcon(fileName5));   // 注意冒号+前缀
      ui->btnStartCollectPoint->setIconSize(QSize(30, 40));
      
      QString fileName6 = QCoreApplication::applicationDirPath() + "/icon/Final.png";
      ui->btnSavePoint->setIcon(QIcon(fileName6));   // 注意冒号+前缀
      ui->btnSavePoint->setIconSize(QSize(30, 40));

      QString fileName7 = QCoreApplication::applicationDirPath() + "/icon/stop.png";
      ui->btnStopCollectPoint->setIcon(QIcon(fileName7));   // 注意冒号+前缀
      ui->btnStopCollectPoint->setIconSize(QSize(30, 40));

      QString fileName8 = QCoreApplication::applicationDirPath() + "/icon/SignIn.png";
      ui->btnSignIn->setIcon(QIcon(fileName8));   // 注意冒号+前缀
      ui->btnSignIn->setIconSize(QSize(30, 40));

      QString fileName9 = QCoreApplication::applicationDirPath() + "/icon/LoadIn.png";
      ui->btnLoadCloud->setIcon(QIcon(fileName9));   // 注意冒号+前缀
      ui->btnLoadCloud->setIconSize(QSize(30, 40));
      
      QString fileName11 = QCoreApplication::applicationDirPath() + "/icon/LoadIn.png";
      ui->btnRecoCloud->setIcon(QIcon(fileName11));   // 注意冒号+前缀
      ui->btnRecoCloud->setIconSize(QSize(30, 40));
      
      QString fileName12= QCoreApplication::applicationDirPath() + "/icon/debug.png";
      ui->btnebug->setIcon(QIcon(fileName12));   // 注意冒号+前缀
      ui->btnebug->setIconSize(QSize(30, 40));
      
      QString fileName13 = QCoreApplication::applicationDirPath() + "/icon/debug.png";
      ui->btnTestShow->setIcon(QIcon(fileName13));   // 注意冒号+前缀
      ui->btnTestShow->setIconSize(QSize(30, 40));
      
       
    }
    // 让索引为 0 的 tab 不可见
    ui->tabWidgetDataShow->removeTab(3);        // 彻底拿掉
    ui->tabWidgetDataShow->removeTab(2);        // 彻底拿掉
    ui->tabWidgetDataShow->removeTab(1);        // 彻底拿掉

    depth_data1.reserve(65535);
    depth_data1_min.reserve(65535);
    depth_data1_max.reserve(65535);
    cpk_depth_data1.reserve(65535);
    width_data1.reserve(65535);
    width_data1_min.reserve(65535);
    width_data1_max.reserve(65535);
    cpk_width_data1.reserve(65535); 
    
    depth_data1_B.reserve(65535);
    depth_data1_B_min.reserve(65535);
    depth_data1_B_max.reserve(65535);
    cpk_depth_data1_B.reserve(65535);
    width_data1_B.reserve(65535);
    width_data1_B_min.reserve(65535);
    width_data1_B_max.reserve(65535);
    cpk_width_data1_B.reserve(65535); 

    setWindowTitle(QStringLiteral("HG Farely Shape3D v1.0"));
    qInstallMessageHandler(PolePieceMeasureWinOutputMessage);//日志 
     ui->tableAB_Show_Data->setRowCount(2);//3行Calucate_Width_Depth.m_mean_width
     ui->tableAB_Show_Data->setColumnCount(10);//3行Calucate_Width_Depth.m_mean_width
     int rows = ui->tableAB_Show_Data->rowCount();
     int tableH = ui->tableAB_Show_Data->viewport()->height();   // 可画区域高度
     ui->tableAB_Show_Data->verticalHeader()->setDefaultSectionSize(tableH / rows);
    //表格属性设置 
    ui->tableAB_Show_Data->resizeRowsToContents();   // 先自动算一次
    for (int i=0;i<3;i++)
    {
        ui->tableAB_Show_Data->setColumnWidth(i, 60);
    }
    ui->tableAB_Show_Data->setColumnWidth(3, 70);

    for (int i=4;i<10;i++)
    {
        ui->tableAB_Show_Data->setColumnWidth(i, 100);
    }
    // 在构造函数里 
    for (int i = 0; i < 10; i++)
    {
       // ui->tableAB_Show_Data->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Stretch);
    }
    ui->tableAB_Show_Data->verticalHeader()->setVisible(true); // 显示垂直表头
    ui->tableAB_Show_Data->setEditTriggers(QAbstractItemView::NoEditTriggers); // 设置表格不可编辑

    ui->tableFrame->setAlternatingRowColors(false); // 设置交替行颜色
    ui->tableFrame->setSelectionBehavior(QAbstractItemView::SelectRows); // 设置选择行为为按行选择
    ui->tableFrame->setColumnWidth(0, 80);
    ui->tableFrame->setColumnWidth(3, 100);
    ui->tableFrame->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->tableFrame->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
    ui->tableFrame->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->tableFrame->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch); // 设置列宽自适应
    ui->tableFrame->verticalHeader()->setVisible(false); // 隐藏垂直表头
    ui->tableFrame->setEditTriggers(QAbstractItemView::NoEditTriggers);
    
    ui->tableFrame_B->setAlternatingRowColors(false); // 设置交替行颜色
    ui->tableFrame_B->setSelectionBehavior(QAbstractItemView::SelectRows); // 设置选择行为为按行选择
    ui->tableFrame_B->setColumnWidth(0, 80);
    ui->tableFrame_B->setColumnWidth(3, 100);
    ui->tableFrame_B->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->tableFrame_B->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
    ui->tableFrame_B->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->tableFrame_B->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch); // 设置列宽自适应
    ui->tableFrame_B->verticalHeader()->setVisible(false); // 隐藏垂直表头

    ui->tb_Widget_A->setRowCount(3);
    ui->tb_Widget_A->setColumnCount(4);
    ui->tb_Widget_A->horizontalHeader()->setVisible(false);
    ui->tb_Widget_A->verticalHeader()->setVisible(false); // 隐藏垂直表头

    ui->tb_Widget_A->setColumnWidth(0, 80);
    ui->tb_Widget_A->setColumnWidth(3, 100);
   
    ui->tb_Widget_A->resizeRowsToContents();
   // ui->tb_Widget_A->setRowHeight(0, 40);
   // ui->tb_Widget_A->setRowHeight(1, 40);
   // ui->tb_Widget_A->setRowHeight(2, 40);
    ui->tb_Widget_A->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->tb_Widget_A->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
    ui->tb_Widget_A->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->tb_Widget_A->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch); // 设置列宽自适应
    ui->tb_Widget_A->setEditTriggers(QAbstractItemView::NoEditTriggers);
    
    ui->tb_Widget_B->setRowCount(3);
    ui->tb_Widget_B->setColumnCount(4);
    ui->tb_Widget_B->horizontalHeader()->setVisible(false);
    ui->tb_Widget_B->verticalHeader()->setVisible(false); // 隐藏垂直表头

    ui->tb_Widget_B->setColumnWidth(0, 80);
    ui->tb_Widget_B->setColumnWidth(3, 100);
    ui->tb_Widget_B->resizeRowsToContents();
   // ui->tb_Widget_B->setRowHeight(0, 40);
   // ui->tb_Widget_B->setRowHeight(1, 40);
   // ui->tb_Widget_B->setRowHeight(2, 40);
    ui->tb_Widget_B->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);
    ui->tb_Widget_B->horizontalHeader()->setSectionResizeMode(3, QHeaderView::Fixed);
    ui->tb_Widget_B->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
    ui->tb_Widget_B->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Stretch); // 设置列宽自适应
    ui->tb_Widget_B->setEditTriggers(QAbstractItemView::NoEditTriggers);
    


    // 设置暗色背景
    ui->Single_A_PointShow->setBackgroundColor(QColor(30, 30, 30));         // 深色背景
    ui->Single_A_PointShow->setAxisColor(Qt::white);                        // 坐标轴白色
    ui->Single_A_PointShow->setGridColor(QColor(100, 100, 100));            // 网格线暗灰色
    ui->Single_A_PointShow->setTickLabelColor(Qt::white);                   // 刻度标签白色
    ui->Single_A_PointShow->setAuxiliaryLineColor(QColor(255, 128, 0));     // 辅助线橙色
    ui->Single_A_PointShow->setPointColor(Qt::cyan);                        // 点云颜色青色
    ui->Single_A_PointShow->setHighlightColor(Qt::yellow);                  // 选中点黄色

    // 设置参数
    ui->Single_A_PointShow->setMargins(30, 25, 20, 5);           // 整体边距
    ui->Single_A_PointShow->setPointRadius(2);                     // 普通点大小稍大
    ui->Single_A_PointShow->setHighlightRadius(4);                 // 高亮点大小
    ui->Single_A_PointShow->setTickLabelFontSize(6);               // 刻度标签字体大小
    ui->Single_A_PointShow->setTickLabelDecimals(3);               // 保留1位小数
    ui->Single_A_PointShow->setNumMainTicks(10);                    // 主刻度数
    ui->Single_A_PointShow->setNumSubTicks(4);                      // 副刻度数
    ui->Single_A_PointShow->setMarginPointCloud(0.01);              // 点云边距留白比例 1% 
    
    // 设置暗色背景
    ui->Single_B_PointShow->setBackgroundColor(QColor(30, 30, 30));         // 深色背景
    ui->Single_B_PointShow->setAxisColor(Qt::white);                        // 坐标轴白色
    ui->Single_B_PointShow->setGridColor(QColor(100, 100, 100));            // 网格线暗灰色
    ui->Single_B_PointShow->setTickLabelColor(Qt::white);                   // 刻度标签白色
    ui->Single_B_PointShow->setAuxiliaryLineColor(QColor(255, 128, 0));     // 辅助线橙色
    ui->Single_B_PointShow->setPointColor(Qt::cyan);                        // 点云颜色青色
    ui->Single_B_PointShow->setHighlightColor(Qt::yellow);                  // 选中点黄色

    // 设置参数
    ui->Single_B_PointShow->setMargins(30, 25, 20, 5);           // 整体边距
    ui->Single_B_PointShow->setPointRadius(2);                     // 普通点大小稍大
    ui->Single_B_PointShow->setHighlightRadius(4);                 // 高亮点大小
    ui->Single_B_PointShow->setTickLabelFontSize(6);               // 刻度标签字体大小
    ui->Single_B_PointShow->setTickLabelDecimals(3);               // 保留1位小数
    ui->Single_B_PointShow->setNumMainTicks(10);                    // 主刻度数
    ui->Single_B_PointShow->setNumSubTicks(4);                      // 副刻度数
    ui->Single_B_PointShow->setMarginPointCloud(0.01);              // 点云边距留白比例 1%
     // 设置表格不可编辑
    //connect(ui->tableFrame, &QTableWidget::itemSelectionChanged, this, &PolePieceMeasureWin::slotTableSelectionChanged);
    connect(ui->tableFrame, &QTableWidget::itemSelectionChanged, this, &PolePieceMeasureWin::new_slotTableSelectionChanged);
    connect(ui->tableFrame_B, &QTableWidget::itemSelectionChanged, this, &PolePieceMeasureWin::new_B_slotTableSelectionChanged);

    coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
    source_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    source_cloud_B = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    up_Points_left = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    up_Points_right = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    down_Points = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

  
    _ThreePoint2RGBcloud.RGB_left_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    _ThreePoint2RGBcloud.RGB_right_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    _ThreePoint2RGBcloud.RGB_down_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
    _ThreePoint2RGBcloud_B.RGB_left_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    _ThreePoint2RGBcloud_B.RGB_right_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    _ThreePoint2RGBcloud_B.RGB_down_Cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 

    // showResultPoint =pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

     ui->openGLWidget_Result->setAxesVisible(true);
     ui->openGLWidget_Result->setAxesShowLabels(true); 
     ui->openGLWidget_Result_B->setAxesVisible(true);
     ui->openGLWidget_Result_B->setAxesShowLabels(true);
     viewer = ui->openGLWidget_Result->m_viewer; 
     viewer_B = ui->openGLWidget_Result_B->m_viewer; 
 
     }
void PolePieceMeasureWin::closeEvent(QCloseEvent* event)
{
    //日志初始化
    auto temp = QMessageBox::information(this, "tooltip", tr("Save parameters upon closing?").toUtf8().constData(),
        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    if (temp == QMessageBox::Yes) {

        DestroyScanTab();
        destroySettingParam();//保存设置参数
        event->accept();
    }
    else if (temp == QMessageBox::Cancel) {
        event->ignore();
        return;
    }
    else {
        event->accept();
    }


    //deleteLater();
}
class NumericTableWidgetItem : public QTableWidgetItem//这个类让 QTableWidget 按数值排序，解决默认字符串排序导致的 "100" < "20" 这类错误。
{
public:
    NumericTableWidgetItem(const QString& text) : QTableWidgetItem(text) {}

    bool operator<(const QTableWidgetItem& other) const override
    {
        double v1 = text().toDouble();
        double v2 = other.text().toDouble();
        return v1 < v2;
    }
};
//加载并显示点云
void PolePieceMeasureWin::updateViewer(
    pcl::PointCloud<pcl::PointXYZ>::Ptr show_source_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_left,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_up_Points_right,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_down_Points)
{
    if (!viewer) return; // 防止空指针崩溃
    viewer->removeAllPointClouds();//将窗口里面所有点云清空

    // 优先使用传入的点云参数，否则用成员变量（默认显示）
    pcl::PointCloud<pcl::PointXYZ>::Ptr sc = show_source_cloud ? show_source_cloud : source_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr upl = show_up_Points_left ? show_up_Points_left : _ThreePoint2RGBcloud.RGB_left_Cloud;//点云类型是pcl::PointXYZRGB（包含XYZ坐标和RGB颜色信息）
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr upr = show_up_Points_right ? show_up_Points_right : _ThreePoint2RGBcloud.RGB_right_Cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dp = show_down_Points ? show_down_Points : _ThreePoint2RGBcloud.RGB_down_Cloud;

    if (sc && !sc->empty())
    {
        if (!viewer->contains("source_cloud"))
        {
            viewer->addPointCloud<pcl::PointXYZ>(sc, "source_cloud");//给这个点云设置一个标识符"source_cloud";使用viewer->addPointCloud将点云数据添加到可视化窗口中
        }
    }

    int ptSize = 15;
    // 使用setPointCloudRenderingProperties设置这个点云的渲染属性，具体是点的大小（PCL_VISUALIZER_POINT_SIZE）为ptSize变量指定的值

    if (upl && !upl->empty())
    {
        if (!viewer->contains("up_Points_left"))
        {
            viewer->addPointCloud<pcl::PointXYZRGB>(upl, "up_Points_left");//给这个点云设置一个标识符"up_Points_left";使用viewer->addPointCloud将点云数据添加到可视化窗口中
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, "up_Points_left");
        }
    }
    if (upr && !upr->empty())
    {
        if (!viewer->contains("up_Points_right"))
        {
            viewer->addPointCloud<pcl::PointXYZRGB>(upr, "up_Points_right");//给这个点云设置一个标识符"up_Points_right";使用viewer->addPointCloud将点云数据添加到可视化窗口中
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, "up_Points_right");
        }
    }
    if (dp && !dp->empty())
    {
        if (!viewer->contains("down_Points"))
        { 
            viewer->addPointCloud<pcl::PointXYZRGB>(dp, "down_Points");//给这个点云设置一个标识符"down_Points";使用viewer->addPointCloud将点云数据添加到可视化窗口中
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ptSize, "down_Points");
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
         
         ui->openGLWidget_Result->setAxesLength(length, length, length);
         
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
             viewer->setCameraPosition(
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

             viewer->setCameraPosition(
                 cam_x, cam_y, cam_z,
                 center_x, center_y, center_z,
                 0.0, 1.0, 0.0); 
         } 
    }
    
    ui->openGLWidget_Result->update();
   // ui->openGLWidget_Result->setCameraUpVector(0, 1, 0);;

    viewer->getRenderWindow()->Render(); 

}
void PolePieceMeasureWin::new_A_updateTable()
{
    ui->tableFrame->clearContents();//清除内容但保留表头
    int rowCount = static_cast<int>(Calucate_Width_Depth.m_width.size());
    if (Calucate_Width_Depth.m_depth.size() != Calucate_Width_Depth.m_width.size() || ID_SELECT.size() != Calucate_Width_Depth.m_width.size()) {
        return;
    }
    else
    {
        for (int i=0;i< ID_SELECT.size();++i)
        {
            ID_NEW.push_back(ID_SELECT[i]);
        }
    }
    if (ID_NEW.size()==0)
    {
        QMessageBox::warning(this, QStringLiteral("warning"),QStringLiteral( "未检出任何槽"));
        return;
    }
    InitLineChartShow();
    num_measure_A++;
    depth_data1.append(QPointF(num_measure_A, QString::number(1.000 * Calucate_Width_Depth.m_mean_depth, 'f', 4).toFloat()));
    depth_data1_min.append(QPointF(num_measure_A, QString::number(1.000 * Calucate_Width_Depth.m_min_depth, 'f', 4).toFloat()));
    depth_data1_max.append(QPointF(num_measure_A, QString::number(1.000 * Calucate_Width_Depth.m_max_depth, 'f', 4).toFloat()));
    /*QVector<QPointF> depth_data1 = {
         {1,QString::number(1.000*Calucate_Width_Depth.m_mean_depth, 'f', 2).toFloat()}, 
         {2,QString::number(1.000 * Calucate_Width_Depth.m_depth[ID_SELECT[0]], 'f', 2).toFloat()},
         {3,QString::number(1.000 * Calucate_Width_Depth.m_depth[ID_SELECT[1]], 'f', 2).toFloat()}, 
         {4,QString::number(1.000 * Calucate_Width_Depth.m_depth[ID_SELECT[2]], 'f', 2).toFloat()}, 
         {5,QString::number(1.000 * Calucate_Width_Depth.m_depth[ID_SELECT[3]], 'f', 2).toFloat()}
    };*/
    //计算CPK
   
    if (depth_data1.size() >=4)
    {
        double sx = 0, sy = 0;
        for (const QPointF& p : depth_data1) { sx += p.x(); sy += p.y(); }
        double n = depth_data1.size();
        QPointF avg = QPointF(sx / n, sy / n);

        SelectedData.meanDepth = avg.y();

        // 一行 lambda 求 y 的标准差
        SelectedData.standardDeviationDepth = [&] {
            const int n = depth_data1.size();
            if (n < 2) return 1000000.0;
            double m = 0;
            for (auto& p : depth_data1) m += p.y();
            m /= n;
            double var = 0;
            for (auto& p : depth_data1)
            {
                double d = p.y() - m;
                var += d * d;
            }
            return qSqrt(var / (n - 1));
            }();

        // 设置上下限值
        double upperLimit = InfoSuanFa.editDetectedDepthMax.toDouble();
        double lowerLimit = InfoSuanFa.editDetectedDepthMin.toDouble();
        double cpu = (upperLimit - SelectedData.meanDepth) / (3 * SelectedData.standardDeviationDepth);
        double cpl = (SelectedData.meanDepth - lowerLimit) / (3 * SelectedData.standardDeviationDepth);
        auto min2_2dec = [](double a, double b) -> double {
            return qRound(qMin(a, b) * 100) / 100.0;
            };

        SelectedData.Cpk_Depth = min2_2dec(cpu, cpl); // v == 2.72  
        cpk_depth_data1.append(QPointF(num_measure_A, SelectedData.Cpk_Depth));
    }
    else
    {
        cpk_depth_data1.clear();
    }
    if (cpk_depth_data1.size() > 4) {
        cpk_depth_data1.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (depth_data1.size() > 4) {
        depth_data1.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (depth_data1_min.size() > 4) {
        depth_data1_min.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (depth_data1_max.size() > 4) {
        depth_data1_max.removeFirst();          // 头部移除，O(n) 会移动数据
    }

     //计算CPK
   // depth_data1.append(QPointF(num_measure_A, QString::number(1000 * Calucate_Width_Depth.m_mean_depth, 'f', 4).toFloat()));
    
    /*QVector<QPointF> depth_data11 = {
        {1,QString::number(1000 * Calucate_Width_Depth.m_mean_depth, 'f', 2).toFloat()},
        {2,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[0]], 'f', 2).toFloat()},
        {3,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[1]], 'f', 2).toFloat()},
        {4,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[2]], 'f', 2).toFloat()},
        {5,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[3]], 'f', 2).toFloat()}
   };*/
    show_A_Depth_Line_Chart(depth_data1, cpk_depth_data1, depth_data1_min, depth_data1_max);

    width_data1.append(QPointF(num_measure_A, QString::number(1.000 * Calucate_Width_Depth.m_mean_width, 'f', 4).toFloat()));
    width_data1_min.append(QPointF(num_measure_A, QString::number(1.000 * Calucate_Width_Depth.m_min_width, 'f', 4).toFloat()));
    width_data1_max.append(QPointF(num_measure_A, QString::number(1.000 * Calucate_Width_Depth.m_max_width, 'f', 4).toFloat()));
    
    if (width_data1.size() >= 4)
    {
        double sx = 0, sy = 0;
        for (const QPointF& p : width_data1) { sx += p.x(); sy += p.y(); }
        double n = width_data1.size();
        QPointF avg = QPointF(sx / n, sy / n);

        SelectedData.meanWidth = avg.y();

        // 一行 lambda 求 y 的标准差
        SelectedData.standardDeviationWidth = [&] {
            const int n = width_data1.size();
            if (n < 2) return 1000000.0;
            double m = 0;
            for (auto& p : width_data1) m += p.y();
            m /= n;
            double var = 0;
            for (auto& p : width_data1)
            {
                double d = p.y() - m;
                var += d * d;
            }
            return qSqrt(var / (n - 1));
            }();

        // 设置上下限值
        double upperLimit = InfoSuanFa.editDetectedWidthMax.toDouble();
        double lowerLimit = InfoSuanFa.editDetectedWidthMin.toDouble();
        double cpu = (upperLimit - SelectedData.meanWidth) / (3 * SelectedData.standardDeviationWidth);
        double cpl = (SelectedData.meanWidth - lowerLimit) / (3 * SelectedData.standardDeviationWidth);
        auto min2_2dec = [](double a, double b) -> double {
            return qRound(qMin(a, b) * 100) / 100.0;
            };

        SelectedData.Cpk_Width = min2_2dec(cpu, cpl); // v == 2.72  
        cpk_width_data1.append(QPointF(num_measure_A, SelectedData.Cpk_Width));
    }
    else
    {
        cpk_width_data1.clear();
    }
    if (cpk_width_data1.size() > 4) {
        cpk_width_data1.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (width_data1.size() > 4) {
        width_data1.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (width_data1_min.size() > 4) {
        width_data1_min.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (width_data1_max.size() > 4) {
        width_data1_max.removeFirst();          // 头部移除，O(n) 会移动数据
    }

    /*{
         {1,QString::number(1000*Calucate_Width_Depth.m_mean_width, 'f', 2).toFloat()},
         {2,QString::number(1000 * Calucate_Width_Depth.m_width[ID_SELECT[0]], 'f', 2).toFloat()}, 
         {3,QString::number(1000 * Calucate_Width_Depth.m_width[ID_SELECT[1]], 'f', 2).toFloat()}, 
         {4,QString::number(1000 * Calucate_Width_Depth.m_width[ID_SELECT[2]], 'f', 2).toFloat()}, 
         {5,QString::number(1000 * Calucate_Width_Depth.m_width[ID_SELECT[3]], 'f', 2).toFloat()}
    };*/ 
    show_A_Width_Line_Chart(width_data1, cpk_width_data1, width_data1_min, width_data1_max);

    ui->tableFrame->setRowCount(rowCount);
    ui->tableFrame->setColumnCount(4);
    double minDepth = InfoSuanFa.editDetectedDepthMin.toDouble(); //ui->editDetectedDepthMin->text().toDouble();
    double maxDepth = InfoSuanFa.editDetectedDepthMax.toDouble();//ui->editDetectedDepthMax->text().toDouble();
    double minWidth = InfoSuanFa.editDetectedWidthMin.toDouble();//ui->editDetectedDepthMax->text().toDouble();
    double maxWidth = InfoSuanFa.editDetectedWidthMax.toDouble();//ui->editDetectedDepthMax->text().toDouble();
    ui->tableFrame->setSortingEnabled(false);//控制是否允许用户通过点击列头来对表格内容进行排序。
    // 遍历 InfoResult，将数据填入表格
    for (int i = 0; i < rowCount; ++i)
    {
        //ID号
        QTableWidgetItem* idItem = new NumericTableWidgetItem(QString::number(ID_NEW[i], 10));
        ui->tableFrame->setItem(i, 0, idItem);
        //宽度
        QTableWidgetItem* widthItem = new NumericTableWidgetItem(QString::number(1.000*Calucate_Width_Depth.m_width[i], 'f', 2));
        ui->tableFrame->setItem(i, 1, widthItem);

        //深度
        QTableWidgetItem* depthItem = new NumericTableWidgetItem(QString::number(1.000*Calucate_Width_Depth.m_depth[i], 'f', 2));
        ui->tableFrame->setItem(i, 2, depthItem);

        // 判断结果
        bool result_width = (minWidth < 1.000*Calucate_Width_Depth.m_width[i] && 1.000 * Calucate_Width_Depth.m_width[i] < maxWidth);;
        bool result_depth = (minDepth < 1.000 * Calucate_Width_Depth.m_depth[i] && 1.000 * Calucate_Width_Depth.m_depth[i] < maxDepth);;
        bool Result = (result_width && result_depth);   // 逻辑与
        QString result = Result ? QStringLiteral("OK") : QStringLiteral("NG");
        QTableWidgetItem* resultItem = new QTableWidgetItem(result);
        if (result == QStringLiteral("NG"))
        {
            resultItem->setForeground(QBrush(Qt::red));
            QFont font = resultItem->font();
            font.setBold(true);
            resultItem->setFont(font);

        }
        else if (result == QStringLiteral("OK"))
        {
            resultItem->setForeground(QBrush(QColor(0, 128, 0)));
            QFont font = resultItem->font();
            font.setBold(true);
            resultItem->setFont(font);
        }
        ui->tableFrame->setItem(i, 3, resultItem);
    }
    ui->tableFrame->setSortingEnabled(true); // 启用排序功能

    // 设置所有单元格内容居中
    for (int row = 0; row < ui->tableFrame->rowCount(); ++row)
    {
        for (int col = 0; col < ui->tableFrame->columnCount(); ++col)
        {
            QTableWidgetItem* item = ui->tableFrame->item(row, col);
            if (item)
                item->setTextAlignment(Qt::AlignCenter);
        }
    }


}

void  PolePieceMeasureWin::new_B_updateTable()//更新表格显示
{
    ui->tableFrame_B->clearContents();//清除内容但保留表头
    int rowCount = static_cast<int>(Calucate_Width_Depth_B.m_width.size());
    if (Calucate_Width_Depth_B.m_depth.size() != Calucate_Width_Depth_B.m_width.size() || ID_SELECT_B.size() != Calucate_Width_Depth_B.m_width.size()) {
        return;
    }
    else
    {
        for (int i = 0; i < ID_SELECT_B.size(); ++i)
        {
            ID_NEW_B.push_back(ID_SELECT_B[i]);
        }
    }
    if (ID_NEW_B.size() == 0)
    {
        QMessageBox::warning(this, QStringLiteral("warning"), QStringLiteral("未检出任何槽"));
        return;
    }
    InitLineChartShow();
    num_measure_B++;
    depth_data1_B.append(QPointF(num_measure_B, QString::number(1.000 * Calucate_Width_Depth_B.m_mean_depth, 'f', 4).toFloat()));
    depth_data1_B_min.append(QPointF(num_measure_B, QString::number(1.000 * Calucate_Width_Depth_B.m_min_depth, 'f', 4).toFloat()));
    depth_data1_B_max.append(QPointF(num_measure_B, QString::number(1.000 * Calucate_Width_Depth_B.m_max_depth, 'f', 4).toFloat()));
    /*QVector<QPointF> depth_data1 = {
         {1,QString::number(1000*Calucate_Width_Depth.m_mean_depth, 'f', 2).toFloat()},
         {2,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[0]], 'f', 2).toFloat()},
         {3,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[1]], 'f', 2).toFloat()},
         {4,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[2]], 'f', 2).toFloat()},
         {5,QString::number(1000 * Calucate_Width_Depth.m_depth[ID_SELECT[3]], 'f', 2).toFloat()}
    };*/
    //计算CPK
     if (depth_data1_B.size() >= 4)
    {
    double sx = 0, sy = 0;
    for (const QPointF& p : depth_data1_B) { sx += p.x(); sy += p.y(); }
    double n = depth_data1_B.size();
    QPointF avg = QPointF(sx / n, sy / n);

    SelectedData_B.meanDepth = avg.y();

    // 一行 lambda 求 y 的标准差
    SelectedData_B.standardDeviationDepth = [&] {
        const int n = depth_data1_B.size();
        if (n < 2) return 1000000.0;
        double m = 0;
        for (auto& p : depth_data1_B) m += p.y();
        m /= n;
        double var = 0;
        for (auto& p : depth_data1_B)
        {
            double d = p.y() - m;
            var += d * d;
        }
        return qSqrt(var / (n - 1));
        }();

    // 设置上下限值
    double upperLimit = InfoSuanFa.editDetectedDepthMax.toDouble();
    double lowerLimit = InfoSuanFa.editDetectedDepthMin.toDouble();
    double cpu = (upperLimit - SelectedData_B.meanDepth) / (3 * SelectedData_B.standardDeviationDepth);
    double cpl = (SelectedData_B.meanDepth - lowerLimit) / (3 * SelectedData_B.standardDeviationDepth);
    auto min2_2dec = [](double a, double b) -> double {
        return qRound(qMin(a, b) * 100) / 100.0;
        };

        SelectedData_B.Cpk_Depth = min2_2dec(cpu, cpl); // v == 2.72   
        cpk_depth_data1_B.append(QPointF(num_measure_B, SelectedData_B.Cpk_Depth));
     }
     else
     {
         cpk_depth_data1_B.clear();
     } 
     if (cpk_depth_data1_B.size() > 4) {
         cpk_depth_data1_B.removeFirst();          // 头部移除，O(n) 会移动数据
     }
     if (depth_data1_B.size() > 4) {
         depth_data1_B.removeFirst();          // 头部移除，O(n) 会移动数据
     }
     if (depth_data1_B_min.size() > 4) {
         depth_data1_B_min.removeFirst();          // 头部移除，O(n) 会移动数据
     }
     if (depth_data1_B_max.size() > 4) {
         depth_data1_B_max.removeFirst();          // 头部移除，O(n) 会移动数据
     }
     //显示B深度曲线
    show_B_Depth_Line_Chart(depth_data1_B, cpk_depth_data1_B, depth_data1_B_min, depth_data1_B_max);

    width_data1_B.append(QPointF(num_measure_B, QString::number(1.000 * Calucate_Width_Depth_B.m_mean_width, 'f', 4).toFloat()));
    width_data1_B_min.append(QPointF(num_measure_B, QString::number(1.000 * Calucate_Width_Depth_B.m_min_width, 'f', 4).toFloat()));
    width_data1_B_max.append(QPointF(num_measure_B, QString::number(1.000 * Calucate_Width_Depth_B.m_max_width, 'f', 4).toFloat())); 


    if (width_data1_B.size() >= 4)
    {
        double sx = 0, sy = 0;
        for (const QPointF& p : width_data1_B) { sx += p.x(); sy += p.y(); }
        double n = width_data1_B.size();
        QPointF avg = QPointF(sx / n, sy / n);

        SelectedData_B.meanWidth = avg.y();

        // 一行 lambda 求 y 的标准差
        SelectedData_B.standardDeviationWidth = [&] {
            const int n = width_data1_B.size();
            if (n < 2) return 1000000.0;
            double m = 0;
            for (auto& p : width_data1_B) m += p.y();
            m /= n;
            double var = 0;
            for (auto& p : width_data1_B)
            {
                double d = p.y() - m;
                var += d * d;
            }
            return qSqrt(var / (n - 1));
            }();

        // 设置上下限值
        double upperLimit = InfoSuanFa.editDetectedWidthMax.toDouble();
        double lowerLimit = InfoSuanFa.editDetectedWidthMin.toDouble();
        double cpu = (upperLimit - SelectedData_B.meanWidth) / (3 * SelectedData_B.standardDeviationWidth);
        double cpl = (SelectedData_B.meanWidth - lowerLimit) / (3 * SelectedData_B.standardDeviationWidth);
        auto min2_2dec = [](double a, double b) -> double {
            return qRound(qMin(a, b) * 100) / 100.0;
            };

        SelectedData_B.Cpk_Width = min2_2dec(cpu, cpl); // v == 2.72   
        cpk_width_data1_B.append(QPointF(num_measure_B, SelectedData_B.Cpk_Width));
    }
    if (cpk_width_data1_B.size() > 4) {
        cpk_width_data1_B.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (width_data1_B.size() > 4) {
        width_data1_B.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (width_data1_B_min.size() > 4) {
        width_data1_B_min.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    if (width_data1_B_max.size() > 4) {
        width_data1_B_max.removeFirst();          // 头部移除，O(n) 会移动数据
    }
    //显示B宽度曲线
    show_B_Width_Line_Chart(width_data1_B, cpk_width_data1_B, width_data1_B_min, width_data1_B_max);

    ui->tableFrame_B->setRowCount(rowCount);
    ui->tableFrame_B->setColumnCount(4);
    double minDepth = InfoSuanFa.editDetectedDepthMin.toDouble(); //ui->editDetectedDepthMin->text().toDouble();
    double maxDepth = InfoSuanFa.editDetectedDepthMax.toDouble();//ui->editDetectedDepthMax->text().toDouble();
    double minWidth = InfoSuanFa.editDetectedWidthMin.toDouble();//ui->editDetectedDepthMax->text().toDouble();
    double maxWidth = InfoSuanFa.editDetectedWidthMax.toDouble();//ui->editDetectedDepthMax->text().toDouble();
    ui->tableFrame_B->setSortingEnabled(false);//控制是否允许用户通过点击列头来对表格内容进行排序。
    // 遍历 InfoResult，将数据填入表格
    for (int i = 0; i < rowCount; ++i)
    {
        //ID号
        QTableWidgetItem* idItem = new NumericTableWidgetItem(QString::number(ID_NEW_B[i], 10));
        ui->tableFrame_B->setItem(i, 0, idItem);
        //宽度
        QTableWidgetItem* widthItem = new NumericTableWidgetItem(QString::number(1.000 * Calucate_Width_Depth_B.m_width[i], 'f', 2));
        ui->tableFrame_B->setItem(i, 1, widthItem);

        //深度
        QTableWidgetItem* depthItem = new NumericTableWidgetItem(QString::number(1.000 * Calucate_Width_Depth_B.m_depth[i], 'f', 2));
        ui->tableFrame_B->setItem(i, 2, depthItem);

        // 判断结果
        bool result_width = (minWidth < 1.000 * Calucate_Width_Depth_B.m_width[i] && 1.000 * Calucate_Width_Depth_B.m_width[i] < maxWidth);;
        bool result_depth = (minDepth < 1.000 * Calucate_Width_Depth_B.m_depth[i] && 1.000 * Calucate_Width_Depth_B.m_depth[i] < maxDepth);;
        bool Result = (result_width && result_depth);   // 逻辑与
        QString result = Result ? QStringLiteral("OK") : QStringLiteral("NG");
        QTableWidgetItem* resultItem = new QTableWidgetItem(result);
        if (result == QStringLiteral("NG"))
        {
            resultItem->setForeground(QBrush(Qt::red));
            QFont font = resultItem->font();
            font.setBold(true);
            resultItem->setFont(font);

        }
        else if (result == QStringLiteral("OK"))
        {
            resultItem->setForeground(QBrush(QColor(0, 128, 0)));
            QFont font = resultItem->font();
            font.setBold(true);
            resultItem->setFont(font);
        }
        ui->tableFrame_B->setItem(i, 3, resultItem);
    }
    ui->tableFrame_B->setSortingEnabled(true); // 启用排序功能

    // 设置所有单元格内容居中
    for (int row = 0; row < ui->tableFrame_B->rowCount(); ++row)
    {
        for (int col = 0; col < ui->tableFrame_B->columnCount(); ++col)
        {
            QTableWidgetItem* item = ui->tableFrame_B->item(row, col);
            if (item)
                item->setTextAlignment(Qt::AlignCenter);
        }
    }

}

void PolePieceMeasureWin::updateTable()
{
    ui->tableFrame->clearContents();//清除内容但保留表头
    int rowCount = static_cast<int>(InfoResult.width.size());
    if (InfoResult.depth.size() != InfoResult.width.size() || ID.size() != InfoResult.width.size()) {
        return;
    }

    ui->tableFrame->setRowCount(rowCount);
    ui->tableFrame->setColumnCount(4);
    double maxDepth = InfoSuanFa.editDetectedDepthMin.toDouble();//ui->editDetectedDepthMin->text().toDouble();
    ui->tableFrame->setSortingEnabled(false);//控制是否允许用户通过点击列头来对表格内容进行排序。
    // 遍历 InfoResult，将数据填入表格
    for (size_t i=0; i<rowCount; ++i)
    {
         //ID号
        QTableWidgetItem* idItem = new NumericTableWidgetItem(QString::number(ID[i],10));
        ui->tableFrame->setItem(i, 0, idItem);
        //宽度
        QTableWidgetItem* widthItem = new NumericTableWidgetItem(QString::number(InfoResult.width[i], 'f', 3));
        ui->tableFrame->setItem(i, 1, widthItem);

        //深度
        QTableWidgetItem* depthItem = new NumericTableWidgetItem(QString::number(InfoResult.depth[i], 'f', 3));
        ui->tableFrame->setItem(i, 2, depthItem);

        // 判断结果
        QString result = (InfoResult.depth[i] <= maxDepth) ? QStringLiteral("NG") : QStringLiteral("OK");
        QTableWidgetItem* resultItem = new QTableWidgetItem(result);
        if (result==QStringLiteral("NG"))
        {
            resultItem->setForeground(QBrush(Qt::red));
            QFont font = resultItem->font();
            font.setBold(true);
            resultItem->setFont(font);

        }
        else if(result== QStringLiteral("OK"))
        {
            resultItem->setForeground(QBrush(QColor(0,128,0)));
            QFont font = resultItem->font();
            font.setBold(true);
            resultItem->setFont(font);  
        }
        ui->tableFrame->setItem(i, 3, resultItem);  
    }
    ui->tableFrame->setSortingEnabled(true); // 启用排序功能

    // 设置所有单元格内容居中
    for (int row = 0; row < ui->tableFrame->rowCount(); ++row)
    {
        for (int col = 0; col < ui->tableFrame->columnCount(); ++col) 
        {
            QTableWidgetItem* item = ui->tableFrame->item(row, col);
            if (item)
                item->setTextAlignment(Qt::AlignCenter);
        }
    }




}
void PolePieceMeasureWin::showPointCloud(pcl::PointCloud<pcl::PointXYZ>& input)
{
    initUIViewer();

    //----------------------------qvtkWidget的初始化-----------------------------------
    //----------------------------初始化点云并显示-------------------------------------
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    //刷新显示窗口
    ui->openGLWidget_Result->repaint();
    //添加点云
    viewer->addPointCloud(input.makeShared(), "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 15, "cloud");
    //刷新
    ui->openGLWidget_Result->update();
    //视角
    viewer->resetCamera();
    //第一次显示
    viewer->spin();
    //----------------------------初始化点云并显示-------------------------------------
}
void PolePieceMeasureWin::initUIViewer()
{
    if (!actionViewer)
    {
        GlobalMyViewRendererXia = vtkRenderer::New();
        GlobalMyViewRenderWindowXia = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
        // 1. 创建坐标轴
        vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
        /* actor2->SetPosition(0, 0, 0);  // 设定坐标轴位置
    actor2->SetTotalLength(100, 100, 100);
    actor2->SetShaftType(1);  // 设置轴的类型
    actor2->SetAxisLabels(1);  // 显示坐标轴标签
    actor2->SetCylinderRadius(0.0002);  // 轴尖半径设置*/
        axes->SetPosition(0, 0, 0);
        axes->SetTotalLength(100, 100, 100); // 箭头长度 (X,Y,Z) 单位：米/毫米
        axes->SetShaftType(1);          // 可选：圆柱/线
        axes->SetCylinderRadius(0.002);       // 如果选圆柱 轴尖半径设置
        axes->SetAxisLabels(1);       // 显示坐标轴标签
       // axes->SetXAxisLabelText("X"); axes->SetYAxisLabelText("Y"); axes->SetZAxisLabelText("Z");
        //axes->SetAxisLabels(false);
        // 2. 加到渲染器
        GlobalMyViewRendererXia->AddActor(axes);
        // 3. 立即刷新
        ui->openGLWidget_Result->renderWindow()->Render();

        GlobalMyViewRenderWindowXia->AddRenderer(GlobalMyViewRendererXia);
        viewer.reset(new pcl::visualization::PCLVisualizer(GlobalMyViewRendererXia, GlobalMyViewRenderWindowXia, "viewer", false));
        ui->openGLWidget_Result->setRenderWindow(viewer->getRenderWindow());//此处qvtkWidget是我这个窗口的名称，自定义的
        viewer->setupInteractor(ui->openGLWidget_Result->interactor(), ui->openGLWidget_Result->renderWindow());
        actionViewer = true;
    }
}

 
void PolePieceMeasureWin::new_B_slotTableSelectionChanged()
{
    auto toQString = [](double val) -> QString {
        return QString::number(val, 'f', 2) + " um";
        };
    QModelIndexList selection = ui->tableFrame_B->selectionModel()->selectedRows();
    int selectionCount = selection.size();

    if ((selectionCount == 0) || (ID_NEW_B.size() != 1 && selectionCount == ID_NEW_B.size()))
    {
        //ui->statusLabel->setAnimatedText(QStringLiteral("正在浏览全局识别结果，可从表格中选择单帧浏览"));
        // 没选中任何行或全选，恢复默认显示
       // updateViewer_B();
        if (ui->checkbox_roi->isChecked())
        {
            updateViewer_B(roi_cloud_B);
        }
        else
        {
            updateViewer_B();
        }
        // 获取宽度阈值
        bool okWidth = false;
        float detectedWidthValue = InfoSuanFa.editDetectedWidthMin.toFloat(); //ui->editDetectedWidthMin->text().toFloat(&okWidth);
        if (!okWidth) detectedWidthValue = std::numeric_limits<float>::max();//如果输入的数据格式

        // 获取深度阈值
        bool okDepth = false;
        bool okDepth1 = false;
        float detectedDepthValue = InfoSuanFa.editDetectedDepthMin.toFloat(); //ui->editDetectedDepthMin->text().toFloat(&okDepth); 
        //if (!okDepth) detectedDepthValue = std::numeric_limits<float>::max();

        float detectedDepthValueMax = InfoSuanFa.editDetectedDepthMax.toFloat();
       // if (!okDepth1) detectedDepthValueMax = std::numeric_limits<float>::max();

        // 显示宽度信息并判断样式
        ui->labelMeanWidth->setText(toQString(1.000*Calucate_Width_Depth.m_mean_width));
        ui->labelMaxWidth->setText(toQString(1.000*Calucate_Width_Depth.m_max_width));
        ui->labelMinWidth->setText(toQString(1.000*Calucate_Width_Depth.m_min_width));

        ui->labelMeanDepth->setText(toQString(1.000*Calucate_Width_Depth.m_mean_depth));
        ui->labelMaxDepth->setText(toQString(1.000*Calucate_Width_Depth.m_max_depth));
        ui->labelMinDepth->setText(toQString(1.000*Calucate_Width_Depth.m_min_depth));

        // 判断并设置宽度标签样式
        auto setStyle = [](QLabel* label, bool highlight) {
            if (highlight)
                label->setStyleSheet("color: red; font-weight: bold;");
            else
                label->setStyleSheet("color: rgb(0, 128, 0);");
            };

        setStyle(ui->labelMeanWidth, Calucate_Width_Depth.m_mean_width > detectedWidthValue);
        setStyle(ui->labelMaxWidth, Calucate_Width_Depth.m_max_width > detectedWidthValue);
        setStyle(ui->labelMinWidth, Calucate_Width_Depth.m_min_width > detectedWidthValue);

        // 判断并设置深度标签样式
        setStyle(ui->labelMeanDepth, Calucate_Width_Depth.m_mean_depth > detectedDepthValue);
        setStyle(ui->labelMaxDepth, Calucate_Width_Depth.m_max_depth > detectedDepthValue);
        setStyle(ui->labelMinDepth, Calucate_Width_Depth.m_min_depth > detectedDepthValue);

        return;
    }

    // 多选时合成点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr show_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_left(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_right(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_down(new pcl::PointCloud<pcl::PointXYZRGB>);
    QVector<double> widths, depths;

    for (const QModelIndex& idx : selection)
    {
        int row = idx.row();
        if (row < 0 || row >= ID_NEW_B.size()) continue; // 防止越界

        // 取第一列ID
        QTableWidgetItem* idItem = ui->tableFrame_B->item(row, 0);
        if (!idItem) continue;

        int idValue = idItem->text().toInt();

        // 根据idValue查找对应数据索引
        int dataIndex = -1;
        for (int i = 0; i < ID_NEW_B.size(); ++i)
        {
            if (ID_NEW_B[i] == idValue)
            {
                dataIndex = i;
                break;
            }
        }
        if (dataIndex == -1) continue;

        // 使用 dataIndex 访问数据而非 row
        // 合并source点云
        if (ui->checkbox_roi->isChecked())
        {
            separate_clouds_B.clear();
            separate_clouds_B = separateCloud(roi_cloud_B);
        }
        if (separate_clouds_B.empty())
        {
            separate_clouds_B = separateCloud(source_cloud_B);

        }
        if (separate_clouds_B[ID_NEW_B[dataIndex]])
        {
            *show_source += *separate_clouds_B[ID_NEW_B[dataIndex]];
        }
        //PolePieceMeasureWin::ThreePoint2RGBcloud threeRGBPoint;
        // 合并left点
        if (_ThreePoint2RGBcloud_B.RGB_left_Cloud && dataIndex < _ThreePoint2RGBcloud_B.RGB_left_Cloud->points.size())
        { 
            show_left->points.push_back(_ThreePoint2RGBcloud_B.RGB_left_Cloud->points[dataIndex]);
        }

        // 合并right点
        if (_ThreePoint2RGBcloud_B.RGB_right_Cloud && dataIndex < _ThreePoint2RGBcloud_B.RGB_right_Cloud->points.size())
        {
            show_right->points.push_back(_ThreePoint2RGBcloud_B.RGB_right_Cloud->points[dataIndex]);
        }

        // 合并down点
        if (_ThreePoint2RGBcloud_B.RGB_down_Cloud && dataIndex < _ThreePoint2RGBcloud_B.RGB_down_Cloud->points.size())
        {
            show_down->points.push_back(_ThreePoint2RGBcloud_B.RGB_down_Cloud->points[dataIndex]);
        }

        // 取宽度和深度改用 dataIndex
        widths.append(Calucate_Width_Depth_B.m_width[dataIndex]);
        depths.append(Calucate_Width_Depth_B.m_depth[dataIndex]);
    }

    // 设置合并后点云参数
    show_left->width = show_left->points.size();
    show_left->height = 1;
    show_left->is_dense = true;

    show_right->width = show_right->points.size();
    show_right->height = 1;
    show_right->is_dense = true;

    show_down->width = show_down->points.size();
    show_down->height = 1;
    show_down->is_dense = true;

    updateViewer_B(show_source, show_left, show_right, show_down);
    /*ui->openGLWidget_Result_B->setCameraUpVector(0, 1, 0);
    ui->openGLWidget_Result_B->showPointCloud(show_source);
    ui->openGLWidget_Result_B->showPointCloud(show_left);
    ui->openGLWidget_Result_B->showPointCloud(show_right);
    ui->openGLWidget_Result_B->showPointCloud(show_down);*/
    // 计算宽度最大最小平均
    double width_max = *std::max_element(widths.begin(), widths.end());
    double width_min = *std::min_element(widths.begin(), widths.end());
    double width_avg = std::accumulate(widths.begin(), widths.end(), 0.0) / widths.size();

    // 计算深度最大最小平均
    double depth_max = *std::max_element(depths.begin(), depths.end());
    double depth_min = *std::min_element(depths.begin(), depths.end());
    double depth_avg = std::accumulate(depths.begin(), depths.end(), 0.0) / depths.size();

    // 获取宽度阈值
    bool okWidth = false;
    float detectedWidthValue = InfoSuanFa.editDetectedWidthMin.toFloat();//ui->editDetectedWidthMin->text().toFloat(&okWidth);
    if (!okWidth) detectedWidthValue = std::numeric_limits<float>::max();

    // 获取深度阈值
    bool okDepth = false;
    float detectedDepthValue = InfoSuanFa.editDetectedDepthMin.toDouble(); //ui->editDetectedDepthMin->text().toFloat(&okDepth);
    if (!okDepth) detectedDepthValue = std::numeric_limits<float>::max();
    /* 同一个 lambda，参数换成单元格指针 */
    auto setItemStyle = [](QTableWidgetItem* item, bool highlight) {
        if (!item) return;
        item->setForeground(highlight
            ? QBrush(Qt::red)                // 高亮：红色加粗
            : QBrush(QColor(255, 255, 255)));    // 普通：绿色
        /* 如果想加粗再补一句 */
        QFont f = item->font();
        f.setBold(highlight);
        item->setFont(f);
        };
    width_avg = 1.000 * width_avg;
    width_max = 1.000 * width_max;
    width_min = 1.000 * width_min;

    depth_avg = 1.000 * depth_avg;
    depth_max = 1.000 * depth_max;
    depth_min = 1.000 * depth_min;
    
    //    /* 第一列：宽度 */
    ui->tb_Widget_B->setItem(0, 1, new QTableWidgetItem(toQString(width_avg)));
    ui->tb_Widget_B->setItem(1, 1, new QTableWidgetItem(toQString(width_max)));
    ui->tb_Widget_B->setItem(2, 1, new QTableWidgetItem(toQString(width_min)));
    ui->tb_Widget_B->setItem(0, 0, new QTableWidgetItem(QStringLiteral("平均宽度")));
    ui->tb_Widget_B->setItem(1, 0, new QTableWidgetItem(QStringLiteral("最大宽度")));
    ui->tb_Widget_B->setItem(2, 0, new QTableWidgetItem(QStringLiteral("最小宽度"))); 

/* 第二列：空白，不用填 */
    /*QTableWidgetItem* widthItem = new NumericTableWidgetItem(QString::number(1000*Calucate_Width_Depth.m_width[i], 'f', 2));
        ui->tableFrame->setItem(i, 1, widthItem);*/
/* 第三列：深度 */
    ui->tb_Widget_B->setItem(0, 2, new QTableWidgetItem(QStringLiteral("平均深度")));
    ui->tb_Widget_B->setItem(1, 2, new QTableWidgetItem(QStringLiteral("最大深度")));
    ui->tb_Widget_B->setItem(2, 2, new QTableWidgetItem(QStringLiteral("最小深度")));
    ui->tb_Widget_B->setItem(0, 3, new QTableWidgetItem(toQString(depth_avg)));
    ui->tb_Widget_B->setItem(1, 3, new QTableWidgetItem(toQString(depth_max)));
    ui->tb_Widget_B->setItem(2, 3, new QTableWidgetItem(toQString(depth_min)));
    // 显示宽度信息并判断样式
    ui->labelMeanWidth->setText(toQString(width_avg));
    ui->labelMaxWidth->setText(toQString(width_max));
    ui->labelMinWidth->setText(toQString(width_min));

    ui->labelMeanDepth->setText(toQString(depth_avg));
    ui->labelMaxDepth->setText(toQString(depth_max));
    ui->labelMinDepth->setText(toQString(depth_min));

    setItemStyle(ui->tb_Widget_B->item(0, 1), InfoSuanFa.editDetectedWidthMin.toDouble() <= width_avg <= InfoSuanFa.editDetectedWidthMax.toDouble());
    setItemStyle(ui->tb_Widget_B->item(1, 1), InfoSuanFa.editDetectedWidthMin.toDouble() <= width_max <= InfoSuanFa.editDetectedWidthMax.toDouble());
    setItemStyle(ui->tb_Widget_B->item(2, 1), InfoSuanFa.editDetectedWidthMin.toDouble() <= width_min <= InfoSuanFa.editDetectedWidthMax.toDouble());
    setItemStyle(ui->tb_Widget_B->item(0, 3), InfoSuanFa.editDetectedDepthMin.toDouble() <= depth_avg <= InfoSuanFa.editDetectedDepthMax.toDouble());
    setItemStyle(ui->tb_Widget_B->item(1, 3), InfoSuanFa.editDetectedDepthMin.toDouble() <= depth_max <= InfoSuanFa.editDetectedDepthMax.toDouble());
    setItemStyle(ui->tb_Widget_B->item(2, 3), InfoSuanFa.editDetectedDepthMin.toDouble() <= depth_min <= InfoSuanFa.editDetectedDepthMax.toDouble());

    auto setStyle = [](QLabel* label, bool highlight) {
        if (highlight)
            label->setStyleSheet("color: red; font-weight: bold;");
        else
            label->setStyleSheet("color: rgb(0, 128, 0);");
        };

    // 判断并设置宽度标签样式
    setStyle(ui->labelMeanWidth, width_avg > detectedWidthValue);
    setStyle(ui->labelMaxWidth, width_max > detectedWidthValue);
    setStyle(ui->labelMinWidth, width_min > detectedWidthValue);

    // 判断并设置深度标签样式
    setStyle(ui->labelMeanDepth, depth_avg > detectedDepthValue);
    setStyle(ui->labelMaxDepth, depth_max > detectedDepthValue);
    setStyle(ui->labelMinDepth, depth_min > detectedDepthValue);

    // 先判断宽度和深度是否超阈值
    bool widthExceeded = (width_avg > detectedWidthValue);
    bool depthExceeded = (depth_avg > detectedDepthValue);

}

void PolePieceMeasureWin::new_slotTableSelectionChanged()
{
    auto toQString = [](double val) -> QString {
        return QString::number(val, 'f', 2) + " um";
        };
    QModelIndexList selection = ui->tableFrame->selectionModel()->selectedRows();
    int selectionCount = selection.size();

    if ((selectionCount == 0) || (ID_NEW.size() != 1 && selectionCount == ID_NEW.size()))
    {
        //ui->statusLabel->setAnimatedText(QStringLiteral("正在浏览全局识别结果，可从表格中选择单帧浏览"));
        // 没选中任何行或全选，恢复默认显示
        if (ui->checkbox_roi->isChecked())
        {
            updateViewer(roi_cloud); 
       }
        else
        {
            updateViewer(); 
        }

        // 获取宽度阈值
        bool okWidth = false;
        float detectedWidthValue = InfoSuanFa.editDetectedWidthMin.toFloat(); //ui->editDetectedWidthMin->text().toFloat(&okWidth);
        if (!okWidth) detectedWidthValue = std::numeric_limits<float>::max();//如果输入的数据格式

        // 获取深度阈值
        bool okDepth = false;
        bool okDepth1 = false;
        float detectedDepthValue = InfoSuanFa.editDetectedDepthMin.toFloat(); //ui->editDetectedDepthMin->text().toFloat(&okDepth); 
        //if (!okDepth) detectedDepthValue = std::numeric_limits<float>::max();

        float detectedDepthValueMax = InfoSuanFa.editDetectedDepthMax.toFloat();
       // if (!okDepth1) detectedDepthValueMax = std::numeric_limits<float>::max();

        // 显示宽度信息并判断样式
        ui->labelMeanWidth->setText(toQString(1.000*Calucate_Width_Depth.m_mean_width));
        ui->labelMaxWidth->setText(toQString(1.000*Calucate_Width_Depth.m_max_width));
        ui->labelMinWidth->setText(toQString(1.000*Calucate_Width_Depth.m_min_width));

        ui->labelMeanDepth->setText(toQString(1.000*Calucate_Width_Depth.m_mean_depth));
        ui->labelMaxDepth->setText(toQString(1.000*Calucate_Width_Depth.m_max_depth));
        ui->labelMinDepth->setText(toQString(1.000*Calucate_Width_Depth.m_min_depth));

        // 判断并设置宽度标签样式
        auto setStyle = [](QLabel* label, bool highlight) {
            if (highlight)
                label->setStyleSheet("color: red; font-weight: bold;");
            else
                label->setStyleSheet("color: rgb(0, 128, 0);");
            };

        setStyle(ui->labelMeanWidth, Calucate_Width_Depth.m_mean_width > detectedWidthValue);
        setStyle(ui->labelMaxWidth, Calucate_Width_Depth.m_max_width > detectedWidthValue);
        setStyle(ui->labelMinWidth, Calucate_Width_Depth.m_min_width > detectedWidthValue);

        // 判断并设置深度标签样式
        setStyle(ui->labelMeanDepth, Calucate_Width_Depth.m_mean_depth > detectedDepthValue);
        setStyle(ui->labelMaxDepth, Calucate_Width_Depth.m_max_depth > detectedDepthValue);
        setStyle(ui->labelMinDepth, Calucate_Width_Depth.m_min_depth > detectedDepthValue);

        return;
    }

    // 多选时合成点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr show_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_left(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_right(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_down(new pcl::PointCloud<pcl::PointXYZRGB>);
    QVector<double> widths, depths;

    for (const QModelIndex& idx : selection)
    {
        int row = idx.row();
        if (row < 0 || row >= ID_NEW.size()) continue; // 防止越界

        // 取第一列ID
        QTableWidgetItem* idItem = ui->tableFrame->item(row, 0);
        if (!idItem) continue;

        int idValue = idItem->text().toInt();

        // 根据idValue查找对应数据索引
        int dataIndex = -1;
        for (int i = 0; i < ID_NEW.size(); ++i)
        {
            if (ID_NEW[i] == idValue)
            {
                dataIndex = i;
                break;
            }
        }
        if (dataIndex == -1) continue;

        // 使用 dataIndex 访问数据而非 row
        // 合并source点云
        if (ui->checkbox_roi->isChecked())
        {
            separate_clouds.clear();
            separate_clouds = separateCloud(roi_cloud); 
        }
        if (separate_clouds.empty())
        {
            separate_clouds = separateCloud(source_cloud);

        }
        if (separate_clouds[ID_NEW[dataIndex]])
        {
            *show_source += *separate_clouds[ID_NEW[dataIndex]];
        }
        //PolePieceMeasureWin::ThreePoint2RGBcloud threeRGBPoint;
        // 合并left点
        if (_ThreePoint2RGBcloud.RGB_left_Cloud && dataIndex < _ThreePoint2RGBcloud.RGB_left_Cloud->points.size())
        { 
            show_left->points.push_back(_ThreePoint2RGBcloud.RGB_left_Cloud->points[dataIndex]);
        }

        // 合并right点
        if (_ThreePoint2RGBcloud.RGB_right_Cloud && dataIndex < _ThreePoint2RGBcloud.RGB_right_Cloud->points.size())
        {
            show_right->points.push_back(_ThreePoint2RGBcloud.RGB_right_Cloud->points[dataIndex]);
        }

        // 合并down点
        if (_ThreePoint2RGBcloud.RGB_down_Cloud && dataIndex < _ThreePoint2RGBcloud.RGB_down_Cloud->points.size())
        {
            show_down->points.push_back(_ThreePoint2RGBcloud.RGB_down_Cloud->points[dataIndex]);
        }

        // 取宽度和深度改用 dataIndex
        widths.append(Calucate_Width_Depth.m_width[dataIndex]);
        depths.append(Calucate_Width_Depth.m_depth[dataIndex]);
    }

    // 设置合并后点云参数
    show_left->width = show_left->points.size();
    show_left->height = 1;
    show_left->is_dense = true;

    show_right->width = show_right->points.size();
    show_right->height = 1;
    show_right->is_dense = true;

    show_down->width = show_down->points.size();
    show_down->height = 1;
    show_down->is_dense = true;

    updateViewer(show_source, show_left, show_right, show_down);

    // 计算宽度最大最小平均
    double width_max = *std::max_element(widths.begin(), widths.end());
    double width_min = *std::min_element(widths.begin(), widths.end());
    double width_avg = std::accumulate(widths.begin(), widths.end(), 0.0) / widths.size();

    // 计算深度最大最小平均
    double depth_max = *std::max_element(depths.begin(), depths.end());
    double depth_min = *std::min_element(depths.begin(), depths.end());
    double depth_avg = std::accumulate(depths.begin(), depths.end(), 0.0) / depths.size();

    // 获取宽度阈值
    bool okWidth = false;
    float detectedWidthValue = InfoSuanFa.editDetectedWidthMin.toFloat();//ui->editDetectedWidthMin->text().toFloat(&okWidth);
    if (!okWidth) detectedWidthValue = std::numeric_limits<float>::max();

    // 获取深度阈值
    bool okDepth = false;
    float detectedDepthValue = InfoSuanFa.editDetectedDepthMin.toDouble(); //ui->editDetectedDepthMin->text().toFloat(&okDepth);
    if (!okDepth) detectedDepthValue = std::numeric_limits<float>::max();
    /* 同一个 lambda，参数换成单元格指针 */
    auto setItemStyle = [](QTableWidgetItem* item, bool highlight) {
        if (!item) return;
        item->setForeground(highlight
            ? QBrush(Qt::red)                // 高亮：红色加粗
            : QBrush(QColor(255, 255, 255)));    // 普通：绿色
        /* 如果想加粗再补一句 */
        QFont f = item->font();
        f.setBold(highlight);
        item->setFont(f);
        };
    width_avg = 1.000 * width_avg;
    width_max = 1.000 * width_max;
    width_min = 1.000 * width_min;

    depth_avg = 1.000 * depth_avg;
    depth_max = 1.000 * depth_max;
    depth_min = 1.000 * depth_min;
    
    //    /* 第一列：宽度 */
    ui->tb_Widget_A->setItem(0, 1, new QTableWidgetItem(toQString(width_avg)));
    ui->tb_Widget_A->setItem(1, 1, new QTableWidgetItem(toQString(width_max)));
    ui->tb_Widget_A->setItem(2, 1, new QTableWidgetItem(toQString(width_min)));
    ui->tb_Widget_A->setItem(0, 0, new QTableWidgetItem(QStringLiteral("平均宽度")));
    ui->tb_Widget_A->setItem(1, 0, new QTableWidgetItem(QStringLiteral("最大宽度")));
    ui->tb_Widget_A->setItem(2, 0, new QTableWidgetItem(QStringLiteral("最小宽度"))); 

/* 第二列：空白，不用填 */
    /*QTableWidgetItem* widthItem = new NumericTableWidgetItem(QString::number(1000*Calucate_Width_Depth.m_width[i], 'f', 2));
        ui->tableFrame->setItem(i, 1, widthItem);*/
/* 第三列：深度 */
    ui->tb_Widget_A->setItem(0, 2, new QTableWidgetItem(QStringLiteral("平均深度")));
    ui->tb_Widget_A->setItem(1, 2, new QTableWidgetItem(QStringLiteral("最大深度")));
    ui->tb_Widget_A->setItem(2, 2, new QTableWidgetItem(QStringLiteral("最小深度")));
    ui->tb_Widget_A->setItem(0, 3, new QTableWidgetItem(toQString(depth_avg)));
    ui->tb_Widget_A->setItem(1, 3, new QTableWidgetItem(toQString(depth_max)));
    ui->tb_Widget_A->setItem(2, 3, new QTableWidgetItem(toQString(depth_min)));
    // 显示宽度信息并判断样式
    ui->labelMeanWidth->setText(toQString(width_avg));
    ui->labelMaxWidth->setText(toQString(width_max));
    ui->labelMinWidth->setText(toQString(width_min));

    ui->labelMeanDepth->setText(toQString(depth_avg));
    ui->labelMaxDepth->setText(toQString(depth_max));
    ui->labelMinDepth->setText(toQString(depth_min));

    setItemStyle(ui->tb_Widget_A->item(0, 1), InfoSuanFa.editDetectedWidthMin.toDouble() <= width_avg <= InfoSuanFa.editDetectedWidthMax.toDouble());
    setItemStyle(ui->tb_Widget_A->item(1, 1), InfoSuanFa.editDetectedWidthMin.toDouble() <= width_max <= InfoSuanFa.editDetectedWidthMax.toDouble());
    setItemStyle(ui->tb_Widget_A->item(2, 1), InfoSuanFa.editDetectedWidthMin.toDouble() <= width_min <= InfoSuanFa.editDetectedWidthMax.toDouble());
    setItemStyle(ui->tb_Widget_A->item(0, 3), InfoSuanFa.editDetectedDepthMin.toDouble() <= depth_avg <= InfoSuanFa.editDetectedDepthMax.toDouble());
    setItemStyle(ui->tb_Widget_A->item(1, 3), InfoSuanFa.editDetectedDepthMin.toDouble() <= depth_max <= InfoSuanFa.editDetectedDepthMax.toDouble());
    setItemStyle(ui->tb_Widget_A->item(2, 3), InfoSuanFa.editDetectedDepthMin.toDouble() <= depth_min <= InfoSuanFa.editDetectedDepthMax.toDouble());

    auto setStyle = [](QLabel* label, bool highlight) {
        if (highlight)
            label->setStyleSheet("color: red; font-weight: bold;");
        else
            label->setStyleSheet("color: rgb(0, 128, 0);");
        };

    // 判断并设置宽度标签样式
    setStyle(ui->labelMeanWidth, width_avg > detectedWidthValue);
    setStyle(ui->labelMaxWidth, width_max > detectedWidthValue);
    setStyle(ui->labelMinWidth, width_min > detectedWidthValue);

    // 判断并设置深度标签样式
    setStyle(ui->labelMeanDepth, depth_avg > detectedDepthValue);
    setStyle(ui->labelMaxDepth, depth_max > detectedDepthValue);
    setStyle(ui->labelMinDepth, depth_min > detectedDepthValue);

    // 先判断宽度和深度是否超阈值
    bool widthExceeded = (width_avg > detectedWidthValue);
    bool depthExceeded = (depth_avg > detectedDepthValue);

}



void PolePieceMeasureWin::slotTableSelectionChanged()
{
          auto toQString = [](double val) -> QString {
            return QString::number(val, 'f', 2) + " mm";
            };

        QModelIndexList selection = ui->tableFrame->selectionModel()->selectedRows();
        int selectionCount = selection.size();

        if ((selectionCount == 0) || (ID.size() != 1 && selectionCount == ID.size()))
        {
            //ui->statusLabel->setAnimatedText(QStringLiteral("正在浏览全局识别结果，可从表格中选择单帧浏览"));
            // 没选中任何行或全选，恢复默认显示
            updateViewer();

            // 获取宽度阈值
             float detectedWidthValue =InfoSuanFa.editDetectedWidthMin.toFloat();
 
            // 获取深度阈值
             float detectedDepthValue = InfoSuanFa.editDetectedDepthMin.toFloat();//ui->editDetectedDepthMin->text().toFloat(&okDepth);
 
            // 显示宽度信息并判断样式
            ui->labelMeanWidth->setText(toQString(InfoResult.meanWidth));
            ui->labelMaxWidth->setText(toQString(InfoResult.maxWidth));
            ui->labelMinWidth->setText(toQString(InfoResult.minWidth));

            ui->labelMeanDepth->setText(toQString(InfoResult.meanDepth));
            ui->labelMaxDepth->setText(toQString(InfoResult.maxDepth));
            ui->labelMinDepth->setText(toQString(InfoResult.minDepth));

            // 判断并设置宽度标签样式
            auto setStyle = [](QLabel* label, bool highlight) {
                if (highlight)
                    label->setStyleSheet("color: red; font-weight: bold;");
                else
                    label->setStyleSheet("color: rgb(0, 128, 0);");
                };

            setStyle(ui->labelMeanWidth, InfoResult.meanWidth > detectedWidthValue);
            setStyle(ui->labelMaxWidth, InfoResult.maxWidth > detectedWidthValue);
            setStyle(ui->labelMinWidth, InfoResult.minWidth > detectedWidthValue);

            // 判断并设置深度标签样式
            setStyle(ui->labelMeanDepth, InfoResult.meanDepth > detectedDepthValue);
            setStyle(ui->labelMaxDepth, InfoResult.maxDepth > detectedDepthValue);
            setStyle(ui->labelMinDepth, InfoResult.minDepth > detectedDepthValue);

            return;
        }

        // 多选时合成点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr show_source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_left(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_right(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr show_down(new pcl::PointCloud<pcl::PointXYZRGB>);
        QVector<double> widths, depths;

        for (const QModelIndex& idx : selection)
        {
            int row = idx.row();
            if (row < 0 || row >= ID.size()) continue; // 防止越界

            // 取第一列ID
            QTableWidgetItem* idItem = ui->tableFrame->item(row, 0);
            if (!idItem) continue;

            int idValue = idItem->text().toInt();

            // 根据idValue查找对应数据索引
            int dataIndex = -1;
            for (int i = 0; i < ID.size(); ++i)
            {
                if (ID[i] == idValue)
                {
                    dataIndex = i;
                    break;
                }
            }
            if (dataIndex == -1) continue;

            // 使用 dataIndex 访问数据而非 row
            // 合并source点云
            if (separate_clouds[ID[dataIndex]])
            {
                *show_source += *separate_clouds[ID[dataIndex]];
            }

            // 合并left点
            if (up_Points_left && dataIndex < up_Points_left->points.size())
            {
                show_left->points.push_back(up_Points_left->points[dataIndex]);
            }

            // 合并right点
            if (up_Points_right && dataIndex < up_Points_right->points.size())
            {
                show_right->points.push_back(up_Points_right->points[dataIndex]);
            }

            // 合并down点
            if (down_Points && dataIndex < down_Points->points.size())
            {
                show_down->points.push_back(down_Points->points[dataIndex]);
            }

            // 取宽度和深度改用 dataIndex
            widths.append(InfoResult.width[dataIndex]);
            depths.append(InfoResult.depth[dataIndex]);
        }

        // 设置合并后点云参数
        show_left->width = show_left->points.size();
        show_left->height = 1;
        show_left->is_dense = true;

        show_right->width = show_right->points.size();
        show_right->height = 1;
        show_right->is_dense = true;

        show_down->width = show_down->points.size();
        show_down->height = 1;
        show_down->is_dense = true;

        updateViewer(show_source, show_left, show_right, show_down);

        // 计算宽度最大最小平均
        double width_max = *std::max_element(widths.begin(), widths.end());
        double width_min = *std::min_element(widths.begin(), widths.end());
        double width_avg = std::accumulate(widths.begin(), widths.end(), 0.0) / widths.size();

        // 计算深度最大最小平均
        double depth_max = *std::max_element(depths.begin(), depths.end());
        double depth_min = *std::min_element(depths.begin(), depths.end());
        double depth_avg = std::accumulate(depths.begin(), depths.end(), 0.0) / depths.size();

        // 获取宽度阈值
         float detectedWidthValue =InfoSuanFa.editDetectedWidthMin.toFloat();
 
        // 获取深度阈值
         float detectedDepthValue = InfoSuanFa.editDetectedDepthMin.toFloat();//ui->editDetectedDepthMin->text().toFloat(&okDepth);
 
        // 显示宽度信息并判断样式
        ui->labelMeanWidth->setText(toQString(width_avg));
        ui->labelMaxWidth->setText(toQString(width_max));
        ui->labelMinWidth->setText(toQString(width_min));

        ui->labelMeanDepth->setText(toQString(depth_avg));
        ui->labelMaxDepth->setText(toQString(depth_max));
        ui->labelMinDepth->setText(toQString(depth_min));

        auto setStyle = [](QLabel* label, bool highlight) {
            if (highlight)
                label->setStyleSheet("color: red; font-weight: bold;");
            else
                label->setStyleSheet("color: rgb(0, 128, 0);");
            };

        // 判断并设置宽度标签样式
        setStyle(ui->labelMeanWidth, width_avg > detectedWidthValue);
        setStyle(ui->labelMaxWidth, width_max > detectedWidthValue);
        setStyle(ui->labelMinWidth, width_min > detectedWidthValue);

        // 判断并设置深度标签样式
        setStyle(ui->labelMeanDepth, depth_avg > detectedDepthValue);
        setStyle(ui->labelMaxDepth, depth_max > detectedDepthValue);
        setStyle(ui->labelMinDepth, depth_min > detectedDepthValue);

        // 先判断宽度和深度是否超阈值
        bool widthExceeded = (width_avg > detectedWidthValue);
        bool depthExceeded = (depth_avg > detectedDepthValue);

        //// 根据不同情况设置statusLabel内容
        //if (widthExceeded && depthExceeded) {
        //    ui->statusLabel->setAnimatedText(QStringLiteral("当前选中帧 平均宽度 和 平均深度 均超阈值"));
        //}
        //else if (widthExceeded) {
        //    ui->statusLabel->setAnimatedText(QStringLiteral("当前选中帧 平均宽度 超阈值"));
        //}
        //else if (depthExceeded) {
        //    ui->statusLabel->setAnimatedText(QStringLiteral("当前选中帧 平均深度 超阈值"));
        //}
        //else {
        //    ui->statusLabel->setAnimatedText(QStringLiteral("当前选中帧OK"));
        //}

 



}
 
PolePieceMeasureWin::~PolePieceMeasureWin()
{
    delete ui; delete ParamUi;
      delete DataExcelUi;
    delete excelDialog;
    delete ParametersDialog;
    delete m_chartView_A_Depth;
    delete m_chartView_A_Width;
    delete m_chartView_B_Depth;
    delete m_chartView_B_Width;
}
void PolePieceMeasureWin::exportToCSV()
{
    QString fileName = QFileDialog::getSaveFileName(this,
        tr("保存CSV文件"),
        "export.csv",
        tr("CSV Files (*.csv)"));

    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, tr("错误"), tr("无法创建文件"));
        return;
    }

    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream.setGenerateByteOrderMark(true); // Excel需要BOM来识别UTF-8

    // 写入表头
    QStringList headers;
    for (int col = 0; col < ui->tableFrame->columnCount(); ++col) {
        QTableWidgetItem* header = ui->tableFrame->horizontalHeaderItem(col);
        headers << (header ? header->text() : QString("Column %1").arg(col + 1));
    }
    stream << headers.join(",") << "\n";

    // 写入数据（处理逗号和引号）
    for (int row = 0; row < ui->tableFrame->rowCount(); ++row) {
        QStringList rowData;
        for (int col = 0; col < ui->tableFrame->columnCount(); ++col) {
            QTableWidgetItem* item = ui->tableFrame->item(row, col);
            QString text = item ? item->text() : "";

            // 如果包含逗号或引号，需要转义
            if (text.contains(",") || text.contains("\"") || text.contains("\n")) {
                text.replace("\"", "\"\""); // 双引号转义
                text = QString("\"%1\"").arg(text);
            }
            rowData << text;
        }
        stream << rowData.join(",") << "\n";
    }

    file.close();
    QMessageBox::information(this, tr("成功"), tr("导出成功！可用Excel打开"));
}
