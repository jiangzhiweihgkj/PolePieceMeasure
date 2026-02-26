
#ifndef  CALLONETIMES_H
#define  CALLONETIMES_H
#include <windows.h>
#include <QMessageBox.h>
#include <QDateTime.h>
#include <QFileDialog>

#include <QTcpSocket>
#include <QTimer>
#include <QtEndian>   // Qt5/Qt6 通用
#include "hgpcl_trans.h" 
#include <QThread>
#include <QObject>
  //*******pcl
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>//统计滤波
#include <pcl/filters/radius_outlier_removal.h>//半径滤波
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/ply_io.h>
 #include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h> 
#include <pcl/surface/mls.h>
//*******pcl
 #include <kApi/kApi.h>
#include <GoSdk/GoSdk.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory.h>

#include "hgHMI.h"//图形化显示

#define RECEIVE_TIMEOUT       (1000) // (20000000) 1S
#define INVALID_RANGE_16BIT     ((signed short)0x8000)          // gocator transmits range data as 16-bit signed integers. 0x8000 signifies invalid range data. 
#define DOUBLE_MAX              ((k64f)1.7976931348623157e+308) // 64-bit double - largest positive value.  
#define INVALID_RANGE_DOUBLE    ((k64f)-DOUBLE_MAX)             // floating point value to represent invalid range data.    
#define FOLAT_MIN                 3.4e-38 

#define NM_TO_MM(VALUE) (((k64f)(VALUE))/1000000.0)
#define UM_TO_MM(VALUE) (((k64f)(VALUE))/1000.0)
class CallOneTimes : public QThread
{

    Q_OBJECT


  public:
    CallOneTimes(void);
    ~CallOneTimes(void);
    void CreateWorkDir(QString mainDir);
    QString _callOneTimesWork = QStringLiteral("");
    QString _callOneTimesMainWork = QStringLiteral("");
    bool success = true;//处理结果失败
    bool success_B = true;//处理结果失败

     int ThreadStatus=-1;
      enum SysState
     {
         SYS_NOWORK = -1,
         SYS_CAMERA_START_COLLECT=0,//开始采集
         SYS_CAMERA_STOP_COLLECT,//停止采集 
         SYS_DEAL_CLOUDS,//点云处理
         SYS_CALIBRATE_SAVE,//保存点云
         SYS_GETPARAM,//获取参数
      };
Q_SIGNALS:
    void onsendStatus(QString);
    void onsendCloudDeal();

protected:
    void run() override;
public:
    typedef struct DataContext
    {
        k32u count;
    } DataContext;

    QString SavePath;
    DataContext contextPointer;
    pcl::PCLPointCloud2 cloud2_save;
    double cloudshowdis = 0.0;
 
#pragma region 相机采集
public: 
    typedef struct ProfilePoint
    {
        double x;   // x-coordinate in engineering units (mm) - position along laser line
        double y;
        double z;   // z-coordinate in engineering units (mm) - height (at the given x position)
        unsigned char intensity;
    } ProfilePoint;
    unsigned int i, j, k, arrayIndex;

    kStatus status;
    kAssembly api = kNULL;
    GoSystem system = kNULL;
    GoSensor sensor = kNULL;
    // char* backupFile = "SystemBackup.bin";
    kIpAddress ipAddress;
    GoSetup setup = kNULL;
    k32u profilePointCount;
    ProfilePoint* profileBuffer = NULL;
 private:
 
#pragma endregion  
#pragma region 相机采集
    //相机连接
    
public: 
    int CameraConnectFlag = -1;//相机连接标志
    void* handle ;   
    pcl::PointCloud<pcl::PointXYZ>::Ptr  SaveAcquireImage();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudsResultDeal;
 
private:
    static CallOneTimes* DataCallModel;     //静态变量--用于回调函数里调用非静态变量
    int nFrameNum = 0, nHeight = 0, nWidth = 0, nDataLen = 0, fXScale;
    uint8_t* pData;
#pragma endregion
#pragma region 图形化显示
public:
    struct CheckResultInfo { 
        int  Number_CameraCaptures = 0;//相机采集次数
        int  Number_Check_Result_OK = 0;//综合OK次数
        int  Number_Check_Result_NG = 0;//综合NG次数
        int  Number_Check_Result_ALL = 0;//总次数
        int  Number_Depth_Result_NG = 0;//深度NG次数
        int  Number_Width_Result_NG = 0;//宽度NG次数
        double  Measure_Depth_Result = 0.0;//深度结果
        double  Measure_Width_Result = 0.0;//宽度结果
        double  Yield_Check_Result_OK = 0;//良品率
        QString Depth_Model =QStringLiteral("10-20");//10-20 深度规格
        QString Width_Model =QStringLiteral("85-120");//85-120
        QString Judgment_Result =QStringLiteral("OK");//85-120
        double Depth_specification_Max = 0.0;//深度规格最大值
        double Depth_specification_Min = 0.0;//深度规格最小值
        double Width_specification_Max = 0.0;//宽度规格最大值
        double Width_specification_Min = 0.0;//宽度规格最小值
     };
    struct 
    {
        CheckResultInfo Result_A;
        CheckResultInfo Result_B;
    }class_A_B;
   
//private Q_SLOTS:

#pragma endregion

#pragma region 欧姆龙通信
Q_SIGNALS:
    void valueChanged(QVector<quint16> val);   // 把最新值抛回 GUI
public:
    bool connectPLC(const QString& ip, quint16 port = 9600);
    bool disconnectPLC();
    bool recvExact(int bytesNeeded);
    /* 读 DM 区，返回 true 表示收到有效帧 */
    bool readDM(int start, int words, QByteArray& data);
    /* 写 DM 区 */
    bool writeDM(int start, const QVector<quint16>& values);
    QTimer*  m_timer;
 private:
    QByteArray makeHeader(int finsLen);
    QByteArray makeReadFrame(int start, int words);
    QByteArray makeWriteFrame(int start, const QVector<quint16>& values);
    bool waitForFrame(QByteArray& frame);

    QTcpSocket* m_sock = nullptr;
    quint8 m_pcNode = 0x0A;   // PC 站号，随意
    quint8 m_plcNode = 0x01;  // PLC 站号，CX-Integrator 里能看到
private slots:
    void onTimeout();
#pragma endregion
};


#pragma endregion
 
#endif // CALLONETIMES_H

