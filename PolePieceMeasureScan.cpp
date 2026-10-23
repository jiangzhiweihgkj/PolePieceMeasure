#include "PolePieceMeasureWin.h"

namespace
{
template <typename CameraType>
bool connectCameraBlocking(CameraType* camera, const QString& sensorIp, QString& errorMessage)
{
    if (camera == nullptr)
    {
        errorMessage = QStringLiteral("相机对象未初始化");
        return false;
    }
    if (sensorIp.trimmed().isEmpty())
    {
        errorMessage = QStringLiteral("相机IP不能为空");
        return false;
    }
    if (camera->CameraConnectFlag == kOK
        && camera->sensor != kNULL
        && GoSensor_IsConnected(camera->sensor))
    {
        return true;
    }
    if ((camera->status = GoSdk_Construct(&camera->api)) != kOK)
    {
        errorMessage = QStringLiteral("初始化Gocator SDK失败");
        return false;
    }
    if ((camera->status = GoSystem_Construct(&camera->system, kNULL)) != kOK)
    {
        errorMessage = QStringLiteral("创建Gocator系统对象失败");
        return false;
    }

    kIpAddress ipAddress;
    if (kIpAddress_Parse(&ipAddress, sensorIp.toLocal8Bit().constData()) != kOK)
    {
        errorMessage = QStringLiteral("相机IP格式错误");
        return false;
    }
    if ((camera->status = GoSystem_FindSensorByIpAddress(camera->system, &ipAddress, &camera->sensor)) != kOK)
    {
        errorMessage = QStringLiteral("未找到相机");
        return false;
    }
    if ((camera->status = GoSensor_Connect(camera->sensor)) != kOK)
    {
        errorMessage = QStringLiteral("相机连接失败");
        return false;
    }

    camera->CameraConnectFlag = kOK;
    if ((camera->status = GoSystem_EnableData(camera->system, kTRUE)) != kOK)
    {
        GoSensor_Disconnect(camera->sensor);
        camera->sensor = kNULL;
        camera->CameraConnectFlag = kOK - 1;
        errorMessage = QStringLiteral("启用相机数据通道失败");
        return false;
    }
    return true;
}

template <typename CameraType>
bool disconnectCameraBlocking(CameraType* camera, QString& errorMessage)
{
    if (camera == nullptr)
    {
        errorMessage = QStringLiteral("相机对象未初始化");
        return false;
    }
    if (camera->sensor == kNULL)
    {
        camera->CameraConnectFlag = kOK - 1;
        return true;
    }
    if ((camera->status = GoSensor_Disconnect(camera->sensor)) != kOK)
    {
        errorMessage = QStringLiteral("相机断开连接失败");
        return false;
    }
    camera->sensor = kNULL;
    camera->CameraConnectFlag = kOK - 1;
    return true;
}
}

void  PolePieceMeasureWin::InitScanTab()
{
    QSettings ini(iniPath, QSettings::IniFormat);
    if (plc == nullptr)
        plc = new Communication();

    if (_callOneTimes == nullptr)
        _callOneTimes = new CallOneTimes();
    _callOneTimes->start();
    _callOneTimes->ThreadStatus = CallOneTimes::SYS_NOWORK;
    if (_callOneTimes_B == nullptr)
        _callOneTimes_B = new CallOneTimes_B();
    _callOneTimes_B->start();
    _callOneTimes_B->ThreadStatus = CallOneTimes::SYS_NOWORK;
    connect(_callOneTimes, SIGNAL(onsendStatus(QString)), this, SLOT(onReceiveStatus(QString)));
    connect(_callOneTimes_B, SIGNAL(onsendStatus(QString)), this, SLOT(onReceiveStatus_B(QString)));
    connect(plc, &Communication::sendCommunicateMessage, this, &PolePieceMeasureWin::receiveCommunicateMessage, Qt::UniqueConnection);
    connect(plc, &Communication::plcSignalsUpdated, this, &PolePieceMeasureWin::onPlcSignalsUpdated, Qt::UniqueConnection);
    
}
void PolePieceMeasureWin::DestroyScanTab()
{
    QSettings ini(iniPath, QSettings::IniFormat);
    //ini.setValue(QStringLiteral("scan/editOutPath"), ui->editOutPath->text());

}

void PolePieceMeasureWin::startCameraTask(bool isCameraA, bool isConnect)
{
    bool& taskRunning = isCameraA ? cameraTaskRunningA : cameraTaskRunningB;
    if (taskRunning)
    {
        return;
    }
    taskRunning = true;

    if (ParamUi != nullptr)
    {
        if (isCameraA)
        {
            ParamUi->btnConnectCamera->setEnabled(false);
            ParamUi->btnDisConnectCamera->setEnabled(false);
            if (isConnect)
            {
                ParamUi->CameraConnectState->setText(QStringLiteral("connecting..."));
            }
        }
        else
        {
            ParamUi->btnConnectCamera_B->setEnabled(false);
            ParamUi->btnDisConnectCamera_B->setEnabled(false);
            if (isConnect)
            {
                ParamUi->CameraConnectState_B->setText(QStringLiteral("connecting..."));
            }
        }
    }

    const QString cameraName = isCameraA ? QStringLiteral("A") : QStringLiteral("B");
    RecordData(cameraName + (isConnect ? QStringLiteral("相机连接中...") : QStringLiteral("相机断开中...")));

    if (isConnect)
    {
        if (isCameraA && _callOneTimes != nullptr)
        {
            connect(_callOneTimes, SIGNAL(onsendCloudDeal()), this, SLOT(onslotReceiveCloud()), Qt::UniqueConnection);
        }
        else if (!isCameraA && _callOneTimes_B != nullptr)
        {
            connect(_callOneTimes_B, SIGNAL(onsendCloudDeal()), this, SLOT(onslotReceiveCloud_B()), Qt::UniqueConnection);
        }
    }

    const QString sensorIp = isCameraA ? InfoCamera.Camra_A_IP : InfoCamera.Camra_B_IP;
    CallOneTimes* cameraA = _callOneTimes;
    CallOneTimes_B* cameraB = _callOneTimes_B;

    QThread* taskThread = QThread::create([this, isCameraA, isConnect, sensorIp, cameraA, cameraB]() {
        QString errorMessage;
        bool success = false;

        if (isCameraA)
        {
            if (cameraA == nullptr)
            {
                errorMessage = QStringLiteral("相机对象未初始化");
            }
            else if (isConnect)
            {
                success = connectCameraBlocking(cameraA, sensorIp, errorMessage);
            }
            else
            {
                success = disconnectCameraBlocking(cameraA, errorMessage);
            }
        }
        else
        {
            if (cameraB == nullptr)
            {
                errorMessage = QStringLiteral("相机对象未初始化");
            }
            else if (isConnect)
            {
                success = connectCameraBlocking(cameraB, sensorIp, errorMessage);
            }
            else
            {
                success = disconnectCameraBlocking(cameraB, errorMessage);
            }
        }

        QMetaObject::invokeMethod(this, [this, isCameraA, isConnect, success, errorMessage]() {
            finishCameraTask(isCameraA, isConnect, success, errorMessage);
        }, Qt::QueuedConnection);
    });

    if (taskThread == nullptr)
    {
        taskRunning = false;
        finishCameraTask(isCameraA, isConnect, false, QStringLiteral("无法创建相机操作线程"));
        return;
    }

    cameraTaskThreads.append(taskThread);
    connect(taskThread, &QThread::finished, this, [this, taskThread]() {
        cameraTaskThreads.removeOne(taskThread);
        taskThread->deleteLater();
    });
    taskThread->start();
}

void PolePieceMeasureWin::finishCameraTask(bool isCameraA, bool isConnect, bool success, const QString& errorMessage)
{
    if (isCameraA)
    {
        cameraTaskRunningA = false;
    }
    else
    {
        cameraTaskRunningB = false;
    }

    const QString cameraName = isCameraA ? QStringLiteral("A") : QStringLiteral("B");
    QLabel* stateLabel = nullptr;
    QPushButton* btnConnect = nullptr;
    QPushButton* btnDisconnect = nullptr;
    hgHMI_labelLED* cameraLed = isCameraA ? labelledCAM_A : labelledCAM_B;

    if (ParamUi != nullptr)
    {
        stateLabel = isCameraA ? ParamUi->CameraConnectState : ParamUi->CameraConnectState_B;
        btnConnect = isCameraA ? ParamUi->btnConnectCamera : ParamUi->btnConnectCamera_B;
        btnDisconnect = isCameraA ? ParamUi->btnDisConnectCamera : ParamUi->btnDisConnectCamera_B;
        if (btnConnect != nullptr)
        {
            btnConnect->setEnabled(true);
        }
        if (btnDisconnect != nullptr)
        {
            btnDisconnect->setEnabled(true);
        }
    }

    if (success)
    {
        const QString state = isConnect ? QStringLiteral("connected") : QStringLiteral("unconnected");
        if (stateLabel != nullptr)
        {
            stateLabel->setText(state);
        }
        if (isCameraA)
        {
            InfoCamera.CameraConnectState_A = state;
        }
        else
        {
            InfoCamera.CameraConnectState_B = state;
        }
        if (cameraLed != nullptr)
        {
            cameraLed->setLedOn(isConnect);
        }
        RecordData(cameraName + (isConnect ? QStringLiteral("相机连接成功") : QStringLiteral("相机断开成功")));
        return;
    }

    if (isConnect)
    {
        const QString state = QStringLiteral("unconnected");
        if (stateLabel != nullptr)
        {
            stateLabel->setText(state);
        }
        if (isCameraA)
        {
            InfoCamera.CameraConnectState_A = state;
        }
        else
        {
            InfoCamera.CameraConnectState_B = state;
        }
        if (cameraLed != nullptr)
        {
            cameraLed->setLedOn(false);
        }
        RecordData(cameraName + QStringLiteral("相机连接失败：") + errorMessage, 2);
        QMessageBox::warning(this, QStringLiteral("Tips"), cameraName + QStringLiteral("相机连接失败\n") + errorMessage);
    }
    else
    {
        RecordData(cameraName + QStringLiteral("相机断开连接失败：") + errorMessage, 2);
        QMessageBox::warning(this, QStringLiteral("Tips"), cameraName + QStringLiteral("相机断开连接失败\n") + errorMessage);
    }
}

void PolePieceMeasureWin::on_btnBrowse_A_clicked()//浏览A
{
    QString  folderPath = QFileDialog::getExistingDirectory(this, tr("Select Folder"),
        "", QFileDialog::ShowDirsOnly
        | QFileDialog::DontUseNativeDialog);
    if (!folderPath.isNull()) {
        DataExcelUi->editPath_A->setText(folderPath);
       // _callOneTimes->SavePath = folderPath;
        InfoExcel.excel_A_OutPath = folderPath;

    }
    else {
        //点的是取消
        return;
    }

}
void PolePieceMeasureWin::on_btnBrowse_B_clicked()//浏览B
{
    QString  folderPath = QFileDialog::getExistingDirectory(this, tr("Select Folder"),
        "", QFileDialog::ShowDirsOnly
        | QFileDialog::DontUseNativeDialog);
    if (!folderPath.isNull()) {
        DataExcelUi->editPath_B->setText(folderPath);
        // _callOneTimes->SavePath = folderPath;
        InfoExcel.excel_B_OutPath = folderPath;

    }
    else {
        //点的是取消
        return;
    }
}
void PolePieceMeasureWin::on_btnInput_A_clicked()//导出A  
{
    QString fileName = DataExcelUi->editPath_A->text() + QStringLiteral("/A.csv");
    /*QString fileName = QFileDialog::getSaveFileName(this,
        tr("保存CSV文件"),
        "export.csv",
        tr("CSV Files (*.csv)"));*/

    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, QStringLiteral("错误"), QStringLiteral("无法创建文件"));
        return;
    }

    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream.setGenerateByteOrderMark(true); // Excel需要BOM来识别UTF-8

    // 写入表头
    QStringList headers;
    for (int col = 0; col < DataExcelUi->tableWidgetExcel_A->columnCount(); ++col) {
        QTableWidgetItem* header = DataExcelUi->tableWidgetExcel_A->horizontalHeaderItem(col);
        headers << (header ? header->text() : QString("Column %1").arg(col + 1));
    }
    stream << headers.join(",") << "\n";

    // 写入数据（处理逗号和引号）
    for (int row = 0; row < DataExcelUi->tableWidgetExcel_A->rowCount(); ++row) {
        QStringList rowData;
        for (int col = 0; col < DataExcelUi->tableWidgetExcel_A->columnCount(); ++col) {
            QTableWidgetItem* item = DataExcelUi->tableWidgetExcel_A->item(row, col);
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
    QMessageBox::information(this, QStringLiteral("成功"), QStringLiteral("导出A面数据成功！可用Excel打开"));
}
void PolePieceMeasureWin::on_btnInput_B_clicked()//导出B
{
    QString fileName = DataExcelUi->editPath_B->text() + QStringLiteral("/B.csv"); 
    if (fileName.isEmpty()) return;

    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, QStringLiteral("错误"), QStringLiteral("无法创建文件"));
        return;
    }

    QTextStream stream(&file);
    stream.setCodec("UTF-8");
    stream.setGenerateByteOrderMark(true); // Excel需要BOM来识别UTF-8

    // 写入表头
    QStringList headers;
    for (int col = 0; col < DataExcelUi->tableWidgetExcel_B->columnCount(); ++col) {
        QTableWidgetItem* header = DataExcelUi->tableWidgetExcel_B->horizontalHeaderItem(col);
        headers << (header ? header->text() : QString("Column %1").arg(col + 1));
    }
    stream << headers.join(",") << "\n";

    // 写入数据（处理逗号和引号）
    for (int row = 0; row < DataExcelUi->tableWidgetExcel_B->rowCount(); ++row) {
        QStringList rowData;
        for (int col = 0; col < DataExcelUi->tableWidgetExcel_B->columnCount(); ++col) {
            QTableWidgetItem* item = DataExcelUi->tableWidgetExcel_B->item(row, col);
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
    QMessageBox::information(this, QStringLiteral("成功"), QStringLiteral("导出B面数据成功！可用Excel打开"));
}

void PolePieceMeasureWin::on_btnOutPutDirectory_Result_clicked()//
{
    QString  folderPath = QFileDialog::getExistingDirectory(this, tr("Select Folder"),
        "", QFileDialog::ShowDirsOnly
        | QFileDialog::DontUseNativeDialog);
    if (!folderPath.isNull()) {
        ParamUi->editOutPath_Result->setText(folderPath);
        _callOneTimes_B->SavePath = folderPath;
        InfoCamera.PointOutPath_B = folderPath;

    }
    else {
        //点的是取消
        return;
    }
}
void PolePieceMeasureWin::on_btnOutPutDirectory_clicked()//
{
    QString  folderPath = QFileDialog::getExistingDirectory(this, tr("Select Folder"),
        "", QFileDialog::ShowDirsOnly
        | QFileDialog::DontUseNativeDialog);
    if (!folderPath.isNull()) {
         ParamUi->editOutPath->setText(folderPath);
         _callOneTimes->SavePath = folderPath;
         InfoCamera.PointOutPath = folderPath;
         
    }
    else {
        //点的是取消
        return;
    }
    
}
 
void  PolePieceMeasureWin::onReceiveStatus(QString content)
{
    RecordData((QStringLiteral("A:")+content),1);
}
void PolePieceMeasureWin::on_btnStartCollectPoint_clicked()//开始采集
{
    //QString path = CreateOkNgTimeDir(ParamUi->editOutPath->text(), true);//创建 年月日/OK|NG/时刻 目录


    //return;
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
    _callOneTimes->ThreadStatus = CallOneTimes::SYS_CAMERA_START_COLLECT;
    _callOneTimes_B->ThreadStatus = CallOneTimes_B::SYS_CAMERA_START_COLLECT;
    //CreateWorkDir(ParamUi->editOutPath->text());
    _callOneTimes->_callOneTimesMainWork = ParamUi->editOutPath->text();
    _callOneTimes_B->_callOneTimesMainWork = ParamUi->editOutPath_Result->text();
}
void PolePieceMeasureWin::CreateWorkDir(QString mainDir)
{
    QDateTime date = QDateTime::currentDateTime();
    //QString strWorkName = QStringLiteral("CL2407221123");//test
    QString strWorkName = QStringLiteral("HG") + date.toString("yyMMddHHmm");
    mainWorkDir = mainDir + QStringLiteral("/") + strWorkName;
    QString pointDir = mainWorkDir + QStringLiteral("/") + pointDirName;
    QDir dir;//创建空目录，如果已存在，则删除后创建
    if (!dir.exists(mainWorkDir)) {
        dir.mkpath(mainWorkDir);
    }
    else {
        if (!dir.exists(mainWorkDir))
            dir.mkpath(mainWorkDir);
    }
    dir.mkpath(pointDir);
    _callOneTimes->SavePath = pointDir;
}
QString PolePieceMeasureWin::CreateOkNgTimeDir(const QString& mainPath,bool isOK)
{
    QDir dir;
    if (!dir.exists(mainPath)) {
        dir.mkpath(mainPath);//主文件夹不存在则创建
    }
    QDateTime now = QDateTime::currentDateTime();
    QString dateDir = mainPath + QStringLiteral("/") + now.toString(QStringLiteral("yyyy-MM-dd"));//大文件夹：年月日
    if (!dir.exists(dateDir)) {
        dir.mkpath(dateDir);//大文件夹不存在则创建
    }
    QString timeName = now.toString(QStringLiteral("HHmmss"));//小文件夹：当前时刻
    QString fullPath;
    if(isOK)
    {
        fullPath = dateDir + QStringLiteral("/OK/") + timeName;
        dir.mkpath(fullPath);
    }
    else
    {
        fullPath = dateDir + QStringLiteral("/NG/") + timeName;
        dir.mkpath(fullPath);
    }
    return fullPath;
}
void PolePieceMeasureWin::deleteOldDateDir(const QString& mainPath, int daysToKeep)
{
    QDir dir(mainPath);
    if (!dir.exists()) return;

    // 设置过滤器：只匹配 yyyy-MM-dd 的年月日文件夹
    dir.setNameFilters(QStringList() << "[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]");
    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    QDate cutoffDate = QDate::currentDate().addDays(-daysToKeep);//保存天数范围外的截止日期
    QRegularExpression re("^(\\d{4})-(\\d{2})-(\\d{2})$");

    foreach(QString folderName, dir.entryList()) {
        QRegularExpressionMatch match = re.match(folderName);
        if (!match.hasMatch()) continue;

        // 解析文件夹名中的年月日
        QDate folderDate(match.captured(1).toInt(), match.captured(2).toInt(), match.captured(3).toInt());
        if (!folderDate.isValid()) continue;

        if (folderDate < cutoffDate) {
            QDir oldDir(dir.absoluteFilePath(folderName));
            oldDir.removeRecursively();//删除超过保存天数的文件夹
        }
    }
}
void PolePieceMeasureWin::deleteOldFolders(const QString& basePath, int daysToKeep)
{
    QDir dir(basePath);
    if (!dir.exists()) return;

    // 设置过滤器：只匹配 SKY + 10位数字 的文件夹
    dir.setNameFilters(QStringList() << "SKY[0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9][0-9]");
    dir.setFilter(QDir::Dirs | QDir::NoDotAndDotDot);

    QDateTime now = QDateTime::currentDateTime();
    QDateTime cutoffDate = now.addDays(-daysToKeep);

    // 正则解析：SKY(yy)(MM)(dd)(HH)(mm)
    QRegularExpression re("^SKY(\\d{2})(\\d{2})(\\d{2})(\\d{2})(\\d{2})$");

    foreach(QString folderName, dir.entryList()) {
        QRegularExpressionMatch match = re.match(folderName);
        if (!match.hasMatch()) continue;

        // 提取各组成部分
        int year = match.captured(1).toInt();
        int month = match.captured(2).toInt();
        int day = match.captured(3).toInt();
        int hour = match.captured(4).toInt();
        int minute = match.captured(5).toInt();

        // 处理年份：25 -> 2025（假设50以下为2000+）
        year = (year < 50) ? (2000 + year) : (1900 + year);

        // 构建日期对象
        QDateTime folderDate(QDate(year, month, day), QTime(hour, minute));
        if (!folderDate.isValid()) {
            // qDebug() << "无效日期格式:" << folderName;
            continue;
        }

        // 判断是否超过10天
        if (folderDate < cutoffDate) {
            QString fullPath = dir.absoluteFilePath(folderName);
            QDir oldDir(fullPath);

            if (oldDir.removeRecursively()) {
                // qDebug() << "已删除:" << folderName
                   //  << "(创建于:" << folderDate.toString("yyyy-MM-dd hh:mm") << ")";
               // updateRecord(QStringLiteral("已删除旧文件夹::%1(创建于%2)").arg(folderName).arg(folderDate.toString("yyyy-MM-dd hh:mm")));

            }
            else {
                //qDebug() << "删除失败:" << folderName;
               // updateRecord(QStringLiteral("❌ 删除失败:%1").arg(folderName));
            }
        }
    }
}

void PolePieceMeasureWin::on_btnStopCollectPoint_clicked()//停止采集
{

    //deleteOldDateDir(ParamUi->editOutPath->text(), 10);

    //return;
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
void PolePieceMeasureWin::on_btnCameraB_clicked()//处理
{
    on_btnConnectCamera_B_clicked();
}
void PolePieceMeasureWin::on_btnCameraA_clicked()//处理
{
    on_btnConnectCamera_clicked();
}
void PolePieceMeasureWin::on_btnSavePoint_clicked()//先暂时换成信息导入的情况
{
   
    //使用堆分配对话框和 UI，避免栈对象生命周期问题
    //excelDialog = new QDialog(this);//#include "ui_DataExcel.h" 
    //DataExcelUi = new Ui::DataExcelClass();
    //DataExcelUi->setupUi(excelDialog);
    //基础配置
    excelDialog->setWindowTitle("excel");
    excelDialog->setWindowFlags(excelDialog->windowFlags() & ~Qt::WindowContextHelpButtonHint);
    //执行对话框，结束后自动释放内存
   
    connect(DataExcelUi->btnInput_A, &QPushButton::clicked, this,&PolePieceMeasureWin::on_btnInput_A_clicked, Qt::UniqueConnection);//导入A
    connect(DataExcelUi->btnBrowse_A, &QPushButton::clicked, this, &PolePieceMeasureWin::on_btnBrowse_A_clicked, Qt::UniqueConnection);//浏览A

    connect(DataExcelUi->btnInput_B, &QPushButton::clicked, this, &PolePieceMeasureWin::on_btnInput_B_clicked, Qt::UniqueConnection);//导入B

    connect(DataExcelUi->btnBrowse_B, &QPushButton::clicked, this, &PolePieceMeasureWin::on_btnBrowse_B_clicked, Qt::UniqueConnection);
         
    //执行对话框，结束后自动释放内存
    if (excelDialog->exec() == QDialog::Accepted) {
        // 可扩展登录成功后的逻辑

    }
  //  delete DataExcelUi;
  //  delete excelDialog; 
}
void PolePieceMeasureWin::on_btnConnectCamera_B_clicked()//连接轮廓仪B
{
    startCameraTask(false, true);
    return;

    GoDataSet dataset = kNULL;

    if (GoSystem_ReceiveData(_callOneTimes_B->system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {
        CallOneTimes_B::ProfilePoint** surfaceBuffer = NULL;
        k32u surfaceBufferHeight = 0;

        printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));
        for (size_t i = 0; i < GoDataSet_Count(dataset); ++i)
        {
            GoDataMsg dataObj = GoDataSet_At(dataset, i);
            switch (GoDataMsg_Type(dataObj))
            {
            case GO_DATA_MESSAGE_TYPE_STAMP:
            {
                GoStampMsg stampMsg = dataObj;
                printf("  Stamp Message batch count: %u\n\n", (k32u)GoStampMsg_Count(stampMsg));

                for (int j = 0; j < GoStampMsg_Count(stampMsg); j++)
                {
                    GoStamp* stamp = GoStampMsg_At(stampMsg, j);
                    printf("  Timestamp: %llu\n", stamp->timestamp);
                    printf("  Encoder position at leading edge: %lld\n", stamp->encoder);
                    printf("  Frame index: %llu\n", stamp->frameIndex);
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
            {
                GoSurfaceMsg surfaceMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
                double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
                double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
                double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
                double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
                double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

                printf("  Surface data width: %lu\n", (k32u)GoSurfaceMsg_Width(surfaceMsg));
                printf("  Surface data length: %lu\n", (k32u)GoSurfaceMsg_Length(surfaceMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<CallOneTimes_B::ProfilePoint**>(malloc(GoSurfaceMsg_Length(surfaceMsg) * sizeof(CallOneTimes_B::ProfilePoint*)));

                    for (size_t j = 0; j < GoSurfaceMsg_Length(surfaceMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<CallOneTimes_B::ProfilePoint*>(malloc(GoSurfaceMsg_Width(surfaceMsg) * sizeof(CallOneTimes_B::ProfilePoint)));
                    }

                    surfaceBufferHeight = (k32u)GoSurfaceMsg_Length(surfaceMsg);
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr collectCloudPoint(new pcl::PointCloud<pcl::PointXYZ>);

                for (rowIdx = 0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++)
                {
                    k16s* data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

                    for (colIdx = 0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++)
                    {
                        // gocator transmits range data as 16-bit signed integers
                        // to translate 16-bit range data to engineering units, the calculation for each point is: 
                        //          X: XOffset + columnIndex * XResolution 
                        //          Y: YOffset + rowIndex * YResolution
                        //          Z: ZOffset + height_map[rowIndex][columnIndex] * ZResolution

                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;


                        if (data[colIdx] != INVALID_RANGE_16BIT)
                        {
                            surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx];
                        }
                        else
                        {
                            surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                        }
                        pcl::PointXYZ point;
                        point.x = static_cast<float>(colIdx * XResolution);
                        point.y = static_cast<float>(rowIdx * YResolution);
                        point.z = static_cast<float>(surfaceBuffer[rowIdx][colIdx].z);
                        collectCloudPoint->push_back(point);
                    }
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
                clouds = collectCloudPoint;
                pcl::PCLPointCloud2 cloud2;
                // pcl::toPCLPointCloud2(*cloud, cloud2);
                pcl::toPCLPointCloud2(*clouds, cloud2);
                QString filePath = "D:/project/PolePieceMeasure/PolePieceMeasureWin/x64/Release/ply/20251118.ply";
                int ret = pcl::io::savePLYFileBinary(filePath.toStdString(), *clouds);
                // int result = pcl::io::savePLYFile("D:/project/PolePieceMeasure/PolePieceMeasureWin/x64/Release/ply/20251118.ply", cloud2, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false, false);

            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD:
            {
                GoSurfacePointCloudMsg surfacePointCloudMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfacePointCloudMsg_XResolution(surfacePointCloudMsg));
                double YResolution = NM_TO_MM(GoSurfacePointCloudMsg_YResolution(surfacePointCloudMsg));
                double ZResolution = NM_TO_MM(GoSurfacePointCloudMsg_ZResolution(surfacePointCloudMsg));
                double XOffset = UM_TO_MM(GoSurfacePointCloudMsg_XOffset(surfacePointCloudMsg));
                double YOffset = UM_TO_MM(GoSurfacePointCloudMsg_YOffset(surfacePointCloudMsg));
                double ZOffset = UM_TO_MM(GoSurfacePointCloudMsg_ZOffset(surfacePointCloudMsg));

                printf("  Surface Point Cloud data width: %lu\n", (k32u)GoSurfacePointCloudMsg_Width(surfacePointCloudMsg));
                printf("  Surface Point Cloud data length: %lu\n", (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<CallOneTimes_B::ProfilePoint**> (malloc(GoSurfacePointCloudMsg_Length(surfacePointCloudMsg) * sizeof(CallOneTimes_B::ProfilePoint*)));

                    for (size_t j = 0; j < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<CallOneTimes_B::ProfilePoint*>(malloc(GoSurfacePointCloudMsg_Width(surfacePointCloudMsg) * sizeof(CallOneTimes_B::ProfilePoint)));
                    }

                    surfaceBufferHeight = (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg);
                }

                for (rowIdx = 0; rowIdx < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); rowIdx++)
                {
                    kPoint3d16s* data = GoSurfacePointCloudMsg_RowAt(surfacePointCloudMsg, rowIdx);

                    for (colIdx = 0; colIdx < GoSurfacePointCloudMsg_Width(surfacePointCloudMsg); colIdx++)
                    {
                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * data[colIdx].x;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * data[colIdx].y;

                        if (data[colIdx].z != INVALID_RANGE_16BIT)
                        {
                            surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx].z;
                        }
                        else
                        {
                            surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                        }
                    }
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY:
            {
                GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
                unsigned int rowIdx, colIdx;
                double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
                double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
                double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
                double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

                printf("  Surface intensity width: %lu\n", (k32u)GoSurfaceIntensityMsg_Width(surfaceIntMsg));
                printf("  Surface intensity height: %lu\n", (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<CallOneTimes_B::ProfilePoint**> (malloc(GoSurfaceIntensityMsg_Length(surfaceIntMsg) * sizeof(CallOneTimes_B::ProfilePoint*)));
                    //                        surfaceBuffer[j] = static_cast<CallOneTimes::ProfilePoint*> (malloc(GoSurfaceMsg_Width(surfaceMsg) * sizeof(CallOneTimes::ProfilePoint)));

                    for (size_t j = 0; j < GoSurfaceIntensityMsg_Length(surfaceIntMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<CallOneTimes_B::ProfilePoint*>(malloc(GoSurfaceIntensityMsg_Width(surfaceIntMsg) * sizeof(CallOneTimes_B::ProfilePoint)));
                    }

                    surfaceBufferHeight = (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg);
                }

                for (rowIdx = 0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++)
                {
                    k8u* data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

                    // gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
                    for (colIdx = 0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg); colIdx++)
                    {
                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                        surfaceBuffer[rowIdx][colIdx].intensity = data[colIdx];
                    }

                }
            }
            break;

            }
        }
    }


}
void PolePieceMeasureWin::on_btnDisConnectCamera_B_clicked()//断开轮廓仪B
{
    startCameraTask(false, false);
}
void PolePieceMeasureWin::on_btnDisConnectCamera_clicked()
{
    startCameraTask(true, false);
}
void PolePieceMeasureWin::on_btnConnectCamera_clicked()
{
    startCameraTask(true, true);
    return;

    GoDataSet dataset = kNULL;

    if (GoSystem_ReceiveData(_callOneTimes->system, &dataset, RECEIVE_TIMEOUT) == kOK)
    {
        CallOneTimes::ProfilePoint** surfaceBuffer = NULL;
        k32u surfaceBufferHeight = 0;

        printf("Dataset count: %u\n", (k32u)GoDataSet_Count(dataset));
        for (size_t i = 0; i < GoDataSet_Count(dataset); ++i)
        {
            GoDataMsg dataObj = GoDataSet_At(dataset, i);
            switch (GoDataMsg_Type(dataObj))
            {
            case GO_DATA_MESSAGE_TYPE_STAMP:
            {
                GoStampMsg stampMsg = dataObj;
                printf("  Stamp Message batch count: %u\n\n", (k32u)GoStampMsg_Count(stampMsg));

                for (int j = 0; j < GoStampMsg_Count(stampMsg); j++)
                {
                    GoStamp* stamp = GoStampMsg_At(stampMsg, j);
                    printf("  Timestamp: %llu\n", stamp->timestamp);
                    printf("  Encoder position at leading edge: %lld\n", stamp->encoder);
                    printf("  Frame index: %llu\n", stamp->frameIndex);
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_UNIFORM_SURFACE:
            {
                GoSurfaceMsg surfaceMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfaceMsg_XResolution(surfaceMsg));
                double YResolution = NM_TO_MM(GoSurfaceMsg_YResolution(surfaceMsg));
                double ZResolution = NM_TO_MM(GoSurfaceMsg_ZResolution(surfaceMsg));
                double XOffset = UM_TO_MM(GoSurfaceMsg_XOffset(surfaceMsg));
                double YOffset = UM_TO_MM(GoSurfaceMsg_YOffset(surfaceMsg));
                double ZOffset = UM_TO_MM(GoSurfaceMsg_ZOffset(surfaceMsg));

                printf("  Surface data width: %lu\n", (k32u)GoSurfaceMsg_Width(surfaceMsg));
                printf("  Surface data length: %lu\n", (k32u)GoSurfaceMsg_Length(surfaceMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<CallOneTimes::ProfilePoint**>( malloc(GoSurfaceMsg_Length(surfaceMsg) * sizeof(CallOneTimes::ProfilePoint*)));

                    for (size_t j = 0; j < GoSurfaceMsg_Length(surfaceMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<CallOneTimes::ProfilePoint*> (malloc(GoSurfaceMsg_Width(surfaceMsg) * sizeof(CallOneTimes::ProfilePoint)));
                    }

                    surfaceBufferHeight = (k32u)GoSurfaceMsg_Length(surfaceMsg);
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr collectCloudPoint(new pcl::PointCloud<pcl::PointXYZ>);

                for (rowIdx = 0; rowIdx < GoSurfaceMsg_Length(surfaceMsg); rowIdx++)
                {
                    k16s* data = GoSurfaceMsg_RowAt(surfaceMsg, rowIdx);

                    for (colIdx = 0; colIdx < GoSurfaceMsg_Width(surfaceMsg); colIdx++)
                    {
                        // gocator transmits range data as 16-bit signed integers
                        // to translate 16-bit range data to engineering units, the calculation for each point is: 
                        //          X: XOffset + columnIndex * XResolution 
                        //          Y: YOffset + rowIndex * YResolution
                        //          Z: ZOffset + height_map[rowIndex][columnIndex] * ZResolution

                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;


                        if (data[colIdx] != INVALID_RANGE_16BIT)
                        {
                            surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx];
                        }
                        else
                        {
                            surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                        }
                        pcl::PointXYZ point;
                        point.x = static_cast<float>(colIdx* XResolution);
                        point.y = static_cast<float>(rowIdx * YResolution);
                        point.z = static_cast<float>(surfaceBuffer[rowIdx][colIdx].z);
                         collectCloudPoint->push_back(point);
                    }
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
                clouds = collectCloudPoint;
                pcl::PCLPointCloud2 cloud2;
                // pcl::toPCLPointCloud2(*cloud, cloud2);
                pcl::toPCLPointCloud2(*clouds, cloud2);
                QString filePath = "D:/project/PolePieceMeasure/PolePieceMeasureWin/x64/Release/ply/20251118.ply";
                int ret = pcl::io::savePLYFileBinary(filePath.toStdString(), *clouds);
               // int result = pcl::io::savePLYFile("D:/project/PolePieceMeasure/PolePieceMeasureWin/x64/Release/ply/20251118.ply", cloud2, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false, false);

            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_POINT_CLOUD:
            {
                GoSurfacePointCloudMsg surfacePointCloudMsg = dataObj;
                unsigned int rowIdx, colIdx;

                double XResolution = NM_TO_MM(GoSurfacePointCloudMsg_XResolution(surfacePointCloudMsg));
                double YResolution = NM_TO_MM(GoSurfacePointCloudMsg_YResolution(surfacePointCloudMsg));
                double ZResolution = NM_TO_MM(GoSurfacePointCloudMsg_ZResolution(surfacePointCloudMsg));
                double XOffset = UM_TO_MM(GoSurfacePointCloudMsg_XOffset(surfacePointCloudMsg));
                double YOffset = UM_TO_MM(GoSurfacePointCloudMsg_YOffset(surfacePointCloudMsg));
                double ZOffset = UM_TO_MM(GoSurfacePointCloudMsg_ZOffset(surfacePointCloudMsg));

                printf("  Surface Point Cloud data width: %lu\n", (k32u)GoSurfacePointCloudMsg_Width(surfacePointCloudMsg));
                printf("  Surface Point Cloud data length: %lu\n", (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<CallOneTimes::ProfilePoint**> (malloc(GoSurfacePointCloudMsg_Length(surfacePointCloudMsg) * sizeof(CallOneTimes::ProfilePoint*)));

                    for (size_t j = 0; j < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<CallOneTimes::ProfilePoint*>(malloc(GoSurfacePointCloudMsg_Width(surfacePointCloudMsg) * sizeof(CallOneTimes::ProfilePoint)));
                    }

                    surfaceBufferHeight = (k32u)GoSurfacePointCloudMsg_Length(surfacePointCloudMsg);
                }

                for (rowIdx = 0; rowIdx < GoSurfacePointCloudMsg_Length(surfacePointCloudMsg); rowIdx++)
                {
                    kPoint3d16s* data = GoSurfacePointCloudMsg_RowAt(surfacePointCloudMsg, rowIdx);

                    for (colIdx = 0; colIdx < GoSurfacePointCloudMsg_Width(surfacePointCloudMsg); colIdx++)
                    {
                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * data[colIdx].x;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * data[colIdx].y;

                        if (data[colIdx].z != INVALID_RANGE_16BIT)
                        {
                            surfaceBuffer[rowIdx][colIdx].z = ZOffset + ZResolution * data[colIdx].z;
                        }
                        else
                        {
                            surfaceBuffer[rowIdx][colIdx].z = INVALID_RANGE_DOUBLE;
                        }
                    }
                }
            }
            break;
            case GO_DATA_MESSAGE_TYPE_SURFACE_INTENSITY:
            {
                GoSurfaceIntensityMsg surfaceIntMsg = dataObj;
                unsigned int rowIdx, colIdx;
                double XResolution = NM_TO_MM(GoSurfaceIntensityMsg_XResolution(surfaceIntMsg));
                double YResolution = NM_TO_MM(GoSurfaceIntensityMsg_YResolution(surfaceIntMsg));
                double XOffset = UM_TO_MM(GoSurfaceIntensityMsg_XOffset(surfaceIntMsg));
                double YOffset = UM_TO_MM(GoSurfaceIntensityMsg_YOffset(surfaceIntMsg));

                printf("  Surface intensity width: %lu\n", (k32u)GoSurfaceIntensityMsg_Width(surfaceIntMsg));
                printf("  Surface intensity height: %lu\n", (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg));

                //allocate memory if needed
                if (surfaceBuffer == NULL)
                {
                    surfaceBuffer = static_cast<CallOneTimes::ProfilePoint**> (malloc(GoSurfaceIntensityMsg_Length(surfaceIntMsg) * sizeof(CallOneTimes::ProfilePoint*)));
                    //                        surfaceBuffer[j] = static_cast<CallOneTimes::ProfilePoint*> (malloc(GoSurfaceMsg_Width(surfaceMsg) * sizeof(CallOneTimes::ProfilePoint)));

                    for (size_t j = 0; j < GoSurfaceIntensityMsg_Length(surfaceIntMsg); j++)
                    {
                        surfaceBuffer[j] = static_cast<CallOneTimes::ProfilePoint*>(malloc(GoSurfaceIntensityMsg_Width(surfaceIntMsg) * sizeof(CallOneTimes::ProfilePoint)));
                    }

                    surfaceBufferHeight = (k32u)GoSurfaceIntensityMsg_Length(surfaceIntMsg);
                }

                for (rowIdx = 0; rowIdx < GoSurfaceIntensityMsg_Length(surfaceIntMsg); rowIdx++)
                {
                    k8u* data = GoSurfaceIntensityMsg_RowAt(surfaceIntMsg, rowIdx);

                    // gocator transmits intensity data as an 8-bit grayscale image of identical width and height as the corresponding height map
                    for (colIdx = 0; colIdx < GoSurfaceIntensityMsg_Width(surfaceIntMsg); colIdx++)
                    {
                        surfaceBuffer[rowIdx][colIdx].x = XOffset + XResolution * colIdx;
                        surfaceBuffer[rowIdx][colIdx].y = YOffset + YResolution * rowIdx;
                        surfaceBuffer[rowIdx][colIdx].intensity = data[colIdx];
                    }

                }
            }
            break; 

            } 
        }  
    }  
}
void PolePieceMeasureWin::on_btnClearRecord_clicked()//清理日志
{
    ui->txtShowRecord->setPlainText("");    // 清空日志
}
void PolePieceMeasureWin::insertAtTop(QTextEdit* textEdit, const QString& text)
{
    QTextCursor cursor(textEdit->document());
    cursor.movePosition(QTextCursor::Start);  // 移动到文档开头
    cursor.insertText(text + "\n");           // 插入文本并换行
}
void  PolePieceMeasureWin::updateRecord(QString _str)
{
    //ui->txtShowRecord->addLog(_str,0);
    /*QDateTime currentDateTimeDepth = QDateTime::currentDateTime();
    QString currentTime = currentDateTimeDepth.toString(QStringLiteral("yyyy-MM-dd hh:mm:ss"));
    QFont f("Microsoft YaHei", 12);
    f.setBold(true);
    ui->txtShowRecord->setFont(f);
    insertAtTop(ui->txtShowRecord, currentTime + QStringLiteral(" ") + _str + QStringLiteral("..."));*/
    //ui->txtShowRecord->setFont(QFont("Microsoft YaHei", 15));   // 字体 + 字号
   // ui->txtShowRecord->insertPlainText(currentTime + " " + _str + "\n");
   // qInfo() << _str;
}
void PolePieceMeasureWin::RecordData(QString _str, int level)
{
    // 将int等级转换为AlarmLevel枚举
    hgHMI_LogTextEdit::AlarmLevel alarmLevel = hgHMI_LogTextEdit::Info; // 默认Info
    switch (level) {
    case 1:
        alarmLevel = hgHMI_LogTextEdit::Warning;
        break;
    case 2:
        alarmLevel = hgHMI_LogTextEdit::Error;
        break;
    case 0:
    default:
        alarmLevel = hgHMI_LogTextEdit::Info;
        break;
    }

    // 调用自定义控件的日志添加函数
    ui->txtShowRecord->addLog(_str, alarmLevel);
    qInfo() << _str;
}
//PLC
void PolePieceMeasureWin::on_btnDisConnectPLC_clicked()//PLC 断开
{
    if (plc)
    {
        plc->stopPlcPolling();   // 先停实时轮询线程，再断开，避免跨线程操作 socket
        plc->disconnectPLC();
    }
    ParamUi->PLCConnectState->setText(QStringLiteral("unconnected"));
    ParamUi->PLCConnectState->setStyleSheet("color: red;");
    InfoPLC.PLCConnectState = QStringLiteral("unconnected");
    RecordData(QStringLiteral("PLC断开连接成功")); //记录日志
    labelledPLC->setLedOn(false); //指示灯灭
}
void PolePieceMeasureWin::on_btnConnectPLC_clicked()//PLC 连接
{
    QString plcip = InfoPLC.PLC_IP; //ui->editPLCIP->text();
    int plcport = InfoPLC.PLC_PORT.toInt();
    if (!plc)
    {
        plc = new Communication();
    }
    // 信号只连一次：Qt::UniqueConnection 保证重复点击也不会重复连接
    connect(plc, &Communication::sendCommunicateMessage, this, &PolePieceMeasureWin::receiveCommunicateMessage, Qt::UniqueConnection);
    connect(plc, &Communication::plcSignalsUpdated, this, &PolePieceMeasureWin::onPlcSignalsUpdated, Qt::UniqueConnection);

    plc->getConnectInfo(plcip, quint16(plcport)); //获取连接信息
    // 实时轮询：连接、读 PLC->CCD、写 CCD->PLC、断线重连都在 Communication::run() 里完成
    plc->startPlcPolling(plcip, quint16(plcport), 100);

    RecordData(QStringLiteral("PLC实时轮询启动中...")); //记录日志
}

void PolePieceMeasureWin::receiveCommunicateMessage(const QString& msg)
{
    RecordData(msg);
}

void PolePieceMeasureWin::on_btnWritePLCReister_clicked()//写入值
{
    plc->writeValue(Communication::AreaDM, 0x0000, 0x0001); //写入寄存器::Area0, 0x0000, 0x0001); //写入寄存器


}
void PolePieceMeasureWin::on_btnReadPLCReister_clicked()//读取值
{
    quint16 value;
    plc->readValue(Communication::AreaDM, 0x0000, value); //读取寄存器::Area0, 0x0000, 0x0001); //读取寄存器
    RecordData(QString("读取寄存器值：%1").arg(value)); //记录日志
}
void PolePieceMeasureWin::slotReceiveOuMoRon(QVector<quint16> val)
{



}
//图形化显示
void PolePieceMeasureWin::on_btnLoadPointsToExcel_clicked()
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
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

    ////// 设置暗色背景
    //ui->widget_3DView->setBackgroundColor(QColor(30, 30, 30));         // 深色背景
    //ui->widget_3DView->setAxisColor(Qt::white);                        // 坐标轴白色
    //ui->widget_3DView->setGridColor(QColor(100, 100, 100));            // 网格线暗灰色
    //ui->widget_3DView->setTickLabelColor(Qt::white);                   // 刻度标签白色
    //ui->widget_3DView->setAuxiliaryLineColor(QColor(255, 128, 0));     // 辅助线橙色
    //ui->widget_3DView->setPointColor(Qt::cyan);                        // 点云颜色青色
    //ui->widget_3DView->setHighlightColor(Qt::yellow);                  // 选中点黄色
    //
    //// 设置参数
    //ui->widget_3DView->setMargins(50, 20, 20, 40);           // 整体边距
    //ui->widget_3DView->setPointRadius(3);                     // 普通点大小稍大
    //ui->widget_3DView->setHighlightRadius(4);                 // 高亮点大小
    //ui->widget_3DView->setTickLabelFontSize(8);               // 刻度标签字体大小
    //ui->widget_3DView->setTickLabelDecimals(1);               // 保留1位小数
    //ui->widget_3DView->setNumMainTicks(10);                    // 主刻度数
    //ui->widget_3DView->setNumSubTicks(4);                      // 副刻度数
    //ui->widget_3DView->setMarginPointCloud(0.01);              // 点云边距留白比例 1%
    
    //ui->widget_3DView->setPointCloud(cloud);
    //ui->widget_3DView->setTheme(1);
    //ui->widget_3DView->setTickLabelDecimals(4);               // 保留1位小数
    //// 添加自定义点，颜色绿色，半径8
    //QVector<QPointF> customPts;
    //customPts << QPointF(4.67, 0.17) << QPointF(4.72, 0.17) << QPointF(4.69, 0.1627);
    //ui->widget_3DView->addCustomPoints(customPts, Qt::green, 6);

 }

//通信新增
// 每轮 PLC->CCD 读取完成后触发（Communication 在工作线程发信号，这里在 UI 线程执行）
void PolePieceMeasureWin::onPlcSignalsUpdated()
{
    if (!plc)
        return;

    const Communication::PlcSignals sig = plc->lastSignals();
    m_plcSignals = sig;

    // 连接指示灯/状态：第一次收到数据即视为已连接
    static bool firstData = true;
    if (firstData)
    {
        firstData = false;
        ParamUi->PLCConnectState->setText(QStringLiteral("connected"));
        ParamUi->PLCConnectState->setStyleSheet("color: green;");
        InfoPLC.PLCConnectState = QStringLiteral("connected");
        labelledPLC->setLedOn(true);
        RecordData(QStringLiteral("PLC连接成功（已收到PLC信号）"));
    }

    // 触发信号：0默认 / 10触发 / 11点检 / 20仅采图不判定
    static float lastCam1Trigger = -1.0f;
    static float lastCam2Trigger = -1.0f;
    if (!qFuzzyCompare(sig.cam1Trigger + 1.0f, lastCam1Trigger + 1.0f))
    {
        lastCam1Trigger = sig.cam1Trigger;
        RecordData(QStringLiteral("相机1触发信号=%1").arg(sig.cam1Trigger));
    }
    if (!qFuzzyCompare(sig.cam2Trigger + 1.0f, lastCam2Trigger + 1.0f))
    {
        lastCam2Trigger = sig.cam2Trigger;
        RecordData(QStringLiteral("相机2触发信号=%1").arg(sig.cam2Trigger));
    }

    // 产品型号变化
    static QString lastModel;
    if (sig.productModel != lastModel)
    {
        lastModel = sig.productModel;
        RecordData(QStringLiteral("产品型号=%1").arg(lastModel));
    }

    // TODO: 在这里按触发值接业务：
    //   10 -> 开始采集/处理，处理完调用 plcWriteCamResult(1 或 2, 10.0f, 结果码) 回写
    //   11 -> 点检
    //   20 -> 只采图，不做数据判定
    // 3D 出现 NG 时可用 sig.cacheEncoder1 / sig.cacheEncoder2 取缓存位实时编码值
}

// 相机结果回写 PLC（相机1：D2314/D2316，相机2：D2318/D2320）
// startValue：启动信号，OK=10 / NG=11
// resultValue：结果信号，OK=10 / 槽深槽宽均NG=11 / 单独槽深NG=12 / 单独槽宽NG=13
void PolePieceMeasureWin::plcWriteCamResult(int cameraIndex, float startValue, float resultValue)
{
    if (!plc)
        return;

    if (cameraIndex == 1)
        plc->setCamera1Command(startValue, resultValue);
    else
        plc->setCamera2Command(startValue, resultValue);
}
