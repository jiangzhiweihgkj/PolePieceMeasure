#include "PolePieceMeasureWin.h"
#include <QtCharts/QLegendMarker>
 
void PolePieceMeasureWin::Show_CurrentAB_Table_Show()
{
    // 在构造函数里 
    // for (int i = 0; i < 10; i++)
    //{
    //    ui->tableAB_Show_Data->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Stretch); 
    //} 
    //ui->tableAB_Show_Data->verticalHeader()->setVisible(true); // 显示垂直表头
    //ui->tableAB_Show_Data->setEditTriggers(QAbstractItemView::NoEditTriggers); // 设置表格不可编辑

    //ui->tableAB_Show_Data->setRowCount(2);//3行Calucate_Width_Depth.m_mean_width
    if (InfoSuanFa.editDetectedWidthMin.toDouble() < 1.000* Calucate_Width_Depth.m_mean_width && 1.000 * Calucate_Width_Depth.m_mean_width < InfoSuanFa.editDetectedWidthMax.toDouble()
        && InfoSuanFa.editDetectedDepthMin.toDouble() < 1.000 * Calucate_Width_Depth.m_mean_depth && 1.000 * Calucate_Width_Depth.m_mean_depth < InfoSuanFa.editDetectedDepthMax.toDouble())
    {
        _callOneTimes->class_A_B.Result_A.Number_Check_Result_OK = _callOneTimes->class_A_B.Result_A.Number_Check_Result_OK + 1;
    }
    else
    {  
         _callOneTimes->class_A_B.Result_A.Number_Check_Result_NG = _callOneTimes->class_A_B.Result_A.Number_Check_Result_NG + 1; 
    }
    int all_number_OK = _callOneTimes->class_A_B.Result_A.Number_Check_Result_OK;
    QTableWidgetItem* widthItem = new QTableWidgetItem(QString::number(all_number_OK));
    ui->tableAB_Show_Data->setItem(0, 0, widthItem);
  
    int all_number_NG = _callOneTimes->class_A_B.Result_A.Number_Check_Result_NG; 
    QTableWidgetItem* widthItem1 = new QTableWidgetItem(QString::number(all_number_NG));
    ui->tableAB_Show_Data->setItem(0, 1, widthItem1); 
    _callOneTimes->class_A_B.Result_A.Number_Check_Result_ALL = all_number_OK + all_number_NG;
    int all_number = _callOneTimes->class_A_B.Result_A.Number_Check_Result_ALL; 
    QTableWidgetItem* widthItem2 = new QTableWidgetItem(QString::number(all_number));
    ui->tableAB_Show_Data->setItem(0, 2, widthItem2); 

    _callOneTimes->class_A_B.Result_A.Yield_Check_Result_OK = all_number_OK *1.0/all_number;
    double all_number_Yield = _callOneTimes->class_A_B.Result_A.Yield_Check_Result_OK; 
    QTableWidgetItem* widthItem3 = new QTableWidgetItem(QString::number(all_number_Yield * 100, 'f', 2) + QStringLiteral("%"));
    ui->tableAB_Show_Data->setItem(0, 3, widthItem3);
    
    if (1.000 * Calucate_Width_Depth.m_mean_depth<InfoSuanFa.editDetectedDepthMin.toDouble()   || 1.000 * Calucate_Width_Depth.m_mean_depth > InfoSuanFa.editDetectedDepthMax.toDouble())
    {
        _callOneTimes->class_A_B.Result_A.Number_Depth_Result_NG = _callOneTimes->class_A_B.Result_A.Number_Depth_Result_NG + 1; 
    }

    int all_number_depth_NG = _callOneTimes->class_A_B.Result_A.Number_Depth_Result_NG;
    QTableWidgetItem* widthItem4 = new QTableWidgetItem(QString::number(all_number_depth_NG));
    ui->tableAB_Show_Data->setItem(0, 4, widthItem4);
    if (InfoSuanFa.editDetectedWidthMin.toDouble() > 1.000 * Calucate_Width_Depth.m_mean_width || 1.000 * Calucate_Width_Depth.m_mean_width > InfoSuanFa.editDetectedWidthMax.toDouble())
    {
        _callOneTimes->class_A_B.Result_A.Number_Width_Result_NG = _callOneTimes->class_A_B.Result_A.Number_Width_Result_NG + 1; 
    }
    int all_number_width_NG = _callOneTimes->class_A_B.Result_A.Number_Width_Result_NG;
    QTableWidgetItem* widthItem5 = new QTableWidgetItem(QString::number(all_number_width_NG));
    ui->tableAB_Show_Data->setItem(0, 5, widthItem5);
    
    double Measure_Depth_Result = _callOneTimes->class_A_B.Result_A.Measure_Depth_Result;
    QTableWidgetItem* widthItem6 = new QTableWidgetItem(QString::number(Measure_Depth_Result, 'f', 2));
    ui->tableAB_Show_Data->setItem(0, 6, widthItem6); 
    
    _callOneTimes->class_A_B.Result_A.Depth_Model = InfoSuanFa.editDetectedDepthMin+"-"+InfoSuanFa.editDetectedDepthMax;
    QString  Depth_Model = _callOneTimes->class_A_B.Result_A.Depth_Model;
    QTableWidgetItem* widthItem7 = new QTableWidgetItem(Depth_Model);
    ui->tableAB_Show_Data->setItem(0, 7, widthItem7);
    
    double Measure_Width_Result = _callOneTimes->class_A_B.Result_A.Measure_Width_Result;
    QTableWidgetItem* widthItem8 = new QTableWidgetItem(QString::number(Measure_Width_Result, 'f', 2));
    ui->tableAB_Show_Data->setItem(0, 8, widthItem8);

    _callOneTimes->class_A_B.Result_A.Width_Model = InfoSuanFa.editDetectedWidthMin + "-" + InfoSuanFa.editDetectedWidthMax; 
    QString  Width_Model = _callOneTimes->class_A_B.Result_A.Width_Model;
    QTableWidgetItem* widthItem9 = new QTableWidgetItem(Width_Model);
    ui->tableAB_Show_Data->setItem(0, 9, widthItem9);

    //// 设置所有单元格内容居中
    for (int row = 0; row < ui->tableAB_Show_Data->rowCount(); ++row)
    {
        for (int col = 0; col < ui->tableAB_Show_Data->columnCount(); ++col)
        {
            QTableWidgetItem* item = ui->tableAB_Show_Data->item(row, col);
            if (item)
                item->setTextAlignment(Qt::AlignCenter);
        }
    }
    auto ToPoint2Data = [](double v) -> QString {
        std::ostringstream o;
        o << std::fixed << std::setprecision(2) << v ;
        return QString::fromStdString(o.str());
        };
    QString A_Max_Width = ToPoint2Data( Calucate_Width_Depth.m_max_width);
    QString A_Mean_Width = ToPoint2Data( Calucate_Width_Depth.m_mean_width);
    QString A_Min_Width = ToPoint2Data( Calucate_Width_Depth.m_min_width);
    
    QString A_Max_Depth = ToPoint2Data( Calucate_Width_Depth.m_max_depth);
    QString A_Mean_Depth = ToPoint2Data( Calucate_Width_Depth.m_mean_depth);
    QString A_Min_Depth = ToPoint2Data( Calucate_Width_Depth.m_min_depth);

    QString result_A = ui->Result_A->text();

    QDateTime date = QDateTime::currentDateTime();
    QString strWorkName = date.toString("yyyy / MM / dd / hh:mm:ss"); 
    int index_A = all_number - 1;
    DataExcelUi->tableWidgetExcel_A->insertRow(index_A);
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 0, new QTableWidgetItem(QString::number(index_A)));//序号
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 1, new QTableWidgetItem(strWorkName));//日期
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 2, new QTableWidgetItem(A_Mean_Depth));//A面槽深均值
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 3, new QTableWidgetItem(A_Max_Depth));//A面槽深最大值
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 4, new QTableWidgetItem(A_Min_Depth));//A面槽深最小值
    if (Calucate_Width_Depth.m_mean_depth<InfoSuanFa.editDetectedDepthMin.toDouble() ||  Calucate_Width_Depth.m_mean_depth > InfoSuanFa.editDetectedDepthMax.toDouble())
    {
        DataExcelUi->tableWidgetExcel_A->setItem(index_A, 5, new QTableWidgetItem(QStringLiteral("NG")));//A面槽深结果

    }
    else
    {
        DataExcelUi->tableWidgetExcel_A->setItem(index_A, 5, new QTableWidgetItem(QStringLiteral("OK")));//A面槽深结果
    }
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 6, new QTableWidgetItem(A_Mean_Width));//A面槽宽均值
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 7, new QTableWidgetItem(A_Max_Width));//A面槽宽最大值
    DataExcelUi->tableWidgetExcel_A->setItem(index_A, 8, new QTableWidgetItem(A_Min_Width));//A面槽宽最小值
    if (InfoSuanFa.editDetectedWidthMin.toDouble() >  Calucate_Width_Depth.m_mean_width || Calucate_Width_Depth.m_mean_width > InfoSuanFa.editDetectedWidthMax.toDouble())
    {
        DataExcelUi->tableWidgetExcel_A->setItem(index_A, 9, new QTableWidgetItem(QStringLiteral("NG")));//A面槽宽结果 
    }
    else
    {
        DataExcelUi->tableWidgetExcel_A->setItem(index_A, 9, new QTableWidgetItem(QStringLiteral("OK")));//A面槽宽结果  
    }

 



    /*ui->tableAB_Show_Data->setItem(1, 0, widthItem);
      widthItem1 = new QTableWidgetItem(QString::number(1004));
    ui->tableAB_Show->setItem(1, 1, widthItem1);
      widthItem2 = new QTableWidgetItem(QString::number(1005));
    ui->tableAB_Show->setItem(1, 2, widthItem2);*/



   // ui->tableAB_Show->setColumnCount(5);//5列

    //ID号
    //QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(ID_NEW[i], 10));
    //ui->tableFrame->setItem(i, 0, idItem);
    ////宽度
    //QTableWidgetItem* widthItem = new QTableWidgetItem(QString::number(Calucate_Width_Depth.m_width[i], 'f', 3));
    //ui->tableFrame->setItem(i, 1, widthItem);

    ////深度
    //QTableWidgetItem* depthItem = new QTableWidgetItem(QString::number(Calucate_Width_Depth.m_depth[i], 'f', 4));
    ////ui->tableFrame->setItem(i, 2, depthItem);
    //ui->tableAB_Show->clearContents();//清除内容但保留表头
    ////int rowCount = static_cast<int>(Calucate_Width_Depth.m_width.size());
    //if (Calucate_Width_Depth.m_depth.size() != Calucate_Width_Depth.m_width.size() || ID_NEW.size() != Calucate_Width_Depth.m_width.size()) {
    //    return;
    //}

    //double maxDepth = ui->editDetectedDepthMin->text().toDouble();
    //ui->tableFrame->setSortingEnabled(false);//控制是否允许用户通过点击列头来对表格内容进行排序。
    //// 遍历 InfoResult，将数据填入表格
    //for (size_t i = 0; i < 2; ++i)
    //{
    //    //ID号
    //   /* QTableWidgetItem* idItem = new NumericTableWidgetItem(QString::number(ID_NEW[i], 10));
    //    ui->tableFrame->setItem(i, 0, idItem);*/
    //    //宽度
    //    QTableWidgetItem* widthItem = new QTableWidgetItem(QString::number(Calucate_Width_Depth.m_mean_depth*1.000));
    //    ui->tableFrame->setItem(i, 1, widthItem);
    //    for (int j = 0;i<3;++j)
    //    {
    //        //深度
    //        QTableWidgetItem* depthItem = new QTableWidgetItem(QString::number(Calucate_Width_Depth.m_depth[j]*1.000, 'f'));
    //        ui->tableFrame->setItem(i, 2, depthItem);
    //    }
    //    

    //    //// 判断结果
    //    //QString result = (Calucate_Width_Depth.m_depth[i] >= maxDepth) ? QStringLiteral("NG") : QStringLiteral("OK");
    //    //QTableWidgetItem* resultItem = new QTableWidgetItem(result);
    //    //if (result == QStringLiteral("NG"))
    //    //{
    //    //    resultItem->setForeground(QBrush(Qt::red));
    //    //    QFont font = resultItem->font();
    //    //    font.setBold(true);
    //    //    resultItem->setFont(font);

    //    //}
    //    //else if (result == QStringLiteral("OK"))
    //    //{
    //    //    resultItem->setForeground(QBrush(QColor(0, 128, 0)));
    //    //    QFont font = resultItem->font();
    //    //    font.setBold(true);
    //    //    resultItem->setFont(font);
    //    //}
    //    //ui->tableFrame->setItem(i, 3, resultItem);
    //}
    //ui->tableFrame->setSortingEnabled(true); // 启用排序功能

   
}

void PolePieceMeasureWin::tableAB_Show()
{
    if (Calucate_Width_Depth.m_depth.size() == 0 || Calucate_Width_Depth_B.m_depth.size() == 0)
    {
       // QMessageBox::warning(this, QStringLiteral("警告"), QStringLiteral("没有测量数据！"));
        return;
    }
    ui->tableAB_Show ->setRowCount(2);// 2行 
    for (int i = 0; i < 8; i++)
    {
        ui->tableAB_Show->horizontalHeader()->setSectionResizeMode(i, QHeaderView::Stretch);// 设置列宽自适应 
    }
    
    double Measure_Depth_Result = Calucate_Width_Depth.m_mean_depth*1.000; //_callOneTimes->class_A_B.Result_A.Measure_Depth_Result;
    QTableWidgetItem* widthItem1 = new QTableWidgetItem(QString::number(Measure_Depth_Result, 'f', 2));
    ui->tableAB_Show->setItem(0, 0, widthItem1);
    
    double m_depth_0 = Calucate_Width_Depth.m_depth[0] * 1.000;
    QTableWidgetItem* widthItem2 = new QTableWidgetItem(QString::number(m_depth_0, 'f', 2));
    ui->tableAB_Show->setItem(0, 1, widthItem2);
    
    double m_depth_1 = Calucate_Width_Depth.m_depth[1] * 1.000;
    QTableWidgetItem* widthItem3 = new QTableWidgetItem(QString::number(m_depth_1, 'f', 2));
    ui->tableAB_Show->setItem(0, 2, widthItem3);
    
    double m_depth_2 = Calucate_Width_Depth.m_depth[2] * 1.000;
    QTableWidgetItem* widthItem4 = new QTableWidgetItem(QString::number(m_depth_2, 'f', 2));
    ui->tableAB_Show->setItem(0, 3, widthItem4); 
    
    
    double Measure_Width_Result = Calucate_Width_Depth.m_mean_width*1.000; //_callOneTimes->class_A_B.Result_A.Measure_Depth_Result;
    QTableWidgetItem* widthItem5 = new QTableWidgetItem(QString::number(Measure_Width_Result, 'f', 2));
    ui->tableAB_Show->setItem(1, 0, widthItem5);
    
    double m_width_0 = Calucate_Width_Depth.m_width[0] * 1.000;
    QTableWidgetItem* widthItem6 = new QTableWidgetItem(QString::number(m_width_0, 'f', 2));
    ui->tableAB_Show->setItem(1, 1, widthItem6);
    
    double m_width_1 = Calucate_Width_Depth.m_width[1] * 1.000;
    QTableWidgetItem* widthItem7 = new QTableWidgetItem(QString::number(m_width_1, 'f', 2));
    ui->tableAB_Show->setItem(1, 2, widthItem7);
    
    double m_width_2 = Calucate_Width_Depth.m_width[2] * 1.000;
    QTableWidgetItem* widthItem8 = new QTableWidgetItem(QString::number(m_width_2, 'f', 2));
    ui->tableAB_Show->setItem(1, 3, widthItem8); 
    
    double Measure_Depth_Result_B = Calucate_Width_Depth_B.m_mean_depth*1.000; //_callOneTimes->class_A_B.Result_A.Measure_Depth_Result;
    QTableWidgetItem* widthItem1_B = new QTableWidgetItem(QString::number(Measure_Depth_Result_B, 'f', 2));
    ui->tableAB_Show->setItem(0, 4, widthItem1_B);
    
    double m_depth_0_B = Calucate_Width_Depth_B.m_depth[0] * 1.000;
    QTableWidgetItem* widthItem2_B = new QTableWidgetItem(QString::number(m_depth_0_B, 'f', 2));
    ui->tableAB_Show->setItem(0, 5, widthItem2_B);
    
    double m_depth_1_B = Calucate_Width_Depth_B.m_depth[1] * 1.000;
    QTableWidgetItem* widthItem3_B = new QTableWidgetItem(QString::number(m_depth_1_B, 'f', 2));
    ui->tableAB_Show->setItem(0, 6, widthItem3_B);
    
    double m_depth_2_B = Calucate_Width_Depth_B.m_depth[2] * 1.000;
    QTableWidgetItem* widthItem4_B = new QTableWidgetItem(QString::number(m_depth_2_B, 'f', 2));
    ui->tableAB_Show->setItem(0, 7, widthItem4_B);
    
    
    double Measure_Width_Result_B = Calucate_Width_Depth_B.m_mean_width*1.000; //_callOneTimes->class_A_B.Result_A.Measure_Depth_Result;
    QTableWidgetItem* widthItem5_B = new QTableWidgetItem(QString::number(Measure_Width_Result_B, 'f', 2));
    ui->tableAB_Show->setItem(1, 4, widthItem5_B);
    
    double m_width_0_B = Calucate_Width_Depth_B.m_width[0] * 1.000;
    QTableWidgetItem* widthItem6_B = new QTableWidgetItem(QString::number(m_width_0_B, 'f', 2));
    ui->tableAB_Show->setItem(1, 5, widthItem6_B);
    
    double m_width_1_B = Calucate_Width_Depth_B.m_width[1] * 1.000;
    QTableWidgetItem* widthItem7_B = new QTableWidgetItem(QString::number(m_width_1_B, 'f', 2));
    ui->tableAB_Show->setItem(1, 6, widthItem7_B);
    
    double m_width_2_B = Calucate_Width_Depth_B.m_width[2] * 1.000;
    QTableWidgetItem* widthItem8_B = new QTableWidgetItem(QString::number(m_width_2_B, 'f', 2));
    ui->tableAB_Show->setItem(1, 7, widthItem8_B);

    for (int row = 0; row < ui->tableAB_Show->rowCount(); ++row)
    {
        for (int col = 0; col < ui->tableAB_Show->columnCount(); ++col)
        {
            QTableWidgetItem* item = ui->tableAB_Show->item(row, col);
            if (item)
                item->setTextAlignment(Qt::AlignCenter);
        }
    }
}
 
void PolePieceMeasureWin::show_A_Depth_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> depth_min, QVector<QPointF> depth_max)
{ 
    if (m_chartView_A_Depth)
    {           // m_chartView_B_Depth 是类成员指针，初始 nullptr
        ui->chartContainer_A_Depth->layout()->removeWidget(m_chartView_A_Depth);
        delete m_chartView_A_Depth;
        m_chartView_A_Depth = nullptr;
    }
    delete ui->chartContainer_A_Depth->layout();   // 把旧布局彻底删掉
    QVBoxLayout* ly = new QVBoxLayout(ui->chartContainer_A_Depth);    // 1. 先给容器加个布局，这样 chartView 能自动撑满 
    ly->setContentsMargins(0, 0, 0, 0); // 2. 布局设置边距为 0
    
    series_A_Depth_Mean->setName(QStringLiteral("Mean"));
    series_A_Depth_Min->setName(QStringLiteral("Min"));
    series_A_Depth_Max->setName(QStringLiteral("Max"));
    series_A_Depth_Signma->setName(QStringLiteral("CPK"));
    series_A_Depth_Mean->setPointsVisible(true);          // 1. 显示端点
    series_A_Depth_Mean->setColor(Qt::yellow);              // 3. 连线和端点一起变色
    series_A_Depth_Mean->setPen(QPen(QBrush(Qt::yellow), 3));   // 颜色保持原来的蓝
    
    series_A_Depth_Min->setPointsVisible(true);          // 1. 显示端点
    series_A_Depth_Min->setColor(Qt::red);              // 3. 连线和端点一起变色
    series_A_Depth_Min->setPen(QPen(QBrush(Qt::red), 3));   // 颜色保持原来的蓝
    
    series_A_Depth_Max->setPointsVisible(true);          // 1. 显示端点
    series_A_Depth_Max->setColor(Qt::blue);              // 3. 连线和端点一起变色
    series_A_Depth_Max->setPen(QPen(QBrush(Qt::blue), 3));   // 颜色保持原来的蓝
    
    series_A_Depth_Signma->setPointsVisible(true);          // 1. 显示端点
    series_A_Depth_Signma->setColor(Qt::green);              // 3. 连线和端点一起变色
    series_A_Depth_Signma->setPen(QPen(QBrush(Qt::green), 3));   // 颜色保持原来的蓝
         
    /*if (data1.isEmpty()) return;
    if (depth_min.isEmpty()) return;
    if (depth_max.isEmpty()) return;
    if (data2.size() < 3) data2.clear();*/

    series_A_Depth_Mean->clear(); 
    for (int i = 0; i < data1.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Depth_Mean->append(i + 1, data1[i].y());
    }

    series_A_Depth_Min->clear(); 
    for (int i = 0; i < depth_min.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Depth_Min->append(i + 1, depth_min[i].y());
    }

    series_A_Depth_Max->clear(); 
    for (int i = 0; i < depth_max.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Depth_Max->append(i + 1, depth_max[i].y());
    }

    series_A_Depth_Signma->clear();
    for (int i = 0; i < data2.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Depth_Signma->append(i + 1, data2[i].y());
    }
   // series_A_Depth_Mean->replace(data1);//平均 
   // series_A_Depth_Signma->replace(data2);//方差


    //显示上下限虚线创建上限虚线
    QLineSeries* upperLimitSeries = new QLineSeries();
    upperLimitSeries->setName(QStringLiteral("规格上限"));
    upperLimitSeries->setPen(QPen(Qt::red, 2, Qt::DashLine));

    // 创建下限虚线
    QLineSeries* lowerLimitSeries = new QLineSeries();
    lowerLimitSeries->setName(QStringLiteral("规格下限"));
    lowerLimitSeries->setPen(QPen(Qt::green, 2, Qt::DashLine));

    //// 设置上下限值
    double upperLimit = InfoSuanFa.editDetectedDepthMax.toDouble();
    double lowerLimit = InfoSuanFa.editDetectedDepthMin.toDouble();
    double minX = 1, maxX = data1.size(); // X轴范围

    // 添加上下限的数据点
    upperLimitSeries->append(minX, upperLimit);
    upperLimitSeries->append(maxX, upperLimit);

    lowerLimitSeries->append(minX, lowerLimit);
    lowerLimitSeries->append(maxX, lowerLimit); 
    // 新建 scatter series 并设置其 marker size
    QGraphicsTextItem* m_headerText;//显示文字在左上角
    QChart* chart = new QChart();  
    m_headerText = new QGraphicsTextItem(chart);//// 挂到 chart 上
    QStringList data;
    
    QString mean_depth = QStringLiteral("Mean = ") + QString::number(1.000 * Calucate_Width_Depth.m_mean_depth, 'f', 2);
    QString Sigma = "None";
    if (data2.size() > 0)
    {
         Sigma = QStringLiteral("Cpk = ") + QString::number(data2[data2.size() - 1].y(), 'f', 2); 
    } 
    data << mean_depth
        << Sigma;
    setHeaderText(m_headerText, data);
    chart->addSeries(series_A_Depth_Mean);
    chart->addSeries(series_A_Depth_Min);
    chart->addSeries(series_A_Depth_Max);
    chart->addSeries(series_A_Depth_Signma);
    chart->addSeries(upperLimitSeries);
    chart->addSeries(lowerLimitSeries);
    chart->createDefaultAxes();

    // 给Y轴上下各留出10%的边距，避免数据点贴住图表的上下边缘
    const auto allAxes = chart->axes();
    for (auto* axis : allAxes) {
        QValueAxis* valueAxis = qobject_cast<QValueAxis*>(axis);
        if (valueAxis == nullptr)
            continue;
        if (valueAxis->alignment().testFlag(Qt::AlignLeft) || valueAxis->alignment().testFlag(Qt::AlignRight)) {
            double span = valueAxis->max() - valueAxis->min();
            if (span <= 0)
                span = 1.0;
            double margin = span * 0.1;
            valueAxis->setRange(valueAxis->min() - margin, valueAxis->max() + margin);
        }
    }

    chart->setTitle(QStringLiteral("A面槽深结果显示"));
    chart->legend()->setAlignment(Qt::AlignBottom); // 想放底部用 Qt::AlignBottom
    // 1. 标题
    chart->setTitleBrush(QBrush(Qt::white));

    // 2. 图例
    if (chart->legend()) {
        chart->legend()->setLabelBrush(QBrush(Qt::white));
        // 如果想把图例背景也变黑
        // chart->legend()->setBrush(QBrush(QColor(50,50,50)));
    }

    // 3. 所有坐标轴
    const auto axes = chart->axes();
    for (auto* axis : axes) {
        axis->setLabelsBrush(QBrush(Qt::white));   // 刻度文字
        axis->setTitleBrush(QBrush(Qt::white));    // 轴标题
    }
    chart->setBackgroundBrush(QBrush(QColor(50, 50, 50)));   // 整个绘图区域背景
    chart->setPlotAreaBackgroundBrush(QBrush(QColor(50, 50, 50))); // 坐标轴围成的矩形区域
    chart->setPlotAreaBackgroundVisible(true);
   // chart->setMargins(QMargins(10, 10, 10, 10)); 
    
    m_chartView_A_Depth = new ZoomChartView(chart);  
    ly->addWidget(m_chartView_A_Depth); // 注意，这里是 addWidget，不是 addLayout
    /*lowerLimitSeries->deleteLater();
    upperLimitSeries->deleteLater();
    m_headerText->deleteLater();
    chart->deleteLater();*/
}

void PolePieceMeasureWin::show_A_Width_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> width_min, QVector<QPointF> width_max)
{
    if (m_chartView_A_Width) {           // m_chartView_B_Depth 是类成员指针，初始 nullptr
        ui->chartContainer_A_Depth->layout()->removeWidget(m_chartView_A_Width);
        delete m_chartView_A_Width;
        m_chartView_A_Width = nullptr;
    }
    delete ui->chartContainer_A_Width->layout();   // 把旧布局彻底删掉
    // 1. 先给容器加个布局，这样 chartView 能自动撑满
    QVBoxLayout* ly = new QVBoxLayout(ui->chartContainer_A_Width);
    ly->setContentsMargins(0, 0, 0, 0);

    
    series_A_Width_Mean->setName(QStringLiteral("Mean"));
    series_A_Width_Min->setName(QStringLiteral("Min"));
    series_A_Width_Max->setName(QStringLiteral("Max"));
    series_A_Width_Signma->setName(QStringLiteral("CPK"));

    series_A_Width_Mean->setPointsVisible(true);          // 1. 显示端点
    series_A_Width_Mean->setColor(Qt::yellow);
    series_A_Width_Mean->setPen(QPen(QBrush(Qt::yellow), 3));    // 颜色保持原来的蓝
    
    series_A_Width_Min->setPointsVisible(true);          // 1. 显示端点
    series_A_Width_Min->setColor(Qt::red);
    series_A_Width_Min->setPen(QPen(QBrush(Qt::red), 3));    // 颜色保持原来的蓝
    
    series_A_Width_Max->setPointsVisible(true);          // 1. 显示端点
    series_A_Width_Max->setColor(Qt::blue);
    series_A_Width_Max->setPen(QPen(QBrush(Qt::blue), 3));    // 颜色保持原来的蓝
    
    series_A_Width_Signma->setPointsVisible(true);          // 1. 显示端点
    series_A_Width_Signma->setColor(Qt::green);
    series_A_Width_Signma->setPen(QPen(QBrush(Qt::green), 3));    // 颜色保持原来的蓝 
     
    series_A_Width_Mean->clear(); 
    for (int i = 0; i < data1.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Width_Mean->append(i + 1, data1[i].y());
    } 

    series_A_Width_Min->clear();
    for (int i = 0; i < width_min.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Width_Min->append(i + 1, width_min[i].y());
    }

    series_A_Width_Max->clear();
    for (int i = 0; i < width_max.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Width_Max->append(i + 1, width_max[i].y());
    }

    series_A_Width_Signma->clear();
    for (int i = 0; i < data2.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_A_Width_Signma->append(i + 1, data2[i].y());
    }
    //series_A_Width_Mean->replace(data1);
   // series_A_Width_Signma->replace(data21);
    //显示上下限虚线
// 创建上限虚线
    QLineSeries* upperLimitSeries = new QLineSeries();
    upperLimitSeries->setName(QStringLiteral("规格上限"));
    upperLimitSeries->setPen(QPen(Qt::red, 2, Qt::DashLine));

    // 创建下限虚线
    QLineSeries* lowerLimitSeries = new QLineSeries();
    lowerLimitSeries->setName(QStringLiteral("规格下限"));
    lowerLimitSeries->setPen(QPen(Qt::green, 2, Qt::DashLine));

    // 设置上下限值
    double upperLimit = InfoSuanFa.editDetectedWidthMax.toDouble();
    double lowerLimit = InfoSuanFa.editDetectedWidthMin.toDouble();
    double minX = 1, maxX = data1.size(); // X轴范围

    // 添加上下限的数据点
    upperLimitSeries->append(minX, upperLimit);
    upperLimitSeries->append(maxX, upperLimit);

    lowerLimitSeries->append(minX, lowerLimit);
    lowerLimitSeries->append(maxX, lowerLimit); 
    QGraphicsTextItem* m_headerText;//显示文字在左上角
    QChart* chart = new QChart();
    m_headerText = new QGraphicsTextItem(chart);//// 挂到 chart 上
    // 三行数据
    QStringList data;

    QString mean_depth = QStringLiteral("Mean = ") + QString::number(1.000 * Calucate_Width_Depth.m_mean_width, 'f', 2);
    QString Sigma = "None";
    if (data2.size() > 0)
    {
        Sigma = QStringLiteral("Cpk = ") + QString::number(data2[data2.size() - 1].y(), 'f', 2);
    }
    data << mean_depth
        << Sigma; 
    setHeaderText(m_headerText, data);
    chart->addSeries(series_A_Width_Mean);
    chart->addSeries(series_A_Width_Min);
    chart->addSeries(series_A_Width_Max);
    chart->addSeries(series_A_Width_Signma);
    chart->addSeries(upperLimitSeries);
    chart->addSeries(lowerLimitSeries);
    chart->createDefaultAxes();

    // 给Y轴上下各留出10%的边距，避免数据点贴住图表的上下边缘
    const auto allAxes = chart->axes();
    for (auto* axis : allAxes) {
        QValueAxis* valueAxis = qobject_cast<QValueAxis*>(axis);
        if (valueAxis == nullptr)
            continue;
        if (valueAxis->alignment().testFlag(Qt::AlignLeft) || valueAxis->alignment().testFlag(Qt::AlignRight)) {
            double span = valueAxis->max() - valueAxis->min();
            if (span <= 0)
                span = 1.0;
            double margin = span * 0.1;
            valueAxis->setRange(valueAxis->min() - margin, valueAxis->max() + margin);
        }
    }

    chart->setTitle(QStringLiteral("A面槽宽结果显示"));
    chart->legend()->setAlignment(Qt::AlignBottom); // 想放底部用 Qt::AlignBottom
    // 新建 scatter series 并设置其 marker size
   // 1. 标题
    chart->setTitleBrush(QBrush(Qt::white));

    // 2. 图例
    if (chart->legend()) {
        chart->legend()->setLabelBrush(QBrush(Qt::white));
        // 如果想把图例背景也变黑
        // chart->legend()->setBrush(QBrush(QColor(50,50,50)));
    }

    // 3. 所有坐标轴
    const auto axes = chart->axes();
    for (auto* axis : axes) {
        axis->setLabelsBrush(QBrush(Qt::white));   // 刻度文字
        axis->setTitleBrush(QBrush(Qt::white));    // 轴标题
    }
    chart->setBackgroundBrush(QBrush(QColor(50, 50, 50)));   // 整个绘图区域背景
    chart->setPlotAreaBackgroundBrush(QBrush(QColor(50, 50, 50))); // 坐标轴围成的矩形区域
    chart->setPlotAreaBackgroundVisible(true); 

    m_chartView_A_Width = new ZoomChartView(chart); 
    // 3. 塞进容器
    ly->addWidget(m_chartView_A_Width); // 注意，这里是 addWidget，不是 addLayout
    /*lowerLimitSeries->deleteLater();
    upperLimitSeries->deleteLater();
    m_headerText->deleteLater();
    chart->deleteLater();*/
}
 

void PolePieceMeasureWin::show_B_Depth_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> depth_min, QVector<QPointF> depth_max)
{ 
    if (m_chartView_B_Depth) {           // m_chartView_B_Depth 是类成员指针，初始 nullptr
        ui->chartContainer_B_Depth->layout()->removeWidget(m_chartView_B_Depth);
        delete m_chartView_B_Depth;
        m_chartView_B_Depth = nullptr;
    }
    delete ui->chartContainer_B_Depth->layout();   // 把旧布局彻底删掉

    // 1. 先给容器加个布局，这样 chartView 能自动撑满
    QVBoxLayout* ly = new QVBoxLayout(ui->chartContainer_B_Depth);
    ly->setContentsMargins(0, 0, 0, 0);

    // 2. 新建 chart 和 view
    
    series_B_Depth_Mean->setName(QStringLiteral("Mean"));
    series_B_Depth_Min->setName(QStringLiteral("Min"));
    series_B_Depth_Max->setName(QStringLiteral("Max"));
    series_B_Depth_Signma->setName(QStringLiteral("CPK"));

    series_B_Depth_Mean->setPointsVisible(true);          // 1. 显示端点
    series_B_Depth_Mean->setColor(Qt::yellow);              // 3. 连线和端点一起变色
    series_B_Depth_Mean->setPen(QPen(QBrush(Qt::yellow), 3));   // 颜色保持原来的蓝
    
    series_B_Depth_Min->setPointsVisible(true);          // 1. 显示端点
    series_B_Depth_Min->setColor(Qt::red);              // 3. 连线和端点一起变色
    series_B_Depth_Min->setPen(QPen(QBrush(Qt::red), 3));   // 颜色保持原来的蓝
    
    series_B_Depth_Max->setPointsVisible(true);          // 1. 显示端点
    series_B_Depth_Max->setColor(Qt::blue);              // 3. 连线和端点一起变色
    series_B_Depth_Max->setPen(QPen(QBrush(Qt::blue), 3));   // 颜色保持原来的蓝
    
    series_B_Depth_Signma->setPointsVisible(true);          // 1. 显示端点
    series_B_Depth_Signma->setColor(Qt::green);              // 3. 连线和端点一起变色
    series_B_Depth_Signma->setPen(QPen(QBrush(Qt::green), 3));   // 颜色保持原来的蓝
       
    /*if (data1.isEmpty()) return;
    if (depth_min.isEmpty()) return;
    if (depth_max.isEmpty()) return;
    if (data2.size() < 3) data2.clear();*/

    series_B_Depth_Mean->clear();
    for (int i = 0; i < data1.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_B_Depth_Mean->append(i + 1, data1[i].y());
    }

    series_B_Depth_Min->clear();
    for (int i = 0; i < depth_min.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_B_Depth_Min->append(i + 1, depth_min[i].y());
    }

    series_B_Depth_Max->clear();
    for (int i = 0; i < depth_max.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_B_Depth_Max->append(i + 1, depth_max[i].y());
    }

    series_B_Depth_Signma->clear();
    for (int i = 0; i < data2.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_B_Depth_Signma->append(i + 1, data2[i].y());
    }
    // 1. 先把数据准备好
    /*QVector<QPointF> data1 = {
        {0,0.5}, {1,22}, {2,43}, {3,23}, {4,35}, {5,23.5},    
        {6,0.5}, {7,22}, {8,13}, {9,24}, {10,5}, {11,23.5} 
    };
    QVector<QPointF> data2 = {
        {0,1}, {1,3}, {2,4}, {3,3}, {4,25}, {5,26},    
        {6,0.5}, {7,22}, {8,13}, {9,24}, {10,5}, {11,23.5} 
    };*/
   // series_B_Depth_Mean->replace(data1);
    //series_B_Depth_Signma->replace(data2);
    //显示上下限虚线
// 创建上限虚线
    QLineSeries* upperLimitSeries = new QLineSeries();
    upperLimitSeries->setName(QStringLiteral("规格上限"));
    upperLimitSeries->setPen(QPen(Qt::red, 1, Qt::DashLine));

    // 创建下限虚线
    QLineSeries* lowerLimitSeries = new QLineSeries();
    lowerLimitSeries->setName(QStringLiteral("规格下限"));
    lowerLimitSeries->setPen(QPen(Qt::green, 1, Qt::DashLine));

    // 设置上下限值
    double upperLimit = InfoSuanFa.editDetectedDepthMax.toDouble();
    double lowerLimit = InfoSuanFa.editDetectedDepthMin.toDouble();
    double minX = 1, maxX = data1.size(); // X轴范围

    // 添加上下限的数据点
    upperLimitSeries->append(minX, upperLimit);
    upperLimitSeries->append(maxX, upperLimit);

    lowerLimitSeries->append(minX, lowerLimit);
    lowerLimitSeries->append(maxX, lowerLimit);
    QGraphicsTextItem* m_headerText;//显示文字在左上角
    QChart* chart = new QChart();
    m_headerText = new QGraphicsTextItem(chart);//// 挂到 chart 上
    // 三行数据
    QStringList data;
 
    QString mean_depth = QStringLiteral("Mean = ") + QString::number(1.000 * Calucate_Width_Depth_B.m_mean_depth, 'f', 2);
    QString Sigma = "None";
    if (data2.size() > 0)
    {
        Sigma = QStringLiteral("Cpk = ") + QString::number(data2[data2.size() - 1].y(), 'f', 2);
    }
    data << mean_depth
        << Sigma;
    setHeaderText(m_headerText, data);
    chart->addSeries(series_B_Depth_Mean);
    chart->addSeries(series_B_Depth_Min);
    chart->addSeries(series_B_Depth_Max);
    chart->addSeries(series_B_Depth_Signma);
    chart->addSeries(upperLimitSeries);
    chart->addSeries(lowerLimitSeries);
    chart->createDefaultAxes();

    // 给Y轴上下各留出10%的边距，避免数据点贴住图表的上下边缘
    const auto allAxes = chart->axes();
    for (auto* axis : allAxes) {
        QValueAxis* valueAxis = qobject_cast<QValueAxis*>(axis);
        if (valueAxis == nullptr)
            continue;
        if (valueAxis->alignment().testFlag(Qt::AlignLeft) || valueAxis->alignment().testFlag(Qt::AlignRight)) {
            double span = valueAxis->max() - valueAxis->min();
            if (span <= 0)
                span = 1.0;
            double margin = span * 0.1;
            valueAxis->setRange(valueAxis->min() - margin, valueAxis->max() + margin);
        }
    }

    chart->setTitle(QStringLiteral("B面槽深结果显示"));
    chart->legend()->setAlignment(Qt::AlignBottom); // 想放底部用 Qt::AlignBottom
    // 1. 标题
    chart->setTitleBrush(QBrush(Qt::white));

    // 2. 图例
    if (chart->legend()) {
        chart->legend()->setLabelBrush(QBrush(Qt::white));
        // 如果想把图例背景也变黑
        // chart->legend()->setBrush(QBrush(QColor(50,50,50)));
    }

    // 3. 所有坐标轴
    const auto axes = chart->axes();
    for (auto* axis : axes) {
        axis->setLabelsBrush(QBrush(Qt::white));   // 刻度文字
        axis->setTitleBrush(QBrush(Qt::white));    // 轴标题
    }
    chart->setBackgroundBrush(QBrush(QColor(50, 50, 50)));   // 整个绘图区域背景
    chart->setPlotAreaBackgroundBrush(QBrush(QColor(50, 50, 50))); // 坐标轴围成的矩形区域
    chart->setPlotAreaBackgroundVisible(true); 
    m_chartView_B_Depth = new ZoomChartView(chart); 
    ly->addWidget(m_chartView_B_Depth); // 注意，这里是 addWidget，不是 addLayout 

    /*lowerLimitSeries->deleteLater();
    upperLimitSeries->deleteLater();
    m_headerText->deleteLater();
    chart->deleteLater();*/
}
 
void PolePieceMeasureWin::show_B_Width_Line_Chart(QVector<QPointF> data1, QVector<QPointF> data2, QVector<QPointF> width_min, QVector<QPointF> width_max)
{
    if (m_chartView_B_Width) {           // m_chartView_B_Depth 是类成员指针，初始 nullptr
        ui->chartContainer_B_Depth->layout()->removeWidget(m_chartView_B_Width);
        delete m_chartView_B_Width;
        m_chartView_B_Width = nullptr;
    }
    delete ui->chartContainer_B_Width->layout();   // 把旧布局彻底删掉
    // 1. 先给容器加个布局，这样 chartView 能自动撑满
    QVBoxLayout* ly = new QVBoxLayout(ui->chartContainer_B_Width);
    ly->setContentsMargins(0, 0, 0, 0); 

    series_B_Width_Mean->setName(QStringLiteral("Mean"));
    series_B_Width_Min->setName(QStringLiteral("Min"));
    series_B_Width_Max->setName(QStringLiteral("Max"));
    series_B_Width_Signma->setName(QStringLiteral("CPK"));
    series_B_Width_Mean->setPointsVisible(true);          // 1. 显示端点
    series_B_Width_Mean->setColor(Qt::yellow);              // 3. 连线和端点一起变色
    // 1. 先把数据准备好
    series_B_Width_Mean->setPen(QPen(QBrush(Qt::yellow), 3));   // 颜色保持原来的蓝
        
    series_B_Width_Min->setPointsVisible(true);          // 1. 显示端点
    series_B_Width_Min->setColor(Qt::red);              // 3. 连线和端点一起变色
    series_B_Width_Min->setPen(QPen(QBrush(Qt::red), 3));   // 颜色保持原来的蓝

    series_B_Width_Max->setPointsVisible(true);          // 1. 显示端点
    series_B_Width_Max->setColor(Qt::blue);              // 3. 连线和端点一起变色
    series_B_Width_Max->setPen(QPen(QBrush(Qt::blue), 3));   // 颜色保持原来的蓝

    series_B_Width_Signma->setPointsVisible(true);          // 1. 显示端点
    series_B_Width_Signma->setColor(Qt::green);              // 3. 连线和端点一起变色
    series_B_Width_Signma->setPen(QPen(QBrush(Qt::green), 3));   // 颜色保持原来的蓝
        // 1. 先把数据准备好
   
    series_B_Width_Mean->clear();
    for (int i = 0; i < data1.size(); ++i) {
         series_B_Width_Mean->append(i + 1, data1[i].y());
    }

    series_B_Width_Min->clear();
    for (int i = 0; i < width_min.size(); ++i) {
         series_B_Width_Min->append(i + 1, width_min[i].y());
    }

    series_B_Width_Max->clear();
    for (int i = 0; i < width_max.size(); ++i) {
         series_B_Width_Max->append(i + 1, width_max[i].y());
    }

    series_B_Width_Signma->clear();
    for (int i = 0; i < data2.size(); ++i) {
        // 横坐标固定 i+1（1,2,3,4），纵坐标是实际宽度
        series_B_Width_Signma->append(i + 1, data2[i].y());
    }
    // series_B_Width_Mean->replace(data1);
   //  series_B_Width_Signma->replace(data2);
     //显示上下限虚线
 // 创建上限虚线
    QLineSeries* upperLimitSeries = new QLineSeries();
    upperLimitSeries->setName(QStringLiteral("规格上限"));
    upperLimitSeries->setPen(QPen(Qt::red, 1, Qt::DashLine));

    // 创建下限虚线
    QLineSeries* lowerLimitSeries = new QLineSeries();
    lowerLimitSeries->setName(QStringLiteral("规格下限"));
    lowerLimitSeries->setPen(QPen(Qt::green, 1, Qt::DashLine));

    // 设置上下限值
    double upperLimit = InfoSuanFa.editDetectedWidthMax.toDouble();
    double lowerLimit = InfoSuanFa.editDetectedWidthMin.toDouble();
    double minX = 1, maxX = data1.size(); // X轴范围

    // 添加上下限的数据点
    upperLimitSeries->append(minX, upperLimit);
    upperLimitSeries->append(maxX, upperLimit);

    lowerLimitSeries->append(minX, lowerLimit);
    lowerLimitSeries->append(maxX, lowerLimit);
    QGraphicsTextItem* m_headerText;//显示文字在左上角
    QChart* chart = new QChart();
    m_headerText = new QGraphicsTextItem(chart);//// 挂到 chart 上
    // 三行数据
    QStringList data;

    QString mean_depth = QStringLiteral("Mean = ") + QString::number(1.000 * Calucate_Width_Depth_B.m_mean_width, 'f', 2);
    QString Sigma = "None";
    if (data2.size() > 0)
    {
        Sigma = QStringLiteral("Cpk = ") + QString::number(data2[data2.size() - 1].y(), 'f', 2);
    }
    data << mean_depth
        << Sigma;
    setHeaderText(m_headerText, data);
    chart->addSeries(series_B_Width_Mean);
    chart->addSeries(series_B_Width_Min);
    chart->addSeries(series_B_Width_Max);
    chart->addSeries(series_B_Width_Signma);
    chart->addSeries(upperLimitSeries);
    chart->addSeries(lowerLimitSeries);
    chart->createDefaultAxes();

    // 给Y轴上下各留出10%的边距，避免数据点贴住图表的上下边缘
    const auto allAxes = chart->axes();
    for (auto* axis : allAxes) {
        QValueAxis* valueAxis = qobject_cast<QValueAxis*>(axis);
        if (valueAxis == nullptr)
            continue;
        if (valueAxis->alignment().testFlag(Qt::AlignLeft) || valueAxis->alignment().testFlag(Qt::AlignRight)) {
            double span = valueAxis->max() - valueAxis->min();
            if (span <= 0)
                span = 1.0;
            double margin = span * 0.1;
            valueAxis->setRange(valueAxis->min() - margin, valueAxis->max() + margin);
        }
    }

    chart->setTitle(QStringLiteral("B面槽宽结果显示"));
    chart->legend()->setAlignment(Qt::AlignBottom); // 想放底部用 Qt::AlignBottom
    // 新建 scatter series 并设置其 marker size
   // 1. 标题
    chart->setTitleBrush(QBrush(Qt::white));

    // 2. 图例
    if (chart->legend()) {
        chart->legend()->setLabelBrush(QBrush(Qt::white));
        // 如果想把图例背景也变黑
        // chart->legend()->setBrush(QBrush(QColor(50,50,50)));
    }

    // 3. 所有坐标轴
    const auto axes = chart->axes();
    for (auto* axis : axes) {
        axis->setLabelsBrush(QBrush(Qt::white));   // 刻度文字
        axis->setTitleBrush(QBrush(Qt::white));    // 轴标题
    }
    chart->setBackgroundBrush(QBrush(QColor(50, 50, 50)));   // 整个绘图区域背景
    chart->setPlotAreaBackgroundBrush(QBrush(QColor(50, 50, 50))); // 坐标轴围成的矩形区域
    chart->setPlotAreaBackgroundVisible(true);
    m_chartView_B_Width = new ZoomChartView(chart);
    // 3. 塞进容器
    ly->addWidget(m_chartView_B_Width); // 注意，这里是 addWidget，不是 addLayout

   /* lowerLimitSeries->deleteLater();
    upperLimitSeries->deleteLater();
    m_headerText->deleteLater();
    chart->deleteLater();*/

}
void PolePieceMeasureWin::setHeaderText(QGraphicsTextItem* m_headerText, const QStringList& lines)//左上角显示平均值，标准差和Cpk
{
    /* 1. 拼成多行文字 */
    QString text = lines.join("\n");

    /* 2. 设置内容 */
    m_headerText->setPlainText(text);

    /* 3. 可选：整体再微调位置（保证第一行左上角 10,10） */
    m_headerText->setPos(10, 10);

    /* 4. 可选：字体、颜色、行高 */
    QFont f("Arial", 10, QFont::Bold);
    m_headerText->setFont(f);
    m_headerText->setDefaultTextColor(Qt::white);

}
void PolePieceMeasureWin::InitLineChartShow()
{ 

         series_A_Depth_Mean = new QLineSeries();
         series_A_Depth_Min = new QLineSeries();
         series_A_Depth_Max = new QLineSeries();
         series_A_Depth_Signma = new QLineSeries();
         series_A_Width_Mean = new QLineSeries();
         series_A_Width_Min = new QLineSeries();
         series_A_Width_Max = new QLineSeries();
         series_A_Width_Signma = new QLineSeries(); 
         series_B_Depth_Mean = new QLineSeries();
         series_B_Depth_Min = new QLineSeries();
         series_B_Depth_Max = new QLineSeries();
         series_B_Depth_Signma = new QLineSeries();
         series_B_Width_Mean = new QLineSeries();
         series_B_Width_Min = new QLineSeries();
         series_B_Width_Max = new QLineSeries();
         series_B_Width_Signma = new QLineSeries(); 
         upperSeries = new QLineSeries();
         lowerSeries = new QLineSeries();
    /*if (series_A_Depth_Mean == nullptr)
    {
        series_A_Depth_Mean = new QLineSeries();
    }
    if (series_A_Depth_Min == nullptr)
    {
        series_A_Depth_Min = new QLineSeries();
    }
    if (series_A_Depth_Max == nullptr)
    {
        series_A_Depth_Max = new QLineSeries();
    }
    if (series_A_Depth_Signma == nullptr)
    {
        series_A_Depth_Signma = new QLineSeries();
    }
    if (series_A_Width_Mean == nullptr)
    {
        series_A_Width_Mean = new QLineSeries();
    }
    if (series_A_Width_Min == nullptr)
    {
        series_A_Width_Min = new QLineSeries();
    }
    if (series_A_Width_Max == nullptr)
    {
        series_A_Width_Max = new QLineSeries();
    }
    if (series_A_Width_Signma == nullptr)
    {
        series_A_Width_Signma = new QLineSeries();
    }
    if (series_B_Depth_Mean == nullptr)
    {
        series_B_Depth_Mean = new QLineSeries();
    }
    if (series_B_Depth_Min == nullptr)
    {
        series_B_Depth_Min = new QLineSeries();
    }
    if (series_B_Depth_Max == nullptr)
    {
        series_B_Depth_Max = new QLineSeries();
    }
    if (series_B_Depth_Signma == nullptr)
    {
        series_B_Depth_Signma = new QLineSeries();
    }
    if (series_B_Width_Mean == nullptr)
    {
        series_B_Width_Mean = new QLineSeries();
    }
    if (series_B_Width_Min == nullptr)
    {
        series_B_Width_Min = new QLineSeries();
    }
    if (series_B_Width_Max == nullptr)
    {
        series_B_Width_Max = new QLineSeries();
    }
    if (series_B_Width_Signma == nullptr)
    {
        series_B_Width_Signma = new QLineSeries();
    }
    if (upperSeries == nullptr)
    {
        upperSeries = new QLineSeries();
    }
    if (lowerSeries == nullptr)
    {
        lowerSeries = new QLineSeries();
    }*/
}

void PolePieceMeasureWin::initIniSettingParam()//加载设置参数
{
    //通信界面参数设置
    CommunicateDialog = new QDialog(this);//#include "ui_DataExcel.h" 
    CommunicateUi = new Ui::CommunicateClass();
    CommunicateUi->setupUi(CommunicateDialog);
    connect(CommunicateUi->btnConnectServer, &QPushButton::clicked, this, [this]() {
        iniSettingParam();
        on_btnConnectServer_clicked();
        });
    connect(CommunicateUi->btnDisConnectServer, &QPushButton::clicked, this, [this]() {
        iniSettingParam();
        on_btnDisConnectServer_clicked();
        });
    connect(CommunicateUi->btnSendMessage, &QPushButton::clicked, this, [this]() {
        iniSettingParam();
        on_btnSendMessage_clicked();
        });
    //表格数据显示
    excelDialog = new QDialog(this);//#include "ui_DataExcel.h" 
    DataExcelUi = new Ui::DataExcelClass();
    DataExcelUi->setupUi(excelDialog);
    /////////////////////////////参数设置界面
    QSettings ini(iniPath, QSettings::IniFormat);
    ParametersDialog = new QDialog(this);
    ParamUi = new Ui::SubWindowClass();
    ParamUi->setupUi(ParametersDialog);
    ParamUi->editCameraIP->setText(ini.value(QStringLiteral("SettingParam/editCameraIP")).toString());
    ParamUi->editCameraIP_B->setText(ini.value(QStringLiteral("SettingParam/editCameraIP_B")).toString());
    ParamUi->editOutPath->setText(ini.value(QStringLiteral("SettingParam/editOutPath")).toString());
    ParamUi->editOutPath_Result->setText(ini.value(QStringLiteral("SettingParam/editOutPath_Result")).toString());
    ParamUi->editPLCIP->setText(ini.value(QStringLiteral("SettingParam/editPLCIP")).toString());
    ParamUi->editPLCPort->setText(ini.value(QStringLiteral("SettingParam/editPLCPort")).toString());
    ParamUi->editEdge->setText(ini.value(QStringLiteral("SettingParam/editEdge")).toString());
    ParamUi->editCheckThreshold->setText(ini.value(QStringLiteral("SettingParam/editCheckThreshold")).toString());
    ParamUi->editROI->setText(ini.value(QStringLiteral("SettingParam/editROI")).toString());
    ParamUi->editPingHuaData->setText(ini.value(QStringLiteral("SettingParam/editPingHuaData")).toString());
    ParamUi->editROI_2->setText(ini.value(QStringLiteral("SettingParam/editROI_2")).toString());
    ParamUi->editbottom->setText(ini.value(QStringLiteral("SettingParam/editbottom")).toString());
    ParamUi->editDetectedDepthMin->setText(ini.value(QStringLiteral("SettingParam/editDetectedDepthMin")).toString());
    ParamUi->editDetectedDepthMax->setText(ini.value(QStringLiteral("SettingParam/editDetectedDepthMax")).toString());
    ParamUi->editDetectedWidthMin->setText(ini.value(QStringLiteral("SettingParam/editDetectedWidthMin")).toString());
    ParamUi->editDetectedWidthMax->setText(ini.value(QStringLiteral("SettingParam/editDetectedWidthMax")).toString());
    ParamUi->editMaxWidth->setText(ini.value(QStringLiteral("SettingParam/editMaxWidth")).toString());
    ParamUi->editRANSAC->setText(ini.value(QStringLiteral("SettingParam/editRANSAC")).toString());
    ParamUi->editAngle->setText(ini.value(QStringLiteral("SettingParam/editAngle")).toString()); 
    ParamUi->edit_Depth_A_Coefficient->setText(ini.value(QStringLiteral("SettingParam/edit_Depth_A_Coefficient")).toString());
    ParamUi->edit_Width_A_Coefficient->setText(ini.value(QStringLiteral("SettingParam/edit_Width_A_Coefficient")).toString());
    ParamUi->edit_Depth_B_Coefficient->setText(ini.value(QStringLiteral("SettingParam/edit_Depth_B_Coefficient")).toString());
    ParamUi->edit_Width_B_Coefficient->setText(ini.value(QStringLiteral("SettingParam/edit_Width_B_Coefficient")).toString());
    ParamUi->editCloudShowDis->setText(ini.value(QStringLiteral("SettingParam/editCloudShowDis")).toString());
    ParamUi->editRadius->setText(ini.value(QStringLiteral("SettingParam/editRadius")).toString());//左肩点斜率控制
    ParamUi->editTorlance->setText(ini.value(QStringLiteral("SettingParam/editTorlance")).toString());//右肩点斜率控制

    DataExcelUi->editPath_A->setText(ini.value(QStringLiteral("SettingParam/editPath_A")).toString());
    DataExcelUi->editPath_B->setText(ini.value(QStringLiteral("SettingParam/editPath_B")).toString());
    //通信界面INI参数设置
    CommunicateUi->editServerIP->setText(ini.value(QStringLiteral("SettingParam/editServerIP")).toString());
    CommunicateUi->editServerPort->setText(ini.value(QStringLiteral("SettingParam/editServerPort")).toString());
    //保存天数
    int day = ini.value(QStringLiteral("SettingParam/SaveCloudDays")).toInt();
    if (day == 0)
    {
        SaveCloudDays = 2;//默认保存天数为2天
    }
    else
    {
        SaveCloudDays=ini.value(QStringLiteral("SettingParam/SaveCloudDays")).toInt();
    }
    int day_B = ini.value(QStringLiteral("SettingParam/SaveCloudDays_B")).toInt(); //= ini.value(QStringLiteral("SettingParam/SaveCloudDays_B")).toInt();
    if (day_B == 0)
    {
        SaveCloudDays_B = 2;//默认保存天数为2天
    }
    else
    {
        SaveCloudDays_B = ini.value(QStringLiteral("SettingParam/SaveCloudDays_B")).toInt();
    }
   
      
}
void PolePieceMeasureWin::iniSettingParam()//加载设置参数
{  
        InfoCamera.Camra_A_IP = ParamUi->editCameraIP->text();
        InfoCamera.Camra_B_IP = ParamUi->editCameraIP_B->text();
        InfoCamera.PointOutPath = ParamUi->editOutPath->text();
        InfoCamera.PointOutPath_B = ParamUi->editOutPath_Result->text();
        InfoCamera.CameraConnectState_A = ParamUi->CameraConnectState->text();
        InfoCamera.CameraConnectState_B = ParamUi->CameraConnectState_B->text();
		InfoHMI.HMI_IP = CommunicateUi->editServerIP->text();//HMI服务器IP
		InfoHMI.HMI_PORT = CommunicateUi->editServerPort->text();//HMI服务器IP

        InfoPLC.PLC_IP = ParamUi->editPLCIP->text();
        InfoPLC.PLC_PORT = ParamUi->editPLCPort->text();
        InfoPLC.PLCConnectState = ParamUi->PLCConnectState->text();

        InfoSuanFa.editEdge = ParamUi->editEdge->text();
        InfoSuanFa.editCheckThreshold = ParamUi->editCheckThreshold->text();
        InfoSuanFa.editRoiPercent = ParamUi->editROI->text();
        InfoSuanFa.editPingHuaXiShu = ParamUi->editPingHuaData->text();
        InfoSuanFa.editCentrePoint = ParamUi->editROI_2->text();
        InfoSuanFa.editGaoDuChaThreshold = ParamUi->editbottom->text();
        InfoSuanFa.editCloudShowDis = ParamUi->editCloudShowDis->text();
        InfoSuanFa.editDetectedDepthMin = ParamUi->editDetectedDepthMin->text();
        InfoSuanFa.editDetectedDepthMax = ParamUi->editDetectedDepthMax->text();
        InfoSuanFa.editDetectedWidthMin = ParamUi->editDetectedWidthMin->text();
        InfoSuanFa.editDetectedWidthMax = ParamUi->editDetectedWidthMax->text();
        InfoSuanFa.editMaxWidth = ParamUi->editMaxWidth->text();
        InfoSuanFa.editRANSAC = ParamUi->editRANSAC->text();
        InfoSuanFa.editAngle = ParamUi->editAngle->text();
        
        InfoSuanFa.edit_Depth_A_Coefficient= ParamUi->edit_Depth_A_Coefficient->text();//A面槽深系数
        InfoSuanFa.edit_Width_A_Coefficient= ParamUi->edit_Width_A_Coefficient->text();//A面槽宽系数
        InfoSuanFa.edit_Depth_B_Coefficient= ParamUi->edit_Depth_B_Coefficient->text();//B面槽深系数
        InfoSuanFa.edit_Width_B_Coefficient= ParamUi->edit_Width_B_Coefficient->text();//B面槽宽系数 

        InfoExcel.excel_A_OutPath = DataExcelUi->editPath_A->text();
        InfoExcel.excel_B_OutPath = DataExcelUi->editPath_B->text();
}
void PolePieceMeasureWin::destroySettingParam()//保存设置参数
{
    QSettings ini(iniPath, QSettings::IniFormat); 

    ini.setValue(QStringLiteral("SettingParam/editOutPath"), ParamUi->editOutPath->text());
    ini.setValue(QStringLiteral("SettingParam/editOutPath_Result"), ParamUi->editOutPath_Result->text());
    ini.setValue(QStringLiteral("SettingParam/editCameraIP"), ParamUi->editCameraIP->text());
    ini.setValue(QStringLiteral("SettingParam/editCameraIP_B"), ParamUi->editCameraIP_B->text());
    ini.setValue(QStringLiteral("SettingParam/editPLCIP"), ParamUi->editPLCIP->text());
    ini.setValue(QStringLiteral("SettingParam/editPLCPort"), ParamUi->editPLCPort->text());
    ini.setValue(QStringLiteral("SettingParam/editEdge"), ParamUi->editEdge->text());
    ini.setValue(QStringLiteral("SettingParam/editCheckThreshold"), ParamUi->editCheckThreshold->text());
    ini.setValue(QStringLiteral("SettingParam/editROI"), ParamUi->editROI->text());
    ini.setValue(QStringLiteral("SettingParam/editPingHuaData"), ParamUi->editPingHuaData->text());
    ini.setValue(QStringLiteral("SettingParam/editROI_2"), ParamUi->editROI_2->text());
    ini.setValue(QStringLiteral("SettingParam/editbottom"), ParamUi->editbottom->text());
    ini.setValue(QStringLiteral("SettingParam/editDetectedDepthMin"), ParamUi->editDetectedDepthMin->text());
    ini.setValue(QStringLiteral("SettingParam/editDetectedDepthMax"), ParamUi->editDetectedDepthMax->text());
    ini.setValue(QStringLiteral("SettingParam/editDetectedWidthMin"), ParamUi->editDetectedWidthMin->text());
    ini.setValue(QStringLiteral("SettingParam/editDetectedWidthMax"), ParamUi->editDetectedWidthMax->text());
    ini.setValue(QStringLiteral("SettingParam/editMaxWidth"), ParamUi->editMaxWidth->text());
    ini.setValue(QStringLiteral("SettingParam/editRANSAC"), ParamUi->editRANSAC->text());
    ini.setValue(QStringLiteral("SettingParam/editAngle"), ParamUi->editAngle->text()); 

    ini.setValue(QStringLiteral("SettingParam/edit_Depth_A_Coefficient"), ParamUi->edit_Depth_A_Coefficient->text()); 
    ini.setValue(QStringLiteral("SettingParam/edit_Width_A_Coefficient"), ParamUi->edit_Width_A_Coefficient->text()); 
    ini.setValue(QStringLiteral("SettingParam/edit_Depth_B_Coefficient"), ParamUi->edit_Depth_B_Coefficient->text()); 
    ini.setValue(QStringLiteral("SettingParam/edit_Width_B_Coefficient"), ParamUi->edit_Width_B_Coefficient->text()); 
    ini.setValue(QStringLiteral("SettingParam/editCloudShowDis"), ParamUi->editCloudShowDis->text());    
    ini.setValue(QStringLiteral("SettingParam/editRadius"), ParamUi->editRadius->text());//左肩点斜率
    ini.setValue(QStringLiteral("SettingParam/editTorlance"), ParamUi->editTorlance->text());//右肩点斜率

	//表格数据显示路径
    ini.setValue(QStringLiteral("SettingParam/editPath_A"), DataExcelUi->editPath_A->text());
    ini.setValue(QStringLiteral("SettingParam/editPath_B"), DataExcelUi->editPath_B->text());
	//通信界面INI参数设置
    ini.setValue(QStringLiteral("SettingParam/editServerIP"), CommunicateUi->editServerIP->text());
    ini.setValue(QStringLiteral("SettingParam/editServerPort"), CommunicateUi->editServerPort->text());

    InfoSuanFa.editRadius = ParamUi->editRadius->text();
    InfoSuanFa.editTorlance = ParamUi->editTorlance->text();

    InfoCamera.Camra_A_IP = ParamUi->editCameraIP->text();
    InfoCamera.Camra_B_IP = ParamUi->editCameraIP_B->text();
    InfoCamera.PointOutPath = ParamUi->editOutPath->text();
    InfoCamera.PointOutPath_B = ParamUi->editOutPath_Result->text();
    InfoCamera.CameraConnectState_A = QStringLiteral("");
    InfoCamera.CameraConnectState_B = QStringLiteral("");
    InfoPLC.PLC_IP = ParamUi->editPLCIP->text();
    InfoPLC.PLC_PORT = ParamUi->editPLCPort->text();
    InfoPLC.PLCConnectState = QStringLiteral("");
    InfoHMI.HMI_IP = CommunicateUi->editServerIP->text();
    InfoHMI.HMI_PORT = CommunicateUi->editServerPort->text();
    InfoHMI.HMIConnectState = QStringLiteral("");

    InfoSuanFa.editEdge = ParamUi->editEdge->text();
    InfoSuanFa.editCheckThreshold = ParamUi->editCheckThreshold->text();
    InfoSuanFa.editRoiPercent = ParamUi->editROI->text();
    InfoSuanFa.editPingHuaXiShu = ParamUi->editPingHuaData->text();
    InfoSuanFa.editCentrePoint = ParamUi->editROI_2->text();
    InfoSuanFa.editGaoDuChaThreshold = ParamUi->editbottom->text();
    InfoSuanFa.editCloudShowDis = ParamUi->editCloudShowDis->text();
    InfoSuanFa.editDetectedDepthMin = ParamUi->editDetectedDepthMin->text();
    InfoSuanFa.editDetectedDepthMax = ParamUi->editDetectedDepthMax->text();
    InfoSuanFa.editDetectedWidthMin = ParamUi->editDetectedWidthMin->text();
    InfoSuanFa.editDetectedWidthMax = ParamUi->editDetectedWidthMax->text();
    InfoSuanFa.editMaxWidth = ParamUi->editMaxWidth->text();
    InfoSuanFa.editRANSAC = ParamUi->editRANSAC->text();
    InfoSuanFa.editAngle = ParamUi->editAngle->text();

    InfoSuanFa.edit_Depth_A_Coefficient = ParamUi->edit_Depth_A_Coefficient->text();//A面槽深系数
    InfoSuanFa.edit_Width_A_Coefficient = ParamUi->edit_Width_A_Coefficient->text();//A面槽宽系数
    InfoSuanFa.edit_Depth_B_Coefficient = ParamUi->edit_Depth_B_Coefficient->text();//B面槽深系数
    InfoSuanFa.edit_Width_B_Coefficient = ParamUi->edit_Width_B_Coefficient->text();//B面槽宽系数  

    InfoExcel.excel_A_OutPath = DataExcelUi->editPath_A->text();
    InfoExcel.excel_B_OutPath = DataExcelUi->editPath_B->text();
}
void PolePieceMeasureWin::on_btnSetting_clicked()//设置
{ 
    iniSettingParam();
     //基础配置
    ParametersDialog->setWindowTitle("Param Setting");
    ParametersDialog->setWindowFlags(ParametersDialog->windowFlags() & ~Qt::WindowContextHelpButtonHint); 
   
     //信号与槽
    connect(ParamUi->btnConnectCamera, &QPushButton::clicked, [this]() { 
        iniSettingParam(); 
        on_btnConnectCamera_clicked(); 
        }); 
    connect(ParamUi->btnDisConnectCamera, &QPushButton::clicked, [this]() {   
        iniSettingParam(); 
        on_btnDisConnectCamera_clicked(); 
        });
    
    connect(ParamUi->btnConnectCamera_B, &QPushButton::clicked, [this]() {   
        iniSettingParam(); 
        on_btnConnectCamera_B_clicked(); 
        }); 
    connect(ParamUi->btnDisConnectCamera_B, &QPushButton::clicked, [this]() {    
        iniSettingParam(); 
        on_btnDisConnectCamera_B_clicked(); 
        });
    
    connect(ParamUi->btnConnectPLC, &QPushButton::clicked, [this]() {   
        iniSettingParam(); 
        on_btnConnectPLC_clicked(); 
        });
    
    //connect(ParamUi->btnDisConnectPLC, &QPushButton::clicked, [this, ParametersDialog, ParamUi]() {
    connect(ParamUi->btnDisConnectPLC, &QPushButton::clicked, [this]() {   
        iniSettingParam(); 
        on_btnDisConnectPLC_clicked(); 
        });
    
    connect(ParamUi->btnOutPutDirectory, &QPushButton::clicked, [this]() {  
        iniSettingParam(); 
        on_btnOutPutDirectory_clicked();
        });
    connect(ParamUi->btnOutPutDirectory_Result, &QPushButton::clicked, [this]() {
        iniSettingParam(); 
        on_btnOutPutDirectory_Result_clicked();
        });
    
    connect(ParamUi->btnParamSave, &QPushButton::clicked, [this]() {   
        iniSettingParam(); 
        destroySettingParam();
        QMessageBox::information(this, "Save Settings", "Settings saved successfully.");

        });
    connect(ParamUi->btnParamRead, &QPushButton::clicked, [this]() {
        initIniSettingParam();
        QMessageBox::information(this, "Save Settings", "Settings read successfully.");

        });
    
    connect(ParamUi->btnStartCollectPoint, &QPushButton::clicked, [this]() {
        iniSettingParam();
        on_btnStartCollectPoint_clicked();
          });
    connect(ParamUi->btnStopCollectPoint, &QPushButton::clicked, [this]() {
        iniSettingParam();
        on_btnStopCollectPoint_clicked();
          });
    
    connect(ParamUi->btnSavePoint, &QPushButton::clicked, [this]() {
        iniSettingParam();
        on_btnSavePoint_clicked();
          }); 

    //执行对话框，结束后自动释放内存
    if (ParametersDialog->exec() == QDialog::Accepted) {
        // 可扩展登录成功后的逻辑

    }
    
}
void PolePieceMeasureWin::on_btnSignIn_clicked()//登录
{ 
    //使用堆分配对话框和 UI，避免栈对象生命周期问题
    QDialog* loginDialog = new QDialog(this);
    Ui::LogIn* loginUi = new Ui::LogIn();
    loginUi->setupUi(loginDialog);
    //基础配置
    loginDialog->setWindowTitle("Login");
    loginDialog->setWindowFlags(loginDialog->windowFlags() & ~Qt::WindowContextHelpButtonHint);
    loginUi->stackedWidget->setCurrentIndex(0);

    // 密码初始化（从 ini 读取或设置默认）
    QSettings settings(iniPath, QSettings::IniFormat);
    if (!settings.contains("Robot/password")) {
        QString defaultPassword = "123";
        currentPassword = hashPassword(defaultPassword);
        settings.setValue("Robot/password", currentPassword);
        settings.sync();
    }
    else {
        currentPassword = settings.value("Robot/password").toString();
    } 
    // 切换到改密码页面
    connect(loginUi->btnChangePassword, &QPushButton::clicked, [loginUi]() {
        loginUi->stackedWidget->setCurrentIndex(1);
        }); 
    //登录验证
    connect(loginUi->btnLogin, &QPushButton::clicked, [this, loginDialog, loginUi]() {
        QString username = loginUi->editUserName->text().trimmed();
        QString password = loginUi->editPassword->text().trimmed(); 
        if (username == "admin" && hashPassword(password) == currentPassword) {
            // 登录成功：更新主窗口用户状态
           // ui->labelCurrentUser->setText(tr("admin"));
           // ui->labelCurrentUser->setStyleSheet("color: red;");
            onLogin();
            loginDialog->accept();
        }
        else {
            // 登录失败：重置主窗口用户状态
          //  ui->labelCurrentUser->setText(tr("operator"));
           // ui->labelCurrentUser->setStyleSheet("");
            onLogout();
            QMessageBox::warning(loginDialog, tr("login failed"), tr("user name or password err!"));
        }
        });

    //  确认修改密码
    connect(loginUi->btnConfirm, &QPushButton::clicked, [this, loginDialog, loginUi]() {
        QString username = loginUi->editUserName2->text().trimmed();
        QString oldPassword = loginUi->editPassword2->text().trimmed();
        QString newPassword = loginUi->editPasswordNew->text().trimmed();
        QString repeatPassword = loginUi->editPasswordNewRepeat->text().trimmed();

        // 验证旧密码
        if (username != "admin" || hashPassword(oldPassword) != currentPassword) {
            QMessageBox::warning(loginDialog, tr("change failed"), tr("user name or password err!"));
            return;
        }
        // 验证新密码
        if (newPassword.isEmpty()) {
            QMessageBox::warning(loginDialog, tr("change failed"), tr("new password cannot be empty!"));
            return;
        }
        if (newPassword != repeatPassword) {
            QMessageBox::warning(loginDialog, tr("change failed"), tr("new password donot match!"));
            return;
        }

        // 更新密码并写入配置
        currentPassword = hashPassword(newPassword);
        QSettings settings(iniPath, QSettings::IniFormat);
        settings.setValue("Robot/password", currentPassword);

        // 反馈给用户
       // ui->labelCurrentUser->setText(tr("operator"));
       // ui->labelCurrentUser->setStyleSheet("");
        onLogout();
        QMessageBox::information(loginDialog, tr("change successful"), tr("change successful,login with new password!"));

        // 重置页面
        loginUi->stackedWidget->setCurrentIndex(0);
        loginUi->editPasswordNew->clear();
        loginUi->editPasswordNewRepeat->clear();
        });

    // 小眼睛按钮逻辑 太丑了 没要了
    //loginUi->toolBtnPassword->setIcon(QIcon(imagePath));
    loginUi->toolBtnPassword->setToolTip(tr("show password"));
    loginUi->toolBtnPassword->setCheckable(false);
    connect(loginUi->toolBtnPassword, &QToolButton::pressed, [loginUi]() {
        loginUi->editPassword->setEchoMode(QLineEdit::Normal);
        });
    connect(loginUi->toolBtnPassword, &QToolButton::released, [loginUi]() {
        loginUi->editPassword->setEchoMode(QLineEdit::Password);
        });

    //执行对话框，结束后自动释放内存
    if (loginDialog->exec() == QDialog::Accepted) {
        // 可扩展登录成功后的逻辑
    }
    delete loginUi;
    delete loginDialog;

}

// 哈希存储
QString PolePieceMeasureWin::hashPassword(const QString& password) {
    QByteArray data = password.toUtf8();
    QByteArray hash = QCryptographicHash::hash(data, QCryptographicHash::Sha1);
    return hash.toHex();  // 转换为十六进制字符串存储
}
void PolePieceMeasureWin::onLogin()
{

}
void PolePieceMeasureWin::onLogout()
{

} 
QVector<QPointF> PolePieceMeasureWin::cloudToXZ(const pcl::PointCloud<pcl::PointXYZ>& cloud, double scaleValue)
{
    QVector<QPointF> vec;
    vec.reserve(cloud.size());
    for (const auto& p : cloud)
        vec.append(QPointF(static_cast<qreal>(p.x* scaleValue),
            static_cast<qreal>(p.z*100)));
    return vec;
}

void PolePieceMeasureWin::on_btnLength_A_Picture_clicked()
{
    // // 打开文件选择对话框，支持的格式为PLY和PCD
    //QString fileName = QFileDialog::getOpenFileName(this, tr("Open Point Cloud File"), "", tr("Point Cloud Files (*.ply *.pcd)"));

    //// 如果未选择文件，则返回
    //if (fileName.isEmpty()) {
    //    return;
    //}

    //// 获取文件扩展名，以区分文件格式
    //QFileInfo fileInfo(fileName);
    //QString ext = fileInfo.suffix().toLower();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    //// 根据扩展名加载相应格式的点云文件
    //int result = -1;
    //if (ext == "ply")
    //{
    //    result = pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toStdString(), *cloud);
    //}
    //else if (ext == "pcd")
    //{
    //    result = pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud);
    //}
    //else
    //{
    //    return;
    //}

    //// 检查文件加载是否成功
    //if (result != 0)
    //{
    //    QMessageBox::critical(this, tr("Error"), tr("无法打开点云文件（路径需要全英文）"));
    //    return;
    //} 


//   // double WidthZoom = ui->editValueZoom->text().toDouble();
//        QVector<QPointF> pts = cloudToXZ(*cloud,1.000);
//
//    InitShow();
//    if (m_chartView_B_Depth) {           // m_chartView_B_Depth 是类成员指针，初始 nullptr
//        ui->Single_A_PointShow->layout()->removeWidget(m_chartView_B_Depth);
//        delete m_chartView_B_Depth;
//        m_chartView_B_Depth = nullptr;
//    }
//    delete ui->Single_A_PointShow->layout();   // 把旧布局彻底删掉
//
//    // 1. 先给容器加个布局，这样 chartView 能自动撑满
//    QVBoxLayout* ly = new QVBoxLayout(ui->Single_A_PointShow);
//    ly->setContentsMargins(0, 0, 0, 0);
//
//    // 2. 新建 chart 和 view
//
//    series_B_Depth_Mean->setName(QStringLiteral("Mean"));
//    series_B_Depth_Signma->setName(QStringLiteral("SignMa"));
//    /* double WidthZoom = ui->editValueZoom->text().toDouble();
//    double HeightZoom = ui->editValueZoomHeight->text().toDouble();*/
//    // 1. 先把数据准备好
//    /*QVector<QPointF> data1 = {
//        {0* WidthZoom,0.5}, {1* WidthZoom,22}, {2* WidthZoom,43}, {3* WidthZoom,23}, {4* WidthZoom,35}, {5* WidthZoom,23.5},
//        {6* WidthZoom,0.5}, {7* WidthZoom,22}, {8* WidthZoom,13}, {9* WidthZoom,24}, {10* WidthZoom,5}, {11* WidthZoom,23.5}
//    };*/
//    QVector<QPointF> data2 = {
//        {0,0}//, {1,0}, {2,0}, {3,0}, {4,0}, {5,0},
//      //  {6,0}, {7,0}, {8,0}, {9,0}, {10,0}, {11,0}
//    };
//   /* QVector<QPointF> data2 = {
//        {0,1}, {1,3}, {2,4}, {3,3}, {4,25}, {5,26},
//        {6,0.5}, {7,22}, {8,13}, {9,24}, {10,5}, {11,23.5}
//    };*/
//    series_B_Depth_Mean->replace(pts);
//    series_B_Depth_Signma->replace(data2);
//    //显示上下限虚线
//// 创建上限虚线
//    QLineSeries* upperLimitSeries = new QLineSeries();
//    upperLimitSeries->setName(QStringLiteral("规格上限"));
//    upperLimitSeries->setPen(QPen(Qt::red, 1, Qt::DashLine));
//
//    // 创建下限虚线
//    QLineSeries* lowerLimitSeries = new QLineSeries();
//    lowerLimitSeries->setName(QStringLiteral("规格下限"));
//    lowerLimitSeries->setPen(QPen(Qt::blue, 1, Qt::DashLine));
//
//    // 设置上下限值
//    double upperLimit = 130.0;
//    double lowerLimit = 20.0;
//    double minX = 0, maxX = pts.size(); // X轴范围
//
//    // 添加上下限的数据点
//    upperLimitSeries->append(minX, upperLimit);
//    upperLimitSeries->append(maxX, upperLimit);
//
//    lowerLimitSeries->append(minX, lowerLimit);
//    lowerLimitSeries->append(maxX, lowerLimit);
//
//    // 2. 轴：先给一份“正圆”比例
//    QValueAxis* axisX = new QValueAxis;
//    QValueAxis* axisY = new QValueAxis;
//    int WidthZoom = ui->editValueZoom->text().toInt();
//    int HeightZoom = ui->editValueZoomHeight->text().toInt();
//    axisX->setRange(0, WidthZoom);
//    axisY->setRange(0, HeightZoom);
//
//    QChart* chart = new QChart();
//    chart->addSeries(series_B_Depth_Mean);
//    chart->addSeries(series_B_Depth_Signma);
//    chart->addSeries(upperLimitSeries);
//    chart->addSeries(lowerLimitSeries);
// //   chart->addAxis(axisX, Qt::AlignBottom);
//   // chart->addAxis(axisY, Qt::AlignLeft);
//    chart->createDefaultAxes();
//    
//     chart->setTitle(QStringLiteral("B面槽深结果显示"));
//    chart->legend()->setAlignment(Qt::AlignRight); // 想放底部用 Qt::AlignBottom
//    // 1. 标题
//    chart->setTitleBrush(QBrush(Qt::white));
//
//    // 2. 图例
//    if (chart->legend()) {
//        chart->legend()->setLabelBrush(QBrush(Qt::white));
//        // 如果想把图例背景也变黑
//        // chart->legend()->setBrush(QBrush(QColor(50,50,50)));
//    }
//
//    // 3. 所有坐标轴
//    const auto axes = chart->axes();
//    for (auto* axis : axes) {
//        axis->setLabelsBrush(QBrush(Qt::white));   // 刻度文字
//        axis->setTitleBrush(QBrush(Qt::white));    // 轴标题
//    }
//    chart->setBackgroundBrush(QBrush(QColor(50, 50, 50)));   // 整个绘图区域背景
//    chart->setPlotAreaBackgroundBrush(QBrush(QColor(50, 50, 50))); // 坐标轴围成的矩形区域
//    chart->setPlotAreaBackgroundVisible(true);
//
//    // 新建 scatter series 并设置其 marker size
//    QScatterSeries* scatterSeries_B_Depth = new QScatterSeries();
//    scatterSeries_B_Depth->setName(QStringLiteral("数据点"));
//    scatterSeries_B_Depth->setMarkerSize(10);   // 设置标记点大小 
//    scatterSeries_B_Depth->setMarkerShape(QScatterSeries::MarkerShapeRectangle); // 设置标记点形状为矩形
//   // scatterSeries_B_Depth->append(series_B_Depth_Mean->points());  // 使用与 line series 相同的数据点
//   // scatterSeries_B_Depth->append(series_B_Depth_Signma->points());  // 使用与 line series 相同的数据点
//
//    // 将 QScatterSeries 添加到图表中
//    chart->addSeries(scatterSeries_B_Depth);
//
//    m_chartView_B_Depth = new ZoomChartView(chart);
//    m_chartView_B_Depth->setFixedSize(400, 160);   // 窗口像素尺寸锁死
//   // m_chartView_B_Depth->setRenderHint(QPainter::Antialiasing);
//    // 3. 塞进容器
//    ly->addWidget(m_chartView_B_Depth); // 注意，这里是 addWidget，不是 addLayout 
}
void PolePieceMeasureWin::on_btnShort_A_Picture_clicked()
{

}
void PolePieceMeasureWin::CalculateStatistics()//计算CPK值
{


}



void PolePieceMeasureWin::on_btnConnectServer_clicked()//连接HMI
{

   /* if (socketClient == nullptr)
    {
        socketClient = new  QTcpSocket();
    }*/
    //QTcpSocket* socketClient = nullptr;//socket客户端对象
    QString ip = CommunicateUi->editServerIP->text();
    quint16 port= CommunicateUi->editServerPort->text().toInt();
    bool ret = connectSocketServer(ip,  port,  3000);//连接socket服务器
    if (ret)
    {
        CommunicateUi->labelHMIConnectState->setText(QStringLiteral("connected"));
    }
    else
    {
        CommunicateUi->labelHMIConnectState->setText(QStringLiteral("unconnected"));
    }
  //  bool writeSocketServerData(const QByteArray & data);//给socket服务器写入数据
 
   // void onSocketReadyRead();//接收socket服务器消息
  //  void onSocketDisconnected();//socket与服务器断开连接
  //  void onSocketError(QAbstractSocket::SocketError error);//socket错误

}
void PolePieceMeasureWin::on_btnDisConnectServer_clicked()//断开HMI
{
    socketAutoReconnect = false;//手动断开，不再自动重连
    if (socketReconnectTimer != nullptr)
    {
        socketReconnectTimer->stop();
    }
    if (socketClient != nullptr && socketClient->state() != QAbstractSocket::UnconnectedState)
    {
        socketClient->abort();
    }
    CommunicateUi->labelHMIConnectState->setText(QStringLiteral("unconnected"));
}
void PolePieceMeasureWin::on_btnSendMessage_clicked()//发送消息
{
    QString message = CommunicateUi->txtEditContentTest->toPlainText(); //获取输入框内容
    if (message.isEmpty())
    {
        return;
    }

    bool ret = writeSocketServerData(message.toUtf8());//给socket服务器写入数据
    if (ret)
    {
        RecordData(QStringLiteral("发送消息：%1").arg(message), 0);
    }
    else
    {
        RecordData(QStringLiteral("发送消息失败：%1").arg(message), 2);
    }
}
void PolePieceMeasureWin::on_btnCommunication_clicked()//通信窗口
{
    iniSettingParam(); 
    //基础配置
    CommunicateDialog->setWindowTitle("Communication Setting");
    CommunicateDialog->setWindowFlags(CommunicateDialog->windowFlags() & ~Qt::WindowContextHelpButtonHint);
   
    connect(CommunicateUi->btnWriteReister, &QPushButton::clicked, [this]() {
        iniSettingParam();
        on_btnWritePLCReister_clicked();
        });
    connect(CommunicateUi->btnReadReister, &QPushButton::clicked, [this]() {
        iniSettingParam();
        on_btnReadPLCReister_clicked();
        });
    //btnConnectServer
    //执行对话框，结束后自动释放内存
    if (CommunicateDialog->exec() == QDialog::Accepted) {
        // 可扩展登录成功后的逻辑
    }




  // delete CommunicationUi;
  //  delete CommunicationDialog;
}
/*#include <QVTKOpenGLNativeWidget.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkJPEGWriter.h>

// 保存当前 VTK 窗口为图片
void saveVTKSnapshot(QVTKOpenGLNativeWidget* vtkWidget, const QString& filename)
{
    vtkRenderWindow* renderWindow = vtkWidget->renderWindow();
    
    // 捕获窗口内容
    vtkSmartPointer<vtkWindowToImageFilter> windowToImage = 
        vtkSmartPointer<vtkWindowToImageFilter>::New();
    windowToImage->SetInput(renderWindow);
    windowToImage->SetInputBufferTypeToRGB();  // 或 RGBA 带透明
    windowToImage->ReadFrontBufferOff();       // 读取 back buffer，避免闪烁
    windowToImage->Update();
    
    // 根据扩展名选择格式
    if (filename.endsWith(".png", Qt::CaseInsensitive)) {
        vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
        writer->SetFileName(filename.toUtf8().constData());
        writer->SetInputConnection(windowToImage->GetOutputPort());
        writer->Write();
    } else if (filename.endsWith(".jpg", Qt::CaseInsensitive)) {
        vtkSmartPointer<vtkJPEGWriter> writer = vtkSmartPointer<vtkVTKWriter>::New();
        writer->SetFileName(filename.toUtf8().constData());
        writer->SetInputConnection(windowToImage->GetOutputPort());
        writer->Write();
    }
}*/
// 通用 CpK 计算：返回 false 表示样本不足/σ≈0，调用方不要追加点
 bool PolePieceMeasureWin::calcCpk(const QVector<QPointF>& data,
    double usl, double lsl,
    double& outMean, double& outSigma, double& outCpk)
{
    if (data.size() < CPK_MIN_SAMPLES)  return false; // 太短不做 SPC
    int n = data.size();
    double sum = 0, sq = 0;
    for (const QPointF& p : data)      sum += p.y();
    double mean = sum / n;
    for (const QPointF& p : data)      sq += (p.y() - mean) * (p.y() - mean);
    double sigma = std::sqrt(sq / (n - 1));
    if (sigma < 1e-9)                  return false;

    double cpu = (usl - mean) / (3.0 * sigma);
    double cpl = (mean - lsl) / (3.0 * sigma);
    outMean = mean;
    outSigma = sigma;
    outCpk = std::min(cpu, cpl);
    return true;
}

QVector<QPointF> PolePieceMeasureWin::lastWindow(const QVector<QPointF>& v) const
{
    return v.mid(qMax(0, v.size() - CPK_WINDOW));
}
