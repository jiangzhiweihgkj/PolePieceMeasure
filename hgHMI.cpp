#include "hgHMI.h"

//////////////////////////////////////////////////////////////////
/// hgHMI_Ini 类实现
/// 管理INI文件与Qt控件的映射读取与保存
/// 
/// 特点：
/// 1. 支持多种常用控件类型（QLineEdit, QRadioButton, QCheckBox, QComboBox, QSpinBox, QDoubleSpinBox, QTextEdit等）
/// 2. 通过映射容器关联控件与INI键值，便于扩展和维护
/// 3. 提供读取、保存和修改检测功能
/// 
/// 
/// 使用示例：
//// 创建窗体类的成员变量指针
//hgHMI_Ini* ini = nullptr;
// 
//// 构造函数里初始化
//ini = new hgHMI_Ini(QStringLiteral("./SkyEyeSetting.ini"), this);
// 
//// 构造映射容器
//QList<QPair<QWidget*, QString>> widgetKeyPairs = {
//    { ui->editBoardThick, "scan/editBoardThick" },
//    { ui->textComments, "process/comments" },
//    // 其他控件...
//};
//ini->setWidgetKeyMap(widgetKeyPairs);
//
//// 读取配置到控件
//ini->load();
//
//// 窗口销毁判断是否修改
//if (ini->isChanged())
//{
//    ini->save();
//}
//
//// 临时读写单个控件值示例：
//ini->writeValue(QStringLiteral("scan/labelCalibCircleY_1"), ui->labelCalibCircleY_1->text());
//ui->labelCalibCircleX_1->setText(ini->readValue(QStringLiteral("scan/labelCalibCircleX_1")).toString());
//ini->m_settings.sync();// 记得调用同步保存，确保写入磁盘
///
//////////////////////////////////////////////////////////////////
hgHMI_Ini::hgHMI_Ini(const QString& iniFilePath, QObject* parent)
    : QObject(parent), m_settings(iniFilePath, QSettings::IniFormat)
{
}

void hgHMI_Ini::setWidgetKeyMap(const QList<QPair<QWidget*, QString>>& widgetKeyPairs)
{
    m_widgetKeyPairs = widgetKeyPairs;
}

QVariant hgHMI_Ini::readValue(const QString& key) const
{
    return m_settings.value(key);
}

void hgHMI_Ini::writeValue(const QString& key, const QVariant& value)
{
    m_settings.setValue(key, value);
}

void hgHMI_Ini::load()
{
    for (const auto& pair : m_widgetKeyPairs)
    {
        loadWidget(pair.first, pair.second);
    }
}

void hgHMI_Ini::save()
{
    for (const auto& pair : m_widgetKeyPairs)
    {
        saveWidget(pair.first, pair.second);
    }
    m_settings.sync();
}

bool hgHMI_Ini::isChanged() const
{
    for (const auto& pair : m_widgetKeyPairs)
    {
        if (isWidgetChanged(pair.first, pair.second))
            return true;
    }
    return false;
}

bool hgHMI_Ini::isWidgetChanged(QWidget* widget, const QString& key) const
{
    if (!widget)
        return false;

    QVariant iniVal = m_settings.value(key);

    if (auto le = qobject_cast<QLineEdit*>(widget))
    {
        return le->text() != iniVal.toString();
    }
    else if (auto rb = qobject_cast<QRadioButton*>(widget))
    {
        bool iniChecked = (iniVal.toString() == "1" || iniVal.toString().toLower() == "true");
        return rb->isChecked() != iniChecked;
    }
    else if (auto cb = qobject_cast<QCheckBox*>(widget))
    {
        bool iniChecked = (iniVal.toString() == "1" || iniVal.toString().toLower() == "true");
        return cb->isChecked() != iniChecked;
    }
    else if (auto combo = qobject_cast<QComboBox*>(widget))
    {
        return combo->currentIndex() != iniVal.toInt();
    }
    else if (auto spin = qobject_cast<QSpinBox*>(widget))
    {
        return spin->value() != iniVal.toInt();
    }
    else if (auto dSpin = qobject_cast<QDoubleSpinBox*>(widget))
    {
        double val = iniVal.toDouble();
        return !qFuzzyCompare(dSpin->value() + 1.0, val + 1.0);
    }
    else if (auto te = qobject_cast<QTextEdit*>(widget))
    {
        return te->toPlainText() != iniVal.toString();
    }
    else if (auto pte = qobject_cast<QPlainTextEdit*>(widget))
    {
        return pte->toPlainText() != iniVal.toString();
    }

    // 其他控件暂不支持，默认无改动
    return false;
}

void hgHMI_Ini::loadWidget(QWidget* widget, const QString& key)
{
    if (!widget)
        return;

    QVariant iniVal = m_settings.value(key);

    if (auto le = qobject_cast<QLineEdit*>(widget))
    {
        le->setText(iniVal.toString());
    }
    else if (auto rb = qobject_cast<QRadioButton*>(widget))
    {
        bool checked = (iniVal.toString() == "1" || iniVal.toString().toLower() == "true");
        rb->setChecked(checked);
    }
    else if (auto cb = qobject_cast<QCheckBox*>(widget))
    {
        bool checked = (iniVal.toString() == "1" || iniVal.toString().toLower() == "true");
        cb->setChecked(checked);
    }
    else if (auto combo = qobject_cast<QComboBox*>(widget))
    {
        int idx = iniVal.toInt();
        if (idx >= 0 && idx < combo->count())
            combo->setCurrentIndex(idx);
    }
    else if (auto spin = qobject_cast<QSpinBox*>(widget))
    {
        spin->setValue(iniVal.toInt());
    }
    else if (auto dSpin = qobject_cast<QDoubleSpinBox*>(widget))
    {
        dSpin->setValue(iniVal.toDouble());
    }
    else if (auto te = qobject_cast<QTextEdit*>(widget))
    {
        te->setPlainText(iniVal.toString());
    }
    else if (auto pte = qobject_cast<QPlainTextEdit*>(widget))
    {
        pte->setPlainText(iniVal.toString());
    }
}

void hgHMI_Ini::saveWidget(QWidget* widget, const QString& key)
{
    if (!widget)
        return;

    if (auto le = qobject_cast<QLineEdit*>(widget))
    {
        m_settings.setValue(key, le->text());
    }
    else if (auto rb = qobject_cast<QRadioButton*>(widget))
    {
        m_settings.setValue(key, rb->isChecked() ? "1" : "0");
    }
    else if (auto cb = qobject_cast<QCheckBox*>(widget))
    {
        m_settings.setValue(key, cb->isChecked() ? "1" : "0");
    }
    else if (auto combo = qobject_cast<QComboBox*>(widget))
    {
        m_settings.setValue(key, combo->currentIndex());
    }
    else if (auto spin = qobject_cast<QSpinBox*>(widget))
    {
        m_settings.setValue(key, spin->value());
    }
    else if (auto dSpin = qobject_cast<QDoubleSpinBox*>(widget))
    {
        m_settings.setValue(key, dSpin->value());
    }
    else if (auto te = qobject_cast<QTextEdit*>(widget))
    {
        m_settings.setValue(key, te->toPlainText());
    }
    else if (auto pte = qobject_cast<QPlainTextEdit*>(widget))
    {
        m_settings.setValue(key, pte->toPlainText());
    }
}

void hgHMI_Ini::sync()
{
    m_settings.sync();
}



//////////////////////////////////////////////////////////////
/// hgHMI_Pushbutton
/// 自定义按钮控件，继承自 QPushButton，支持自定义外观和指示灯显示。
///
/// 特点：
/// - 鼠标悬停时显示手型光标。
/// - 支持圆角矩形背景和右上角指示灯显示。
///
/// 使用方式：
/// 1. 直接将 QPushButton 提升为 hgHMI_Pushbutton。
///
//////////////////////////////////////////////////////////////
hgHMI_Pushbutton::hgHMI_Pushbutton(QWidget* parent)
    : QPushButton(parent)
{
    setFont(QFont("微软雅黑", 18, QFont::Normal));
    setCursor(Qt::PointingHandCursor);

    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    setMinimumSize(100, 60); // 可调整
    setCheckable(false); // 用自定义状态
    setText("");
}

hgHMI_Pushbutton::hgHMI_Pushbutton(const QString& text, QWidget* parent)
    : QPushButton(text, parent)
{
    setFont(QFont("微软雅黑", 18, QFont::Normal));
    setCursor(Qt::PointingHandCursor);

    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    setMinimumSize(100, 60); // 可调整
    setCheckable(false); // 用自定义状态
}

void hgHMI_Pushbutton::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event)
        QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 画圆角矩形背景
    painter.setBrush(pressed ? pressedColor : normalColor);
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(rect(), 0, 0);

    // 画中间文字
    painter.setPen(QColor(220, 220, 220));
    QFontMetrics fm(font());
    QString text = this->text();
    int textWidth = fm.horizontalAdvance(text);
    int textHeight = fm.height();
    int x = (width() - textWidth) / 2;
    int y = (height() + textHeight) / 2 - fm.descent();
    painter.drawText(QPoint(x, y), text);

    // 画右上角指示灯
    if (indicatorVisible)
    {
        int indicatorRadius = 8; // 指示灯半径
        int margin = 10;
        int cx = width() - indicatorRadius - margin;
        int cy = margin + indicatorRadius;
        painter.setBrush(indicatorOn ? lightColorOn : lightColorOff);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPoint(cx, cy), indicatorRadius, indicatorRadius);
    }

}

void hgHMI_Pushbutton::mousePressEvent(QMouseEvent* event)
{
    pressed = true;
    update();
    QPushButton::mousePressEvent(event);
}

void hgHMI_Pushbutton::mouseReleaseEvent(QMouseEvent* event)
{
    if (pressed) {
        // 切换指示灯状态
        indicatorOn = !indicatorOn;
        update();
    }
    pressed = false;
    QPushButton::mouseReleaseEvent(event);
}

void hgHMI_Pushbutton::setLightVisible(bool visible)
{
    if (indicatorVisible != visible)
    {
        indicatorVisible = visible;
        update();
    }
}

bool hgHMI_Pushbutton::isLightVisible() const
{
    return indicatorVisible;
}

void hgHMI_Pushbutton::setLightOn(bool on)
{
    if (indicatorOn != on)
    {
        indicatorOn = on;
        update();
    }
}

bool hgHMI_Pushbutton::isLightOn() const
{
    return indicatorOn;
}



//////////////////////////////////////////////////////////////
/// hgHMI_StatusLabel
/// 带淡入淡出动画效果的标签控件，继承自 QLabel。
///
/// 特点：
/// - 支持设置文本时自动淡出旧文本并淡入新文本。
/// - 利用 QGraphicsOpacityEffect 对象实现透明度动画。
/// - 动画时间默认400毫秒，支持平滑切换文本。
///
/// 使用方法：
/// 1. 直接将 QLabel 提升为 hgHMI_StatusLabel。
/// 2. 调用 set_Text(QStringLiteral()) 设置新文本，控件自动执行淡出-切换-淡入流程。
///
//////////////////////////////////////////////////////////////
hgHMI_StatusLabel::hgHMI_StatusLabel(QWidget* parent)
    : QLabel(parent),
    fadingOut(true)  // 初始设置为淡出阶段（默认）
{
    // 创建透明度效果，并应用到标签上
    opacityEffect = new QGraphicsOpacityEffect(this);
    this->setGraphicsEffect(opacityEffect);
    opacityEffect->setOpacity(1.0); // 初始完全不透明

    // 创建动画对象，作用于透明度属性
    fadeAnim = new QPropertyAnimation(opacityEffect, "opacity", this);
    fadeAnim->setDuration(400); // 动画时长

    // 连接动画结束信号到自定义槽函数
    connect(fadeAnim, &QPropertyAnimation::finished, this, &hgHMI_StatusLabel::onFadeAnimFinished);
}

void hgHMI_StatusLabel::set_Text(const QString& newText) // 设置带动画的文本显示
{
    // 缓存新文本，待淡出动画完成后设置
    m_newText = newText;

    // 如果动画正在运行，先停止，保证动画状态清晰
    if (fadeAnim->state() == QAbstractAnimation::Running)
    {
        fadeAnim->stop();
    }

    // 标记当前动画为淡出阶段
    fadingOut = true;

    // 配置动画为从不透明 (1.0) 到透明 (0.0)
    fadeAnim->setStartValue(1.0);
    fadeAnim->setEndValue(0.0);

    // 启动淡出动画
    fadeAnim->start();
}

void hgHMI_StatusLabel::onFadeAnimFinished() // 动画完成时的槽函数，根据当前阶段执行不同操作
{
    if (fadingOut)
    {
        // 淡出动画完成，设置新文本
        this->setText(m_newText);

        // 切换动画阶段为淡入
        fadingOut = false;

        // 配置动画为从透明 (0.0) 到不透明 (1.0)
        fadeAnim->setStartValue(0.0);
        fadeAnim->setEndValue(1.0);

        // 启动淡入动画
        fadeAnim->start();
    }
}


///////////////////////////////////////////////////////////////
/// hgHMI_MinimizeLabel
/// 可点击最小化窗口的标签控件，继承自 QLabel。
/// 
/// 特点：
/// - 鼠标悬停时显示手型光标。
/// - 点击标签时最小化所属顶层窗口。
/// 
/// 使用方法：
/// 1. 直接将 QLabel 提升为 hgHMI_MinimizeLabel。
/// 
////////////////////////////////////////////////////////////////
hgHMI_MinimizeLabel::hgHMI_MinimizeLabel(QWidget* parent)
    : QLabel(parent)
{
    setCursor(Qt::PointingHandCursor);
    setToolTip(QStringLiteral("单击最小化，双击最大化"));
    m_singleClickTimer.setSingleShot(true);
    m_singleClickTimer.setInterval(150);
    connect(&m_singleClickTimer, &QTimer::timeout, this, &hgHMI_MinimizeLabel::onSingleClickTimeout);
}

void hgHMI_MinimizeLabel::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        if (m_doubleClickFlag)
        {
            // 已处理为双击，不做处理
            m_doubleClickFlag = false;
        }
        else
        {
            // 不是双击，开启单击定时器
            m_singleClickTimer.start();
        }
    }
    QLabel::mouseReleaseEvent(event);
}

void hgHMI_MinimizeLabel::mouseDoubleClickEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        // 标记为双击，后续 mouseReleaseEvent 不会执行单击动作
        m_doubleClickFlag = true;
        if (m_singleClickTimer.isActive())
            m_singleClickTimer.stop();

        QWidget* win = window();
        if (win)
        {
            if (win->isMaximized())
                win->showNormal();
            else
                win->showMaximized();
        }
    }
    QLabel::mouseDoubleClickEvent(event);
}

void hgHMI_MinimizeLabel::onSingleClickTimeout()
{
    QWidget* win = window();
    if (win)
        win->showMinimized();
}

//////////////////////////////////////////////////////////////
/// hgHMI_LogWidget
/// 简单日志展示控件，基于 QTableWidget 实现。
///
/// 特点：
/// - 三列显示时间、等级和内容。
/// - 根据日志等级设置文字颜色。
/// - 支持添加日志、清空和获取日志数量。
/// - 表格不可编辑，支持选中整行。
///
/// 使用方法：
/// 1. 使用 addLog(QString, AlarmLevel) 添加日志。
/// 2. clearLogs() 清空日志。
/// 3. logCount() 获取日志条数。
///
//////////////////////////////////////////////////////////////
hgHMI_LogWidget::hgHMI_LogWidget(QWidget* parent) : QWidget(parent)
{
    table = new QTableWidget(this);
    table->setColumnCount(3);
    QStringList headers = { QStringLiteral("时间"), QStringLiteral("等级"), QStringLiteral("内容") };
    table->setHorizontalHeaderLabels(headers);

    table->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionBehavior(QAbstractItemView::SelectRows);
    table->setSelectionMode(QAbstractItemView::SingleSelection);
    table->setShowGrid(false);

    QFont font;
    font.setPointSize(10);
    table->setFont(font);

    auto layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(table);
    setLayout(layout);
}

void hgHMI_LogWidget::addLog(const QString& message, AlarmLevel level)
{
    int row = table->rowCount();
    table->insertRow(row);

    QString timeStr = currentTimestamp();

    QTableWidgetItem* timeItem = new QTableWidgetItem(timeStr);
    QTableWidgetItem* levelItem = new QTableWidgetItem([&]() {
        switch (level)
        {
        case Info: return QStringLiteral("信息");
        case Warning: return QStringLiteral("警告");
        case Error: return QStringLiteral("错误");
        default: return QStringLiteral("未知");
        }
        }());
    QTableWidgetItem* messageItem = new QTableWidgetItem(message);

    QColor color = levelColor(level);
    timeItem->setForeground(color);
    levelItem->setForeground(color);
    messageItem->setForeground(color);

    table->setItem(row, 0, timeItem);
    table->setItem(row, 1, levelItem);
    table->setItem(row, 2, messageItem);

    table->scrollToBottom();
}

void hgHMI_LogWidget::clearLogs()
{
    table->setRowCount(0);
}

int hgHMI_LogWidget::logCount() const
{
    return table->rowCount();
}

QColor hgHMI_LogWidget::levelColor(AlarmLevel level) const
{
    switch (level)
    {
    case Info:    return QColor(50, 50, 50);
    case Warning: return QColor(255, 140, 0);
    case Error:   return QColor(200, 0, 0);
    default:      return QColor(0, 0, 0);
    }
}

QString hgHMI_LogWidget::currentTimestamp() const
{
    return QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
}



//////////////////////////////////////////////////////////////
/// hgHMI_LogTextEdit
/// 日志文本显示控件，基于 QTextEdit 实现。
///
/// 特点：
/// - 自动显示日志时间戳。
/// - 日志按等级颜色区分文字颜色。
///
/// 使用方式：
/// 1. 调用 addLog(QString, AlarmLevel) 添加日志，会追加显示。
///
//////////////////////////////////////////////////////////////
hgHMI_LogTextEdit::hgHMI_LogTextEdit(QWidget* parent)
    : QTextEdit(parent)
{
    setReadOnly(true);           // 设置只读，防止用户编辑
    setAcceptRichText(true);     // 支持富文本显示
    setFont(QFont("微软雅黑", 10));
}

void hgHMI_LogTextEdit::addLog(const QString& message, AlarmLevel level)
{
    QString color;
    switch (level) {
    case Info:    color = "black"; break;
    case Warning: color = "orange"; break;
    case Error:   color = "red"; break;
    default:      color = "gray"; break;
    }

    QString timeStr = QDateTime::currentDateTime().toString("yyyy/MM/dd HH:mm:ss");

    QString html = QString(
        "<div style='color:gray;'>[%1]</div>"
        "<div style='color:%2;'>%3</div>"
    ).arg(timeStr)
        .arg(color)
        .arg(message.toHtmlEscaped());

    append(html);
}

void hgHMI_LogTextEdit::addLog(const QString& message, int level)
{
    // 将int等级转换为AlarmLevel枚举
    AlarmLevel alarmLevel = Info; // 默认Info
    switch (level) {
    case 1:
        alarmLevel = Warning;
        break;
    case 2:
        alarmLevel = Error;
        break;
    case 0:
    default:
        alarmLevel = Info;
        break;
    }

    // 调用自定义控件的日志添加函数
    addLog(message, alarmLevel);
}



//////////////////////////////////////////////////////////////
/// hgHMI_TimerStatus
/// 自定义的状态栏控件，显示当前时间。
///
/// 特点：
/// - 显示格式为“当前时间: yyyy/MM/dd HH:mm:ss”
/// - 每秒自动刷新显示时间。
///
/// 使用方法：
/// 1. hgHMI_TimerStatus* myStatusWidget = new hgHMI_TimerStatus(this);
///    statusBar()->addPermanentWidget(myStatusWidget);
///
//////////////////////////////////////////////////////////////
hgHMI_TimerStatus::hgHMI_TimerStatus(QWidget* parent)
    : QWidget(parent)
{
    timeLabel = new QLabel(this);
    timeLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

    // 设置字体和样式
    QFont font("微软雅黑", 12, QFont::Bold);
    timeLabel->setFont(font);

    QHBoxLayout* layout = new QHBoxLayout(this);
    layout->setContentsMargins(5, 0, 5, 0);
    layout->addWidget(timeLabel);

    this->setLayout(layout);

    // 创建定时器每秒更新时间
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &hgHMI_TimerStatus::updateTime);
    timer->start(1000);

    updateTime();
}

hgHMI_TimerStatus::~hgHMI_TimerStatus()
{
}

void hgHMI_TimerStatus::updateTime()
{
    QString currentTime = QDateTime::currentDateTime().toString("yyyy/MM/dd HH:mm:ss");
    timeLabel->setText(QStringLiteral("当前时间: %1").arg(currentTime));
}



//////////////////////////////////////////////////////////////
///  hgHMI_VelueBar
///  自定义数值条控件，支持水平或垂直显示。
///
/// 特点：
/// - 支持设置预警和报警阈值，改变颜色显示。
/// - 支持水平和垂直两种方向显示。
/// - 显示当前百分比文字。
///
/// 使用方法：
/// 1. 调用 setMaximum(int) 设置最大值。
/// 2. 调用 setValue(int) 设置当前值。
/// 3. 可选调用 setOrientation(Orientation) 切换方向。
/// 4. 可选调用 setWarningThreshold(int) 和 setAlarmThreshold(int) 设置颜色阈值。
///
//////////////////////////////////////////////////////////////
hgHMI_VelueBar::hgHMI_VelueBar(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(100, 20);
}

void hgHMI_VelueBar::setMaximum(int max)
{
    if (max > 0)
    {
        m_maximum = max;
        if (m_value > m_maximum)
            m_value = m_maximum;
        update();
    }
}

void hgHMI_VelueBar::setValue(int val)
{
    if (val < 0)
        val = 0;
    if (val > m_maximum)
        val = m_maximum;

    if (m_value != val)
    {
        m_value = val;
        update();
    }
}

void hgHMI_VelueBar::setOrientation(Orientation ori)
{
    if (m_orientation != ori)
    {
        m_orientation = ori;
        update();
    }
}

void hgHMI_VelueBar::setWarningThreshold(int warningThreshold)
{
    m_warningThreshold = warningThreshold;
    update();
}

void hgHMI_VelueBar::setAlarmThreshold(int alarmThreshold)
{
    m_alarmThreshold = alarmThreshold;
    update();
}

void hgHMI_VelueBar::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 计算进度比例
    double ratio = (m_maximum > 0) ? static_cast<double>(m_value) / m_maximum : 0.0;
    ratio = qBound(0.0, ratio, 1.0);

    // 选择颜色
    QColor fillColor = m_colorNormal;
    if (m_value >= m_alarmThreshold)
        fillColor = m_colorAlarm;
    else if (m_value >= m_warningThreshold)
        fillColor = m_colorWarning;

    // 绘制背景
    painter.setPen(Qt::black);
    painter.setBrush(QColor(230, 230, 230));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    QRect progressRect;
    if (m_orientation == Horizontal)
    {
        int w = static_cast<int>(width() * ratio);
        progressRect = QRect(0, 0, w, height());
    }
    else // Vertical
    {
        int h = static_cast<int>(height() * ratio);
        progressRect = QRect(0, height() - h, width(), h);
    }

    // 绘制进度条
    painter.setBrush(fillColor);
    painter.drawRect(progressRect);

    // 绘制百分比文字
    QString text = QString("%1%").arg(static_cast<int>(ratio * 100));

    QFont font = painter.font();
    font.setBold(true);
    painter.setFont(font);

    QFontMetrics fm(font);
    int textWidth = fm.horizontalAdvance(text);
    int textHeight = fm.height();

    int textX = (width() - textWidth) / 2;
    int textY = (height() + textHeight) / 2 - fm.descent();

    painter.setPen(Qt::black);
    painter.drawText(textX, textY, text);
}



//////////////////////////////////////////////////////////////
///  hgHMI_ProgressBar
///  自定义进度条控件，显示百分比，支持运行中和完成两种颜色。
///
/// 特点：
/// - 进度范围固定为0~100。
/// - 支持重置功能，恢复为0且未完成状态。
/// - 绘制背景和进度条，进度完成时颜色不同。
/// - 中间显示当前百分比文本。
///
/// 使用方法：
/// 1. 调用 setProgress(int) 设置进度（0-100）。
/// 2. 调用 reset() 重置进度条。
/// 
/// 示例：
//m_ProgressBar = new hgHMI_ProgressBar(this);
//m_ProgressBar->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
//m_ProgressBar->setMinimumWidth(200);
//statusBar()->addPermanentWidget(m_ProgressBar);
//m_ProgressBar->reset();
//m_ProgressBar->hide();
//
//
//for (int i = 0; i <= 95; ++i)// 模拟进度条从0到95，主线程阻塞刷新
//{
//    m_ProgressBar->setProgress(i);    
//    QCoreApplication::processEvents();// 强制处理事件，刷新界面，避免界面卡死
//    QThread::msleep(10);// 模拟耗时
//}
//
//m_ProgressBar->setProgress(100);
//QTimer::singleShot(1000, [this]() {// 延时1秒后重置并隐藏进度条
//    m_ProgressBar->reset();
//    m_ProgressBar->hide();
//    });
///
//////////////////////////////////////////////////////////////
hgHMI_ProgressBar::hgHMI_ProgressBar(QWidget* parent)
    : QWidget(parent),
    m_progress(0),
    m_finished(false)
{
    setMinimumSize(200, 25);
}

void hgHMI_ProgressBar::setProgress(int value)
{
    if (value < 0) value = 0;
    if (value > 100) value = 100;

    if (m_progress != value)
    {
        m_progress = value;
        if (m_progress == 100)
            m_finished = true;
        else
            m_finished = false;

        update();  // 触发重绘
    }
}

void hgHMI_ProgressBar::reset()
{
    m_progress = 0;
    m_finished = false;
    update();
}

void hgHMI_ProgressBar::paintEvent(QPaintEvent* /*event*/)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制背景
    painter.setPen(Qt::black);
    painter.setBrush(QColor(230, 230, 230));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));

    // 计算进度宽度
    int progressWidth = (width() * m_progress) / 100;

    // 选择颜色
    QColor fillColor = m_finished ? m_colorFinished : m_colorRunning;

    // 绘制进度条
    painter.setBrush(fillColor);
    painter.drawRect(0, 0, progressWidth, height());

    // 绘制文字
    QString text = QString("%1%").arg(m_progress);
    QFont font = painter.font();
    font.setBold(true);
    painter.setFont(font);

    QFontMetrics fm(font);
    int textWidth = fm.horizontalAdvance(text);
    int textHeight = fm.height();

    int textX = (width() - textWidth) / 2;
    int textY = (height() + textHeight) / 2 - fm.descent();

    painter.setPen(Qt::black);
    painter.drawText(textX, textY, text);
}



//////////////////////////////////////////////////////////////
///  hgHMI_FoldableGroupBox
///  可折叠的分组框控件，支持点击标题栏折叠和展开内容区域。
/// 
/// 特点：
/// - 继承自 QGroupBox，支持标准分组框功能。
/// - 点击标题栏区域可折叠/展开内容区域。
/// - 使用动画平滑过渡折叠和展开状态。
/// - 自动计算内容区域高度，适应布局变化。
/// 
/// 使用方法：
/// 1. 直接将 QGroupBox 提升为 hgHMI_FoldableGroupBox。
/// 2. 添加 widget 子控件到分组框中，点击标题栏可折叠/展开内容。
/////////////////////////////////////////////////////////////////
hgHMI_FoldableGroupBox::hgHMI_FoldableGroupBox(QWidget* parent)
    : QGroupBox(parent),
    m_animation(new QPropertyAnimation(this, "maximumHeight", this)),  // 绑定动画控制控件最大高度属性
    m_folded(false),
    m_titleHeight(25),
    m_expandedHeight(0)
{
    setCheckable(false);    // 取消 QGroupBox 自带的勾选功能（使用自定义折叠）
    setMouseTracking(true);

    // 只设置默认的 titleHovered 高亮背景色，保持其他样式不变
    setStyleSheet(QStringLiteral(R"(
QGroupBox[titleHovered="true"]::title {
    background-color: #4170b8;  /* 默认高亮色，蓝色调 */
    color: #FFFFFF;
}
)"));


    // 动画设置
    m_animation->setDuration(300);                      // 动画时长300毫秒
    m_animation->setEasingCurve(QEasingCurve::InOutQuad); // 缓动曲线，平滑动画
    // 动画结束时触发槽函数，用于隐藏或显示内容控件
    connect(m_animation, &QPropertyAnimation::finished, this, &hgHMI_FoldableGroupBox::onAnimationFinished);

    // 延迟收集内容widget，保证构造后布局已就绪（布局完成后再收集子控件）
    QTimer::singleShot(0, this, &hgHMI_FoldableGroupBox::collectContentWidgets);
}

hgHMI_FoldableGroupBox::hgHMI_FoldableGroupBox(const QString& title, QWidget* parent)
    : hgHMI_FoldableGroupBox(parent)
{
    setTitle(title);  // 设置标题
}

int hgHMI_FoldableGroupBox::getTitleHeight() const
{
    QStyleOptionGroupBox option;
    initStyleOption(&option);
    // 获取标题栏矩形，QStyle提供子控件尺寸
    QRect titleRect = style()->subControlRect(QStyle::CC_GroupBox, &option, QStyle::SC_GroupBoxLabel, this);
    // 兼容某些样式下titleRect.y()不为0，使用bottom()作为高度
    return titleRect.bottom();
}

void hgHMI_FoldableGroupBox::collectContentWidgets()
{
    m_contentWidgets.clear();

    // 遍历所有子对象
    const auto children = this->children();
    for (QObject* obj : children) {
        QWidget* w = qobject_cast<QWidget*>(obj);
        // 只收集父控件是当前控件的直接子widget，排除本身
        if (w && w->parentWidget() == this && w != this) {
            m_contentWidgets.append(w);
        }
    }

    // 记录当前标题栏高度（动态获取）
    m_titleHeight = getTitleHeight();
    // sizeHint()返回理想大小，记录展开时整体高度
    m_expandedHeight = sizeHint().height();

    // 设置当前最大高度为展开高度，保证内容显示完整
    setMaximumHeight(m_expandedHeight);
}

void hgHMI_FoldableGroupBox::showEvent(QShowEvent* event)
{
    QGroupBox::showEvent(event);
    m_titleHeight = getTitleHeight();
    if (m_expandedHeight == 0) {
        // 第一次显示时计算展开高度
        m_expandedHeight = sizeHint().height();
        setMaximumHeight(m_expandedHeight);
    }
}

void hgHMI_FoldableGroupBox::resizeEvent(QResizeEvent* event)
{
    QGroupBox::resizeEvent(event);
    m_titleHeight = getTitleHeight();
    if (!m_folded) {
        // 只有展开状态才更新展开高度，确保动画终点正确
        m_expandedHeight = sizeHint().height();
    }
}

bool hgHMI_FoldableGroupBox::isFolded() const
{
    return m_folded;
}

void hgHMI_FoldableGroupBox::setFolded(bool folded)
{
    if (m_folded == folded)
        return;  // 状态无变化，直接返回

    m_folded = folded;
    m_animation->stop();  // 停止当前动画

    m_titleHeight = getTitleHeight();

    if (m_folded)
    {
        // 折叠时，动画开始时内容可见，动画结束后隐藏内容
        m_animation->setStartValue(height());
        m_animation->setEndValue(m_titleHeight + m_foldedExtraHeight);
    }
    else
    {
        // 展开时，动画开始前显示内容，动画过程中一直可见
        for (auto w : m_contentWidgets)
            if (w) w->setVisible(true);

        // 计算展开时的总高度
        m_expandedHeight = sizeHint().height();
        m_animation->setStartValue(height());
        m_animation->setEndValue(m_expandedHeight);
    }
    m_animation->start();  // 启动动画
}

void hgHMI_FoldableGroupBox::mousePressEvent(QMouseEvent* event)
{
    m_titleHeight = getTitleHeight();
    // 只监听标题栏垂直范围内的点击
    if (event->pos().y() >= 0 && event->pos().y() <= m_titleHeight) {
        toggleFolded();
        event->accept();  // 事件处理完成，不传递
        return;
    }
    QGroupBox::mousePressEvent(event);  // 其他区域正常处理
}

void hgHMI_FoldableGroupBox::toggleFolded()
{
    setFolded(!m_folded);
}

void hgHMI_FoldableGroupBox::onAnimationFinished()
{
    if (m_folded)
    {
        // 折叠动画结束，隐藏内容控件，只显示标题栏
        for (auto w : m_contentWidgets)
            if (w) w->setVisible(false);

        setMaximumHeight(m_titleHeight + m_foldedExtraHeight);  // 最大高度限制为标题栏高度
    }
    else
    {
        // 展开动画结束，确保内容始终显示，最大高度为展开高度
        for (auto w : m_contentWidgets)
            if (w) w->setVisible(true);

        setMaximumHeight(m_expandedHeight);
    }
}

void hgHMI_FoldableGroupBox::setStyle(const QString& styleNameOrSheet)
{
    QString str = styleNameOrSheet.trimmed();

    if (str.compare("blue", Qt::CaseInsensitive) == 0) {
        setStyleSheet(styleSheetBlue());
    }
    else if (str.compare("yellow", Qt::CaseInsensitive) == 0) {
        setStyleSheet(styleSheetYellow());
    }
    else if (str.compare("green", Qt::CaseInsensitive) == 0) {
        setStyleSheet(styleSheetGreen());
    }
    else if (str.compare("dark", Qt::CaseInsensitive) == 0) {
        setStyleSheet(styleSheetDark());
    }
    else {
        // 直接应用传入字符串，空字符串表示清除样式
        setStyleSheet(styleNameOrSheet);
    }
}

QString hgHMI_FoldableGroupBox::styleSheetBlue() const
{
    return QStringLiteral(R"(
QGroupBox {
    border: 2px solid qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                      stop:0 #5A81A8, stop:1 #274472);
    border-radius: 10px;
    background-color: rgba(230, 238, 245, 90%);
    margin-top: 20px;
}
QGroupBox::title {
    color: #FFFFFF;
    background-color: #274472;
    font: 75 12pt "Microsoft YaHei UI";
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 4px 16px;
    border-radius: 6px;
}
/* 悬停高亮样式 */
QGroupBox[titleHovered="true"]::title {
    background-color: #4170b8;
    color: #ffffff;
}
QGroupBox:disabled {
    border-color: #A5B7C6;
    color: #95A5A6;
}
QGroupBox QCheckBox {
    color: #274472;
}
)");
}

QString hgHMI_FoldableGroupBox::styleSheetYellow() const
{
    return QStringLiteral(R"(
QGroupBox {
    border: 2px solid qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                      stop:0 #E6C84B, stop:1 #C9A12B);
    border-radius: 10px;
    background-color: rgba(250, 245, 230, 90%);
    margin-top: 20px;
}
QGroupBox::title {
    color: #FFFFFF;
    background-color: #C9A12B;
    font: 75 12pt "Microsoft YaHei UI";
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 4px 16px;
    border-radius: 6px;
}
QGroupBox[titleHovered="true"]::title {
    background-color: #E0C95A;
    color: #FFFFFF;
}
QGroupBox:disabled {
    border-color: #D8C98B;
    color: #B5A973;
}
QGroupBox QCheckBox {
    color: #C9A12B;
}
)");
}

QString hgHMI_FoldableGroupBox::styleSheetGreen() const
{
    return QStringLiteral(R"(
QGroupBox {
    border: 2px solid qlineargradient(x1:0, y1:0, x2:1, y2:1,
                                      stop:0 #5C8A43, stop:1 #4B6E3A);
    border-radius: 10px;
    background-color: rgba(229, 237, 224, 90%);
    margin-top: 20px;
}
QGroupBox::title {
    color: #FFFFFF;
    background-color: #4B6E3A;
    font: 75 12pt "Microsoft YaHei UI";
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 4px 16px;
    border-radius: 6px;
}
QGroupBox[titleHovered="true"]::title {
    background-color: #6CA24D;
    color: #FFFFFF;
}
QGroupBox:disabled {
    border-color: #BFC9AE;
    color: #A0A0A0;
}
QGroupBox QCheckBox {
    color: #4B6E3A;
}
)");
}

QString hgHMI_FoldableGroupBox::styleSheetDark() const
{
    return QStringLiteral(R"(
QGroupBox {
    border: none;
    border-radius: 4px;
    background-color: #1E1E1E; /* 深暗灰背景 */
    margin-top: 18px;
}
QGroupBox::title {
    color: #BBBBBB; /* 默认字体浅灰 */
    background-color: #2C2C2C; /* 标题默认深灰 */
    font: 75 11pt "Microsoft YaHei UI";
    subcontrol-origin: margin;
    subcontrol-position: top left;
    padding: 4px 14px;
    border-radius: 3px;
}
QGroupBox[titleHovered="true"]::title {
    background-color: #000000; /* 悬停黑色背景 */
    color: #FFFFFF; /* 悬停白色字体 */
}
QGroupBox:disabled {
    color: #555555;
}
QGroupBox QCheckBox {
    color: #BBBBBB;
}
)");
}

void hgHMI_FoldableGroupBox::mouseMoveEvent(QMouseEvent* event)
{
    int y = event->pos().y();
    bool hovered = (y >= 0 && y <= m_titleHeight);
    if (hovered != m_titleHovered) {
        m_titleHovered = hovered;
        setProperty("titleHovered", m_titleHovered); // 设置属性供样式表使用
        style()->unpolish(this); // 重新应用样式
        style()->polish(this);
        update();
    }
    QGroupBox::mouseMoveEvent(event);
}

void hgHMI_FoldableGroupBox::leaveEvent(QEvent* event)
{
    if (m_titleHovered) {
        m_titleHovered = false;
        setProperty("titleHovered", false);
        style()->unpolish(this);
        style()->polish(this);
        update();
    }
    QGroupBox::leaveEvent(event);
}




//////////////////////////////////////////////////////////////
///  hgHMI_VTKCheck
///  VTK自定义交互样式，支持鼠标左键点击点拾取。
///
/// 功能：
/// - 使用 vtkPointPicker 在渲染器中拾取三维点坐标，通过回调函数传递拾取到的点坐标
///
/// 使用方法：
/// 1. 设置 clickCallback 回调函数，接受三维坐标参数。
/// 2. 将该交互样式绑定到 vtkRenderWindowInteractor。
/// 
/// 示例：
//  auto interactor = m_viewer->getRenderWindow()->GetInteractor();
//  auto style = vtkSmartPointer<hgHMI_VTKCheck>::New();
//  style->clickCallback = [this](double x, double y, double z) {
//      if (!std::isnan(x))
// 
//       };
//  interactor->SetInteractorStyle(style);
///
//////////////////////////////////////////////////////////////
vtkStandardNewMacro(hgHMI_VTKCheck);

hgHMI_VTKCheck::hgHMI_VTKCheck()
    : hasPoint1(false),                    // 初始化无右键第一个点
    leftSphereSize(0.0),                 // 左球默认半径0
    rightSphereSize(0.0)                 // 右球默认半径0
{
    // 构造函数，无需额外操作
}

void hgHMI_VTKCheck::setSphereSize(bool isLeft, double size)// 设置左球或右球尺寸，<=0隐藏，>0显示对应尺寸
{
    if (isLeft) {
        leftSphereSize = size;            // 设置左球尺寸
        if (leftSphereSize <= 0) {
            RemoveActor(leftSphereActor); // 尺寸<=0则移除左球
        }
        else {
            if (leftSphereActor) {
                // 动态更新左球半径
                vtkSphereSource* sphereSrc = dynamic_cast<vtkSphereSource*>(leftSphereActor->GetMapper()->GetInputAlgorithm());
                if (sphereSrc) {
                    sphereSrc->SetRadius(leftSphereSize);
                    sphereSrc->Update();
                    leftSphereActor->GetMapper()->Update();
                    leftSphereActor->SetVisibility(1);
                }
                if (this->Interactor)
                    this->Interactor->GetRenderWindow()->Render(); // 刷新渲染
            }
        }
    }
    else {
        rightSphereSize = size;           // 设置右球尺寸
        if (rightSphereSize <= 0) {
            RemoveActor(rightSphereActor1); // 隐藏右球第1点
            RemoveActor(rightSphereActor2); // 隐藏右球第2点
            if (lineActor)
            {
                RemoveActor(lineActor);      // 移除连线
                lineActor = nullptr;
            }
        }
        else {
            if (rightSphereActor1) {
                vtkSphereSource* sphereSrc = dynamic_cast<vtkSphereSource*>(rightSphereActor1->GetMapper()->GetInputAlgorithm());
                if (sphereSrc) {
                    sphereSrc->SetRadius(rightSphereSize);
                    sphereSrc->Update();
                    rightSphereActor1->GetMapper()->Update();
                    rightSphereActor1->SetVisibility(1);
                }
            }
            if (rightSphereActor2) {
                vtkSphereSource* sphereSrc = dynamic_cast<vtkSphereSource*>(rightSphereActor2->GetMapper()->GetInputAlgorithm());
                if (sphereSrc) {
                    sphereSrc->SetRadius(rightSphereSize);
                    sphereSrc->Update();
                    rightSphereActor2->GetMapper()->Update();
                    rightSphereActor2->SetVisibility(1);
                }
            }
            if (!lineActor && rightSphereActor1 && rightSphereActor2)
            {
                // 恢复显示时，如果连线不存在且两个点都存在，则重绘连线
                double p1[3], p2[3];
                vtkSphereSource* sphereSrc1 = dynamic_cast<vtkSphereSource*>(rightSphereActor1->GetMapper()->GetInputAlgorithm());
                vtkSphereSource* sphereSrc2 = dynamic_cast<vtkSphereSource*>(rightSphereActor2->GetMapper()->GetInputAlgorithm());
                if (sphereSrc1 && sphereSrc2)
                {
                    sphereSrc1->GetCenter(p1);
                    sphereSrc2->GetCenter(p2);
                    UpdateLineActor(p1, p2); // 重新绘制连线
                }
            }
            if (this->Interactor)
                this->Interactor->GetRenderWindow()->Render(); // 刷新渲染
        }
    }
}

void hgHMI_VTKCheck::UpdateSphereActor(double center[3], vtkSmartPointer<vtkActor>& actor,
    double color[3], double radius)// 创建或更新球体actor函数
{
    vtkRenderer* renderer = nullptr;
    if (this->Interactor && this->Interactor->GetRenderWindow())
        renderer = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer(); // 获取渲染器

    if (!renderer)
        return;

    if (!actor)
    {
        // 第一次创建球体
        vtkSmartPointer<vtkSphereSource> sphereSrc = vtkSmartPointer<vtkSphereSource>::New();
        sphereSrc->SetCenter(center);   // 设置球心
        sphereSrc->SetRadius(radius);   // 设置半径
        sphereSrc->SetThetaResolution(24); // 水平分辨率
        sphereSrc->SetPhiResolution(24);   // 垂直分辨率
        sphereSrc->Update();

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(sphereSrc->GetOutputPort()); // 连接几何数据

        actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);       // 赋予mapper
        actor->GetProperty()->SetColor(color);    // 设置颜色
        actor->GetProperty()->SetOpacity(0.8);    // 半透明

        renderer->AddActor(actor);      // 加入渲染器
    }
    else
    {
        // 更新已有球体的球心位置
        vtkSphereSource* sphereSrc = dynamic_cast<vtkSphereSource*>(actor->GetMapper()->GetInputAlgorithm());
        if (sphereSrc)
        {
            sphereSrc->SetCenter(center);
            sphereSrc->Update();
        }
        actor->GetMapper()->Update();
    }

    if (this->Interactor)
        this->Interactor->GetRenderWindow()->Render(); // 刷新渲染
}

void hgHMI_VTKCheck::UpdateLineActor(const double p1[3], const double p2[3])
{
    vtkRenderer* renderer = nullptr;
    if (this->Interactor && this->Interactor->GetRenderWindow())
        renderer = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();

    if (!renderer)
        return;

    if (!lineActor)
    {
        vtkSmartPointer<vtkLineSource> lineSource = vtkSmartPointer<vtkLineSource>::New();
        lineSource->SetPoint1(p1);      // 设置线段起点
        lineSource->SetPoint2(p2);      // 设置线段终点
        lineSource->Update();

        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(lineSource->GetOutputPort());

        lineActor = vtkSmartPointer<vtkActor>::New();
        lineActor->SetMapper(mapper);
        lineActor->GetProperty()->SetColor(0.0, 0.0, 1.0); // 蓝色线段
        lineActor->GetProperty()->SetLineWidth(2.0);        // 线宽

        renderer->AddActor(lineActor);   // 添加到渲染器
    }
    else
    {
        vtkLineSource* lineSource = dynamic_cast<vtkLineSource*>(lineActor->GetMapper()->GetInputConnection(0, 0)->GetProducer());
        if (lineSource)
        {
            lineSource->SetPoint1(p1);  // 更新起点
            lineSource->SetPoint2(p2);  // 更新终点
            lineSource->Update();
            lineActor->GetMapper()->Update();
            lineActor->SetVisibility(1);
        }
    }

    if (this->Interactor)
        this->Interactor->GetRenderWindow()->Render(); // 刷新渲染
}

void hgHMI_VTKCheck::RemoveActor(vtkSmartPointer<vtkActor>& actor)// 移除actor函数
{
    if (actor && this->Interactor && this->Interactor->GetRenderWindow())
    {
        vtkRenderer* renderer = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
        if (renderer && renderer->HasViewProp(actor))
        {
            renderer->RemoveActor(actor);  // 从渲染器移除actor

            if (this->Interactor)
                this->Interactor->GetRenderWindow()->Render(); // 刷新渲染
        }
        actor = nullptr;                  // 置空智能指针
    }
}

void hgHMI_VTKCheck::OnLeftButtonDown()// 鼠标左键按下事件
{
    auto interactor = this->Interactor;
    if (!interactor)
    {
        vtkInteractorStyleTrackballCamera::OnLeftButtonDown(); // 交给父类处理
        return;
    }

    mouseDownX = interactor->GetEventPosition()[0];  // 记录按下鼠标X
    mouseDownY = interactor->GetEventPosition()[1];  // 记录按下鼠标Y
    mouseDownTime = std::chrono::steady_clock::now(); // 记录按下时间

    vtkInteractorStyleTrackballCamera::OnLeftButtonDown(); // 调用父类处理
}

void hgHMI_VTKCheck::OnLeftButtonUp()// 鼠标左键弹起事件
{
    auto interactor = this->Interactor;
    if (!interactor)
    {
        vtkInteractorStyleTrackballCamera::OnLeftButtonUp(); // 交给父类
        return;
    }

    int x = interactor->GetEventPosition()[0]; // 弹起鼠标X
    int y = interactor->GetEventPosition()[1]; // 弹起鼠标Y
    auto mouseUpTime = std::chrono::steady_clock::now(); // 弹起时间

    // 计算鼠标移动距离和持续时间
    auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(mouseUpTime - mouseDownTime).count();
    double moveDist = std::sqrt((x - mouseDownX) * (x - mouseDownX) + (y - mouseDownY) * (y - mouseDownY));

    // 判断是否满足点击条件
    if (durationMs <= maxClickDuration && moveDist <= maxClickMoveDistance)
    {
        vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
        picker->SetTolerance(0.01);       // 设置拾取容差

        vtkRenderer* renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
        if (picker->Pick(x, y, 0, renderer)) // 执行拾取
        {
            double pickedPos[3];
            picker->GetPickPosition(pickedPos); // 获取拾取点三维坐标

            RemoveActor(leftSphereActor);        // 删除之前的左键球体

            double red[3] = { 1.0, 0.0, 0.0 };
            UpdateSphereActor(pickedPos, leftSphereActor, red, leftSphereSize); // 显示新球

            if (clickCallback)
                clickCallback(pickedPos[0], pickedPos[1], pickedPos[2]); // 回调通知
        }
        else
        {
            if (clickCallback)
                clickCallback(std::numeric_limits<double>::quiet_NaN(),  // 拾取失败回调NaN
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN());
        }
    }

    vtkInteractorStyleTrackballCamera::OnLeftButtonUp(); // 父类处理
}

void hgHMI_VTKCheck::OnRightButtonDown()
{
    // 设置光标为抓手
    if (this->Interactor)
        this->Interactor->GetRenderWindow()->SetCurrentCursor(VTK_CURSOR_HAND);

    auto interactor = this->Interactor;
    if (!interactor)
        return;

    // 记录鼠标按下位置和时间
    rightButtonPressed = true;  // 标记右键按下
    rightDragging = false;
    rightDownX = interactor->GetEventPosition()[0];
    rightDownY = interactor->GetEventPosition()[1];
    rightMouseDownTime = std::chrono::steady_clock::now();
}

void hgHMI_VTKCheck::OnMouseMove()
{
    auto interactor = this->Interactor;
    if (!interactor)
    {
        vtkInteractorStyleTrackballCamera::OnMouseMove();
        return;
    }

    if (rightButtonPressed)
    {
        int x = interactor->GetEventPosition()[0];
        int y = interactor->GetEventPosition()[1];

        double dist = std::sqrt((x - rightDownX) * (x - rightDownX) + (y - rightDownY) * (y - rightDownY));
        if (!rightDragging && dist > rightDragThreshold)
        {
            rightDragging = true;
            this->StartPan();
        }
        if (rightDragging)
        {
            vtkInteractorStyleTrackballCamera::OnMouseMove();
            return;
        }
    }

    vtkInteractorStyleTrackballCamera::OnMouseMove();
}

void hgHMI_VTKCheck::OnRightButtonUp()
{
    // 恢复光标为箭头
    if (this->Interactor)
        this->Interactor->GetRenderWindow()->SetCurrentCursor(VTK_CURSOR_ARROW);

    rightButtonPressed = false;  // 右键松开
    auto interactor = this->Interactor;
    if (!interactor)
        return;

    int x = interactor->GetEventPosition()[0];
    int y = interactor->GetEventPosition()[1];
    auto mouseUpTime = std::chrono::steady_clock::now();
    double moveDist = std::sqrt((x - rightDownX) * (x - rightDownX) + (y - rightDownY) * (y - rightDownY));
    auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(mouseUpTime - rightMouseDownTime).count();

    if (rightDragging)
    {
        // 拖拽结束，停止平移
        this->EndPan();
        rightDragging = false;
        return;
    }
    // 没有拖拽，判定为点击，执行两点测量
    if (moveDist <= rightDragThreshold && durationMs < maxRightClickIntervalMs)
    {
        vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
        picker->SetTolerance(0.01);     // 设置拾取容差

        vtkRenderer* renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
        bool pickSuccess = picker->Pick(x, y, 0, renderer); // 执行拾取
        if (!pickSuccess)
            return;

        double pickedPos[3]{ 0,0,0 };
        picker->GetPickPosition(pickedPos); // 获取拾取点

        if (!hasPoint1)
        {
            // 第一次右键点击，记录点和时间
            point1[0] = pickedPos[0];
            point1[1] = pickedPos[1];
            point1[2] = pickedPos[2];
            hasPoint1 = true;
            rightClickTime = mouseUpTime;

            // 清除旧球和线段
            RemoveActor(rightSphereActor1);
            RemoveActor(rightSphereActor2);
            if (lineActor)
            {
                RemoveActor(lineActor);
                lineActor = nullptr;
            }
            double blue[3] = { 0.0, 0.0, 1.0 };
            UpdateSphereActor(point1, rightSphereActor1, blue, rightSphereSize); // 新建第1点球
        }
        else
        {
            auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(mouseUpTime - rightClickTime).count();
            if (dt > maxRightClickIntervalMs)
            {
                // 超时重置
                RemoveActor(rightSphereActor1);
                RemoveActor(rightSphereActor2);
                hasPoint1 = false;
                OnRightButtonUp(); // 递归重新处理（这句可选）
                return;
            }
            else if (dt < minRightClickIntervalMs)
            {
                // 时间太短忽略
                return;
            }
            else
            {
                // 第二次点击，记录第2点
                point2[0] = pickedPos[0];
                point2[1] = pickedPos[1];
                point2[2] = pickedPos[2];

                RemoveActor(rightSphereActor2);     // 移除旧第2点球体
                double blue[3] = { 0.0, 0.0, 1.0 };
                UpdateSphereActor(point2, rightSphereActor2, blue, rightSphereSize); // 新第2点球体

                // 计算距离
                double dx = point1[0] - point2[0];
                double dy = point1[1] - point2[1];
                double dz = point1[2] - point2[2];
                double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (rightSphereSize > 0) UpdateLineActor(point1, point2); // 绘制连线

                if (distanceCallback)
                    distanceCallback(point1[0], point1[1], point1[2],  // 回调两点坐标和距离
                        point2[0], point2[1], point2[2],
                        dist);

                hasPoint1 = false; // 重置状态
            }
        }
    }

    this->Interactor->GetRenderWindow()->SetCurrentCursor(VTK_CURSOR_ARROW); // 恢复为箭头
    vtkInteractorStyleTrackballCamera::OnRightButtonUp();
}

void hgHMI_VTKCheck::OnMiddleButtonDown()
{
    this->StartDolly();
}

void hgHMI_VTKCheck::OnMiddleButtonUp()
{
    this->EndDolly();
}



//////////////////////////////////////////////////////////////
///  hgHMI_VTKdoubleCheck
///  基于 hgHMI_VTKCheck 扩展，实现双击检测和拾取。
///
/// 功能：
/// - 继承单击检测，增加双击判定逻辑（时间间隔和位置限制）
/// - 第一次点击拾取但不回调，第二次点击判定为双击时回调双击事件
///
/// 使用方式：
/// 1. 设置 doubleClickCallback 回调函数接受双击拾取点坐标。
/// 2. 绑定该交互样式到 vtkRenderWindowInteractor。
///
//////////////////////////////////////////////////////////////
vtkStandardNewMacro(hgHMI_VTKdoubleCheck);

hgHMI_VTKdoubleCheck::hgHMI_VTKdoubleCheck() = default;

void hgHMI_VTKdoubleCheck::OnLeftButtonDown()
{
    if (this->Interactor)
    {
        this->Interactor->GetEventPosition(mouseDownX, mouseDownY);
        mouseDownTime = std::chrono::steady_clock::now();
    }
    hgHMI_VTKCheck::OnLeftButtonDown();
}

void hgHMI_VTKdoubleCheck::OnLeftButtonUp()
{
    auto interactor = this->Interactor;
    if (!interactor)
    {
        hgHMI_VTKCheck::OnLeftButtonUp();
        return;
    }

    int x, y;
    interactor->GetEventPosition(x, y);
    auto mouseUpTime = std::chrono::steady_clock::now();

    auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(mouseUpTime - mouseDownTime).count();

    double dist = std::sqrt((x - mouseDownX) * (x - mouseDownX) + (y - mouseDownY) * (y - mouseDownY));

    if (durationMs <= maxClickDuration && dist <= maxClickMoveDistance)
    {
        auto now = std::chrono::steady_clock::now();

        if (clickCount == 0)
        {
            firstClickTime = now;
            clickCount = 1;
            PickPointNoCallback(x, y);
        }
        else
        {
            auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(now - firstClickTime).count();
            if (interval <= maxDoubleClickInterval)
            {
                clickCount = 0;
                hasFirstClickPos = false;

                vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
                vtkRenderer* renderer = interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();

                if (picker->Pick(x, y, 0, renderer))
                {
                    double pos[3];
                    picker->GetPickPosition(pos);
                    if (doubleClickCallback)
                        doubleClickCallback(pos[0], pos[1], pos[2]);
                }
                else
                {
                    if (doubleClickCallback)
                        doubleClickCallback(std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::quiet_NaN());
                }
                return;
            }
            else
            {
                clickCount = 0;
                hasFirstClickPos = false;
                return;
            }
        }
    }
    else
    {
        clickCount = 0;
        hasFirstClickPos = false;
    }

    hgHMI_VTKCheck::OnLeftButtonUp();
}

void hgHMI_VTKdoubleCheck::PickPointNoCallback(int x, int y)
{
    // 临时替换回调为不调用，确保第一次点击拾取但不回调
    auto oldCallback = this->clickCallback;
    this->clickCallback = nullptr;

    vtkSmartPointer<vtkPointPicker> picker = vtkSmartPointer<vtkPointPicker>::New();
    vtkRenderer* renderer = this->Interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();

    if (picker->Pick(x, y, 0, renderer))
    {
        picker->GetPickPosition(firstClickPos);
        hasFirstClickPos = true;
    }
    else
    {
        hasFirstClickPos = false;
    }

    // 恢复回调
    this->clickCallback = oldCallback;
}



/////////////////////////////////////////////////////////////////
///  hgHMI_PCLViewerWidget
///  点云三维显示控件（PCL + VTK 集成）
///
/// 特点：
/// - 继承自 QVTKOpenGLNativeWidget，自动管理OpenGL与VTK渲染，无需手动初始化。
/// - 直接加载 PCL 点云。
/// - 自动适配点云整体范围。
/// - 提供三维坐标轴样式接口。
/// - 内置鼠标单击选点功能，并通过回调接口返回点坐标，便于二次开发。
/// - 外部可获得底层 PCLVisualizer 指针。
///
/// 使用方法：
/// 1. 将 QVTKOpenGLNativeWidget 提升为 hgHMI_PCLViewerWidget。
/// 2. 调用 showPointCloud() 加载并显示点云数据。
/// 3. 通过 setPointPickCallback() 绑定选点回调，获取用户点击点的三维坐标。
/// 
/// 示例：hgHMI_PCLViewerWidget 基本设置 （自动初始化，默认无轴）
//m_viewer = ui->openGLWidget_Result->m_viewer;           // 将可视化接口暴露给全局变量
//ui->openGLWidget_Result->setAxesVisible(true);          // 显示坐标轴
//ui->openGLWidget_Result->setAxesLength(100, 100, 100);  // 设置轴长度
//ui->openGLWidget_Result->setAxesShaftType(1);           // 设置轴类型为线型
//ui->openGLWidget_Result->setAxesShowLabels(false);      // 不显示轴标签
//ui->openGLWidget_Result->setAxesShowArrow(false);       // 不显示箭头
//ui->openGLWidget_Result->showPointCloud(Cloud);         // 支持点云和智能指针传入
//
//// 设置点拾取回调
//ui->openGLWidget_Result->setPointPickCallback([this](double x, double y, double z) {
//    if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
//    {
//        QString msg = QStringLiteral("拾取点坐标: x=%1, y=%2, z=%3")
//            .arg(x, 0, 'f', 3)
//            .arg(y, 0, 'f', 3)
//            .arg(z, 0, 'f', 3);
//        updateStates(msg);
//    }
//    });
///
/////////////////////////////////////////////////////////////////
hgHMI_PCLViewerWidget::hgHMI_PCLViewerWidget(QWidget* parent)
    : QVTKOpenGLNativeWidget(parent),
    m_renderer(vtkSmartPointer<vtkRenderer>::New()),
    m_axesActor(vtkSmartPointer<vtkAxesActor>::New())
{
    vtkRenderWindow* rw = this->renderWindow();
    rw->AddRenderer(m_renderer);

    m_viewer.reset(new pcl::visualization::PCLVisualizer(m_renderer, rw, "viewer", false));
    m_viewer->setupInteractor(this->interactor(), rw);
    initializeVTKScene();

    // 创建自定义交互样式
    m_interactorStyle = vtkSmartPointer<hgHMI_VTKCheck>::New();

    // 设置点击回调，转发给成员回调
    m_interactorStyle->clickCallback = [this](double x, double y, double z) {
        if (m_pointPickCallback)
            m_pointPickCallback(x, y, z);
        };

    // 右键两点距离拾取回调
    if (m_distancePickCallback)
        m_interactorStyle->distanceCallback = m_distancePickCallback;

    // 设置自定义交互样式
    this->interactor()->SetInteractorStyle(m_interactorStyle);
}

hgHMI_PCLViewerWidget::~hgHMI_PCLViewerWidget()
{
    if (m_viewer)
    {
        m_viewer->close();
        m_viewer.reset();
    }
    m_renderer = nullptr;
    m_axesActor = nullptr;
}

void hgHMI_PCLViewerWidget::initializeVTKScene()
{
    m_renderer->SetBackground(0, 0, 0);// 设置背景色
    m_viewer->initCameraParameters();// 初始化PCL Viewer摄像头参数
}

void hgHMI_PCLViewerWidget::clear()
{
    if (m_viewer)
    {
        m_viewer->removeAllPointClouds();
        m_viewer->removeAllShapes();
        m_viewer->removeAllCoordinateSystems();
    }

    // 坐标轴用vtkAxesActor管理，单独处理
    if (m_axesVisible)
    {
        if (!m_renderer->HasViewProp(m_axesActor))
            m_renderer->AddActor(m_axesActor);
    }
    else
    {
        if (m_renderer->HasViewProp(m_axesActor))
            m_renderer->RemoveActor(m_axesActor);
    }

    this->update();
}

void hgHMI_PCLViewerWidget::showPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    if (!cloud || cloud->empty())
        return;

    m_viewer->removeAllPointClouds();
    m_viewer->removeAllShapes();

    m_viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    computeCloudRange(cloud);
    resetCameraToTop();
    this->update();
}

void hgHMI_PCLViewerWidget::showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    if (!cloud || cloud->empty())
        return;

    if (m_viewer->contains("cloud"))
        m_viewer->removePointCloud("cloud");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbHandler(cloud);
    m_viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgbHandler, "cloud");

    computeCloudRange(cloud);
    resetCameraToTop();
    this->update();
}

void hgHMI_PCLViewerWidget::showPointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
    showPointCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(cloud));
}

void hgHMI_PCLViewerWidget::showPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    showPointCloud(std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(cloud));
}

void hgHMI_PCLViewerWidget::showDebugCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud, bool newWindow)
{
    // 将裸对象转换为智能指针后调用智能指针版本
    auto cloudPtr = std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(cloud);
    showDebugCloud(cloudPtr, newWindow);
}

void hgHMI_PCLViewerWidget::showDebugCloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, bool newWindow)
{
    auto cloudPtr = std::make_shared<const pcl::PointCloud<pcl::PointXYZRGB>>(cloud);
    showDebugCloud(cloudPtr, newWindow);
}

void hgHMI_PCLViewerWidget::showDebugCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, bool newWindow)
{
    if (!cloud || cloud->empty())
        return;

    if (!newWindow)
    {
        // 关闭并清理之前所有调试窗口
        for (auto& dlg : m_debugDialogs)
        {
            if (dlg)
                dlg->close();
        }
        m_debugDialogs.clear();
        m_debugViewers.clear();
        m_debugCloudCounts.clear();
    }

    // 新建调试窗口
    QDialog* dialog = new QDialog(nullptr);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowFlags(dialog->windowFlags() | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
    dialog->setWindowTitle(QStringLiteral("点云调试窗口 - PointXYZ"));
    dialog->resize(this->size());

    QVBoxLayout* layout = new QVBoxLayout(dialog);
    hgHMI_PCLViewerWidget* viewer = new hgHMI_PCLViewerWidget(dialog);
    layout->addWidget(viewer);

    // 配置坐标轴显示参数
    viewer->setAxesLength(1.0, 1.0, 1.0);
    viewer->setAxesShaftType(2);
    viewer->setAxesShowLabels(true);
    viewer->setAxesShowArrow(true);
    viewer->setAxesVisible(true);

    dialog->setLayout(layout);
    dialog->show();

    // 记录新窗口和viewer，初始化点云计数
    m_debugDialogs.append(dialog);
    m_debugViewers.append(viewer);
    m_debugCloudCounts.append(0);

    int idx = m_debugDialogs.size() - 1;

    // 添加点云到viewer
    std::string cloudName = "debug_cloud_" + std::to_string(m_debugCloudCounts[idx]++);
    viewer->m_viewer->addPointCloud<pcl::PointXYZ>(cloud, cloudName);

    viewer->computeCloudRange(cloud);
    viewer->resetCameraToTop();
    viewer->update();

    dialog->raise();
    dialog->activateWindow();
}

void hgHMI_PCLViewerWidget::showDebugCloud(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud, bool newWindow)
{
    if (!cloud || cloud->empty())
        return;

    if (!newWindow)
    {
        for (auto& dlg : m_debugDialogs)
        {
            if (dlg)
                dlg->close();
        }
        m_debugDialogs.clear();
        m_debugViewers.clear();
        m_debugCloudCounts.clear();
    }

    QDialog* dialog = new QDialog(nullptr);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowFlags(dialog->windowFlags() | Qt::WindowMinMaxButtonsHint | Qt::WindowCloseButtonHint);
    dialog->setWindowTitle(QStringLiteral("点云调试窗口 - PointXYZRGB"));
    dialog->resize(this->size());

    QVBoxLayout* layout = new QVBoxLayout(dialog);
    hgHMI_PCLViewerWidget* viewer = new hgHMI_PCLViewerWidget(dialog);
    layout->addWidget(viewer);

    viewer->setAxesLength(1.0, 1.0, 1.0);
    viewer->setAxesShaftType(2);
    viewer->setAxesShowLabels(true);
    viewer->setAxesShowArrow(true);
    viewer->setAxesVisible(true);

    dialog->setLayout(layout);
    dialog->show();

    m_debugDialogs.append(dialog);
    m_debugViewers.append(viewer);
    m_debugCloudCounts.append(0);

    int idx = m_debugDialogs.size() - 1;

    std::string cloudName = "debug_cloud_" + std::to_string(m_debugCloudCounts[idx]++);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbHandler(cloud);
    viewer->m_viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgbHandler, cloudName);

    viewer->computeCloudRange(cloud);
    viewer->resetCameraToTop();
    viewer->update();

    dialog->raise();
    dialog->activateWindow();
}

void hgHMI_PCLViewerWidget::computeCloudRange(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    m_cloudMin = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);
    m_cloudMax = Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z);
    m_cloudCenter = (m_cloudMin + m_cloudMax) * 0.5f;
}

void hgHMI_PCLViewerWidget::computeCloudRange(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    m_cloudMin = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);
    m_cloudMax = Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z);
    m_cloudCenter = (m_cloudMin + m_cloudMax) * 0.5f;
}

void hgHMI_PCLViewerWidget::resetCameraToTop()
{
    double length_x = m_cloudMax.x() - m_cloudMin.x();
    double length_y = m_cloudMax.y() - m_cloudMin.y();
    double length_z = m_cloudMax.z() - m_cloudMin.z();
    double length = (length_x + length_y + length_z) / 3.0;

    double cam_x = m_cloudCenter.x();
    double cam_y = m_cloudCenter.y();
    double cam_z = m_cloudMax.z() + length * 4;

    m_camPos = Eigen::Vector3f(cam_x, cam_y, cam_z);
    m_camView = Eigen::Vector3f(m_cloudCenter.x(), m_cloudCenter.y(), m_cloudCenter.z());

    m_viewer->setCameraPosition(
        m_camPos.x(), m_camPos.y(), m_camPos.z(),
        m_camView.x(), m_camView.y(), m_camView.z(),
        m_camUp.x(), m_camUp.y(), m_camUp.z()
    );
    m_viewer->spinOnce(10);
    this->repaint();
}

void hgHMI_PCLViewerWidget::setCameraUpVector(float up_x, float up_y, float up_z)
{
    // 更新保存的向上向量
    m_camUp = Eigen::Vector3f(up_x, up_y, up_z);

    // 应用到PCLViewer
    m_viewer->setCameraPosition(
        m_camPos.x(), m_camPos.y(), m_camPos.z(),
        m_camView.x(), m_camView.y(), m_camView.z(),
        up_x, up_y, up_z);
    m_viewer->spinOnce(10);
}

void hgHMI_PCLViewerWidget::resetCameraToBox(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
{
    if (!m_viewer) return;

    vtkCamera* cam = m_viewer->getRendererCollection()->GetFirstRenderer()->GetActiveCamera();
    if (!cam) return;

    // 计算包围盒中心和尺寸
    double centerX = (minX + maxX) * 0.5;
    double centerY = (minY + maxY) * 0.5;
    double centerZ = (minZ + maxZ) * 0.5;

    double boxWidth = maxX - minX;
    double boxHeight = maxY - minY;
    double boxDepth = maxZ - minZ;

    // 获取渲染窗口大小，计算宽高比
    int* winSize = m_viewer->getRenderWindow()->GetSize();
    double aspect = (winSize[1] == 0) ? 1.0 : static_cast<double>(winSize[0]) / winSize[1];

    // 相机视角（垂直方向，单位度）
    double fovY = cam->GetViewAngle();
    double fovYRad = vtkMath::RadiansFromDegrees(fovY);
    double fovXRad = 2.0 * atan(tan(fovYRad / 2.0) * aspect);

    // 计算相机距离，保证包围盒完整进入视锥
    double distY = ((boxHeight * 1.1) / 2.0) / tan(fovYRad / 2.0);
    double distX = ((boxWidth * 1.1) / 2.0) / tan(fovXRad / 2.0);
    double distance = std::max(distX, distY) + boxDepth / 2.0;

    // 设置相机位置（这里默认相机在包围盒中心的正Z方向）
    double camX = centerX;
    double camY = centerY;
    double camZ = centerZ + distance;

    m_camPos = Eigen::Vector3f(camX, camY, camZ);
    m_camView = Eigen::Vector3f(centerX, centerY, centerZ);

    m_viewer->setCameraPosition(
        m_camPos.x(), m_camPos.y(), m_camPos.z(),
        m_camView.x(), m_camView.y(), m_camView.z(),
        m_camUp.x(), m_camUp.y(), m_camUp.z()
    );
    m_viewer->spinOnce(10);
    this->repaint();
}

void hgHMI_PCLViewerWidget::setPointPickCallback(HgHMI_PointPickCallback cb)
{
    m_pointPickCallback = cb;
}

void hgHMI_PCLViewerWidget::setDistancePickCallback(HgHMI_DistancePickCallback cb)
{
    m_distancePickCallback = cb;
    if (m_interactorStyle)
        m_interactorStyle->distanceCallback = m_distancePickCallback;
}

void hgHMI_PCLViewerWidget::setPickSphereSize(bool isLeft, double size)
{
    if (m_interactorStyle)
        m_interactorStyle->setSphereSize(isLeft, size);
}

void hgHMI_PCLViewerWidget::updateAxes()
{
    if (m_axesVisible)
    {
        m_axesActor->SetTotalLength(m_axesLength[0], m_axesLength[1], m_axesLength[2]);

        // 设置轴杆类型
        m_axesActor->SetShaftType(m_axesShaftType);

        // 设置箭头类型或隐藏箭头
        if (m_axesShowArrow)
        {
            m_axesActor->SetTipType(vtkAxesActor::CONE_TIP);
            m_axesActor->SetUserDefinedTip(nullptr);
        }
        else
        {
            // 隐藏箭头：使用用户自定义空箭头几何体
            vtkSmartPointer<vtkPolyData> emptyTip = vtkSmartPointer<vtkPolyData>::New();
            m_axesActor->SetUserDefinedTip(emptyTip);
            m_axesActor->SetTipType(vtkAxesActor::USER_DEFINED_TIP);
        }

        m_axesActor->SetAxisLabels(m_axesShowLabels);

        if (!m_renderer->HasViewProp(m_axesActor))
            m_renderer->AddActor(m_axesActor);
    }
    else
    {
        if (m_renderer->HasViewProp(m_axesActor))
            m_renderer->RemoveActor(m_axesActor);
    }

    m_renderer->ResetCamera();
    vtkCamera* camera = m_renderer->GetActiveCamera();
    camera->SetViewUp(0.0, 1.0, 0.0);
    double maxAxisLength = std::max({ m_axesLength[0], m_axesLength[1], m_axesLength[2] });
    double distance = maxAxisLength * 10.0;  // 根据实际需要调整距离
    camera->SetPosition(0.0, 0.0, distance);
    camera->SetFocalPoint(0.0, 0.0, 0.0);
    m_renderer->ResetCameraClippingRange();
    this->renderWindow()->Render();
}

void hgHMI_PCLViewerWidget::setAxesVisible(bool visible)
{
    m_axesVisible = visible;
    updateAxes();
}

void hgHMI_PCLViewerWidget::setAxesLength(double x_len, double y_len, double z_len)
{
    m_axesLength[0] = x_len;
    m_axesLength[1] = y_len;
    m_axesLength[2] = z_len;
    if (m_axesVisible)
        updateAxes();
}

void hgHMI_PCLViewerWidget::setAxesShaftType(int shaftType)
{
    // 用户输入 1: 线, 2: 圆柱, 3: 用户自定义
    switch (shaftType) {
    case 1:
        m_axesShaftType = vtkAxesActor::LINE_SHAFT;
        break;
    case 2:
        m_axesShaftType = vtkAxesActor::CYLINDER_SHAFT;
        break;
    case 3:
        m_axesShaftType = vtkAxesActor::USER_DEFINED_SHAFT;
        break;
    default:
        m_axesShaftType = vtkAxesActor::LINE_SHAFT;
        break;
    }

    if (m_axesVisible)
        updateAxes();
}

void hgHMI_PCLViewerWidget::setAxesShowLabels(bool showLabels)
{
    m_axesShowLabels = showLabels;
    if (m_axesVisible)
        updateAxes();
}

void hgHMI_PCLViewerWidget::setAxesShowArrow(bool showArrow)
{
    m_axesShowArrow = showArrow;
    if (m_axesVisible)
        updateAxes();
}



// 辅助函数：点集与路径转换
static QPainterPath pointsToPath(const QVector<QPointF>& points, bool closed = true)
{
    QPainterPath path;
    if (points.isEmpty())
        return path;

    path.moveTo(points[0]);
    for (int i = 1; i < points.size(); ++i)
        path.lineTo(points[i]);
    if (closed)
        path.closeSubpath();
    return path;
}
static QVector<QVector<QPointF>> splitPathToPolygons(const QPainterPath& path)
{
    QVector<QVector<QPointF>> polygons;
    QVector<QPointF> currentPoly;

    for (int i = 0; i < path.elementCount(); ++i)
    {
        auto e = path.elementAt(i);
        if (e.isMoveTo())
        {
            if (!currentPoly.isEmpty())
            {
                // 闭合多边形
                if (currentPoly.first() != currentPoly.last())
                    currentPoly.push_back(currentPoly.first());
                polygons.push_back(currentPoly);
                currentPoly.clear();
            }
            currentPoly.push_back(QPointF(e.x, e.y));
        }
        else if (e.isLineTo())
        {
            currentPoly.push_back(QPointF(e.x, e.y));
        }
    }
    if (!currentPoly.isEmpty())
    {
        if (currentPoly.first() != currentPoly.last())
            currentPoly.push_back(currentPoly.first());
        polygons.push_back(currentPoly);
    }

    return polygons;
}
static QVector<QPointF> pathToPoints(const QPainterPath& path)
{
    QVector<QPointF> points;
    for (int i = 0; i < path.elementCount(); ++i)
    {
        const QPainterPath::Element& e = path.elementAt(i);
        if (e.isMoveTo() || e.isLineTo())
            points.push_back(QPointF(e.x, e.y));
    }
    return points;
}
double polygonArea(const QVector<QPointF>& pts)
{
    double area = 0;
    int n = pts.size();
    for (int i = 0; i < n; ++i)
    {
        QPointF p1 = pts[i];
        QPointF p2 = pts[(i + 1) % n];
        area += (p1.x() * p2.y() - p2.x() * p1.y());
    }
    return std::abs(area) / 2.0;
}
QImage nearestNeighborScale2x(const QImage& src)
{
    if (src.isNull())
        return QImage();

    int w = src.width();
    int h = src.height();
    QImage dst(w * 2, h * 2, src.format());

    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            QRgb pixel = src.pixel(x, y);

            dst.setPixel(x * 2, y * 2, pixel);
            dst.setPixel(x * 2 + 1, y * 2, pixel);
            dst.setPixel(x * 2, y * 2 + 1, pixel);
            dst.setPixel(x * 2 + 1, y * 2 + 1, pixel);
        }
    }
    return dst;
}
QImage extendImageWithMargin(const QImage& srcImage, int margin)
{
    if (srcImage.isNull() || margin <= 0)
        return srcImage;

    int newWidth = srcImage.width() + 2 * margin;
    int newHeight = srcImage.height() + 2 * margin;

    QImage::Format fmt = srcImage.format();
    QImage extendedImage(newWidth, newHeight, fmt);

    // 针对不同格式分别处理背景填充
    switch (fmt)
    {
    case QImage::Format_Indexed8:
        // 拷贝调色板
        extendedImage.setColorTable(srcImage.colorTable());
        extendedImage.fill(0);
        break;
    case QImage::Format_Grayscale8:
        extendedImage.fill(0); // 灰度的0就是黑
        break;
    default:
        extendedImage.fill(Qt::black);
        break;
    }

    // 原图拷贝到中心
    QPainter painter(&extendedImage);
    painter.drawImage(margin, margin, srcImage);
    painter.end();

    return extendedImage;
}

//////////////////////////////////////////////////////////////
///  hgHMI_QPixelImage
///  基于 QGraphicsView 的图像显示与绘制控件
//////////////////////////////////////////////////////////////
hgHMI_QPixelImage::hgHMI_QPixelImage(QWidget* parent)
    : QGraphicsView(parent),
    m_scene(new QGraphicsScene(this)),
    m_pixmapItem(nullptr),
    m_cursorCircle(nullptr),
    m_isDragging(false),
    m_mouseInside(false),
    m_isLeftPressed(false),
    m_isRightPressed(false),
    m_isRightDragging(false),
    m_lastRightDragPos(QPoint(0, 0)),
    m_isEraseMode(false),
    m_tracks(),
    m_tracksHistory()
{
	setScene(m_scene); // 设置场景
	setBackgroundBrush(QBrush(QColor(50, 50, 50))); // 设置背景颜色为深灰色
	setMouseTracking(true); // 启用鼠标跟踪以接收鼠标移动事件
	setRenderHint(QPainter::Antialiasing, true); // 抗锯齿
	setDragMode(QGraphicsView::NoDrag); // 不使用默认拖拽模式
	setTransformationAnchor(QGraphicsView::AnchorUnderMouse); // 缩放时以鼠标位置为锚点
	setResizeAnchor(QGraphicsView::AnchorUnderMouse); // 调整视图大小时以鼠标位置为锚点
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); // 禁用滚动条
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff); // 禁用滚动条

    m_cursorCircle = new QGraphicsEllipseItem();
    QPen pen(Qt::white);
    pen.setWidth(2);
    m_cursorCircle->setPen(pen);
    m_cursorCircle->setBrush(QBrush(QColor(255, 255, 255, 50)));
    m_cursorCircle->setVisible(false);
    m_scene->addItem(m_cursorCircle);
	m_cursorCircle->setZValue(10000); // 确保在最上层

    // 预览光标圈初始化
    m_previewCursorCircle = new QGraphicsEllipseItem();
	m_previewCursorCircle->setZValue(10001); // 确保在光标圈之上
    m_previewCursorCircle->setPen(m_cursorCircle->pen());    // 直接用真实光标的Pen（白色实线）
    m_previewCursorCircle->setBrush(m_cursorCircle->brush()); // 直接用真实光标的Brush（半透明白色）
    m_previewCursorCircle->setVisible(false);
    m_scene->addItem(m_previewCursorCircle);

    // 渐变消失用定时器
    if (m_cursorCircle)
    {
        m_cursorPenAlpha = m_cursorCircle->pen().color().alphaF();
        m_cursorBrushAlpha = m_cursorCircle->brush().color().alphaF();
    }
    m_previewFadeTimer = new QTimer(this);
    m_previewFadeTimer->setInterval(30); // 30ms步进
    connect(m_previewFadeTimer, &QTimer::timeout, this, [this]() {
        m_previewAlpha -= 0.06;
        if (m_previewAlpha <= 0.0) {
            m_previewFadeTimer->stop();
            m_previewCursorCircle->setVisible(false);
            m_previewAlpha = 1.0;
        }
        else {
            // pen
            QPen pen = m_previewCursorCircle->pen();
            QColor penColor = pen.color();
            penColor.setAlphaF(m_cursorPenAlpha * m_previewAlpha);
            pen.setColor(penColor);
            m_previewCursorCircle->setPen(pen);

            // brush
            QBrush brush = m_previewCursorCircle->brush();
            QColor brushColor = brush.color();
            brushColor.setAlphaF(m_cursorBrushAlpha * m_previewAlpha);
            brush.setColor(brushColor);
            m_previewCursorCircle->setBrush(brush);
        }
        });
}

void hgHMI_QPixelImage::setImage(const QImage& image)
{
    if (!m_scene)
        return;

    QImage extendedImage = image;

    // 移除除光标以外的所有非图片图元（轨迹等）
    auto items = m_scene->items();
    for (auto item : items)
    {
        if (item != m_cursorCircle && item != m_previewCursorCircle && item != m_pixmapItem)
        {
            m_scene->removeItem(item);
        }
    }
    clearDrawings(); // 清除绘制轨迹及对象容器，会delete轨迹对象和对应图元

    // 删除旧的图片图元
    if (m_pixmapItem)
    {
        m_scene->removeItem(m_pixmapItem);
        delete m_pixmapItem;
        m_pixmapItem = nullptr;
    }

    // 如果传入图片有效，创建新图片图元，并设置场景大小
    if (!extendedImage.isNull())
    {
        QPixmap pixmap = QPixmap::fromImage(extendedImage);
        m_pixmapItem = m_scene->addPixmap(pixmap);
        m_imageSize = extendedImage.size();
        m_scene->setSceneRect(QRectF(QPointF(0, 0), m_imageSize));
        // 绘制模式不重置视角
        if (!m_leftDragEnabled && !m_isEraseMode && !m_isPointMoveMode)
            fitInView(m_scene->sceneRect(), Qt::KeepAspectRatio);
    }
    else
    {
        m_imageSize = QSize();
        m_scene->setSceneRect(QRectF());
    }
    setImageRotation(angleDegrees);

    // 确保光标图元在场景中且状态正确
    if (m_cursorCircle && !m_scene->items().contains(m_cursorCircle))
        m_scene->addItem(m_cursorCircle);
    if (m_cursorCircle)
        m_cursorCircle->setVisible(m_mouseInside);

    if (m_previewCursorCircle && !m_scene->items().contains(m_previewCursorCircle))
        m_scene->addItem(m_previewCursorCircle);
    if (m_previewCursorCircle)
        m_previewCursorCircle->setVisible(false);
}

void hgHMI_QPixelImage::setCursorRadius(qreal radius)
{
    m_cursorRadius = radius;
	m_cursorRadiusPressed = radius * 2.0 / 3.0;
	pointDragTolerance = m_cursorRadius + outLineWidth + outPointRadius; // 点拖拽容差设置为光标半径+线宽
}

QSize hgHMI_QPixelImage::imageSize() const
{
    return m_imageSize;
}

void hgHMI_QPixelImage::clearDrawings()
{
    if (!m_scene)
        return;

    // 删除除图片和光标圆外的所有Item，但不delete，留给Image_Track析构处理
    QList<QGraphicsItem*> items = m_scene->items();
    for (auto item : items)
    {
        if (item == m_pixmapItem || item == m_cursorCircle || item == m_previewCursorCircle)
            continue;
        m_scene->removeItem(item);
        // 不delete item
    }

    // 释放轨迹对象，Image_Track析构中会删除图元
    qDeleteAll(m_tracks);
    m_tracks.clear();
}

void hgHMI_QPixelImage::reset()
{
    clearDrawings();
    m_tracksHistory.clear();

    if (m_pixmapItem)
    {
        m_scene->removeItem(m_pixmapItem);
        delete m_pixmapItem;
        m_pixmapItem = nullptr;
    }

    m_lastPos = QPointF(0, 0);
    m_isLeftPressed = false;
    m_isRightPressed = false;
    m_isDragging = false;
    m_dragPath = QPainterPath();

    if (m_cursorCircle)
    {
        m_cursorCircle->setVisible(m_mouseInside);
    }
}

void hgHMI_QPixelImage::clearTrackGraphics(Image_Track* track)
{
    if (!track)
        return;

    for (auto item : track->pointItems)
    {
        if (item && item->scene())
            item->scene()->removeItem(item);
        delete item;
    }
    track->pointItems.clear();

    if (track->lineItem)
    {
        if (track->lineItem->scene())
            track->lineItem->scene()->removeItem(track->lineItem);
        delete track->lineItem;
        track->lineItem = nullptr;
    }
}

void hgHMI_QPixelImage::updateTrackGraphics(Image_Track* track)
{
    if (!track)
        return;

    QPen pen(outLineColor, outLineWidth);

    if (track->points.size() >= 3)
    {
        QPainterPath path;
        path.moveTo(track->points[0]);
        for (int i = 1; i < track->points.size(); ++i)
            path.lineTo(track->points[i]);
        path.closeSubpath();
        track->lineItem = m_scene->addPath(path, pen);
    }
    else if (track->points.size() >= 2)
    {
        QPainterPath path;
        path.moveTo(track->points[0]);
        for (int i = 1; i < track->points.size(); ++i)
            path.lineTo(track->points[i]);
        track->lineItem = m_scene->addPath(path, pen);
    }
    else
    {
        track->lineItem = nullptr;
    }
    // 设置父项为图片，保证随图片旋转
    if (track->lineItem)
        track->lineItem->setParentItem(m_pixmapItem);

    for (const QPointF& pt : track->points)
    {
        QGraphicsEllipseItem* ellipse = m_scene->addEllipse(
            pt.x() - outPointRadius, pt.y() - outPointRadius,
            2 * outPointRadius, 2 * outPointRadius,
            QPen(Qt::NoPen),
            QBrush(outPointColor));
        // 设置父项为图片
        ellipse->setParentItem(m_pixmapItem);
        track->pointItems.push_back(ellipse);
    }
}

QVector<QPointF> hgHMI_QPixelImage::scenePathToImagePoints(const QPainterPath& scenePath)
{
    QVector<QPointF> imagePoints;
    if (!m_pixmapItem)
        return pathToPoints(scenePath);

    int count = scenePath.elementCount();
    for (int i = 0; i < count; ++i)
    {
        QPainterPath::Element e = scenePath.elementAt(i);
        QPointF scenePt(e.x, e.y);
        QPointF imagePt = m_pixmapItem->mapFromScene(scenePt);
        imagePoints.push_back(imagePt);
    }
    return imagePoints;
}

QPointF hgHMI_QPixelImage::viewPosToImageCoords(const QPointF& viewPos) const
{
    if (m_imageSize.isEmpty() || !m_pixmapItem)
    {
        // 没有图片时，直接返回视图坐标当作图像坐标（或返回控件中心）
        return mapToScene(viewPos.toPoint());
    }

    QPointF scenePos = mapToScene(viewPos.toPoint());
    QPointF imagePos = m_pixmapItem->mapFromScene(scenePos);// 将场景坐标映射到图片局部坐标（考虑旋转、缩放、平移）

    // 限制坐标范围在图片尺寸内
    qreal x = qBound(0.0, imagePos.x(), (qreal)m_imageSize.width());
    qreal y = qBound(0.0, imagePos.y(), (qreal)m_imageSize.height());

    return QPointF(x, y);
}

void hgHMI_QPixelImage::drawOpenPolyline(const std::vector<Eigen::Vector2d>& points)
{
    if (points.size() < 2)
        return;
    saveTracksToHistory();

    // 新建轨迹对象
    Image_Track* track = new Image_Track();

    // 填充点
    for (const auto& pt : points)
    {
        track->points.push_back(QPointF(pt(0), pt(1)));
    }

    // 添加到轨迹集合
    updateTrackGraphics(track);
    m_tracks.push_back(track);
}

void hgHMI_QPixelImage::drawClosedPolyline(const std::vector<Eigen::Vector2d>& points, int partIndex, int outerIndex, bool isOut, bool change)
{
    if (points.size() < 3)
        return;
	if (change) saveTracksToHistory(); // 不需要发射变更信号时不保存历史

    Image_Track* track = new Image_Track();

    // 填充点
    for (const auto& pt : points)
    {
        track->points.push_back(QPointF(pt(0), pt(1)));
    }

    // 显式闭合点序列
    if (!points.empty())
    {
        // 如果点数>=3且最后一点不等于第一点，补充首点闭合
        if (track->points.front() != track->points.back())
        {
            track->points.push_back(track->points.front());
        }
    }

    updateTrackGraphics(track);
	track->isOut = isOut;
    track->partIndex = partIndex;
    track->outerIndex = outerIndex;
    m_tracks.push_back(track);
	if (change) emit trackChanged(m_tracks); // 通知添加新轨迹
}

void hgHMI_QPixelImage::drawTexts(const std::vector<QString>& texts, const std::vector<Eigen::Vector2d>& positions)
{
    int count = qMin(texts.size(), positions.size());
    for (int i = 0; i < count; ++i)
    {
        QGraphicsTextItem* item = m_scene->addText(texts[i]);
        item->setDefaultTextColor(outPointColor);
        item->setPos(positions[i](0), positions[i](1));
        Q_UNUSED(item);
    }
}

void hgHMI_QPixelImage::updateCursorCirclePosition(const QPointF& imagePos, bool pressed)
{
    qreal r = pressed ? m_cursorRadiusPressed : m_cursorRadius;
    if (!m_cursorCircle)
        return;
    // 图像坐标转为场景坐标，兼容旋转、缩放、平移等
    QPointF scenePos;
    if (m_pixmapItem) scenePos = m_pixmapItem->mapToScene(imagePos);
	else scenePos = imagePos;

    m_cursorCircle->setRect(scenePos.x() - r, scenePos.y() - r, 2 * r, 2 * r);
}

void hgHMI_QPixelImage::mousePressEvent(QMouseEvent* event)
{
    QPointF imagePos = viewPosToImageCoords(event->pos());

    if (event->button() == Qt::LeftButton)
    {
        m_isLeftPressed = true;
        if (m_isPointMoveMode) {
            // 先检测是否靠近已有点，进入点拖拽模式
            bool foundPointToDrag = false;
            //setPointTolerance(m_cursorRadiusPressed);
            for (Image_Track* track : m_tracks)
            {
                int closestPointIndex = -1;
                double minDistance = pointDragTolerance + 1.0; // 初始化为阈值+1，保证后续判断有效

                for (int i = 0; i < track->points.size(); ++i)
                {
                    double dist = QLineF(track->points[i], imagePos).length();
                    if (dist <= pointDragTolerance && dist < minDistance)
                    {
                        closestPointIndex = i;
                        minDistance = dist;
                    }
                }

                if (closestPointIndex != -1)
                {
                    // 找到最近的点，进入拖拽编辑模式
                    m_isPointDragging = true;
                    m_draggingTrack = track;
                    m_draggingTrack->updateIsClosed();
                    m_draggingPointIndex = closestPointIndex;

                    m_lastPos = imagePos;
                    setCursor(Qt::SizeAllCursor); // 更改为合适的拖动光标
                    if (m_cursorCircle)  m_cursorCircle->setVisible(false);

                    saveTracksToHistory();// 保存历史状态
                    break;
                }
            }
        }
		else { // 非点拖拽模式，检测是否为绘制或擦除模式
            m_isPointDragging = false; m_draggingTrack = nullptr;  m_draggingPointIndex = -1;
            m_lastPos = imagePos;
            updateCursorCirclePosition(imagePos, true);

            if (!m_isEraseMode && m_leftDragEnabled)
            {
                m_isDragging = false;  // 先不认为拖动，等待移动判断
                m_dragPath = QPainterPath();
                if(m_pixmapItem) m_dragPath.moveTo(m_pixmapItem->mapToScene(imagePos));
				else m_dragPath.moveTo(imagePos);

                // 创建新的实时路径图元
                if (m_currentDragPathItem)
                {
                    m_scene->removeItem(m_currentDragPathItem);
                    delete m_currentDragPathItem;
                    m_currentDragPathItem = nullptr;
                }
                m_currentDragPathItem = m_scene->addPath(m_dragPath, QPen(outLineColor, outLineWidth));

                emit onMousePositionValueChangedLeftPress(imagePos);
            }
            else if (m_isEraseMode)
            {
                // 擦除模式，实时擦除
                erasePoints(imagePos);
                updateCursorCirclePosition(imagePos, true);
            }
        } 
    }
    else if (event->button() == Qt::RightButton)
    {
        m_isRightPressed = true;
        m_lastPos = imagePos;

        m_isRightDragging = true;
        m_lastRightDragPos = event->pos();

        // 设置抓手光标（闭合手）
        setCursor(Qt::ClosedHandCursor);
        if (m_cursorCircle)
            m_cursorCircle->setVisible(false);

        updateCursorCirclePosition(imagePos, false);

        emit onMousePositionValueChangedRightPress(imagePos);
    }

    QGraphicsView::mousePressEvent(event);
}

void hgHMI_QPixelImage::mouseMoveEvent(QMouseEvent* event)
{
    QPointF imagePos = viewPosToImageCoords(event->pos());

    if (m_isPointMoveMode && m_isPointDragging && m_draggingTrack && m_draggingPointIndex >= 0)
    {
        // 更新拖拽点坐标
        m_draggingTrack->points[m_draggingPointIndex] = imagePos;

        // 判断是否闭合轨迹
        if (!m_draggingTrack->points.isEmpty())
        {
            int lastIndex = m_draggingTrack->points.size() - 1;
            
            if (m_draggingTrack->isClosed)
            {
                if (m_draggingPointIndex == 0)
                {
                    // 同时更新尾点
                    m_draggingTrack->points[lastIndex] = imagePos;
                }
                else if (m_draggingPointIndex == lastIndex)
                {
                    // 同时更新首点
                    m_draggingTrack->points[0] = imagePos;
                }
            }
        }

        // 更新轨迹图元显示
        clearTrackGraphics(m_draggingTrack);
        updateTrackGraphics(m_draggingTrack);
        viewport()->update();

        m_lastPos = imagePos;
    }

    if (m_isLeftPressed && !m_isDragging && !m_isPointDragging) {
        m_isDragging = true;
    }

    if (m_mouseInside)
    {
        if (!m_isDragging)
            updateCursorCirclePosition(imagePos, false);
        else
            updateCursorCirclePosition(imagePos, true);
    }

    if (m_isDragging && !m_isEraseMode && m_leftDragEnabled)
    {
        QPointF lastScenePos = m_dragPath.currentPosition();
        QPointF currentScenePos;
        if (m_pixmapItem) currentScenePos = m_pixmapItem->mapToScene(imagePos);
		else currentScenePos = imagePos;

        if (QLineF(lastScenePos, currentScenePos).length() >= m_minDistance)
        {
            m_dragPath.lineTo(currentScenePos);
            if (m_currentDragPathItem)
                m_currentDragPathItem->setPath(m_dragPath);
        }
    }

    if (m_isEraseMode && m_isLeftPressed)
    {
        erasePoints(imagePos);
        updateCursorCirclePosition(imagePos, true);
    }

    if (m_isRightDragging)
    {
        QPoint curPos = event->pos();
        QPoint delta = curPos - m_lastRightDragPos;

        translate(-delta.x(), -delta.y());

        m_lastRightDragPos = curPos;
    }

    if (m_isLeftPressed && !m_isEraseMode)
    {
        QPointF delta = imagePos - m_lastPos;
        emit onMousePositionValueChangedLeftMove(delta);
        m_lastPos = imagePos;
    }
    if (m_isRightPressed)
    {
        QPointF delta = imagePos - m_lastPos;
        emit onMousePositionValueChangedRightMove(delta);
        m_lastPos = imagePos;
    }

    QGraphicsView::mouseMoveEvent(event);
}

void hgHMI_QPixelImage::mouseReleaseEvent(QMouseEvent* event)
{
    QPointF imagePos = viewPosToImageCoords(event->pos());
    updateCursorCirclePosition(imagePos, false);

    // 删除实时路径图元
    if (!m_mouseInside)
    {
        m_scene->removeItem(m_currentDragPathItem);
        delete m_currentDragPathItem;
        m_currentDragPathItem = nullptr;
    }

    if (event->button() == Qt::LeftButton)
    {
        m_isLeftPressed = false;

        if (m_isPointMoveMode && m_isPointDragging)
        {
            m_isPointDragging = false;
            // 释放鼠标恢复光标状态
            if (m_mouseInside) {
                setCursor(Qt::BlankCursor);
                if (m_cursorCircle)
                    m_cursorCircle->setVisible(true);
            }
            else setCursor(Qt::ArrowCursor);

            if (m_draggingTrack)
            {
                emit trackChanged(m_tracks);
            }

            m_draggingTrack = nullptr;
            m_draggingPointIndex = -1;
        }
        else if (m_isDragging) {
            m_isDragging = false;
            if (!m_isEraseMode && m_leftDragEnabled)
            {
                QVector<QPointF> scenePoints = scenePathToImagePoints(m_dragPath);
                QPainterPath scenePath = pointsToPath(scenePoints);
                QVector<QPointF> newKeyPoints = extractKeyPointsFromPath(scenePath); // 将绘制轨迹化为多边形角点

                if (m_tracks.isEmpty())
                {
                    // 无轨迹，直接添加
                    std::vector<Eigen::Vector2d> eigenPts;
                    for (const QPointF& pt : newKeyPoints)
                        eigenPts.emplace_back(pt.x(), pt.y());
                    drawClosedPolyline(eigenPts);
                }
                else
                {
                    QPainterPath newPath = pointsToPath(newKeyPoints, true);
                    bool intersectedAnyTrack = false;

                    QPainterPath mergedNewPath = newPath;// 新轨迹所有需要合并的部分，先用QPainterPath合并
                    QVector<int> tracksToRemove;// 用来存储需要删除的旧轨迹索引

                    // 循环遍历所有轨迹，处理交集和拆分（倒序遍历：约定外轮廓靠前）
                    for (int i = m_tracks.size() - 1; i >= 0; --i)
                    {
                        Image_Track* oldTrack = m_tracks[i];
                        QPointF originalStartPoint = m_tracks[i]->points.front();// 保存旧轨迹起点
                        if (!oldTrack)
                            continue;

                        QPainterPath oldPath = pointsToPath(oldTrack->points, true);
                        QPainterPath intersectPath = oldPath.intersected(newPath); // 计算区域面交集
                        if (intersectPath.isEmpty()) continue;// 无区域面交集，保留轨迹不变
                        intersectedAnyTrack = true;

                        // 如果新轨迹完全在旧轨迹内
                        if (oldPath.contains(newPath)) {
                            // 直接添加新轨迹
                            std::vector<Eigen::Vector2d> eigenPts;
                            for (const QPointF& pt : newKeyPoints)
                                eigenPts.emplace_back(pt.x(), pt.y());
                            int maxOuterIndex = -1;
                            for (const auto& track : m_tracks)
                            {
                                if (track && track->partIndex == m_tracks[i]->partIndex)
                                {
                                    if (track->outerIndex > maxOuterIndex)
                                        maxOuterIndex = track->outerIndex;
                                }
                            }
                            saveTracksToHistory();
                            drawClosedPolyline(eigenPts, m_tracks[i]->partIndex, maxOuterIndex + 1);
                            break; // 结束处理
                        }

						// 如果旧轨迹完全在新轨迹内
                        if (newPath.contains(oldPath)) {
                            std::vector<Eigen::Vector2d> eigenPts;
                            for (const QPointF& pt : newKeyPoints)
                                eigenPts.emplace_back(pt.x(), pt.y());
                            bool isOuter = false;
                            int outerIndex = -1;
                            for (auto& track : m_tracks)
                            {
                                if (track && (track->partIndex == m_tracks[i]->partIndex) && track->isOut)
                                {
                                    QPainterPath temp = pointsToPath(track->points, true);
                                    if (newPath.contains(temp)) isOuter = true; // 被包含的旧轮廓是否为外轮廓
                                }
                            }

                            for (auto& track : m_tracks)
                            {
                                if (track && track->partIndex == m_tracks[i]->partIndex)
                                {
                                    if (isOuter) {
                                        track->isOut = false; // 如果新轨迹包含旧轨迹为外轮廓，则新轨迹为外轮廓，所有原始轮廓变为内轮廓
                                        track->outerIndex += 1; // 所有同partIndex轨迹outerIndex加1
                                    }
                                    else {
                                        // 只是包含了某个内轮廓，则新轨迹为内轮廓，原本的outerIndex不变,添加为新的内轮廓
                                        if (track->outerIndex > outerIndex)
                                            outerIndex = track->outerIndex;
                                    }
                                }
                            }

                            saveTracksToHistory();
                            drawClosedPolyline(eigenPts, m_tracks[i]->partIndex, outerIndex + 1, true); // 添加为外轮廓
                            break; // 结束处理
                        }

                        // 有区域面交集，拆分旧轨迹
                        QPainterPath diffPath = oldPath.subtracted(intersectPath); // 计算差集
                        QVector<QVector<QPointF>> splitPolygons = splitPathToPolygons(diffPath); // 拆分成多个多边形
                        if (splitPolygons.isEmpty()) continue;

                        // 选择面积最大的多边形保留
                        int maxAreaIndex = 0;
                        double maxArea = 0;
                        for (int j = 0; j < splitPolygons.size(); ++j)
                        {
                            double area = polygonArea(splitPolygons[j]);
                            if (area > maxArea)
                            {
                                maxArea = area;
                                maxAreaIndex = j;
                            }
                        }
                        QVector<QPointF> retainedPart = splitPolygons[maxAreaIndex];

                        // 转成QPainterPath后，合并入新路径
                        QPainterPath retainedPath = pointsToPath(retainedPart, true);
                        mergedNewPath = mergedNewPath.united(retainedPath);
                        QVector<QPointF> finalPts = pathToPoints(mergedNewPath);

                        // 找到最接近原点的新点序列索引
                        int closestIndex = 0;
                        double minDist = std::numeric_limits<double>::max();
                        for (int idx = 0; idx < finalPts.size(); ++idx)
                        {
                            double dist = QLineF(originalStartPoint, finalPts[idx]).length();
                            if (dist < minDist)
                            {
                                minDist = dist;
                                closestIndex = idx;
                            }
                        }

                        // 旋转点序列使起点对齐
                        bool isClosed = (finalPts.front() == finalPts.back());

                        int n = isClosed ? finalPts.size() - 1 : finalPts.size();

                        QVector<QPointF> reorderedPts;
                        for (int k = 0; k < n; ++k)
                        {
                            reorderedPts.push_back(finalPts[(closestIndex + k) % n]);
                        }

                        // 确保闭合
                        if (reorderedPts.front() != reorderedPts.back())
                        {
                            reorderedPts.push_back(reorderedPts.front());
                        }

                        saveTracksToHistory(); // 保存历史
                        clearTrackGraphics(m_tracks[i]);
                        m_tracks[i]->points = reorderedPts;
                        updateTrackGraphics(m_tracks[i]);
                        emit trackChanged(m_tracks);
                        break; // 已处理该轨迹，跳出循环
                    }

                    if (!intersectedAnyTrack)
                    {
                        // 没有和任何轨迹相交，直接添加为新轨迹
                        std::vector<Eigen::Vector2d> eigenPts;
                        for (const QPointF& pt : newKeyPoints)
                            eigenPts.emplace_back(pt.x(), pt.y());
                        saveTracksToHistory(); // 保存历史
                        drawClosedPolyline(eigenPts, -1, -1, true); // 标记为新增的外轮廓
                    }
                }

                if (m_currentDragPathItem)
                {
                    m_scene->removeItem(m_currentDragPathItem);
                    delete m_currentDragPathItem;
                    m_currentDragPathItem = nullptr;
                }

                m_dragPath = QPainterPath();
            }
        }
        else
        {
            if (!m_isEraseMode && m_leftDragEnabled) {
                // 单击添加点逻辑，设定最大距离阈值
                insertPointAtNearestTrack(imagePos, maxClickDistance);
            }
        }

        emit onMousePositionValueChangedLeftRelease(imagePos);
    }
    else if (event->button() == Qt::RightButton)
    {
        m_isRightPressed = false;

        if (m_isRightDragging)
        {
            m_isRightDragging = false;
            // 释放鼠标恢复光标状态
            if (m_mouseInside) {
                setCursor(Qt::BlankCursor);
                if (m_cursorCircle)
                    m_cursorCircle->setVisible(true);
            }
            else setCursor(Qt::ArrowCursor);
        }

        emit onMousePositionValueChangedRightRelease(imagePos);
    }
    QGraphicsView::mouseReleaseEvent(event);
}

void hgHMI_QPixelImage::wheelEvent(QWheelEvent* event)
{
    double scaleFactor = 1.15; // 缩放因子
    if (event->angleDelta().y() > 0)
    {
        // 放大
        scale(scaleFactor, scaleFactor);
    }
    else
    {
        // 缩小
        scale(1.0 / scaleFactor, 1.0 / scaleFactor);
    }
}

void hgHMI_QPixelImage::enterEvent(QEvent* event)
{
    QGraphicsView::enterEvent(event);

    m_mouseInside = true;
    if (!m_isRightDragging)
    {
        setCursor(Qt::BlankCursor);
        if (m_cursorCircle)
            m_cursorCircle->setVisible(true);
    }
    else
    {
        // 保持抓手光标，圆隐藏
        setCursor(Qt::ClosedHandCursor);
        if (m_cursorCircle)
            m_cursorCircle->setVisible(false);
    }

    if (m_cursorCircle)
        m_cursorCircle->setVisible(true);
}

void hgHMI_QPixelImage::leaveEvent(QEvent* event)
{
    QGraphicsView::leaveEvent(event);

    m_mouseInside = false;
    setCursor(Qt::ArrowCursor);
    if (m_cursorCircle)
        m_cursorCircle->setVisible(false);

    setCursor(Qt::ArrowCursor);

    if (m_isDragging)
    {
        m_isDragging = false;

        // 删除拖拽路径图元
        if (m_currentDragPathItem)
        {
            m_scene->removeItem(m_currentDragPathItem);
            delete m_currentDragPathItem;
            m_currentDragPathItem = nullptr;
        }
    }

    if (m_isRightDragging)
    {
        m_isRightDragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

void hgHMI_QPixelImage::setImageRotation(qreal angle)
{
    if (!m_pixmapItem) {
        angleDegrees = angle;
        return;
    }

    // 设置旋转中心为图片中心，避免旋转错位
    QSize sz = m_imageSize;
    m_pixmapItem->setTransformOriginPoint(sz.width() / 2.0, sz.height() / 2.0);

    // 设置旋转角度（顺时针为正角度，单位为度）
    m_pixmapItem->setRotation(angle);
}

void hgHMI_QPixelImage::drawPointMarker(const QPointF& pos)
{
    QGraphicsEllipseItem* ellipse = m_scene->addEllipse(
        pos.x() - outPointRadius, pos.y() - outPointRadius,
        2 * outPointRadius, 2 * outPointRadius,
        QPen(Qt::NoPen),
        QBrush(Qt::blue)
    );
    Q_UNUSED(ellipse);
}

QVector<QPointF> hgHMI_QPixelImage::extractKeyPointsFromPath(const QPainterPath& inputPath) const
{
    // 从 QPainterPath 中提取点
    std::vector<Eigen::Vector3d> points;
    int count = inputPath.elementCount();
    points.reserve(count);
    for (int i = 0; i < count; ++i)
    {
        QPainterPath::Element elem = inputPath.elementAt(i);
        points.emplace_back(elem.x, elem.y, 0.0); // z = 0
    }

    // 调用 Douglas-Peucker 算法
    std::vector<Eigen::Vector3d> simplifiedPoints = hgCloud::douglasPeucker(points, m_minDistance);

    // 转换回 QVector<QPointF>
    QVector<QPointF> result;
    result.reserve((int)simplifiedPoints.size());
    for (const Eigen::Vector3d& pt : simplifiedPoints)
    {
        result.append(QPointF(pt.x(), pt.y()));
    }

    return result;

    //QVector<QPointF> points;
    //for (int i = 0; i < inputPath.elementCount(); ++i)
    //{
    //    const QPainterPath::Element& e = inputPath.elementAt(i);
    //    if (e.isMoveTo() || e.isLineTo())
    //        points.push_back(QPointF(e.x, e.y));
    //}
    //return points;
}

void hgHMI_QPixelImage::insertPointAtNearestTrack(const QPointF& pos, double maxDistance)
{
    int nearestTrackIndex = -1;  // 最近轨迹索引
    int insertIndex = -1;         // 插入点索引
    double minDist = maxDistance; // 最短距离阈值

    // 遍历所有轨迹
    for (int t = 0; t < m_tracks.size(); ++t)
    {
        Image_Track* track = m_tracks[t];
        const auto& pts = track->points;
        int n = pts.size();

        if (n == 0) continue;
        else if (n == 1)
        {
            // 轨迹只有一个点，判断点击点到该点的距离
            double dist = QLineF(pos, pts[0]).length();
            if (dist < minDist)
            {
                minDist = dist;
                nearestTrackIndex = t;
                insertIndex = 1; // 插入点在唯一点后面（末尾）
            }
        }
        else if (n == 2)
        {
            // 两点轨迹，计算点击点在两点连线的投影位置
            QPointF p1 = pts[0], p2 = pts[1];
            QLineF line(p1, p2);
            double lengthSquared = line.length() * line.length();

            // 计算t，表示点击点在线段上的参数位置
            double t = 0.0;
            if (lengthSquared > 1e-12)
            {
                t = ((pos.x() - p1.x()) * (p2.x() - p1.x()) + (pos.y() - p1.y()) * (p2.y() - p1.y())) / lengthSquared;
            }

            // 计算点击点到线段的距离
            QPointF proj; // 投影点
            if (t < 0.0)
                proj = p1;
            else if (t > 1.0)
                proj = p2;
            else
                proj = QPointF(p1.x() + t * (p2.x() - p1.x()), p1.y() + t * (p2.y() - p1.y()));

            double dist = QLineF(pos, proj).length();

            if (dist < minDist)
            {
                minDist = dist;

                nearestTrackIndex = t;

                if (t < 0.0)
                {
                    insertIndex = 0; // 在开头插入
                }
                else if (t <= 1.0)
                {
                    insertIndex = 1; // 插入点0和点1之间
                }
                else
                {
                    insertIndex = 2; // 末尾插入
                }
            }
        }
        else // n >= 3
        {
            // 先找距离点击点最近的轨迹点索引
            int idxNearest = -1;
            double distNearest = maxDistance;

            for (int i = 0; i < n; ++i)
            {
                double dist = QLineF(pos, pts[i]).length();
                if (dist < distNearest)
                {
                    distNearest = dist;
                    idxNearest = i;
                }
            }

            if (idxNearest == -1)
                continue; // 本轨迹无满足距离的点

            // 找前后相邻点构成的线段，计算点击点到这些线段距离
            auto distanceToSegment = [](const QPointF& p, const QPointF& a, const QPointF& b) -> double {
                QLineF line(a, b);
                double lengthSquared = line.length() * line.length();
                if (lengthSquared < 1e-12)
                    return QLineF(p, a).length();

                double t = ((p.x() - a.x()) * (b.x() - a.x()) + (p.y() - a.y()) * (b.y() - a.y())) / lengthSquared;

                if (t < 0.0)
                    return QLineF(p, a).length();
                else if (t > 1.0)
                    return QLineF(p, b).length();
                else
                {
                    QPointF foot(a.x() + t * (b.x() - a.x()), a.y() + t * (b.y() - a.y()));
                    return QLineF(p, foot).length();
                }
                };

            double distPrev = std::numeric_limits<double>::max();
            double distNext = std::numeric_limits<double>::max();

            // 计算与前一个点组成线段距离
            if (idxNearest > 0)
                distPrev = distanceToSegment(pos, pts[idxNearest - 1], pts[idxNearest]);

            // 计算与后一个点组成线段距离
            if (idxNearest < n - 1)
                distNext = distanceToSegment(pos, pts[idxNearest], pts[idxNearest + 1]);

            // 根据较近线段选择插入位置
            if (distPrev < distNext)
            {
                if (distPrev < minDist)
                {
                    minDist = distPrev;
                    nearestTrackIndex = t;
                    insertIndex = idxNearest; // 插入到前面点之后，即idxNearest点前插入
                }
            }
            else
            {
                if (distNext < minDist)
                {
                    minDist = distNext;
                    nearestTrackIndex = t;
                    insertIndex = idxNearest + 1; // 插入到idxNearest点和其后点之间
                }
            }
        }
    }

    if (nearestTrackIndex >= 0 && insertIndex >= 0)
    {
        saveTracksToHistory();

        Image_Track* track = m_tracks[nearestTrackIndex];

        // 插入点击点pos，插入位置insertIndex（范围已经合理）
        if (insertIndex >= 0 && insertIndex <= track->points.size())
        {
            track->points.insert(insertIndex, pos);
        }
        else
        {
            track->points.push_back(pos);
        }

        clearTrackGraphics(track);
        emit trackChanged(m_tracks);
        updateTrackGraphics(track);
    }
    else
    {
        // 点击点距离所有轨迹过远，可考虑新建轨迹或忽略
        /*
        Image_Track* newTrack = new Image_Track();
        newTrack->points.push_back(pos);
        m_tracks.push_back(newTrack);
        updateTrackGraphics(newTrack);
        */
    }
}

void hgHMI_QPixelImage::erasePoints(const QPointF& pos)
{
    bool erasedAny = false;

    if (m_isWholeTrackEraseMode)
    {
        // 整体擦除模式：只要轨迹上有点靠近，就删除整条轨迹
        for (int t = m_tracks.size() - 1; t >= 0; --t)
        {
            Image_Track* track = m_tracks[t];
            bool closeEnough = false;
            for (const QPointF& pt : track->points)
            {
                if (QLineF(pt, pos).length() <= m_cursorRadiusPressed)
                {
                    closeEnough = true;
                    break;
                }
            }

            if (closeEnough)
            {
                saveTracksToHistory();
                // 删除轨迹所有点图元
                for (QGraphicsEllipseItem* item : track->pointItems)
                {
                    if (item && item->scene())
                        item->scene()->removeItem(item);
                    delete item;
                }
                track->pointItems.clear();

                // 删除轨迹线图元
                if (track->lineItem)
                {
                    if (track->lineItem->scene())
                        track->lineItem->scene()->removeItem(track->lineItem);
                    delete track->lineItem;
                    track->lineItem = nullptr;
                }

                // 删除轨迹对象
                emit trackDeleted(track);
                delete track;
                m_tracks.removeAt(t);
                erasedAny = true;
            }
        }
    }
    else // 点擦除模式
    {
        for (int t = m_tracks.size() - 1; t >= 0; --t)
        {
            Image_Track* track = m_tracks[t];

            // 遍历轨迹点
            for (int i = track->points.size() - 1; i >= 0; --i)
            {
                // 不擦除起点和终点
                if (i == 0 || i == track->points.size() - 1)
                    continue;
                if (QLineF(track->points[i], pos).length() <= m_cursorRadiusPressed)
                {
                    saveTracksToHistory();
                    // 删除点图元
                    for (int j = 0; j < track->pointItems.size(); j++) {
                        m_scene->removeItem(track->pointItems[j]);
                        delete track->pointItems[j];
                        track->pointItems[j] = nullptr;
                    }
                    // 删除轨迹图元
                    if (track->lineItem)
                    {
                        m_scene->removeItem(track->lineItem);
                        delete track->lineItem;
                        track->lineItem = nullptr;
                    }
                    track->points.removeAt(i);
                    updateTrackGraphics(track);
                    emit trackChanged(m_tracks);
                    erasedAny = true;
                }
            }
            if (track->points.size() == 2) // 只剩起点和终点，删除整条轨迹
            {
                emit trackDeleted(track);
                delete track;
                m_tracks.removeAt(t);
            }
        }
    }

    if (erasedAny)
    {
        viewport()->update();
    }
}

void hgHMI_QPixelImage::setEraseMode(bool erase)
{
    m_isEraseMode = erase;
    if (erase)
    {
        // 进入擦除模式时，取消绘制拖拽状态，禁用左键拖拽
        m_leftDragEnabled = false;
        m_isDragging = false;
        m_isLeftPressed = false;
        m_isPointMoveMode = false;
        m_isPointDragging = false;

        m_dragPath = QPainterPath();
        if (m_cursorCircle)
            m_cursorCircle->setVisible(m_mouseInside);
        setCursor(Qt::CrossCursor); // 擦除模式用十字，可按需自定义
    }
    else
    {
        if (m_cursorCircle)
            m_cursorCircle->setVisible(m_mouseInside);
        setCursor(Qt::ArrowCursor);
    }
}

void hgHMI_QPixelImage::setWholeTrackEraseMode(bool enabled)
{
    if (m_isWholeTrackEraseMode != enabled)
    {
        m_isWholeTrackEraseMode = enabled;
        if (enabled)
        {
            // 进入整体擦除模式时，需要进入擦除模式，禁用绘制模式
            m_isEraseMode = true;
            m_leftDragEnabled = false;
            setCursor(Qt::CrossCursor);
        }
        else
        {
            // 退出整体擦除模式时恢复默认光标
            setCursor(Qt::ArrowCursor);
        }

        if (m_cursorCircle)
            m_cursorCircle->setVisible(m_mouseInside);
    }
}

bool hgHMI_QPixelImage::isWholeTrackEraseMode() const
{
    return m_isWholeTrackEraseMode;
}

void hgHMI_QPixelImage::setLeftDragEnabled(bool enabled)
{
    m_leftDragEnabled = enabled;
    if (enabled) {
        m_isEraseMode = false;
        m_isWholeTrackEraseMode = false;
        m_isPointMoveMode = false;
        m_isPointDragging = false;
    }
}

void hgHMI_QPixelImage::setPointMoveMode(bool enabled)
{
    m_isPointMoveMode = enabled;
    if (enabled) {
        m_isEraseMode = false;
        m_isWholeTrackEraseMode = false;
        m_leftDragEnabled = false;
        m_isDragging = false;
        m_isLeftPressed = false;
    }
}

void hgHMI_QPixelImage::setLineWidthAndPointRadius(double lineWidth, double pointRadius)
{
    outLineWidth = lineWidth;
    outPointRadius = pointRadius;
	pointDragTolerance = m_cursorRadius + outLineWidth + outPointRadius; // 点拖拽容差设置为光标半径+线宽+点半径

    // 更新所有轨迹图元的线宽和点尺寸
    for (Image_Track* track : m_tracks)
    {
        if (track->lineItem)
        {
            QPen pen = track->lineItem->pen();
            pen.setWidthF(outLineWidth);
            track->lineItem->setPen(pen);
        }

        for (QGraphicsEllipseItem* ptItem : track->pointItems)
        {
            if (ptItem)
            {
                QPointF center = ptItem->rect().center();
                QRectF newRect(center.x() - outPointRadius, center.y() - outPointRadius,
                    2 * outPointRadius, 2 * outPointRadius);
                ptItem->setRect(newRect);
            }
        }
    }
    // 更新光标圆的线宽
    if (m_cursorCircle)
    {
        QPen pen = m_cursorCircle->pen();
        pen.setWidthF(outLineWidth);
        m_cursorCircle->setPen(pen);
        m_cursorCircle->update();
    }
    if (m_previewCursorCircle)
    {
        QPen pen = m_previewCursorCircle->pen();
        pen.setWidthF(outLineWidth);
        m_previewCursorCircle->setPen(pen);
        m_previewCursorCircle->update();
    }
}

void hgHMI_QPixelImage::setTrackLineColor(Image_Track* track, const QColor& color)
{
    if (!track)
        return;

    if (track->lineItem)
    {
        QPen pen = track->lineItem->pen();
        pen.setColor(color);
        track->lineItem->setPen(pen);
    }
}

void hgHMI_QPixelImage::setTrackLineColor(int index, const QColor& color)
{
    if (index < 0 || index >= m_tracks.size())
        return;
    setTrackLineColor(m_tracks[index], color);
}

void hgHMI_QPixelImage::setTracksColor(const QColor& lineColor, const QColor& pointColor)
{
    // 保存全局颜色变量
    outLineColor = lineColor;
    outPointColor = pointColor;

    // 遍历所有轨迹，更新对应图元颜色
    for (auto track : m_tracks)
    {
        if (!track)
            continue;

        // 更新轨迹线颜色
        if (track->lineItem)
        {
            QPen pen = track->lineItem->pen();
            pen.setColor(outLineColor);
            track->lineItem->setPen(pen);
        }

        // 更新所有轨迹点颜色
        for (auto pointItem : track->pointItems)
        {
            if (!pointItem)
                continue;
            QBrush brush = pointItem->brush();
            brush.setColor(outPointColor);
            pointItem->setBrush(brush);
        }
    }
}

void hgHMI_QPixelImage::saveTracksToHistory()
{
    QVector<TrackHistoryData> currentTracks;
    for (const auto& track : m_tracks)
    {
        TrackHistoryData data;
        data.points = track->points;
        data.partIndex = track->partIndex;
        data.outerIndex = track->outerIndex;
        data.isOut = track->isOut;
        currentTracks.push_back(data);
    }
    m_tracksHistory.push(currentTracks);
}

void hgHMI_QPixelImage::undo()
{
    if (m_tracksHistory.isEmpty())
        return;

    clearDrawings();// 清除现有所有图元和轨迹对象
    QVector<TrackHistoryData> prevTracks = m_tracksHistory.pop();// 弹出历史记录并重建轨迹

    for (const TrackHistoryData& data : prevTracks)
    {
        if (data.points.size() >= 2)
        {
            Image_Track* track = new Image_Track();
            track->points = data.points;
            track->partIndex = data.partIndex;
            track->outerIndex = data.outerIndex;
            track->isOut = data.isOut;

            updateTrackGraphics(track);
            m_tracks.push_back(track);
        }
    }
    emit trackChanged(m_tracks);
    viewport()->update();
}

void hgHMI_QPixelImage::previewCursorRadius(qreal radius)
{
    QPoint viewportCenter = viewport()->rect().center();
    QPointF sceneCenter = mapToScene(viewportCenter);

    m_previewCursorCircle->setRect(sceneCenter.x() - radius, sceneCenter.y() - radius, 2 * radius, 2 * radius);

    m_previewAlpha = 1.0;

    // 设置预览光标样式为真实光标样式
    m_previewCursorCircle->setPen(m_cursorCircle->pen());
    m_previewCursorCircle->setBrush(m_cursorCircle->brush());

    // 用原始透明度乘以当前渐变透明度，保证同步渐变
    QPen pen = m_previewCursorCircle->pen();
    QColor penColor = pen.color();
    penColor.setAlphaF(m_cursorPenAlpha * m_previewAlpha);
    pen.setColor(penColor);
    m_previewCursorCircle->setPen(pen);

    QBrush brush = m_previewCursorCircle->brush();
    QColor brushColor = brush.color();
    brushColor.setAlphaF(m_cursorBrushAlpha * m_previewAlpha);
    brush.setColor(brushColor);
    m_previewCursorCircle->setBrush(brush);

    m_previewCursorCircle->setVisible(true);

    m_previewFadeTimer->stop();
    m_previewFadeTimer->start();
}



//////////////////////////////////////////////////////////////
/// hgHMI_PointCloud2DView实现
/// 点云XZ视图控件
/// 应用于单帧点云的XZ平面显示与交互
/// 
/////////////////////////////////////////////////////////////////
//// 设置点云
//ui->widget->setPointCloud(debugCloud);
//
//// 设置暗色背景
//ui->widget->setBackgroundColor(QColor(30, 30, 30));         // 深色背景
//ui->widget->setAxisColor(Qt::white);                        // 坐标轴白色
//ui->widget->setGridColor(QColor(100, 100, 100));            // 网格线暗灰色
//ui->widget->setTickLabelColor(Qt::white);                   // 刻度标签白色
//ui->widget->setAuxiliaryLineColor(QColor(255, 128, 0));     // 辅助线橙色
//ui->widget->setPointColor(Qt::cyan);                        // 点云颜色青色
//ui->widget->setHighlightColor(Qt::yellow);                  // 选中点黄色
//
//// 设置参数
//ui->widget->setMargins(90, 40, 20, 40);           // 整体边距
//ui->widget->setPointRadius(4);                     // 普通点大小稍大
//ui->widget->setHighlightRadius(6);                 // 高亮点大小
//
//ui->widget->setTickLabelFontSize(8);               // 刻度标签字体大小
//ui->widget->setTickLabelDecimals(1);               // 保留1位小数
//ui->widget->setNumMainTicks(10);                    // 主刻度数
//ui->widget->setNumSubTicks(4);                      // 副刻度数
//ui->widget->setMarginPointCloud(0.01);              // 点云边距留白比例 1%
//
//// 添加自定义点，颜色绿色，半径8
//QVector<QPointF> customPts;
//customPts << QPointF(0.5, 0.5) << QPointF(1.0, 1.0) << QPointF(1.5, 0.7);
//ui->widget->addCustomPoints(customPts, Qt::green, 8);
///
/////////////////////////////////////////////////////////////////
hgHMI_PointCloud2DView::hgHMI_PointCloud2DView(QWidget* parent)
    : QWidget(parent),
    minX(0), maxX(1), minZ(0), maxZ(1),
    scale(1.0),
    centerScene(0.5, 0.5)
{
     setMinimumSize(400, 300);
     setMouseTracking(true);
}

void hgHMI_PointCloud2DView::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud, char axisX, char axisY)
{
    points.clear();
    if (!cloud) return;

    points.reserve(cloud->size());

    // lambda：根据axis字符获取对应坐标值
    auto getCoord = [](const pcl::PointXYZ& pt, char axis) -> double {
        switch (axis) {
        case 'x':
        case 'X':
            return pt.x;
        case 'y':
        case 'Y':
            return pt.y;
        case 'z':
        case 'Z':
            return pt.z;
        default:
            return 0.0; // 默认0
        }
        };

    for (const auto& pt : *cloud) {
        double xVal = getCoord(pt, axisX);
        double yVal = getCoord(pt, axisY);
        points.push_back(QPointF(xVal, yVal));
    }

    updateBoundingRect();
    selectedPointIndex = -1; // 新点云取消选中
    update();
}

void hgHMI_PointCloud2DView::setPointCloud(const pcl::PointCloud<pcl::PointXYZ>* cloud, char axisX, char axisY)
{
    points.clear();
    if (!cloud) return;

    points.reserve(cloud->size());

    auto getCoord = [](const pcl::PointXYZ& pt, char axis) -> double {
        switch (axis) {
        case 'x':
        case 'X':
            return pt.x;
        case 'y':
        case 'Y':
            return pt.y;
        case 'z':
        case 'Z':
            return pt.z;
        default:
            return 0.0;
        }
        };

    for (const auto& pt : *cloud) {
        double xVal = getCoord(pt, axisX);
        double yVal = getCoord(pt, axisY);
        points.push_back(QPointF(xVal, yVal));
    }

    updateBoundingRect();
    selectedPointIndex = -1;
    update();
}

void hgHMI_PointCloud2DView::addCustomPoints(const QVector<QPointF>& pts, const QColor& color, int radius)
{
    for (const auto& pt : pts) {
        customPoints.push_back({ pt, color, radius });
    }
    update();
}

void hgHMI_PointCloud2DView::clearCustomPoints()
{
    customPoints.clear();
    update();
}

void hgHMI_PointCloud2DView::setTheme(int themeId)
{
    if (themeId == 1) // 暗色主题
    {
        setBackgroundColor(QColor(30, 30, 30));         // 深色背景
        setAxisColor(Qt::white);                        // 坐标轴白色
        setGridColor(QColor(100, 100, 100));            // 网格线暗灰色
        setTickLabelColor(Qt::white);                   // 刻度标签白色
        setAuxiliaryLineColor(QColor(255, 128, 0));     // 辅助线橙色
        setPointColor(Qt::cyan);                        // 点云颜色青色
        setHighlightColor(Qt::yellow);                  // 选中点黄色

        setMargins(90, 40, 20, 40);           // 整体边距
        setPointRadius(4);                     // 普通点稍大
        setHighlightRadius(6);                 // 高亮点稍大

        setTickLabelFontSize(8);               // 刻度标签字体大小
        setTickLabelDecimals(1);               // 保留1位小数
        setNumMainTicks(10);                    // 主刻度数
        setNumSubTicks(4);                      // 副刻度数
        setMarginPointCloud(0.01);              // 点云边距留白比例 1%
    }
    else // 默认主题
    {
        setBackgroundColor(Qt::white);
        setAxisColor(Qt::black);
        setGridColor(Qt::gray);
        setTickLabelColor(Qt::black);
        setAuxiliaryLineColor(Qt::red);
        setPointColor(Qt::blue);
        setHighlightColor(Qt::red);

        setMargins(90, 40, 20, 40);
        setPointRadius(3);
        setHighlightRadius(5);

        setTickLabelFontSize(8);
        setTickLabelDecimals(2);
        setNumMainTicks(10);
        setNumSubTicks(4);
        setMarginPointCloud(0.01);
    }

    update(); // 主题切换后重绘
}

void hgHMI_PointCloud2DView::setMargins(double left, double bottom, double top, double right)
{
    marginLeft = left;
    marginBottom = bottom;
    marginTop = top;
    marginRight = right;
    update();
}

void hgHMI_PointCloud2DView::getMargins(double& left, double& bottom, double& top, double& right) const
{
    left = marginLeft;
    bottom = marginBottom;
    top = marginTop;
    right = marginRight;
}

void hgHMI_PointCloud2DView::setPointColor(const QColor& color)
{
    normalPointColor = color;
    update();
}

void hgHMI_PointCloud2DView::setPointRadius(int radius)
{
    normalPointRadius = radius;
    update();
}

int hgHMI_PointCloud2DView::pointRadius() const
{
    return normalPointRadius;
}

void hgHMI_PointCloud2DView::setHighlightColor(const QColor& color)
{
    highlightPointColor = color;
    update();
}

void hgHMI_PointCloud2DView::setHighlightRadius(int radius)
{
    highlightPointRadius = radius;
    update();
}

int hgHMI_PointCloud2DView::highlightRadius() const
{
    return highlightPointRadius;
}

void hgHMI_PointCloud2DView::setTickLabelFontSize(int fontSize)
{
    tickFontSize = fontSize;
    update();
}

void hgHMI_PointCloud2DView::setTickLabelDecimals(int decimals)
{
    tickLabelDecimals = decimals;
    update();
}

void hgHMI_PointCloud2DView::setNumMainTicks(int count)
{
    if (count > 0 && count != numMainTicks) {
        numMainTicks = count;
        update();
    }
}

void hgHMI_PointCloud2DView::setNumSubTicks(int count)
{
    if (count >= 0 && count != numSubTicks) {
        numSubTicks = count;
        update();
    }
}

void hgHMI_PointCloud2DView::setMarginPointCloud(double margin)
{
    if (margin >= 0) { // 可以允许0或更大
        marginPointCloud = margin;
        updateBoundingRect();
        update();
    }
}

void hgHMI_PointCloud2DView::setBackgroundColor(const QColor& color)
{
    if (backgroundColor != color) {
        backgroundColor = color;
        update(); // 触发重绘
    }
}

void hgHMI_PointCloud2DView::setAxisColor(const QColor& color) {
    if (axisColor != color) {
        axisColor = color;
        update();
    }
}

void hgHMI_PointCloud2DView::setGridColor(const QColor& color) {
    if (gridColor != color) {
        gridColor = color;
        update();
    }
}

void hgHMI_PointCloud2DView::setTickLabelColor(const QColor& color) {
    if (tickLabelColor != color) {
        tickLabelColor = color;
        update();
    }
}

void hgHMI_PointCloud2DView::setAuxiliaryLineColor(const QColor& color) {
    if (auxiliaryLineColor != color) {
        auxiliaryLineColor = color;
        update();
    }
}

void hgHMI_PointCloud2DView::updateBoundingRect()
{
    if (points.isEmpty()) {
        minX = -1; maxX = 1;
        minZ = -1; maxZ = 1;
        centerScene = QPointF(0, 0);
        return;
    }
    minX = maxX = points[0].x();
    minZ = maxZ = points[0].y();
    for (const auto& p : points) {
        minX = std::min(minX, p.x());
        maxX = std::max(maxX, p.x());
        minZ = std::min(minZ, p.y());
        maxZ = std::max(maxZ, p.y());
    }
    // 增加10%边距，避免点绘制到边框边缘
    double dx = maxX - minX;
    double dz = maxZ - minZ;
    minX -= dx * marginPointCloud;
    maxX += dx * marginPointCloud;
    minZ -= dz * marginPointCloud;
    maxZ += dz * marginPointCloud;

    // 缩放中心设为包围盒中心
    centerScene = QPointF((minX + maxX) / 2, (minZ + maxZ) / 2);
}

QPointF hgHMI_PointCloud2DView::mapToWidget(double x, double z) const
{
    double w = width(), h = height();

    // 计算当前视图范围（已缩放）
    double dx = (maxX - minX) / scale;
    double dz = (maxZ - minZ) / scale;

    // 视图左下角（场景坐标）
    double sceneX0 = centerScene.x() - dx / 2;
    double sceneZ0 = centerScene.y() - dz / 2;

    // 计算点相对视图左下角的比例位置
    double xRatio = (x - sceneX0) / dx;
    double zRatio = (dz - (z - sceneZ0)) / dz; // Z轴图像向上，Qt坐标系Y向下需反转

    // 转换为窗口坐标
    double xWidget = marginLeft + xRatio * (w - marginLeft - marginRight);
    double zWidget = marginTop + zRatio * (h - marginTop - marginBottom);

    return QPointF(xWidget, zWidget);
}

QPointF hgHMI_PointCloud2DView::mapToScene(const QPoint& widgetPt) const
{
    double w = width(), h = height();

    double dx = (maxX - minX) / scale;
    double dz = (maxZ - minZ) / scale;

    double sceneX0 = centerScene.x() - dx / 2;
    double sceneZ0 = centerScene.y() - dz / 2;

    // 反算场景坐标
    double x = (widgetPt.x() - marginLeft) / (w - marginLeft - marginRight) * dx + sceneX0;
    double z = dz - (widgetPt.y() - marginTop) / (h - marginTop - marginBottom) * dz + sceneZ0;
    return QPointF(x, z);
}

void hgHMI_PointCloud2DView::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.fillRect(rect(), backgroundColor);

    drawGrid(painter);    // 先绘制网格线
    drawAxes(painter);    // 再绘制坐标轴和刻度
    drawPoints(painter);  // 最后绘制点云和选中点辅助线
}

void hgHMI_PointCloud2DView::drawGrid(QPainter& painter)
{
    double w = width(), h = height();

    double dx = (maxX - minX) / scale;
    double dz = (maxZ - minZ) / scale;
    double sceneX0 = centerScene.x() - dx / 2;
    double sceneZ0 = centerScene.y() - dz / 2;

    QPen gridPen(gridColor);
    gridPen.setStyle(Qt::DotLine);
    gridPen.setWidth(1);
    painter.setPen(gridPen);

    // 画X方向网格线（垂直于X轴，平行Z轴）
    for (int i = 0; i <= numMainTicks; ++i) {
        double xVal = sceneX0 + i * dx / numMainTicks;
        for (int j = 0; j <= numSubTicks; ++j) {
            double subXVal = xVal + j * (dx / numMainTicks) / numSubTicks;
            if (subXVal > sceneX0 + dx) break;

            QPointF p1 = mapToWidget(subXVal, sceneZ0);
            QPointF p2 = mapToWidget(subXVal, sceneZ0 + dz);
            painter.drawLine(p1, p2);
        }
    }

    // 画Z方向网格线（垂直于Z轴，平行X轴）
    for (int i = 0; i <= numMainTicks; ++i) {
        double zVal = sceneZ0 + i * dz / numMainTicks;
        for (int j = 0; j <= numSubTicks; ++j) {
            double subZVal = zVal + j * (dz / numMainTicks) / numSubTicks;
            if (subZVal > sceneZ0 + dz) break;

            QPointF p1 = mapToWidget(sceneX0, subZVal);
            QPointF p2 = mapToWidget(sceneX0 + dx, subZVal);
            painter.drawLine(p1, p2);
        }
    }
}

void hgHMI_PointCloud2DView::drawAxes(QPainter& painter)
{
    double w = width(), h = height();

    QPointF origin(marginLeft, h - marginBottom);
    QPointF xEnd(w - marginRight, h - marginBottom);
    QPointF zEnd(marginLeft, marginTop);

    QPen axisPen(axisColor, 2);
    painter.setPen(axisPen);
    painter.drawLine(origin, xEnd); // X轴
    painter.drawLine(origin, zEnd); // Z轴

    painter.setPen(tickLabelColor);
    painter.setFont(QFont("Arial", tickFontSize));
    int numTicks = 5;
    double dx = (maxX - minX) / scale;
    double dz = (maxZ - minZ) / scale;
    double sceneX0 = centerScene.x() - dx / 2;
    double sceneZ0 = centerScene.y() - dz / 2;

    // 主刻度长度与副刻度长度
    int mainTickLen = 7;
    int subTickLen = 4;
    int subTicks = 4;

    QPen tickPen(tickLabelColor);
    painter.setPen(tickPen);

    // X轴主副刻度和标签
    for (int i = 0; i <= numTicks; ++i) {
        double xVal = sceneX0 + i * dx / numTicks;
        QPointF pt = mapToWidget(xVal, sceneZ0);

        // 主刻度线
        painter.drawLine(QPointF(pt.x(), origin.y()), QPointF(pt.x(), origin.y() + mainTickLen));
		QString label = QString::number(xVal, 'f', tickLabelDecimals); // 轴标签保留1位小数
        painter.drawText(pt.x() - 20, origin.y() + mainTickLen + 2, 40, 15, Qt::AlignCenter, label);

        if (i == numTicks) break;

        // 副刻度线
        for (int s = 1; s < subTicks; ++s) {
            double subXVal = xVal + s * (dx / numTicks) / subTicks;
            QPointF subPt = mapToWidget(subXVal, sceneZ0);
            painter.drawLine(QPointF(subPt.x(), origin.y()), QPointF(subPt.x(), origin.y() + subTickLen));
        }
    }

    // Z轴主副刻度和标签
    for (int i = 0; i <= numTicks; ++i) {
        double zVal = sceneZ0 + i * dz / numTicks;
        QPointF pt = mapToWidget(sceneX0, zVal);

        // 主刻度线
        painter.drawLine(QPointF(origin.x() - mainTickLen, pt.y()), QPointF(origin.x(), pt.y()));
		QString label = QString::number(zVal, 'f', tickLabelDecimals); // 轴标签保留1位小数
        painter.drawText(0, pt.y() - 7, marginLeft - mainTickLen - 5, 15, Qt::AlignRight | Qt::AlignVCenter, label);

        if (i == numTicks) break;

        // 副刻度线
        for (int s = 1; s < subTicks; ++s) {
            double subZVal = zVal + s * (dz / numTicks) / subTicks;
            QPointF subPt = mapToWidget(sceneX0, subZVal);
            painter.drawLine(QPointF(origin.x() - subTickLen, subPt.y()), QPointF(origin.x(), subPt.y()));
        }
    }

    // 坐标轴标签
    //painter.setFont(QFont("Arial", 10, QFont::Bold));
    //painter.drawText(marginLeft, h - marginBottom + 22, w - marginLeft - marginRight, 20, Qt::AlignCenter, "X");

    //painter.save();
    //painter.translate(10, marginTop + (h - marginTop - marginBottom) / 2);
    //painter.rotate(-90);
    //painter.drawText(QRectF(0, 0, h - marginTop - marginBottom, 20), Qt::AlignCenter, "Z");
    //painter.restore();
}

void hgHMI_PointCloud2DView::drawPoints(QPainter& painter)
{
    double w = width(), h = height();

    const int pointSize = normalPointRadius;
	const int highlightSize = highlightPointRadius;

    // 先绘制普通点
    painter.setPen(Qt::NoPen);
    painter.setBrush(normalPointColor);
    for (int i = 0; i < points.size(); ++i) {
        QPointF pixel = mapToWidget(points[i].x(), points[i].y());
        if (pixel.x() < marginLeft || pixel.x() > w - marginRight ||
            pixel.y() < marginTop || pixel.y() > h - marginBottom)
            continue;
        if (i == selectedPointIndex) continue; // 选中点后面单独绘制
        painter.drawEllipse(pixel, pointSize, pointSize);
    }

    // 绘制选中点和辅助线
    if (selectedPointIndex >= 0 && selectedPointIndex < points.size()) {
        QPointF selPixel = mapToWidget(points[selectedPointIndex].x(), points[selectedPointIndex].y());

        // 红色高亮选中点
        painter.setBrush(highlightPointColor);
        painter.drawEllipse(selPixel, highlightSize, highlightSize);

        // 画两条红色虚线辅助线，垂直投影到X轴和Z轴
        QPen dashPen(auxiliaryLineColor);
        dashPen.setStyle(Qt::DashLine);
        dashPen.setWidth(1);
        painter.setPen(dashPen);

        double originX = marginLeft;
        double originY = height() - marginBottom;

        // 垂直线到X轴（横线）
        painter.drawLine(selPixel, QPointF(selPixel.x(), originY));

        // 垂直线到Z轴（竖线）
        painter.drawLine(selPixel, QPointF(originX, selPixel.y()));

        // 绘制轴上对应的数值标签
        painter.setPen(tickLabelColor);
        painter.setFont(QFont("Arial", tickFontSize, QFont::Bold));

        QString xLabel = QString::number(points[selectedPointIndex].x(), 'f', tickLabelDecimals + 1);
        QString zLabel = QString::number(points[selectedPointIndex].y(), 'f', tickLabelDecimals + 1);

        // X轴数值，绘制在辅助线下方居中
        painter.drawText(QRectF(selPixel.x() - 30, originY + 2, 60, 15), Qt::AlignCenter, xLabel);

        // Z轴数值，绘制在辅助线左侧垂直居中
        painter.drawText(QRectF(0, selPixel.y() - 7, marginLeft - 5, 15), Qt::AlignRight | Qt::AlignVCenter, zLabel);
    }

    // 自定义点
    for (const auto& p : customPoints) {
        QPointF pixel = mapToWidget(p.pos.x(), p.pos.y());
        if (pixel.x() < marginLeft || pixel.x() > w - marginRight ||
            pixel.y() < marginTop || pixel.y() > h - marginBottom)
            continue;
        painter.setBrush(p.color);
        painter.drawEllipse(pixel, p.radius, p.radius);
    }
}

void hgHMI_PointCloud2DView::wheelEvent(QWheelEvent* event)
{
    QPointF mouseSceneBefore = mapToScene(event->pos());

    if (event->angleDelta().y() > 0)
        scale *= 1.15;
    else
        scale /= 1.15;

    // 限制缩放范围
    if (scale < 0.001) scale = 0.001;
    else if (scale > 1000.0) scale = 1000.0;

    QPointF mouseSceneAfter = mapToScene(event->pos());

    // 调整视图中心，使缩放以鼠标位置为中心
    QPointF delta = mouseSceneBefore - mouseSceneAfter;
    centerScene += delta;

    update();
}

void hgHMI_PointCloud2DView::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton) {
        // 右键拖动视图
        dragging = true;
        lastMousePos = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
    else if (event->button() == Qt::LeftButton) {
        // 左键选中点
        QPointF clickedScenePt = mapToScene(event->pos());

        double minDist = std::numeric_limits<double>::max();
        int index = -1;
        const double selectThreshold = 10; // 窗口坐标选择阈值，单位像素
        for (int i = 0; i < points.size(); ++i) {
            QPointF ptWidget = mapToWidget(points[i].x(), points[i].y());
            double dist = std::hypot(ptWidget.x() - event->pos().x(), ptWidget.y() - event->pos().y());
            if (dist < minDist && dist < selectThreshold) {
                minDist = dist;
                index = i;
            }
        }
        selectedPointIndex = index;
        update();
    }
}

void hgHMI_PointCloud2DView::mouseMoveEvent(QMouseEvent* event)
{
    if (dragging && (event->buttons() & Qt::RightButton)) {
        QPoint delta = event->pos() - lastMousePos;
        lastMousePos = event->pos();

        double w = width(), h = height();

        double dx = (maxX - minX) / scale;
        double dz = (maxZ - minZ) / scale;

        double sceneDxPerPixel = dx / (w - marginLeft - marginRight);
        double sceneDzPerPixel = dz / (h - marginTop - marginBottom);

        centerScene.rx() -= delta.x() * sceneDxPerPixel;
        centerScene.ry() += delta.y() * sceneDzPerPixel;

        update();
    }
}

void hgHMI_PointCloud2DView::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton) {
        dragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

 
//////////////////////////////////////////////////////////////
/// hgHMI_labelLED
/// 圆形指示灯控件，支持颜色设置、闪烁、边框、阴影等效果
///
/// 特点：
/// - 圆形 LED 指示灯效果
/// - 支持亮起和熄灭两种颜色
/// - 支持闪烁动画效果
/// - 支持边框自动跟随LED颜色
/// - 支持阴影和透明度
///
/// 使用方法：
/// 1. 直接创建或通过提升方式使用
/// 2. setLedColor() 设置颜色
/// 3. setLedOn() 控制亮灭
/// 4. setBlinking() 开启闪烁
/// 5. setAutoBorderColor(true) 启用边框自动跟随颜色
///
/// 示例：
// hgHMI_labelLED* led = new hgHMI_labelLED(this);
// led->setOnColor(Qt::green);
// led->setOffColor(Qt::red);
// led->setLedSize(20);
// led->setBorderEnabled(true);
// led->setAutoBorderColor(true);  // 边框自动匹配LED颜色
// led->setLedOn(true);  // 绿色灯+绿色边框
// led->setLedOn(false); // 红色灯+红色边框
///
//////////////////////////////////////////////////////////////
hgHMI_labelLED::hgHMI_labelLED(QWidget* parent)
    : QLabel(parent)
    , m_onColor(Qt::green)
    , m_offColor(QColor(120, 120, 120))
    , m_size(16)
    , m_isOn(true)
    , m_ledState(true)
    , m_borderEnabled(false)
    , m_borderWidth(1)
    , m_borderColor(QColor(80, 80, 80))
    , m_autoBorderColor(false)  // 默认不自动匹配
    , m_shadowEnabled(false)
    , m_blinking(false)
    , m_blinkInterval(500)
    , m_blinkTimer(nullptr)
{
    setFixedSize(m_size, m_size);
    setAlignment(Qt::AlignCenter);
    setText(QStringLiteral(""));

    m_color = m_onColor;
    updateStyleSheet();
}

hgHMI_labelLED::hgHMI_labelLED(const QColor& color, int size, QWidget* parent)
    : hgHMI_labelLED(parent)
{
    m_onColor = color;
    m_offColor = color.darker(150);
    m_size = size;
    setFixedSize(m_size, m_size);
    m_color = m_onColor;
    updateStyleSheet();
}

hgHMI_labelLED::hgHMI_labelLED(const QString& colorName, int size, QWidget* parent)
    : hgHMI_labelLED(QColor(colorName), size, parent)
{
}

void hgHMI_labelLED::setLedColor(const QColor& color)
{
    m_onColor = color;
    if (m_isOn && !m_blinking) {
        m_color = m_onColor;
        updateStyleSheet();
        if (m_autoBorderColor) updateBorderColor();
    }
}

void hgHMI_labelLED::setLedColor(const QString& colorName)
{
    setLedColor(QColor(colorName));
}

void hgHMI_labelLED::setLedSize(int size)
{
    if (size <= 0) return;
    m_size = size;
    setFixedSize(m_size, m_size);
    updateStyleSheet();
}

void hgHMI_labelLED::setLedOn(bool on)
{
    if (m_isOn == on) return;
    m_isOn = on;
    m_ledState = on;

    if (!m_blinking) {
        m_color = m_isOn ? m_onColor : m_offColor;
        updateStyleSheet();
        if (m_autoBorderColor) updateBorderColor();
    }
}

void hgHMI_labelLED::toggle()
{
    setLedOn(!m_isOn);
}

void hgHMI_labelLED::setOnColor(const QColor& color)
{
    m_onColor = color;
    if (m_isOn && !m_blinking) {
        m_color = m_onColor;
        updateStyleSheet();
        if (m_autoBorderColor) updateBorderColor();
    }
}

void hgHMI_labelLED::setOffColor(const QColor& color)
{
    m_offColor = color;
    if (!m_isOn && !m_blinking) {
        m_color = m_offColor;
        updateStyleSheet();
        if (m_autoBorderColor) updateBorderColor();
    }
}

void hgHMI_labelLED::setBorderEnabled(bool enabled)
{
    m_borderEnabled = enabled;
    updateStyleSheet();
}

void hgHMI_labelLED::setBorderWidth(int width)
{
    if (width < 0) return;
    m_borderWidth = width;
    updateStyleSheet();
}

void hgHMI_labelLED::setBorderColor(const QColor& color)
{
    m_borderColor = color;
    m_autoBorderColor = false;  // 手动设置边框颜色时，关闭自动匹配
    updateStyleSheet();
}

void hgHMI_labelLED::setAutoBorderColor(bool autoColor)
{
    m_autoBorderColor = autoColor;
    if (m_autoBorderColor) {
        updateBorderColor();
    }
}

void hgHMI_labelLED::setShadowEnabled(bool enabled)
{
    m_shadowEnabled = enabled;
    update();
}

void hgHMI_labelLED::setBlinking(bool blinking, int intervalMs)
{
    if (m_blinking == blinking && m_blinkInterval == intervalMs) return;

    m_blinking = blinking;
    m_blinkInterval = intervalMs;

    if (m_blinking) {
        startBlinkTimer();
    }
    else {
        stopBlinkTimer();
        m_ledState = m_isOn;
        m_color = m_isOn ? m_onColor : m_offColor;
        update();
        if (m_autoBorderColor) updateBorderColor();
    }
}

void hgHMI_labelLED::setOpacity(qreal opacity)
{
    QGraphicsOpacityEffect* effect = new QGraphicsOpacityEffect(this);
    effect->setOpacity(opacity);
    setGraphicsEffect(effect);
}

void hgHMI_labelLED::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event)
        QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QRectF rect(0, 0, width(), height());
    QRectF innerRect = rect.adjusted(m_borderWidth, m_borderWidth, -m_borderWidth, -m_borderWidth);

    // 确保内圈有效
    if (innerRect.width() <= 0 || innerRect.height() <= 0) {
        innerRect = rect;
    }

    // 绘制阴影
    if (m_shadowEnabled && !m_blinking) {
        painter.save();
        QRectF shadowRect = rect.adjusted(1, 1, 1, 1);
        painter.setBrush(QColor(0, 0, 0, 50));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(shadowRect);
        painter.restore();
    }

    // 绘制边框（如果启用自动边框，边框颜色已经通过 updateBorderColor 更新）
    if (m_borderEnabled) {
        QPen pen(m_borderColor);
        pen.setWidth(m_borderWidth);
        painter.setPen(pen);
    }
    else {
        painter.setPen(Qt::NoPen);
    }

    // 绘制指示灯主体
    painter.setBrush(m_color);
    painter.drawEllipse(innerRect);

    // 添加高光效果（只在 LED 实际点亮时）
    if (m_ledState) {
        painter.save();
        // 高光区域：圆心偏左上方的亮斑
        QPointF highlightCenter = innerRect.center() - QPointF(innerRect.width() / 6, innerRect.height() / 6);
        QRadialGradient gradient(highlightCenter, innerRect.width() / 3);
        gradient.setColorAt(0, QColor(255, 255, 255, 120));
        gradient.setColorAt(0.5, QColor(255, 255, 255, 40));
        gradient.setColorAt(1, QColor(255, 255, 255, 0));
        painter.setBrush(gradient);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(innerRect);
        painter.restore();
    }
}

void hgHMI_labelLED::resizeEvent(QResizeEvent* event)
{
    QLabel::resizeEvent(event);
    updateStyleSheet();
}

void hgHMI_labelLED::onBlinkTimeout()
{
    if (!m_blinking) return;

    m_ledState = !m_ledState;
    m_color = m_ledState ? m_onColor : m_offColor;
    update();

    // 闪烁时自动更新边框颜色
    if (m_autoBorderColor) {
        updateBorderColor();
    }
}

void hgHMI_labelLED::updateStyleSheet()
{
    generateStyleSheet();
    setStyleSheet(m_cachedStyleSheet);
}

void hgHMI_labelLED::generateStyleSheet()
{
    int radius = m_size / 2;

    QString borderStyle;
    if (m_borderEnabled) {
        borderStyle = QStringLiteral("border: %1px solid %2;")
            .arg(m_borderWidth)
            .arg(m_borderColor.name());
    }
    else {
        borderStyle = QStringLiteral("border: none;");
    }

    m_cachedStyleSheet = QStringLiteral(
        "background-color: %1;"
        "border-radius: %2px;"
        "%3"
    ).arg(m_color.name())
        .arg(radius)
        .arg(borderStyle);
}

void hgHMI_labelLED::startBlinkTimer()
{
    if (m_blinkTimer) {
        m_blinkTimer->stop();
        delete m_blinkTimer;
        m_blinkTimer = nullptr;
    }

    m_blinkTimer = new QTimer(this);
    connect(m_blinkTimer, &QTimer::timeout, this, &hgHMI_labelLED::onBlinkTimeout);
    m_blinkTimer->start(m_blinkInterval);
}

void hgHMI_labelLED::stopBlinkTimer()
{
    if (m_blinkTimer) {
        m_blinkTimer->stop();
        delete m_blinkTimer;
        m_blinkTimer = nullptr;
    }
}

void hgHMI_labelLED::updateBorderColor()
{
    if (!m_autoBorderColor) return;

    // 根据当前 LED 颜色设置边框颜色
    // 边框颜色比 LED 颜色稍暗，增加立体感
    QColor borderColor = m_color.darker(120);
    m_borderColor = borderColor;

    // 重新生成样式表
    if (m_borderEnabled) {
        updateStyleSheet();
    }
}