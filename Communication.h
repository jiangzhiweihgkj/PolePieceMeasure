#ifndef COMMUNICATION_H
#define COMMUNICATION_H
// Qt 5.14 的 qvector.h 中 QVector::operator== 使用了 MSVC 已弃用的
// stdext::make_checked_array_iterator，新版 MSVC 会报 STL4043(C4996)。
// 必须在包含任何 Qt/STL 头之前定义这个安静宏；
// 更彻底的做法是在 项目属性 -> C/C++ -> 预处理器 里加同名宏（对所有文件生效）。
#ifndef _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING
#  define _SILENCE_STDEXT_ARR_ITERS_DEPRECATION_WARNING
#endif
#include <QThread>
#include <QTcpSocket>
#include <QVector>
#include <QString>
#include <QDateTime>
#include <QMutex>
#include <atomic>

//QT_BEGIN_NAMESPACE
//namespace Ui { class CommunicationClass; };
//QT_END_NAMESPACE

class Communication : public QThread
{
	Q_OBJECT
Q_SIGNALS:
	// 跨线程（队列连接）时参数不能用非 const 引用，否则 Qt 会丢弃该信号
	void sendCommunicateMessage(const QString& msg);
	// 每完成一轮 "PLC -> CCD 信号读取" 后发一次（不带参数，避免元类型注册）
	void plcSignalsUpdated();
public:
	// FINS 存储区代码（按字访问）
	enum FinsArea
	{
		AreaDM = 0x82,   // D/DM 数据区
		AreaCIO = 0xB0,  // CIO 区
		AreaWR = 0xB1,   // W 链接区
		AreaHR = 0xB2,   // H 保持区
		AreaAR = 0xB3    // A 辅助继电器区
	};

	// ============ 3D 通讯信号表（华工-蚀刻机视觉通讯-20260416 / "3D通讯"页） ============
	// PLC -> CCD（本软件读取）
	//   D2300 REAL 相机1触发：0默认 / 10触发 / 11点检 / 20仅采图不判定
	//   D2302 REAL 相机2触发：同上
	//   D2304 REAL 相机1完成：0默认 / 10完成(OK)
	//   D2306 REAL 相机2完成：0默认 / 10完成(OK)
	//   D2330 REAL 缓存位1(3D1与3D2之间缓存)实时编码值
	//   D2332 REAL 缓存位2(2D线扫缓存)实时编码值
	//   D10000 String 产品型号
	struct PlcSignals
	{
		float cam1Trigger = 0.0f;    // D2300
		float cam2Trigger = 0.0f;    // D2302
		float cam1Done = 0.0f;       // D2304
		float cam2Done = 0.0f;       // D2306
		float cacheEncoder1 = 0.0f;  // D2330
		float cacheEncoder2 = 0.0f;  // D2332
		QString productModel;        // D10000
		QDateTime stamp;             // 采样时间
	};

	// CCD -> PLC（本软件写入）
	//   D2312 REAL 心跳：默认0，每1秒给一次1
	//   D2314 REAL 相机1启动：0默认 / OK=10 / NG=11
	//   D2316 REAL 相机1结果：0默认 / OK=10 / 槽深槽宽均NG=11 / 单独槽深NG=12 / 单独槽宽NG=13
	//   D2318 REAL 相机2启动：0默认 / OK=10 / NG=11
	//   D2320 REAL 相机2结果：同 D2316
	struct PlcCommands
	{
		float heartbeat = 0.0f;      // D2312
		float cam1Start = 0.0f;      // D2314
		float cam1Result = 0.0f;     // D2316
		float cam2Start = 0.0f;      // D2318
		float cam2Result = 0.0f;     // D2320
	};

	Communication(void);
	~Communication(void);
	void getConnectInfo(const QString& ip, quint16 port);
	void getWriteCommunicateData(quint16 startAddress, quint16 count, QVector<quint16>& values);
	// 连接欧姆龙PLC（FINS/TCP，本软件作为客户端）
	// ip：PLC的IP地址；port：FINS/TCP端口，默认9600
	// serverNode：PLC节点号；clientNode：本机节点号（0 表示由PLC自动分配）
	bool connectPLC(const QString& ip, quint16 port = 9600, int timeoutMs = 3000,
		quint8 serverNode = 0x01, quint8 clientNode = 0x0A);
	// 内部实现：emitError=false 时失败不发送信号（供 run() 自动重连使用，避免刷屏）
	bool connectPLCInternal(const QString& ip, quint16 port, int timeoutMs,
		quint8 serverNode, quint8 clientNode, bool emitError);
	void disconnectPLC();
	bool isConnected() const;

	// 实时读取单个地址的值（一个寄存器字）
	bool readValue(FinsArea area, quint16 address, quint16& value, int timeoutMs = 3000);
	// 实时读取多个连续地址的值（从 startAddress 开始的 count 个字，最多 480 个）
	bool readValues(FinsArea area, quint16 startAddress, quint16 count,
		QVector<quint16>& values, int timeoutMs = 3000);
	// 写入单个寄存器
	bool writeValue(FinsArea area, quint16 address, quint16 value, int timeoutMs = 3000);
	// 写入多个连续寄存器（最多 480 个）
	bool writeValues(FinsArea area, quint16 startAddress,
		const QVector<quint16>& values, int timeoutMs = 3000);

	// 写入单个实数（REAL，占连续2个字，低字在前）
	bool writeReal(FinsArea area, quint16 address, float value, int timeoutMs = 3000);
	// 读取单个实数（REAL，占连续2个字，低字在前）
	bool readReal(FinsArea area, quint16 address, float& value, int timeoutMs = 3000);
	// 批量读/写实数（count 个 REAL = count*2 个字，一次成帧）
	bool readReals(FinsArea area, quint16 startAddress, int count,
		QVector<float>& values, int timeoutMs = 3000);
	bool writeReals(FinsArea area, quint16 startAddress,
		const QVector<float>& values, int timeoutMs = 3000);
	// 读取字符串（产品型号）：wordCount 个字，每字 2 个 ASCII 字符（高字节在前），遇 0x00 结束
	bool readString(FinsArea area, quint16 startAddress, int wordCount,
		QString& text, int timeoutMs = 3000);

	// 最近一次通信失败的原因
	QString lastError() const;
public://读
	quint16 r_startAddress = 0; // 起始地址
	quint16 r_count = 0; // 读取或写入的字节数
	QVector<quint16> r_values;// 读取或写入的值
public://写
	quint16 w_startAddress = 0; // 写入的地址startAddress;  
	quint16 w_count = 0; // 写入或写入的字节数count;  
	QVector<quint16> w_values;// 写入的值

public:
	// ============ 实时轮询（逻辑全部在 run() 的 while 循环里） ============
	// 启动轮询线程：自动连接、周期读 PLC->CCD、变化时写 CCD->PLC、断线自动重连
	// intervalMs：轮询周期，默认 100ms（读写各一次，帧内批量）
	void startPlcPolling(const QString& ip, quint16 port, int intervalMs = 100);
	// 停止轮询线程（阻塞等待线程退出）
	void stopPlcPolling();
	bool isPolling() const { return m_pollingActive.load(); }

	// 整体设置要写给 PLC 的信号（覆盖全部字段，心跳会在 run() 里被重新计算）
	void setPublishCommands(const PlcCommands& cmd);
	// 只改相机1/相机2的启动+结果，不影响另一台相机与其它信号（推荐用这个）
	void setCamera1Command(float startValue, float resultValue);
	void setCamera2Command(float startValue, float resultValue);
	// 最近一次从 PLC 读到的信号快照（线程安全）
	PlcSignals lastSignals() const;

	// 轮询周期（毫秒），startPlcPolling 会覆盖
	int m_intervalMs = 100;
	// 产品型号字符串占用的字数（D10000 起，1 字 = 2 字符）
	int m_modelStringWords = 10;
	// "谁写谁清"：PLC 完成/触发归零后自动清零本机写入的信号
	bool m_autoClearCommands = true;

protected:
	void run() override;
public:
	// 组装存储区读取指令帧（FINS 命令 0101）
	QByteArray buildReadFrame(FinsArea area, quint16 startAddress, quint16 count);
	// 组装存储区写入指令帧（FINS 命令 0102）
	QByteArray buildWriteFrame(FinsArea area, quint16 startAddress, const QVector<quint16>& values);
	// 发送 FINS 指令帧并接收应答（finsResponse 为去掉 FINS/TCP 头后的应答帧）
	bool sendFinsFrame(const QByteArray& finsFrame, QByteArray& finsResponse, int timeoutMs);
	// 从 socket 精确读取指定字节数
	bool readExact(qint64 bytesNeeded, QByteArray& out, int timeoutMs);

public:
	// 保证 socket 在"当前线程"里（必要时从其它线程迁移过来）
	void ensureSocket();

public:
	//Ui::CommunicationClass *ui;

	QTcpSocket* m_sock = nullptr;  // FINS/TCP 客户端 socket（不设 parent，便于 moveToThread）
	quint8 m_serverNode = 0x01;    // PLC节点号（DA1）
	quint8 m_clientNode = 0x0A;    // 本机节点号（SA1）
	quint8 m_sid = 0x00;           // FINS 序列号，每帧自增
	QString m_lastError;           // 最近一次错误信息
public:
	QString m_ip = QStringLiteral("127.0.0.1");
	quint16 m_port = 9600;
	int m_timeoutMs = 3000;

private:
	mutable QMutex m_dataMutex;    // 保护 m_signals / m_commands
	PlcSignals m_signals;          // 最近一次读到的信号
	PlcCommands m_commands;        // 待写出的信号
	std::atomic<bool> m_pollingActive{ false };

};

#endif // COMMUNICATION_H
