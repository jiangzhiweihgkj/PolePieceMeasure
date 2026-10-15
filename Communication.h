#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <QThread>
#include <QTcpSocket>
#include <QVector>

//QT_BEGIN_NAMESPACE
//namespace Ui { class CommunicationClass; };
//QT_END_NAMESPACE

class Communication : public QThread
{
	Q_OBJECT
Q_SIGNALS:
	void sendCommunicateMessage(QString& msg);
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
	// 实时读取多个连续地址的值（从 startAddress 开始的 count 个字，最多 999 个）
	bool readValues(FinsArea area, quint16 startAddress, quint16 count,
		QVector<quint16>& values, int timeoutMs = 3000);
	// 写入单个寄存器
	bool writeValue(FinsArea area, quint16 address, quint16 value, int timeoutMs = 3000);
	// 写入多个连续寄存器（最多 999 个）
	bool writeValues(FinsArea area, quint16 startAddress,
		const QVector<quint16>& values, int timeoutMs = 3000);

	// 写入单个实数（REAL，占连续2个字，低字在前）
	bool writeReal(FinsArea area, quint16 address, float value, int timeoutMs = 3000);
	// 读取单个实数（REAL，占连续2个字，低字在前）
	bool readReal(FinsArea area, quint16 address, float& value, int timeoutMs = 3000);

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
/*D2312(REAL)	心跳信号	默认值0，隔1S给一次1；	CCD-PLC
D2314(REAL)	相机1启动信号	默认值0，相机1：OK为10，NG为11	CCD-PLC
D2316(REAL)	相机1结果信号	默认值0，相机1：OK为10，槽深槽宽均NG为11，单独槽深NG为12，单独槽宽NG为13	CCD-PLC
D2318(REAL)	相机2启动信号	默认值0，相机2：OK为10，NG为11	CCD-PLC
D2320(REAL)	相机2结果信号	默认值0，相机2：OK为10，槽深槽宽均NG为11，单独槽深NG为12，单独槽宽NG为13	CCD-PLC
*/

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
	//Ui::CommunicationClass *ui;

	QTcpSocket* m_sock = nullptr;  // FINS/TCP 客户端 socket
	quint8 m_serverNode = 0x01;    // PLC节点号（DA1）
	quint8 m_clientNode = 0x0A;    // 本机节点号（SA1）
	quint8 m_sid = 0x00;           // FINS 序列号，每帧自增
	QString m_lastError;           // 最近一次错误信息
public:
	QString m_ip=QStringLiteral("127.0.0.1");
	quint16 m_port = 9600; 
	int m_timeoutMs = 3000;
	 
};

#endif // COMMUNICATION_H
