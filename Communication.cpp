#include "Communication.h"
#include <cstring>
#include <QElapsedTimer>
//#include "ui_Communication.h"

namespace
{
	// FINS/TCP 命令码
	//const quint32 FinsTcpCommand_Connect = 0x00000000;    // 连接请求
	//const quint32 FinsTcpCommand_FinsFrame = 0x00000002;  // FINS 帧发送
	const quint32 FinsTcpCommand_Connect = 0x00000000;    // 连接请求
	const quint32 FinsTcpCommand_ConnectR = 0x00000001;    // 连接应答
	const quint32 FinsTcpCommand_FinsFrame = 0x00000002;    // FINS 帧发送

	// 大端追加 16 位
	void appendU16(QByteArray& buf, quint16 value)
	{
		buf.append(char((value >> 8) & 0xFF));
		buf.append(char(value & 0xFF));
	}

	// 大端追加 32 位
	void appendU32(QByteArray& buf, quint32 value)
	{
		buf.append(char((value >> 24) & 0xFF));
		buf.append(char((value >> 16) & 0xFF));
		buf.append(char((value >> 8) & 0xFF));
		buf.append(char(value & 0xFF));
	}

	// 大端读取 32 位（调用前需保证 buf 至少有 4 字节，调用处用 mid 截取）
	quint32 readU32(const QByteArray& buf)
	{
		return (quint32(quint8(buf.at(0))) << 24)
			| (quint32(quint8(buf.at(1))) << 16)
			| (quint32(quint8(buf.at(2))) << 8)
			| quint32(quint8(buf.at(3)));
	}
}

Communication::Communication(void)
{  
	m_sock = new QTcpSocket(this);
}

Communication::~Communication(void)
{
	requestInterruption();
	if (!wait(8000))
	{
		if (m_sock != nullptr)
			m_sock->abort();
		wait();
	}
	disconnectPLC(); 
}
void Communication::run()
{
	while (!isInterruptionRequested())
	{
		/*if (m_sock == nullptr || m_sock->state() != QAbstractSocket::ConnectedState)
		{
			if (m_sock != nullptr && m_sock->state() != QAbstractSocket::UnconnectedState)
				disconnectPLC();

			if (m_ip.isEmpty() ||
				!connectPLCInternal(m_ip, m_port, m_timeoutMs, m_serverNode, m_clientNode, false))
			{
				QThread::msleep(1000);
				continue;
			}
		}*/

		// 等待PLC数据，用于及时发现断线；超时只表示暂时无数据，不当作错误
		if (!m_sock->waitForReadyRead(1000) && m_sock->state() != QAbstractSocket::ConnectedState)
		{
			m_lastError = QStringLiteral("与PLC连接已断开");
			disconnectPLC();
			continue;
		}

		bool write_heart = writeValue(FinsArea::AreaDM, 2312, 0x0001, m_timeoutMs);
		if (!write_heart)
		{
			m_lastError = QStringLiteral("写入PLC心跳信号失败");
			disconnectPLC();
			continue;
		}
		// 写入从 w_startAddress 开始的多个连续寄存器
		if (!w_values.isEmpty())
		{
			bool res_Write = writeValues(FinsArea::AreaDM, w_startAddress, w_values, m_timeoutMs);
			if (!res_Write)
			{
				m_lastError = QStringLiteral("写入PLC失败");
				disconnectPLC();
				continue;
			}
		}
		// 读取从 r_startAddress 开始的 count 个连续地址的值
		if (r_count > 0)
		{
			bool res_Read = readValues(FinsArea::AreaDM, r_startAddress,
				r_count, r_values, m_timeoutMs);
			if (!res_Read)
			{
				m_lastError = QStringLiteral("读取PLC失败");
				disconnectPLC();
				continue;
			}
		}

		 /*   D2312(REAL)	心跳信号	默认值0，隔1S给一次1；	CCD - PLC
			D2314(REAL)	相机1启动信号	默认值0，相机1：OK为10，NG为11	CCD - PLC
			D2316(REAL)	相机1结果信号	默认值0，相机1：OK为10，槽深槽宽均NG为11，单独槽深NG为12，单独槽宽NG为13	CCD - PLC
			D2318(REAL)	相机2启动信号	默认值0，相机2：OK为10，NG为11	CCD - PLC
			D2320(REAL)	相机2结果信号	默认值0，相机2：OK为10，槽深槽宽均NG为11，单独槽深NG为12，单独槽宽NG为13	CCD - PLC*/

		QThread::msleep(500);

	}
}

// 连接欧姆龙PLC（FINS/TCP，本软件作为客户端）
bool Communication::connectPLC(const QString& ip, quint16 port, int timeoutMs,
	quint8 serverNode, quint8 clientNode)
{
	return connectPLCInternal(ip, port, timeoutMs, serverNode, clientNode, true);
}

bool Communication::connectPLCInternal(const QString& ip, quint16 port, int timeoutMs,
	quint8 serverNode, quint8 clientNode, bool emitError)
{
	m_lastError.clear();

	if (ip.isEmpty() || port == 0)
	{
		m_lastError = QStringLiteral("PLC的IP地址或端口无效");
		if (emitError)
			emit sendCommunicateMessage(m_lastError);
		return false;
	}

	if (m_sock == nullptr)
		m_sock = new QTcpSocket(this);

	if (m_sock->state() != QAbstractSocket::UnconnectedState)
		disconnectPLC();

	m_serverNode = serverNode;
	m_clientNode = clientNode;

	QElapsedTimer elapsed;
	elapsed.start();
	const auto remainingMs = [&elapsed, timeoutMs]() -> int {
		return qMax(1, timeoutMs - int(elapsed.elapsed()));
	};

	// 1. 建立 TCP 连接
	m_sock->connectToHost(ip, port);
	if (!m_sock->waitForConnected(remainingMs()))
	{
		m_lastError = QStringLiteral("TCP连接PLC失败：%1").arg(m_sock->errorString());
		if (emitError)
			emit sendCommunicateMessage(m_lastError); // 发送通信错误信息
		disconnectPLC();
		return false;
	}
	m_sock->setSocketOption(QAbstractSocket::LowDelayOption, 1);

	// 2. 发送 FINS/TCP 连接请求帧（共 20 字节）
	QByteArray request;
	request.reserve(20);
	request.append("FINS", 4);
	appendU32(request, 12);                      // 长度：命令(4)+错误码(4)+节点(4)
	appendU32(request, FinsTcpCommand_Connect);  // 命令：连接请求
	appendU32(request, 0);                       // 错误码
	appendU32(request, m_clientNode);            // 客户端节点号（0=由PLC自动分配）
	if (m_sock->write(request) != request.size() || !m_sock->waitForBytesWritten(remainingMs()))
	{
		m_lastError = QStringLiteral("发送FINS/TCP连接请求失败：%1").arg(m_sock->errorString());
		if (emitError)
			emit sendCommunicateMessage(m_lastError); // 发送通信错误信息
		disconnectPLC();
		return false;
	}

	// 3. 接收 20 字节连接应答
	QByteArray response;
	if (!readExact(24, response, remainingMs()))
	//if (!readExact(20, response, remainingMs()))
	{
		

		m_lastError = QStringLiteral("接收FINS/TCP连接应答失败");
		if (emitError)
			emit sendCommunicateMessage(m_lastError); // 发送通信错误信息 
		disconnectPLC();
		return false;
	}
	if (response.left(4) != QByteArrayLiteral("FINS")
		|| readU32(response.mid(4, 4)) != 16
		//|| readU32(response.mid(4, 4)) != 12
		|| readU32(response.mid(8, 4)) != FinsTcpCommand_ConnectR)
		//|| readU32(response.mid(8, 4)) != FinsTcpCommand_Connect)
	{
		qDebug() << "应答长度:" << response.size()
			<< "内容:" << response.toHex(' ');
		m_lastError = QStringLiteral("FINS/TCP连接应答格式错误");
		if (emitError)
			emit sendCommunicateMessage(m_lastError);
		disconnectPLC();
		return false;
	}

	const quint32 errorCode = readU32(response.mid(12, 4));
	if (errorCode != 0)
	{
		m_lastError = QStringLiteral("PLC拒绝连接，错误码：%1").arg(errorCode);
		if (emitError)
			emit sendCommunicateMessage(m_lastError); // 发送通信错误信息 
		disconnectPLC();
		return false;
	}

	// 保存PLC分配的客户端节点号
	const quint32 assignedNode = readU32(response.mid(20, 4));
	//const quint32 assignedNode = readU32(response.mid(16, 4));
	if (assignedNode != 0 && assignedNode <= 0xFE)
		m_clientNode = quint8(assignedNode);

	return true;
}

void Communication::disconnectPLC()
{
	if (m_sock == nullptr)
		return;

	if (m_sock->state() != QAbstractSocket::UnconnectedState)
	{
		m_sock->disconnectFromHost();
		if (m_sock->state() != QAbstractSocket::UnconnectedState)
			m_sock->waitForDisconnected(1000);
		if (m_sock->state() != QAbstractSocket::UnconnectedState)
			m_sock->abort();
	}
}

bool Communication::isConnected() const
{
	return m_sock != nullptr && m_sock->state() == QAbstractSocket::ConnectedState;
}

QString Communication::lastError() const
{
	return m_lastError;
}

// 实时读取单个地址的值（一个寄存器字）
bool Communication::readValue(FinsArea area, quint16 address, quint16& value, int timeoutMs)
{
	QVector<quint16> values;
	if (!readValues(area, address, 1, values, timeoutMs))
		return false;

	value = values.value(0);
	return true;
}

// 实时读取从 startAddress 开始的 count 个连续地址的值
bool Communication::readValues(FinsArea area, quint16 startAddress, quint16 count,
	QVector<quint16>& values, int timeoutMs)
{
	m_lastError.clear();
	values.clear();

	if (!isConnected())
	{
		m_lastError = QStringLiteral("未连接PLC");
		return false;
	}
	if (count == 0 || count > 999)
	{
		m_lastError = QStringLiteral("读取数量必须在1~999之间");
		return false;
	}

	QByteArray finsResponse;
	if (!sendFinsFrame(buildReadFrame(area, startAddress, count), finsResponse, timeoutMs))
		return false;

	// FINS 应答：10 字节头 + 命令码(2) + 结束代码(2) + 数据
	if (finsResponse.size() < 14)
	{
		m_lastError = QStringLiteral("FINS读取应答长度错误");
		return false;
	}
	if (quint8(finsResponse.at(10)) != 0x01 || quint8(finsResponse.at(11)) != 0x01)
	{
		m_lastError = QStringLiteral("FINS读取应答命令码不匹配");
		return false;
	}

	const quint16 endCode = (quint16(quint8(finsResponse.at(12))) << 8) | quint8(finsResponse.at(13));
	if (endCode != 0)
	{
		m_lastError = QStringLiteral("FINS读取失败，结束代码：0x%1").arg(endCode, 4, 16, QChar('0'));
		return false;
	}

	if (finsResponse.size() < 14 + int(count) * 2)
	{
		m_lastError = QStringLiteral("FINS读取应答数据不完整");
		return false;
	}

	values.reserve(count);
	for (int i = 0; i < count; ++i)
	{
		const int offset = 14 + i * 2;
		const quint16 word = (quint16(quint8(finsResponse.at(offset))) << 8)
			| quint8(finsResponse.at(offset + 1));
		values.append(word);
	}
	return true;
}

// 写入单个寄存器
bool Communication::writeValue(FinsArea area, quint16 address, quint16 value, int timeoutMs)
{
	QVector<quint16> values;
	values.append(value);
	return writeValues(area, address, values, timeoutMs);
}

// 写入从 startAddress 开始的多个连续寄存器
bool Communication::writeValues(FinsArea area, quint16 startAddress,
	const QVector<quint16>& values, int timeoutMs)
{ 
	m_lastError.clear();

	if (!isConnected())
	{
		m_lastError = QStringLiteral("未连接PLC");
		return false;
	}
	if (values.isEmpty() || values.size() > 999)
	{
		m_lastError = QStringLiteral("写入数量必须在1~999之间");
		return false;
	}

	QByteArray finsResponse;
	if (!sendFinsFrame(buildWriteFrame(area, startAddress, values), finsResponse, timeoutMs))
		return false;

	// FINS 应答：10 字节头 + 命令码(2) + 结束代码(2)
	if (finsResponse.size() < 14)
	{
		m_lastError = QStringLiteral("FINS写入应答长度错误");
		return false;
	}
	if (quint8(finsResponse.at(10)) != 0x01 || quint8(finsResponse.at(11)) != 0x02)
	{
		m_lastError = QStringLiteral("FINS写入应答命令码不匹配");
		return false;
	}

	const quint16 endCode = (quint16(quint8(finsResponse.at(12))) << 8) | quint8(finsResponse.at(13));
	if (endCode != 0)
	{
		m_lastError = QStringLiteral("FINS写入失败，结束代码：0x%1").arg(endCode, 4, 16, QChar('0'));
		return false;
	}
	return true;
}

// 写入单个实数（REAL，占连续2个字，低字在前）
bool Communication::writeReal(FinsArea area, quint16 address, float value, int timeoutMs)
{
	quint32 raw = 0;
	memcpy(&raw, &value, sizeof(raw));  // 取 IEEE754 位模式

	QVector<quint16> words;
	words.append(quint16(raw & 0xFFFF));          // 低字
	words.append(quint16((raw >> 16) & 0xFFFF));  // 高字
	return writeValues(area, address, words, timeoutMs);
}

// 读取单个实数（REAL，占连续2个字，低字在前）
bool Communication::readReal(FinsArea area, quint16 address, float& value, int timeoutMs)
{
	QVector<quint16> words;
	if (!readValues(area, address, 2, words, timeoutMs))
		return false;

	const quint32 raw = quint32(words.at(0)) | (quint32(words.at(1)) << 16);
	memcpy(&value, &raw, sizeof(value));
	return true;
}

// 组装存储区读取指令帧（FINS 命令 0101）
QByteArray Communication::buildReadFrame(FinsArea area, quint16 startAddress, quint16 count)
{
	QByteArray frame;
	frame.reserve(18);

	// FINS 指令头（10 字节）
	frame.append(char(0x80));          // ICF：需要应答
	frame.append(char(0x00));          // RSV：保留
	frame.append(char(0x02));          // GCT：网关计数
	frame.append(char(0x00));          // DNA：目标网络号（本机网络）
	frame.append(char(m_serverNode));  // DA1：目标节点号（PLC）
	frame.append(char(0x00));          // DA2：目标单元号（CPU单元）
	frame.append(char(0x00));          // SNA：源网络号
	frame.append(char(m_clientNode));  // SA1：源节点号（本机）
	frame.append(char(0x00));          // SA2：源单元号
	frame.append(char(++m_sid));       // SID：序列号

	// 命令码：0101 = 存储区读取
	frame.append(char(0x01));
	frame.append(char(0x01));
	frame.append(char(quint8(area)));  // 存储区代码
	frame.append(char(0x00));          // 起始地址（3字节，二进制大端）
	frame.append(char((startAddress >> 8) & 0xFF));
	frame.append(char(startAddress & 0xFF));
	appendU16(frame, count);           // 读取数量（二进制大端）
	return frame;
}

// 组装存储区写入指令帧（FINS 命令 0102）
QByteArray Communication::buildWriteFrame(FinsArea area, quint16 startAddress, const QVector<quint16>& values)
{
	QByteArray frame;
	frame.reserve(18 + values.size() * 2);

	// FINS 指令头（10 字节）
	frame.append(char(0x80));          // ICF：需要应答
	frame.append(char(0x00));          // RSV：保留
	frame.append(char(0x02));          // GCT：网关计数
	frame.append(char(0x00));          // DNA：目标网络号（本机网络）
	frame.append(char(m_serverNode));  // DA1：目标节点号（PLC）
	frame.append(char(0x00));          // DA2：目标单元号（CPU单元）
	frame.append(char(0x00));          // SNA：源网络号
	frame.append(char(m_clientNode));  // SA1：源节点号（本机）
	frame.append(char(0x00));          // SA2：源单元号
	frame.append(char(++m_sid));       // SID：序列号

	// 命令码：0102 = 存储区写入
	frame.append(char(0x01));
	frame.append(char(0x02));
	frame.append(char(quint8(area)));  // 存储区代码
	frame.append(char(0x00));          // 起始地址（3字节，二进制大端）
	frame.append(char((startAddress >> 8) & 0xFF));
	frame.append(char(startAddress & 0xFF));
	appendU16(frame, quint16(values.size())); // 写入数量（二进制大端）
	for (quint16 value : values)
		appendU16(frame, value);              // 数据区：每字2字节大端
	return frame;
}

// 发送 FINS 指令帧并接收应答（finsResponse 为去掉 FINS/TCP 头后的 FINS 应答帧）
bool Communication::sendFinsFrame(const QByteArray& finsFrame, QByteArray& finsResponse, int timeoutMs)
{
	finsResponse.clear();

	if (!isConnected())
	{
		m_lastError = QStringLiteral("未连接PLC");
		return false;
	}

	// FINS/TCP 帧头（16 字节）+ FINS 指令帧
	QByteArray pack;
	pack.reserve(16 + finsFrame.size());
	pack.append("FINS", 4);
	appendU32(pack, quint32(8 + finsFrame.size()));  // 长度：命令(4)+错误码(4)+FINS帧
	appendU32(pack, FinsTcpCommand_FinsFrame);       // 命令：FINS帧发送
	appendU32(pack, 0);                              // 错误码
	pack.append(finsFrame);

	if (m_sock->write(pack) != pack.size() || !m_sock->waitForBytesWritten(timeoutMs))
	{
		m_lastError = QStringLiteral("发送FINS指令失败：%1").arg(m_sock->errorString());
		return false;
	}

	// 接收 16 字节 FINS/TCP 应答头
	QByteArray head;
	if (!readExact(16, head, timeoutMs))
	{
		m_lastError = QStringLiteral("接收FINS应答超时");
		return false;
	}
	if (head.left(4) != QByteArrayLiteral("FINS"))
	{
		m_lastError = QStringLiteral("FINS应答头错误");
		return false;
	}
	if (readU32(head.mid(8, 4)) != FinsTcpCommand_FinsFrame)
	{
		m_lastError = QStringLiteral("FINS应答命令码错误");
		return false;
	}

	const quint32 tcpError = readU32(head.mid(12, 4));
	if (tcpError != 0)
	{
		m_lastError = QStringLiteral("FINS/TCP错误码：%1").arg(tcpError);
		return false;
	}

	const quint32 length = readU32(head.mid(4, 4));
	if (length < 8)
	{
		m_lastError = QStringLiteral("FINS应答长度错误");
		return false;
	}

	// 长度减去命令(4)和错误码(4)即为 FINS 应答帧长度
	const qint64 finsLen = qint64(length) - 8;
	if (finsLen > 0 && !readExact(finsLen, finsResponse, timeoutMs))
	{
		m_lastError = QStringLiteral("接收FINS应答数据超时");
		return false;
	}
	return true;
}

// 从 socket 精确读取指定字节数
bool Communication::readExact(qint64 bytesNeeded, QByteArray& out, int timeoutMs)
{
	out.clear();
	if (bytesNeeded <= 0)
		return true;

	QElapsedTimer elapsed;
	elapsed.start();
	while (out.size() < bytesNeeded)
	{
		const qint64 missing = bytesNeeded - out.size();
		if (m_sock->bytesAvailable() < missing)
		{
			const int remaining = timeoutMs - int(elapsed.elapsed());
			if (remaining <= 0 || !m_sock->waitForReadyRead(remaining))
				return false;
		}
		const QByteArray chunk = m_sock->read(missing);
		if (chunk.isEmpty())
		{
			if (m_sock->state() != QAbstractSocket::ConnectedState)
				return false;
			continue;
		}
		out.append(chunk);
	}
	return true;
}
void Communication::getWriteCommunicateData(quint16 startAddress, quint16 count, QVector<quint16>& values)
{
	w_startAddress = startAddress;
	w_count = count;
	w_values = values;  
}
void Communication::getConnectInfo(const QString& ip, quint16 port)
{
	m_ip = ip;
	m_port = port;
}
