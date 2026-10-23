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


	// IEEE754 位模式 <-> 欧姆龙 DM 里的 REAL（低字在前）
	quint16 realLowWord(float value)
	{
		quint32 raw = 0;
		memcpy(&raw, &value, sizeof(raw));
		return quint16(raw & 0xFFFF);
	}

	quint16 realHighWord(float value)
	{
		quint32 raw = 0;
		memcpy(&raw, &value, sizeof(raw));
		return quint16((raw >> 16) & 0xFFFF);
	}

	float wordsToReal(quint16 lowWord, quint16 highWord)
	{
		const quint32 raw = quint32(lowWord) | (quint32(highWord) << 16);
		float value = 0.0f;
		memcpy(&value, &raw, sizeof(value));
		return value;
	}
}

	// 逐字节比较两个 quint16 序列，避免实例化 QVector::operator==
	// （Qt5 的该函数会用到 MSVC 弃用的 stdext::make_checked_array_iterator，触发 STL4043）
	bool sameWords(const QVector<quint16>& a, const QVector<quint16>& b)
	{
		if (a.size() != b.size())
			return false;
		if (a.isEmpty())
			return true;
		return memcmp(a.constData(), b.constData(), size_t(a.size()) * sizeof(quint16)) == 0;
	}


Communication::Communication(void)
{  
	m_sock = nullptr;   // socket 延迟到真正使用它的线程里创建（见 ensureSocket()）
}

Communication::~Communication(void)
{
	requestInterruption();
	if (!wait(3000))
	{
		if (m_sock != nullptr)
			m_sock->abort();
		wait(1000);
	}
	disconnectPLC();

	// socket 故意不设 parent（为了能 moveToThread），这里手动释放
	if (m_sock != nullptr)
	{
		m_sock->abort();
		delete m_sock;
		m_sock = nullptr;
	}
}
void Communication::run()
{
	// ==================== 通讯表（华工-蚀刻机视觉通讯-20260416 / "3D通讯"页） ====================
	// PLC -> CCD（读）
	//   D2300 REAL 相机1触发   D2302 REAL 相机2触发
	//   D2304 REAL 相机1完成   D2306 REAL 相机2完成     => 连续 8 字：D2300~D2307
	//   D2330 REAL 缓存位1编码值 D2332 REAL 缓存位2编码值 => 连续 4 字：D2330~D2333
	//   D10000 String 产品型号
	// CCD -> PLC（写）
	//   D2312 心跳 / D2314 相机1启动 / D2316 相机1结果 / D2318 相机2启动 / D2320 相机2结果
	//   => 连续 10 字：D2312~D2321
	const quint16 kAddrTriggerDone = 2300;    // 4 个 REAL
	const quint16 kAddrCacheEncoder = 2330;   // 2 个 REAL
	const quint16 kAddrHeartbeat = 2312;      // 5 个 REAL
	const quint16 kAddrProductModel = 10000;  // String

	const float kCamOk = 10.0f;               // 触发/完成/OK 的取值
	const float kCamNg = 11.0f;   // 仅作说明；NG 值由界面回写，避免 C4189
	Q_UNUSED(kCamNg);

	ensureSocket();                           // socket 必须属于本线程

	bool connected = false;
	int  failCount = 0;
	int  backoffMs = 500;
	QElapsedTimer beatTimer;
	beatTimer.start();
	QElapsedTimer modelTimer;
	modelTimer.start();
	QVector<quint16> lastOut;                 // 上次写给 PLC 的 10 个字
	QString lastModel;

	m_pollingActive = true;

	while (!isInterruptionRequested())
	{
		QElapsedTimer cycle;
		cycle.start();

		// ---------------- 1. 连接 / 断线自动重连（指数退避） ----------------
		if (!connected)
		{
			if (m_ip.isEmpty())
			{
				m_lastError = QStringLiteral("PLC IP 未设置");
				QThread::msleep(500);
				continue;
			}
			if (!connectPLCInternal(m_ip, m_port, qBound(500, m_timeoutMs, 2000),
				m_serverNode, m_clientNode, false))
			{
				emit sendCommunicateMessage(QStringLiteral("PLC连接失败：%1").arg(m_lastError));
				QThread::msleep(backoffMs);
				backoffMs = qMin(backoffMs * 2, 5000);
				continue;
			}
			connected = true;
			failCount = 0;
			backoffMs = 500;
			lastOut.clear();
			emit sendCommunicateMessage(QStringLiteral("PLC已连接：%1:%2").arg(m_ip).arg(m_port));
		}
		
		QString err;

		// ---------------- 2. 读 PLC -> CCD ----------------
		// 2.1 触发/完成信号：D2300/2302/2304/2306 一次读回 8 个字
		QVector<float> trigDone;
		if (!readReals(AreaDM, kAddrTriggerDone, 4, trigDone, m_timeoutMs))
			err = QStringLiteral("读 D2300~D2307（触发/完成）失败：%1").arg(m_lastError);

		// 2.2 缓存位实时编码值：D2330~D2333
		QVector<float> encoders;
		if (err.isEmpty() && !readReals(AreaDM, kAddrCacheEncoder, 2, encoders, m_timeoutMs))
			err = QStringLiteral("读 D2330~D2333（缓存编码值）失败：%1").arg(m_lastError);

		// 2.3 产品型号：变化不频繁，1 秒刷新一次即可
		QString model = lastModel;
		if (err.isEmpty() && modelTimer.elapsed() >= 1000)
		{
			QString fresh;
			if (readString(AreaDM, kAddrProductModel, m_modelStringWords, fresh, m_timeoutMs))
			{
				model = fresh;
				lastModel = fresh;
			}
			else
			{
				err = QStringLiteral("读 D10000（产品型号）失败：%1").arg(m_lastError);
			}
			modelTimer.restart();
		}

		// 2.4 失败处理：连续 3 轮失败就重连（sendFinsFrame 出错时会 abort，这里要重新连）
		if (!err.isEmpty())
		{
			emit sendCommunicateMessage(err);
			if (!isConnected())
				connected = false;
			if (++failCount >= 3)
			{
				disconnectPLC();
				connected = false;
				failCount = 0;
				emit sendCommunicateMessage(QStringLiteral("PLC通信异常，准备重连"));
			}
			QThread::msleep(qBound(20, m_intervalMs, 1000));
			continue;
		}
		failCount = 0;

		// 2.5 缓存快照 + 通知界面
		PlcSignals sig;
		sig.cam1Trigger = trigDone.value(0);   // D2300
		sig.cam2Trigger = trigDone.value(1);   // D2302
		sig.cam1Done = trigDone.value(2);      // D2304
		sig.cam2Done = trigDone.value(3);      // D2306
		sig.cacheEncoder1 = encoders.value(0); // D2330
		sig.cacheEncoder2 = encoders.value(1); // D2332
		sig.productModel = model;              // D10000
		sig.stamp = QDateTime::currentDateTime();
		{
			QMutexLocker lock(&m_dataMutex);
			m_signals = sig;
		}
		emit plcSignalsUpdated();

		// ---------------- 3. 写 CCD -> PLC（D2312~D2321 一帧写完） ----------------
		PlcCommands cmd;
		{
			QMutexLocker lock(&m_dataMutex);
			cmd = m_commands;
		}

		// 3.1 心跳：默认 0，每 1 秒给一次 1（保持一个轮询周期）
		cmd.heartbeat = (quint64(beatTimer.elapsed() % 1000) < quint64(qMax(20, m_intervalMs))) ? 1.0f : 0.0f;

		// 3.2 "谁写谁清"：PLC 握手完成后把本机写的信号清掉
		if (m_autoClearCommands)
		{
			if (sig.cam1Done == kCamOk)     cmd.cam1Start = 0.0f;   // 相机1完成 -> 清启动信号
			if (sig.cam2Done == kCamOk)     cmd.cam2Start = 0.0f;   // 相机2完成 -> 清启动信号
			if (sig.cam1Trigger == 0.0f)    cmd.cam1Result = 0.0f;  // 触发归零 -> 清上次结果
			if (sig.cam2Trigger == 0.0f)    cmd.cam2Result = 0.0f;
		}

		// 3.3 组帧：5 个 REAL，低字在前
		QVector<quint16> out;
		out.reserve(10);
		out.append(realLowWord(cmd.heartbeat));  out.append(realHighWord(cmd.heartbeat));   // D2312
		out.append(realLowWord(cmd.cam1Start));  out.append(realHighWord(cmd.cam1Start));   // D2314
		out.append(realLowWord(cmd.cam1Result)); out.append(realHighWord(cmd.cam1Result));  // D2316
		out.append(realLowWord(cmd.cam2Start));  out.append(realHighWord(cmd.cam2Start));   // D2318
		out.append(realLowWord(cmd.cam2Result)); out.append(realHighWord(cmd.cam2Result));  // D2320

		// 3.4 内容没变就不发，减少 PLC 负载（心跳每秒必变，所以至少 1 次/秒）
		if (!sameWords(out, lastOut))
		{
			if (!writeValues(AreaDM, kAddrHeartbeat, out, m_timeoutMs))
			{
				emit sendCommunicateMessage(QStringLiteral("写 D2312~D2321 失败：%1").arg(m_lastError));
				if (!isConnected())
					connected = false;
				if (++failCount >= 3)
				{
					disconnectPLC();
					connected = false;
					failCount = 0;
				}
			}
			else
			{
				lastOut = out;
				QMutexLocker lock(&m_dataMutex);
				m_commands = cmd;   // 存回（含心跳和自动清零的结果），界面可查
			}
		}

		// ---------------- 4. 周期控制（可被 stopPlcPolling() 打断） ----------------
		for (int waited = int(cycle.elapsed()); waited < m_intervalMs && !isInterruptionRequested(); waited += 10)
			QThread::msleep(10);
	}

	if (connected)
		disconnectPLC();
	m_pollingActive = false;
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

	ensureSocket();

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
	if (m_pollingActive && QThread::currentThread() != this)
	{
		m_lastError = QStringLiteral("PLC实时轮询已启动：界面线程请用 setPublishCommands()/lastSignals()，不要直接读写");
		return false;
	}

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
	if (m_pollingActive && QThread::currentThread() != this)
	{
		m_lastError = QStringLiteral("PLC实时轮询已启动：界面线程请用 setPublishCommands()/lastSignals()，不要直接读写");
		return false;
	}


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
	frame.append(char((startAddress >> 8) & 0xFF));  // 起始地址：字地址高字节
	frame.append(char(startAddress & 0xFF));         // 起始地址：字地址低字节
	frame.append(char(0x00));                        // 位号：字访问固定 0x00
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
	frame.append(char((startAddress >> 8) & 0xFF));  // 起始地址：字地址高字节
	frame.append(char(startAddress & 0xFF));         // 起始地址：字地址低字节
	frame.append(char(0x00));                        // 位号：字访问固定 0x00
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

	// 发请求前丢掉上一请求残留的字节，避免应答整体错一位
	if (m_sock->bytesAvailable() > 0)
		m_sock->readAll();

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
		m_sock->abort();   // 失败即断链，交给 run() 重连，避免半包污染后续帧
		return false;
	}

	// 接收 16 字节 FINS/TCP 应答头
	QByteArray head;
	if (!readExact(16, head, timeoutMs))
	{
		m_lastError = QStringLiteral("接收FINS应答超时");
		m_sock->abort();
		return false;
	}
	if (head.left(4) != QByteArrayLiteral("FINS"))
	{
		m_lastError = QStringLiteral("FINS应答头错误");
		m_sock->abort();
		return false;
	}
	if (readU32(head.mid(8, 4)) != FinsTcpCommand_FinsFrame)
	{
		m_lastError = QStringLiteral("FINS应答命令码错误");
		m_sock->abort();
		return false;
	}

	const quint32 tcpError = readU32(head.mid(12, 4));
	if (tcpError != 0)
	{
		m_lastError = QStringLiteral("FINS/TCP错误码：%1").arg(tcpError);
		if (m_sock->bytesAvailable() > 0)
			m_sock->readAll();
		return false;
	}

	const quint32 length = readU32(head.mid(4, 4));
	if (length < quint32(8) || length > quint32(4096))
	{
		m_lastError = QStringLiteral("FINS应答长度错误");
		m_sock->abort();
		return false;
	}

	// 长度减去命令(4)和错误码(4)即为 FINS 应答帧长度
	const qint64 finsLen = qint64(length) - 8;
	if (finsLen > 0 && !readExact(finsLen, finsResponse, timeoutMs))
	{
		m_lastError = QStringLiteral("接收FINS应答数据超时");
		m_sock->abort();
		return false;
	}

	// 校验 SID（FINS 头第 10 字节），避免把迟到/残留应答当成本次结果
	if (finsResponse.size() > 9 && quint8(finsResponse.at(9)) != m_sid)
	{
		m_lastError = QStringLiteral("FINS应答SID不匹配（期望%1，实际%2）")
			.arg(int(m_sid))
			.arg(int(quint8(finsResponse.at(9))));
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

// ==================== 实时轮询接口 ====================
void Communication::startPlcPolling(const QString& ip, quint16 port, int intervalMs)
{
	if (isRunning())
		return;                       // 已在轮询，重复启动直接忽略

	m_ip = ip;
	m_port = port;
	m_intervalMs = qBound(20, intervalMs, 5000);
	m_lastError.clear();

	start();                          // run() 里完成连接 + 读写 + 重连
}

void Communication::stopPlcPolling()
{
	if (!isRunning())
		return;

	requestInterruption();
	if (!wait(3000))
	{
		if (m_sock != nullptr)
			m_sock->abort();          // 卡在 waitFor 上时强断，让 run() 尽快退出
		wait(1000);
	}
}

void Communication::setPublishCommands(const PlcCommands& cmd)
{
	QMutexLocker lock(&m_dataMutex);
	m_commands = cmd;
}

void Communication::setCamera1Command(float startValue, float resultValue)
{
	QMutexLocker lock(&m_dataMutex);
	m_commands.cam1Start = startValue;
	m_commands.cam1Result = resultValue;
}

void Communication::setCamera2Command(float startValue, float resultValue)
{
	QMutexLocker lock(&m_dataMutex);
	m_commands.cam2Start = startValue;
	m_commands.cam2Result = resultValue;
}

Communication::PlcSignals Communication::lastSignals() const
{
	QMutexLocker lock(&m_dataMutex);
	return m_signals;
}

void Communication::ensureSocket()
{
	// socket 只能在"使用它的线程"里，否则 waitForReadyRead/write 都会失败
	if (m_sock != nullptr && m_sock->thread() != QThread::currentThread())
	{
		m_sock->abort();
		m_sock->moveToThread(QThread::currentThread());
	}
	if (m_sock == nullptr)
		m_sock = new QTcpSocket();    // 不设 parent，才能 moveToThread
}

// ==================== 批量读/写 REAL 与 String ====================
bool Communication::readReals(FinsArea area, quint16 startAddress, int count,
	QVector<float>& values, int timeoutMs)
{
	values.clear();
	if (count <= 0 || count > 480)
	{
		m_lastError = QStringLiteral("实数个数必须在1~480之间");
		return false;
	}

	QVector<quint16> words;
	if (!readValues(area, startAddress, quint16(count * 2), words, timeoutMs))
		return false;
	if (words.size() < count * 2)
	{
		m_lastError = QStringLiteral("实数应答数据不完整");
		return false;
	}

	values.reserve(count);
	for (int i = 0; i < count; ++i)
		values.append(wordsToReal(words.at(i * 2), words.at(i * 2 + 1)));
	return true;
}

bool Communication::writeReals(FinsArea area, quint16 startAddress,
	const QVector<float>& values, int timeoutMs)
{
	if (values.isEmpty() || values.size() > 480)
	{
		m_lastError = QStringLiteral("实数个数必须在1~480之间");
		return false;
	}

	QVector<quint16> words;
	words.reserve(values.size() * 2);
	for (float value : values)
	{
		words.append(realLowWord(value));    // 低字在前
		words.append(realHighWord(value));
	}
	return writeValues(area, startAddress, words, timeoutMs);
}

bool Communication::readString(FinsArea area, quint16 startAddress, int wordCount,
	QString& text, int timeoutMs)
{
	text.clear();
	if (wordCount <= 0 || wordCount > 480)
	{
		m_lastError = QStringLiteral("字符串长度必须在1~480字之间");
		return false;
	}

	QVector<quint16> words;
	if (!readValues(area, startAddress, quint16(wordCount), words, timeoutMs))
		return false;

	// 欧姆龙 DM 里的 String：每字 2 个 ASCII 字符，高字节在前，遇 0x00 结束
	QByteArray raw;
	raw.reserve(words.size() * 2);
	for (quint16 word : words)
	{
		raw.append(char((word >> 8) & 0xFF));
		raw.append(char(word & 0xFF));
	}
	const int nul = raw.indexOf('\0');
	if (nul >= 0)
		raw.truncate(nul);
	while (!raw.isEmpty() && (raw.at(raw.size() - 1) == '\xFF' || raw.at(raw.size() - 1) == ' '))
		raw.chop(1);

	text = QString::fromLatin1(raw).trimmed();
	return true;
}

