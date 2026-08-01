#include "aspepprotocol.h"

#include <QtEndian>

#include <cstring>

namespace {
/*
 * ASPEP uses different IDs for the two data directions:
 *   0x9: controller -> performer request (MCTL_ASYNC/DATA_PACKET)
 *   0xA: performer -> controller synchronous MCP response (MCTL_SYNC)
 *
 * 0xA is also named ACK by the generic control definitions, but MCSDK's MCP
 * response carries a non-zero length field and must be parsed as a data frame.
 */
constexpr quint32 kAspepRequestData = 0x9U;
constexpr quint32 kAspepSyncData = 0xAU;
constexpr quint32 kAspepPing = 0x6U;
constexpr quint32 kAspepBeacon = 0x5U;
constexpr quint32 kAspepNack = 0xFU;
constexpr quint32 kMcpUserCommandMotor1 = 0x0101U;
constexpr quint32 kTelemetryMagic = 0x31434F46U;
constexpr quint32 kUartDiagnosticMagic = 0x31443355U;
constexpr quint8 kProtocolVersion = 3U;
constexpr int kLegacyTelemetrySize = 48;
constexpr int kTelemetrySize = 88;
constexpr int kHeaderPayloadGapMs = 2;
constexpr quint8 kMcpOk = 0;

constexpr quint8 kCrc4Table[16] = {
    0x0, 0x7, 0xE, 0x9, 0xB, 0xC, 0x5, 0x2,
    0x1, 0x6, 0xF, 0x8, 0xA, 0xD, 0x4, 0x3
};
}

AspepProtocol::AspepProtocol(QObject *parent)
    : QObject(parent)
    , serial_(this)
{
    qRegisterMetaType<FocTelemetry>();
    handshakeTimer_.setSingleShot(false);
    handshakeTimer_.setInterval(500);
    requestTimer_.setSingleShot(true);
    requestTimer_.setInterval(300);

    connect(&serial_, &SerialTransport::bytesReceived,
            this, &AspepProtocol::onBytesReceived);
    connect(&serial_, &SerialTransport::transportError, this,
            [this](const QString &message) {
                emit protocolError(message);
                disconnectPort();
            });
    connect(&handshakeTimer_, &QTimer::timeout,
            this, &AspepProtocol::onHandshakeTimeout);
    connect(&requestTimer_, &QTimer::timeout,
            this, &AspepProtocol::onRequestTimeout);
}

bool AspepProtocol::connectPort(const QString &portName, quint32 baudRate)
{
    disconnectPort();
    ++linkGeneration_;
    emit diagnosticMessage(tr("打开串口 %1，参数 %2 baud, 8-N-1")
                               .arg(portName)
                               .arg(baudRate));
    if (!serial_.open(portName, baudRate)) {
        emit diagnosticMessage(tr("打开失败：%1").arg(serial_.errorString()));
        emit connectionChanged(false, serial_.errorString());
        return false;
    }

    emit diagnosticMessage(tr("串口打开成功，清空收发缓存并启动 ASPEP 握手"));
    receiveBuffer_.clear();
    commandQueue_.clear();
    requestOutstanding_ = false;
    handshakeAttempts_ = 0;
    receivedAnyBytes_ = false;
    invalidHeaderByteCount_ = 0;
    outstandingCommandId_ = 0;
    consecutiveTelemetryTimeouts_ = 0;
    telemetryLogAnnounced_ = false;
    uartDiagnosticLogAnnounced_ = false;
    uartDiagnosticLogTimer_.restart();
    setLinkState(LinkState::WaitingBeacon, tr("串口已打开，正在进行 ASPEP 握手…"));
    sendBeacon();
    handshakeTimer_.start();
    return true;
}

void AspepProtocol::disconnectPort()
{
    ++linkGeneration_;
    const bool wasOpen = serial_.isOpen();
    handshakeTimer_.stop();
    requestTimer_.stop();
    serial_.close();
    receiveBuffer_.clear();
    commandQueue_.clear();
    requestOutstanding_ = false;
    handshakeAttempts_ = 0;
    outstandingCommandId_ = 0;
    setLinkState(LinkState::Closed, tr("未连接"));
    if (wasOpen) {
        emit diagnosticMessage(tr("串口已关闭"));
    }
}

bool AspepProtocol::isPortOpen() const
{
    return serial_.isOpen();
}

bool AspepProtocol::isConnected() const
{
    return state_ == LinkState::Connected;
}

QStringList AspepProtocol::availablePorts()
{
    return SerialTransport::availablePorts();
}

void AspepProtocol::requestTelemetry()
{
    if (!requestOutstanding_ && commandQueue_.isEmpty()) {
        queueCommand(0);
    }
}

void AspepProtocol::setMode(bool positionMode)
{
    queueCommand(1, QByteArray(1, positionMode ? '\x01' : '\x00'));
}

void AspepProtocol::setSpeedRpm(qint16 rpm, quint32 durationMs)
{
    QByteArray arguments;
    appendS16(arguments, rpm);
    appendU32(arguments, durationMs);
    queueCommand(2, arguments);
}

void AspepProtocol::setPositionCdeg(qint32 cdeg, quint32 durationMs)
{
    QByteArray arguments;
    appendS32(arguments, cdeg);
    appendU32(arguments, durationMs);
    queueCommand(3, arguments);
}

void AspepProtocol::startMotor()
{
    queueCommand(4);
}

void AspepProtocol::stopMotor()
{
    queueCommand(5);
}

void AspepProtocol::acknowledgeFault()
{
    queueCommand(6);
}

quint32 AspepProtocol::addHeaderCrc(quint32 header)
{
    header &= 0x0FFFFFFFU;
    quint8 crc = 0;
    for (int shift = 0; shift <= 24; shift += 4) {
        crc = kCrc4Table[crc ^ static_cast<quint8>((header >> shift) & 0xFU)];
    }
    return header | (static_cast<quint32>(crc) << 28);
}

bool AspepProtocol::validHeaderCrc(quint32 header)
{
    return addHeaderCrc(header) == header;
}

quint32 AspepProtocol::readU32(const char *data)
{
    quint32 value;
    std::memcpy(&value, data, sizeof(value));
    return qFromLittleEndian(value);
}

quint16 AspepProtocol::readU16(const char *data)
{
    quint16 value;
    std::memcpy(&value, data, sizeof(value));
    return qFromLittleEndian(value);
}

qint16 AspepProtocol::readS16(const char *data)
{
    return static_cast<qint16>(readU16(data));
}

qint32 AspepProtocol::readS32(const char *data)
{
    return static_cast<qint32>(readU32(data));
}

void AspepProtocol::appendS16(QByteArray &data, qint16 value)
{
    const quint16 littleEndian = qToLittleEndian(static_cast<quint16>(value));
    data.append(reinterpret_cast<const char *>(&littleEndian), sizeof(littleEndian));
}

void AspepProtocol::appendS32(QByteArray &data, qint32 value)
{
    const quint32 littleEndian = qToLittleEndian(static_cast<quint32>(value));
    data.append(reinterpret_cast<const char *>(&littleEndian), sizeof(littleEndian));
}

void AspepProtocol::appendU32(QByteArray &data, quint32 value)
{
    const quint32 littleEndian = qToLittleEndian(value);
    data.append(reinterpret_cast<const char *>(&littleEndian), sizeof(littleEndian));
}

void AspepProtocol::sendBeacon()
{
    quint32 header = kAspepBeacon |
                     (0U << 4) |       // version
                     (0U << 7) |       // data CRC disabled
                     (7U << 8) |       // controller RX payload: 256 B
                     (7U << 14) |      // synchronous TX payload: 256 B
                     (32U << 21);      // asynchronous TX payload: 2048 B
    header = qToLittleEndian(addHeaderCrc(header));
    const QByteArray frame(reinterpret_cast<const char *>(&header), sizeof(header));
    logFrame(QStringLiteral("TX BEACON"), frame);
    serial_.write(frame);
}

void AspepProtocol::sendPing()
{
    quint32 header = qToLittleEndian(addHeaderCrc(kAspepPing));
    const QByteArray frame(reinterpret_cast<const char *>(&header), sizeof(header));
    logFrame(QStringLiteral("TX PING"), frame);
    serial_.write(frame);
}

void AspepProtocol::onBytesReceived(const QByteArray &data)
{
    receivedAnyBytes_ = true;
    if (state_ != LinkState::Connected) {
        logFrame(QStringLiteral("RX RAW"), data);
    }
    receiveBuffer_.append(data);
    processFrames();
}

void AspepProtocol::processFrames()
{
    while (receiveBuffer_.size() >= 4) {
        const quint32 header = readU32(receiveBuffer_.constData());
        if (!validHeaderCrc(header)) {
            ++invalidHeaderByteCount_;
            emit diagnosticMessage(
                tr("ASPEP 头 CRC 无效，丢弃字节 0x%1 并重新同步")
                    .arg(static_cast<quint8>(receiveBuffer_.front()),
                         2, 16, QLatin1Char('0')));
            receiveBuffer_.remove(0, 1);
            continue;
        }

        const quint32 type = header & 0xFU;
        const bool isDataFrame =
            (type == kAspepRequestData) ||
            ((type == kAspepSyncData) && ((header & 0x1FFF0U) != 0U));
        const int payloadLength = isDataFrame
                                      ? static_cast<int>((header & 0x1FFF0U) >> 4)
                                      : 0;
        const int frameLength = 4 + payloadLength;
        if (receiveBuffer_.size() < frameLength) {
            if (state_ != LinkState::Connected) {
            emit diagnosticMessage(
                tr("ASPEP 帧未收完整：当前 %1 字节，需要 %2 字节")
                    .arg(receiveBuffer_.size())
                    .arg(frameLength));
            }
            return;
        }

        if (state_ != LinkState::Connected) {
            emit diagnosticMessage(
                tr("ASPEP 帧：类型 0x%1，负载 %2 字节")
                    .arg(type, 1, 16)
                    .arg(payloadLength));
        }
        if (isDataFrame) {
            processDataFrame(receiveBuffer_.mid(4, payloadLength));
        } else {
            processControlFrame(header);
        }
        receiveBuffer_.remove(0, frameLength);
    }
}

void AspepProtocol::processControlFrame(quint32 header)
{
    const quint32 type = header & 0xFU;
    if (type == kAspepBeacon && state_ == LinkState::WaitingBeacon) {
        emit diagnosticMessage(tr("收到 BEACON，双方能力参数匹配"));
        handshakeAttempts_ = 0;
        setLinkState(LinkState::WaitingPing, tr("ASPEP 参数已匹配，正在建立会话…"));
        sendPing();
    } else if (type == kAspepPing && state_ == LinkState::WaitingPing) {
        emit diagnosticMessage(tr("收到 PING，ASPEP 握手完成"));
        handshakeTimer_.stop();
        setLinkState(LinkState::Connected, tr("已连接"));
        requestTelemetry();
    } else if (type == kAspepNack) {
        const quint8 errorCode = static_cast<quint8>((header >> 8U) & 0xFFU);
        const quint8 failedCommandId = outstandingCommandId_;
        QString reason;
        switch (errorCode) {
        case 1: reason = tr("不支持的包类型"); break;
        case 2: reason = tr("包长度超出协商范围"); break;
        case 4: reason = tr("ASPEP 头 CRC 错误"); break;
        case 5: reason = tr("数据 CRC 错误"); break;
        default: reason = tr("未知错误"); break;
        }
        requestTimer_.stop();
        requestOutstanding_ = false;
        outstandingCommandId_ = 0;
        if (failedCommandId == 0U) {
            ++consecutiveTelemetryTimeouts_;
            if ((consecutiveTelemetryTimeouts_ == 1) ||
                ((consecutiveTelemetryTimeouts_ % 20) == 0)) {
                emit protocolError(
                    tr("遥测 ASPEP NACK：%1（连续 %2 次，重复信息已抑制）")
                        .arg(reason)
                        .arg(consecutiveTelemetryTimeouts_));
            }
        } else {
            emit protocolError(
                tr("MCP 用户命令 %1 收到 ASPEP NACK：%2")
                    .arg(failedCommandId)
                    .arg(reason));
        }
        sendNextCommand();
    } else {
        emit diagnosticMessage(
            tr("收到未用于当前握手阶段的控制帧，类型 0x%1")
                .arg(type, 1, 16));
    }
}

void AspepProtocol::processDataFrame(const QByteArray &payload)
{
    const quint8 completedCommandId = outstandingCommandId_;
    requestTimer_.stop();
    requestOutstanding_ = false;
    outstandingCommandId_ = 0;

    if (payload.isEmpty()) {
        emit protocolError(tr("收到空的 MCP 响应"));
        sendNextCommand();
        return;
    }

    const quint8 status = static_cast<quint8>(payload.back());
    if (status != kMcpOk) {
        QString statusReason;
        switch (status) {
        case 0x01: statusReason = tr("命令未执行"); break;
        case 0x02: statusReason = tr("未知命令"); break;
        case 0x08: statusReason = tr("同步发送缓冲区空间不足"); break;
        case 0x0D: statusReason = tr("用户回调未注册，请确认已烧录包含 FocAppProtocol_Init 的新固件"); break;
        default: statusReason = tr("未定义错误"); break;
        }
        emit diagnosticMessage(
            tr("MCP 响应失败：状态码 0x%1（%2），负载长度 %3")
                .arg(status, 2, 16, QLatin1Char('0'))
                .arg(statusReason)
                .arg(payload.size()));
        emit protocolError(tr("MCP 命令失败，状态码 0x%1：%2")
                               .arg(status, 2, 16, QLatin1Char('0'))
                               .arg(statusReason));
        sendNextCommand();
        return;
    }

    if (payload.size() >= kLegacyTelemetrySize + 1 &&
        readU32(payload.constData()) == kTelemetryMagic) {
        FocTelemetry value;
        value.protocolVersion = static_cast<quint8>(payload[4]);
        value.mode = static_cast<quint8>(payload[5]);
        value.motorState = static_cast<quint8>(payload[6]);
        value.currentFaults = readU16(payload.constData() + 8);
        value.occurredFaults = readU16(payload.constData() + 10);
        value.iqA = readS32(payload.constData() + 12) / 1000.0;
        value.idA = readS32(payload.constData() + 16) / 1000.0;
        value.iqRefA = readS32(payload.constData() + 20) / 1000.0;
        value.idRefA = readS32(payload.constData() + 24) / 1000.0;
        value.uqV = readS32(payload.constData() + 28) / 1000.0;
        value.udV = readS32(payload.constData() + 32) / 1000.0;
        value.speedReferenceRpm = readS16(payload.constData() + 36);
        value.speedMeasuredRpm = readS16(payload.constData() + 38);
        value.targetDegree = readS32(payload.constData() + 40) / 100.0;
        value.currentDegree = readS32(payload.constData() + 44) / 100.0;
        if ((payload.size() >= kTelemetrySize + 1) &&
            (readU32(payload.constData() + 48) == kUartDiagnosticMagic)) {
            value.uartDiagnosticsAvailable = true;
            value.uartInitialized = payload[52] != 0;
            value.uartLinkActive = payload[53] != 0;
            value.uartInitStage = static_cast<quint8>(payload[54]);
            value.uartLastTxStatus = static_cast<quint8>(payload[55]);
            value.uartError = readU32(payload.constData() + 56);
            value.uartReceivedBytes = readU32(payload.constData() + 60);
            value.uartValidCommandFrames = readU32(payload.constData() + 64);
            value.uartCrcErrors = readU32(payload.constData() + 68);
            value.uartProtocolErrors = readU32(payload.constData() + 72);
            value.uartTelemetryAttempts = readU32(payload.constData() + 76);
            value.uartTelemetrySent = readU32(payload.constData() + 80);
            value.uartTelemetryErrors = readU32(payload.constData() + 84);
        }
        if (value.protocolVersion != kProtocolVersion) {
            emit protocolError(
                tr("固件协议版本为 %1，MIX 控制台需要版本 3；请烧录最新 MIX 固件")
                    .arg(value.protocolVersion));
        }
        consecutiveTelemetryTimeouts_ = 0;
        if (!telemetryLogAnnounced_) {
            telemetryLogAnnounced_ = true;
            emit diagnosticMessage(tr("遥测接收正常，周期数据日志已自动静默"));
        } else if (completedCommandId != 0U) {
            emit diagnosticMessage(
                tr("MCP 用户命令 %1 执行成功").arg(completedCommandId));
        }
        if (value.uartDiagnosticsAvailable &&
            (!uartDiagnosticLogAnnounced_ ||
             (uartDiagnosticLogTimer_.elapsed() >= 1000))) {
            uartDiagnosticLogAnnounced_ = true;
            uartDiagnosticLogTimer_.restart();
            emit diagnosticMessage(
                QStringLiteral(
                    "USART3 DIAG init=%1 stage=%2 link=%3 "
                    "rx_bytes=%4 valid_cmd=%5 crc_err=%6 proto_err=%7 "
                    "tx_try=%8 tx_ok=%9 tx_err=%10 hal_status=%11 hal_error=0x%12")
                    .arg(value.uartInitialized ? QStringLiteral("READY")
                                               : QStringLiteral("FAILED"))
                    .arg(value.uartInitStage)
                    .arg(value.uartLinkActive ? QStringLiteral("ONLINE")
                                              : QStringLiteral("OFFLINE"))
                    .arg(value.uartReceivedBytes)
                    .arg(value.uartValidCommandFrames)
                    .arg(value.uartCrcErrors)
                    .arg(value.uartProtocolErrors)
                    .arg(value.uartTelemetryAttempts)
                    .arg(value.uartTelemetrySent)
                    .arg(value.uartTelemetryErrors)
                    .arg(value.uartLastTxStatus)
                    .arg(value.uartError, 8, 16, QLatin1Char('0')));
        }
        emit telemetryReceived(value);
    } else {
        emit diagnosticMessage(
            tr("MCP 成功响应，但未发现有效遥测块：负载长度 %1")
                .arg(payload.size()));
    }

    sendNextCommand();
}

void AspepProtocol::queueCommand(quint8 command, const QByteArray &arguments)
{
    if (!isConnected()) {
        return;
    }

    QByteArray commandData;
    commandData.append(static_cast<char>(command));
    commandData.append(arguments);
    if ((command == 2U) || (command == 3U)) {
        for (auto iterator = commandQueue_.begin(); iterator != commandQueue_.end();) {
            if (!iterator->isEmpty() &&
                static_cast<quint8>(iterator->at(0)) == command) {
                iterator = commandQueue_.erase(iterator);
            } else {
                ++iterator;
            }
        }
    }
    commandQueue_.append(commandData);
    sendNextCommand();
}

void AspepProtocol::sendNextCommand()
{
    if (!isConnected() || requestOutstanding_ || commandQueue_.isEmpty()) {
        return;
    }

    const QByteArray command = commandQueue_.takeFirst();
    QByteArray payload;
    const quint16 mcpHeader = qToLittleEndian(static_cast<quint16>(kMcpUserCommandMotor1));
    payload.append(reinterpret_cast<const char *>(&mcpHeader), sizeof(mcpHeader));
    payload.append(command);

    quint32 aspepHeader = addHeaderCrc(
        (static_cast<quint32>(payload.size()) << 4U) | kAspepRequestData);
    aspepHeader = qToLittleEndian(aspepHeader);

    const QByteArray headerBytes(reinterpret_cast<const char *>(&aspepHeader),
                                 sizeof(aspepHeader));
    const quint8 commandId = static_cast<quint8>(command.at(0));
    const quint32 generation = linkGeneration_;

    /*
     * MCSDK 6.4.2 receives the 4-byte ASPEP header with DMA, then polls the
     * DMA completion from its 2 kHz SysTick before arming DMA for the payload.
     * Sending header and payload in one uninterrupted Windows write can overrun
     * the disabled USART RX FIFO.  Leave two milliseconds for that re-arm.
     */
    if (commandId != 0U) {
        logFrame(tr("TX MCP CMD %1 HEADER").arg(commandId), headerBytes);
    }
    if (!serial_.write(headerBytes) || !serial_.flush()) {
        return;
    }

    requestOutstanding_ = true;
    outstandingCommandId_ = commandId;
    QTimer::singleShot(kHeaderPayloadGapMs, this,
                       [this, payload, commandId, generation] {
        if ((generation != linkGeneration_) || !isConnected()) {
            return;
        }

        if (commandId != 0U) {
            logFrame(tr("TX MCP CMD %1 PAYLOAD").arg(commandId), payload);
        }
        if (serial_.write(payload)) {
            requestTimer_.start();
        } else {
            requestOutstanding_ = false;
        }
    });
}

void AspepProtocol::setLinkState(LinkState state, const QString &status)
{
    state_ = state;
    emit connectionChanged(state == LinkState::Connected, status);
}

void AspepProtocol::onHandshakeTimeout()
{
    ++handshakeAttempts_;
    emit diagnosticMessage(
        tr("ASPEP 握手等待超时，第 %1/8 次；当前阶段=%2")
            .arg(handshakeAttempts_)
            .arg(state_ == LinkState::WaitingBeacon ? tr("等待 BEACON")
                                                     : tr("等待 PING")));
    if (handshakeAttempts_ >= 8) {
        if (!receivedAnyBytes_) {
            emit protocolError(
                tr("ASPEP 握手超时且未收到任何字节：请确认 MIX 固件为 1,843,200 baud，并检查 COM 口、TX/RX 和共地"));
        } else if (invalidHeaderByteCount_ > 0U) {
            emit protocolError(
                tr("收到串口数据但 ASPEP 头无效：通常是波特率不一致、线路噪声或接错串口"));
        } else {
            emit protocolError(
                tr("收到数据但 ASPEP 握手未完成：请确认固件启用了 MCSDK ASPEP/MCP"));
        }
        disconnectPort();
        return;
    }

    if (state_ == LinkState::WaitingBeacon) {
        sendBeacon();
    } else if (state_ == LinkState::WaitingPing) {
        sendPing();
    }
}

void AspepProtocol::onRequestTimeout()
{
    const quint8 timedOutCommandId = outstandingCommandId_;
    requestOutstanding_ = false;
    outstandingCommandId_ = 0;
    commandQueue_.clear();
    if (timedOutCommandId == 0U) {
        ++consecutiveTelemetryTimeouts_;
        if ((consecutiveTelemetryTimeouts_ == 1) ||
            ((consecutiveTelemetryTimeouts_ % 20) == 0)) {
            emit protocolError(
                tr("遥测响应超时（连续 %1 次，重复信息已抑制）")
                    .arg(consecutiveTelemetryTimeouts_));
        }
    } else {
        emit protocolError(
            tr("MCP 用户命令 %1 响应超时").arg(timedOutCommandId));
    }
}

void AspepProtocol::logFrame(const QString &direction, const QByteArray &data)
{
    emit diagnosticMessage(
        QStringLiteral("%1 [%2]  %3")
            .arg(direction)
            .arg(data.size())
            .arg(QString::fromLatin1(data.toHex(' ').toUpper())));
}
