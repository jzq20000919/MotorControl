#include "mqttclient.h"

#include <QAbstractSocket>
#include <QNetworkProxy>

namespace {
constexpr quint8 kMqttConnect = 1U;
constexpr quint8 kMqttConnAck = 2U;
constexpr quint8 kMqttPublish = 3U;
constexpr quint8 kMqttPubAck = 4U;
constexpr quint8 kMqttSubAck = 9U;
constexpr quint8 kMqttPingResp = 13U;
constexpr quint16 kKeepAliveSeconds = 20U;
}

MqttClient::MqttClient(QObject *parent)
    : QObject(parent)
{
    // The broker is on the local network; never route raw MQTT through the
    // Windows HTTP/system proxy.
    socket_.setProxy(QNetworkProxy::NoProxy);
    keepAliveTimer_.setInterval(10000);
    connect(&socket_, &QTcpSocket::connected,
            this, &MqttClient::onTcpConnected);
    connect(&socket_, &QTcpSocket::disconnected,
            this, &MqttClient::onTcpDisconnected);
    connect(&socket_, &QTcpSocket::readyRead,
            this, &MqttClient::onReadyRead);
    connect(&socket_, &QTcpSocket::errorOccurred, this,
            [this](QAbstractSocket::SocketError) {
                emit errorOccurred(socket_.errorString());
                QTimer::singleShot(0, this, [this] {
                    if (socket_.state() == QAbstractSocket::UnconnectedState &&
                        !disconnectSignalEmitted_) {
                        disconnectSignalEmitted_ = true;
                        emit disconnected();
                    }
                });
            });
    connect(&keepAliveTimer_, &QTimer::timeout,
            this, &MqttClient::sendPing);
}

MqttClient::~MqttClient()
{
    keepAliveTimer_.stop();
    socket_.blockSignals(true);
    mqttConnected_ = false;
    pingOutstanding_ = false;
    socket_.abort();
}

void MqttClient::connectToBroker(const QString &host, quint16 port,
                                 const QString &clientId)
{
    disconnectFromBroker();
    inputBuffer_.clear();
    clientId_ = clientId;
    disconnectSignalEmitted_ = false;
    emit diagnosticMessage(
        tr("Connecting to MQTT broker %1:%2").arg(host).arg(port));
    socket_.connectToHost(host, port);
}

void MqttClient::disconnectFromBroker()
{
    keepAliveTimer_.stop();
    if (mqttConnected_) {
        sendPacket(0xE0U, {});
    }
    mqttConnected_ = false;
    if (socket_.state() != QAbstractSocket::UnconnectedState) {
        socket_.disconnectFromHost();
        if (socket_.state() != QAbstractSocket::UnconnectedState) {
            socket_.abort();
        }
    }
}

bool MqttClient::isSocketOpen() const
{
    return socket_.state() != QAbstractSocket::UnconnectedState;
}

bool MqttClient::isConnected() const
{
    return mqttConnected_;
}

bool MqttClient::subscribe(const QString &topic, quint8 qos)
{
    if (!mqttConnected_ || topic.isEmpty() || qos > 1U) {
        return false;
    }
    QByteArray body;
    const quint16 identifier = nextPacketId();
    body.append(static_cast<char>((identifier >> 8U) & 0xFFU));
    body.append(static_cast<char>(identifier & 0xFFU));
    appendMqttString(body, topic.toUtf8());
    body.append(static_cast<char>(qos));
    return sendPacket(0x82U, body);
}

bool MqttClient::publish(const QString &topic, const QByteArray &payload,
                         quint8 qos, bool retain)
{
    if (!mqttConnected_ || topic.isEmpty() || qos > 1U) {
        return false;
    }
    QByteArray body;
    appendMqttString(body, topic.toUtf8());
    if (qos == 1U) {
        const quint16 identifier = nextPacketId();
        body.append(static_cast<char>((identifier >> 8U) & 0xFFU));
        body.append(static_cast<char>(identifier & 0xFFU));
    }
    body.append(payload);
    const quint8 header = static_cast<quint8>(
        0x30U | (qos << 1U) | (retain ? 1U : 0U));
    return sendPacket(header, body);
}

void MqttClient::onTcpConnected()
{
    disconnectSignalEmitted_ = false;
    pingOutstanding_ = false;
    QByteArray variableHeader;
    appendMqttString(variableHeader, QByteArrayLiteral("MQTT"));
    variableHeader.append('\x04');
    variableHeader.append('\x02');
    variableHeader.append(static_cast<char>(kKeepAliveSeconds >> 8U));
    variableHeader.append(static_cast<char>(kKeepAliveSeconds & 0xFFU));

    QByteArray payload;
    appendMqttString(payload, clientId_.toUtf8());
    variableHeader.append(payload);
    sendPacket(0x10U, variableHeader);
}

void MqttClient::onTcpDisconnected()
{
    const bool wasConnected = mqttConnected_;
    mqttConnected_ = false;
    pingOutstanding_ = false;
    keepAliveTimer_.stop();
    inputBuffer_.clear();
    if (wasConnected) {
        emit diagnosticMessage(tr("MQTT broker disconnected"));
    }
    if (!disconnectSignalEmitted_) {
        disconnectSignalEmitted_ = true;
        emit disconnected();
    }
}

void MqttClient::onReadyRead()
{
    inputBuffer_.append(socket_.readAll());
    processInput();
}

void MqttClient::sendPing()
{
    if (!mqttConnected_) {
        return;
    }
    if (pingOutstanding_) {
        emit errorOccurred(tr("MQTT keepalive timed out"));
        socket_.abort();
        return;
    }
    if (sendPacket(0xC0U, {})) {
        pingOutstanding_ = true;
    }
}

void MqttClient::appendMqttString(QByteArray &target,
                                  const QByteArray &value)
{
    const quint16 length = static_cast<quint16>(
        qMin<qsizetype>(value.size(), 65535));
    target.append(static_cast<char>((length >> 8U) & 0xFFU));
    target.append(static_cast<char>(length & 0xFFU));
    target.append(value.constData(), length);
}

QByteArray MqttClient::encodeRemainingLength(qsizetype length)
{
    QByteArray encoded;
    do {
        quint8 byte = static_cast<quint8>(length % 128);
        length /= 128;
        if (length > 0) {
            byte |= 0x80U;
        }
        encoded.append(static_cast<char>(byte));
    } while (length > 0);
    return encoded;
}

bool MqttClient::sendPacket(quint8 header, const QByteArray &body)
{
    if (socket_.state() != QAbstractSocket::ConnectedState) {
        return false;
    }
    QByteArray packet;
    packet.append(static_cast<char>(header));
    packet.append(encodeRemainingLength(body.size()));
    packet.append(body);
    return socket_.write(packet) == packet.size();
}

quint16 MqttClient::nextPacketId()
{
    ++packetId_;
    if (packetId_ == 0U) {
        ++packetId_;
    }
    return packetId_;
}

void MqttClient::processInput()
{
    while (inputBuffer_.size() >= 2) {
        qsizetype multiplier = 1;
        qsizetype remaining = 0;
        qsizetype lengthBytes = 0;
        bool completeLength = false;
        for (qsizetype index = 1; index < inputBuffer_.size() && index <= 4;
             ++index) {
            const quint8 byte = static_cast<quint8>(inputBuffer_[index]);
            remaining += static_cast<qsizetype>(byte & 0x7FU) * multiplier;
            ++lengthBytes;
            if ((byte & 0x80U) == 0U) {
                completeLength = true;
                break;
            }
            multiplier *= 128;
        }
        if (!completeLength) {
            return;
        }
        const qsizetype packetLength = 1 + lengthBytes + remaining;
        if (inputBuffer_.size() < packetLength) {
            return;
        }
        const quint8 header = static_cast<quint8>(inputBuffer_[0]);
        const QByteArray body = inputBuffer_.mid(1 + lengthBytes, remaining);
        inputBuffer_.remove(0, packetLength);
        processPacket(header, body);
    }
}

void MqttClient::processPacket(quint8 header, const QByteArray &body)
{
    const quint8 type = header >> 4U;
    if (type == kMqttConnAck) {
        if (body.size() != 2 || static_cast<quint8>(body[1]) != 0U) {
            const int result = body.size() >= 2
                ? static_cast<quint8>(body[1]) : -1;
            emit errorOccurred(
                tr("MQTT connection refused, result %1").arg(result));
            socket_.disconnectFromHost();
            return;
        }
        mqttConnected_ = true;
        keepAliveTimer_.start();
        emit diagnosticMessage(tr("MQTT 3.1.1 session established"));
        emit connected();
        return;
    }

    if (type == kMqttPublish) {
        if (body.size() < 2) {
            return;
        }
        const quint16 topicLength =
            (static_cast<quint16>(static_cast<quint8>(body[0])) << 8U) |
            static_cast<quint8>(body[1]);
        qsizetype offset = 2 + topicLength;
        if (offset > body.size()) {
            return;
        }
        const QString topic = QString::fromUtf8(body.constData() + 2,
                                                 topicLength);
        const quint8 qos = (header >> 1U) & 0x03U;
        quint16 identifier = 0U;
        if (qos > 0U) {
            if (offset + 2 > body.size()) {
                return;
            }
            identifier =
                (static_cast<quint16>(
                    static_cast<quint8>(body[offset])) << 8U) |
                static_cast<quint8>(body[offset + 1]);
            offset += 2;
        }
        emit messageReceived(topic, body.mid(offset));
        if (qos == 1U) {
            QByteArray acknowledgement;
            acknowledgement.append(
                static_cast<char>((identifier >> 8U) & 0xFFU));
            acknowledgement.append(static_cast<char>(identifier & 0xFFU));
            sendPacket(0x40U, acknowledgement);
        }
        return;
    }

    if (type == kMqttPingResp) {
        pingOutstanding_ = false;
        return;
    }

    if (type == kMqttPubAck || type == kMqttSubAck) {
        return;
    }
}
