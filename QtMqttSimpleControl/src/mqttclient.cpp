#include "mqttclient.h"

#include <QAbstractSocket>
#include <QNetworkProxy>

namespace {
constexpr quint8 kMqttConnAck = 2U;
constexpr quint16 kKeepAliveSeconds = 20U;
}

MqttClient::MqttClient(QObject *parent)
    : QObject(parent)
{
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
            });
    connect(&keepAliveTimer_, &QTimer::timeout,
            this, &MqttClient::sendPing);
}

void MqttClient::connectToBroker(const QString &host, quint16 port,
                                 const QString &clientId)
{
    disconnectFromBroker();
    inputBuffer_.clear();
    clientId_ = clientId;
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
        socket_.abort();
    }
}

bool MqttClient::isConnected() const
{
    return mqttConnected_;
}

bool MqttClient::publishQos1(const QString &topic,
                             const QByteArray &payload)
{
    if (!mqttConnected_ || topic.isEmpty()) {
        return false;
    }

    QByteArray body;
    appendMqttString(body, topic.toUtf8());
    const quint16 identifier = nextPacketId();
    body.append(static_cast<char>((identifier >> 8U) & 0xFFU));
    body.append(static_cast<char>(identifier & 0xFFU));
    body.append(payload);
    return sendPacket(0x32U, body);
}

void MqttClient::onTcpConnected()
{
    QByteArray body;
    appendMqttString(body, QByteArrayLiteral("MQTT"));
    body.append('\x04');
    body.append('\x02');
    body.append(static_cast<char>(kKeepAliveSeconds >> 8U));
    body.append(static_cast<char>(kKeepAliveSeconds & 0xFFU));
    appendMqttString(body, clientId_.toUtf8());
    if (!sendPacket(0x10U, body)) {
        emit errorOccurred(tr("Failed to send MQTT CONNECT"));
    }
}

void MqttClient::onTcpDisconnected()
{
    const bool wasConnected = mqttConnected_;
    mqttConnected_ = false;
    keepAliveTimer_.stop();
    inputBuffer_.clear();
    if (wasConnected) {
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
    if (mqttConnected_) {
        sendPacket(0xC0U, {});
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
        bool lengthComplete = false;
        for (qsizetype index = 1;
             index < inputBuffer_.size() && index <= 4; ++index) {
            const quint8 byte = static_cast<quint8>(inputBuffer_[index]);
            remaining += static_cast<qsizetype>(byte & 0x7FU) * multiplier;
            ++lengthBytes;
            if ((byte & 0x80U) == 0U) {
                lengthComplete = true;
                break;
            }
            multiplier *= 128;
        }
        if (!lengthComplete) {
            return;
        }

        const qsizetype packetLength = 1 + lengthBytes + remaining;
        if (inputBuffer_.size() < packetLength) {
            return;
        }
        const quint8 header = static_cast<quint8>(inputBuffer_[0]);
        const QByteArray body = inputBuffer_.mid(1 + lengthBytes, remaining);
        inputBuffer_.remove(0, packetLength);

        if ((header >> 4U) != kMqttConnAck) {
            continue;
        }
        if (body.size() != 2 || static_cast<quint8>(body[1]) != 0U) {
            const int result = body.size() >= 2
                ? static_cast<quint8>(body[1]) : -1;
            emit errorOccurred(
                tr("MQTT connection refused, result %1").arg(result));
            socket_.abort();
            continue;
        }
        if (!mqttConnected_) {
            mqttConnected_ = true;
            keepAliveTimer_.start();
            emit connected();
        }
    }
}
