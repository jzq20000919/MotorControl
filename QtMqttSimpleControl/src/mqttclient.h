#pragma once

#include <QByteArray>
#include <QObject>
#include <QString>
#include <QTcpSocket>
#include <QTimer>

class MqttClient final : public QObject
{
    Q_OBJECT

public:
    explicit MqttClient(QObject *parent = nullptr);

    void connectToBroker(const QString &host, quint16 port,
                         const QString &clientId);
    void disconnectFromBroker();
    bool isConnected() const;
    bool publishQos1(const QString &topic, const QByteArray &payload);

signals:
    void connected();
    void disconnected();
    void errorOccurred(const QString &message);

private slots:
    void onTcpConnected();
    void onTcpDisconnected();
    void onReadyRead();
    void sendPing();

private:
    static void appendMqttString(QByteArray &target, const QByteArray &value);
    static QByteArray encodeRemainingLength(qsizetype length);
    bool sendPacket(quint8 header, const QByteArray &body);
    quint16 nextPacketId();
    void processInput();

    QTcpSocket socket_;
    QTimer keepAliveTimer_;
    QByteArray inputBuffer_;
    QString clientId_;
    bool mqttConnected_ = false;
    quint16 packetId_ = 0U;
};
