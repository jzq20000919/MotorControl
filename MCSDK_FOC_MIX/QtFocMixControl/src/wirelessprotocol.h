#pragma once

#include "mqttclient.h"

#include <QElapsedTimer>
#include <QJsonValue>
#include <QObject>
#include <QTimer>

struct FocTelemetry
{
    quint8 protocolVersion = 0;
    quint8 mode = 0;
    quint8 motorState = 0;
    quint16 currentFaults = 0;
    quint16 occurredFaults = 0;
    double iqA = 0.0;
    double idA = 0.0;
    double iqRefA = 0.0;
    double idRefA = 0.0;
    double uqV = 0.0;
    double udV = 0.0;
    qint16 speedReferenceRpm = 0;
    qint16 speedMeasuredRpm = 0;
    double targetDegree = 0.0;
    double currentDegree = 0.0;
    bool uartDiagnosticsAvailable = false;
    bool uartInitialized = false;
    bool uartLinkActive = false;
    quint8 uartInitStage = 0;
    quint8 uartLastTxStatus = 0;
    quint32 uartError = 0;
    quint32 uartReceivedBytes = 0;
    quint32 uartValidCommandFrames = 0;
    quint32 uartCrcErrors = 0;
    quint32 uartProtocolErrors = 0;
    quint32 uartTelemetryAttempts = 0;
    quint32 uartTelemetrySent = 0;
    quint32 uartTelemetryErrors = 0;
};

class WirelessProtocol final : public QObject
{
    Q_OBJECT

public:
    explicit WirelessProtocol(QObject *parent = nullptr);

    bool connectBroker(const QString &host, quint16 port = 1883U);
    void disconnectBroker();
    bool isBrokerOpen() const;
    bool isBrokerConnected() const;
    bool isConnected() const;

    void setMode(bool positionMode);
    void setSpeedRpm(qint16 rpm, quint32 durationMs);
    void setPositionCdeg(qint32 cdeg, quint32 durationMs);
    void startMotor();
    void stopMotor();
    void acknowledgeFault();
    void zeroPosition();

signals:
    void connectionChanged(bool connected, const QString &status);
    void telemetryReceived(const FocTelemetry &telemetry);
    void protocolError(const QString &message);
    void diagnosticMessage(const QString &message);

private slots:
    void onMqttConnected();
    void onMqttDisconnected();
    void onMqttMessage(const QString &topic, const QByteArray &payload);
    void checkTelemetryTimeout();

private:
    void publishCommand(const QString &command,
                        const QJsonValue &value = QJsonValue(),
                        quint32 durationMs = 0U);
    void processTelemetry(const QByteArray &payload);
    void processAcknowledgement(const QByteArray &payload);
    void updateLinkState(bool linkActive, const QString &transportName);

    MqttClient mqtt_;
    QTimer telemetryWatchdog_;
    QElapsedTimer lastTelemetry_;
    quint32 nextCommandId_ = 0U;
    bool gatewayConnected_ = false;
    bool telemetrySeen_ = false;
};

Q_DECLARE_METATYPE(FocTelemetry)
