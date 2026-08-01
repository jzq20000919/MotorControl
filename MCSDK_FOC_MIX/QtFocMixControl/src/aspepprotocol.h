#pragma once

#include "serialtransport.h"

#include <QByteArray>
#include <QElapsedTimer>
#include <QList>
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

class AspepProtocol final : public QObject
{
    Q_OBJECT

public:
    explicit AspepProtocol(QObject *parent = nullptr);

    bool connectPort(const QString &portName, quint32 baudRate = 1843200);
    void disconnectPort();
    bool isPortOpen() const;
    bool isConnected() const;
    static QStringList availablePorts();

    void requestTelemetry();
    void setMode(bool positionMode);
    void setSpeedRpm(qint16 rpm, quint32 durationMs);
    void setPositionCdeg(qint32 cdeg, quint32 durationMs);
    void startMotor();
    void stopMotor();
    void acknowledgeFault();

signals:
    void connectionChanged(bool connected, const QString &status);
    void telemetryReceived(const FocTelemetry &telemetry);
    void protocolError(const QString &message);
    void diagnosticMessage(const QString &message);

private slots:
    void onBytesReceived(const QByteArray &data);
    void onHandshakeTimeout();
    void onRequestTimeout();

private:
    enum class LinkState { Closed, WaitingBeacon, WaitingPing, Connected };

    static quint32 addHeaderCrc(quint32 header);
    static bool validHeaderCrc(quint32 header);
    static quint32 readU32(const char *data);
    static quint16 readU16(const char *data);
    static qint16 readS16(const char *data);
    static qint32 readS32(const char *data);
    static void appendS16(QByteArray &data, qint16 value);
    static void appendS32(QByteArray &data, qint32 value);
    static void appendU32(QByteArray &data, quint32 value);

    void sendBeacon();
    void sendPing();
    void processFrames();
    void processControlFrame(quint32 header);
    void processDataFrame(const QByteArray &payload);
    void queueCommand(quint8 command, const QByteArray &arguments = {});
    void sendNextCommand();
    void setLinkState(LinkState state, const QString &status);
    void logFrame(const QString &direction, const QByteArray &data);

    SerialTransport serial_;
    QByteArray receiveBuffer_;
    QList<QByteArray> commandQueue_;
    LinkState state_ = LinkState::Closed;
    bool requestOutstanding_ = false;
    QTimer handshakeTimer_;
    QTimer requestTimer_;
    int handshakeAttempts_ = 0;
    bool receivedAnyBytes_ = false;
    quint32 invalidHeaderByteCount_ = 0;
    quint32 linkGeneration_ = 0;
    quint8 outstandingCommandId_ = 0;
    int consecutiveTelemetryTimeouts_ = 0;
    bool telemetryLogAnnounced_ = false;
    bool uartDiagnosticLogAnnounced_ = false;
    QElapsedTimer uartDiagnosticLogTimer_;
};

Q_DECLARE_METATYPE(FocTelemetry)
