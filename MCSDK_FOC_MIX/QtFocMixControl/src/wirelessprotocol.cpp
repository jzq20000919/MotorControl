#include "wirelessprotocol.h"

#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>

namespace {
const QString kCommandTopic = QStringLiteral("motor/control/command");
const QString kTelemetryTopic = QStringLiteral("motor/control/telemetry");
const QString kAckTopic = QStringLiteral("motor/control/ack");
const QString kStatusTopic = QStringLiteral("motor/control/status");
constexpr qint64 kTelemetryTimeoutMs = 1500;
}

WirelessProtocol::WirelessProtocol(QObject *parent)
    : QObject(parent)
    , mqtt_(this)
{
    qRegisterMetaType<FocTelemetry>();
    telemetryWatchdog_.setInterval(500);

    connect(&mqtt_, &MqttClient::connected,
            this, &WirelessProtocol::onMqttConnected);
    connect(&mqtt_, &MqttClient::disconnected,
            this, &WirelessProtocol::onMqttDisconnected);
    connect(&mqtt_, &MqttClient::messageReceived,
            this, &WirelessProtocol::onMqttMessage);
    connect(&mqtt_, &MqttClient::errorOccurred,
            this, [this](const QString &message) {
                emit protocolError(message);
            });
    connect(&mqtt_, &MqttClient::diagnosticMessage,
            this, &WirelessProtocol::diagnosticMessage);
    connect(&telemetryWatchdog_, &QTimer::timeout,
            this, &WirelessProtocol::checkTelemetryTimeout);
}

bool WirelessProtocol::connectBroker(const QString &host, quint16 port)
{
    if (host.trimmed().isEmpty() || port == 0U) {
        emit protocolError(tr("Invalid MQTT broker address"));
        return false;
    }
    gatewayConnected_ = false;
    telemetrySeen_ = false;
    telemetryWatchdog_.stop();
    emit connectionChanged(false, tr("Connecting to MQTT broker..."));
    const QString clientId = QStringLiteral("qt-foc-%1")
        .arg(QCoreApplication::applicationPid());
    mqtt_.connectToBroker(host.trimmed(), port, clientId);
    return true;
}

void WirelessProtocol::disconnectBroker()
{
    telemetryWatchdog_.stop();
    gatewayConnected_ = false;
    telemetrySeen_ = false;
    mqtt_.disconnectFromBroker();
    emit connectionChanged(false, tr("Disconnected"));
}

bool WirelessProtocol::isBrokerOpen() const
{
    return mqtt_.isSocketOpen();
}

bool WirelessProtocol::isBrokerConnected() const
{
    return mqtt_.isConnected();
}

bool WirelessProtocol::isConnected() const
{
    return mqtt_.isConnected() && gatewayConnected_;
}

void WirelessProtocol::setMode(bool positionMode)
{
    publishCommand(QStringLiteral("set_mode"), positionMode ? 1 : 0);
}

void WirelessProtocol::setSpeedRpm(qint16 rpm, quint32 durationMs)
{
    publishCommand(QStringLiteral("set_speed"), rpm, durationMs);
}

void WirelessProtocol::setPositionCdeg(qint32 cdeg, quint32 durationMs)
{
    publishCommand(QStringLiteral("set_position"), cdeg, durationMs);
}

void WirelessProtocol::startMotor()
{
    publishCommand(QStringLiteral("start"));
}

void WirelessProtocol::stopMotor()
{
    publishCommand(QStringLiteral("stop"));
}

void WirelessProtocol::acknowledgeFault()
{
    publishCommand(QStringLiteral("ack_fault"));
}

void WirelessProtocol::zeroPosition()
{
    publishCommand(QStringLiteral("zero_position"));
}

void WirelessProtocol::onMqttConnected()
{
    mqtt_.subscribe(kTelemetryTopic, 0U);
    mqtt_.subscribe(kAckTopic, 1U);
    mqtt_.subscribe(kStatusTopic, 1U);
    telemetryWatchdog_.start();
    emit connectionChanged(
        false,
        tr("MQTT connected, waiting for ESP32/STM32 telemetry"));
    publishCommand(QStringLiteral("claim"));
}

void WirelessProtocol::onMqttDisconnected()
{
    telemetryWatchdog_.stop();
    gatewayConnected_ = false;
    telemetrySeen_ = false;
    emit connectionChanged(false, tr("MQTT disconnected"));
}

void WirelessProtocol::onMqttMessage(const QString &topic,
                                     const QByteArray &payload)
{
    if (topic == kTelemetryTopic) {
        processTelemetry(payload);
    } else if (topic == kAckTopic) {
        processAcknowledgement(payload);
    } else if (topic == kStatusTopic) {
        emit diagnosticMessage(
            tr("ESP32 status: %1").arg(QString::fromUtf8(payload)));
    }
}

void WirelessProtocol::checkTelemetryTimeout()
{
    if (!mqtt_.isConnected()) {
        return;
    }
    if (!telemetrySeen_ || !lastTelemetry_.isValid() ||
        lastTelemetry_.elapsed() > kTelemetryTimeoutMs) {
        if (gatewayConnected_ || !telemetrySeen_) {
            gatewayConnected_ = false;
            emit connectionChanged(
                false,
                tr("MQTT online, ESP32 telemetry offline"));
        }
    }
}

void WirelessProtocol::publishCommand(const QString &command,
                                      const QJsonValue &value,
                                      quint32 durationMs)
{
    if (!mqtt_.isConnected()) {
        emit protocolError(tr("MQTT broker is not connected"));
        return;
    }
    QJsonObject object;
    object.insert(QStringLiteral("id"),
                  static_cast<qint64>(++nextCommandId_));
    object.insert(QStringLiteral("cmd"), command);
    if (!value.isUndefined() && !value.isNull()) {
        object.insert(QStringLiteral("value"), value);
    }
    if (durationMs > 0U) {
        object.insert(QStringLiteral("duration_ms"),
                      static_cast<qint64>(durationMs));
    }
    const QByteArray payload =
        QJsonDocument(object).toJson(QJsonDocument::Compact);
    if (!mqtt_.publish(kCommandTopic, payload, 1U, false)) {
        emit protocolError(tr("Failed to publish MQTT command"));
        return;
    }
    emit diagnosticMessage(
        tr("MQTT TX %1: %2")
            .arg(kCommandTopic, QString::fromUtf8(payload)));
}

void WirelessProtocol::processTelemetry(const QByteArray &payload)
{
    QJsonParseError parseError;
    const QJsonDocument document = QJsonDocument::fromJson(payload, &parseError);
    if (parseError.error != QJsonParseError::NoError ||
        !document.isObject()) {
        emit protocolError(tr("Invalid ESP32 telemetry JSON: %1")
                               .arg(parseError.errorString()));
        return;
    }
    const QJsonObject object = document.object();
    FocTelemetry telemetry;
    telemetry.protocolVersion = static_cast<quint8>(
        object.value(QStringLiteral("version")).toInt());
    telemetry.mode = static_cast<quint8>(
        object.value(QStringLiteral("mode")).toInt());
    const bool motorFault =
        object.value(QStringLiteral("motor_fault")).toBool();
    const bool running = object.value(QStringLiteral("running")).toBool();
    telemetry.motorState = motorFault ? 10U : (running ? 6U : 0U);
    telemetry.currentFaults = static_cast<quint16>(
        object.value(QStringLiteral("faults")).toInt());
    telemetry.occurredFaults = telemetry.currentFaults;
    telemetry.iqA = object.value(QStringLiteral("iq_ma")).toInt() / 1000.0;
    telemetry.idA = object.value(QStringLiteral("id_ma")).toInt() / 1000.0;
    telemetry.iqRefA =
        object.value(QStringLiteral("iq_ref_ma")).toInt() / 1000.0;
    telemetry.idRefA =
        object.value(QStringLiteral("id_ref_ma")).toInt() / 1000.0;
    telemetry.uqV = object.value(QStringLiteral("uq_mv")).toInt() / 1000.0;
    telemetry.udV = object.value(QStringLiteral("ud_mv")).toInt() / 1000.0;
    telemetry.speedReferenceRpm = static_cast<qint16>(
        object.value(QStringLiteral("speed_ref_rpm")).toInt());
    telemetry.speedMeasuredRpm = static_cast<qint16>(
        object.value(QStringLiteral("speed_rpm")).toInt());
    telemetry.targetDegree =
        object.value(QStringLiteral("target_cdeg")).toInt() / 100.0;
    telemetry.currentDegree =
        object.value(QStringLiteral("position_cdeg")).toInt() / 100.0;

    const int transport = object.value(QStringLiteral("transport")).toInt();
    telemetry.uartDiagnosticsAvailable = transport == 1;
    telemetry.uartInitialized =
        object.value(QStringLiteral("uart_online")).toBool() || transport == 1;
    telemetry.uartLinkActive =
        object.value(QStringLiteral("uart_online")).toBool();
    telemetry.uartReceivedBytes = static_cast<quint32>(
        object.value(QStringLiteral("rx_frames")).toDouble());
    telemetry.uartValidCommandFrames = telemetry.uartReceivedBytes;
    telemetry.uartTelemetryAttempts = static_cast<quint32>(
        object.value(QStringLiteral("tx_frames")).toDouble());
    telemetry.uartTelemetrySent = telemetry.uartTelemetryAttempts;
    telemetry.uartTelemetryErrors = static_cast<quint32>(
        object.value(QStringLiteral("tx_errors")).toDouble());

    telemetrySeen_ = true;
    lastTelemetry_.restart();
    const bool linkActive =
        object.value(QStringLiteral("link_active")).toBool();
    const QString transportName = transport == 1
        ? QStringLiteral("USART")
        : (transport == 2 ? QStringLiteral("CAN")
                          : QStringLiteral("NONE"));
    updateLinkState(linkActive, transportName);
    emit telemetryReceived(telemetry);
}

void WirelessProtocol::processAcknowledgement(const QByteArray &payload)
{
    const QJsonDocument document = QJsonDocument::fromJson(payload);
    if (!document.isObject()) {
        emit diagnosticMessage(
            tr("MQTT ACK: %1").arg(QString::fromUtf8(payload)));
        return;
    }
    const QJsonObject object = document.object();
    const bool accepted = object.value(QStringLiteral("ok")).toBool();
    const QString message =
        object.value(QStringLiteral("message")).toString();
    const int id = object.value(QStringLiteral("id")).toInt();
    emit diagnosticMessage(
        tr("MQTT ACK #%1: %2").arg(id).arg(message));
    if (!accepted) {
        emit protocolError(tr("ESP32 rejected command #%1: %2")
                               .arg(id).arg(message));
    }
}

void WirelessProtocol::updateLinkState(bool linkActive,
                                       const QString &transportName)
{
    if (gatewayConnected_ == linkActive && telemetrySeen_) {
        return;
    }
    gatewayConnected_ = linkActive;
    emit connectionChanged(
        linkActive,
        linkActive
            ? tr("Wireless control online (%1)").arg(transportName)
            : tr("MQTT online, STM32 link offline"));
}
