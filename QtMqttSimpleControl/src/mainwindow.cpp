#include "mainwindow.h"

#include <QCoreApplication>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QSpinBox>
#include <QVBoxLayout>

namespace {
const QString kCommandTopic = QStringLiteral("motor/control/command");
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , mqtt_(this)
{
    setWindowTitle(tr("MQTT 简易电机控制"));
    setMinimumSize(620, 390);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);

    auto *brokerGroup = new QGroupBox(tr("Broker"), central);
    auto *brokerLayout = new QHBoxLayout(brokerGroup);
    hostEdit_ = new QLineEdit(QStringLiteral("192.168.10.4"), brokerGroup);
    portSpin_ = new QSpinBox(brokerGroup);
    portSpin_->setRange(1, 65535);
    portSpin_->setValue(1883);
    connectButton_ = new QPushButton(tr("连接"), brokerGroup);
    brokerLayout->addWidget(new QLabel(tr("IP"), brokerGroup));
    brokerLayout->addWidget(hostEdit_, 1);
    brokerLayout->addWidget(new QLabel(tr("端口"), brokerGroup));
    brokerLayout->addWidget(portSpin_);
    brokerLayout->addWidget(connectButton_);
    root->addWidget(brokerGroup);

    auto *statusGroup = new QGroupBox(tr("MQTT 状态"), central);
    auto *statusLayout = new QHBoxLayout(statusGroup);
    mqttIndicator_ = new QLabel(QStringLiteral("●"), statusGroup);
    mqttIndicator_->setStyleSheet(QStringLiteral("color:#d32f2f;font-size:20px;"));
    mqttStateLabel_ = new QLabel(tr("MQTT OFFLINE"), statusGroup);
    mqttStateLabel_->setMinimumWidth(120);
    statusLabel_ = new QLabel(tr("未连接"), statusGroup);
    statusLayout->addWidget(mqttIndicator_);
    statusLayout->addWidget(mqttStateLabel_);
    statusLayout->addWidget(statusLabel_, 1);
    root->addWidget(statusGroup);

    auto *commandGroup = new QGroupBox(tr("仅发送控制命令"), central);
    auto *form = new QFormLayout(commandGroup);

    auto *speedRow = new QWidget(commandGroup);
    auto *speedLayout = new QHBoxLayout(speedRow);
    speedLayout->setContentsMargins(0, 0, 0, 0);
    speedSlider_ = new QSlider(Qt::Horizontal, speedRow);
    speedSlider_->setRange(-2600, 2600);
    speedSlider_->setSingleStep(100);
    speedSlider_->setPageStep(500);
    speedSlider_->setValue(0);
    speedValueLabel_ = new QLabel(tr("0 RPM"), speedRow);
    speedValueLabel_->setMinimumWidth(82);
    speedValueLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    sendSpeedButton_ = new QPushButton(tr("发送速度"), speedRow);
    speedLayout->addWidget(speedSlider_, 1);
    speedLayout->addWidget(speedValueLabel_);
    speedLayout->addWidget(sendSpeedButton_);
    form->addRow(tr("速度设定"), speedRow);

    auto *positionRow = new QWidget(commandGroup);
    auto *positionLayout = new QHBoxLayout(positionRow);
    positionLayout->setContentsMargins(0, 0, 0, 0);
    positionSlider_ = new QSlider(Qt::Horizontal, positionRow);
    positionSlider_->setRange(0, 35999);
    positionSlider_->setSingleStep(100);
    positionSlider_->setPageStep(1000);
    positionSlider_->setValue(0);
    positionValueLabel_ = new QLabel(tr("0.00°"), positionRow);
    positionValueLabel_->setMinimumWidth(82);
    positionValueLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    sendPositionButton_ = new QPushButton(tr("发送位置"), positionRow);
    positionLayout->addWidget(positionSlider_, 1);
    positionLayout->addWidget(positionValueLabel_);
    positionLayout->addWidget(sendPositionButton_);
    form->addRow(tr("位置设定"), positionRow);

    auto *runRow = new QWidget(commandGroup);
    auto *runLayout = new QHBoxLayout(runRow);
    runLayout->setContentsMargins(0, 0, 0, 0);
    startButton_ = new QPushButton(tr("启动电机"), runRow);
    stopButton_ = new QPushButton(tr("停止电机"), runRow);
    startButton_->setStyleSheet(QStringLiteral(
        "QPushButton{background:#2e7d32;color:white;padding:7px;}"));
    stopButton_->setStyleSheet(QStringLiteral(
        "QPushButton{background:#c62828;color:white;padding:7px;}"));
    runLayout->addWidget(startButton_);
    runLayout->addWidget(stopButton_);
    form->addRow(tr("电机启停"), runRow);
    root->addWidget(commandGroup);

    auto *note = new QLabel(
        tr("状态灯表示本程序与 MQTT Broker 的连接状态；本程序只发布控制命令，不显示电机遥测。"),
        central);
    note->setWordWrap(true);
    root->addWidget(note);
    root->addStretch();
    setCentralWidget(central);

    connect(connectButton_, &QPushButton::clicked,
            this, &MainWindow::toggleConnection);
    connect(sendSpeedButton_, &QPushButton::clicked,
            this, &MainWindow::sendSpeed);
    connect(sendPositionButton_, &QPushButton::clicked,
            this, &MainWindow::sendPosition);
    connect(startButton_, &QPushButton::clicked,
            this, &MainWindow::startMotor);
    connect(stopButton_, &QPushButton::clicked,
            this, &MainWindow::stopMotor);
    connect(speedSlider_, &QSlider::valueChanged, this, [this](int value) {
        speedValueLabel_->setText(tr("%1 RPM").arg(value));
    });
    connect(positionSlider_, &QSlider::valueChanged, this, [this](int value) {
        positionValueLabel_->setText(
            tr("%1°").arg(value / 100.0, 0, 'f', 2));
    });
    connect(&mqtt_, &MqttClient::connected, this, [this] {
        setConnected(true, tr("MQTT Broker 已连接"));
    });
    connect(&mqtt_, &MqttClient::disconnected, this, [this] {
        setConnected(false, tr("MQTT Broker 已断开"));
    });
    connect(&mqtt_, &MqttClient::errorOccurred, this,
            [this](const QString &message) {
                setConnected(false, tr("连接错误：%1").arg(message));
                showMqttStatus(
                    QStringLiteral("MQTT ERROR"),
                    QStringLiteral("#d32f2f"),
                    tr("连接错误：%1").arg(message));
            });

    setConnected(false, tr("未连接"));
}

void MainWindow::toggleConnection()
{
    if (mqtt_.isConnected()) {
        mqtt_.disconnectFromBroker();
        setConnected(false, tr("已断开"));
        return;
    }

    const QString host = hostEdit_->text().trimmed();
    if (host.isEmpty()) {
        showMqttStatus(QStringLiteral("MQTT ERROR"),
                       QStringLiteral("#d32f2f"), tr("请输入 Broker IP"));
        return;
    }
    showMqttStatus(QStringLiteral("MQTT CONNECTING"),
                   QStringLiteral("#f9a825"), tr("正在连接…"));
    const QString clientId = QStringLiteral("qt-simple-control-%1")
        .arg(QCoreApplication::applicationPid());
    mqtt_.connectToBroker(
        host, static_cast<quint16>(portSpin_->value()), clientId);
}

void MainWindow::sendSpeed()
{
    publishCommand(QStringLiteral("set_speed"), speedSlider_->value(),
                   tr("速度命令"));
}

void MainWindow::sendPosition()
{
    publishCommand(QStringLiteral("set_position"), positionSlider_->value(),
                   tr("位置命令"));
}

void MainWindow::startMotor()
{
    publishCommand(QStringLiteral("start"), 0, tr("启动命令"));
}

void MainWindow::stopMotor()
{
    publishCommand(QStringLiteral("stop"), 0, tr("停止命令"));
}

void MainWindow::setConnected(bool connected, const QString &message)
{
    hostEdit_->setEnabled(!connected);
    portSpin_->setEnabled(!connected);
    connectButton_->setText(connected ? tr("断开") : tr("连接"));
    sendSpeedButton_->setEnabled(connected);
    sendPositionButton_->setEnabled(connected);
    startButton_->setEnabled(connected);
    stopButton_->setEnabled(connected);
    showMqttStatus(
        connected ? QStringLiteral("MQTT ONLINE")
                  : QStringLiteral("MQTT OFFLINE"),
        connected ? QStringLiteral("#2e7d32")
                  : QStringLiteral("#d32f2f"),
        message);
}

bool MainWindow::publishCommand(const QString &command, qint64 value,
                                const QString &description)
{
    const QByteArray payload = makeCommand(command, value);
    if (!mqtt_.publishQos1(kCommandTopic, payload)) {
        statusLabel_->setText(tr("%1发送失败").arg(description));
        return false;
    }
    statusLabel_->setText(tr("%1已发送").arg(description));
    return true;
}

void MainWindow::showMqttStatus(const QString &state, const QString &color,
                                const QString &message)
{
    mqttIndicator_->setStyleSheet(
        QStringLiteral("color:%1;font-size:20px;").arg(color));
    mqttStateLabel_->setText(state);
    statusLabel_->setText(message);
}

QByteArray MainWindow::makeCommand(const QString &command, qint64 value)
{
    QJsonObject object;
    object.insert(QStringLiteral("id"),
                  static_cast<qint64>(++nextCommandId_));
    object.insert(QStringLiteral("cmd"), command);
    object.insert(QStringLiteral("value"), value);
    return QJsonDocument(object).toJson(QJsonDocument::Compact);
}
