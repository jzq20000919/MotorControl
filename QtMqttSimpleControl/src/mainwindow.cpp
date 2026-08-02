#include "mainwindow.h"

#include <QCoreApplication>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
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
    setMinimumSize(520, 300);

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

    statusLabel_ = new QLabel(tr("未连接"), central);
    root->addWidget(statusLabel_);

    auto *commandGroup = new QGroupBox(tr("仅发送控制命令"), central);
    auto *form = new QFormLayout(commandGroup);

    auto *speedRow = new QWidget(commandGroup);
    auto *speedLayout = new QHBoxLayout(speedRow);
    speedLayout->setContentsMargins(0, 0, 0, 0);
    speedSpin_ = new QSpinBox(speedRow);
    speedSpin_->setRange(-2600, 2600);
    speedSpin_->setSuffix(tr(" RPM"));
    sendSpeedButton_ = new QPushButton(tr("发送速度"), speedRow);
    speedLayout->addWidget(speedSpin_, 1);
    speedLayout->addWidget(sendSpeedButton_);
    form->addRow(tr("速度设定"), speedRow);

    auto *positionRow = new QWidget(commandGroup);
    auto *positionLayout = new QHBoxLayout(positionRow);
    positionLayout->setContentsMargins(0, 0, 0, 0);
    positionSpin_ = new QDoubleSpinBox(positionRow);
    positionSpin_->setRange(0.0, 359.99);
    positionSpin_->setDecimals(2);
    positionSpin_->setSingleStep(1.0);
    positionSpin_->setSuffix(tr("°"));
    sendPositionButton_ = new QPushButton(tr("发送位置"), positionRow);
    positionLayout->addWidget(positionSpin_, 1);
    positionLayout->addWidget(sendPositionButton_);
    form->addRow(tr("位置设定"), positionRow);
    root->addWidget(commandGroup);

    auto *note = new QLabel(
        tr("本程序只发布 motor/control/command，不订阅或显示电机反馈。"),
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
    connect(&mqtt_, &MqttClient::connected, this, [this] {
        setConnected(true, tr("MQTT Broker 已连接"));
    });
    connect(&mqtt_, &MqttClient::disconnected, this, [this] {
        setConnected(false, tr("MQTT Broker 已断开"));
    });
    connect(&mqtt_, &MqttClient::errorOccurred, this,
            [this](const QString &message) {
                setConnected(false, tr("连接错误：%1").arg(message));
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
        statusLabel_->setText(tr("请输入 Broker IP"));
        return;
    }
    statusLabel_->setText(tr("正在连接…"));
    const QString clientId = QStringLiteral("qt-simple-control-%1")
        .arg(QCoreApplication::applicationPid());
    mqtt_.connectToBroker(
        host, static_cast<quint16>(portSpin_->value()), clientId);
}

void MainWindow::sendSpeed()
{
    const QByteArray payload = makeCommand(
        QStringLiteral("set_speed"), speedSpin_->value());
    if (!mqtt_.publishQos1(kCommandTopic, payload)) {
        statusLabel_->setText(tr("速度命令发送失败"));
    }
}

void MainWindow::sendPosition()
{
    const qint64 positionCdeg = qRound64(positionSpin_->value() * 100.0);
    const QByteArray payload = makeCommand(
        QStringLiteral("set_position"), positionCdeg);
    if (!mqtt_.publishQos1(kCommandTopic, payload)) {
        statusLabel_->setText(tr("位置命令发送失败"));
    }
}

void MainWindow::setConnected(bool connected, const QString &message)
{
    hostEdit_->setEnabled(!connected);
    portSpin_->setEnabled(!connected);
    connectButton_->setText(connected ? tr("断开") : tr("连接"));
    sendSpeedButton_->setEnabled(connected);
    sendPositionButton_->setEnabled(connected);
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
