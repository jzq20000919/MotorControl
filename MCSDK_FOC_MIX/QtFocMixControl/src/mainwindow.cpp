#include "mainwindow.h"

#include "positiondial.h"

#include <QButtonGroup>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRadioButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStatusBar>
#include <QTimer>
#include <QTextBlock>
#include <QTime>
#include <QVBoxLayout>

#include <cmath>

namespace {
constexpr quint16 kFaultDuration = 0x0001U;
constexpr quint16 kFaultOverVoltage = 0x0002U;
constexpr quint16 kFaultUnderVoltage = 0x0004U;
constexpr quint16 kFaultOverTemperature = 0x0008U;
constexpr quint16 kFaultStartup = 0x0010U;
constexpr quint16 kFaultSpeedFeedback = 0x0020U;
constexpr quint16 kFaultOverCurrent = 0x0040U;
constexpr quint16 kFaultSoftware = 0x0080U;
constexpr quint16 kFaultDriverProtection = 0x0400U;

double normalizeDegree(double degree)
{
    double normalized = std::fmod(degree, 360.0);
    if (normalized < 0.0) {
        normalized += 360.0;
    }
    return normalized;
}

double nearestMultiTurnTarget(double singleTurnTarget, double currentMultiTurn)
{
    return currentMultiTurn + std::remainder(singleTurnTarget - currentMultiTurn, 360.0);
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , protocol_(this)
{
    setWindowTitle(tr("MCSDK FOC 位置 / 转速控制台"));
    resize(1180, 940);
    setMinimumSize(980, 820);

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(22, 18, 22, 22);
    root->setSpacing(14);
    root->addWidget(createHeader());
    root->addWidget(createModeAndActions());

    auto *content = new QHBoxLayout;
    content->setSpacing(14);
    content->addWidget(createTelemetryPanel(), 3);
    content->addWidget(createControlPanel(), 4);
    root->addLayout(content, 1);
    root->addWidget(createSerialLogPanel());
    setCentralWidget(central);

    telemetryTimer_ = new QTimer(this);
    telemetryTimer_->setInterval(50);

    connect(telemetryTimer_, &QTimer::timeout,
            &protocol_, &AspepProtocol::requestTelemetry);
    connect(&protocol_, &AspepProtocol::connectionChanged,
            this, &MainWindow::onConnectionChanged);
    connect(&protocol_, &AspepProtocol::telemetryReceived,
            this, &MainWindow::onTelemetry);
    connect(&protocol_, &AspepProtocol::protocolError,
            this, &MainWindow::onProtocolError);
    connect(&protocol_, &AspepProtocol::diagnosticMessage,
            this, &MainWindow::appendSerialLog);

    refreshPorts();
    setControlsEnabled(false);
    statusBar()->showMessage(tr("请选择串口并连接"));

    setStyleSheet(QStringLiteral(R"(
        QMainWindow, QWidget#centralWidget { background: #0b111b; color: #dbe7f7; }
        QWidget { font-family: "Microsoft YaHei UI"; font-size: 10pt; }
        QFrame#panel { background: #121c2a; border: 1px solid #22334a; border-radius: 12px; }
        QFrame#card { background: #172334; border: 1px solid #253951; border-radius: 9px; }
        QLabel#title { color: #f3f7fd; font-size: 17pt; font-weight: 700; }
        QLabel#subtitle { color: #7186a3; font-size: 9pt; }
        QLabel#cardTitle { color: #7f94af; font-size: 9pt; }
        QLabel#cardValue { color: #f4f8ff; font-size: 19pt; font-weight: 700; }
        QLabel#unit { color: #647a96; font-size: 8pt; }
        QPushButton, QComboBox, QSpinBox, QDoubleSpinBox {
            background: #1b2a3e; color: #dce8f7; border: 1px solid #304766;
            border-radius: 7px; padding: 7px 12px;
        }
        QPushButton:hover { border-color: #4aaed8; background: #20334b; }
        QPushButton:pressed { background: #152338; }
        QPushButton:disabled { color: #53647a; border-color: #263548; }
        QPushButton#primary { background: #087aa5; border-color: #18a4d3; font-weight: 700; }
        QPushButton#danger { background: #7b2d39; border-color: #b14b59; }
        QRadioButton {
            background: #172438; border: 1px solid #2b405c; border-radius: 8px;
            padding: 9px 18px; font-weight: 600;
        }
        QRadioButton::indicator { width: 0; height: 0; }
        QRadioButton:checked { background: #0f789b; border-color: #42c8ed; color: white; }
        QSlider::groove:horizontal { height: 7px; background: #253951; border-radius: 3px; }
        QSlider::sub-page:horizontal { background: #25a9d6; border-radius: 3px; }
        QSlider::handle:horizontal {
            width: 20px; margin: -7px 0; border-radius: 10px;
            background: #dff7ff; border: 2px solid #27b8e6;
        }
        QStatusBar { color: #7890ad; }
    )"));
    central->setObjectName(QStringLiteral("centralWidget"));
}

QWidget *MainWindow::createHeader()
{
    auto *panel = new QFrame;
    panel->setObjectName(QStringLiteral("panel"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(18, 13, 18, 13);

    auto *titles = new QVBoxLayout;
    auto *title = new QLabel(tr("FOC Motion Console"));
    title->setObjectName(QStringLiteral("title"));
    auto *subtitle = new QLabel(tr("STM32 MCSDK · 编码器位置环 · ASPEP/MCP"));
    subtitle->setObjectName(QStringLiteral("subtitle"));
    titles->addWidget(title);
    titles->addWidget(subtitle);
    layout->addLayout(titles);
    layout->addStretch();

    connectionDot_ = new QLabel(QStringLiteral("●"));
    connectionDot_->setStyleSheet(QStringLiteral("color:#566579;font-size:14pt"));
    connectionText_ = new QLabel(tr("未连接"));
    layout->addWidget(connectionDot_);
    layout->addWidget(connectionText_);

    portCombo_ = new QComboBox;
    portCombo_->setMinimumWidth(105);
    baudCombo_ = new QComboBox;
    baudCombo_->setEditable(true);
    baudCombo_->setMinimumWidth(115);
    const QList<quint32> baudRates = {
        115200U, 230400U, 460800U, 921600U, 1000000U,
        1500000U, 1843200U, 2000000U
    };
    for (quint32 baudRate : baudRates) {
        baudCombo_->addItem(QString::number(baudRate), baudRate);
    }
    baudCombo_->setCurrentIndex(baudCombo_->findData(1843200U));
    baudCombo_->setToolTip(tr("必须与 STM32 固件 USART2 波特率一致；当前工程默认 1,843,200"));
    auto *refreshButton = new QPushButton(tr("刷新"));
    connectButton_ = new QPushButton(tr("连接"));
    connectButton_->setObjectName(QStringLiteral("primary"));
    layout->addSpacing(12);
    layout->addWidget(portCombo_);
    layout->addWidget(baudCombo_);
    layout->addWidget(refreshButton);
    layout->addWidget(connectButton_);

    connect(refreshButton, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    connect(connectButton_, &QPushButton::clicked, this, &MainWindow::toggleConnection);
    return panel;
}

QWidget *MainWindow::createModeAndActions()
{
    auto *panel = new QFrame;
    panel->setObjectName(QStringLiteral("panel"));
    auto *layout = new QHBoxLayout(panel);
    layout->setContentsMargins(16, 11, 16, 11);

    layout->addWidget(new QLabel(tr("控制模式")));
    speedModeButton_ = new QRadioButton(tr("转速控制"));
    positionModeButton_ = new QRadioButton(tr("位置控制"));
    positionModeButton_->setChecked(true);
    auto *modeGroup = new QButtonGroup(panel);
    modeGroup->setExclusive(true);
    modeGroup->addButton(speedModeButton_);
    modeGroup->addButton(positionModeButton_);
    layout->addWidget(speedModeButton_);
    layout->addWidget(positionModeButton_);
    layout->addStretch();

    auto *startButton = new QPushButton(tr("启动电机"));
    startButton->setObjectName(QStringLiteral("primary"));
    auto *stopButton = new QPushButton(tr("停止"));
    stopButton->setObjectName(QStringLiteral("danger"));
    auto *faultButton = new QPushButton(tr("故障复位"));
    auto *zeroButton = new QPushButton(tr("转到 0°"));
    for (QPushButton *button : {startButton, stopButton, faultButton, zeroButton}) {
        button->setProperty("requiresConnection", true);
        layout->addWidget(button);
    }

    connect(speedModeButton_, &QRadioButton::clicked,
            this, &MainWindow::onModeChanged);
    connect(positionModeButton_, &QRadioButton::clicked,
            this, &MainWindow::onModeChanged);
    connect(startButton, &QPushButton::clicked, &protocol_, &AspepProtocol::startMotor);
    connect(stopButton, &QPushButton::clicked, &protocol_, &AspepProtocol::stopMotor);
    connect(faultButton, &QPushButton::clicked,
            &protocol_, &AspepProtocol::acknowledgeFault);
    connect(zeroButton, &QPushButton::clicked, this, [this] {
        positionTargetSpin_->setValue(0.0);
        submitPositionTarget();
    });
    return panel;
}

QWidget *MainWindow::createTelemetryPanel()
{
    auto *panel = new QFrame;
    panel->setObjectName(QStringLiteral("panel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(15, 15, 15, 15);
    auto *heading = new QLabel(tr("实时遥测"));
    heading->setStyleSheet(QStringLiteral("font-size:13pt;font-weight:700"));
    layout->addWidget(heading);

    auto *grid = new QGridLayout;
    grid->setSpacing(10);
    grid->addWidget(createValueCard(QStringLiteral("iq"), tr("Iq 测量"), tr("A"),
                                    QStringLiteral("#40d1ff")), 0, 0);
    grid->addWidget(createValueCard(QStringLiteral("id"), tr("Id 测量"), tr("A"),
                                    QStringLiteral("#40d1ff")), 0, 1);
    grid->addWidget(createValueCard(QStringLiteral("iqref"), tr("Iq Ref"), tr("A"),
                                    QStringLiteral("#ffb454")), 1, 0);
    grid->addWidget(createValueCard(QStringLiteral("idref"), tr("Id Ref"), tr("A"),
                                    QStringLiteral("#ffb454")), 1, 1);
    grid->addWidget(createValueCard(QStringLiteral("uq"), tr("Uq"), tr("V"),
                                    QStringLiteral("#a78bfa")), 2, 0);
    grid->addWidget(createValueCard(QStringLiteral("ud"), tr("Ud"), tr("V"),
                                    QStringLiteral("#a78bfa")), 2, 1);
    grid->addWidget(createValueCard(QStringLiteral("speedref"), tr("转速 Ref"), tr("RPM"),
                                    QStringLiteral("#45e0a8")), 3, 0);
    grid->addWidget(createValueCard(QStringLiteral("speed"), tr("转速测量"), tr("RPM"),
                                    QStringLiteral("#45e0a8")), 3, 1);
    grid->addWidget(createValueCard(QStringLiteral("state"), tr("电机状态"), QString(),
                                    QStringLiteral("#f2f6fc")), 4, 0);
    grid->addWidget(createValueCard(QStringLiteral("fault"), tr("当前 / 历史故障字"), QString(),
                                    QStringLiteral("#ff6f7d")), 4, 1);
    layout->addLayout(grid);
    layout->addWidget(createFaultIndicatorPanel());
    layout->addStretch();
    return panel;
}

QWidget *MainWindow::createFaultIndicatorPanel()
{
    struct FaultDescription
    {
        quint16 bit;
        const char *name;
    };

    static constexpr FaultDescription faults[] = {
        {kFaultDriverProtection, "BKIN / 驱动保护"},
        {kFaultOverCurrent, "过流"},
        {kFaultOverVoltage, "母线过压"},
        {kFaultUnderVoltage, "母线欠压"},
        {kFaultOverTemperature, "过温"},
        {kFaultSpeedFeedback, "速度反馈"},
        {kFaultDuration, "FOC 超时"},
        {kFaultStartup, "启动失败"},
        {kFaultSoftware, "软件故障"}
    };

    auto *panel = new QFrame;
    panel->setObjectName(QStringLiteral("faultPanel"));
    panel->setStyleSheet(QStringLiteral(
        "QFrame#faultPanel {"
        " background:#0e1723; border:1px solid #253951; border-radius:8px;"
        "}"));

    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(5);

    auto *header = new QHBoxLayout;
    auto *title = new QLabel(tr("保护与异常指示"));
    title->setStyleSheet(QStringLiteral("font-weight:700;color:#b9c9dc"));
    auto *legend = new QLabel(tr("红=当前  黄=历史  灰=正常"));
    legend->setStyleSheet(QStringLiteral("color:#71839a;font-size:8pt"));
    header->addWidget(title);
    header->addStretch();
    header->addWidget(legend);
    layout->addLayout(header);

    auto *grid = new QGridLayout;
    grid->setHorizontalSpacing(8);
    grid->setVerticalSpacing(3);
    const qsizetype faultCount =
        static_cast<qsizetype>(sizeof(faults) / sizeof(faults[0]));
    for (qsizetype index = 0; index < faultCount; ++index) {
        const FaultDescription &fault = faults[index];
        auto *indicator = new QLabel(
            QStringLiteral("●  %1").arg(QString::fromUtf8(fault.name)));
        indicator->setStyleSheet(
            QStringLiteral("color:#53647a;padding:2px 4px"));
        indicator->setToolTip(
            tr("红色：当前正在触发；黄色：曾触发但当前已解除；灰色：未检测到"));
        faultIndicators_.insert(fault.bit, indicator);
        grid->addWidget(indicator,
                        static_cast<int>(index / 2),
                        static_cast<int>(index % 2));
    }
    layout->addLayout(grid);
    return panel;
}

QWidget *MainWindow::createControlPanel()
{
    auto *panel = new QFrame;
    panel->setObjectName(QStringLiteral("panel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(18, 15, 18, 18);

    auto *heading = new QLabel(tr("目标控制"));
    heading->setStyleSheet(QStringLiteral("font-size:13pt;font-weight:700"));
    layout->addWidget(heading);

    controlStack_ = new QStackedWidget;

    auto *speedPage = new QWidget;
    auto *speedLayout = new QVBoxLayout(speedPage);
    speedLayout->setContentsMargins(12, 38, 12, 20);
    speedLayout->addStretch();
    auto *speedValue = new QLabel(tr("速度给定"));
    speedValue->setAlignment(Qt::AlignCenter);
    speedValue->setStyleSheet(QStringLiteral("color:#8296b1;font-size:10pt"));
    speedLayout->addWidget(speedValue);

    speedSpin_ = new QSpinBox;
    speedSpin_->setRange(-2600, 2600);
    speedSpin_->setKeyboardTracking(false);
    speedSpin_->setSuffix(tr(" RPM"));
    speedSpin_->setAlignment(Qt::AlignCenter);
    speedSpin_->setMinimumHeight(48);
    speedSpin_->setStyleSheet(QStringLiteral("font-size:18pt;font-weight:700"));
    speedLayout->addWidget(speedSpin_, 0, Qt::AlignHCenter);

    auto *speedDurationRow = new QHBoxLayout;
    speedDurationRow->addStretch();
    speedDurationRow->addWidget(new QLabel(tr("速度斜坡时间")));
    speedDurationSpin_ = new QDoubleSpinBox;
    speedDurationSpin_->setRange(0.0, 60.0);
    speedDurationSpin_->setDecimals(2);
    speedDurationSpin_->setValue(0.5);
    speedDurationSpin_->setSuffix(tr(" s"));
    speedDurationSpin_->setMinimumWidth(120);
    speedDurationRow->addWidget(speedDurationSpin_);
    speedDurationRow->addStretch();
    speedLayout->addLayout(speedDurationRow);

    speedSlider_ = new QSlider(Qt::Horizontal);
    speedSlider_->setRange(-2600, 2600);
    speedSlider_->setTickPosition(QSlider::TicksBelow);
    speedSlider_->setTickInterval(650);
    speedLayout->addSpacing(30);
    speedLayout->addWidget(speedSlider_);
    auto *range = new QHBoxLayout;
    range->addWidget(new QLabel(QStringLiteral("−2600")));
    range->addStretch();
    range->addWidget(new QLabel(QStringLiteral("0 RPM")));
    range->addStretch();
    range->addWidget(new QLabel(QStringLiteral("+2600")));
    speedLayout->addLayout(range);
    auto *speedZero = new QPushButton(tr("回零速"));
    auto *speedApply = new QPushButton(tr("应用转速"));
    speedApply->setObjectName(QStringLiteral("primary"));
    auto *speedButtons = new QHBoxLayout;
    speedButtons->addStretch();
    speedButtons->addWidget(speedZero);
    speedButtons->addWidget(speedApply);
    speedButtons->addStretch();
    speedLayout->addSpacing(26);
    speedLayout->addLayout(speedButtons);
    speedLayout->addStretch();

    auto *positionPage = new QWidget;
    auto *positionLayout = new QVBoxLayout(positionPage);
    positionDial_ = new PositionDial;
    positionLayout->addWidget(positionDial_, 1, Qt::AlignCenter);
    auto *positionValues = new QHBoxLayout;
    positionCurrentLabel_ = new QLabel(tr("当前位置  0.00°"));
    positionTargetLabel_ = new QLabel(tr("目标位置  0.00°"));
    positionCurrentLabel_->setStyleSheet(QStringLiteral("color:#45d1ff;font-weight:700"));
    positionTargetLabel_->setStyleSheet(QStringLiteral("color:#ffad42;font-weight:700"));
    positionValues->addStretch();
    positionValues->addWidget(positionCurrentLabel_);
    positionValues->addSpacing(30);
    positionValues->addWidget(positionTargetLabel_);
    positionValues->addStretch();
    positionLayout->addLayout(positionValues);

    auto *positionInput = new QHBoxLayout;
    positionInput->addStretch();
    positionInput->addWidget(new QLabel(tr("输入目标角度")));
    positionTargetSpin_ = new QDoubleSpinBox;
    positionTargetSpin_->setRange(0.0, 359.99);
    positionTargetSpin_->setDecimals(2);
    positionTargetSpin_->setSingleStep(1.0);
    positionTargetSpin_->setSuffix(tr("°"));
    positionTargetSpin_->setWrapping(true);
    positionTargetSpin_->setKeyboardTracking(false);
    positionTargetSpin_->setAlignment(Qt::AlignCenter);
    positionTargetSpin_->setMinimumWidth(145);
    auto *moveToAngleButton = new QPushButton(tr("转到该角度"));
    moveToAngleButton->setObjectName(QStringLiteral("primary"));
    positionInput->addWidget(positionTargetSpin_);
    positionInput->addWidget(new QLabel(tr("运动时间")));
    positionDurationSpin_ = new QDoubleSpinBox;
    positionDurationSpin_->setRange(0.1, 120.0);
    positionDurationSpin_->setDecimals(2);
    positionDurationSpin_->setValue(1.0);
    positionDurationSpin_->setSuffix(tr(" s"));
    positionDurationSpin_->setMinimumWidth(105);
    positionInput->addWidget(positionDurationSpin_);
    positionInput->addWidget(moveToAngleButton);
    positionInput->addStretch();
    positionLayout->addLayout(positionInput);

    auto *hint = new QLabel(tr("点击或拖动圆盘，松开鼠标立即提交；数字输入后点击“转到该角度”"));
    hint->setAlignment(Qt::AlignCenter);
    hint->setStyleSheet(QStringLiteral("color:#687e9a"));
    positionLayout->addWidget(hint);

    controlStack_->addWidget(speedPage);
    controlStack_->addWidget(positionPage);
    controlStack_->setCurrentIndex(1);
    layout->addWidget(controlStack_, 1);

    connect(speedSlider_, &QSlider::valueChanged, speedSpin_, &QSpinBox::setValue);
    connect(speedSpin_, &QSpinBox::valueChanged, speedSlider_, &QSlider::setValue);
    connect(speedSpin_, &QSpinBox::valueChanged, this, &MainWindow::onSpeedChanged);
    connect(speedSlider_, &QSlider::sliderReleased, this, &MainWindow::submitSpeedTarget);
    connect(speedApply, &QPushButton::clicked, this, &MainWindow::submitSpeedTarget);
    connect(speedZero, &QPushButton::clicked, this, [this] {
        speedSpin_->setValue(0);
        submitSpeedTarget();
    });
    connect(positionDial_, &PositionDial::targetChanged,
            this, &MainWindow::onPositionTargetChanged);
    connect(positionDial_, &PositionDial::interactionFinished,
            this, [this](double) { submitPositionTarget(); });
    connect(positionTargetSpin_, &QDoubleSpinBox::valueChanged,
            this, &MainWindow::onPositionTargetChanged);
    connect(moveToAngleButton, &QPushButton::clicked,
            this, &MainWindow::submitPositionTarget);
    return panel;
}

QWidget *MainWindow::createSerialLogPanel()
{
    auto *panel = new QFrame;
    panel->setObjectName(QStringLiteral("panel"));
    auto *layout = new QVBoxLayout(panel);
    layout->setContentsMargins(15, 11, 15, 13);

    auto *header = new QHBoxLayout;
    auto *heading = new QLabel(tr("串口输出 / 协议诊断"));
    heading->setStyleSheet(QStringLiteral("font-size:11pt;font-weight:700"));
    auto *clearButton = new QPushButton(tr("清空信息"));
    header->addWidget(heading);
    header->addStretch();
    header->addWidget(clearButton);
    layout->addLayout(header);

    serialLog_ = new QPlainTextEdit;
    serialLog_->setReadOnly(true);
    serialLog_->setMaximumBlockCount(2000);
    serialLog_->setMinimumHeight(145);
    serialLog_->setPlaceholderText(
        tr("连接后将在这里显示串口参数、TX/RX 原始字节、ASPEP 握手和 MCP 响应。"));
    serialLog_->setStyleSheet(QStringLiteral(
        "QPlainTextEdit {"
        " background:#080d14; color:#9fb4cc; border:1px solid #263950;"
        " border-radius:7px; padding:7px; font-family:Consolas,monospace;"
        " font-size:9pt;"
        "}"));
    layout->addWidget(serialLog_);

    connect(clearButton, &QPushButton::clicked, serialLog_, &QPlainTextEdit::clear);
    return panel;
}

QWidget *MainWindow::createValueCard(const QString &key, const QString &title,
                                     const QString &unit, const QString &color)
{
    auto *card = new QFrame;
    card->setObjectName(QStringLiteral("card"));
    auto *layout = new QVBoxLayout(card);
    layout->setContentsMargins(12, 9, 12, 9);
    auto *titleLabel = new QLabel(title);
    titleLabel->setObjectName(QStringLiteral("cardTitle"));
    auto *valueLabel = new QLabel(QStringLiteral("—"));
    valueLabel->setObjectName(QStringLiteral("cardValue"));
    valueLabel->setStyleSheet(QStringLiteral("color:%1").arg(color));
    auto *unitLabel = new QLabel(unit);
    unitLabel->setObjectName(QStringLiteral("unit"));
    layout->addWidget(titleLabel);
    auto *line = new QHBoxLayout;
    line->addWidget(valueLabel);
    line->addStretch();
    line->addWidget(unitLabel, 0, Qt::AlignBottom);
    layout->addLayout(line);
    values_.insert(key, valueLabel);
    return card;
}

void MainWindow::refreshPorts()
{
    const QString previous = portCombo_->currentText();
    portCombo_->clear();
    portCombo_->addItems(AspepProtocol::availablePorts());
    const int previousIndex = portCombo_->findText(previous);
    if (previousIndex >= 0) {
        portCombo_->setCurrentIndex(previousIndex);
    }
}

void MainWindow::toggleConnection()
{
    if (protocol_.isPortOpen()) {
        protocol_.disconnectPort();
        return;
    }

    if (portCombo_->currentText().isEmpty()) {
        QMessageBox::information(this, tr("没有串口"), tr("未检测到可用串口，请连接设备后刷新。"));
        return;
    }
    bool baudOk = false;
    const quint32 baudRate = baudCombo_->currentText().toUInt(&baudOk);
    if (!baudOk || baudRate == 0U) {
        QMessageBox::warning(this, tr("波特率错误"), tr("请输入有效的正整数波特率。"));
        return;
    }
    protocol_.connectPort(portCombo_->currentText(), baudRate);
}

void MainWindow::onConnectionChanged(bool connected, const QString &status)
{
    if (connected) {
        connectionDot_->setStyleSheet(QStringLiteral("color:#40dda4;font-size:14pt"));
    } else if (protocol_.isPortOpen()) {
        connectionDot_->setStyleSheet(QStringLiteral("color:#ffb454;font-size:14pt"));
    } else {
        connectionDot_->setStyleSheet(QStringLiteral("color:#566579;font-size:14pt"));
    }
    connectionText_->setText(status);
    connectButton_->setText(protocol_.isPortOpen() ? tr("断开") : tr("连接"));
    portCombo_->setEnabled(!protocol_.isPortOpen());
    baudCombo_->setEnabled(!protocol_.isPortOpen());
    setControlsEnabled(connected);
    if (connected) {
        telemetryTimer_->start();
    } else {
        telemetryTimer_->stop();
        updateFaultIndicators(0U, 0U);
        speedTargetLocallySet_ = false;
        positionTargetLocallySet_ = false;
    }
    statusBar()->showMessage(status);
}

void MainWindow::onTelemetry(const FocTelemetry &telemetry)
{
    applyingTelemetry_ = true;
    setValue(QStringLiteral("iq"), QString::number(telemetry.iqA, 'f', 3));
    setValue(QStringLiteral("id"), QString::number(telemetry.idA, 'f', 3));
    setValue(QStringLiteral("iqref"), QString::number(telemetry.iqRefA, 'f', 3));
    setValue(QStringLiteral("idref"), QString::number(telemetry.idRefA, 'f', 3));
    setValue(QStringLiteral("uq"), QString::number(telemetry.uqV, 'f', 2));
    setValue(QStringLiteral("ud"), QString::number(telemetry.udV, 'f', 2));
    setValue(QStringLiteral("speedref"), QString::number(telemetry.speedReferenceRpm));
    setValue(QStringLiteral("speed"), QString::number(telemetry.speedMeasuredRpm));
    setValue(QStringLiteral("state"), motorStateName(telemetry.motorState));
    setValue(QStringLiteral("fault"),
             QStringLiteral("0x%1 / 0x%2")
                 .arg(telemetry.currentFaults, 4, 16, QLatin1Char('0'))
                 .arg(telemetry.occurredFaults, 4, 16, QLatin1Char('0')));
    updateFaultIndicators(telemetry.currentFaults, telemetry.occurredFaults);

    speedModeButton_->setChecked(telemetry.mode == 0);
    positionModeButton_->setChecked(telemetry.mode == 1);
    controlStack_->setCurrentIndex(telemetry.mode == 1 ? 1 : 0);
    if ((telemetry.mode == 0) && !speedTargetLocallySet_ &&
        !speedSlider_->isSliderDown() && !speedSpin_->hasFocus()) {
        speedSpin_->setValue(telemetry.speedReferenceRpm);
    }
    currentMultiTurnDegree_ = telemetry.currentDegree;
    const double currentSingleTurn = normalizeDegree(telemetry.currentDegree);
    const double targetSingleTurn = normalizeDegree(telemetry.targetDegree);
    positionDial_->setCurrentDegree(currentSingleTurn);
    positionCurrentLabel_->setText(tr("当前位置  %1°").arg(currentSingleTurn, 0, 'f', 2));
    if (!positionTargetLocallySet_ && !positionDial_->isDragging() &&
        !positionTargetSpin_->hasFocus()) {
        const QSignalBlocker targetBlocker(positionTargetSpin_);
        positionTargetSpin_->setValue(targetSingleTurn);
        positionDial_->setTargetDegree(targetSingleTurn);
        positionTargetLabel_->setText(
            tr("目标位置  %1°").arg(targetSingleTurn, 0, 'f', 2));
    }
    applyingTelemetry_ = false;
}

void MainWindow::onProtocolError(const QString &message)
{
    statusBar()->showMessage(message, 5000);
    appendSerialLog(tr("错误：%1").arg(message));
}

void MainWindow::appendSerialLog(const QString &message)
{
    if (serialLog_ == nullptr) {
        return;
    }
    serialLog_->appendPlainText(
        QStringLiteral("[%1] %2")
            .arg(QTime::currentTime().toString(QStringLiteral("HH:mm:ss.zzz")),
                 message));
}

void MainWindow::onModeChanged()
{
    const bool positionMode = positionModeButton_->isChecked();
    controlStack_->setCurrentIndex(positionMode ? 1 : 0);

    if (positionMode) {
        positionTargetLocallySet_ = false;
        const double currentSingleTurn = normalizeDegree(currentMultiTurnDegree_);
        const QSignalBlocker targetBlocker(positionTargetSpin_);
        positionTargetSpin_->setValue(currentSingleTurn);
        positionDial_->setTargetDegree(currentSingleTurn);
        positionTargetLabel_->setText(
            tr("目标位置  %1°").arg(currentSingleTurn, 0, 'f', 2));
    } else {
        speedTargetLocallySet_ = false;
        const QSignalBlocker spinBlocker(speedSpin_);
        const QSignalBlocker sliderBlocker(speedSlider_);
        speedSpin_->setValue(0);
        speedSlider_->setValue(0);
    }

    if (!applyingTelemetry_ && protocol_.isConnected()) {
        protocol_.setMode(positionMode);
    }
}

void MainWindow::onSpeedChanged(int rpm)
{
    Q_UNUSED(rpm)
    if (!applyingTelemetry_) {
        speedTargetLocallySet_ = true;
    }
}

void MainWindow::submitSpeedTarget()
{
    if (applyingTelemetry_ || !protocol_.isConnected() ||
        !speedModeButton_->isChecked()) {
        return;
    }

    speedTargetLocallySet_ = true;
    const int rpm = speedSpin_->value();
    const quint32 durationMs = qRound(speedDurationSpin_->value() * 1000.0);
    protocol_.setSpeedRpm(static_cast<qint16>(rpm), durationMs);
    appendSerialLog(tr("提交速度目标：%1 RPM（%2 s）")
                        .arg(rpm)
                        .arg(speedDurationSpin_->value(), 0, 'f', 2));
}

void MainWindow::onPositionTargetChanged(double degree)
{
    if (applyingTelemetry_) {
        return;
    }

    positionTargetLocallySet_ = true;
    if ((positionTargetSpin_ != nullptr) && !positionTargetSpin_->hasFocus()) {
        const QSignalBlocker targetBlocker(positionTargetSpin_);
        positionTargetSpin_->setValue(degree);
    }
    positionDial_->setTargetDegree(degree);
    positionTargetLabel_->setText(tr("目标位置  %1°").arg(degree, 0, 'f', 2));
}

void MainWindow::submitPositionTarget()
{
    const double degree = positionTargetSpin_->value();
    positionTargetLocallySet_ = true;

    /*
     * Do not take the value back from telemetry here.  The edit loses focus
     * before QPushButton::clicked is emitted, so a 20 Hz telemetry update used
     * to overwrite the newly entered number with the previous target.
     */
    if (!positionModeButton_->isChecked()) {
        positionModeButton_->setChecked(true);
        controlStack_->setCurrentIndex(1);
        protocol_.setMode(true);
    }

    positionDial_->setTargetDegree(degree);
    positionTargetLabel_->setText(
        tr("目标位置  %1°").arg(degree, 0, 'f', 2));
    const double multiTurnTarget = nearestMultiTurnTarget(degree, currentMultiTurnDegree_);
    const quint32 durationMs = qRound(positionDurationSpin_->value() * 1000.0);
    protocol_.setPositionCdeg(qRound(multiTurnTarget * 100.0), durationMs);
    appendSerialLog(tr("提交位置目标：%1°（多圈目标 %2°，%3 s）")
                        .arg(degree, 0, 'f', 2)
                        .arg(multiTurnTarget, 0, 'f', 2)
                        .arg(positionDurationSpin_->value(), 0, 'f', 2));
}

void MainWindow::setValue(const QString &key, const QString &value)
{
    if (QLabel *label = values_.value(key, nullptr)) {
        label->setText(value);
    }
}

void MainWindow::updateFaultIndicators(quint16 currentFaults,
                                       quint16 occurredFaults)
{
    for (auto iterator = faultIndicators_.cbegin();
         iterator != faultIndicators_.cend();
         ++iterator) {
        QLabel *indicator = iterator.value();
        const quint16 bit = iterator.key();
        if ((currentFaults & bit) != 0U) {
            indicator->setStyleSheet(
                QStringLiteral("color:#ff5368;font-weight:700;padding:2px 4px"));
        } else if ((occurredFaults & bit) != 0U) {
            indicator->setStyleSheet(
                QStringLiteral("color:#ffb454;font-weight:700;padding:2px 4px"));
        } else {
            indicator->setStyleSheet(
                QStringLiteral("color:#53647a;padding:2px 4px"));
        }
    }
}

void MainWindow::setControlsEnabled(bool enabled)
{
    speedModeButton_->setEnabled(enabled);
    positionModeButton_->setEnabled(enabled);
    controlStack_->setEnabled(enabled);
    const auto buttons = findChildren<QPushButton *>();
    for (QPushButton *button : buttons) {
        if (button->property("requiresConnection").toBool()) {
            button->setEnabled(enabled);
        }
    }
}

QString MainWindow::motorStateName(quint8 state)
{
    switch (state) {
    case 0:  return tr("IDLE");
    case 2:  return tr("编码器对齐");
    case 4:  return tr("启动");
    case 6:  return tr("运行");
    case 8:  return tr("停止");
    case 10: return tr("故障发生");
    case 11: return tr("故障待复位");
    case 12: return tr("等待 ICL");
    case 16: return tr("自举充电");
    case 17: return tr("偏置校准");
    case 19: return tr("切换");
    case 20: return tr("停机等待");
    case 21: return tr("OTF 检测");
    case 22: return tr("OTF 制动");
    default:
        return tr("状态 %1").arg(state);
    }
}
