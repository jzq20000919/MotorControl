#pragma once

#include "wirelessprotocol.h"

#include <QHash>
#include <QMainWindow>
#include <QTimer>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QPlainTextEdit;
class QRadioButton;
class QSlider;
class QSpinBox;
class QStackedWidget;
class PositionDial;

class MainWindow final : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void toggleConnection();
    void onConnectionChanged(bool connected, const QString &status);
    void onTelemetry(const FocTelemetry &telemetry);
    void onProtocolError(const QString &message);
    void appendWirelessLog(const QString &message);
    void onModeChanged();
    void onSpeedChanged(int rpm);
    void submitSpeedTarget();
    void onPositionTargetChanged(double degree);
    void submitPositionTarget();

private:
    QWidget *createHeader();
    QWidget *createModeAndActions();
    QWidget *createTelemetryPanel();
    QWidget *createFaultIndicatorPanel();
    QWidget *createControlPanel();
    QWidget *createWirelessLogPanel();
    QWidget *createValueCard(const QString &key, const QString &title,
                             const QString &unit, const QString &color);
    void setValue(const QString &key, const QString &value);
    void updateFaultIndicators(quint16 currentFaults, quint16 occurredFaults);
    void setControlsEnabled(bool enabled);
    void applyTelemetryMode(int mode);
    void requestModeChange(bool positionMode);
    static QString motorStateName(quint8 state);

    WirelessProtocol protocol_;
    QLineEdit *brokerHostEdit_ = nullptr;
    QSpinBox *brokerPortSpin_ = nullptr;
    QPushButton *connectButton_ = nullptr;
    QLabel *connectionDot_ = nullptr;
    QLabel *connectionText_ = nullptr;
    QRadioButton *speedModeButton_ = nullptr;
    QRadioButton *positionModeButton_ = nullptr;
    QStackedWidget *controlStack_ = nullptr;
    QSlider *speedSlider_ = nullptr;
    QSpinBox *speedSpin_ = nullptr;
    QDoubleSpinBox *speedDurationSpin_ = nullptr;
    PositionDial *positionDial_ = nullptr;
    QDoubleSpinBox *positionTargetSpin_ = nullptr;
    QDoubleSpinBox *positionDurationSpin_ = nullptr;
    QLabel *positionCurrentLabel_ = nullptr;
    QLabel *positionTargetLabel_ = nullptr;
    QPlainTextEdit *wirelessLog_ = nullptr;
    QHash<QString, QLabel *> values_;
    QHash<quint16, QLabel *> faultIndicators_;
    bool applyingTelemetry_ = false;
    bool speedTargetLocallySet_ = false;
    bool positionTargetLocallySet_ = false;
    QTimer modeChangeTimeout_;
    int pendingMode_ = -1;
    int lastTelemetryMode_ = 0;
    bool hasTelemetryMode_ = false;
    double currentMultiTurnDegree_ = 0.0;
};
