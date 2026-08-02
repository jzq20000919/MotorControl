#pragma once

#include "mqttclient.h"

#include <QMainWindow>

class QLabel;
class QLineEdit;
class QPushButton;
class QSlider;
class QSpinBox;

class MainWindow final : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void toggleConnection();
    void sendSpeed();
    void sendPosition();
    void startMotor();
    void stopMotor();
    void setConnected(bool connected, const QString &message);

private:
    QByteArray makeCommand(const QString &command, qint64 value);
    bool publishCommand(const QString &command, qint64 value,
                        const QString &description);
    void showMqttStatus(const QString &state, const QString &color,
                        const QString &message);

    MqttClient mqtt_;
    QLineEdit *hostEdit_ = nullptr;
    QSpinBox *portSpin_ = nullptr;
    QPushButton *connectButton_ = nullptr;
    QLabel *mqttIndicator_ = nullptr;
    QLabel *mqttStateLabel_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QSlider *speedSlider_ = nullptr;
    QLabel *speedValueLabel_ = nullptr;
    QPushButton *sendSpeedButton_ = nullptr;
    QSlider *positionSlider_ = nullptr;
    QLabel *positionValueLabel_ = nullptr;
    QPushButton *sendPositionButton_ = nullptr;
    QPushButton *startButton_ = nullptr;
    QPushButton *stopButton_ = nullptr;
    quint32 nextCommandId_ = 0U;
};
