#pragma once

#include "mqttclient.h"

#include <QMainWindow>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
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
    void setConnected(bool connected, const QString &message);

private:
    QByteArray makeCommand(const QString &command, qint64 value);

    MqttClient mqtt_;
    QLineEdit *hostEdit_ = nullptr;
    QSpinBox *portSpin_ = nullptr;
    QPushButton *connectButton_ = nullptr;
    QLabel *statusLabel_ = nullptr;
    QSpinBox *speedSpin_ = nullptr;
    QPushButton *sendSpeedButton_ = nullptr;
    QDoubleSpinBox *positionSpin_ = nullptr;
    QPushButton *sendPositionButton_ = nullptr;
    quint32 nextCommandId_ = 0U;
};
