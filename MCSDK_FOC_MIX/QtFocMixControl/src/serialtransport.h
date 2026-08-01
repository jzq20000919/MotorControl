#pragma once

#include <QByteArray>
#include <QObject>
#include <QStringList>
#include <QTimer>

#ifdef Q_OS_WIN
#include <qt_windows.h>
#endif

class SerialTransport final : public QObject
{
    Q_OBJECT

public:
    explicit SerialTransport(QObject *parent = nullptr);
    ~SerialTransport() override;

    static QStringList availablePorts();
    bool open(const QString &portName, quint32 baudRate);
    void close();
    bool isOpen() const;
    bool write(const QByteArray &data);
    bool flush();
    QString errorString() const;

signals:
    void bytesReceived(const QByteArray &data);
    void transportError(const QString &message);

private slots:
    void pollInput();

private:
#ifdef Q_OS_WIN
    HANDLE handle_ = INVALID_HANDLE_VALUE;
#endif
    QTimer pollTimer_;
    QString errorString_;
};
