#include "serialtransport.h"

#include <QSet>

#include <algorithm>
#include <iterator>

SerialTransport::SerialTransport(QObject *parent)
    : QObject(parent)
{
    pollTimer_.setInterval(4);
    connect(&pollTimer_, &QTimer::timeout, this, &SerialTransport::pollInput);
}

SerialTransport::~SerialTransport()
{
    close();
}

QStringList SerialTransport::availablePorts()
{
    QStringList ports;

#ifdef Q_OS_WIN
    HKEY key = nullptr;
    if (RegOpenKeyExW(HKEY_LOCAL_MACHINE, L"HARDWARE\\DEVICEMAP\\SERIALCOMM", 0,
                      KEY_QUERY_VALUE, &key) == ERROR_SUCCESS) {
        for (DWORD index = 0;; ++index) {
            wchar_t valueName[512]{};
            BYTE valueData[512]{};
            DWORD nameLength = static_cast<DWORD>(std::size(valueName));
            DWORD dataLength = sizeof(valueData);
            DWORD valueType = 0;
            const LONG status = RegEnumValueW(key, index, valueName, &nameLength,
                                              nullptr, &valueType, valueData, &dataLength);
            if (status == ERROR_NO_MORE_ITEMS) {
                break;
            }
            if (status != ERROR_SUCCESS ||
                (valueType != REG_SZ && valueType != REG_EXPAND_SZ)) {
                continue;
            }
            const QString port = QString::fromWCharArray(
                reinterpret_cast<const wchar_t *>(valueData)).trimmed();
            if (!port.isEmpty() && !ports.contains(port, Qt::CaseInsensitive)) {
                ports.append(port);
            }
        }
        RegCloseKey(key);
    }

    wchar_t target[512];
    for (int index = 1; index <= 256; ++index) {
        const QString name = QStringLiteral("COM%1").arg(index);
        if (QueryDosDeviceW(reinterpret_cast<LPCWSTR>(name.utf16()), target,
                            static_cast<DWORD>(std::size(target))) != 0) {
            if (!ports.contains(name, Qt::CaseInsensitive)) {
                ports.append(name);
            }
        }
    }

    auto portNumber = [](const QString &name) {
        bool ok = false;
        const int number = name.mid(3).toInt(&ok);
        return ok ? number : 10000;
    };
    std::sort(ports.begin(), ports.end(), [&](const QString &left, const QString &right) {
        return portNumber(left) < portNumber(right);
    });
#endif

    return ports;
}

bool SerialTransport::open(const QString &portName, quint32 baudRate)
{
    close();
    errorString_.clear();

#ifdef Q_OS_WIN
    const QString devicePath = QStringLiteral("\\\\.\\%1").arg(portName);
    handle_ = CreateFileW(reinterpret_cast<LPCWSTR>(devicePath.utf16()),
                          GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                          FILE_ATTRIBUTE_NORMAL, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
        errorString_ = tr("无法打开 %1（Windows 错误 %2）")
                           .arg(portName)
                           .arg(GetLastError());
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(handle_, &dcb)) {
        errorString_ = tr("无法读取串口参数（Windows 错误 %1）").arg(GetLastError());
        close();
        return false;
    }

    dcb.BaudRate = baudRate;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fDsrSensitivity = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(handle_, &dcb)) {
        errorString_ = tr("无法设置 %1 baud（Windows 错误 %2）")
                           .arg(baudRate)
                           .arg(GetLastError());
        close();
        return false;
    }

    COMMTIMEOUTS timeouts{};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100;
    SetCommTimeouts(handle_, &timeouts);
    SetupComm(handle_, 4096, 4096);
    PurgeComm(handle_, PURGE_RXCLEAR | PURGE_TXCLEAR);

    pollTimer_.start();
    return true;
#else
    Q_UNUSED(portName)
    Q_UNUSED(baudRate)
    errorString_ = tr("当前构建仅实现了 Windows 串口后端");
    return false;
#endif
}

void SerialTransport::close()
{
    pollTimer_.stop();
#ifdef Q_OS_WIN
    if (handle_ != INVALID_HANDLE_VALUE) {
        CloseHandle(handle_);
        handle_ = INVALID_HANDLE_VALUE;
    }
#endif
}

bool SerialTransport::isOpen() const
{
#ifdef Q_OS_WIN
    return handle_ != INVALID_HANDLE_VALUE;
#else
    return false;
#endif
}

bool SerialTransport::write(const QByteArray &data)
{
#ifdef Q_OS_WIN
    if (!isOpen()) {
        return false;
    }

    DWORD written = 0;
    if (!WriteFile(handle_, data.constData(), static_cast<DWORD>(data.size()),
                   &written, nullptr) ||
        written != static_cast<DWORD>(data.size())) {
        errorString_ = tr("串口写入失败（Windows 错误 %1）").arg(GetLastError());
        emit transportError(errorString_);
        return false;
    }
    return true;
#else
    Q_UNUSED(data)
    return false;
#endif
}

bool SerialTransport::flush()
{
#ifdef Q_OS_WIN
    if (!isOpen()) {
        return false;
    }
    if (!FlushFileBuffers(handle_)) {
        errorString_ = tr("等待串口发送完成失败（Windows 错误 %1）").arg(GetLastError());
        emit transportError(errorString_);
        return false;
    }
    return true;
#else
    return false;
#endif
}

QString SerialTransport::errorString() const
{
    return errorString_;
}

void SerialTransport::pollInput()
{
#ifdef Q_OS_WIN
    if (!isOpen()) {
        return;
    }

    COMSTAT status{};
    DWORD errors = 0;
    if (!ClearCommError(handle_, &errors, &status)) {
        errorString_ = tr("串口读取状态失败（Windows 错误 %1）").arg(GetLastError());
        emit transportError(errorString_);
        close();
        return;
    }

    while (status.cbInQue > 0) {
        QByteArray data(static_cast<qsizetype>(std::min<DWORD>(status.cbInQue, 2048)), '\0');
        DWORD readCount = 0;
        if (!ReadFile(handle_, data.data(), static_cast<DWORD>(data.size()),
                      &readCount, nullptr)) {
            errorString_ = tr("串口读取失败（Windows 错误 %1）").arg(GetLastError());
            emit transportError(errorString_);
            close();
            return;
        }
        if (readCount == 0) {
            break;
        }
        data.resize(static_cast<qsizetype>(readCount));
        emit bytesReceived(data);

        if (!ClearCommError(handle_, &errors, &status)) {
            break;
        }
    }
#endif
}
