
#include "driver.h"

#include "passthrudef.h"
#include "deviceprotocol/deviceprotocol.hpp"

#include <QTime>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <QtDebug>

constexpr quint16 ProductId = 0x5741;
constexpr qint32 Baudrate = 1'000'000UL;

Driver::Driver()
: d_deviceOpened(false),
  d_port(QSerialPort())
{

}

Driver::~Driver()
{

}

void Driver::thread(std::stop_token stopToken)
{
  // инициализация потока
  bool errorOccurred = false;

  while (!stopToken.stop_requested() && !errorOccurred) {
    /// проверка очереди команд на исполнение

    /// проверка отсутствия ошибок порта
    QSerialPort::SerialPortError error = d_port.error();
    if (error != QSerialPort::NoError) {
      /// handle port error
      qDebug() << "Port has error: " << error;
    }

    DeviceAnswer ans;
    if (receiveAnswer(ans)) {

      qDebug() << "Received answer with id: " << ans.id;

      switch (ans.id)
      {
      case DeviceAnswer::INFO:
        /* code */
        break;
      
      default:
        break;
      }
    }
  }

  // деинициализация потока

}

int Driver::open()
{
  if (d_deviceOpened) {
    /// set error description
    qDebug() << "Device is already opened.\n";
    return ERR_DEVICE_IN_USE;
  }

  qDebug() << "Searching for device.\n";
  bool deviceFound = false;
  const auto serialPorts = QSerialPortInfo::availablePorts();
  for (const auto &port : serialPorts) {
    // qDebug() << "\n"
    //              << "Port:" << port.portName() << "\n"
    //              << "Location:" << port.systemLocation() << "\n"
    //              << "Description:" << port.description() << "\n"
    //              << "Manufacturer:" << port.manufacturer() << "\n"
    //              << "Serial number:" << port.serialNumber() << "\n"
    //              << "Vendor Identifier:"
    //              << (port.hasVendorIdentifier()
    //                  ? QByteArray::number(port.vendorIdentifier(), 16)
    //                  : QByteArray()) << "\n"
    //              << "Product Identifier:"
    //              << (port.hasProductIdentifier()
    //                  ? QByteArray::number(port.productIdentifier(), 16)
    //                  : QByteArray());

    if (port.hasProductIdentifier() && port.productIdentifier() == ProductId) {
      deviceFound = true;
      d_port.setPort(port);
      break;
    }
  }

  if (!deviceFound) {
    /// set error description
    qDebug() << "Device not found.\n";
    return ERR_DEVICE_NOT_CONNECTED;
  }

  d_port.setBaudRate(Baudrate);
  if (!d_port.open(QIODeviceBase::ReadWrite)) {
    /// set error description
    qDebug() << "Device is already in use.\n";
    return ERR_DEVICE_IN_USE;
  }


  /// RESET DEVICE BEFORE INFO REQUEST AND RESET INPUT BUFFER
  DeviceCommand infoRequest;
  DeviceAnswer infoAnswer;
  infoRequest.id = DeviceCommand::INFO;
  infoAnswer.id = DeviceAnswer::DEFAULT;

  if (!sendCommand(infoRequest)) {
    /// set error description
    qDebug() << "Failed to send initial info command.";
    d_port.close();
    return ERR_FAILED;
  }

//  QTime timer = QTime::currentTime().addMSecs(200);
  QTimer timer = QTimer();
  timer.setTimerType(Qt::CoarseTimer);
  timer.setSingleShot(true);
  timer.start(200);
  d_port.waitForReadyRead(200);
  while (!receiveAnswer(infoAnswer) /*&& timer.remainingTime()*/);

  if (infoAnswer.id != DeviceAnswer::INFO) {
    qDebug() << "Failed to request device info.";
    d_port.close();
    return ERR_FAILED;
  }

  /// Fill device info string

  /// Launch device thread
  d_deviceOpened = true;
  d_devThread = std::jthread(std::bind_front(&Driver::thread, this));

  qDebug() << "Successfully opened device and started thread.";
  return STATUS_NOERROR;
}

int Driver::close()
{
  if (d_devThread.joinable()) {
    d_devThread.request_stop();
    d_devThread.join();
  }
  d_deviceOpened = false;
  d_port.close();

  /// очистка буферов и прочего

  return STATUS_NOERROR;
}

bool Driver::sendCommand(const DeviceCommand &cmd)
{
  qint64 len = 0;
  std::string sendBuf;
  switch (cmd.id)
  {
  case DeviceCommand::Id::INFO:
    len = 1;
    sendBuf.resize(len);
    sendBuf[0] = cmd.id;
    break;
  
  default:
    /// handle default case
    qDebug() << "Command has unknown id: " << cmd.id;
    return false;
  }
  
  if (d_port.write(sendBuf.c_str(), len) <= 0 || !d_port.waitForBytesWritten(5)) {
    /// set error description
    qDebug() << "Failed to write command to port.";
    return false;
  }

  return true;
}

bool Driver::receiveAnswer(DeviceAnswer &ans)
{
  size_t bytes = d_port.bytesAvailable();
  if (bytes == 0) {
    return false;
  }

  std::unique_ptr<char[]> buf(new char[bytes]);
  d_port.read(buf.get(), bytes);
  if (!d_devBuf.put(buf.get(), bytes)) {
    qDebug() << "Failed to put data in circular buffer. Data size - " << std::to_string(bytes);
    return false;
  }

  char controlChar = 0;
  d_devBuf.peek(1, &ans.id);
  switch (ans.id)
  {
  case DeviceAnswer::INFO:
    /// заменить литералы размера команды на константы
    if (d_devBuf.used() < 8) {
      break;
    }

    d_devBuf.get(1, reinterpret_cast<char*>(&ans.id));
    d_devBuf.get(3, reinterpret_cast<char*>(&ans.arg.info.hwVer));
    d_devBuf.get(3, reinterpret_cast<char*>(&ans.arg.info.fwVer));

    break;


  default:
    qDebug() << "Unknown answer id received. Resetting buffer.";
    d_devBuf.reset();
    return false;
  }

  d_devBuf.get(1, &controlChar);
  if (controlChar != ControlChar) {
    /// command receive error ALARM
    qDebug() << "Received malformed command. Resetting buffer.";
    d_devBuf.reset();
    return false;
  }

  return true;
}
