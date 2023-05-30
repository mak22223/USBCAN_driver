
#include "driver.h"

#include "passthrudef.h"
#include "deviceprotocol/deviceprotocol.hpp"

#include <chrono>
#include <QTime>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include "thread"

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
    static ThreadCommand commandOnExecution;
    {
      std::unique_lock lock(commandQueueMutex, std::defer_lock);
      if (lock.try_lock()) {
        if (!commands.empty() && (commands.front().status == ThreadCommand::Status::PENDING)) {
          commands.front().status = ThreadCommand::Status::EXECUTING;
          commandOnExecution = commands.front();

          /// нужно отправить команду
          DeviceCommand cmd;
          if (!prepareCommand(commandOnExecution, cmd)) {
            /// неверно задана команда или её аргументы
            qDebug() << "Неверно задана команда или её аргументы";
            errorOccurred = true;
          }
          /// проверить результат отправки команды
          sendCommand(cmd);
        }
      }
    }

    /// возможно нужно вызывать waitForReadyRead()

    /// проверка отсутствия ошибок порта
    QSerialPort::SerialPortError error = d_port.error();
    if (error != QSerialPort::NoError) {
      /// handle port error
      qDebug() << "Port has error: " << error;
      /// method to close port, clear commands, filters and etc.
      errorOccurred = true;
      continue;
    }

    /// проверка наличия сообщений в буфере отправки и отправка их

    if (!d_port.waitForBytesWritten(0) || !d_port.waitForReadyRead(0)) {
      d_port.clearError();
    }

    DeviceAnswer ans;
    if (receiveAnswer(ans)) {

      qDebug() << "Received answer with id: " << ans.id;
      std::unique_lock lock(commandQueueMutex, std::defer_lock);
      switch (ans.id)
      {
      case DeviceAnswer::Id::INFO:
        /* code */
        break;

      case DeviceAnswer::Id::ERROR:
        if (!handleErrorCode(ans)) {
          /// unexpected error code received
          qDebug() << "Unexpected error code received: " << ans.arg.error.code;
        }
        break;

      case DeviceAnswer::Id::MSG_RECV8:
        /// pass msg to filters
        /// add to buffer if passed filtering
        break;
      
      case DeviceAnswer::Id::MSG_SEND:
        /// sent msg acknowledge
        break;
      
      case DeviceAnswer::Id::FLTR_SET:
        /// store filter id
        break;
      
      case DeviceAnswer::Id::READ_VBATT:
        if (commandOnExecution.id == ThreadCommand::Id::READ_VBATT) {
          lock.lock();
          assert(!commands.empty());
          commands.front().returnCode = ThreadCommand::ReturnCode::OK;
          commands.front().result.readVBatt.millivolts = ans.arg.readVBatt.millivolts;
          commands.front().status = ThreadCommand::Status::DONE;
          lock.unlock();
          commandOnExecution = ThreadCommand();
        } else {
          /// received some answer while there was no request
          qDebug() << "Received VBATT answer while there was no request.";
        }
        break;
      
      default:
        qDebug() << "Cannot handle DeviceAnswer with id: " << ans.id;
        break;
      }
    }
  }

  // деинициализация потока
  d_port.close();
}

/*           PassThru interface section             */

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

  d_port.waitForReadyRead(200);
  if (!receiveAnswer(infoAnswer)) {
    qDebug() << "Failed to receive device answer.";
    d_port.close();
    return ERR_FAILED;
  }

  if (infoAnswer.id != DeviceAnswer::INFO) {
    qDebug() << "Failed to request device info.";
    d_port.close();
    return ERR_FAILED;
  }

  /// Fill device info string

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

int Driver::connect(
  ulong ProtocolID,
  ulong Flags,
  ulong BaudRate,
  ulong *pChannelID)
{
  
}

int Driver::disconnect(ulong ChannelID)
{

}

long Driver::readMsgs(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pNumMsgs,
  ulong Timeout)
{

}
long Driver::writeMsgs(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pNumMsgs,
  ulong Timeout)
{

}

long Driver::startMsgFilter(
  ulong ChannelID,
  ulong FilterType,
  PassThruMsg *pMaskMsg,
  PassThruMsg *pPatternMsg,
  PassThruMsg *pFlowControlMsg,
  ulong *pFilterID)
{

}

long Driver::stopMsgFilter(
  ulong ChannelID,
  ulong FilterID)
{

}

long Driver::setProgrammingVoltage(
  ulong DeviceID,
  ulong PinNumber,
  ulong Voltage)
{

}

long Driver::readVersion(
  ulong DeviceID,
  char *pFirmwareVersion,
  char *pDllVersion,
  char *pApiVersion)
{

}

long Driver::getLastError(char *pErrorDescription)
{

}

long Driver::ioctl(
  ulong ChannelID,
  ulong IoctlID,
  void *pInput,
  void *pOutput)
{
  std::unique_lock lock(commandQueueMutex, std::defer_lock);
  ThreadCommand cmd;
  switch (IoctlID)
  {
  case PassThruIoctlId::READ_VBATT:
    /// check for valid deviceid

    lock.lock();
    cmd.id = ThreadCommand::Id::READ_VBATT;
    commands.push(cmd);
    lock.unlock();
    break;

  /// cases for valid but unsupported IoctlIDs
  
  default:
    return ERR_INVAILD_IOCTL_ID;
  }

  /// add timeout and maybe add a delay betweer read operations
  while (commands.front().status != ThreadCommand::Status::DONE);

  lock.lock();
  ThreadCommand doneCmd;
  doneCmd = commands.front();
  commands.pop();
  lock.unlock();

  switch (doneCmd.id)
  {
  case ThreadCommand::Id::READ_VBATT:
    if (doneCmd.returnCode != ThreadCommand::ReturnCode::OK) {
      /// failed to get vbatt answer
      return ERR_FAILED;
    }

    /// round to tenth of volts
    *(ulong*)pOutput = doneCmd.result.readVBatt.millivolts;
    break;

  /// cases for valid but unsupported IoctlIDs
  
  default:
    qDebug() << "Device thread executed unknown command.";
    break;
  }

  return STATUS_NOERROR;
}

/*            Private functions section            */

bool Driver::sendCommand(const DeviceCommand &cmd)
{
  std::string sendBuf;
  switch (cmd.id)
  {
  case DeviceCommand::Id::INFO:
    sendBuf.resize(1 + 1);
    sendBuf[0] = cmd.id;
    break;

  case DeviceCommand::Id::READ_VBATT:
    sendBuf.resize(1 + 1);
    sendBuf[0] = cmd.id;
    break;
  
  default:
    /// handle default case
    qDebug() << "Command has unknown id: " << cmd.id;
    return false;
  }
  
  sendBuf[sendBuf.size() - 1] = ControlChar;

  if (d_port.write(sendBuf.c_str(), sendBuf.size()) <= 0 || !d_port.waitForBytesWritten(5)) {
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

  case DeviceAnswer::Id::READ_VBATT:
    if (d_devBuf.used() < 4) {
      break;
    }

    d_devBuf.get(1, reinterpret_cast<char*>(&ans.id));
    d_devBuf.get(2, reinterpret_cast<char*>(&ans.arg.readVBatt.millivolts));
    
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

bool Driver::handleErrorCode(const DeviceAnswer &ans)
{
  /// implement method
  return false;
}

bool Driver::prepareCommand(const ThreadCommand &req, DeviceCommand &cmd)
{
  switch (req.id)
  {
  case ThreadCommand::Id::READ_VBATT:
    cmd.id = DeviceCommand::Id::READ_VBATT;
    break;
  
  default:
    qDebug() << "Cannot prepare DeviceCommand from ThreadCommand with ID: " << req.id;
    return false;
  }

  return true;
}
