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
: d_deviceOpen(false)
{

}
Driver::~Driver()
{

}

void Driver::thread(std::stop_token stopToken)
{
  // инициализация потока
  bool errorOccurred = false;
  QSerialPort port;

  {
    qDebug() << "Searching for device.\n";
    bool deviceFound = false;
    const auto serialPorts = QSerialPortInfo::availablePorts();
    for (const auto &portInfo : serialPorts) {
      if (portInfo.hasProductIdentifier() && portInfo.productIdentifier() == ProductId) {
        deviceFound = true;
        port.setPort(portInfo);
        port.setBaudRate(Baudrate);
        break;
      }
    }

    if (!deviceFound) {
      qDebug() << "Device not found.\n";
      errorOccurred = true;
    } else {
      if (!port.open(QIODeviceBase::ReadWrite)) {
        qDebug() << "Device is already in use.\n";
        errorOccurred = true;
      }
    }
  }

  while (!stopToken.stop_requested() && !errorOccurred) {
    // проверка очереди команд на исполнение
    static ThreadCommand commandOnExecution;
    {
      std::unique_lock lock(commandQueueMutex, std::defer_lock);
      if (lock.try_lock()) {
        if (!commands.empty() && (commands.front().status == ThreadCommand::Status::PENDING)) {
          commands.front().status = ThreadCommand::Status::EXECUTING;
          commandOnExecution = commands.front();

          DeviceCommand cmd;
          if (!prepareCommand(commandOnExecution, cmd)) {
            /// неверно задана команда или её аргументы
            qDebug() << "Неверно задана команда или её аргументы";
            errorOccurred = true;
          }
          /// проверить результат отправки команды
          sendCommand(port, cmd);
        }
      }
    }

    // проверка отсутствия ошибок порта
    QSerialPort::SerialPortError error = port.error();
    if (error != QSerialPort::NoError) {
      qDebug() << "Port has error: " << error;
      errorOccurred = true;
      continue;
    }

    if (!port.waitForBytesWritten(0) || !port.waitForReadyRead(0)) {
      port.clearError();
    }

    DeviceAnswer ans;
    if (receiveAnswer(port, ans)) {

      qDebug() << "Received answer with id: " << int(ans.id);

      std::unique_lock lock(commandQueueMutex, std::defer_lock);

      if (ans.id == DeviceAnswer::Id::ERROR) {
        switch (ans.arg.error.code)
        {
        case DeviceAnswer::ErrorCode::RESET_OK:
          if (commandOnExecution.id != ThreadCommand::Id::RESET) {
            qDebug() << "Received RESET_OK error code while there was no request.";
            break;
          }

          qDebug() << "Received RESET_OK error code.";
          lock.lock();
          assert(!commands.empty());
          commands.front().returnCode = ThreadCommand::ReturnCode::OK;
          commands.front().status = ThreadCommand::Status::DONE;
          lock.unlock();
          commandOnExecution = ThreadCommand();
          break;
        
        default:
          break;
        }
      }

      switch (ans.id)
      {
      case DeviceAnswer::Id::INFO:
        if (commandOnExecution.id == ThreadCommand::Id::INFO) {
          lock.lock();
          assert(!commands.empty());
          commands.front().returnCode = ThreadCommand::ReturnCode::OK;
          for (int i = 0; i < 6; ++i) {
            commands.front().result.info.hwVer[i] = ans.arg.info.hwVer[i];
          }
          commands.front().status = ThreadCommand::Status::DONE;
          lock.unlock();
          commandOnExecution = ThreadCommand();
        } else {
          /// received some answer while there was no request
          qDebug() << "Received INFO answer while there was no request.";
        }
        break;

      case DeviceAnswer::Id::ERROR:
        
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

      case DeviceAnswer::Id::RESET:
        if (commandOnExecution.id == ThreadCommand::Id::RESET) {
          lock.lock();
          assert(!commands.empty());
          commands.front().returnCode = ThreadCommand::ReturnCode::OK;
          commands.front().status = ThreadCommand::Status::DONE;
          lock.unlock();
          commandOnExecution = ThreadCommand();
        } else {
          /// received some answer while there was no request
          qDebug() << "Received RESET answer while there was no request.";
        }
        break;

      default:
        qDebug() << "Cannot handle DeviceAnswer with id: " << ans.id;
        break;
      }
    }
  }

  // деинициализация потока
  /// ошибка во время исполнения команды не завершит команду и повесит запрашивающий поток
  port.close();
}

/*           PassThru interface section             */

int Driver::open()
{
  using namespace std::chrono_literals;

  if (d_deviceOpen) {
    /// set error description
    qDebug() << "Device is already open.\n";
    return ERR_DEVICE_IN_USE;
  }

  d_devThread = std::jthread(std::bind_front(&Driver::thread, this));

  // Init device
  QTimer timeout;
  timeout.setTimerType(Qt::CoarseTimer);
  timeout.setSingleShot(true);
  std::unique_lock lock(commandQueueMutex, std::defer_lock);
  ThreadCommand resetCmd;
  resetCmd.id = ThreadCommand::Id::RESET;
  lock.lock();
  commands.push(resetCmd);
  lock.unlock();

  timeout.start(200ms);
  while (commands.front().status != ThreadCommand::Status::DONE && timeout.isActive());

  if (commands.front().status != ThreadCommand::Status::DONE || commands.front().returnCode != ThreadCommand::ReturnCode::OK) {
    /// set error description
    /// clear command queue, stop thread
    qDebug() << "Failed to reset device.";
    return ERR_FAILED;
  }

  lock.lock();
  commands.pop();
  lock.unlock();

  ThreadCommand infoCmd;
  infoCmd.id = ThreadCommand::Id::INFO;
  lock.lock();
  commands.push(infoCmd);
  lock.unlock();

  timeout.start(200ms);
  while (commands.front().status != ThreadCommand::Status::DONE && timeout.isActive());

  if (commands.front().status != ThreadCommand::Status::DONE || commands.front().returnCode != ThreadCommand::ReturnCode::OK) {
    /// set error description
    /// clear command queue, stop thread
    qDebug() << "Failed to get device info.";
    return ERR_FAILED;
  }

  infoCmd = commands.front();
  lock.lock();
  commands.pop();
  lock.unlock();

  /// Fill device info string

  d_deviceOpen = true;

  qDebug() << "Successfully opened device and started thread.";
  return STATUS_NOERROR;
}

int Driver::close()
{
  if (d_devThread.joinable()) {
    d_devThread.request_stop();
    d_devThread.join();
  }
  d_deviceOpen = false;

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

    cmd.id = ThreadCommand::Id::READ_VBATT;
    lock.lock();
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
    return ERR_FAILED;
    break;
  }

  return STATUS_NOERROR;
}

/*            Private functions section            */

bool Driver::sendCommand(QSerialPort &port, const DeviceCommand &cmd)
{
  std::string sendBuf;
  switch (cmd.id)
  {
  case DeviceCommand::Id::INFO:
    sendBuf.resize(1 + 1);
    sendBuf[0] = cmd.id;
    break;

  case DeviceCommand::Id::RESET:
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

  if (port.write(sendBuf.c_str(), sendBuf.size()) <= 0 || !port.waitForBytesWritten(5)) {
    /// set error description
    qDebug() << "Failed to write command to port.";
    return false;
  }

  return true;
}

bool Driver::receiveAnswer(QSerialPort &port, DeviceAnswer &ans)
{
  if (!port.waitForReadyRead(0) && port.error() == QSerialPort::TimeoutError) {
    port.clearError();
  }
  size_t bytes = port.bytesAvailable();
  if (bytes == 0) {
    return false;
  }

  std::unique_ptr<char[]> buf(new char[bytes]);
  port.read(buf.get(), bytes);
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

  case DeviceAnswer::Id::ERROR:
    d_devBuf.get(1, reinterpret_cast<char*>(&ans.id));
    d_devBuf.get(1, reinterpret_cast<char*>(&ans.arg.error.code));
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

bool Driver::prepareCommand(const ThreadCommand &req, DeviceCommand &cmd)
{
  switch (req.id)
  {
  case ThreadCommand::Id::READ_VBATT:
    cmd.id = DeviceCommand::Id::READ_VBATT;
    break;

  case ThreadCommand::Id::INFO:
    cmd.id = DeviceCommand::Id::INFO;
    break;

  case ThreadCommand::Id::RESET:
    cmd.id = DeviceCommand::Id::RESET;
    break;
  
  default:
    qDebug() << "Cannot prepare DeviceCommand from ThreadCommand with ID: " << req.id;
    return false;
  }

  return true;
}
