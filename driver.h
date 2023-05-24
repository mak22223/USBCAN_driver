
#ifndef DRIVER_H
#define DRIVER_H

#include "passthrudef.h"
#include "deviceprotocol/deviceprotocol.hpp"

#include "circularbuffer.hpp"

#include <QtSerialPort/QSerialPort>
#include <queue>
#include <thread>
#include <vector>

using ulong = unsigned long;

struct Channel {
  std::vector<PassThruMsg> recMsgs;
  std::vector<PassThruMsg> sentMsgs;
  std::vector<PassThruMsg> passFilters;
  std::vector<PassThruMsg> blockFilters;
  std::vector<PassThruMsg> flowControlFilters;
  /// циклические сообщения
};

struct ThreadCommand
{
  enum {
    INTERFACE_INIT = 0x00,
    INTERFACE_CFG = 0x01,
    INTERFACE_DEINIT = 0x02
  } cmdId;

  union
  {
    /* data */
  } arg;
  
};

class Driver
{
public:
  Driver();
  ~Driver();

  // PassThru section
  int open();
  int close();
  int connect(
    ulong ProtocolID,
    ulong Flags,
    ulong BaudRate,
    ulong *pChannelID);
  int disconnect(ulong ChannelID);
  long readMsgs(
    ulong ChannelID,
    PassThruMsg *pMsg,
    ulong *pNumMsgs,
    ulong Timeout);
  long writeMsgs(
    ulong ChannelID,
    PassThruMsg *pMsg,
    ulong *pNumMsgs,
    ulong Timeout);
  long startMsgFilter(
    ulong ChannelID,
    ulong FilterType,
    PassThruMsg *pMaskMsg,
    PassThruMsg *pPatternMsg,
    PassThruMsg *pFlowControlMsg,
    ulong *pFilterID);
  long stopMsgFilter(
    ulong ChannelID,
    ulong FilterID);
  long setProgrammingVoltage(
    ulong DeviceID,
    ulong PinNumber,
    ulong Voltage);
  long readVersion(
    ulong DeviceID,
    char *pFirmwareVersion,
    char *pDllVersion,
    char *pApiVersion);
  long getLastError(char *pErrorDescription);
  long ioctl(
    ulong ChannelID,
    ulong IoctlID,
    void *pInput,
    void *pOutput);

  bool isOpen() const;  

protected:
  void thread(std::stop_token stopToken);
  bool sendCommand(const DeviceCommand &cmd);
  bool receiveAnswer(DeviceAnswer &ans);

protected:
  std::jthread d_devThread;

  bool d_deviceOpened;
  QSerialPort d_port;
  CircularBuffer<char, 4608> d_devBuf;

  std::string d_deviceInfo;

  std::vector<Channel> channels;
  std::queue<ThreadCommand> commands;


  

};

#endif // DRIVER_H
