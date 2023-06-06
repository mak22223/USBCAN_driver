
#ifndef DRIVER_H
#define DRIVER_H

#include "passthrudef.h"
#include "deviceprotocol/deviceprotocol.hpp"

#include "circularbuffer.hpp"

#include <QtSerialPort/QSerialPort>
#include <mutex>
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
  /// id установленных фильтров
};

struct ThreadCommand
{
  enum Id {
    DEFAULT,
    INTERFACE_INIT = 0x01U,
    INTERFACE_CFG = 0x02U,
    INTERFACE_DEINIT = 0x03U,
    FLTR_SET = 0x04U,
    FLTR_UNSET = 0x05U,
    READ_VBATT = 0x06U,
    INFO = 0x07U,
    RESET = 0x08U,
  };

  enum Status {
    PENDING = 0U,
    EXECUTING = 1U,
    DONE = 2U,
  };

  enum ReturnCode {
    OK = 0U,
    ERROR
  };

  Id id = Id::DEFAULT;
  Status status = Status::PENDING;

  union
  {
    struct {

    } itfInit;

    struct {

    } itfCfg;

    struct {
      uint8_t itfType;
    } itfDeinit;

    struct {

    } fltrSet;

    struct {

    } fltrUnset;
  } arg;

  uint32_t returnCode;
  union
  {
    struct
    {
      char hwVer[3];
      char fwVer[3];
    } info;

    struct
    {
      
    } fltrSet;

    struct
    {
      uint32_t millivolts;
    } readVBatt;
  } result;
  
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

protected:
  void thread(std::stop_token stopToken);
  bool prepareCommand(const ThreadCommand &req, DeviceCommand &cmd);
  bool sendCommand(QSerialPort &port, const DeviceCommand &cmd);
  bool receiveAnswer(QSerialPort &port, DeviceAnswer &ans);

protected:
  std::jthread d_devThread;

  bool d_deviceOpen;
  CircularBuffer<char, 4608> d_devBuf;

  std::string d_deviceInfo;

  std::vector<Channel> channels;
  std::queue<ThreadCommand> commands;
  std::mutex commandQueueMutex;


  

};

#endif // DRIVER_H
