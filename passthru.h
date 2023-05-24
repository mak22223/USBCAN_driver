
#ifndef PASSTHRU_H
#define PASSTHRU_H

#include "passthrudef.h"
#include "driver.h"

#include <map>
#include <memory>

using ulong = unsigned long;

class PassThru
{
public:
  static bool initPassThruInterface();
  static bool deinitPassThruInterface();
  static PassThru& getInstance();

  long open(void*, ulong *pDeviceID);
  long close(ulong DeviceID);
  long connect(
    ulong DeviceID,
    ulong ProtocolID,
    ulong Flags,
    ulong BaudRate,
    ulong *pChannelID);
  long disconnect(ulong ChannelID);
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
  long startPeriodicMsg(
    ulong ChannelID,
    PassThruMsg *pMsg,
    ulong *pMsgID,
    ulong TimeInterval);
  long stopPeriodicMsg(
    ulong ChannelID,
    ulong MsgID);
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
  PassThru();

protected:
  static PassThru *instance;

  Driver d_driver;
  bool d_deviceOpened;
  std::map<ulong, std::shared_ptr<Driver>> d_channels;
  // массив активных устройств
  // map устройство - логический канал
  // буферы для полученных сообщений
  // буферы отправляемых сообщений
  // фильтры сообщений

};



#endif // PASSTHRU_H
