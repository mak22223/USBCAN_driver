
#include "passthru.h"

#include <iostream>

PassThru* PassThru::instance = nullptr;

PassThru::PassThru()
{

}

bool PassThru::initPassThruInterface()
{
  if (instance == nullptr) {
    instance = new PassThru();
    return true;
  }
  return false;
}

bool PassThru::deinitPassThruInterface()
{
  if (instance != nullptr) {
    delete instance;
    instance = nullptr;
    return true;
  }
  return false;
}

PassThru& PassThru::getInstance()
{
  return *instance;
}

long PassThru::open(void*, unsigned long *pDeviceID)
{
  // if (!d_driver.open()) {
  //   return ERR_DEVICE_NOT_CONNECTED;
  // }

  return STATUS_NOERROR;
}

long close(ulong DeviceID)
{

}

long connect(
  ulong DeviceID,
  ulong ProtocolID,
  ulong Flags,
  ulong BaudRate,
  ulong *pChannelID)
{

}

long disconnect(ulong ChannelID)
{

}

long readMsgs(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pNumMsgs,
  ulong Timeout)
{

}

long writeMsgs(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pNumMsgs,
  ulong Timeout)
{

}

long startPeriodicMsg(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pMsgID,
  ulong TimeInterval)
{

}

long stopPeriodicMsg(
  ulong ChannelID,
  ulong MsgID)
{

}

long startMsgFilter(
  ulong ChannelID,
  ulong FilterType,
  PassThruMsg *pMaskMsg,
  PassThruMsg *pPatternMsg,
  PassThruMsg *pFlowControlMsg,
  ulong *pFilterID)
{

}

long stopMsgFilter(
  ulong ChannelID,
  ulong FilterID)
{

}

long setProgrammingVoltage(
  ulong DeviceID,
  ulong PinNumber,
  ulong Voltage)
{

}

long readVersion(
  ulong DeviceID,
  char *pFirmwareVersion,
  char *pDllVersion,
  char *pApiVersion)
{

}

long getLastError(char *pErrorDescription)
{

}

long ioctl(
  ulong ChannelID,
  ulong IoctlID,
  void *pInput,
  void *pOutput)
{

}
