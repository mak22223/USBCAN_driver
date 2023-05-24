
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
  if (pDeviceID == nullptr) {
    return ERR_NULL_PARAMETER;
  }

  ulong status = d_driver.open();
  *pDeviceID = 0;
  
  return status;
}

long PassThru::close(ulong DeviceID)
{
  if (DeviceID != 0) {
    return ERR_INVALID_DEVICE_ID;
  }

  return d_driver.close();
}

long PassThru::connect(
  ulong DeviceID,
  ulong ProtocolID,
  ulong Flags,
  ulong BaudRate,
  ulong *pChannelID)
{

}

long PassThru::disconnect(ulong ChannelID)
{

}

long PassThru::readMsgs(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pNumMsgs,
  ulong Timeout)
{

}

long PassThru::writeMsgs(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pNumMsgs,
  ulong Timeout)
{

}

long PassThru::startPeriodicMsg(
  ulong ChannelID,
  PassThruMsg *pMsg,
  ulong *pMsgID,
  ulong TimeInterval)
{

}

long PassThru::stopPeriodicMsg(
  ulong ChannelID,
  ulong MsgID)
{

}

long PassThru::startMsgFilter(
  ulong ChannelID,
  ulong FilterType,
  PassThruMsg *pMaskMsg,
  PassThruMsg *pPatternMsg,
  PassThruMsg *pFlowControlMsg,
  ulong *pFilterID)
{

}

long PassThru::stopMsgFilter(
  ulong ChannelID,
  ulong FilterID)
{

}

long PassThru::setProgrammingVoltage(
  ulong DeviceID,
  ulong PinNumber,
  ulong Voltage)
{

}

long PassThru::readVersion(
  ulong DeviceID,
  char *pFirmwareVersion,
  char *pDllVersion,
  char *pApiVersion)
{

}

long PassThru::getLastError(char *pErrorDescription)
{

}

long PassThru::ioctl(
  ulong ChannelID,
  ulong IoctlID,
  void *pInput,
  void *pOutput)
{

}
