#include "canlib_global.h"
#include "passthru.h"

#include <iostream>

BOOL WINAPI DllMain(HINSTANCE hinstDLL, // handle to DLL module
  DWORD fdwReason,    // reason for calling function
  LPVOID lpvReserved) // reserved
{


  // Perform actions based on the reason for calling.
  switch (fdwReason) {
  case DLL_PROCESS_ATTACH:
    std::cout << "DLL_PROCESS_ATTACH" << std::endl;
    // Initialize once for each new process.
    // Return FALSE to fail DLL load.

    if(PassThru::initPassThruInterface()) {
      std::cout << "Initialized PassThruInterface." << std::endl;
    }
    break;

  case DLL_THREAD_ATTACH:
    std::cout << "DLL_THREAD_ATTACH" << std::endl;
    // Do thread-specific initialization.
    break;

  case DLL_THREAD_DETACH:
    std::cout << "DLL_THREAD_DETACH" << std::endl;
    // Do thread-specific cleanup.
    break;

  case DLL_PROCESS_DETACH:
    std::cout << "DLL_PROCESS_DETACH" << std::endl;

    if (lpvReserved != nullptr) {
      break; // do not do cleanup if process termination scenario
    }

    // Perform any necessary cleanup.
    break;
  }
  return TRUE; // Successful DLL_PROCESS_ATTACH.
}

extern "C" CANLIB_EXPORT long WINAPI PassThruOpen(
  void *pName,
  unsigned long *pDeviceID)
{
  (void)pName;
  return PassThru::getInstance().open(nullptr, pDeviceID);
}

extern "C" CANLIB_EXPORT long WINAPI PassThruClose(unsigned long DeviceID)
{
  return PassThru::getInstance().close(DeviceID);
}

extern "C" CANLIB_EXPORT long WINAPI PassThruConnect(
  unsigned long DeviceID,
  unsigned long ProtocolID,
  unsigned long Flags,
  unsigned long BaudRate,
  unsigned long *pChannelID)
{
  
}

extern "C" CANLIB_EXPORT long WINAPI
PassThruDisconnect(unsigned long ChannelID)
{

}

extern "C" CANLIB_EXPORT long WINAPI PassThruReadMsgs(
  unsigned long ChannelID,
  PassThruMsg *pMsg,
  unsigned long *pNumMsgs,
  unsigned long Timeout)
{

}

extern "C" long CANLIB_EXPORT WINAPI PassThruWriteMsgs(
  unsigned long ChannelID,
  PassThruMsg *pMsg,
  unsigned long *pNumMsgs,
  unsigned long Timeout)
{

}

extern "C" long CANLIB_EXPORT WINAPI
PassThruStartPeriodicMsg(
  unsigned long ChannelID,
  PassThruMsg *pMsg,
  unsigned long *pMsgID,
  unsigned long TimeInterval)
{

}

extern "C" long CANLIB_EXPORT WINAPI
PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID)
{

}

extern "C" long CANLIB_EXPORT WINAPI
PassThruStartMsgFilter(
  unsigned long ChannelID,
  unsigned long FilterType,
  PassThruMsg *pMaskMsg,
  PassThruMsg *pPatternMsg,
  PassThruMsg *pFlowControlMsg,
  unsigned long *pFilterID)
{

}

extern "C" long CANLIB_EXPORT WINAPI
PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID)
{

}

extern "C" long CANLIB_EXPORT WINAPI PassThruSetProgrammingVoltage(
  unsigned long DeviceID,
  unsigned long PinNumber,
  unsigned long Voltage)
{

}

extern "C" long CANLIB_EXPORT WINAPI PassThruReadVersion(
  unsigned long DeviceID,
  char *pFirmwareVersion,
  char *pDllVersion,
  char *pApiVersion)
{

}

extern "C" long CANLIB_EXPORT WINAPI
PassThruGetLastError(char *pErrorDescription)
{

}

extern "C" long CANLIB_EXPORT WINAPI PassThruIoctl(
  unsigned long ChannelID,
  unsigned long IoctlID,
  void *pInput,
  void *pOutput)
{
  
}
