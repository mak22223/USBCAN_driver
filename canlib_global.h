
#ifndef CANLIB_GLOBAL_H
#define CANLIB_GLOBAL_H

#include <QtCore/qglobal.h>
#include <minwindef.h>

#if defined(CANLIB_LIBRARY)
#define CANLIB_EXPORT Q_DECL_EXPORT
#else
#define CANLIB_EXPORT Q_DECL_IMPORT
#endif

#include "passthrudef.h"

extern "C" CANLIB_EXPORT long WINAPI PassThruOpen(void *pName,
                                                  unsigned long *pDeviceID);

extern "C" CANLIB_EXPORT long WINAPI PassThruClose(unsigned long DeviceID);

extern "C" CANLIB_EXPORT long WINAPI PassThruConnect(unsigned long DeviceID,
                                                     unsigned long ProtocolID,
                                                     unsigned long Flags,
                                                     unsigned long BaudRate,
                                                     unsigned long *pChannelID);

extern "C" CANLIB_EXPORT long WINAPI
PassThruDisconnect(unsigned long ChannelID);

extern "C" CANLIB_EXPORT long WINAPI PassThruReadMsgs(unsigned long ChannelID,
                                                      PassThruMsg *pMsg,
                                                      unsigned long *pNumMsgs,
                                                      unsigned long Timeout);

extern "C" long CANLIB_EXPORT WINAPI PassThruWriteMsgs(unsigned long ChannelID,
                                                       PassThruMsg *pMsg,
                                                       unsigned long *pNumMsgs,
                                                       unsigned long Timeout);

extern "C" long CANLIB_EXPORT WINAPI
PassThruStartPeriodicMsg(unsigned long ChannelID, PassThruMsg *pMsg,
                         unsigned long *pMsgID, unsigned long TimeInterval);

extern "C" long CANLIB_EXPORT WINAPI
PassThruStopPeriodicMsg(unsigned long ChannelID, unsigned long MsgID);

extern "C" long CANLIB_EXPORT WINAPI
PassThruStartMsgFilter(unsigned long ChannelID, unsigned long FilterType,
                       PassThruMsg *pMaskMsg, PassThruMsg *pPatternMsg,
                       PassThruMsg *pFlowControlMsg, unsigned long *pFilterID);

extern "C" long CANLIB_EXPORT WINAPI
PassThruStopMsgFilter(unsigned long ChannelID, unsigned long FilterID);

extern "C" long CANLIB_EXPORT WINAPI PassThruSetProgrammingVoltage(
    unsigned long DeviceID, unsigned long PinNumber, unsigned long Voltage);

extern "C" long CANLIB_EXPORT WINAPI PassThruReadVersion(unsigned long DeviceID,
                                                         char *pFirmwareVersion,
                                                         char *pDllVersion,
                                                         char *pApiVersion);

extern "C" long CANLIB_EXPORT WINAPI
PassThruGetLastError(char *pErrorDescription);

extern "C" long CANLIB_EXPORT WINAPI PassThruIoctl(unsigned long ChannelID,
                                                   unsigned long IoctlID,
                                                   void *pInput, void *pOutput);

#endif // CANLIB_GLOBAL_H
