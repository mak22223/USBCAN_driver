#pragma once

#include <inttypes.h>

constexpr char ControlChar = 0x5U;

enum class DeviceInterface : uint8_t {
  HS_CAN = 0,
  MS_CAN,
};

struct DeviceCommand
{
  enum Id {
    INFO,
    RESET,
    ITF_INIT,
    ITF_CONFIG,
    ITF_DEINIT,
    MSG_SEND8,
    FLTR_SET,
    FLTR_UNSET,
    READ_VBATT,
  };

  char id;

  union {
    struct {

    } info;

    struct {

    } reset;

    struct {
      DeviceInterface itf;
      uint16_t flags;
      uint32_t speed;
    } itfInit;
      
    struct {
      DeviceInterface itf;
      uint8_t paramType;
      uint32_t data;
    } itfConfig;

    struct {
      DeviceInterface itf;
    } itfDeinit;

    struct {
      DeviceInterface itf;
      uint16_t flags;
      uint16_t len;
      char data[8];
    } msgSend8;

    struct {
      DeviceInterface itf;
      uint32_t filter;
      uint32_t mask;
    } fltrSet;

    struct {
      DeviceInterface itf;
      uint8_t id;
    } fltrUnset;

    struct {

    } readVBatt;
  } arg;
};

struct DeviceAnswer
{
  enum Id {
    DEFAULT,
    INFO,
    RESET,
    ERROR,
    ITF_INIT,
    ITF_CONFIG,
    ITF_DEINIT,
    MSG_RECV8,
    MSG_SEND,
    FLTR_SET,
    FLTR_UNSET,
    READ_VBATT,
  };

  enum ErrorCode {
    RESET_OK = 1,
  };

  char id;

  union {
    struct {
      char hwVer[3];
      char fwVer[3];
    } info;

    struct {

    } reset;

    struct {
      ErrorCode code;
      union {
        struct {

        } reset;

        struct {
          DeviceInterface itf;
        } itfErr;

      } errorPayload;
    } error;

    struct {
      // replaced with 'error'
    } itfInit;

    struct {
      // replaced with 'error'
    } itfConfig;

    struct {
      DeviceInterface itf;
      uint16_t flags;
      uint8_t length;
      uint8_t data[8];
    } msgRecv8;
    
    struct {
      DeviceInterface itf;
      uint32_t id;
      uint32_t timestamp;
    } msgSend;

    struct {
      DeviceInterface itf;
      uint8_t id;
    } fltrSet;

    struct {
      // replaced with 'error'
    } fltrUnset;

    struct {
      uint16_t millivolts;
    } readVBatt;
  } arg;
};
