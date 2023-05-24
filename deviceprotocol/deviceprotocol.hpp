#pragma once

#include <inttypes.h>

constexpr char ControlChar = 0x5U;

struct DeviceCommand
{
  enum Id {
    INFO,
    RESET,
    ITF_INIT,
    ITF_CONFIG,
    ITF_DEINIT,
    MSG_SEND,
    FLTR_SET,
    FLTR_UNSET,
  };

  char id;

  union {
    struct {

    } info;

    struct {

    } reset;

    struct {
      uint8_t type;
      uint16_t flags;
      uint32_t speed;
    } itfInit;
      
    struct {
      uint8_t type;
      uint8_t paramType;
      uint32_t data;
    } itfConfig;

    struct {
      uint8_t type;
    } itfDeinit;

    struct {
      uint8_t type;
      uint16_t flags;
      uint16_t len;
      char data[8];
    } msgSend8;

    struct {

    } fltrSet;

    struct {

    } fltrUnset;
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
    MSG_RECV,
    FLTR_SET,
    FLTR_UNSET,
  };

  char id;

  union {
    struct {
      char hwVer[3];
      char fwVer[3];
    } info;

    struct {

    } reset;
  } arg;
};
