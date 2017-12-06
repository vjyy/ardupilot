#pragma once

#include "ap_version.h"

#define THISFIRMWARE "APM:Copter V3.6.3-dev"
#define FIRMWARE_VERSION 3,6,3,FIRMWARE_VERSION_TYPE_DEV

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
