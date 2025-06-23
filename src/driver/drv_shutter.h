#ifndef _DRV_SHUTTER_H_
#define _DRV_SHUTTER_H_

#include <stdint.h>             // For uint32_t
#include <stdbool.h>            // For bool type
#include "../httpserver/new_http.h" // For http_request_t
#include "../cmnds/cmd_public.h" // For commandResult_t (used by CMD_*_Handler prototypes)

// Main driver functions registered in drv_main.c's driver_s table
void DRV_Shutter_Init();
void Shutter_OnEverySecond();
void Shutter_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState);

// Home Assistant Discovery function (called by drv_main.c through onHassDiscovery)
void Shutter_DoDiscovery(const char* topic);

// MQTT related functions exposed by the driver
void Shutter_PublishState(int index); // To publish current shutter state via MQTT

// Command Handlers - these functions are registered with CMD_RegisterCommand
commandResult_t CMD_SetShutter_Handler(const void* context, const char* cmd, const char* args, int flags);
commandResult_t CMD_ShutterPosition_Handler(const void* context, const char* cmd, const char* args, int flags);
commandResult_t CMD_ShutterSetInterlockDelay_Handler(const void* context, const char* cmd, const char* args, int flags);
commandResult_t CMD_ShutterSetTravelTime_Handler(const void* context, const char* cmd, const char* args, int flags);
commandResult_t CMD_ShutterState_Handler(const void* context, const char* cmd, const char* args, int flags);

#endif /* _DRV_SHUTTER_H_ */