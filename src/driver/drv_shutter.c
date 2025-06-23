// ================
// File: src/driver/drv_shutter.c
// ================

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <stdbool.h>

#include "../new_common.h"
#include "../new_pins.h"
#include "../hal/hal_pins.h"
#include "../logging/logging.h"
#include "../cmnds/cmd_public.h"
#include "../httpserver/new_http.h"

#include "../../httpserver/hass.h"
#include "../mqtt/new_mqtt.h"
#include "../new_cfg.h"

#ifndef LOG_FEATURE_SHUTTER
#define LOG_FEATURE_SHUTTER LOG_FEATURE_DRV
#endif

#define MAX_SHUTTERS 10
#define SHUTTER_FULL_TIME 30000
#define DEFAULT_INTERLOCK_DELAY 500
#define SHUTTER_POSITION_REPORT_THRESHOLD 2.0f

typedef struct {
    int channelOpen;
    int channelClose;
    float position;
    int direction;
    uint32_t startTime;
    uint32_t travelTime;
    uint32_t interlockDelay;
    uint32_t lastSwitchTime;
    int targetDirection;
    float targetPosition;
    bool active;
    float lastReportedPosition;

    int lastSentDirection;
    float lastSentPosition;
    char lastSentState[16];
    uint32_t lastPublishTime;
    bool needsRepublish;
} Shutter_t;

static Shutter_t g_shutters[MAX_SHUTTERS];

void Shutter_PublishState(int index);
void Shutter_DoDiscovery(const char* topic);

static void Shutter_SetRelayChannels(int index, int openValue, int closeValue);
static void Shutter_Internal_Stop(int index, bool updateLastSwitchTime);
static void Shutter_DoSetPosition(int index, float targetPos);
static void Shutter_SetInterlockDelay(int index, uint32_t delayMs);
static void Shutter_SetTravelTime(int index, uint32_t travelTimeMs);

void Shutter_OnEverySecond() {
    uint32_t now = rtos_get_time();

    for (int i = 0; i < MAX_SHUTTERS; i++) {
        Shutter_t* s = &g_shutters[i];
        if (!s->active) continue;

        if (s->direction == -2) {
            uint32_t elapsedSinceInterlockStart = now - s->startTime;
            if (elapsedSinceInterlockStart >= s->interlockDelay) {
                s->direction = s->targetDirection;
                s->startTime = now;

                if (s->direction == 1) {
                    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) interlock complete, now opening...", i, s->channelOpen, s->channelClose);
                    CHANNEL_Set(s->channelOpen, 1, 0);
                    CHANNEL_Set(s->channelClose, 0, 0);
                }
                else if (s->direction == -1) {
                    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) interlock complete, now closing...", i, s->channelOpen, s->channelClose);
                    CHANNEL_Set(s->channelOpen, 0, 0);
                    CHANNEL_Set(s->channelClose, 1, 0);
                }
                Shutter_PublishState(i);
            }
            else {
                addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) in interlock wait (target %.1f), %lu ms remaining", i, s->channelOpen, s->channelClose, s->targetPosition, s->interlockDelay - elapsedSinceInterlockStart);
            }
            continue;
        }

        if (s->direction != 0) {
            uint32_t elapsedSinceSegmentStart = now - s->startTime;
            float movedPercentInSegment = (elapsedSinceSegmentStart * 100.0f) / s->travelTime;

            s->position += s->direction * movedPercentInSegment;

            if (s->position > 100.0f) s->position = 100.0f;
            if (s->position < 0.0f) s->position = 0.0f;

            s->startTime = now;

            bool limitReached = false;
            bool targetReached = false;

            if (s->direction == 1 && s->position >= 100.0f) {
                s->position = 100.0f;
                limitReached = true;
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) fully open.", i, s->channelOpen, s->channelClose);
            }
            else if (s->direction == -1 && s->position <= 0.0f) {
                s->position = 0.0f;
                limitReached = true;
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) fully closed.", i, s->channelOpen, s->channelClose);
            }
            else if (s->direction == 1 && s->position >= s->targetPosition) {
                s->position = s->targetPosition;
                targetReached = true;
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) reached target position %.1f.", i, s->channelOpen, s->channelClose, s->targetPosition);
            }
            else if (s->direction == -1 && s->position <= s->targetPosition) {
                s->position = s->targetPosition;
                targetReached = true;
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) reached target position %.1f.", i, s->channelOpen, s->channelClose, s->targetPosition);
            }

            if (limitReached || targetReached) {
                addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Stopping Shutter %i: Setting Ch %i=0, Ch %i=0", i, s->channelOpen, s->channelClose);
                CHANNEL_Set(s->channelOpen, 0, 0);
                CHANNEL_Set(s->channelClose, 0, 0);
                s->direction = 0;
                Shutter_PublishState(i);
            }
            if (fabs(s->position - s->lastReportedPosition) >= SHUTTER_POSITION_REPORT_THRESHOLD ||
                (s->direction != 0 && s->lastReportedPosition == s->position)) {
                Shutter_PublishState(i);
                s->lastReportedPosition = s->position;
            }
        }
    }
}

void Shutter_PublishState(int index) {
    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
        addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter_PublishState: Invalid or unconfigured Shutter index %i", index);
        return;
    }
    Shutter_t* s = &g_shutters[index];

    const uint32_t MIN_PUBLISH_INTERVAL_MS = 500;
    const uint32_t FORCE_PUBLISH_INTERVAL_MS = 60000;

    uint32_t now = rtos_get_time();

    const char* current_ha_state_str;
    if (s->direction == 1) {
        current_ha_state_str = "opening";
    }
    else if (s->direction == -1) {
        current_ha_state_str = "closing";
    }
    else {
        if (s->position >= 99.0f) {
            current_ha_state_str = "open";
        }
        else if (s->position <= 1.0f) {
            current_ha_state_str = "closed";
        }
        else {
            current_ha_state_str = "stopped";
        }
    }

    bool has_direction_changed = (s->direction != s->lastSentDirection);
    bool has_position_changed = (s->position != s->lastSentPosition);
    bool has_state_string_changed = (strcmp(current_ha_state_str, s->lastSentState) != 0);
    bool force_publish = ((now - s->lastPublishTime) >= FORCE_PUBLISH_INTERVAL_MS);

    if (MQTT_IsReady() &&
        (has_direction_changed || has_position_changed || has_state_string_changed ||
            force_publish || s->needsRepublish))
    {
        if ((now - s->lastPublishTime) < MIN_PUBLISH_INTERVAL_MS && !force_publish && !s->needsRepublish) {
            addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter %i: Rate limiting publish. Will publish later.", index);
            return;
        }

        char topic[128];
        char payload[32];
        const char* clientId = CFG_GetMQTTClientId();
        OBK_Publish_Result publish_res;

        if (has_direction_changed || force_publish || s->needsRepublish) {
            snprintf(topic, sizeof(topic), "%s/shutter%d/direction/get", clientId, index);
            snprintf(payload, sizeof(payload), "%d", s->direction);
            publish_res = MQTT_Publish("", topic, payload, OBK_PUBLISH_FLAG_RAW_TOPIC_NAME);
            if (publish_res == OBK_PUBLISH_OK) {
                s->lastSentDirection = s->direction;
                addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter %i Direction Publish to %s: %s", index, topic, payload);
            }
            else {
                addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "Shutter %i Direction Publish FAILED: %d", index, publish_res);
            }
        }

        if (has_position_changed || force_publish || s->needsRepublish) {
            snprintf(topic, sizeof(topic), "%s/shutter%d/position/get", clientId, index);
            snprintf(payload, sizeof(payload), "%.0f", s->position);
            publish_res = MQTT_Publish("", topic, payload, OBK_PUBLISH_FLAG_RAW_TOPIC_NAME);
            if (publish_res == OBK_PUBLISH_OK) {
                s->lastSentPosition = s->position;
                addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter %i Position Publish to %s: %s", index, topic, payload);
            }
            else {
                addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "Shutter %i Position Publish FAILED: %d", index, publish_res);
            }
        }

        if (has_state_string_changed || force_publish || s->needsRepublish) {
            snprintf(topic, sizeof(topic), "%s/shutter%d/state/get", clientId, index);
            publish_res = MQTT_Publish("", topic, current_ha_state_str, OBK_PUBLISH_FLAG_RAW_TOPIC_NAME);
            if (publish_res == OBK_PUBLISH_OK) {
                strncpy(s->lastSentState, current_ha_state_str, sizeof(s->lastSentState) - 1);
                s->lastSentState[sizeof(s->lastSentState) - 1] = '\0';
                addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter %i State String Publish to %s: %s", index, topic, current_ha_state_str);
            }
            else {
                addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "Shutter %i State String Publish FAILED: %d", index, publish_res);
            }
        }

        if (publish_res == OBK_PUBLISH_OK || publish_res == OBK_PUBLISH_MEM_FAIL) {
            s->lastPublishTime = now;
            s->needsRepublish = false;
        }
    }
    else if (!MQTT_IsReady()) {
        s->needsRepublish = true;
        addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter %i state change, but MQTT not ready. Setting needsRepublish flag.", index);
    }

    s->lastReportedPosition = s->position;
}

static void Shutter_SetRelayChannels(int index, int openValue, int closeValue) {
    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) return;

    Shutter_t* s = &g_shutters[index];

    addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i): Commanding Ch %i=%i, Ch %i=%i", index, s->channelOpen, s->channelClose, s->channelOpen, openValue, s->channelClose, closeValue);
    CHANNEL_Set(s->channelOpen, openValue, 0);
    CHANNEL_Set(s->channelClose, closeValue, 0);

    if (openValue != 0 || closeValue != 0) {
        s->lastSwitchTime = rtos_get_time();
    }
}

static void Shutter_Internal_Stop(int index, bool updateLastSwitchTime) {
    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
        return;
    }
    Shutter_t* s = &g_shutters[index];

    uint32_t now = rtos_get_time();
    if (s->direction == 1 || s->direction == -1) {
        uint32_t elapsedSinceMovementStart = now - s->startTime;
        float movedPercent = (elapsedSinceMovementStart * 100.0f) / s->travelTime;
        s->position += s->direction * movedPercent;
        if (s->position > 100.0f) s->position = 100.0f;
        if (s->position < 0.0f) s->position = 0.0f;
    }

    Shutter_SetRelayChannels(index, 0, 0);
    s->direction = 0;

    if (updateLastSwitchTime) {
        s->lastSwitchTime = now;
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) stopped. Position approx %.1f%%", index, s->channelOpen, s->channelClose, s->position);

    Shutter_PublishState(index);
}

static void Shutter_DoSetPosition(int index, float targetPos) {
    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
        addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "SetPos: Invalid shutter index %i", index);
        return;
    }
    Shutter_t* s = &g_shutters[index];

    if (targetPos > 100.0f) targetPos = 100.0f;
    if (targetPos < 0.0f) targetPos = 0.0f;

    uint32_t now = rtos_get_time();

    int currentOpenChannelValue = CHANNEL_Get(s->channelOpen);
    int currentCloseChannelValue = CHANNEL_Get(s->channelClose);
    bool wasAnyRelayOnPhysically = (currentOpenChannelValue == 1 || currentCloseChannelValue == 1);

    int newDirection = 0;
    if (targetPos > s->position + 0.5f) {
        newDirection = 1;
    }
    else if (targetPos < s->position - 0.5f) {
        newDirection = -1;
    }

    if (newDirection == 0) {
        Shutter_Internal_Stop(index, true);
        s->position = targetPos;
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) target %.1f reached.", index, s->channelOpen, s->channelClose, targetPos);
        Shutter_PublishState(index);
        return;
    }

    Shutter_Internal_Stop(index, false);

    bool interlockRequired = (s->interlockDelay > 0 && wasAnyRelayOnPhysically);

    s->targetPosition = targetPos;

    if (interlockRequired) {
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i): Entering interlock wait of %ums before direction change.",
            index, s->channelOpen, s->channelClose, s->interlockDelay);

        s->direction = -2;
        s->startTime = now;
        s->targetDirection = newDirection;
        Shutter_PublishState(index);
        return;
    }
    else {
        s->direction = newDirection;
        s->startTime = now;

        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i): Interlock not required. Starting immediate movement.", index, s->channelOpen, s->channelClose);

        if (s->direction == 1) {
            Shutter_SetRelayChannels(index, 1, 0);
        }
        else if (s->direction == -1) {
            Shutter_SetRelayChannels(index, 0, 1);
        }
        Shutter_PublishState(index);
    }
}

static void Shutter_SetInterlockDelay(int index, uint32_t delayMs) {
    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
        addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "SetInterlock: Invalid shutter index %i", index);
        return;
    }
    g_shutters[index].interlockDelay = delayMs;
    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) interlock delay set to %ums", index, g_shutters[index].channelOpen, g_shutters[index].channelClose, (unsigned int)delayMs);
    Shutter_PublishState(index);
}

static void Shutter_SetTravelTime(int index, uint32_t travelTimeMs) {
    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
        addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "SetTravelTime: Invalid shutter index %i", index);
        return;
    }
    if (travelTimeMs < 1000) {
        addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "SetTravelTime: Invalid travel time %ums for Shutter %i (Ch %i/%i), must be >= 1000ms", (unsigned int)travelTimeMs, index, g_shutters[index].channelOpen, g_shutters[index].channelClose);
        return;
    }
    g_shutters[index].travelTime = travelTimeMs;
    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i (Ch %i/%i) full travel time set to %ums", index, g_shutters[index].channelOpen, g_shutters[index].channelClose, (unsigned int)travelTimeMs);
    Shutter_PublishState(index);
}

void Shutter_DoDiscovery(const char* topic) {
    HassDeviceInfo* dev_info = NULL;

    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter_DoDiscovery: Publishing HA discovery messages.");

    for (int i = 0; i < MAX_SHUTTERS; i++) {
        Shutter_t* s = &g_shutters[i];
        if (!s->active) {
            addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter_DoDiscovery: Shutter %i is not active, skipping discovery.", i);
            continue;
        }

        dev_info = hass_createCover(i, NULL);
        if (dev_info == NULL) {
            addLogAdv(LOG_ERROR, LOG_FEATURE_SHUTTER, "Shutter_DoDiscovery: Failed to create HassDeviceInfo for Shutter %i.", i);
            continue;
        }

        MQTT_QueuePublish(topic, dev_info->channel, hass_build_discovery_json(dev_info), OBK_PUBLISH_FLAG_RETAIN);
        addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Shutter_DoDiscovery: Queued discovery for Shutter %i. Channel: %s", i, dev_info->channel);

        hass_free_device_info(dev_info);
    }
}

static commandResult_t CMD_GetShutterIndexArg(const void* context, const char* cmd, const char* args, int* shutterIndex) {
    Tokenizer_TokenizeString(args, 0);

    if (Tokenizer_GetArgsCount() < 1) {
        int activeShutterCount = 0;
        int firstActiveShutterIndex = -1;
        for (int i = 0; i < MAX_SHUTTERS; ++i) {
            if (g_shutters[i].active) {
                activeShutterCount++;
                firstActiveShutterIndex = i;
            }
        }

        if (activeShutterCount == 1 && firstActiveShutterIndex != -1) {
            *shutterIndex = firstActiveShutterIndex;
            addLogAdv(LOG_DEBUG, LOG_FEATURE_CMD, "%s: No ShutterIndex provided, defaulting to single active Shutter %i.", cmd, *shutterIndex);
        }
        else {
            addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "%s: Requires ShutterIndex. Multiple or no shutters configured, cannot default.", cmd);
            return CMD_RES_NOT_ENOUGH_ARGUMENTS;
        }
    }
    else {
        *shutterIndex = Tokenizer_GetArgInteger(0);
    }

    if (*shutterIndex < 0 || *shutterIndex >= MAX_SHUTTERS || !g_shutters[*shutterIndex].active) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: Invalid or unconfigured ShutterIndex %d. Valid range 0-%d for configured shutters.", cmd, *shutterIndex, MAX_SHUTTERS - 1);
        return CMD_RES_BAD_ARGUMENT;
    }
    return CMD_RES_OK;
}

commandResult_t CMD_ShutterOpen_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int index;
    if (CMD_GetShutterIndexArg(context, cmd, args, &index) != CMD_RES_OK) {
        return CMD_RES_BAD_ARGUMENT;
    }

    Shutter_t* s = &g_shutters[index];
    if (s->direction == 1) {
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "ShutterOpen %i: Already opening, triggering stop.", index);
        Shutter_Internal_Stop(index, true);
    }
    else {
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "ShutterOpen %i: Opening shutter.", index);
        Shutter_DoSetPosition(index, 100.0f);
    }
    return CMD_RES_OK;
}

commandResult_t CMD_ShutterClose_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int index;
    if (CMD_GetShutterIndexArg(context, cmd, args, &index) != CMD_RES_OK) {
        return CMD_RES_BAD_ARGUMENT;
    }

    Shutter_t* s = &g_shutters[index];
    if (s->direction == -1) {
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "ShutterClose %i: Already closing, triggering stop.", index);
        Shutter_Internal_Stop(index, true);
    }
    else {
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "ShutterClose %i: Closing shutter.", index);
        Shutter_DoSetPosition(index, 0.0f);
    }
    return CMD_RES_OK;
}

commandResult_t CMD_ShutterStop_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int index;
    if (CMD_GetShutterIndexArg(context, cmd, args, &index) != CMD_RES_OK) {
        return CMD_RES_BAD_ARGUMENT;
    }

    Shutter_Internal_Stop(index, true);
    return CMD_RES_OK;
}

commandResult_t CMD_ShutterPosition_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int index;
    float pos;

    if (strlen(cmd) > strlen("ShutterPosition") && strncmp(cmd, "ShutterPosition", strlen("ShutterPosition")) == 0 && isdigit((unsigned char)cmd[strlen("ShutterPosition")])) {
        const char* num_start = cmd + strlen("ShutterPosition");
        index = atoi(num_start);
        pos = atof(args);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MQTT '%s' called: Shutter %i, Position %.1f.", cmd, index, pos);
    }
    else {
        Tokenizer_TokenizeString(args, 0);
        if (Tokenizer_GetArgsCount() == 1) {
            index = 0;
            pos = Tokenizer_GetArgFloat(0);
            addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "%s: Received single argument (%.1f), assuming position for Shutter 0.", cmd, pos);
        }
        else if (Tokenizer_GetArgsCount() >= 2) {
            index = Tokenizer_GetArgInteger(0);
            pos = Tokenizer_GetArgFloat(1);
            addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "%s: Received two arguments, Shutter %i, Position %.1f.", cmd, index, pos);
        }
        else {
            addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "%s: Requires Position (0-100) or ShutterIndex + Position.", cmd);
            return CMD_RES_NOT_ENOUGH_ARGUMENTS;
        }

        if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
            addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: Invalid or unconfigured ShutterIndex %d.", cmd, index);
            return CMD_RES_BAD_ARGUMENT;
        }
    }

    Shutter_DoSetPosition(index, pos);
    return CMD_RES_OK;
}

commandResult_t CMD_SetShutter_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int shutterIdx, openChan, closeChan;
    Tokenizer_TokenizeString(args, 0);
    if (Tokenizer_GetArgsCount() < 3) {
        addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "%s: Requires ShutterIndex, OpenChannel, and CloseChannel. Usage: setShutter <shutter_num> <open_chan> <close_chan>.", cmd);
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }
    shutterIdx = Tokenizer_GetArgInteger(0);
    openChan = Tokenizer_GetArgInteger(1);
    closeChan = Tokenizer_GetArgInteger(2);

    if (shutterIdx < 0 || shutterIdx >= MAX_SHUTTERS) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: ShutterIndex %d is out of valid range (0-%d).", cmd, shutterIdx, MAX_SHUTTERS - 1);
        return CMD_RES_BAD_ARGUMENT;
    }

    if (openChan <= 0 || closeChan <= 0 || openChan >= CHANNEL_MAX || closeChan >= CHANNEL_MAX) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: Channel numbers must be positive and less than %i.", cmd, CHANNEL_MAX);
        return CMD_RES_BAD_ARGUMENT;
    }
    if (openChan == closeChan) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: Open and Close channels must be different.", cmd);
        return CMD_RES_BAD_ARGUMENT;
    }

    for (int i = 0; i < MAX_SHUTTERS; ++i) {
        if (i == shutterIdx) continue;

        if (g_shutters[i].active && (g_shutters[i].channelOpen == openChan || g_shutters[i].channelClose == openChan ||
            g_shutters[i].channelOpen == closeChan || g_shutters[i].channelClose == closeChan)) {
            addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: One or both channels (%d/%d) are already used by shutter %i.", cmd, openChan, closeChan, i);
            return CMD_RES_BAD_ARGUMENT;
        }
    }

    Shutter_t* s = &g_shutters[shutterIdx];

    if (s->active) {
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Reconfiguring Shutter %i. Old channels: %i/%i", shutterIdx, s->channelOpen, s->channelClose);
        CHANNEL_Set(s->channelOpen, 0, 0);
        CHANNEL_Set(s->channelClose, 0, 0);
    }

    s->channelOpen = openChan;
    s->channelClose = closeChan;

    s->position = 0.0f;
    s->direction = 0;
    s->travelTime = SHUTTER_FULL_TIME;
    s->interlockDelay = DEFAULT_INTERLOCK_DELAY;
    s->lastSwitchTime = rtos_get_time();
    s->targetDirection = 0;
    s->targetPosition = 0.0f;
    s->active = true;
    s->lastReportedPosition = -1.0f;

    s->lastSentDirection = -99;
    s->lastSentPosition = -999.0f;
    s->lastSentState[0] = '\0';
    s->lastPublishTime = 0;
    s->needsRepublish = true;

    char cmdBuf[64];

    snprintf(cmdBuf, sizeof(cmdBuf), "SetChannelPrivate %i 1", s->channelOpen);
    CMD_ExecuteCommand(cmdBuf, 0);
    snprintf(cmdBuf, sizeof(cmdBuf), "SetChannelPrivate %i 1", s->channelClose);
    CMD_ExecuteCommand(cmdBuf, 0);

    snprintf(cmdBuf, sizeof(cmdBuf), "SetChannelVisible %i 0", s->channelOpen);
    CMD_ExecuteCommand(cmdBuf, 0);
    snprintf(cmdBuf, sizeof(cmdBuf), "SetChannelVisible %i 0", s->channelClose);
    CMD_ExecuteCommand(cmdBuf, 0);

    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i channels %i and %i marked as private (hidden from MQTT/GUI).", shutterIdx, s->channelOpen, s->channelClose);

    addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "Initializing Shutter %i: Setting Ch %i=0, Ch %i=0", shutterIdx, s->channelOpen, s->channelClose);
    CHANNEL_Set(s->channelOpen, 0, 0);
    CHANNEL_Set(s->channelClose, 0, 0);

    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter %i defined: Open Ch %i, Close Ch %i. Ensure these channels are mapped on pins with a suitable role (e.g., IOR_Relay).",
        shutterIdx, s->channelOpen, s->channelClose);

    Shutter_PublishState(shutterIdx);
    return CMD_RES_OK;
}

commandResult_t CMD_ShutterSetInterlockDelay_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int index;
    Tokenizer_TokenizeString(args, 0);

    if (Tokenizer_GetArgsCount() < 1) {
        addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "%s: Requires DelayMs or ShutterIndex + DelayMs.", cmd);
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }

    uint32_t delayMs;
    if (Tokenizer_GetArgsCount() == 1) {
        index = 0;
        delayMs = Tokenizer_GetArgInteger(0);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "%s: Received single argument (delay %u), assuming Shutter 0.", cmd, delayMs);
    }
    else {
        index = Tokenizer_GetArgInteger(0);
        delayMs = Tokenizer_GetArgInteger(1);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "%s: Received two arguments, Shutter %i, Delay %u.", cmd, index, delayMs);
    }

    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: Invalid or unconfigured ShutterIndex %d.", cmd, index);
        return CMD_RES_BAD_ARGUMENT;
    }

    Shutter_SetInterlockDelay(index, delayMs);
    return CMD_RES_OK;
}

commandResult_t CMD_ShutterSetTravelTime_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int index;
    Tokenizer_TokenizeString(args, 0);

    if (Tokenizer_GetArgsCount() < 1) {
        addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "%s: Requires TravelTimeMs or ShutterIndex + TravelTimeMs.", cmd);
        return CMD_RES_NOT_ENOUGH_ARGUMENTS;
    }

    uint32_t travelTimeMs;
    if (Tokenizer_GetArgsCount() == 1) {
        index = 0;
        travelTimeMs = Tokenizer_GetArgInteger(0);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "%s: Received single argument (travel time %u), assuming Shutter 0.", cmd, travelTimeMs);
    }
    else {
        index = Tokenizer_GetArgInteger(0);
        travelTimeMs = Tokenizer_GetArgInteger(1);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "%s: Received two arguments, Shutter %i, Travel Time %u.", cmd, index, travelTimeMs);
    }

    if (index < 0 || index >= MAX_SHUTTERS || !g_shutters[index].active) {
        addLogAdv(LOG_ERROR, LOG_FEATURE_CMD, "%s: Invalid or unconfigured ShutterIndex %d.", cmd, index);
        return CMD_RES_BAD_ARGUMENT;
    }

    Shutter_SetTravelTime(index, travelTimeMs);
    return CMD_RES_OK;
}

commandResult_t CMD_ShutterState_Handler(const void* context, const char* cmd, const char* args, int flags) {
    int shutterIndex;
    int actionEnum;

    if (strlen(cmd) > strlen("shutterstate") && strncmp(cmd, "shutterstate", strlen("shutterstate")) == 0 && isdigit((unsigned char)cmd[strlen("shutterstate")])) {
        shutterIndex = atoi(cmd + strlen("shutterstate"));
        actionEnum = atoi(args);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "MQTT '%s' called: Shutter %i, Action Enum %i.", cmd, shutterIndex, actionEnum);
    }
    else {
        Tokenizer_TokenizeString(args, 0);
        if (Tokenizer_GetArgsCount() < 2) {
            addLogAdv(LOG_WARN, LOG_FEATURE_CMD, "%s: Requires ShutterIndex and action enum (0=close, 1=open, 2=stop). Usage: shutterstate <index> <enum>.", cmd);
            return CMD_RES_NOT_ENOUGH_ARGUMENTS;
        }
        shutterIndex = Tokenizer_GetArgInteger(0);
        actionEnum = Tokenizer_GetArgInteger(1);
        addLogAdv(LOG_INFO, LOG_FEATURE_CMD, "%s called: Shutter %i, Action Enum %i.", cmd, shutterIndex, actionEnum);
    }

    if (shutterIndex < 0 || shutterIndex >= MAX_SHUTTERS || !g_shutters[shutterIndex].active) {
        addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "%s: Invalid or inactive shutter index %i.", cmd, shutterIndex);
        return CMD_RES_BAD_ARGUMENT;
    }

    switch (actionEnum) {
    case 0:
        Shutter_DoSetPosition(shutterIndex, 0.0f);
        break;
    case 1:
        Shutter_DoSetPosition(shutterIndex, 100.0f);
        break;
    case 2:
        Shutter_Internal_Stop(shutterIndex, true);
        break;
    default:
        addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "%s: Unknown action enum %i for shutter %i. Expected 0, 1, or 2.", cmd, actionEnum, shutterIndex);
        return CMD_RES_BAD_ARGUMENT;
    }

    return CMD_RES_OK;
}

void DRV_Shutter_Init() {
    for (int i = 0; i < MAX_SHUTTERS; ++i) {
        g_shutters[i].active = false;
        g_shutters[i].lastReportedPosition = -1.0f;

        g_shutters[i].lastSentDirection = -99;
        g_shutters[i].lastSentPosition = -999.0f;
        g_shutters[i].lastSentState[0] = '\0';
        g_shutters[i].lastPublishTime = 0;
        g_shutters[i].needsRepublish = false;
    }

    CMD_RegisterCommand("SetShutter", CMD_SetShutter_Handler, NULL);
    CMD_RegisterCommand("ShutterOpen", CMD_ShutterOpen_Handler, NULL);
    CMD_RegisterCommand("ShutterClose", CMD_ShutterClose_Handler, NULL);
    CMD_RegisterCommand("ShutterStop", CMD_ShutterStop_Handler, NULL);
    CMD_RegisterCommand("ShutterPosition", CMD_ShutterPosition_Handler, NULL);
    CMD_RegisterCommand("ShutterSetInterlockDelay", CMD_ShutterSetInterlockDelay_Handler, NULL);
    CMD_RegisterCommand("ShutterSetTravelTime", CMD_ShutterSetTravelTime_Handler, NULL);

    CMD_RegisterCommand("shutterstate", CMD_ShutterState_Handler, NULL);
    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Registered console/HTTP command: shutterstate");

    for (int i = 0; i < MAX_SHUTTERS; ++i) {
        char cmdName[32];

        snprintf(cmdName, sizeof(cmdName), "shutterstate%d", i);
        CMD_RegisterCommand(cmdName, CMD_ShutterState_Handler, NULL);
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Dynamically registered MQTT command: %s", cmdName);

        snprintf(cmdName, sizeof(cmdName), "ShutterPosition%d", i);
        CMD_RegisterCommand(cmdName, CMD_ShutterPosition_Handler, NULL);
        addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Dynamically registered MQTT command: %s", cmdName);
    }

    addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "Shutter driver initialized with %i max configurable slots. Use setShutter command (e.g. from autoexec.bat) after configuring pins and channels to define shutters.", MAX_SHUTTERS);
}

void Shutter_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState) {
    if (bPreState) {
        char param_buffer[32];
        int processed_shutter_idx = -1;

        if (http_getArg(request->url, "shutterIdx", param_buffer, sizeof(param_buffer))) {
            processed_shutter_idx = atoi(param_buffer);

            if (processed_shutter_idx < 0 || processed_shutter_idx >= MAX_SHUTTERS || !g_shutters[processed_shutter_idx].active) {
                addLogAdv(LOG_WARN, LOG_FEATURE_SHUTTER, "HTTP: Invalid or inactive shutter index %i submitted. Ignoring request.", processed_shutter_idx);
                processed_shutter_idx = -1;
            }
            else {
                addLogAdv(LOG_DEBUG, LOG_FEATURE_SHUTTER, "HTTP: Processing command for Shutter %i (bPreState=true)", processed_shutter_idx);
            }
        }

        if (processed_shutter_idx != -1) {
            char cmd_str_buffer[64];

            if (http_getArg(request->url, "shutterUpActual", param_buffer, sizeof(param_buffer))) {
                snprintf(cmd_str_buffer, sizeof(cmd_str_buffer), "ShutterOpen %i", processed_shutter_idx);
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "HTTP: Shutter %i - Running command: %s", processed_shutter_idx, cmd_str_buffer);
                CMD_ExecuteCommand(cmd_str_buffer, 0);
            }
            else if (http_getArg(request->url, "shutterDownActual", param_buffer, sizeof(param_buffer))) {
                snprintf(cmd_str_buffer, sizeof(cmd_str_buffer), "ShutterClose %i", processed_shutter_idx);
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "HTTP: Shutter %i - Running command: %s", processed_shutter_idx, cmd_str_buffer);
                CMD_ExecuteCommand(cmd_str_buffer, 0);
            }
            else if (http_getArg(request->url, "setPos", param_buffer, sizeof(param_buffer))) {
                float target_pos = atof(param_buffer);
                snprintf(cmd_str_buffer, sizeof(cmd_str_buffer), "ShutterPosition %i %.1f", processed_shutter_idx, target_pos);
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "HTTP: Shutter %i - Running command by slider: %s", processed_shutter_idx, cmd_str_buffer);
                CMD_ExecuteCommand(cmd_str_buffer, 0);
            }
            else if (http_getArg(request->url, "setInterlockDelay", param_buffer, sizeof(param_buffer))) {
                uint32_t delay_ms = atoi(param_buffer);
                snprintf(cmd_str_buffer, sizeof(cmd_str_buffer), "ShutterSetInterlockDelay %i %u", processed_shutter_idx, delay_ms);
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "HTTP: Shutter %i - Running command for delay: %s", processed_shutter_idx, cmd_str_buffer);
                CMD_ExecuteCommand(cmd_str_buffer, 0);
            }
            else if (http_getArg(request->url, "setTravelTime", param_buffer, sizeof(param_buffer))) {
                uint32_t travel_time_ms = atoi(param_buffer);
                snprintf(cmd_str_buffer, sizeof(cmd_str_buffer), "ShutterSetTravelTime %i %u", processed_shutter_idx, travel_time_ms);
                addLogAdv(LOG_INFO, LOG_FEATURE_SHUTTER, "HTTP: Shutter %i - Running command for travel time: %s", processed_shutter_idx, cmd_str_buffer);
                CMD_ExecuteCommand(cmd_str_buffer, 0);
            }
        }
        return;
    }

    hprintf255(request, "<h3>Shutter Controls</h3>");

    for (int i = 0; i < MAX_SHUTTERS; i++) {
        Shutter_t* s = &g_shutters[i];
        if (!s->active) {
            continue;
        }

        int open_relay_state = CHANNEL_Get(s->channelOpen);
        int close_relay_state = CHANNEL_Get(s->channelClose);
        const char* open_relay_text = (open_relay_state == 1) ? "ON" : "OFF";
        const char* close_relay_text = (close_relay_state == 1) ? "ON" : "OFF";

        hprintf255(request, "<h4>Shutter %i (Ch %i/%i) - Position: %.1f%%</h4>",
            i, s->channelOpen, s->channelClose, s->position);

        hprintf255(request, "<form action=\"index\" method=\"get\">");
        hprintf255(request, "<input type=\"hidden\" name=\"shutterIdx\" value=\"%i\">", i);
        hprintf255(request, "<div>");
        hprintf255(request, "<label for=\"pos_slider_%i\" style=\"display: inline-block; width: 40px;\">Closed</label>", i);
        hprintf255(request, "<input type=\"range\" id=\"pos_slider_%i\" name=\"setPos\" min=\"0\" max=\"100\" value=\"%.0f\" onchange=\"this.form.submit()\" style=\"width: calc(100%% - 100px); display: inline-block;\">", i, s->position);
        hprintf255(request, "<label for=\"pos_slider_%i\" style=\"display: inline-block; width: 40px; text-align: right;\">Open</label>", i);
        hprintf255(request, "</div>");
        hprintf255(request, "</form><br>");

        hprintf255(request, "<div style=\"text-align: center; font-size: 1.8em; margin-bottom: 5px;\">");
        hprintf255(request, "<span style=\"font-weight: bold;\">%s</span> ", open_relay_text);
        hprintf255(request, "<span style=\"font-weight: bold;\">%s</span>", close_relay_text);
        hprintf255(request, "</div><br/>");

        hprintf255(request, "<form action=\"index\" method=\"get\">");
        hprintf255(request, "<input type=\"hidden\" name=\"shutterIdx\" value=\"%i\">", i);
        hprintf255(request, "<div style=\"text-align: center;\">");
        hprintf255(request, "<button type=\"submit\" name=\"shutterUpActual\" value=\"1\" style=\"width: 80px; height: 40px; font-size: 1.5em; background-color: #007bff; color: white; border: none; cursor: pointer;\">&#9650;</button>");
        hprintf255(request, "<button type=\"submit\" name=\"shutterDownActual\" value=\"1\" style=\"width: 80px; height: 40px; font-size: 1.5em; margin-left: 10px; background-color: #007bff; color: white; border: none; cursor: pointer;\">&#9660;</button>");
        hprintf255(request, "</div>");
        hprintf255(request, "</form><br>");

        hprintf255(request, "<form action=\"index\" method=\"get\">");
        hprintf255(request, "<input type=\"hidden\" name=\"shutterIdx\" value=\"%i\">", i);
        hprintf255(request, "<div style=\"margin-top: 15px; text-align: center;\">");
        hprintf255(request, "<label for=\"interlock_delay_%i\">Interlock Delay (ms):</label>", i);
        hprintf255(request, "<input type=\"number\" id=\"interlock_delay_%i\" name=\"setInterlockDelay\" value=\"%u\" min=\"0\" step=\"100\" style=\"width: 80px; margin-left: 5px;\">", i, (unsigned int)s->interlockDelay);
        hprintf255(request, "<button type=\"submit\" name=\"submitDelay\" value=\"Set\" style=\"margin-left: 5px;\">Set</button>");
        hprintf255(request, "</div>");
        hprintf255(request, "</form>");

        hprintf255(request, "<form action=\"index\" method=\"get\">");
        hprintf255(request, "<input type=\"hidden\" name=\"shutterIdx\" value=\"%i\">", i);
        hprintf255(request, "<div style=\"margin-top: 10px; text-align: center;\">");
        hprintf255(request, "<label for=\"travel_time_%i\">Full Travel Time (ms):</label>", i);
        hprintf255(request, "<input type=\"number\" id=\"travel_time_%i\" name=\"setTravelTime\" value=\"%u\" min=\"1000\" step=\"1000\" style=\"width: 100px; margin-left: 5px;\">", i, (unsigned int)s->travelTime);
        hprintf255(request, "<button type=\"submit\" name=\"submitTravel\" value=\"Set\" style=\"margin-left: 5px;\">Set</button>");
        hprintf255(request, "</div>");
        hprintf255(request, "</form><br>");
    }
}