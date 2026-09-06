// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once
#include <stdint.h>
#include <stdbool.h>

#define SETTINGS_STR_LEN   64
#define SETTINGS_TOPIC_LEN 64

extern uint8_t AbschNummer;
extern uint8_t KehrschleifenBaustein;

extern char ssid[SETTINGS_STR_LEN];
extern char password[SETTINGS_STR_LEN];
extern char mqtt_server[SETTINGS_STR_LEN];
extern char client_name[SETTINGS_STR_LEN];

extern uint8_t networkByte1;
extern uint8_t networkByte2;
extern uint8_t gatewayByte3;
extern uint8_t gatewayByte4;

extern char MqttSet[SETTINGS_TOPIC_LEN];
extern char MqttExtA[SETTINGS_TOPIC_LEN];
extern char MqttExtB[SETTINGS_TOPIC_LEN];
extern char MqttOccupied[SETTINGS_TOPIC_LEN];
extern char MqttVoltage[SETTINGS_TOPIC_LEN];
extern char MqttStatus[SETTINGS_TOPIC_LEN];

typedef enum {
    SETTINGS_OK,
    SETTINGS_OK_REBOOT,
    SETTINGS_ERR_JSON,
    SETTINGS_ERR_NVS,
} settings_result_t;

void settings_load_from_nvs(void);
bool settings_save_to_nvs(void);
void settings_save_wifi_to_nvs(void);
settings_result_t settings_apply_json(const char *json_str, char *tx_out, size_t tx_out_len);
