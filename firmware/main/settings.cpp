// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "settings.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "cJSON.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "settings";
static const char *NVS_NS = "settings";

// ---------------------------------------------------------------------------
// Default values (compiled-in fallbacks)
// ---------------------------------------------------------------------------

uint8_t AbschNummer            = 1;
uint8_t KehrschleifenBaustein  = 1;

char ssid[SETTINGS_STR_LEN]        = "SSID";
char password[SETTINGS_STR_LEN]    = "PASSWORD";
char mqtt_server[SETTINGS_STR_LEN] = "10.1.0.5";
char client_name[SETTINGS_STR_LEN] = "test_client";

uint8_t networkByte1 = 10;
uint8_t networkByte2 = 1;
uint8_t gatewayByte3 = 0;
uint8_t gatewayByte4 = 1;

char MqttSet[SETTINGS_TOPIC_LEN]      = "otter/Set";
char MqttExtA[SETTINGS_TOPIC_LEN]     = "otter/KB/ExtA";
char MqttExtB[SETTINGS_TOPIC_LEN]     = "otter/KB/ExtB";
char MqttOccupied[SETTINGS_TOPIC_LEN] = "otter/KB/Occupied";
char MqttVoltage[SETTINGS_TOPIC_LEN]  = "otter/KB/Voltage";
char MqttStatus[SETTINGS_TOPIC_LEN]   = "otter/KB/Status";

// ---------------------------------------------------------------------------
// NVS load — falls back to compiled defaults for any missing key
// ---------------------------------------------------------------------------

void settings_load_from_nvs(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "No saved settings, using defaults");
        return;
    }

    // uint8 scalars
    nvs_get_u8(h, "absch_nr", &AbschNummer);
    nvs_get_u8(h, "kb",       &KehrschleifenBaustein);
    nvs_get_u8(h, "net_b1",   &networkByte1);
    nvs_get_u8(h, "net_b2",   &networkByte2);
    nvs_get_u8(h, "gw_b3",    &gatewayByte3);
    nvs_get_u8(h, "gw_b4",    &gatewayByte4);

    // strings
    size_t len;
    len = SETTINGS_STR_LEN; nvs_get_str(h, "ssid",      ssid,        &len);
    len = SETTINGS_STR_LEN; nvs_get_str(h, "password",  password,    &len);
    len = SETTINGS_STR_LEN; nvs_get_str(h, "mqtt_srv",  mqtt_server, &len);
    len = SETTINGS_STR_LEN; nvs_get_str(h, "client_nm", client_name, &len);

    len = SETTINGS_TOPIC_LEN; nvs_get_str(h, "mqtt_set",  MqttSet,      &len);
    len = SETTINGS_TOPIC_LEN; nvs_get_str(h, "mqtt_exta", MqttExtA,     &len);
    len = SETTINGS_TOPIC_LEN; nvs_get_str(h, "mqtt_extb", MqttExtB,     &len);
    len = SETTINGS_TOPIC_LEN; nvs_get_str(h, "mqtt_occ",  MqttOccupied, &len);
    len = SETTINGS_TOPIC_LEN; nvs_get_str(h, "mqtt_volt", MqttVoltage,  &len);
    len = SETTINGS_TOPIC_LEN; nvs_get_str(h, "mqtt_stat", MqttStatus,   &len);

    nvs_close(h);
    ESP_LOGI(TAG, "Settings loaded from NVS");
}

// ---------------------------------------------------------------------------
// NVS save — all non-WiFi settings
// ---------------------------------------------------------------------------

bool settings_save_to_nvs(void) {
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NS, NVS_READWRITE, &h));

    nvs_set_u8(h, "absch_nr", AbschNummer);
    nvs_set_u8(h, "kb",       KehrschleifenBaustein);
    nvs_set_u8(h, "net_b1",   networkByte1);
    nvs_set_u8(h, "net_b2",   networkByte2);
    nvs_set_u8(h, "gw_b3",    gatewayByte3);
    nvs_set_u8(h, "gw_b4",    gatewayByte4);
    nvs_set_str(h, "mqtt_srv",  mqtt_server);
    nvs_set_str(h, "client_nm", client_name);

    nvs_set_str(h, "mqtt_set",  MqttSet);
    nvs_set_str(h, "mqtt_exta", MqttExtA);
    nvs_set_str(h, "mqtt_extb", MqttExtB);
    nvs_set_str(h, "mqtt_occ",  MqttOccupied);
    nvs_set_str(h, "mqtt_volt", MqttVoltage);
    nvs_set_str(h, "mqtt_stat", MqttStatus);

    esp_err_t err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Settings saved to NVS");
    return true;
}

// ---------------------------------------------------------------------------
// Apply JSON blob to settings and persist — used by serial 'set' and MQTT
// ---------------------------------------------------------------------------

settings_result_t settings_apply_json(const char *json_str, char *tx_out, size_t tx_out_len) {
    if (tx_out && tx_out_len > 0) tx_out[0] = '\0';

    cJSON *root = cJSON_Parse(json_str);
    if (!root) return SETTINGS_ERR_JSON;

    cJSON *item;

    if (tx_out && (item = cJSON_GetObjectItem(root, "tx")) && cJSON_IsString(item))
        snprintf(tx_out, tx_out_len, "%s", item->valuestring);

    if ((item = cJSON_GetObjectItem(root, "AbschNummer"))           && cJSON_IsNumber(item)) AbschNummer           = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(root, "KehrschleifenBaustein")) && cJSON_IsNumber(item)) KehrschleifenBaustein = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(root, "networkByte1"))          && cJSON_IsNumber(item)) networkByte1          = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(root, "networkByte2"))          && cJSON_IsNumber(item)) networkByte2          = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(root, "gatewayByte3"))          && cJSON_IsNumber(item)) gatewayByte3          = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(root, "gatewayByte4"))          && cJSON_IsNumber(item)) gatewayByte4          = (uint8_t)item->valueint;
    if ((item = cJSON_GetObjectItem(root, "mqtt_server"))           && cJSON_IsString(item)) snprintf(mqtt_server, SETTINGS_STR_LEN,  "%s", item->valuestring);
    if ((item = cJSON_GetObjectItem(root, "client_name"))           && cJSON_IsString(item)) snprintf(client_name, SETTINGS_STR_LEN,  "%s", item->valuestring);
    if ((item = cJSON_GetObjectItem(root, "MqttSet"))               && cJSON_IsString(item)) snprintf(MqttSet,      SETTINGS_TOPIC_LEN, "%s", item->valuestring);
    if ((item = cJSON_GetObjectItem(root, "MqttExtA"))              && cJSON_IsString(item)) snprintf(MqttExtA,     SETTINGS_TOPIC_LEN, "%s", item->valuestring);
    if ((item = cJSON_GetObjectItem(root, "MqttExtB"))              && cJSON_IsString(item)) snprintf(MqttExtB,     SETTINGS_TOPIC_LEN, "%s", item->valuestring);
    if ((item = cJSON_GetObjectItem(root, "MqttOccupied"))          && cJSON_IsString(item)) snprintf(MqttOccupied, SETTINGS_TOPIC_LEN, "%s", item->valuestring);
    if ((item = cJSON_GetObjectItem(root, "MqttVoltage"))           && cJSON_IsString(item)) snprintf(MqttVoltage,  SETTINGS_TOPIC_LEN, "%s", item->valuestring);
    if ((item = cJSON_GetObjectItem(root, "MqttStatus"))            && cJSON_IsString(item)) snprintf(MqttStatus,   SETTINGS_TOPIC_LEN, "%s", item->valuestring);

    bool do_reboot = (item = cJSON_GetObjectItem(root, "reboot")) && cJSON_IsTrue(item);

    cJSON_Delete(root);

    if (!settings_save_to_nvs()) return SETTINGS_ERR_NVS;

    return do_reboot ? SETTINGS_OK_REBOOT : SETTINGS_OK;
}

// ---------------------------------------------------------------------------
// NVS save — WiFi credentials only
// ---------------------------------------------------------------------------

void settings_save_wifi_to_nvs(void) {
    nvs_handle_t h;
    ESP_ERROR_CHECK(nvs_open(NVS_NS, NVS_READWRITE, &h));
    nvs_set_str(h, "ssid",     ssid);
    nvs_set_str(h, "password", password);
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "WiFi credentials saved to NVS");
}
