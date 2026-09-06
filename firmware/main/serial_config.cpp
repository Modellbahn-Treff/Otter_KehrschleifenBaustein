// Copyright (C) 2026 Modellbahn-Treff for Kids GmbH
// SPDX-License-Identifier: GPL-3.0-or-later

#include "serial_config.h"
#include "settings.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>

static const char *TAG = "serial_cfg";

#define LINE_BUF_SIZE 2048
#define UART_BUF_SIZE 2048

// ---------------------------------------------------------------------------
// UART helpers
// ---------------------------------------------------------------------------

static void uart_print(const char *s) {
    uart_write_bytes(UART_NUM_0, s, strlen(s));
}

static void uart_println(const char *s) {
    uart_print(s);
    uart_print("\r\n");
}

// ---------------------------------------------------------------------------
// Command: get — print all settings as JSON (password masked)
// ---------------------------------------------------------------------------

static void cmd_get(void) {
    cJSON *root = cJSON_CreateObject();

    cJSON_AddNumberToObject(root, "AbschNummer",           AbschNummer);
    cJSON_AddNumberToObject(root, "KehrschleifenBaustein", KehrschleifenBaustein);
    cJSON_AddStringToObject(root, "ssid",                  ssid);
    cJSON_AddStringToObject(root, "password",              "***");
    cJSON_AddStringToObject(root, "mqtt_server",           mqtt_server);
    cJSON_AddStringToObject(root, "client_name",           client_name);
    cJSON_AddNumberToObject(root, "networkByte1",          networkByte1);
    cJSON_AddNumberToObject(root, "networkByte2",          networkByte2);
    cJSON_AddNumberToObject(root, "gatewayByte3",          gatewayByte3);
    cJSON_AddNumberToObject(root, "gatewayByte4",          gatewayByte4);

    cJSON_AddStringToObject(root, "MqttSet",      MqttSet);
    cJSON_AddStringToObject(root, "MqttExtA",     MqttExtA);
    cJSON_AddStringToObject(root, "MqttExtB",     MqttExtB);
    cJSON_AddStringToObject(root, "MqttOccupied", MqttOccupied);
    cJSON_AddStringToObject(root, "MqttVoltage",  MqttVoltage);
    cJSON_AddStringToObject(root, "MqttStatus",   MqttStatus);

    char *s = cJSON_Print(root);
    uart_println(s);
    cJSON_free(s);
    cJSON_Delete(root);
}

// ---------------------------------------------------------------------------
// Command: set {...} — update non-WiFi settings from JSON, save to NVS
// ---------------------------------------------------------------------------

static void cmd_set(const char *json_str) {
    char tx[64];
    settings_result_t result = settings_apply_json(json_str, tx, sizeof(tx));
    if (result == SETTINGS_ERR_JSON) {
        uart_println("ERROR: JSON parse failed");
    } else if (result == SETTINGS_ERR_NVS) {
        uart_println("ERROR: EEPROM write failed");
    } else {
        uart_println("OK: Settings saved. Restart to apply.");
    }
}

// ---------------------------------------------------------------------------
// Command: wifi {...} — update WiFi credentials, save, restart
// ---------------------------------------------------------------------------

static void cmd_wifi(const char *json_str) {
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        uart_println("ERROR: JSON parse failed");
        return;
    }

    cJSON *item;
    bool changed = false;

    if ((item = cJSON_GetObjectItem(root, "ssid"))     && cJSON_IsString(item)) { snprintf(ssid,     SETTINGS_STR_LEN, "%s", item->valuestring); changed = true; }
    if ((item = cJSON_GetObjectItem(root, "password")) && cJSON_IsString(item)) { snprintf(password, SETTINGS_STR_LEN, "%s", item->valuestring); changed = true; }

    cJSON_Delete(root);

    if (!changed) {
        uart_println("ERROR: No 'ssid' or 'password' key found in JSON");
        return;
    }

    settings_save_wifi_to_nvs();
    uart_println("OK: WiFi credentials saved. Restarting now...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

// ---------------------------------------------------------------------------
// Command: reset — erase NVS settings namespace, restart
// ---------------------------------------------------------------------------

static void cmd_reset(void) {
    nvs_handle_t h;
    if (nvs_open("settings", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);
        nvs_commit(h);
        nvs_close(h);
    }
    uart_println("OK: Settings erased. Restarting with factory defaults...");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

// ---------------------------------------------------------------------------
// Command: help
// ---------------------------------------------------------------------------

static void cmd_help(void) {
    uart_println("");
    uart_println("Serial Configuration Console");
    uart_println("----------------------------");
    uart_println("  get                     - Print all current settings as JSON");
    uart_println("  set {JSON}              - Update settings (non-WiFi), save to NVS");
    uart_println("  wifi {\"ssid\":\"x\",\"password\":\"y\"} - Update WiFi credentials, restart");
    uart_println("  restart                 - Restart the device");
    uart_println("  reset                   - Erase saved settings, restart with defaults");
    uart_println("  help                    - Show this message");
    uart_println("");
    uart_println("set JSON keys (all optional):");
    uart_println("  AbschNummer, KehrschleifenBaustein, mqtt_server, client_name");
    uart_println("  networkByte1, networkByte2, gatewayByte3, gatewayByte4");
    uart_println("  MqttSet      (topic for writing settings via MQTT, default: otter/Set)");
    uart_println("  MqttExtA     (topic for external input A, default: otter/KB/ExtA)");
    uart_println("  MqttExtB     (topic for external input B, default: otter/KB/ExtB)");
    uart_println("  MqttOccupied (topic for the occupancy/Besetztmeldung, default: otter/KB/Occupied)");
    uart_println("  MqttVoltage  (topic for the current loop voltage, default: otter/KB/Voltage)");
    uart_println("  MqttStatus   (topic for the current Kehrschleifen status, default: otter/KB/Status)");
}

// ---------------------------------------------------------------------------
// Command dispatcher
// ---------------------------------------------------------------------------

static void dispatch(char *line) {
    // strip leading whitespace
    while (*line && isspace((unsigned char)*line)) line++;
    if (*line == '\0') return;

    if (strncasecmp(line, "get", 3) == 0 && (line[3] == '\0' || isspace((unsigned char)line[3]))) {
        cmd_get();
    } else if (strncasecmp(line, "set", 3) == 0 && isspace((unsigned char)line[3])) {
        cmd_set(line + 4);
    } else if (strncasecmp(line, "wifi", 4) == 0 && isspace((unsigned char)line[4])) {
        cmd_wifi(line + 5);
    } else if (strncasecmp(line, "restart", 7) == 0) {
        uart_println("Restarting...");
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_restart();
    } else if (strncasecmp(line, "reset", 5) == 0) {
        cmd_reset();
    } else if (strncasecmp(line, "help", 4) == 0) {
        cmd_help();
    } else {
        uart_println("ERROR: Unknown command. Type 'help' for usage.");
    }
}

// ---------------------------------------------------------------------------
// UART task
// ---------------------------------------------------------------------------

static void serial_config_task(void *pvParameters) {
    uart_config_t uart_config = {};
    uart_config.baud_rate  = 115200;
    uart_config.data_bits  = UART_DATA_8_BITS;
    uart_config.parity     = UART_PARITY_DISABLE;
    uart_config.stop_bits  = UART_STOP_BITS_1;
    uart_config.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, UART_BUF_SIZE, 0, 0, NULL, 0);

    uart_println("\r\nSerial config ready. Type 'help' for commands.");

    static char line[LINE_BUF_SIZE];
    int pos = 0;

    while (true) {
        uint8_t ch;
        int len = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        // echo character back
        uart_write_bytes(UART_NUM_0, (const char *)&ch, 1);

        if (ch == '\n' || ch == '\r') {
            uart_print("\r\n");
            line[pos] = '\0';
            if (pos > 0) {
                dispatch(line);
            }
            pos = 0;
        } else if (ch == 127 || ch == '\b') {
            // backspace
            if (pos > 0) {
                pos--;
                uart_print("\b \b");
            }
        } else if (pos < LINE_BUF_SIZE - 1) {
            line[pos++] = (char)ch;
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void serial_config_start(void) {
    xTaskCreate(serial_config_task, "serial_cfg", 8192, NULL, 5, NULL);
}
